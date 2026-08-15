#!/usr/bin/env python3
import argparse
import json
import os
import signal
import subprocess
import sys
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from gazebo_msgs.srv import DeleteEntity
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.parameter import Parameter
from rclpy.utilities import remove_ros_args
from std_srvs.srv import Empty


DEFAULT_EVENTS_DIR = '/root/turtlebot3_ws/events'
OBSTACLE_NAME = 'tc01_obstacle'


def parse_args():
    parser = argparse.ArgumentParser(description='Repeatable TC-01 benchmark')
    parser.add_argument('--runs', type=int, default=5)
    parser.add_argument('--start-x', type=float, default=-2.0)
    parser.add_argument('--start-y', type=float, default=-0.5)
    parser.add_argument('--goal-x', type=float, default=2.5)
    parser.add_argument('--goal-y', type=float, default=0.0)
    parser.add_argument('--distance-ahead', type=float, default=0.75)
    parser.add_argument('--scan-threshold', type=float, default=0.65)
    parser.add_argument('--warmup-sec', type=float, default=5.5)
    parser.add_argument('--event-timeout-sec', type=float, default=15.0)
    parser.add_argument('--motion-timeout-sec', type=float, default=30.0)
    parser.add_argument('--events-dir', default=DEFAULT_EVENTS_DIR)
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def make_pose(navigator, x, y):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.w = 1.0
    return pose


def call_service(node, client, request, timeout_sec=5.0):
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError(f'Service unavailable: {client.srv_name}')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        raise TimeoutError(f'Service timed out: {client.srv_name}')
    if future.exception() is not None:
        raise RuntimeError(str(future.exception()))
    return future.result()


def delete_obstacle(node, delete_client):
    request = DeleteEntity.Request()
    request.name = OBSTACLE_NAME
    response = call_service(node, delete_client, request)
    return bool(response.success), response.status_message


def reset_world(node, reset_world_client):
    call_service(node, reset_world_client, Empty.Request())


def stop_process(process):
    if process is None:
        return
    try:
        process_group = os.getpgid(process.pid)
    except ProcessLookupError:
        return
    os.killpg(process_group, signal.SIGINT)
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3.0)


def wait_for_ros_duration(node, duration_sec, wall_timeout_sec=60.0):
    start_ros_sec = node.get_clock().now().nanoseconds / 1e9
    wall_deadline = time.monotonic() + wall_timeout_sec
    while rclpy.ok():
        now_ros_sec = node.get_clock().now().nanoseconds / 1e9
        if now_ros_sec - start_ros_sec >= duration_sec:
            return
        if time.monotonic() >= wall_deadline:
            raise TimeoutError(
                f'ROS time did not advance {duration_sec:.1f}s within '
                f'{wall_timeout_sec:.1f}s wall time'
            )
        rclpy.spin_once(node, timeout_sec=0.1)


def find_existing_agent_pids():
    result = subprocess.run(
        ['pgrep', '-f', '[t]racero_agent/agent'],
        capture_output=True,
        text=True,
        check=False,
    )
    return [line for line in result.stdout.splitlines() if line.strip()]


def wait_for_run_event(events_dir, run_id, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    pattern = f'event_tc01_{run_id}_*.json'
    while time.monotonic() < deadline:
        for candidate in Path(events_dir).glob(pattern):
            try:
                with open(candidate, encoding='utf-8') as event_file:
                    payload = json.load(event_file)
                if payload.get('run_id') == run_id:
                    return str(candidate)
            except (OSError, json.JSONDecodeError):
                pass
        time.sleep(0.25)
    return None


def validate_event(json_path, scan_threshold):
    errors = []
    warnings = []
    with open(json_path, encoding='utf-8') as event_file:
        payload = json.load(event_file)

    if payload.get('event_type') != 'obstacle_near':
        errors.append('event_type is not obstacle_near')

    trigger_time = payload.get('trigger_time')
    window = payload.get('window') or {}
    pre = window.get('pre_5s') or []
    post = window.get('post_2s') or []
    if not isinstance(trigger_time, (int, float)):
        errors.append('trigger_time is missing or invalid')
        trigger_time = 0.0
    if not pre:
        errors.append('window.pre_5s is empty')
    if not post:
        errors.append('window.post_2s is empty')

    samples = pre + post
    timestamps = [sample.get('timestamp') for sample in samples]
    if any(not isinstance(stamp, (int, float)) for stamp in timestamps):
        errors.append('one or more samples have invalid timestamps')
    elif any(left > right for left, right in zip(timestamps, timestamps[1:])):
        errors.append('sample timestamps are not non-decreasing')

    pre_span = 0.0
    post_span = 0.0
    if pre and isinstance(trigger_time, (int, float)):
        pre_span = trigger_time - pre[0]['timestamp']
        if pre_span < 4.5:
            errors.append(f'pre window only covers {pre_span:.2f}s')
    if post and isinstance(trigger_time, (int, float)):
        post_span = post[-1]['timestamp'] - trigger_time
        if post_span < 1.7:
            errors.append(f'post window only covers {post_span:.2f}s')

    required_fields = (
        'front_min_distance',
        'odom',
        'cmd_vel',
        'costmap_summary',
        'nav_status',
    )
    for field in required_fields:
        if not any(sample.get(field) is not None for sample in samples):
            errors.append(f'no sample contains {field}')

    moving_samples = [
        sample for sample in pre
        if abs((sample.get('odom') or {}).get('linear_speed', 0.0)) > 0.03
        or abs((sample.get('cmd_vel') or {}).get('linear_x', 0.0)) > 0.03
    ]
    if not moving_samples:
        errors.append('no robot motion observed before trigger')

    observed_min = min(
        (sample.get('front_min_distance', 999.0) for sample in samples),
        default=999.0,
    )
    if observed_min >= scan_threshold:
        errors.append(
            f'front minimum {observed_min:.3f}m did not cross '
            f'{scan_threshold:.3f}m threshold'
        )

    if not any(sample.get('nav_status') == 'EXECUTING' for sample in samples):
        warnings.append('navigation status never reported EXECUTING')

    return {
        'passed': not errors,
        'errors': errors,
        'warnings': warnings,
        'event_file': json_path,
        'pre_samples': len(pre),
        'post_samples': len(post),
        'pre_span_sec': round(pre_span, 3),
        'post_span_sec': round(post_span, 3),
        'observed_min_distance': round(observed_min, 3),
    }


def run_once(iteration, options, navigator, delete_client, reset_world_client):
    print(f'\n[Run {iteration}/{options.runs}] resetting scenario')
    try:
        navigator.cancelTask()
    except Exception:
        pass

    deleted, delete_message = delete_obstacle(navigator, delete_client)
    if deleted:
        print(f'  removed prior obstacle: {delete_message}')

    reset_world(navigator, reset_world_client)
    start_pose = make_pose(
        navigator,
        options.start_x,
        options.start_y,
    )
    navigator.setInitialPose(start_pose)
    time.sleep(2.0)
    navigator.clearAllCostmaps()
    time.sleep(1.0)

    existing_agent_pids = find_existing_agent_pids()
    if existing_agent_pids:
        raise RuntimeError(
            'Existing tracero_agent processes detected: '
            + ', '.join(existing_agent_pids)
        )
    run_id = (
        f'benchmark_{time.time_ns()}_run_{iteration:02d}_'
        f'{uuid.uuid4().hex[:8]}'
    )
    log_dir = Path(options.events_dir) / 'benchmark_logs'
    log_dir.mkdir(parents=True, exist_ok=True)
    agent_log_path = log_dir / f'run_{iteration:02d}_agent.log'
    spawn_log_path = log_dir / f'run_{iteration:02d}_spawn.log'

    agent_process = None
    spawn_process = None
    with open(agent_log_path, 'w', encoding='utf-8') as agent_log:
        try:
            agent_process = subprocess.Popen(
                [
                    'ros2', 'run', 'tracero_agent', 'agent',
                    '--ros-args',
                    '-p', 'use_sim_time:=true',
                    '-p', f'scan_threshold:={options.scan_threshold}',
                    '-p', f'output_dir:={options.events_dir}',
                    '-p', f'run_id:={run_id}',
                ],
                stdout=agent_log,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
            print(
                f'  warming Agent buffer for '
                f'{options.warmup_sec:.1f}s ROS time'
            )
            wait_for_ros_duration(
                navigator,
                options.warmup_sec,
                wall_timeout_sec=max(60.0, options.warmup_sec * 10.0),
            )
            if agent_process.poll() is not None:
                return {
                    'passed': False,
                    'errors': ['Agent exited during warmup'],
                    'warnings': [],
                    'agent_log': str(agent_log_path),
                }

            goal_pose = make_pose(
                navigator,
                options.goal_x,
                options.goal_y,
            )
            navigator.goToPose(goal_pose)
            print(
                f'  navigating to ({options.goal_x:.2f}, '
                f'{options.goal_y:.2f})'
            )

            with open(spawn_log_path, 'w', encoding='utf-8') as spawn_log:
                spawn_process = subprocess.Popen(
                    [
                        'ros2', 'run', 'tracero_agent', 'spawn_tc01',
                        '--ros-args',
                        '-p', 'use_sim_time:=true',
                        '-p', f'entity_name:={OBSTACLE_NAME}',
                        '-p', f'distance_ahead:={options.distance_ahead}',
                        '-p',
                        f'motion_timeout_sec:={options.motion_timeout_sec}',
                    ],
                    stdout=spawn_log,
                    stderr=subprocess.STDOUT,
                    text=True,
                    start_new_session=True,
                )
                try:
                    spawn_return_code = spawn_process.wait(
                        timeout=options.motion_timeout_sec + 10.0
                    )
                except subprocess.TimeoutExpired:
                    stop_process(spawn_process)
                    return {
                        'passed': False,
                        'errors': ['spawn_tc01 timed out waiting for motion'],
                        'warnings': [],
                        'spawn_log': str(spawn_log_path),
                    }

            if spawn_return_code != 0:
                return {
                    'passed': False,
                    'errors': [f'spawn_tc01 exited with {spawn_return_code}'],
                    'warnings': [],
                    'spawn_log': str(spawn_log_path),
                }

            event_path = wait_for_run_event(
                options.events_dir,
                run_id,
                options.event_timeout_sec,
            )
            if event_path is None:
                return {
                    'passed': False,
                    'errors': ['Agent did not create an event JSON in time'],
                    'warnings': [],
                    'agent_log': str(agent_log_path),
                    'spawn_log': str(spawn_log_path),
                }

            result = validate_event(event_path, options.scan_threshold)
            result['run_id'] = run_id
            result['agent_log'] = str(agent_log_path)
            result['spawn_log'] = str(spawn_log_path)
            return result
        finally:
            stop_process(spawn_process)
            stop_process(agent_process)
            try:
                navigator.cancelTask()
            except Exception:
                pass
            try:
                delete_obstacle(navigator, delete_client)
            except Exception:
                pass
            time.sleep(1.0)


def main():
    options = parse_args()
    Path(options.events_dir).mkdir(parents=True, exist_ok=True)
    rclpy.init(args=sys.argv)
    navigator = BasicNavigator()
    navigator.set_parameters([
        Parameter('use_sim_time', Parameter.Type.BOOL, True),
    ])
    delete_client = navigator.create_client(DeleteEntity, '/delete_entity')
    reset_world_client = navigator.create_client(Empty, '/reset_world')

    report = {
        'benchmark': 'TC-01',
        'started_at': datetime.now(timezone.utc).isoformat(),
        'configuration': vars(options),
        'runs': [],
    }

    try:
        initial_pose = make_pose(
            navigator,
            options.start_x,
            options.start_y,
        )
        navigator.setInitialPose(initial_pose)
        print('Waiting for Nav2 to become active...')
        navigator.waitUntilNav2Active()

        for iteration in range(1, options.runs + 1):
            try:
                result = run_once(
                    iteration,
                    options,
                    navigator,
                    delete_client,
                    reset_world_client,
                )
            except Exception as error:
                result = {
                    'passed': False,
                    'errors': [f'{type(error).__name__}: {error}'],
                    'warnings': [],
                }
            result['iteration'] = iteration
            report['runs'].append(result)
            status = 'PASS' if result['passed'] else 'FAIL'
            print(f'  {status}: {result.get("errors", [])}')

        passed = sum(run['passed'] for run in report['runs'])
        report['passed_runs'] = passed
        report['total_runs'] = options.runs
        report['pass_rate'] = passed / options.runs if options.runs else 0.0
        report['finished_at'] = datetime.now(timezone.utc).isoformat()
        report_path = Path(options.events_dir) / (
            f'benchmark_tc01_{int(time.time())}.json'
        )
        with open(report_path, 'w', encoding='utf-8') as report_file:
            json.dump(report, report_file, indent=2, ensure_ascii=False)

        print(
            f'\nTC-01 benchmark: {passed}/{options.runs} passed '
            f'({report["pass_rate"] * 100:.1f}%)'
        )
        print(f'Report: {report_path}')
        return 0 if passed == options.runs else 1
    finally:
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
