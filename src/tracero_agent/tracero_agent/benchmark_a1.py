#!/usr/bin/env python3
import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import Parameter
from rclpy.utilities import remove_ros_args
from std_srvs.srv import Empty


DEFAULT_REPORT_DIR = '/root/turtlebot3_ws/events'


def parse_args():
    parser = argparse.ArgumentParser(
        description='Benchmark Nav2 A1 goal-tolerance fault'
    )
    parser.add_argument('--runs', type=int, default=1)
    parser.add_argument('--start-x', type=float, default=-2.0)
    parser.add_argument('--start-y', type=float, default=-0.5)
    parser.add_argument('--start-yaw', type=float, default=0.0)
    parser.add_argument('--goal-x', type=float, default=2.5)
    parser.add_argument('--goal-y', type=float, default=-0.5)
    parser.add_argument('--goal-yaw', type=float, default=0.0)
    parser.add_argument('--timeout-sec', type=float, default=30.0)
    parser.add_argument('--expected-tolerance', type=float, default=0.25)
    parser.add_argument('--expect-fault', action='store_true')
    parser.add_argument('--report-dir', default=DEFAULT_REPORT_DIR)
    parser.add_argument('--robot-id', default='tc-01')
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def yaw_to_quaternion(yaw):
    import math

    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def make_pose(navigator, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    # A zero stamp asks Nav2/TF to use the latest transform.  This keeps the
    # benchmark robust when Gazebo publishes simulated time while the
    # benchmark process starts before its /clock parameter is applied.
    pose.header.stamp.sec = 0
    pose.header.stamp.nanosec = 0
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = yaw_to_quaternion(yaw)
    return pose


def read_runtime_tolerance(navigator):
    names = ['goal_checker.xy_goal_tolerance']
    client = navigator.create_client(
        GetParameters,
        '/controller_server/get_parameters',
    )
    if not client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError('controller_server parameter service unavailable')
    request = GetParameters.Request()
    request.names = names
    future = client.call_async(request)
    rclpy.spin_until_future_complete(navigator, future, timeout_sec=5.0)
    if not future.done() or future.exception() is not None:
        raise RuntimeError('failed to query controller_server parameters')
    values = future.result().values
    if not values or values[0].type != Parameter.Type.DOUBLE.value:
        raise RuntimeError('goal_checker.xy_goal_tolerance is not a double')
    return float(values[0].double_value)


def reset_simulation(navigator):
    """Reset the Gazebo world so every run starts from the same robot pose."""
    client = navigator.create_client(Empty, '/reset_world')
    if not client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError('/reset_world service unavailable')
    future = client.call_async(Empty.Request())
    rclpy.spin_until_future_complete(navigator, future, timeout_sec=5.0)
    if not future.done() or future.exception() is not None:
        raise RuntimeError('failed to reset Gazebo world')
    time.sleep(1.0)


def run_once(options, navigator, iteration):
    result = {
        'iteration': iteration,
        'started_at': datetime.now(timezone.utc).isoformat(),
    }
    navigator.cancelTask()
    reset_simulation(navigator)
    result['simulation_reset'] = True
    navigator.setInitialPose(
        make_pose(navigator, options.start_x, options.start_y, options.start_yaw)
    )
    time.sleep(2.0)
    navigator.clearAllCostmaps()
    runtime_tolerance = read_runtime_tolerance(navigator)
    result['runtime_xy_goal_tolerance'] = runtime_tolerance
    result['expected_xy_goal_tolerance'] = options.expected_tolerance
    tolerance_matches = abs(runtime_tolerance - options.expected_tolerance) < 1e-6
    if not tolerance_matches:
        result['passed'] = False
        result['errors'] = ['runtime xy_goal_tolerance does not match expectation']
        return result

    navigator.goToPose(
        make_pose(navigator, options.goal_x, options.goal_y, options.goal_yaw)
    )
    deadline = time.monotonic() + options.timeout_sec
    while not navigator.isTaskComplete():
        if time.monotonic() >= deadline:
            navigator.cancelTask()
            result['task_status'] = 'TIMEOUT'
            break
        rclpy.spin_once(navigator, timeout_sec=0.1)
    else:
        task_result = navigator.getResult()
        result['task_status'] = {
            TaskResult.SUCCEEDED: 'SUCCEEDED',
            TaskResult.CANCELED: 'CANCELED',
            TaskResult.FAILED: 'FAILED',
        }.get(task_result, str(task_result))

    if options.expect_fault:
        result['passed'] = result['task_status'] != 'SUCCEEDED'
        result['errors'] = [] if result['passed'] else [
            'A1 fault did not prevent navigation from succeeding before timeout'
        ]
    else:
        result['passed'] = result['task_status'] == 'SUCCEEDED'
        result['errors'] = [] if result['passed'] else [
            'baseline navigation did not succeed before timeout'
        ]
    return result


def main():
    options = parse_args()
    if options.runs < 1:
        raise SystemExit('--runs must be at least 1')
    Path(options.report_dir).mkdir(parents=True, exist_ok=True)
    rclpy.init(args=sys.argv)
    navigator = BasicNavigator()
    # Set the clock source before any pose or action timestamp is generated.
    # The benchmark is normally launched with use_sim_time, but setting it
    # explicitly also makes direct invocations deterministic.
    navigator.set_parameters([
        Parameter('use_sim_time', Parameter.Type.BOOL, True),
    ])
    report = {
        'benchmark': 'A1',
        'configuration': vars(options),
        'runs': [],
    }
    try:
        navigator.setInitialPose(
            make_pose(navigator, options.start_x, options.start_y, options.start_yaw)
        )
        print('Waiting for Nav2 to become active...')
        navigator.waitUntilNav2Active()
        for iteration in range(1, options.runs + 1):
            try:
                run_result = run_once(options, navigator, iteration)
            except Exception as error:
                run_result = {
                    'iteration': iteration,
                    'passed': False,
                    'errors': [f'{type(error).__name__}: {error}'],
                }
            report['runs'].append(run_result)
            print('  PASS' if run_result['passed'] else f"  FAIL: {run_result['errors']}")
        passed = sum(item['passed'] for item in report['runs'])
        report['passed_runs'] = passed
        report['total_runs'] = options.runs
        report['pass_rate'] = passed / options.runs
        report_path = Path(options.report_dir) / f'benchmark_a1_{int(time.time())}.json'
        report_path.write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(f'\nA1 benchmark: {passed}/{options.runs} passed ({report["pass_rate"] * 100:.1f}%)')
        print(f'Report: {report_path}')
        return 0 if passed == options.runs else 1
    finally:
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
