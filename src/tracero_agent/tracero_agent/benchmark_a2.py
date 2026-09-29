#!/usr/bin/env python3
import argparse
import json
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import Parameter
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty


DEFAULT_REPORT_DIR = '/root/turtlebot3_ws/events'
OBSTACLE_NAME = 'a2_range_obstacle'


def parse_args():
    parser = argparse.ArgumentParser(description='Repeatable Nav2 A2 costmap range benchmark')
    parser.add_argument('--runs', type=int, default=1)
    parser.add_argument('--start-x', type=float, default=-2.0)
    parser.add_argument('--start-y', type=float, default=-0.5)
    parser.add_argument('--start-yaw', type=float, default=0.0)
    parser.add_argument('--goal-x', type=float, default=-1.0)
    parser.add_argument('--goal-y', type=float, default=-0.5)
    parser.add_argument('--goal-yaw', type=float, default=0.0)
    parser.add_argument('--obstacle-distance', type=float, default=0.75)
    parser.add_argument('--obstacle-size', type=float, default=0.30)
    parser.add_argument('--scan-detect-range', type=float, default=1.10)
    parser.add_argument('--min-costmap-cells', type=int, default=1)
    parser.add_argument('--timeout-sec', type=float, default=45.0)
    parser.add_argument('--expected-range', type=float, default=2.5)
    parser.add_argument('--expect-fault', action='store_true')
    parser.add_argument('--report-dir', default=DEFAULT_REPORT_DIR)
    parser.add_argument('--robot-id', default='tc-01')
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def make_pose(x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp.sec = 0
    pose.header.stamp.nanosec = 0
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = yaw_to_quaternion(yaw)
    return pose


def call_service(node, client, request, timeout_sec=5.0):
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError(f'service unavailable: {client.srv_name}')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        raise TimeoutError(f'service timed out: {client.srv_name}')
    if future.exception() is not None:
        raise RuntimeError(str(future.exception()))
    return future.result()


def read_parameter(node, service_name, parameter_name):
    client = node.create_client(GetParameters, service_name)
    request = GetParameters.Request()
    request.names = [parameter_name]
    response = call_service(node, client, request)
    if not response.values or response.values[0].type != Parameter.Type.DOUBLE.value:
        raise RuntimeError(f'{service_name} returned a non-double for {parameter_name}')
    return float(response.values[0].double_value)


def obstacle_sdf(size):
    return f'''<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="a2_range_box">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>{size} {size} 0.5</size></box></geometry>
        <material><ambient>0.1 0.2 0.8 1</ambient><diffuse>0.1 0.2 0.8 1</diffuse></material>
      </visual>
      <collision name="collision">
        <geometry><box><size>{size} {size} 0.5</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>'''


def reset_and_spawn(node, options, delete_client, reset_client, spawn_client):
    try:
        call_service(node, delete_client, DeleteEntity.Request(name=OBSTACLE_NAME))
    except (RuntimeError, TimeoutError):
        pass
    call_service(node, reset_client, Empty.Request())
    time.sleep(1.0)
    request = SpawnEntity.Request()
    request.name = OBSTACLE_NAME
    request.xml = obstacle_sdf(options.obstacle_size)
    request.initial_pose.position.x = options.start_x + options.obstacle_distance * math.cos(options.start_yaw)
    request.initial_pose.position.y = options.start_y + options.obstacle_distance * math.sin(options.start_yaw)
    request.initial_pose.position.z = 0.25
    request.reference_frame = 'world'
    response = call_service(node, spawn_client, request)
    if not response.success:
        raise RuntimeError(f'failed to spawn A2 obstacle: {response.status_message}')
    time.sleep(1.5)


def wait_for_messages(node, state, seconds=2.0):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if state['scan'] is None:
        raise RuntimeError('no /scan message received')
    if state['local_costmap'] is None:
        raise RuntimeError('no /local_costmap/costmap message received')
    if state['global_costmap'] is None:
        raise RuntimeError('no /global_costmap/costmap message received')


def scan_forward_range(msg):
    if not msg.ranges:
        return None
    center = round((0.0 - msg.angle_min) / msg.angle_increment)
    values = []
    for index in range(max(0, center - 3), min(len(msg.ranges), center + 4)):
        value = float(msg.ranges[index])
        if math.isfinite(value):
            values.append(value)
    return min(values) if values else None


def costmap_stats(grid, x, y, radius=0.35):
    resolution = float(grid.info.resolution)
    width = int(grid.info.width)
    height = int(grid.info.height)
    origin_x = float(grid.info.origin.position.x)
    origin_y = float(grid.info.origin.position.y)
    near = []
    total_occupied = 0
    for index, value in enumerate(grid.data):
        if value >= 50:
            total_occupied += 1
            cell_x = origin_x + (index % width + 0.5) * resolution
            cell_y = origin_y + (index // width + 0.5) * resolution
            if math.hypot(cell_x - x, cell_y - y) <= radius:
                near.append(int(value))
    return {
        'frame_id': grid.header.frame_id,
        'total_occupied_cells': total_occupied,
        'occupied_cells_near_obstacle': len(near),
        'max_cost_near_obstacle': max(near) if near else 0,
    }


def run_once(options, navigator, iteration):
    state = {'scan': None, 'odom': None, 'local_costmap': None, 'global_costmap': None}
    subscriptions = [
        navigator.create_subscription(LaserScan, '/scan', lambda msg: state.__setitem__('scan', msg), 10),
        navigator.create_subscription(Odometry, '/odom', lambda msg: state.__setitem__('odom', msg), 10),
        navigator.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda msg: state.__setitem__('local_costmap', msg), 10),
        navigator.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda msg: state.__setitem__('global_costmap', msg), 10),
    ]
    delete_client = navigator.create_client(DeleteEntity, '/delete_entity')
    reset_client = navigator.create_client(Empty, '/reset_world')
    spawn_client = navigator.create_client(SpawnEntity, '/spawn_entity')
    result = {'iteration': iteration, 'started_at': datetime.now(timezone.utc).isoformat()}
    obstacle_x = options.start_x + options.obstacle_distance * math.cos(options.start_yaw)
    obstacle_y = options.start_y + options.obstacle_distance * math.sin(options.start_yaw)
    try:
        navigator.cancelTask()
        reset_and_spawn(navigator, options, delete_client, reset_client, spawn_client)
        navigator.setInitialPose(make_pose(options.start_x, options.start_y, options.start_yaw))
        time.sleep(2.0)
        navigator.clearAllCostmaps()
        wait_for_messages(navigator, state, 2.0)
        baseline_local = costmap_stats(state['local_costmap'], obstacle_x, obstacle_y)
        baseline_global = costmap_stats(state['global_costmap'], obstacle_x, obstacle_y)
        runtime_local = read_parameter(navigator, '/local_costmap/local_costmap/get_parameters', 'voxel_layer.scan.obstacle_max_range')
        runtime_global = read_parameter(navigator, '/global_costmap/global_costmap/get_parameters', 'voxel_layer.scan.obstacle_max_range')
        after_scan = scan_forward_range(state['scan'])
        after_local = costmap_stats(state['local_costmap'], obstacle_x, obstacle_y)
        after_global = costmap_stats(state['global_costmap'], obstacle_x, obstacle_y)
        result.update({
            'obstacle_position': {'x': obstacle_x, 'y': obstacle_y},
            'runtime_local_obstacle_max_range': runtime_local,
            'runtime_global_obstacle_max_range': runtime_global,
            'expected_obstacle_max_range': options.expected_range,
            'scan_forward_range': after_scan,
            'scan_detected_obstacle': after_scan is not None and after_scan <= options.scan_detect_range,
            'local_costmap_before': baseline_local,
            'local_costmap_after': after_local,
            'global_costmap_before': baseline_global,
            'global_costmap_after': after_global,
        })
        local_marked = after_local['occupied_cells_near_obstacle'] >= options.min_costmap_cells
        global_marked = after_global['occupied_cells_near_obstacle'] >= options.min_costmap_cells
        # The global costmap includes the static map, so it may contain
        # occupied cells near the test pose even when the live LaserScan was
        # discarded. Use the rolling local costmap as the dynamic evidence.
        result['local_costmap_marked_obstacle'] = local_marked
        result['global_costmap_marked_obstacle'] = global_marked
        result['costmap_marked_obstacle'] = local_marked
        result['costmap_signature'] = 'MARKED' if result['costmap_marked_obstacle'] else 'NOT_MARKED'
        navigator.goToPose(make_pose(options.goal_x, options.goal_y, options.goal_yaw))
        deadline = time.monotonic() + options.timeout_sec
        while not navigator.isTaskComplete():
            if time.monotonic() >= deadline:
                navigator.cancelTask()
                result['task_status'] = 'TIMEOUT'
                break
            rclpy.spin_once(navigator, timeout_sec=0.1)
        else:
            result['task_status'] = {
                TaskResult.SUCCEEDED: 'SUCCEEDED',
                TaskResult.CANCELED: 'CANCELED',
                TaskResult.FAILED: 'FAILED',
            }.get(navigator.getResult(), 'UNKNOWN')
        tolerance_ok = abs(runtime_local - options.expected_range) < 1e-6 and abs(runtime_global - options.expected_range) < 1e-6
        if options.expect_fault:
            result['passed'] = tolerance_ok and result['scan_detected_obstacle'] and not result['costmap_marked_obstacle']
            result['errors'] = [] if result['passed'] else ['A2 signature not observed: scan/costmap evidence does not match the range fault']
        else:
            result['passed'] = tolerance_ok and result['scan_detected_obstacle'] and result['costmap_marked_obstacle']
            result['errors'] = [] if result['passed'] else ['normal baseline did not show a marked obstacle']
    except Exception as error:
        result['passed'] = False
        result['errors'] = [f'{type(error).__name__}: {error}']
    finally:
        navigator.cancelTask()
        for subscription in subscriptions:
            navigator.destroy_subscription(subscription)
    return result


def main():
    options = parse_args()
    if options.runs < 1:
        raise SystemExit('--runs must be at least 1')
    Path(options.report_dir).mkdir(parents=True, exist_ok=True)
    rclpy.init(args=sys.argv)
    navigator = BasicNavigator()
    navigator.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    report = {'benchmark': 'A2', 'configuration': vars(options), 'runs': []}
    try:
        navigator.setInitialPose(make_pose(options.start_x, options.start_y, options.start_yaw))
        print('Waiting for Nav2 to become active...')
        navigator.waitUntilNav2Active()
        for iteration in range(1, options.runs + 1):
            run_result = run_once(options, navigator, iteration)
            report['runs'].append(run_result)
            print('  PASS' if run_result['passed'] else f"  FAIL: {run_result['errors']}")
        passed = sum(item['passed'] for item in report['runs'])
        report['passed_runs'] = passed
        report['total_runs'] = options.runs
        report['pass_rate'] = passed / options.runs
        report_path = Path(options.report_dir) / f'benchmark_a2_{int(time.time())}.json'
        report_path.write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(f'\nA2 benchmark: {passed}/{options.runs} passed ({report["pass_rate"] * 100:.1f}%)')
        print(f'Report: {report_path}')
        return 0 if passed == options.runs else 1
    finally:
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
