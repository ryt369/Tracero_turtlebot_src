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
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.parameter import Parameter
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener


DEFAULT_REPORT_DIR = '/root/turtlebot3_ws/events'
DEFAULT_URDF = (
    '/root/turtlebot3_ws/src/turtlebot3_simulations/'
    'turtlebot3_gazebo/urdf/turtlebot3_burger.urdf'
)
OBSTACLE_NAME = 'd2_tf_box'


def parse_args():
    parser = argparse.ArgumentParser(description='Repeatable D2 lidar TF extrinsic benchmark')
    parser.add_argument('--runs', type=int, default=1)
    parser.add_argument('--start-x', type=float, default=-2.0)
    parser.add_argument('--start-y', type=float, default=-0.5)
    parser.add_argument('--start-yaw', type=float, default=0.0)
    parser.add_argument('--obstacle-distance', type=float, default=0.75)
    parser.add_argument('--obstacle-size', type=float, default=0.30)
    parser.add_argument('--scan-detect-range', type=float, default=1.10)
    parser.add_argument('--min-costmap-cells', type=int, default=5)
    parser.add_argument('--expected-sensor-yaw', type=float, default=0.0)
    parser.add_argument('--timeout-sec', type=float, default=45.0)
    parser.add_argument('--model-urdf', default=DEFAULT_URDF)
    parser.add_argument('--expect-fault', action='store_true')
    parser.add_argument('--report-dir', default=DEFAULT_REPORT_DIR)
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_to_yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def wrapped_delta(final_yaw, initial_yaw):
    return math.atan2(math.sin(final_yaw - initial_yaw), math.cos(final_yaw - initial_yaw))


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


def obstacle_sdf(size):
    return f'''<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="d2_tf_box">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>{size} {size} 0.5</size></box></geometry>
        <material><ambient>0.8 0.2 0.1 1</ambient><diffuse>0.8 0.2 0.1 1</diffuse></material>
      </visual>
      <collision name="collision">
        <geometry><box><size>{size} {size} 0.5</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>'''


def reset_robot(node, delete_client, reset_client):
    try:
        call_service(node, delete_client, DeleteEntity.Request(name=OBSTACLE_NAME))
    except (RuntimeError, TimeoutError):
        pass
    call_service(node, reset_client, Empty.Request())
    time.sleep(1.0)


def spawn_obstacle(node, options, spawn_client):
    request = SpawnEntity.Request()
    request.name = OBSTACLE_NAME
    request.xml = obstacle_sdf(options.obstacle_size)
    request.initial_pose.position.x = options.start_x + options.obstacle_distance * math.cos(options.start_yaw)
    request.initial_pose.position.y = options.start_y + options.obstacle_distance * math.sin(options.start_yaw)
    request.initial_pose.position.z = 0.25
    request.reference_frame = 'world'
    response = call_service(node, spawn_client, request)
    if not response.success:
        raise RuntimeError(f'failed to spawn D2 obstacle: {response.status_message}')
    time.sleep(1.5)


def wait_for_messages(node, state, seconds=2.0):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    missing = []
    if state['scan'] is None:
        missing.append('/scan')
    if state['odom'] is None:
        missing.append('/odom')
    if state['local_costmap'] is None:
        missing.append('/local_costmap/costmap')
    if missing:
        raise RuntimeError(f'no messages received: {", ".join(missing)}')


def scan_forward_range(message):
    if not message.ranges:
        return None
    center = round((0.0 - message.angle_min) / message.angle_increment)
    values = []
    for index in range(max(0, center - 3), min(len(message.ranges), center + 4)):
        value = float(message.ranges[index])
        if math.isfinite(value):
            values.append(value)
    return min(values) if values else None


def costmap_stats(grid, x, y, radius=0.25):
    resolution = float(grid.info.resolution)
    width = int(grid.info.width)
    near = []
    for index, value in enumerate(grid.data):
        if value < 50:
            continue
        cell_x = float(grid.info.origin.position.x) + (index % width + 0.5) * resolution
        cell_y = float(grid.info.origin.position.y) + (index // width + 0.5) * resolution
        if math.hypot(cell_x - x, cell_y - y) <= radius:
            near.append(int(value))
    return {
        'frame_id': grid.header.frame_id,
        'occupied_cells_near_point': len(near),
        'max_cost_near_point': max(near) if near else 0,
    }


def lookup_sensor_yaw(tf_buffer):
    try:
        transform = tf_buffer.lookup_transform(
            'base_link', 'base_scan', rclpy.time.Time()
        )
    except TransformException as error:
        raise RuntimeError(f'cannot lookup base_link -> base_scan: {error}')
    return quaternion_to_yaw(transform.transform.rotation), transform


def run_once(options, navigator, tf_buffer, state, iteration):
    subscriptions = []
    delete_client = navigator.create_client(DeleteEntity, '/delete_entity')
    reset_client = navigator.create_client(Empty, '/reset_world')
    spawn_client = navigator.create_client(SpawnEntity, '/spawn_entity')
    result = {
        'iteration': iteration,
        'started_at': datetime.now(timezone.utc).isoformat(),
    }
    obstacle_x = options.start_x + options.obstacle_distance * math.cos(options.start_yaw)
    obstacle_y = options.start_y + options.obstacle_distance * math.sin(options.start_yaw)
    try:
        state.update({'scan': None, 'odom': None, 'local_costmap': None})
        subscriptions.extend([
            navigator.create_subscription(LaserScan, '/scan', lambda message: state.__setitem__('scan', message), 10),
            navigator.create_subscription(Odometry, '/odom', lambda message: state.__setitem__('odom', message), 10),
            navigator.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda message: state.__setitem__('local_costmap', message), 10),
        ])
        reset_robot(navigator, delete_client, reset_client)
        navigator.setInitialPose(make_pose(options.start_x, options.start_y, options.start_yaw))
        time.sleep(2.0)
        navigator.clearAllCostmaps()
        wait_for_messages(navigator, state, 3.0)
        sensor_yaw, transform = lookup_sensor_yaw(tf_buffer)
        odom_pose = state['odom'].pose.pose
        odom_yaw = quaternion_to_yaw(odom_pose.orientation)
        front_x = odom_pose.position.x + options.obstacle_distance * math.cos(odom_yaw)
        front_y = odom_pose.position.y + options.obstacle_distance * math.sin(odom_yaw)
        rear_x = odom_pose.position.x - options.obstacle_distance * math.cos(odom_yaw)
        rear_y = odom_pose.position.y - options.obstacle_distance * math.sin(odom_yaw)
        baseline_front = costmap_stats(state['local_costmap'], front_x, front_y)
        baseline_rear = costmap_stats(state['local_costmap'], rear_x, rear_y)
        spawn_obstacle(navigator, options, spawn_client)
        time.sleep(1.5)
        navigator.clearAllCostmaps()
        wait_for_messages(navigator, state, 2.0)
        front_stats = costmap_stats(state['local_costmap'], front_x, front_y)
        rear_stats = costmap_stats(state['local_costmap'], rear_x, rear_y)
        forward_range = scan_forward_range(state['scan'])
        front_delta = front_stats['occupied_cells_near_point'] - baseline_front['occupied_cells_near_point']
        rear_delta = rear_stats['occupied_cells_near_point'] - baseline_rear['occupied_cells_near_point']
        front_marked = front_delta >= options.min_costmap_cells
        rear_marked = rear_delta >= options.min_costmap_cells
        sensor_yaw_error = abs(wrapped_delta(sensor_yaw, options.expected_sensor_yaw))
        result.update({
            'obstacle_position_world': {'x': obstacle_x, 'y': obstacle_y},
            'sensor_tf': {
                'parent_frame': transform.header.frame_id,
                'child_frame': transform.child_frame_id,
                'yaw': sensor_yaw,
                'translation': {
                    'x': transform.transform.translation.x,
                    'y': transform.transform.translation.y,
                    'z': transform.transform.translation.z,
                },
            },
            'expected_sensor_yaw': options.expected_sensor_yaw,
            'sensor_yaw_error': sensor_yaw_error,
            'scan_frame': state['scan'].header.frame_id,
            'scan_forward_range': forward_range,
            'scan_detected_front_obstacle': forward_range is not None and forward_range <= options.scan_detect_range,
            'front_costmap': front_stats,
            'rear_costmap': rear_stats,
            'baseline_front_costmap': baseline_front,
            'baseline_rear_costmap': baseline_rear,
            'front_costmap_delta_cells': front_delta,
            'rear_costmap_delta_cells': rear_delta,
            'front_costmap_marked': front_marked,
            'rear_costmap_marked': rear_marked,
        })
        if options.expect_fault:
            result['passed'] = (
                sensor_yaw_error <= 0.2
                and result['scan_detected_front_obstacle']
                and not front_marked
                and rear_marked
            )
            result['errors'] = [] if result['passed'] else [
                'D2 signature not observed: scan, TF, and costmap direction do not match the fault'
            ]
        else:
            result['passed'] = (
                sensor_yaw_error <= 0.2
                and result['scan_detected_front_obstacle']
                and front_marked
                and not rear_marked
            )
            result['errors'] = [] if result['passed'] else [
                'normal baseline did not place the scan obstacle in front of the robot'
            ]
    except Exception as error:
        result['passed'] = False
        result['errors'] = [f'{type(error).__name__}: {error}']
    finally:
        for subscription in subscriptions:
            navigator.destroy_subscription(subscription)
        navigator.cancelTask()
    return result


def main():
    options = parse_args()
    if options.runs < 1:
        raise SystemExit('--runs must be at least 1')
    Path(options.report_dir).mkdir(parents=True, exist_ok=True)
    rclpy.init(args=sys.argv)
    navigator = BasicNavigator()
    navigator.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    tf_buffer = Buffer()
    TransformListener(tf_buffer, navigator)
    state = {'scan': None, 'odom': None, 'local_costmap': None}
    report = {
        'benchmark': 'D2',
        'configuration': vars(options),
        'runs': [],
    }
    try:
        navigator.setInitialPose(make_pose(options.start_x, options.start_y, options.start_yaw))
        print('Waiting for Nav2 to become active...')
        navigator.waitUntilNav2Active()
        for iteration in range(1, options.runs + 1):
            run_result = run_once(options, navigator, tf_buffer, state, iteration)
            report['runs'].append(run_result)
            print('  PASS' if run_result['passed'] else f"  FAIL: {run_result['errors']}")
        passed = sum(item['passed'] for item in report['runs'])
        report['passed_runs'] = passed
        report['total_runs'] = options.runs
        report['pass_rate'] = passed / options.runs
        report_path = Path(options.report_dir) / f'benchmark_d2_{int(time.time())}.json'
        report_path.write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(f'\nD2 benchmark: {passed}/{options.runs} passed ({report["pass_rate"] * 100:.1f}%)')
        print(f'Report: {report_path}')
        return 0 if passed == options.runs else 1
    finally:
        navigator.cancelTask()
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
