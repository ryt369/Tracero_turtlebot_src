#!/usr/bin/env python3
import argparse
import json
import math
import re
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.utilities import remove_ros_args
from std_srvs.srv import Empty


DEFAULT_REPORT_DIR = '/root/turtlebot3_ws/events'
DEFAULT_MODEL_SDF = (
    '/root/turtlebot3_ws/src/turtlebot3_simulations/'
    'turtlebot3_gazebo/models/turtlebot3_burger/model.sdf'
)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Repeatable C1 wheel-separation kinematics benchmark'
    )
    parser.add_argument('--runs', type=int, default=1)
    parser.add_argument('--cmd-topic', default='/cmd_vel')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--truth-topic', default='/c1_truth/ground_truth')
    parser.add_argument('--angular-z', type=float, default=0.35)
    parser.add_argument('--duration-sec', type=float, default=3.0)
    parser.add_argument('--publish-rate', type=float, default=20.0)
    parser.add_argument('--settle-sec', type=float, default=0.5)
    parser.add_argument('--physical-separation', type=float, default=0.16)
    parser.add_argument('--expected-separation', type=float, default=0.16)
    parser.add_argument('--model-sdf', default=DEFAULT_MODEL_SDF)
    parser.add_argument('--max-normal-yaw-error', type=float, default=0.20)
    parser.add_argument('--min-fault-ratio', type=float, default=1.40)
    parser.add_argument('--expect-fault', action='store_true')
    parser.add_argument('--report-dir', default=DEFAULT_REPORT_DIR)
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def quaternion_to_yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def wrapped_delta(final_yaw, initial_yaw):
    return math.atan2(
        math.sin(final_yaw - initial_yaw),
        math.cos(final_yaw - initial_yaw),
    )


def call_service(node, client, timeout_sec=5.0):
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError(f'service unavailable: {client.srv_name}')
    future = client.call_async(Empty.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        raise TimeoutError(f'service timed out: {client.srv_name}')
    if future.exception() is not None:
        raise RuntimeError(str(future.exception()))
    return future.result()


def read_wheel_separation(path):
    content = Path(path).read_text(encoding='utf-8')
    match = re.search(r'<wheel_separation>\s*([-+0-9.eE]+)\s*</wheel_separation>', content)
    if match is None:
        raise RuntimeError(f'wheel_separation not found in {path}')
    return float(match.group(1))


def wait_for_state(node, state, timeout_sec):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if state['odom'] is not None and state['truth'] is not None:
            return
    missing = []
    if state['odom'] is None:
        missing.append('odom')
    if state['truth'] is None:
        missing.append('Gazebo ground truth')
    raise TimeoutError(f'waiting for {", ".join(missing)}')


def truth_pose(state):
    return state['truth'].pose.pose


def zero_twist(publisher):
    publisher.publish(Twist())


def run_once(options, node, publisher, reset_client, state, iteration, configured):
    result = {
        'iteration': iteration,
        'started_at': datetime.now(timezone.utc).isoformat(),
        'configured_wheel_separation': configured,
        'expected_wheel_separation': options.expected_separation,
        'physical_wheel_separation': options.physical_separation,
    }
    try:
        zero_twist(publisher)
        call_service(node, reset_client)
        time.sleep(options.settle_sec)
        state['odom'] = None
        state['truth'] = None
        wait_for_state(node, state, 8.0)
        initial_odom = state['odom'].pose.pose
        initial_truth = truth_pose(state)

        command = Twist()
        command.angular.z = options.angular_z
        deadline = time.monotonic() + options.duration_sec
        period = 1.0 / options.publish_rate
        while time.monotonic() < deadline:
            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=min(period, 0.1))
        zero_twist(publisher)
        settle_deadline = time.monotonic() + options.settle_sec
        while time.monotonic() < settle_deadline:
            zero_twist(publisher)
            rclpy.spin_once(node, timeout_sec=0.05)

        final_odom = state['odom'].pose.pose
        final_truth = truth_pose(state)
        odom_delta = wrapped_delta(
            quaternion_to_yaw(final_odom.orientation),
            quaternion_to_yaw(initial_odom.orientation),
        )
        truth_delta = wrapped_delta(
            quaternion_to_yaw(final_truth.orientation),
            quaternion_to_yaw(initial_truth.orientation),
        )
        yaw_error = abs(wrapped_delta(truth_delta, odom_delta))
        measured_ratio = (
            abs(truth_delta) / abs(odom_delta)
            if abs(odom_delta) > 1e-3
            else None
        )
        commanded_yaw = abs(options.angular_z * options.duration_sec)
        measured_command_to_truth_ratio = (
            abs(truth_delta) / commanded_yaw
            if commanded_yaw > 1e-3
            else None
        )
        expected_ratio = configured / options.physical_separation
        result.update({
            'command_angular_z': options.angular_z,
            'command_duration_sec': options.duration_sec,
            'odom_yaw_delta': odom_delta,
            'gazebo_yaw_delta': truth_delta,
            'yaw_error': yaw_error,
            'measured_truth_to_odom_ratio': measured_ratio,
            'measured_command_to_truth_ratio': measured_command_to_truth_ratio,
            'expected_command_to_truth_ratio': expected_ratio,
            'odom_final_position': {
                'x': final_odom.position.x,
                'y': final_odom.position.y,
            },
            'gazebo_final_position': {
                'x': final_truth.position.x,
                'y': final_truth.position.y,
            },
        })
        separation_matches = abs(configured - options.expected_separation) < 1e-6
        if measured_ratio is None or measured_command_to_truth_ratio is None:
            result['passed'] = False
            result['errors'] = ['yaw did not change enough to measure the ratio']
        elif options.expect_fault:
            result['passed'] = (
                separation_matches
                and yaw_error <= options.max_normal_yaw_error
                and measured_command_to_truth_ratio >= options.min_fault_ratio
            )
            result['errors'] = [] if result['passed'] else [
                'C1 signature not observed: actual yaw did not exceed the command yaw'
            ]
        else:
            result['passed'] = (
                separation_matches
                and yaw_error <= options.max_normal_yaw_error
                and abs(measured_command_to_truth_ratio - 1.0) <= 0.20
            )
            result['errors'] = [] if result['passed'] else [
                'normal model did not follow the commanded yaw'
            ]
    except Exception as error:
        zero_twist(publisher)
        result['passed'] = False
        result['errors'] = [f'{type(error).__name__}: {error}']
    finally:
        zero_twist(publisher)
    return result


def main():
    options = parse_args()
    if options.runs < 1:
        raise SystemExit('--runs must be at least 1')
    if options.publish_rate <= 0 or options.duration_sec <= 0:
        raise SystemExit('--publish-rate and --duration-sec must be positive')
    configured = read_wheel_separation(options.model_sdf)
    Path(options.report_dir).mkdir(parents=True, exist_ok=True)
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('benchmark_c1')
    publisher = node.create_publisher(Twist, options.cmd_topic, 10)
    state = {'odom': None, 'truth': None}
    node.create_subscription(
        Odometry, options.odom_topic, lambda message: state.__setitem__('odom', message), 10
    )
    node.create_subscription(
        Odometry, options.truth_topic, lambda message: state.__setitem__('truth', message), 10
    )
    reset_client = node.create_client(Empty, '/reset_world')
    report = {
        'benchmark': 'C1',
        'configuration': vars(options),
        'configured_wheel_separation': configured,
        'runs': [],
    }
    try:
        print('Waiting for Gazebo odometry and ground truth...')
        wait_for_state(node, state, 15.0)
        for iteration in range(1, options.runs + 1):
            run_result = run_once(
                options, node, publisher, reset_client, state, iteration, configured
            )
            report['runs'].append(run_result)
            print('  PASS' if run_result['passed'] else f"  FAIL: {run_result['errors']}")
        passed = sum(item['passed'] for item in report['runs'])
        report['passed_runs'] = passed
        report['total_runs'] = options.runs
        report['pass_rate'] = passed / options.runs
        report_path = Path(options.report_dir) / f'benchmark_c1_{int(time.time())}.json'
        report_path.write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(f'\nC1 benchmark: {passed}/{options.runs} passed ({report["pass_rate"] * 100:.1f}%)')
        print(f'Report: {report_path}')
        return 0 if passed == options.runs else 1
    finally:
        zero_twist(publisher)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
