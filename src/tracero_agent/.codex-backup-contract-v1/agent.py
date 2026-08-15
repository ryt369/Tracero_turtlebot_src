#!/usr/bin/env python3
import collections
import json
import math
import os
import re
from typing import Dict, List

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class TraceroAgent(Node):
    def __init__(self):
        super().__init__('tracero_agent')

        self.declare_parameter('scan_threshold', 0.65)
        self.declare_parameter('output_dir', '/root/turtlebot3_ws/events')
        self.declare_parameter('robot_id', 'turtlebot3_burger_01')
        self.declare_parameter('run_id', 'manual')

        self.scan_threshold = self.get_parameter('scan_threshold').value
        self.output_dir = self.get_parameter('output_dir').value
        self.robot_id = self.get_parameter('robot_id').value
        self.run_id = str(self.get_parameter('run_id').value)

        os.makedirs(self.output_dir, exist_ok=True)

        self.fov_rad = math.radians(25.0)
        self.history_window_sec = 5.0
        self.post_trigger_sec = 2.0
        self.cooldown_sec = 5.0

        self.data_buffer = collections.deque()
        self.is_collecting_post = False
        self.trigger_time = 0.0
        self.post_start_time = 0.0
        self.last_trigger_time = -999.0
        self.trigger_front_min = 0.0
        self.pre_5s_snapshot = []

        self.latest_odom = None
        self.latest_cmd_vel = None
        self.latest_costmap_summary = None
        self.latest_nav_status = 'UNKNOWN'

        qos_scan = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        qos_costmap = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_scan)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.costmap_callback,
            qos_costmap,
        )
        self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.nav_status_callback,
            10,
        )

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(
            f'Tracero Agent initialized. Threshold: {self.scan_threshold}m; '
            f'run_id: {self.run_id}'
        )

    def get_current_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def scan_callback(self, msg: LaserScan):
        now_sec = self.get_current_sec()
        front_ranges = []
        for index, distance in enumerate(msg.ranges):
            angle = msg.angle_min + index * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if (
                abs(angle) <= self.fov_rad
                and msg.range_min <= distance <= msg.range_max
                and not math.isnan(distance)
                and not math.isinf(distance)
            ):
                front_ranges.append(distance)

        front_min_distance = min(front_ranges) if front_ranges else 999.0
        sample = {
            'timestamp': now_sec,
            'front_min_distance': round(front_min_distance, 3),
            'odom': self.latest_odom,
            'cmd_vel': self.latest_cmd_vel,
            'costmap_summary': self.latest_costmap_summary,
            'nav_status': self.latest_nav_status,
        }
        self.data_buffer.append(sample)

        if (
            front_min_distance < self.scan_threshold
            and not self.is_collecting_post
            and now_sec - self.last_trigger_time > self.cooldown_sec
        ):
            self.trigger_event(now_sec, front_min_distance)

    def odom_callback(self, msg: Odometry):
        self.latest_odom = {
            'x': round(msg.pose.pose.position.x, 3),
            'y': round(msg.pose.pose.position.y, 3),
            'linear_speed': round(msg.twist.twist.linear.x, 3),
        }

    def cmd_vel_callback(self, msg: Twist):
        self.latest_cmd_vel = {
            'linear_x': round(msg.linear.x, 3),
            'angular_z': round(msg.angular.z, 3),
        }

    def costmap_callback(self, msg: OccupancyGrid):
        now_sec = self.get_current_sec()
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.latest_costmap_summary = {
            'stamp': round(stamp_sec, 3),
            'age_ms': round(max(0.0, (now_sec - stamp_sec) * 1000.0), 1),
            'max_cost': int(max(msg.data) if msg.data else 0),
        }

    def nav_status_callback(self, msg: GoalStatusArray):
        if msg.status_list:
            status_code = msg.status_list[-1].status
            status_map = {
                1: 'ACCEPTED',
                2: 'EXECUTING',
                4: 'SUCCEEDED',
                5: 'CANCELED',
                6: 'ABORTED',
            }
            self.latest_nav_status = status_map.get(
                status_code,
                f'CODE_{status_code}',
            )

    def trigger_event(self, trigger_sec: float, min_dist: float):
        self.get_logger().warn(
            f'Event Triggered! Obstacle distance: {min_dist:.2f}m'
        )
        self.is_collecting_post = True
        self.trigger_time = trigger_sec
        self.post_start_time = trigger_sec
        self.last_trigger_time = trigger_sec
        self.trigger_front_min = min_dist
        cutoff = trigger_sec - self.history_window_sec
        self.pre_5s_snapshot = [
            sample
            for sample in self.data_buffer
            if sample['timestamp'] >= cutoff
        ]

    def control_loop(self):
        now_sec = self.get_current_sec()
        cutoff = now_sec - (self.history_window_sec + 1.0)
        while (
            self.data_buffer
            and self.data_buffer[0]['timestamp'] < cutoff
        ):
            self.data_buffer.popleft()

        if (
            self.is_collecting_post
            and now_sec - self.post_start_time >= self.post_trigger_sec
        ):
            post_2s_data = [
                sample
                for sample in self.data_buffer
                if self.trigger_time
                <= sample['timestamp']
                <= self.trigger_time + self.post_trigger_sec
            ]
            self.save_event_json(self.pre_5s_snapshot, post_2s_data)
            self.is_collecting_post = False

    def save_event_json(self, pre_5s: List[Dict], post_2s: List[Dict]):
        safe_run_id = re.sub(r'[^A-Za-z0-9_.-]', '_', self.run_id)
        filename = (
            f'event_tc01_{safe_run_id}_'
            f'{int(self.trigger_time * 1000)}.json'
        )
        filepath = os.path.join(self.output_dir, filename)
        event_payload = {
            'event_type': 'obstacle_near',
            'run_id': self.run_id,
            'trigger_time': round(self.trigger_time, 3),
            'robot_id': self.robot_id,
            'trigger_distance': round(self.trigger_front_min, 3),
            'window': {
                'pre_5s': pre_5s,
                'post_2s': post_2s,
            },
            'params_snapshot': {
                'controller_frequency': 20.0,
                'inflation_radius': 0.55,
                'update_frequency': 5.0,
            },
            'static_index_version': 'dev',
        }

        try:
            with open(filepath, 'w', encoding='utf-8') as event_file:
                json.dump(event_payload, event_file, indent=2, ensure_ascii=False)
            self.get_logger().info(
                f'Successfully saved event JSON to: {filepath}'
            )
        except Exception as error:
            self.get_logger().error(f'Failed to save JSON: {error}')


def main(args=None):
    rclpy.init(args=args)
    node = TraceroAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
