#!/usr/bin/env python3
import collections
import json
import math
import os
import re
import tempfile
import time
import urllib.error
import urllib.request
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
        self.declare_parameter('robot_id', 'tc-01')
        self.declare_parameter('run_id', 'manual')
        self.declare_parameter('event_type', 'obstacle_near')
        self.declare_parameter('static_index_version', 'v2')
        self.declare_parameter('backend_base_url', '')
        self.declare_parameter('http_timeout_sec', 3.0)
        self.declare_parameter('controller_frequency', 20.0)
        self.declare_parameter('update_frequency', 5.0)
        self.declare_parameter('inflation_radius', 0.55)

        self.scan_threshold = float(self.get_parameter('scan_threshold').value)
        self.output_dir = str(self.get_parameter('output_dir').value)
        self.robot_id = str(self.get_parameter('robot_id').value)
        self.run_id = str(self.get_parameter('run_id').value)
        self.event_type = str(self.get_parameter('event_type').value)
        self.static_index_version = str(
            self.get_parameter('static_index_version').value
        )
        self.backend_base_url = str(
            self.get_parameter('backend_base_url').value
        ).rstrip('/')
        self.http_timeout_sec = float(
            self.get_parameter('http_timeout_sec').value
        )
        self.params_snapshot = {
            'controller_frequency': float(
                self.get_parameter('controller_frequency').value
            ),
            'update_frequency': float(
                self.get_parameter('update_frequency').value
            ),
            'inflation_radius': float(
                self.get_parameter('inflation_radius').value
            ),
            'scan_threshold': self.scan_threshold,
        }

        os.makedirs(self.output_dir, exist_ok=True)
        self.fov_rad = math.radians(25.0)
        self.history_window_sec = 5.0
        self.post_trigger_sec = 2.0
        self.cooldown_sec = 5.0

        self.data_buffer = collections.deque()
        self.is_collecting_post = False
        self.trigger_window_time = 0.0
        self.trigger_unix_time = 0.0
        self.last_trigger_time = -999.0
        self.trigger_front_min = 0.0
        self.pre_5s_snapshot = []

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
        if self.backend_base_url:
            self.post_json(
                '/api/ingest/params',
                {
                    'timestamp': time.time(),
                    'params': self.params_snapshot,
                },
            )

    def get_ros_time(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def append_topic_sample(self, topic: str, data: Dict) -> Dict:
        sample = {
            'topic': topic,
            'timestamp': time.time(),
            'data': data,
        }
        self.data_buffer.append(sample)
        return sample

    def scan_callback(self, msg: LaserScan):
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
        sample = self.append_topic_sample(
            '/scan',
            {
                'front_min_distance': round(front_min_distance, 3),
                'range_min': round(float(msg.range_min), 3),
                'range_max': round(float(msg.range_max), 3),
            },
        )
        unix_time = sample['timestamp']
        if (
            front_min_distance < self.scan_threshold
            and not self.is_collecting_post
            and unix_time - self.last_trigger_time > self.cooldown_sec
        ):
            self.trigger_event(unix_time, front_min_distance)

    def odom_callback(self, msg: Odometry):
        self.append_topic_sample(
            '/odom',
            {
                'frame_id': msg.header.frame_id,
                'child_frame_id': msg.child_frame_id,
                'position': {
                    'x': round(msg.pose.pose.position.x, 3),
                    'y': round(msg.pose.pose.position.y, 3),
                },
                'linear_speed': round(msg.twist.twist.linear.x, 3),
                'angular_speed': round(msg.twist.twist.angular.z, 3),
            },
        )

    def cmd_vel_callback(self, msg: Twist):
        self.append_topic_sample(
            '/cmd_vel',
            {
                'linear_x': round(msg.linear.x, 3),
                'angular_z': round(msg.angular.z, 3),
            },
        )

    def costmap_callback(self, msg: OccupancyGrid):
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.append_topic_sample(
            '/local_costmap/costmap',
            {
                'source_ros_timestamp': round(stamp_sec, 3),
                'age_ms': round(
                    max(0.0, (self.get_ros_time() - stamp_sec) * 1000.0),
                    1,
                ),
                'max_cost': int(max(msg.data) if msg.data else 0),
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
            },
        )

    def nav_status_callback(self, msg: GoalStatusArray):
        if not msg.status_list:
            return
        status_code = msg.status_list[-1].status
        status_map = {
            1: 'ACCEPTED',
            2: 'EXECUTING',
            4: 'SUCCEEDED',
            5: 'CANCELED',
            6: 'ABORTED',
        }
        self.append_topic_sample(
            '/navigate_to_pose/_action/status',
            {
                'status_code': status_code,
                'status': status_map.get(status_code, f'CODE_{status_code}'),
            },
        )

    def trigger_event(
        self,
        trigger_unix_time: float,
        min_dist: float,
    ):
        self.get_logger().warn(
            f'Event Triggered! Obstacle distance: {min_dist:.2f}m'
        )
        self.is_collecting_post = True
        self.trigger_window_time = trigger_unix_time
        self.trigger_unix_time = trigger_unix_time
        self.last_trigger_time = trigger_unix_time
        self.trigger_front_min = min_dist
        cutoff = trigger_unix_time - self.history_window_sec
        self.pre_5s_snapshot = [
            sample for sample in self.data_buffer
            if sample['timestamp'] >= cutoff
        ]

    def control_loop(self):
        now_unix_time = time.time()
        cutoff = now_unix_time - (self.history_window_sec + 1.0)
        while self.data_buffer and self.data_buffer[0]['timestamp'] < cutoff:
            self.data_buffer.popleft()

        if (
            self.is_collecting_post
            and now_unix_time - self.trigger_unix_time >= self.post_trigger_sec
        ):
            post_2s_data = [
                sample for sample in self.data_buffer
                if self.trigger_unix_time
                <= sample['timestamp']
                <= self.trigger_unix_time + self.post_trigger_sec
            ]
            self.save_event_json(self.pre_5s_snapshot, post_2s_data)
            self.is_collecting_post = False

    @staticmethod
    def public_samples(samples: List[Dict]) -> List[Dict]:
        return [
            {
                'topic': sample['topic'],
                'timestamp': round(sample['timestamp'], 6),
                'data': sample['data'],
            }
            for sample in samples
        ]

    def save_event_json(self, pre_5s: List[Dict], post_2s: List[Dict]):
        safe_run_id = re.sub(r'[^A-Za-z0-9_.-]', '_', self.run_id)
        filename = (
            f'event_tc01_{safe_run_id}_'
            f'{int(self.trigger_unix_time * 1000)}.json'
        )
        filepath = os.path.join(self.output_dir, filename)
        event_payload = {
            'event_type': self.event_type,
            'trigger_time': round(self.trigger_unix_time, 6),
            'robot_id': self.robot_id,
            'window': {
                'pre_5s': self.public_samples(pre_5s),
                'post_2s': self.public_samples(post_2s),
            },
            'params_snapshot': self.params_snapshot,
            'static_index_version': self.static_index_version,
            'run_id': self.run_id,
            'trigger_distance': round(self.trigger_front_min, 3),
        }

        try:
            self.atomic_write_json(filepath, event_payload)
            self.get_logger().info(
                f'Successfully saved event JSON to: {filepath}'
            )
            if self.backend_base_url:
                self.post_json('/api/ingest/event', event_payload)
        except Exception as error:
            self.get_logger().error(f'Failed to save event JSON: {error}')

    @staticmethod
    def atomic_write_json(filepath: str, payload: Dict):
        directory = os.path.dirname(filepath)
        descriptor, temporary_path = tempfile.mkstemp(
            dir=directory,
            prefix='.event_tc01_',
            suffix='.tmp',
        )
        try:
            with os.fdopen(descriptor, 'w', encoding='utf-8') as event_file:
                os.fchmod(event_file.fileno(), 0o644)
                json.dump(payload, event_file, indent=2, ensure_ascii=False)
                event_file.flush()
                os.fsync(event_file.fileno())
            os.replace(temporary_path, filepath)
        except Exception:
            try:
                os.unlink(temporary_path)
            except OSError:
                pass
            raise

    def post_json(self, endpoint: str, payload: Dict):
        request = urllib.request.Request(
            f'{self.backend_base_url}{endpoint}',
            data=json.dumps(payload, ensure_ascii=False).encode('utf-8'),
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        try:
            with urllib.request.urlopen(
                request,
                timeout=self.http_timeout_sec,
            ) as response:
                if not 200 <= response.status < 300:
                    raise RuntimeError(f'HTTP {response.status}')
            self.get_logger().info(f'Posted payload to {endpoint}')
        except (urllib.error.URLError, OSError, RuntimeError) as error:
            self.get_logger().error(f'POST {endpoint} failed: {error}')


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
