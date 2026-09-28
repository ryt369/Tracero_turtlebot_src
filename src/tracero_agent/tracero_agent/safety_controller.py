#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def clamp_towards(current: float, target: float, maximum_delta: float) -> float:
    if abs(target - current) <= maximum_delta:
        return target
    direction = 1.0 if target > current else -1.0
    return current + direction * maximum_delta


def front_minimum(scan: LaserScan, half_fov_rad: float) -> Optional[float]:
    values = []
    for index, distance in enumerate(scan.ranges):
        angle = scan.angle_min + index * scan.angle_increment
        if abs(angle) > half_fov_rad:
            continue
        if math.isfinite(distance) and distance >= scan.range_min:
            maximum = scan.range_max if scan.range_max > 0.0 else float('inf')
            if distance <= maximum:
                values.append(float(distance))
    return min(values) if values else None


class SafetyController(Node):
    def __init__(self):
        super().__init__('tracero_safety_controller')
        self.declare_parameter('input_cmd_topic', '/cmd_vel_nav')
        self.declare_parameter('output_cmd_topic', '/cmd_vel')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('max_decel_linear', 0.8)
        self.declare_parameter('max_decel_angular', 1.5)
        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('sensor_latency', 0.10)
        self.declare_parameter('safety_margin', 0.10)
        self.declare_parameter('minimum_trigger_distance', 0.35)
        self.declare_parameter('release_distance', 0.90)
        self.declare_parameter('clear_duration', 0.50)
        self.declare_parameter('front_fov_degrees', 50.0)

        self.input_cmd_topic = str(
            self.get_parameter('input_cmd_topic').value
        )
        self.output_cmd_topic = str(
            self.get_parameter('output_cmd_topic').value
        )
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.max_decel_linear = float(
            self.get_parameter('max_decel_linear').value
        )
        self.max_decel_angular = float(
            self.get_parameter('max_decel_angular').value
        )
        self.control_frequency = float(
            self.get_parameter('control_frequency').value
        )
        self.sensor_latency = float(
            self.get_parameter('sensor_latency').value
        )
        self.safety_margin = float(
            self.get_parameter('safety_margin').value
        )
        self.minimum_trigger_distance = float(
            self.get_parameter('minimum_trigger_distance').value
        )
        self.release_distance = float(
            self.get_parameter('release_distance').value
        )
        self.clear_duration = float(
            self.get_parameter('clear_duration').value
        )
        self.front_fov_rad = math.radians(
            float(self.get_parameter('front_fov_degrees').value) / 2.0
        )

        self.latest_input = Twist()
        self.latest_scan_distance: Optional[float] = None
        self.measured_speed: Optional[float] = None
        self.output = Twist()
        self.braking = False
        self.clear_since: Optional[float] = None
        self.brake_started_at = 0.0
        self.velocity_before_brake = 0.0

        self.create_subscription(
            Twist, self.input_cmd_topic, self.command_callback, 10
        )
        self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, 10
        )
        self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )
        self.command_publisher = self.create_publisher(
            Twist, self.output_cmd_topic, 10
        )
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray, '/tracero/safety_event', 10
        )
        self.timer = self.create_timer(
            1.0 / max(self.control_frequency, 1.0), self.control_loop
        )

        self.get_logger().info(
            'Safety controller ready: '
            f'{self.input_cmd_topic} -> {self.output_cmd_topic}; '
            f'max_decel_linear={self.max_decel_linear:.3f} m/s^2'
        )

    def command_callback(self, message: Twist):
        self.latest_input = message

    def scan_callback(self, message: LaserScan):
        self.latest_scan_distance = front_minimum(message, self.front_fov_rad)

    def odom_callback(self, message: Odometry):
        self.measured_speed = abs(float(message.twist.twist.linear.x))

    def current_speed(self) -> float:
        if self.measured_speed is not None:
            return self.measured_speed
        return abs(float(self.latest_input.linear.x))

    def trigger_distance(self) -> float:
        speed = self.current_speed()
        stopping_distance = speed * speed / (
            2.0 * max(self.max_decel_linear, 1e-6)
        )
        latency_distance = speed * self.sensor_latency
        return max(
            self.minimum_trigger_distance,
            stopping_distance + latency_distance + self.safety_margin,
        )

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def start_braking(self, now: float):
        self.braking = True
        self.clear_since = None
        self.brake_started_at = now
        self.velocity_before_brake = abs(float(self.output.linear.x))
        self.publish_diagnostic(now, True)
        self.get_logger().warn(
            'Emergency braking triggered: '
            f'distance={self.latest_scan_distance}m, '
            f'input_speed={self.velocity_before_brake:.3f}m/s'
        )

    def can_release(self, now: float) -> bool:
        if self.latest_scan_distance is None:
            self.clear_since = None
            return False
        if self.latest_scan_distance < self.release_distance:
            self.clear_since = None
            return False
        if self.clear_since is None:
            self.clear_since = now
            return False
        return now - self.clear_since >= self.clear_duration

    def control_loop(self):
        now = self.now_seconds()
        if (
            not self.braking
            and self.latest_scan_distance is not None
            and self.latest_scan_distance <= self.trigger_distance()
            and abs(float(self.latest_input.linear.x)) > 1e-3
        ):
            self.start_braking(now)

        target = Twist()
        if not self.braking:
            target = self.latest_input

        period = 1.0 / max(self.control_frequency, 1.0)
        self.output.linear.x = clamp_towards(
            self.output.linear.x,
            target.linear.x,
            self.max_decel_linear * period,
        )
        self.output.linear.y = target.linear.y
        self.output.linear.z = target.linear.z
        self.output.angular.x = target.angular.x
        self.output.angular.y = target.angular.y
        self.output.angular.z = clamp_towards(
            self.output.angular.z,
            target.angular.z,
            self.max_decel_angular * period,
        )
        self.command_publisher.publish(self.output)

        if self.braking and self.can_release(now):
            self.braking = False
            self.clear_since = None
            self.publish_diagnostic(now, False)
            self.get_logger().info('Emergency braking released.')

    def publish_diagnostic(self, now: float, active: bool):
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.WARN if active else DiagnosticStatus.OK
        status.name = 'tracero_safety_controller'
        status.message = 'emergency_brake' if active else 'brake_released'
        values = {
            'event_type': 'emergency_brake' if active else 'brake_released',
            'reason': 'obstacle_near' if active else 'obstacle_cleared',
            'active': str(active).lower(),
            'distance': str(self.latest_scan_distance),
            'velocity_measured': f'{self.current_speed():.6f}',
            'velocity_before': f'{self.velocity_before_brake:.6f}',
            'velocity_command': f'{self.output.linear.x:.6f}',
            'max_decel_linear': f'{self.max_decel_linear:.6f}',
            'brake_duration': f'{max(0.0, now - self.brake_started_at):.6f}',
            'source': 'tracero_safety_controller',
        }
        status.values = [
            KeyValue(key=key, value=value) for key, value in values.items()
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diagnostic_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
