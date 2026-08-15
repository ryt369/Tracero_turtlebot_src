#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Quaternion


def get_yaw_from_quaternion(q: Quaternion) -> float:
    """四元数转 Yaw 偏航角 (弧度)"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class TC01Spawner(Node):
    def __init__(self):
        super().__init__('spawn_tc01')

        # 1. 声明与获取参数
        self.declare_parameter('distance_ahead', 0.75)
        self.declare_parameter('speed_threshold', 0.03)
        self.declare_parameter('motion_timeout_sec', 30.0)
        self.declare_parameter('entity_name', 'tc01_obstacle')

        self.distance_ahead = float(self.get_parameter('distance_ahead').value)
        self.speed_threshold = float(self.get_parameter('speed_threshold').value)
        self.motion_timeout_sec = float(self.get_parameter('motion_timeout_sec').value)
        self.entity_name = str(self.get_parameter('entity_name').value)

        # 2. 状态标识
        self.odom_received = False
        self.spawn_triggered = False
        self.current_pose = None
        self.current_yaw = 0.0
        self.current_speed = 0.0
        self.exit_code = 1
        self.started_at = time.monotonic()

        # 3. 创建 Odom 订阅与 Service 客户端
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        self.timeout_timer = self.create_timer(0.2, self.check_motion_timeout)

        self.get_logger().info(f"TC-01 Spawner Initialized. Target distance ahead: {self.distance_ahead}m")

    def odom_callback(self, msg: Odometry):
        if self.spawn_triggered:
            return

        self.current_pose = msg.pose.pose
        self.current_yaw = get_yaw_from_quaternion(msg.pose.pose.orientation)
        self.current_speed = msg.twist.twist.linear.x
        self.odom_received = True

        # 检查小车是否已经在导航行驶中
        if abs(self.current_speed) > self.speed_threshold:
            self.spawn_triggered = True
            self.spawn_obstacle_in_front()

    def check_motion_timeout(self):
        if self.spawn_triggered:
            return
        elapsed = time.monotonic() - self.started_at
        if elapsed >= self.motion_timeout_sec:
            self.get_logger().error(
                f'Robot did not move within {self.motion_timeout_sec:.1f}s'
            )
            self.exit_code = 2
            rclpy.shutdown()

    def spawn_obstacle_in_front(self):
        self.get_logger().info("Robot motion detected! Calculating obstacle position in front of heading...")

        # 核心算法：按车头朝向推算前方 0.75m 处的目标坐标
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y

        obs_x = robot_x + self.distance_ahead * math.cos(self.current_yaw)
        obs_y = robot_y + self.distance_ahead * math.sin(self.current_yaw)

        # 构造 SDF 实体描述 (0.3m x 0.3m x 0.5m 红/灰颜色立方体)
        obstacle_sdf = """
        <?xml version="1.0" ?>
        <sdf version="1.6">
          <model name="tc01_box">
            <static>true</static>
            <link name="link">
              <visual name="visual">
                <geometry>
                  <box>
                    <size>0.3 0.3 0.5</size>
                  </box>
                </geometry>
                <material>
                  <ambient>0.8 0.1 0.1 1</ambient>
                  <diffuse>0.8 0.1 0.1 1</diffuse>
                </material>
              </visual>
              <collision name="collision">
                <geometry>
                  <box>
                    <size>0.3 0.3 0.5</size>
                  </box>
                </geometry>
              </collision>
            </link>
          </model>
        </sdf>
        """.strip()

        # 校验 Gazebo 服务可用性
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /spawn_entity not available!")
            self.exit_code = 3
            rclpy.shutdown()
            return

        request = SpawnEntity.Request()
        request.name = self.entity_name
        request.xml = obstacle_sdf
        request.robot_namespace = ""
        request.initial_pose.position.x = obs_x
        request.initial_pose.position.y = obs_y
        request.initial_pose.position.z = 0.25  # 半高，贴地放稳
        request.reference_frame = "world"

        self.get_logger().info(
            f"Sending spawn request for '{self.entity_name}' at "
            f"pos: ({obs_x:.2f}, {obs_y:.2f})"
        )
        
        future = self.client.call_async(request)
        future.add_done_callback(self.spawn_callback)

    def spawn_callback(self, future):
        try:
            response = future.result()
            # 显式校验 response.success
            if response.success:
                self.exit_code = 0
                self.get_logger().info(
                    f"Successfully spawned obstacle: {response.status_message}"
                )
            else:
                self.exit_code = 4
                self.get_logger().error(
                    f"Failed to spawn obstacle: {response.status_message}"
                )
        except Exception as e:
            self.exit_code = 5
            self.get_logger().error(f"Service call failed with exception: {str(e)}")
        finally:
            # 执行完成后正常退出节点
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TC01Spawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.exit_code = 130
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    main()
