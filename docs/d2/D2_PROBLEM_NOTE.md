# D2 雷达 TF 外参方向故障

## 故障定义

Gazebo SDF 中的雷达物理安装保持向前，`/scan` 仍然使用真实前向测量。故障只修改 `robot_state_publisher` 使用的 URDF 固定关节 `scan_joint`：

正常：

```xml
<origin xyz="-0.032 0 0.172" rpy="0 0 0"/>
```

D2 故障：

```xml
<origin xyz="-0.032 0 0.172" rpy="0 0 3.14159265"/>
```

这相当于把雷达的 TF 朝向旋转 180°，但没有改变 Gazebo 中雷达的物理朝向：

```text
真实雷达看到前方障碍物
  -> /scan 数值仍然正常
  -> base_link -> base_scan TF 反向
  -> Nav2 costmap 将点云投影到机器人后方
```

## 文件和切换

实际被 `robot_state_publisher` 读取的文件是：

```text
/root/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/urdf/turtlebot3_burger.urdf
```

`install/turtlebot3_gazebo/share/.../urdf/turtlebot3_burger.urdf` 是指向该源码文件的符号链接。D2 变体保存在 `tracero_agent/config/d2/`，用工具切换：

```bash
source /opt/ros/humble/setup.bash
source /root/turtlebot3_ws/install/setup.bash

# 正常 TF
ros2 run tracero_agent apply_d2_urdf --variant normal

# D2 故障 TF
ros2 run tracero_agent apply_d2_urdf --variant fault
```

切换前必须停止 Gazebo、`robot_state_publisher` 和 Nav2；切换后重新启动整套仿真。运行中的节点不会重新读取 URDF。

## 构建

```bash
cd /root/turtlebot3_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tracero_agent --symlink-install
source /root/turtlebot3_ws/install/setup.bash
```

## 启动顺序

先应用变体并启动 Gazebo：

```bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=30
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py \
  use_sim_time:=true x_pose:=-2.0 y_pose:=-0.5
```

另一个终端启动 Nav2：

```bash
source /opt/ros/humble/setup.bash
source /root/nav2_ws/install/setup.bash
source /root/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger ROS_DOMAIN_ID=30

ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=true use_composition:=False autostart:=true \
  map:=/root/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/map/map.yaml \
  params_file:=/root/turtlebot3_ws/install/tracero_agent/share/tracero_agent/config/common/burger.yaml
```

`benchmark_d2` 会自动设置初始位姿并等待 Nav2 active。测试过程中停止其它会发布 `/cmd_vel` 的控制器即可；D2 benchmark 本身不发送导航速度。

## Benchmark 判据

每轮会：

1. 重置 Gazebo。
2. 在机器人前方 `0.75 m` 生成静态箱体。
3. 设置 AMCL 初始位姿并清空 costmap。
4. 读取 `/scan`、`base_link -> base_scan` TF、`/local_costmap/costmap` 和 `/odom`。
5. 在机器人前方和后方分别检查 local costmap 占用；判据使用生成箱体前后的新增占用单元格，排除静态地图原有占用。

正常配置：

```text
scan_detected_front_obstacle = true
sensor TF yaw 接近 0
front_costmap_delta_cells >= 5
rear_costmap_delta_cells = 0
```

D2 故障配置：

```text
scan_detected_front_obstacle = true
sensor TF yaw 接近 π
front_costmap_delta_cells = 0
rear_costmap_delta_cells >= 5
```

正常 smoke test：

```bash
ros2 run tracero_agent benchmark_d2 \
  --runs 1 --expected-sensor-yaw 0.0 \
  --model-urdf /root/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/urdf/turtlebot3_burger.urdf
```

D2 smoke test：

```bash
ros2 run tracero_agent benchmark_d2 \
  --runs 1 --expected-sensor-yaw 3.14159265 --expect-fault \
  --model-urdf /root/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/urdf/turtlebot3_burger.urdf
```

报告写入：

```text
/root/turtlebot3_ws/events/benchmark_d2_<timestamp>.json
```

## 重复验证和恢复

正常配置：

```bash
ros2 run tracero_agent apply_d2_urdf --variant normal
ros2 run tracero_agent benchmark_d2 --runs 20 --expected-sensor-yaw 0.0
```

D2 故障配置：

```bash
ros2 run tracero_agent apply_d2_urdf --variant fault
ros2 run tracero_agent benchmark_d2 --runs 20 \
  --expected-sensor-yaw 3.14159265 --expect-fault
```

建议收录标准：正常配置至少 19/20 次通过，故障配置至少 18/20 次通过。测试结束后停止仿真，恢复正常 URDF：

```bash
ros2 run tracero_agent apply_d2_urdf --variant normal
```
