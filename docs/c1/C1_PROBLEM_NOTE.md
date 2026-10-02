# C1 轮距运动学参数故障

## 故障定义

C1 修改 Gazebo 差速驱动插件的 `wheel_separation`。Burger 实际左右轮中心位于
`y=+0.08 m` 和 `y=-0.08 m`，物理轮距为 `0.16 m`。正常模型中的插件参数也是：

```xml
<wheel_separation>0.160</wheel_separation>
```

C1 故障模型把插件参数改成 `0.320 m`。插件会按错误的轮距把 `/cmd_vel` 转换为左右轮速度，实际机器人旋转速度约为期望值的两倍。当前 Gazebo 差速插件也会用实际轮速更新 `/odom`，因此 `/odom` 与 p3d 真值仍会保持一致；故障证据是两者相对命令角速度都放大，而不是两者彼此分离。

```text
/cmd_vel
  -> Gazebo 差速插件按错误轮距计算左右轮速度
  -> 真实模型和 /odom 的 yaw 都偏离命令预期
```

## 文件和切换

实际被 Gazebo 加载的文件是：

```text
/root/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf
```

`install/turtlebot3_gazebo/share/.../model.sdf` 当前是指向该源码文件的符号链接。C1 变体保存在 `tracero_agent/config/c1/`，通过切换工具写回实际加载文件。

每次切换前必须停止 Gazebo，并在启动新的 Gazebo 前应用变体：

```bash
source /opt/ros/humble/setup.bash
source /root/turtlebot3_ws/install/setup.bash

# 正常模型
ros2 run tracero_agent apply_c1_model --variant normal

# C1 故障模型
ros2 run tracero_agent apply_c1_model --variant fault
```

切换工具会打印目标文件的 SHA256。启动 Gazebo 后，不能在运行中替换模型文件来改变已经生成的机器人；必须停止并重新启动 Gazebo。

## 构建

```bash
cd /root/turtlebot3_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tracero_agent --symlink-install
source /root/turtlebot3_ws/install/setup.bash
```

如果 `ros2 run` 找不到新命令，重新 source 上面的 workspace。

## Gazebo 启动

C1 benchmark 不需要 Nav2，但必须有 Gazebo、Burger 和 `/c1_truth/ground_truth`、`/odom`、`/cmd_vel`。两个里程计话题分别来自 Gazebo p3d 真值插件和差速插件：

```bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=30
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py \
  use_sim_time:=true \
  x_pose:=-2.0 y_pose:=-0.5
```

运行 benchmark 时停止 Nav2 或其它会发布 `/cmd_vel` 的控制器，避免多个节点同时控制机器人。

## Benchmark 判据

```bash
ros2 run tracero_agent benchmark_c1 \
  --runs 1 \
  --expected-separation 0.160 \
  --model-sdf /root/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf
```

每轮会重置 Gazebo，发布 `angular.z=0.35 rad/s` 持续 3 秒，然后比较：

```text
odom_yaw_delta
gazebo_ground_truth_yaw_delta
measured_command_to_truth_ratio
```

正常配置要求：

```text
|truth_yaw - odom_yaw| <= 0.20 rad
实际 yaw / 命令 yaw 接近 1
```

C1 配置要求：

```bash
ros2 run tracero_agent benchmark_c1 \
  --runs 1 \
  --expected-separation 0.320 \
  --expect-fault \
  --model-sdf /root/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf
```

故障判据：

```text
|truth_yaw - odom_yaw| <= 0.20 rad
实际 yaw / 命令 yaw >= 1.40
```

报告写入：

```text
/root/turtlebot3_ws/events/benchmark_c1_<timestamp>.json
```

## 反复验证

正常模型执行 20 次：

```bash
ros2 run tracero_agent apply_c1_model --variant normal
ros2 run tracero_agent benchmark_c1 --runs 20 \
  --expected-separation 0.160
```

C1 故障模型执行 20 次：

```bash
ros2 run tracero_agent apply_c1_model --variant fault
ros2 run tracero_agent benchmark_c1 --runs 20 \
  --expected-separation 0.320 --expect-fault
```

建议收录标准：正常配置至少 19/20 次通过，故障配置至少 18/20 次通过。完成测试后恢复正常模型：

```bash
ros2 run tracero_agent apply_c1_model --variant normal
```
