# 数据说明

## 两种数据区别：

### event_tc01_..._run_05_...json（微观黑匣子）

- 触发机制：仅在测试过程中动态触发。当雷达或传感器检测到机器人与障碍物距离小于设定阈值（如 0.65m）时生成。

- 核心内容：体积较大（通常几千行），包含触发瞬间的时间戳，以及触发前 5 秒与后 2 秒内高频记录的里程计（odom）、速度指令（cmd_vel）和代价地图（Costmap）连续帧数据。

- 主要用途：用于故障复现与算法微观调试，精确排查机器人为什么差一点撞上或发生了碰撞。

### benchmark_tc01_1786776764.json（宏观总结报告）

- 生成机制：在整批测试（如命令设置了跑 5 轮 --runs 5）运行结束或中断退出时必定生成。

- 核心内容：体积较小（通常几十行），包含测试参数（起始点、目标点）、总运行轮次、通过率（pass_rate）以及每一轮（Run 1 ~ Run 5）的最终状态与对应日志索引。

- 主要用途：用于评估导航与避障算法的总体稳定性和成功率，适合作为实验汇报的数据依据。


# 跑数据的方法：

## 具体操作

终端 1，启动 Gazebo：

```
docker exec -it turtlebot3_src bash

source /opt/ros/humble/setup.bash
source /root/nav2_ws/install/setup.bash
source /root/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=30

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

终端 2，启动 Nav2：

```
docker exec -it turtlebot3_src bash

source /opt/ros/humble/setup.bash
source /root/nav2_ws/install/setup.bash
source /root/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=30

ros2 launch turtlebot3_navigation2 navigation2.launch.py \ use_sim_time:=True \ map:=/root/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/map/map.yaml
```

终端 3，先检查环境：

```
docker exec -it turtlebot3_src bash

source /opt/ros/humble/setup.bash
source /root/nav2_ws/install/setup.bash
source /root/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=30

ros2 service type /spawn_entity
ros2 service type /delete_entity
ros2 service type /set_entity_state
ros2 action info /navigate_to_pose
```

先只跑一轮：


```
ros2 run tracero_agent benchmark_tc01 \
  --runs 1 \
  --start-x -2.0 \
  --start-y -0.5 \
  --goal-x 2.5 \
  --goal-y 0.0 \
  --distance-ahead 0.75 \
  --scan-threshold 0.65 \
  --ros-args -p use_sim_time:=true
```

一轮通过后再跑五轮：

```
ros2 run tracero_agent benchmark_tc01 \
  --runs 5 \
  --goal-x 2.5 \
  --goal-y 0.0 \
  --ros-args -p use_sim_time:=true
```