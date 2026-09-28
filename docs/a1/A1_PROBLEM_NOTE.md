# A1 参数故障与 Nav2 排错工作记录

更新时间：2026-09-28

## 1. A1 故障定义

A1 属于 Nav2 控制器参数故障。故障参数是：

```yaml
controller_server:
  ros__parameters:
    goal_checker:
      xy_goal_tolerance: 0.001
```

正常值为：

```yaml
xy_goal_tolerance: 0.25
```

含义：机器人只有在平面位置误差小于该值时，`SimpleGoalChecker` 才会把目标判定为到达。`0.001` 是 1 mm，属于很严格的目标位置阈值。它不是理论上在所有地图和所有目标上都必然失败的参数；是否失败取决于地图、起点、终点、控制器和仿真收敛误差。

当前参数文件：

```text
/root/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/param/burger.yaml
/root/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/param/burger_A1.yaml
```

`burger_A1.yaml` 从正常 `burger.yaml` 复制，只把 `controller_server.goal_checker.xy_goal_tolerance` 从 `0.25` 改成 `0.001`。`FollowPath` 下原有的 `xy_goal_tolerance: 0.05` 没有改动；该字段属于 DWB critic 配置，不是本题用于读取的 goal checker 参数。

## 2. Docker 和 ROS 环境

Docker 容器：

```text
turtlebot3_src
```

容器内工作空间：

```text
/root/turtlebot3_ws
/root/nav2_ws
```

容器内源码和事件报告：

```text
/root/turtlebot3_ws/src/tracero_agent
/root/turtlebot3_ws/src/turtlebot3
/root/turtlebot3_ws/events
```

每个新终端先执行：

```bash
source /opt/ros/humble/setup.bash
source /root/nav2_ws/install/setup.bash
source /root/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=30
```

Windows 侧的 `C:\Users\27229\Documents\ChatGPT\Tracero` 只是本次对话的工作区副本，不是 Docker 的真实源码挂载目录。Docker 的 WSL 挂载关系是：

```text
/home/ryt/workspace/turtlebot3_src/turtlebot3_ws -> /root/turtlebot3_ws
/home/ryt/workspace/turtlebot3_src/nav2_ws       -> /root/nav2_ws
```

## 3. 问题一：Gazebo spawn_entity.py 退出码 1

典型输出：

```text
[ERROR] [spawn_entity.py-4]: process has died ... exit code 1
Spawn status: Entity [burger] already exists.
```

### 原因

通常是重复启动了 `turtlebot3_world.launch.py`：

1. 第一套 Gazebo 已经运行，`burger` 实体已经存在。
2. 第二套 `gzserver` 因端口、资源或 Gazebo master 冲突退出。
3. 第二套 `spawn_entity.py` 找到已有实体 `burger`，报告 `Entity [burger] already exists`。

也可能是 Gazebo server 尚未加载 `libgazebo_ros_factory.so`，导致 `/spawn_entity` 服务暂时不存在。这种情况下要等待服务出现，不要立即重复启动整套 launch。

### 检查

```bash
ps -ef | grep -E "gzserver|gzclient|turtlebot3_world.launch" | grep -v grep
ros2 service list | grep spawn_entity
ros2 topic list | grep -E "clock|scan|odom|cmd_vel"
```

### 推荐处理

如果已有有效 Gazebo，直接使用，不要再次启动：

```bash
ros2 service list | grep spawn_entity
ros2 topic list | grep -E "clock|scan|odom|cmd_vel"
```

如果要完整重启，先结束旧 Gazebo，再只启动一次：

```bash
pkill -INT -f "ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py" || true
pkill -INT -f gzserver || true
pkill -INT -f gzclient || true
sleep 5

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

## 4. 问题二：Nav2 controller_server lifecycle 激活失败

典型输出：

```text
[lifecycle_manager_navigation]: Failed to change state for node: controller_server
[lifecycle_manager_navigation]: Failed to bring up all requested nodes. Aborting bringup.
```

### 实际原因

之前同时启动了两套 Nav2：

```text
旧实例：params_file:=.../burger.yaml
新实例：params_file:=.../burger_A1.yaml
```

两套实例使用相同的节点名：

```text
/controller_server
/planner_server
/amcl
/map_server
/lifecycle_manager_navigation
```

旧节点已经是 `active`，新 lifecycle manager 再次尝试激活同名节点，就会出现：

```text
No transition matching 1 found for current state active
Unable to start transition 1 from current state active
```

这不是 A1 参数格式错误。A1 controller 日志已经证明插件可以创建：

```text
Created goal checker : goal_checker of type nav2_controller::SimpleGoalChecker
Created controller : FollowPath of type dwb_core::DWBLocalPlanner
```

### 检查重复实例

```bash
ps -ef | grep -E "bringup_launch|controller_server|planner_server|lifecycle_manager" | grep -v grep
ros2 node list | grep -E "controller_server|planner_server|amcl|map_server|lifecycle_manager"
```

同名节点出现两次时，先停止所有 Nav2，再启动一套：

```bash
pkill -INT -f "ros2 launch nav2_bringup bringup_launch.py" || true
pkill -INT -f "/root/nav2_ws/install/nav2_" || true
sleep 5
```

然后只运行一条 Nav2 launch 命令。

## 5. 问题三：Gazebo 仿真时间和 Nav2 系统时间不一致

典型日志：

```text
Transform data too old when converting from map to odom
Data time: 1790588197s ...
Transform time: 2051s ...
```

### 原因

Gazebo、里程计和 TF 使用仿真时间，例如几千秒；Nav2 某些节点或 benchmark goal 使用系统墙钟时间，例如 Unix epoch 秒数。controller 拿墙钟时间的 path/goal 去查仿真时间 TF，就会认为 TF 过期。

### 修复

正常参数文件和 A1 参数文件都要包含：

```yaml
/**:
  ros__parameters:
    use_sim_time: true
```

启动 Nav2 时也传入：

```bash
use_sim_time:=true
```

检查：

```bash
ros2 param get /controller_server use_sim_time
ros2 param get /planner_server use_sim_time
ros2 param get /amcl use_sim_time
```

预期全部为：

```text
Boolean value is: True
```

`benchmark_a1.py` 中的 PoseStamped 使用零时间戳：

```python
pose.header.stamp.sec = 0
pose.header.stamp.nanosec = 0
```

零时间戳让 Nav2/TF 使用 latest transform，避免 benchmark 启动早期生成墙钟时间 pose。

## 6. 问题四：没有初始位姿时 map -> odom 不存在

地图文件只保存环境结构，不保存本次启动的机器人定位。Nav2 启动后如果 AMCL 没有收到 `/initialpose`，会出现：

```text
AMCL cannot publish a pose or update the transform. Please set the initial pose.
Invalid frame ID "map"
Timed out waiting for transform from base_link to map
```

发布实验起点：

```bash
ros2 topic pub --once /initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: -2.0, y: -0.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

检查：

```bash
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map odom
```

确认有 `/amcl_pose` 消息并且 `tf2_echo` 能持续输出变换后，才开始导航 benchmark。

## 7. benchmark 的多轮实验修复

原 benchmark 每轮只做了：

```text
cancelTask
setInitialPose
clearAllCostmaps
```

它没有把 Gazebo 中的机器人实体真正移动回起点。第一轮执行后机器人已经移动，第二轮却把 AMCL 假设回起点，造成物理位置、里程计和定位状态不一致。结果是正常参数的多轮基线也可能超时。

当前 `benchmark_a1.py` 每轮增加：

```python
reset_simulation(navigator)
```

该函数调用：

```text
/reset_world
```

然后再发布初始位姿、清空 costmap、查询实际 tolerance 和执行导航。每轮报告增加：

```json
"simulation_reset": true
```

注意：`/reset_world` 会重置整个 Gazebo 世界。当前 A1 场景是静态地图和单机器人，因此适用；如果以后加入动态障碍物，必须在记录中说明 reset 的范围，或者改成专门的机器人实体复位服务。

## 8. 正常参数启动命令

确保 Gazebo 已运行且只运行一套 Nav2：

```bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=true \
  use_composition:=False \
  autostart:=true \
  map:=/root/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/map/map.yaml \
  params_file:=/root/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/param/burger.yaml
```

启动后发布初始位姿，然后检查：

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 param get /controller_server goal_checker.xy_goal_tolerance
```

预期：

```text
active [3]
active [3]
Double value is: 0.25
```

## 9. A1 故障注入方式

A1 不修改 Nav2 C++ 源码，只替换启动参数文件：

```text
正常：burger.yaml
故障：burger_A1.yaml
```

故障启动命令：

```bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=true \
  use_composition:=False \
  autostart:=true \
  map:=/root/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/map/map.yaml \
  params_file:=/root/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/param/burger_A1.yaml
```

确认注入成功：

```bash
ros2 param get /controller_server goal_checker.xy_goal_tolerance
```

预期：

```text
Double value is: 0.001
```

## 10. 单次 benchmark 命令

正常参数 smoke test：

```bash
ros2 run tracero_agent benchmark_a1 \
  --runs 1 \
  --start-x -2.0 --start-y -0.5 \
  --goal-x 0.5 --goal-y -0.5 \
  --expected-tolerance 0.25 \
  --timeout-sec 45 \
  --ros-args -p use_sim_time:=true
```

A1 故障 smoke test：

```bash
ros2 run tracero_agent benchmark_a1 \
  --runs 1 \
  --start-x -2.0 --start-y -0.5 \
  --goal-x 0.5 --goal-y -0.5 \
  --expected-tolerance 0.001 \
  --expect-fault \
  --timeout-sec 45 \
  --ros-args -p use_sim_time:=true
```

注意输出：

```text
A1 benchmark: 1/1 passed
```

只表示 benchmark 判据通过。必须查看 report 中的：

```json
"runtime_xy_goal_tolerance"
"task_status"
"simulation_reset"
"passed"
```

在 `--expect-fault` 模式下，`TIMEOUT` 或 `FAILED` 会被视为故障症状并计为 benchmark pass；`SUCCEEDED` 会被视为故障未表现并计为 benchmark fail。

## 11. 反复运行命令集合

推荐使用相同起点、终点和 timeout，先做正常基线，再切换 A1：

正常参数 20 次：

```bash
ros2 run tracero_agent benchmark_a1 \
  --runs 20 \
  --start-x -2.0 --start-y -0.5 \
  --goal-x 0.5 --goal-y -0.5 \
  --expected-tolerance 0.25 \
  --timeout-sec 45 \
  --ros-args -p use_sim_time:=true
```

A1 参数 20 次：

```bash
ros2 run tracero_agent benchmark_a1 \
  --runs 20 \
  --start-x -2.0 --start-y -0.5 \
  --goal-x 0.5 --goal-y -0.5 \
  --expected-tolerance 0.001 \
  --expect-fault \
  --timeout-sec 45 \
  --ros-args -p use_sim_time:=true
```

正式收录建议：

```text
正常参数：20 次至少 19 次 SUCCEEDED
A1 参数：20 次至少 18 次 TIMEOUT 或 FAILED
```

两组测试之间必须：

1. 停止上一套 Nav2。
2. 确认没有重复的 `/controller_server`、`/planner_server`、`/amcl`。
3. 使用另一份参数文件启动唯一的一套 Nav2。
4. 确认 runtime tolerance 后再运行 benchmark。

查找 report：

```bash
ls -lt /root/turtlebot3_ws/events/benchmark_a1_*.json | head
cat /root/turtlebot3_ws/events/benchmark_a1_<timestamp>.json
```

## 12. 如何恢复正常参数

停止 A1 Nav2：

```bash
pkill -INT -f "ros2 launch nav2_bringup bringup_launch.py" || true
pkill -INT -f "/root/nav2_ws/install/nav2_" || true
sleep 5
```

重新用正常参数启动：

```bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=true \
  use_composition:=False \
  autostart:=true \
  map:=/root/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/map/map.yaml \
  params_file:=/root/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/param/burger.yaml
```

确认恢复：

```bash
ros2 param get /controller_server goal_checker.xy_goal_tolerance
```

预期：

```text
Double value is: 0.25
```

恢复正常参数不需要重新编译 Nav2；A1 是启动时参数注入。只有修改了 `tracero_agent` 源码时才需要重新构建：

```bash
cd /root/turtlebot3_ws
colcon build --packages-select tracero_agent --symlink-install
source /root/turtlebot3_ws/install/setup.bash
```

## 13. 当前已验证结果

修复 benchmark 的 `/reset_world` 后，在同一起点终点和 45 秒 timeout 下：

正常参数：

```text
3/3 SUCCEEDED
```

A1 参数：

```text
3/3 TIMEOUT
```

对应 report：

```text
/root/turtlebot3_ws/events/benchmark_a1_1790596331.json
/root/turtlebot3_ws/events/benchmark_a1_1790596588.json
```

这证明当前场景下 A1 表现稳定，但结论应写成“在当前场景中稳定造成超时”，不能写成“该参数在所有场景中必然导致导航失败”。
