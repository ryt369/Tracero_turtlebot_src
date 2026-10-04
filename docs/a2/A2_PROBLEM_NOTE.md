# A2 costmap 感知距离故障

## 故障定义

A2 将 Nav2 costmap 的 `obstacle_max_range` 从正常值 `2.5 m` 注入为 `0.30 m`。`raytrace_max_range` 保持 `3.0 m`，因此 `/scan` 仍可看到远处障碍物，但 costmap 不会在 0.30 m 之外把它标记为障碍物。

修改的参数路径：

```text
local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.obstacle_max_range
local_costmap.local_costmap.ros__parameters.voxel_layer.scan.obstacle_max_range
global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.obstacle_max_range
global_costmap.global_costmap.ros__parameters.voxel_layer.scan.obstacle_max_range
```

`benchmark_a2.py` 每轮会删除旧障碍、调用 `/reset_world`、在起点前方 0.75 m 生成静态箱体、设置初始位姿、读取实际参数、读取 `/scan`、统计 local/global costmap 占用单元格，再执行导航。动态障碍判据以 rolling local costmap 为准；global costmap 还包含静态地图，只作为辅助观测。核心判据是：

```text
runtime_*_obstacle_max_range = 0.30
scan_detected_obstacle = true
local_costmap_marked_obstacle = false
```

## 构建

```bash
# 每个新终端先加载这三个工作区；benchmark_a2 依赖 nav2_simple_commander
source /opt/ros/humble/setup.bash
source /root/nav2_ws/install/setup.bash
source /root/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger ROS_DOMAIN_ID=30

cd /root/turtlebot3_ws
colcon build --packages-select tracero_agent --symlink-install
```

构建完成后，新的终端仍需执行上面的三个 `source` 命令；只加载
`/opt/ros/humble` 和 `turtlebot3_ws` 会导致 `ModuleNotFoundError:
nav2_simple_commander`。

## 正常参数 smoke test

```bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=true use_composition:=False autostart:=true \
  map:=/root/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/map/map.yaml \
  params_file:=/root/turtlebot3_ws/install/tracero_agent/share/tracero_agent/config/common/burger.yaml
```

```bash
ros2 run tracero_agent benchmark_a2 \
  --runs 1 --start-x -2.0 --start-y -0.5 \
  --goal-x -1.0 --goal-y -0.5 --obstacle-distance 0.75 \
  --expected-range 2.5 --timeout-sec 45 \
  --ros-args -p use_sim_time:=true
```

## A2 故障 smoke test

先停止旧 Nav2，避免同名 lifecycle 节点：

```bash
pkill -INT -f "ros2 launch nav2_bringup bringup_launch.py" || true
pkill -INT -f "/root/nav2_ws/install/nav2_" || true
sleep 5
```

```bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=true use_composition:=False autostart:=true \
  map:=/root/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/map/map.yaml \
  params_file:=/root/turtlebot3_ws/install/tracero_agent/share/tracero_agent/config/a2/burger_A2.yaml
```

确认参数：

```bash
ros2 param get /local_costmap/local_costmap voxel_layer.scan.obstacle_max_range
ros2 param get /global_costmap/global_costmap voxel_layer.scan.obstacle_max_range
```

两项都应为 `Double value is: 0.3`。

```bash
ros2 run tracero_agent benchmark_a2 \
  --runs 1 --start-x -2.0 --start-y -0.5 \
  --goal-x -1.0 --goal-y -0.5 --obstacle-distance 0.75 \
  --expected-range 0.30 --expect-fault --timeout-sec 45 \
  --ros-args -p use_sim_time:=true
```

## 20 次重复验证

正常参数：

```bash
ros2 run tracero_agent benchmark_a2 --runs 20 \
  --start-x -2.0 --start-y -0.5 --goal-x -1.0 --goal-y -0.5 \
  --obstacle-distance 0.75 --expected-range 2.5 --timeout-sec 45 \
  --ros-args -p use_sim_time:=true
```

A2 参数：

```bash
ros2 run tracero_agent benchmark_a2 --runs 20 \
  --start-x -2.0 --start-y -0.5 --goal-x -1.0 --goal-y -0.5 \
  --obstacle-distance 0.75 --expected-range 0.30 --expect-fault \
  --timeout-sec 45 --ros-args -p use_sim_time:=true
```

建议标准：正常配置至少 19/20 次 `local_costmap_marked_obstacle=true`；A2 至少 18/20 次同时满足 `scan_detected_obstacle=true` 和 `local_costmap_marked_obstacle=false`。导航 `task_status` 记录为补充，不作为唯一判据。

## 恢复

停止 A2 Nav2 后，重新使用 `burger.yaml` 启动。确认：

```bash
ros2 param get /local_costmap/local_costmap voxel_layer.scan.obstacle_max_range
```

应恢复为 `Double value is: 2.5`。
