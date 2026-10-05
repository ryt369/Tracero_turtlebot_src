# Tracero TurtleBot3 Source

ROS 2 Humble integration workspace for TurtleBot3, Gazebo, Nav2 and
`tracero_agent`. The Agent captures TC-01 event windows, writes the agreed JSON
contract, optionally posts data to the backend, and builds a tree-sitter static
topic-to-source index.

## Repository layout

```text
src/tracero_agent/   ROS 2 Agent, TC-01 benchmark and static indexer
src/scripts/         Existing bag extraction utilities
events/              Checked-in TC-01 example output
Dockerfile           Reproducible Humble/Gazebo/Nav2 runtime
docker/              Container entrypoint
```

Fault-injection resources are kept inside the `tracero_agent` package:

```text
src/tracero_agent/config/common/burger.yaml  normal Nav2 parameters
src/tracero_agent/config/a1/burger_A1.yaml  A1 goal-tolerance fault
src/tracero_agent/config/a2/burger_A2.yaml  A2 costmap-range fault
src/tracero_agent/config/c1/                  C1 Gazebo model variants
src/tracero_agent/config/d2/                  D2 URDF variants
```

Generated `build`, `install` and `log` directories are intentionally excluded.
Nav2 and TurtleBot3 runtime packages are installed from Humble binaries. Their
source trees are fetched at pinned commits inside the image for tree-sitter
indexing, but are not rebuilt.

## Build the image

Run from the repository root:

```bash
docker build \
  --build-arg TRACERO_AGENT_COMMIT=$(git rev-parse HEAD) \
  -t tracero-turtlebot3:humble .
```

The image records the pinned Nav2/TurtleBot3 commits and the Agent commit in
`/root/source-versions.json`. If the working tree is not committed, the
Agent value remains `working-tree` and should not be treated as an immutable
release version.

The default Ubuntu package mirror is USTC for stable builds in mainland China.
Override it when needed:

```bash
docker build \
  --build-arg UBUNTU_MIRROR=http://archive.ubuntu.com/ubuntu \
  --build-arg ROS_MIRROR=http://packages.ros.org/ros2/ubuntu \
  -t tracero-turtlebot3:humble .
```

## Start the container

On Linux or WSL2 with Docker Desktop integration:

```bash
xhost +local:docker

docker run -it --rm \
  --name tracero-turtlebot3 \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v tracero-events:/root/turtlebot3_ws/events \
  tracero-turtlebot3:humble
```

The image defaults to `TURTLEBOT3_MODEL=burger`, `ROS_DOMAIN_ID=30`, and
Cyclone DDS. Override them with `docker run -e NAME=value` when required.

## Build the static index

Inside the container:

```bash
ros2 run tracero_agent build_static_index \
  --output /root/turtlebot3_ws/events/static_index.json
```

When `--version` is omitted, the indexer derives an immutable content
fingerprint such as `tc01-a1b2c3d4e5f6` and writes it to
`/root/turtlebot3_ws/events/static_index_version.txt`. Agent and benchmark
read that file automatically, so events reference the same index version.
Use `--version NAME` only when reproducing a previously named index.

Upload it to the backend:

```bash
ros2 run tracero_agent build_static_index \
  --output /root/turtlebot3_ws/events/static_index.json \
  --backend-base-url http://BACKEND_HOST:PORT \
  --upload \
  --strict
```

The index always contains the TC-01 topics. Missing source mappings remain as
empty `publishers` or `subscribers` arrays. Omit `--strict` for local
development when commit metadata is unavailable; use it for formal delivery.

## Run TC-01

Start Gazebo and Nav2 first, then run:

```bash
ros2 run tracero_agent benchmark_tc01 \
  --runs 5 \
  --start-x -2.0 \
  --start-y -0.5 \
  --goal-x 2.5 \
  --goal-y -0.5 \
  --distance-ahead 0.75 \
  --scan-threshold 0.65 \
  --ros-args -p use_sim_time:=true
```

To post parameter snapshots and events, add:

```bash
--backend-base-url http://BACKEND_HOST:PORT
```

See [src/tracero_agent/STATIC_INDEX.md](src/tracero_agent/STATIC_INDEX.md) for
additional static-index options and [events/README.md](events/README.md) for
the event datasets.

## Run TC-01-brake

TC-01-brake routes Nav2 commands through `tracero_safety_controller`:
Nav2 publishes `/cmd_vel_nav`, the safety controller exclusively publishes
`/cmd_vel`, and the Agent records `/tracero/safety_event`. Start Nav2 with its
controller output remapped and the safety controller started:

```bash
ros2 launch tracero_agent tc01_brake_nav2.launch.py \
  nav2_launch_file:=/path/to/bringup_launch.py \
  params_file:=/path/to/nav2_params.yaml \
  map:=/path/to/map.yaml
```

Then run:

```bash
ros2 run tracero_agent benchmark_tc01 \
  --brake \
  --external-safety-controller \
  --brake-input-topic /cmd_vel_nav \
  --runs 1 \
  --ros-args -p use_sim_time:=true
```

The controller limits linear deceleration with `--max-decel-linear` and
publishes a `diagnostic_msgs/msg/DiagnosticArray` event on
`/tracero/safety_event`. Its trigger distance includes stopping distance,
sensor latency, and a safety margin. The static index includes this diagnostic
topic so the backend can associate the safety decision with its source.

## Live dashboard bridge

The optional rosbridge endpoint exposes only the live dashboard topics used by
the frontend: `/odom`, `/plan`, and
`/navigate_to_pose/_action/status`. It is not an event store and should not be
used as a browser control channel. The launch file disables action goals and
does not expose ROS services or parameters.

Install the Humble package once in the runtime image. For image builds, add
this package to the image's apt install list:

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-rosbridge-server
```

Start it after sourcing the ROS and workspace setup files:

```bash
ros2 launch tracero_agent rosbridge.launch.py \
  address:=0.0.0.0 port:=9090
```

The frontend connects to `ws://<machine-a>:9090` with `roslibjs`. Verify the
endpoint before handing it to the frontend:

```bash
ros2 topic list | grep -E '^/(odom|plan|navigate_to_pose/_action/status)$'
ss -ltn | grep ':9090'
```

Keep port 9090 on a trusted network. If the bridge must be exposed beyond the
local machine, put it behind an authenticated reverse proxy and keep command
topics outside the allowed topic scope.

## A1/A2 parameter files

Build and source `tracero_agent` before launching Nav2 so the package resources
are available under `install/tracero_agent/share/tracero_agent/config`:

```bash
cd /root/turtlebot3_ws
colcon build --packages-select tracero_agent --symlink-install
source /root/turtlebot3_ws/install/setup.bash
```

Use these paths with Nav2's `params_file` argument:

```text
normal: /root/turtlebot3_ws/install/tracero_agent/share/tracero_agent/config/common/burger.yaml
A1:     /root/turtlebot3_ws/install/tracero_agent/share/tracero_agent/config/a1/burger_A1.yaml
A2:     /root/turtlebot3_ws/install/tracero_agent/share/tracero_agent/config/a2/burger_A2.yaml
```

## C1 Gazebo wheel-separation fault

C1 changes the Gazebo differential-drive plugin's `wheel_separation` from the
normal `0.160 m` to the fault value `0.320 m`. The robot's physical wheel
positions do not change, but the plugin converts angular velocity commands
using the wrong kinematic parameter, so the measured yaw response is too large.
The C1 benchmark compares `/cmd_vel`, `/odom`, and Gazebo ground truth; Nav2 is
not required.

Apply a model variant before starting Gazebo. Stop and restart Gazebo whenever
the variant changes:

```bash
source /opt/ros/humble/setup.bash
source /root/turtlebot3_ws/install/setup.bash

# normal model
ros2 run tracero_agent apply_c1_model --variant normal
# C1 fault model
ros2 run tracero_agent apply_c1_model --variant fault
```

The variants are stored in `config/c1/model.sdf` and
`config/c1/model_C1.sdf`. Run the benchmark with the model currently loaded by
Gazebo:

```bash
ros2 run tracero_agent benchmark_c1 --runs 1 \
  --expected-separation 0.160

ros2 run tracero_agent benchmark_c1 --runs 1 \
  --expected-separation 0.320 --expect-fault
```

## D2 lidar TF extrinsic-direction fault

D2 changes the `base_link -> base_scan` fixed joint in the URDF. The normal
sensor yaw is `0.0`; the fault variant uses `3.14159265 rad` (180 degrees).
Gazebo's physical lidar still points forward, while `robot_state_publisher`
reports the sensor frame facing backward. `/scan` therefore still sees a front
obstacle, but Nav2's costmap can place the observation behind the robot.

Apply the URDF variant before restarting Gazebo, `robot_state_publisher`, and
Nav2:

```bash
source /opt/ros/humble/setup.bash
source /root/turtlebot3_ws/install/setup.bash

# normal TF
ros2 run tracero_agent apply_d2_urdf --variant normal
# D2 fault TF
ros2 run tracero_agent apply_d2_urdf --variant fault
```

Run the benchmark after the full simulation stack is active:

```bash
ros2 run tracero_agent benchmark_d2 --runs 1 \
  --expected-sensor-yaw 0.0

ros2 run tracero_agent benchmark_d2 --runs 1 \
  --expected-sensor-yaw 3.14159265 --expect-fault
```

The D2 variants are stored in `config/d2/turtlebot3_burger.urdf` and
`config/d2/turtlebot3_burger_D2.urdf`. Restore the normal variant after testing:

```bash
ros2 run tracero_agent apply_d2_urdf --variant normal
```

## 中文说明

### A1：Nav2 目标位置容差故障

A1 修改 `controller_server.goal_checker.xy_goal_tolerance`。正常值为
`0.25 m`，故障值为 `0.001 m`。故障值要求机器人几乎精确停在目标点，可能导致
导航长时间收敛或任务超时。使用 `config/a1/burger_A1.yaml` 启动 Nav2，使用
`config/common/burger.yaml` 作为正常基线。该故障由 `benchmark_a1` 检查运行时
参数和导航结果。

### A2：costmap 障碍物感知距离故障

A2 将 local/global costmap 的 `obstacle_max_range` 从正常值 `2.5 m` 改为
`0.30 m`。激光雷达仍可能检测到远处障碍物，但 costmap 不会在 0.30 m 以外标记
它。使用 `config/a2/burger_A2.yaml` 启动故障 Nav2，并用 `benchmark_a2` 同时检查
运行时参数、`/scan` 和 costmap 结果。

### C1：Gazebo 轮距运动学故障

C1 将 Gazebo 差速插件的 `wheel_separation` 从正常的 `0.160 m` 注入为
`0.320 m`。物理轮子位置没有改变，但插件按照错误轮距计算轮速，因此机器人的
实际角速度响应会偏大。C1 主要比较 `/cmd_vel`、`/odom` 和 Gazebo 真值，不需要
启动 Nav2。切换模型前必须停止 Gazebo，切换后重新启动。

配置文件：

```text
config/c1/model.sdf       正常模型
config/c1/model_C1.sdf    C1 故障模型
```

切换和验证：

```bash
ros2 run tracero_agent apply_c1_model --variant normal
ros2 run tracero_agent benchmark_c1 --runs 1 --expected-separation 0.160

ros2 run tracero_agent apply_c1_model --variant fault
ros2 run tracero_agent benchmark_c1 --runs 1 \
  --expected-separation 0.320 --expect-fault
```

### D2：雷达 TF 外参方向故障

D2 将 URDF 中 `base_link -> base_scan` 固定关节的 yaw 从 `0` 改为
`3.14159265 rad`。Gazebo 中雷达物理方向仍向前，但 TF 把雷达坐标系报告为向后，
会使 Nav2 costmap 将前方观测投影到后方。`/scan` 仍可能检测到前方障碍物，故障
证据是传感器 TF 方向和 costmap 占用位置不一致。

配置文件：

```text
config/d2/turtlebot3_burger.urdf       正常 URDF
config/d2/turtlebot3_burger_D2.urdf    D2 故障 URDF
```

切换前必须停止 Gazebo、`robot_state_publisher` 和 Nav2，切换后重新启动完整仿真：

```bash
ros2 run tracero_agent apply_d2_urdf --variant normal
ros2 run tracero_agent benchmark_d2 --runs 1 --expected-sensor-yaw 0.0

ros2 run tracero_agent apply_d2_urdf --variant fault
ros2 run tracero_agent benchmark_d2 --runs 1 \
  --expected-sensor-yaw 3.14159265 --expect-fault

# 测试结束后恢复
ros2 run tracero_agent apply_d2_urdf --variant normal
```

四类故障的详细重复测试、判据和恢复步骤分别见仓库中的
`docs/a1/A1_PROBLEM_NOTE.md`、`docs/a2/A2_PROBLEM_NOTE.md`、
`docs/c1/C1_PROBLEM_NOTE.md` 和 `docs/d2/D2_PROBLEM_NOTE.md`。
