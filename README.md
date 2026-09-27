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
