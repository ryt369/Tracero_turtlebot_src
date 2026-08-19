# Static Topic Index

The static indexer maps ROS 2 topic declarations to Python and C++ source
locations. It keeps unresolved TC-01 topics in the output with empty
`publishers` or `subscribers` arrays.

Install the parser dependencies in the runtime image:

```bash
python3 -m pip install -r requirements-static-index.txt
```

Build the default TC-01 index:

```bash
ros2 run tracero_agent build_static_index \
  --version v1 \
  --output /root/turtlebot3_ws/events/static_index_v1.json
```

Upload the same index to the backend:

```bash
ros2 run tracero_agent build_static_index \
  --version v1 \
  --output /root/turtlebot3_ws/events/static_index_v1.json \
  --backend-base-url http://BACKEND_HOST:PORT \
  --upload
```

Custom roots use a stable label and an absolute path:

```bash
ros2 run tracero_agent build_static_index \
  --source-root turtlebot3_ws/src=/root/turtlebot3_ws/src \
  --source-root nav2_ws/src=/root/nav2_ws/src
```

The `version` value must match each event's `static_index_version`.
