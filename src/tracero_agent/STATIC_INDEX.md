# Static Topic Index

The current output format is `v2`. Each source record includes repository
metadata, package, function, source line range, original code lines, and
highlight candidates. The Docker image creates `/root/source-versions.json`
from the pinned source commits before removing the source repositories' Git
metadata.

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
  --version v2 \
  --output /root/turtlebot3_ws/events/static_index_v2.json
```

Upload the same index to the backend:

```bash
ros2 run tracero_agent build_static_index \
  --version v2 \
  --output /root/turtlebot3_ws/events/static_index_v2.json \
  --backend-base-url http://BACKEND_HOST:PORT \
  --upload
```

Custom roots use a stable label and an absolute path:

```bash
ros2 run tracero_agent build_static_index \
  --source-root turtlebot3_ws/src=/root/turtlebot3_ws/src \
  --source-root nav2_ws/src=/root/nav2_ws/src
```

The `version` value must match each event's `static_index_version`. The event
version identifies the index schema/version; each source record's `commit`
identifies the immutable repository revision that was scanned.

Each mapping uses `file_path` relative to its `repository`. `code` is an array
of original source lines, where `code[0]` corresponds to `line_start`.
`highlight_lines` currently marks the static match range. B may replace it
with the smaller set of problem lines when constructing the final developer
view for C.
