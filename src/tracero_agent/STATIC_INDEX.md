# Static Topic Index

The current output schema is `v2`. Each source record includes repository
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
  --output /root/turtlebot3_ws/events/static_index.json
```

If `--version` is omitted, the indexer computes an immutable content
fingerprint such as `tc01-a1b2c3d4e5f6` and writes it to
`/root/turtlebot3_ws/events/static_index_version.txt`. The Agent and TC-01
benchmark read this file automatically. Use `--version NAME` only when
reproducing a previously named index.

Upload the same index to the backend:

```bash
ros2 run tracero_agent build_static_index \
  --output /root/turtlebot3_ws/events/static_index.json \
  --backend-base-url http://BACKEND_HOST:PORT \
  --upload \
  --strict
```

Without `--strict`, missing repository commit metadata is allowed for local
development and should be treated as a warning. Use `--strict` for formal
delivery; it rejects any source record without a repository and commit.

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
`highlight_lines` is empty in the static index because static analysis cannot
prove which line caused a runtime fault. B may fill it with runtime-supported
problem lines when constructing the final developer view for C. The optional
`related_params` field is currently an empty array and is reserved for the
parameter mapping added by the data pipeline.

Before uploading, the indexer validates that `line_start`, `line_end`, `code`,
and `highlight_lines` are internally consistent. The backend response version
is also checked when it returns a JSON version.
