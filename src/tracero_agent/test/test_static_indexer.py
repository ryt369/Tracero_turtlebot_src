import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

from tracero_agent.static_indexer import (
    SourceRoot,
    apply_source_versions,
    build_index,
    load_source_versions,
    upload_index,
)


def write_source(root: Path, relative_path: str, source: str):
    path = root / relative_path
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(source, encoding='utf-8')


def test_python_publishers_and_subscribers(tmp_path):
    write_source(
        tmp_path,
        'agent.py',
        '''
class Agent(Node):
    def __init__(self):
        super().__init__('fixture_agent')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

    def send(self, message):
        self.pub.publish(message)
''',
    )
    payload = build_index(
        [SourceRoot('fixture/src', tmp_path)],
        'v1',
        ['/cmd_vel', '/scan', '/missing'],
    )

    publisher = payload['index']['/cmd_vel']['publishers'][0]
    subscriber = payload['index']['/scan']['subscribers'][0]
    assert publisher['node'] == 'fixture_agent'
    assert publisher['line'] == 9
    assert '.publish(message)' in publisher['snippet']
    assert publisher['message_type'] == 'Twist'
    assert publisher['file_path'] == 'agent.py'
    assert publisher['function_name'] == 'Agent.send'
    assert publisher['line_start'] == 9
    assert publisher['line_end'] == 9
    assert publisher['code'] == ['        self.pub.publish(message)']
    assert publisher['highlight_lines'] == [9]
    assert subscriber['line'] == 6
    assert payload['index']['/missing'] == {
        'publishers': [],
        'subscribers': [],
    }


def test_cpp_publishers_and_subscribers(tmp_path):
    write_source(
        tmp_path,
        'controller.cpp',
        '''
Controller::Controller()
: Node("controller_server")
{
  velocity_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, callback);
}

void Controller::send(geometry_msgs::msg::Twist message)
{
  velocity_pub_->publish(message);
}
''',
    )
    payload = build_index(
        [SourceRoot('fixture/src', tmp_path)],
        'v1',
        ['/cmd_vel', '/scan'],
    )

    publisher = payload['index']['/cmd_vel']['publishers'][0]
    subscriber = payload['index']['/scan']['subscribers'][0]
    assert publisher['node'] == 'controller_server'
    assert publisher['line'] == 12
    assert 'publish(message)' in publisher['snippet']
    assert publisher['message_type'] == 'geometry_msgs::msg::Twist'
    assert publisher['function_name'] == 'Controller::send'
    assert publisher['line_start'] == 12
    assert publisher['line_end'] == 12
    assert publisher['highlight_lines'] == [12]
    assert subscriber['line'] == 6


def test_source_versions_add_repository_and_commit(tmp_path):
    source_versions = tmp_path / 'source-versions.json'
    source_versions.write_text(
        '{"repositories": {"navigation2": {'
        '"commit": "abc123", '
        '"source_root": "nav2_ws/src/navigation2"}}}',
        encoding='utf-8',
    )
    repositories = load_source_versions(source_versions)
    roots = apply_source_versions(
        [SourceRoot('nav2_ws/src/navigation2', tmp_path)],
        repositories,
    )
    payload = build_index(roots, 'v2', ['/scan'])
    assert roots[0].repository == 'navigation2'
    assert roots[0].commit == 'abc123'
    assert payload['version'] == 'v2'


def test_package_and_source_metadata(tmp_path):
    package_root = tmp_path / 'demo_package'
    (package_root / 'package.xml').parent.mkdir(parents=True)
    (package_root / 'package.xml').write_text(
        '<package><name>demo_package</name></package>',
        encoding='utf-8',
    )
    write_source(
        package_root,
        'node.py',
        """class Demo(Node):
    def __init__(self):
        super().__init__('demo')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def send(self, message):
        self.pub.publish(message)
""",
    )
    payload = build_index(
        [SourceRoot(
            'navigation2',
            tmp_path,
            'navigation2',
            'abc123',
        )],
        'v2',
        ['/cmd_vel'],
    )
    record = payload['index']['/cmd_vel']['publishers'][0]
    assert record['repository'] == 'navigation2'
    assert record['commit'] == 'abc123'
    assert record['package'] == 'demo_package'
    assert not record['file_path'].startswith('/')


def test_upload_uses_static_index_endpoint():
    received = {}

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            length = int(self.headers['Content-Length'])
            received['path'] = self.path
            received['payload'] = json.loads(self.rfile.read(length))
            self.send_response(204)
            self.end_headers()

        def log_message(self, format_string, *args):
            pass

    server = HTTPServer(('127.0.0.1', 0), Handler)
    thread = threading.Thread(target=server.handle_request)
    thread.start()
    try:
        payload = {'version': 'v1', 'index': {}}
        upload_index(
            f'http://127.0.0.1:{server.server_port}',
            payload,
            2.0,
        )
    finally:
        thread.join(timeout=2.0)
        server.server_close()

    assert received == {
        'path': '/api/ingest/static_index',
        'payload': payload,
    }
