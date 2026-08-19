#!/usr/bin/env python3
import argparse
import ast
import json
import os
import re
import sys
import tempfile
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


TC01_TOPICS = (
    '/scan',
    '/odom',
    '/cmd_vel',
    '/local_costmap/costmap',
    '/navigate_to_pose/_action/status',
)
SOURCE_SUFFIXES = {'.py': 'python', '.cpp': 'cpp', '.cc': 'cpp',
                   '.cxx': 'cpp', '.h': 'cpp', '.hh': 'cpp',
                   '.hpp': 'cpp', '.hxx': 'cpp'}
EXCLUDED_PARTS = {
    '.git',
    '__pycache__',
    'build',
    'events',
    'install',
    'log',
    'test',
    'tests',
}


@dataclass(frozen=True)
class SourceRoot:
    label: str
    path: Path


@dataclass(frozen=True)
class Location:
    file: str
    line: int
    snippet: str


@dataclass(frozen=True)
class Endpoint:
    topic: str
    role: str
    variable: str
    node: str
    location: Location
    language: str
    message_type: str


def load_tree_sitter():
    try:
        from tree_sitter import Language, Parser
        import tree_sitter_cpp
        import tree_sitter_python
    except ImportError as error:
        raise RuntimeError(
            'tree-sitter dependencies are missing. Install '
            'requirements-static-index.txt before running the indexer.'
        ) from error

    languages = {
        'cpp': Language(tree_sitter_cpp.language()),
        'python': Language(tree_sitter_python.language()),
    }
    return Parser, languages


def make_parser(language_name: str):
    parser_class, languages = load_tree_sitter()
    parser = parser_class()
    parser.language = languages[language_name]
    return parser


def walk_nodes(root_node) -> Iterable:
    stack = [root_node]
    while stack:
        node = stack.pop()
        yield node
        stack.extend(reversed(node.children))


def node_text(source: bytes, node) -> str:
    return source[node.start_byte:node.end_byte].decode(
        'utf-8',
        errors='replace',
    )


def normalized_variable(value: str) -> str:
    identifiers = re.findall(r'[A-Za-z_]\w*', value)
    return identifiers[-1] if identifiers else ''


def normalize_topic(topic: str) -> str:
    topic = topic.strip()
    if not topic:
        return ''
    return topic if topic.startswith('/') else f'/{topic}'


def decode_string_literal(value: str, language: str) -> str:
    value = value.strip()
    if language == 'python':
        try:
            decoded = ast.literal_eval(value)
            return decoded if isinstance(decoded, str) else ''
        except (SyntaxError, ValueError):
            return ''

    match = re.fullmatch(r'(?:u8|u|U|L)?"((?:\\.|[^"\\])*)"', value)
    if not match:
        return ''
    try:
        return bytes(match.group(1), 'utf-8').decode('unicode_escape')
    except UnicodeDecodeError:
        return match.group(1)


def compact_snippet(source_text: str, node, limit: int = 300) -> str:
    lines = source_text.splitlines()
    start = node.start_point[0]
    end = min(node.end_point[0] + 1, start + 4)
    snippet = ' '.join(line.strip() for line in lines[start:end]).strip()
    return snippet if len(snippet) <= limit else f'{snippet[:limit - 3]}...'


def find_ancestor(node, types: Sequence[str]):
    current = node.parent
    while current is not None:
        if current.type in types:
            return current
        current = current.parent
    return None


def assigned_variable(source: bytes, call_node, language: str) -> str:
    if language == 'python':
        assignment = find_ancestor(call_node, ('assignment',))
        if assignment is None:
            return ''
        left = assignment.child_by_field_name('left')
        return normalized_variable(node_text(source, left)) if left else ''

    assignment = find_ancestor(
        call_node,
        ('assignment_expression', 'init_declarator'),
    )
    if assignment is None:
        return ''
    left = assignment.child_by_field_name('left')
    if left is None:
        left = assignment.child_by_field_name('declarator')
    return normalized_variable(node_text(source, left)) if left else ''


def infer_node_name(source_text: str, source_path: Path) -> str:
    patterns = (
        r'super\(\)\.__init__\(\s*["\']([^"\']+)["\']',
        r'(?:LifecycleNode|Node)\s*\(\s*["\']([^"\']+)["\']',
    )
    for pattern in patterns:
        match = re.search(pattern, source_text)
        if match:
            return match.group(1)
    return source_path.stem


def source_location(
    source_text: str,
    node,
    root: SourceRoot,
    source_path: Path,
) -> Location:
    relative = source_path.relative_to(root.path).as_posix()
    return Location(
        file=f'{root.label}/{relative}',
        line=node.start_point[0] + 1,
        snippet=compact_snippet(source_text, node),
    )


def call_arguments(node) -> List:
    arguments = node.child_by_field_name('arguments')
    return list(arguments.named_children) if arguments is not None else []


def parse_python_file(root: SourceRoot, source_path: Path) -> List[Endpoint]:
    source = source_path.read_bytes()
    source_text = source.decode('utf-8', errors='replace')
    tree = make_parser('python').parse(source)
    node_name = infer_node_name(source_text, source_path)
    endpoints = []
    publish_locations: Dict[str, List[Location]] = {}

    for node in walk_nodes(tree.root_node):
        if node.type != 'call':
            continue
        function = node.child_by_field_name('function')
        if function is None:
            continue
        function_text = node_text(source, function)
        method_name = function_text.rsplit('.', 1)[-1]
        if method_name == 'publish':
            variable = normalized_variable(function_text.rsplit('.', 1)[0])
            publish_locations.setdefault(variable, []).append(
                source_location(source_text, node, root, source_path)
            )
            continue
        if method_name not in ('create_publisher', 'create_subscription'):
            continue

        arguments = call_arguments(node)
        if len(arguments) < 2:
            continue
        topic = normalize_topic(
            decode_string_literal(node_text(source, arguments[1]), 'python')
        )
        if not topic:
            continue
        role = 'publishers' if method_name == 'create_publisher' else 'subscribers'
        endpoints.append(Endpoint(
            topic=topic,
            role=role,
            variable=assigned_variable(source, node, 'python'),
            node=node_name,
            location=source_location(source_text, node, root, source_path),
            language='python',
            message_type=node_text(source, arguments[0]),
        ))

    return bind_publish_locations(endpoints, publish_locations)


def cpp_method(function_text: str) -> Tuple[str, str]:
    match = re.search(
        r'(create_publisher|create_subscription)\s*<(.+)>$',
        function_text,
    )
    if not match:
        return '', ''
    return match.group(1), match.group(2).strip()


def parse_cpp_file(root: SourceRoot, source_path: Path) -> List[Endpoint]:
    source = source_path.read_bytes()
    source_text = source.decode('utf-8', errors='replace')
    tree = make_parser('cpp').parse(source)
    node_name = infer_node_name(source_text, source_path)
    endpoints = []
    publish_locations: Dict[str, List[Location]] = {}

    for node in walk_nodes(tree.root_node):
        if node.type != 'call_expression':
            continue
        function = node.child_by_field_name('function')
        if function is None:
            continue
        function_text = node_text(source, function)
        if re.search(r'(?:->|\.)publish$', function_text):
            receiver = re.split(r'(?:->|\.)publish$', function_text)[0]
            variable = normalized_variable(receiver)
            publish_locations.setdefault(variable, []).append(
                source_location(source_text, node, root, source_path)
            )
            continue

        method_name, message_type = cpp_method(function_text)
        if not method_name:
            continue
        arguments = call_arguments(node)
        if not arguments:
            continue
        topic = normalize_topic(
            decode_string_literal(node_text(source, arguments[0]), 'cpp')
        )
        if not topic:
            continue
        role = 'publishers' if method_name == 'create_publisher' else 'subscribers'
        endpoints.append(Endpoint(
            topic=topic,
            role=role,
            variable=assigned_variable(source, node, 'cpp'),
            node=node_name,
            location=source_location(source_text, node, root, source_path),
            language='cpp',
            message_type=message_type,
        ))

    return bind_publish_locations(endpoints, publish_locations)


def bind_publish_locations(
    endpoints: List[Endpoint],
    publish_locations: Dict[str, List[Location]],
) -> List[Endpoint]:
    bound = []
    for endpoint in endpoints:
        locations = [endpoint.location]
        if endpoint.role == 'publishers' and endpoint.variable:
            locations = publish_locations.get(endpoint.variable) or locations
        for location in locations:
            bound.append(Endpoint(
                topic=endpoint.topic,
                role=endpoint.role,
                variable=endpoint.variable,
                node=endpoint.node,
                location=location,
                language=endpoint.language,
                message_type=endpoint.message_type,
            ))
    return bound


def excluded_path(path: Path, root: Path) -> bool:
    relative_parts = path.relative_to(root).parts[:-1]
    return any(
        part in EXCLUDED_PARTS or part.startswith('.codex-backup')
        for part in relative_parts
    )


def source_files(root: SourceRoot) -> Iterable[Tuple[Path, str]]:
    for path in sorted(root.path.rglob('*')):
        language = SOURCE_SUFFIXES.get(path.suffix.lower())
        if language and path.is_file() and not excluded_path(path, root.path):
            yield path, language


def endpoint_payload(endpoint: Endpoint) -> Dict:
    return {
        'node': endpoint.node,
        'file': endpoint.location.file,
        'line': endpoint.location.line,
        'snippet': endpoint.location.snippet,
        'language': endpoint.language,
        'message_type': endpoint.message_type,
    }


def deduplicate(records: List[Dict]) -> List[Dict]:
    unique = {}
    for record in records:
        key = (
            record['node'],
            record['file'],
            record['line'],
            record['language'],
            record['message_type'],
        )
        unique[key] = record
    return sorted(
        unique.values(),
        key=lambda item: (item['file'], item['line'], item['node']),
    )


def build_index(
    roots: Sequence[SourceRoot],
    version: str,
    topics: Sequence[str],
) -> Dict:
    normalized_topics = tuple(dict.fromkeys(normalize_topic(t) for t in topics))
    index = {
        topic: {'publishers': [], 'subscribers': []}
        for topic in normalized_topics if topic
    }
    for root in roots:
        if not root.path.is_dir():
            raise ValueError(f'Source root does not exist: {root.path}')
        for source_path, language in source_files(root):
            try:
                if language == 'python':
                    endpoints = parse_python_file(root, source_path)
                else:
                    endpoints = parse_cpp_file(root, source_path)
            except (OSError, UnicodeError):
                continue
            for endpoint in endpoints:
                if endpoint.topic in index:
                    index[endpoint.topic][endpoint.role].append(
                        endpoint_payload(endpoint)
                    )

    for topic_data in index.values():
        topic_data['publishers'] = deduplicate(topic_data['publishers'])
        topic_data['subscribers'] = deduplicate(topic_data['subscribers'])
    return {'version': version, 'index': index}


def parse_source_root(value: str) -> SourceRoot:
    if '=' not in value:
        raise argparse.ArgumentTypeError(
            'source roots must use LABEL=/absolute/path format'
        )
    label, path = value.split('=', 1)
    if not label.strip() or not path.strip():
        raise argparse.ArgumentTypeError(
            'source roots must contain a non-empty label and path'
        )
    return SourceRoot(label.strip().strip('/'), Path(path).expanduser())


def atomic_write_json(output_path: Path, payload: Dict):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_path = tempfile.mkstemp(
        dir=output_path.parent,
        prefix=f'.{output_path.name}.',
        suffix='.tmp',
    )
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8') as output_file:
            os.fchmod(output_file.fileno(), 0o644)
            json.dump(payload, output_file, indent=2, ensure_ascii=False)
            output_file.flush()
            os.fsync(output_file.fileno())
        os.replace(temporary_path, output_path)
    except Exception:
        try:
            os.unlink(temporary_path)
        except OSError:
            pass
        raise


def upload_index(base_url: str, payload: Dict, timeout_sec: float):
    request = urllib.request.Request(
        f'{base_url.rstrip("/")}/api/ingest/static_index',
        data=json.dumps(payload, ensure_ascii=False).encode('utf-8'),
        headers={'Content-Type': 'application/json'},
        method='POST',
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout_sec) as response:
            if not 200 <= response.status < 300:
                raise RuntimeError(f'HTTP {response.status}')
    except (urllib.error.URLError, OSError) as error:
        raise RuntimeError(f'static index upload failed: {error}') from error


def default_roots() -> List[SourceRoot]:
    candidates = (
        ('turtlebot3_ws/src', Path('/root/turtlebot3_ws/src')),
        ('nav2_ws/src', Path('/root/nav2_ws/src')),
    )
    return [SourceRoot(label, path) for label, path in candidates if path.is_dir()]


def parse_args(argv: Optional[Sequence[str]] = None):
    parser = argparse.ArgumentParser(
        description='Build a tree-sitter ROS2 topic-to-source index',
    )
    parser.add_argument(
        '--source-root',
        action='append',
        type=parse_source_root,
        help='repeatable LABEL=/absolute/path source root',
    )
    parser.add_argument('--version', default='v1')
    parser.add_argument('--topic', action='append')
    parser.add_argument(
        '--output',
        default='/root/turtlebot3_ws/events/static_index_v1.json',
    )
    parser.add_argument('--backend-base-url', default='')
    parser.add_argument('--upload', action='store_true')
    parser.add_argument('--http-timeout-sec', type=float, default=5.0)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None):
    options = parse_args(argv)
    roots = options.source_root or default_roots()
    if not roots:
        print('No source roots were provided or discovered.', file=sys.stderr)
        return 2
    topics = options.topic or list(TC01_TOPICS)
    try:
        payload = build_index(roots, options.version, topics)
        output_path = Path(options.output)
        atomic_write_json(output_path, payload)
        print(f'Static index written to: {output_path}')
        for topic, entries in payload['index'].items():
            print(
                f'  {topic}: {len(entries["publishers"])} publisher(s), '
                f'{len(entries["subscribers"])} subscriber(s)'
            )
        if options.upload:
            if not options.backend_base_url.strip():
                raise ValueError('--upload requires --backend-base-url')
            upload_index(
                options.backend_base_url,
                payload,
                options.http_timeout_sec,
            )
            print('Static index uploaded successfully.')
        return 0
    except (RuntimeError, ValueError, OSError) as error:
        print(f'ERROR: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
