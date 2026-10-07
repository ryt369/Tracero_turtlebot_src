"""Collect a small, evidence-oriented snapshot from running Nav2 nodes."""

import time
from typing import Any, Dict, Mapping, Sequence

import rclpy
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters


RUNTIME_PARAMETER_SPECS: Mapping[str, Sequence[str]] = {
    '/controller_server': (
        'controller_frequency',
        'goal_checker.xy_goal_tolerance',
        'FollowPath.xy_goal_tolerance',
    ),
    '/local_costmap/local_costmap': (
        'update_frequency',
        'publish_frequency',
        'inflation_layer.inflation_radius',
        'obstacle_layer.scan.obstacle_max_range',
        'voxel_layer.scan.obstacle_max_range',
    ),
    '/global_costmap/global_costmap': (
        'update_frequency',
        'publish_frequency',
        'obstacle_layer.scan.obstacle_max_range',
        'voxel_layer.scan.obstacle_max_range',
    ),
    '/planner_server': (
        'expected_planner_frequency',
    ),
    '/amcl': (
        'update_min_d',
        'update_min_a',
        'transform_tolerance',
    ),
}


def parameter_value_to_python(value: ParameterValue) -> Any:
    """Convert a ROS 2 parameter value into a JSON-compatible value."""
    value_type = value.type
    if value_type == ParameterType.PARAMETER_NOT_SET:
        return None
    if value_type == ParameterType.PARAMETER_BOOL:
        return bool(value.bool_value)
    if value_type == ParameterType.PARAMETER_INTEGER:
        return int(value.integer_value)
    if value_type == ParameterType.PARAMETER_DOUBLE:
        return float(value.double_value)
    if value_type == ParameterType.PARAMETER_STRING:
        return str(value.string_value)
    if value_type == ParameterType.PARAMETER_BYTE_ARRAY:
        return list(value.byte_array_value)
    if value_type == ParameterType.PARAMETER_BOOL_ARRAY:
        return list(value.bool_array_value)
    if value_type == ParameterType.PARAMETER_INTEGER_ARRAY:
        return list(value.integer_array_value)
    if value_type == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return list(value.double_array_value)
    if value_type == ParameterType.PARAMETER_STRING_ARRAY:
        return list(value.string_array_value)
    raise ValueError(f'unsupported ROS parameter type: {value_type}')


def build_snapshot(
    node_results: Mapping[str, Dict[str, Any]],
    captured_at_unix: float,
) -> Dict[str, Any]:
    """Build the stable JSON envelope used by events and backend uploads."""
    critical_params: Dict[str, Any] = {}
    warnings = []
    available_count = 0

    for node_name, result in node_results.items():
        if result.get('status') == 'available':
            available_count += 1
        warnings.extend(result.get('warnings', []))
        for parameter_name, value in result.get('parameters', {}).items():
            critical_params[f'{node_name}.{parameter_name}'] = value

    if available_count == len(node_results):
        status = 'available'
    elif available_count == 0:
        status = 'unavailable'
    else:
        status = 'partial'

    return {
        'schema_version': 'runtime-params-v1',
        'captured_at_unix': round(captured_at_unix, 6),
        'source': 'runtime_ros_parameter_services',
        'status': status,
        'nodes': dict(node_results),
        'critical_params': critical_params,
        'warnings': warnings,
    }


class RuntimeParameterCollector:
    """Read the selected Nav2 parameters without failing the Agent startup."""

    def __init__(self, node, timeout_sec: float = 5.0, retries: int = 2):
        self.node = node
        self.timeout_sec = max(0.1, float(timeout_sec))
        self.retries = max(1, int(retries))

    def collect(self) -> Dict[str, Any]:
        """Query every configured Nav2 node and return a structured snapshot."""
        captured_at_unix = time.time()
        node_results = {
            node_name: self._collect_node(node_name, parameter_names)
            for node_name, parameter_names in RUNTIME_PARAMETER_SPECS.items()
        }
        return build_snapshot(node_results, captured_at_unix)

    def _collect_node(
        self,
        node_name: str,
        parameter_names: Sequence[str],
    ) -> Dict[str, Any]:
        """Query one node, preserving unavailable and partial results."""
        service_name = f'{node_name}/get_parameters'
        client = self.node.create_client(GetParameters, service_name)
        warnings = []
        for attempt in range(1, self.retries + 1):
            if not client.wait_for_service(timeout_sec=self.timeout_sec):
                warnings.append(
                    f'{service_name} unavailable (attempt {attempt}/'
                    f'{self.retries})'
                )
                continue

            request = GetParameters.Request()
            request.names = list(parameter_names)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(
                self.node,
                future,
                timeout_sec=self.timeout_sec,
            )
            if not future.done():
                future.cancel()
                warnings.append(
                    f'{service_name} timed out (attempt {attempt}/'
                    f'{self.retries})'
                )
                continue
            try:
                response = future.result()
            except Exception as error:  # pragma: no cover - middleware error
                warnings.append(f'{service_name} failed: {error}')
                continue
            if response is None or len(response.values) != len(parameter_names):
                warnings.append(f'{service_name} returned an invalid response')
                continue

            parameters = {}
            for name, value in zip(parameter_names, response.values):
                try:
                    parameters[name] = parameter_value_to_python(value)
                except ValueError as error:
                    warnings.append(f'{service_name}/{name}: {error}')
                    parameters[name] = None

            status = 'available' if not warnings else 'partial'
            missing = [name for name, value in parameters.items() if value is None]
            if missing:
                warnings.extend(
                    f'{service_name}/{name} is not declared'
                    for name in missing
                )
                status = 'partial'
            return {
                'status': status,
                'service': service_name,
                'attempts': attempt,
                'parameters': parameters,
                'warnings': warnings,
            }

        return {
            'status': 'unavailable',
            'service': service_name,
            'attempts': self.retries,
            'parameters': {name: None for name in parameter_names},
            'warnings': warnings,
        }
