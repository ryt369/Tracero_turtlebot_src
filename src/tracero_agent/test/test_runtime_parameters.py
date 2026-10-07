from rcl_interfaces.msg import ParameterType, ParameterValue

from tracero_agent.runtime_parameters import (
    build_snapshot,
    parameter_value_to_python,
)


def test_parameter_value_to_python_converts_scalars_and_arrays():
    values = [
        (ParameterType.PARAMETER_BOOL, True, True),
        (ParameterType.PARAMETER_INTEGER, 3, 3),
        (ParameterType.PARAMETER_DOUBLE, 0.25, 0.25),
        (ParameterType.PARAMETER_STRING, 'burger', 'burger'),
    ]
    for value_type, value, expected in values:
        message = ParameterValue(type=value_type)
        if value_type == ParameterType.PARAMETER_BOOL:
            message.bool_value = value
        elif value_type == ParameterType.PARAMETER_INTEGER:
            message.integer_value = value
        elif value_type == ParameterType.PARAMETER_DOUBLE:
            message.double_value = value
        else:
            message.string_value = value
        assert parameter_value_to_python(message) == expected

    message = ParameterValue(
        type=ParameterType.PARAMETER_DOUBLE_ARRAY,
        double_array_value=[0.1, 0.2],
    )
    assert parameter_value_to_python(message) == [0.1, 0.2]


def test_build_snapshot_preserves_partial_node_status_and_warnings():
    snapshot = build_snapshot(
        {
            '/controller_server': {
                'status': 'available',
                'parameters': {
                    'goal_checker.xy_goal_tolerance': 0.001,
                },
                'warnings': [],
            },
            '/amcl': {
                'status': 'unavailable',
                'parameters': {'transform_tolerance': None},
                'warnings': ['/amcl/get_parameters unavailable'],
            },
        },
        1791200000.123456,
    )

    assert snapshot['status'] == 'partial'
    assert snapshot['schema_version'] == 'runtime-params-v1'
    assert snapshot['critical_params'][
        '/controller_server.goal_checker.xy_goal_tolerance'
    ] == 0.001
    assert snapshot['critical_params']['/amcl.transform_tolerance'] is None
    assert snapshot['warnings'] == ['/amcl/get_parameters unavailable']
