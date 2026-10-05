"""Launch a read-only rosbridge endpoint for Tracero's live dashboard."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start rosbridge with only the topics used by the live dashboard."""
    return LaunchDescription([
        DeclareLaunchArgument('address', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value='9090'),
        DeclareLaunchArgument('retry_startup_delay', default_value='2.0'),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='tracero_rosbridge_websocket',
            output='screen',
            parameters=[{
                'address': LaunchConfiguration('address'),
                'port': LaunchConfiguration('port'),
                'retry_startup_delay': LaunchConfiguration(
                    'retry_startup_delay'
                ),
                # Glob values use rosbridge's "['pattern', ...]" syntax.
                # Subscriptions are limited to dashboard state topics and
                # browser-side topic publication/action/service calls are
                # disabled.  The websocket is not a general ROS control port.
                'topics_sub_glob': (
                    "['/odom', '/plan', '/navigate_to_pose/_action/status']"
                ),
                'topics_pub_glob': '[]',
                'services_glob': '[]',
                'actions_glob': '[]',
                'send_action_goals_in_new_thread': False,
            }],
        ),
    ])
