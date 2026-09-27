from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    start_nav2 = LaunchConfiguration('start_nav2')
    nav2_launch_file = LaunchConfiguration('nav2_launch_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')

    nav2_group = GroupAction(
        condition=IfCondition(start_nav2),
        actions=[
            SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_file),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'autostart': 'true',
                    'params_file': params_file,
                    'map': map_file,
                }.items(),
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_nav2', default_value='true'),
        DeclareLaunchArgument(
            'nav2_launch_file',
            default_value=(
                '/opt/ros/humble/share/nav2_bringup/launch/'
                'bringup_launch.py'
            ),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('map', default_value=''),
        nav2_group,
        Node(
            package='tracero_agent',
            executable='safety_controller',
            name='tracero_safety_controller',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
