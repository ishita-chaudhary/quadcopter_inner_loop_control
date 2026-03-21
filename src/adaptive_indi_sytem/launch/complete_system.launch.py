import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('adaptive_indi_system')
    config_file = os.path.join(pkg_share, 'config', 'adaptive_indi_params.yaml')

    # Declare launch arguments for easy override
    enable_indi_arg = DeclareLaunchArgument(
        'enable_indi',
        default_value='false',
        description='Enable INDI controller (true/false)'
    )
    
    hover_thrust_arg = DeclareLaunchArgument(
        'hover_thrust',
        default_value='0.54',
        description='Hover thrust (0.0-1.0)'
    )

    adaptive_indi_node = Node(
        package='adaptive_indi_system',
        executable='adaptive_indi_system_refactored',
        name='adaptive_indi_system',
        output='screen',
        parameters=[
            config_file,
            {
                'enable_indi': LaunchConfiguration('enable_indi'),
                'hover_thrust': LaunchConfiguration('hover_thrust'),
            }
        ]
    )

    offboard_manager = Node(
        package='adaptive_indi_system',
        executable='offboard_manager_node',
        name='offboard_manager',
        output='screen',
        parameters=[{'takeoff_altitude': -5.0}]
    )

    return LaunchDescription([
        enable_indi_arg,
        hover_thrust_arg,
        adaptive_indi_node,
        offboard_manager
    ])