import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('adaptive_indi_system')
    
    # Path to YOUR specific config file
    config_file = os.path.join(pkg_share, 'config', 'adaptive_indi_params.yaml')

    # Node: Name MUST match the name in the YAML file
    adaptive_indi_node = Node(
        package='adaptive_indi_system',
        executable='adaptive_indi_system_refactored',
        name='adaptive_indi_system',  # <--- MUST MATCH YAML
        output='screen',
        parameters=[config_file]
    )

    # Offboard Manager
    offboard_manager = Node(
        package='adaptive_indi_system',
        executable='offboard_manager_node',
        name='offboard_manager',
        output='screen',
        parameters=[{'takeoff_altitude': -5.0}]
    )

    return LaunchDescription([
        adaptive_indi_node,
        offboard_manager
    ])



