from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    package_share_dir = get_package_share_directory('adaptive_indi_system')
    
    # Path to the YAML config file
    config_file = os.path.join(package_share_dir, 'config', 'adaptive_indi_params.yaml')
    
    # Launch arguments
    enable_indi_arg = DeclareLaunchArgument(
        'enable_indi',
        default_value='true',
        description='Enable INDI controller (true) or use PID only (false)'
    )
    
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value='',
        description='Path to waypoints YAML file (empty = use default waypoints)'
    )
    
    waypoint_radius_arg = DeclareLaunchArgument(
        'waypoint_radius',
        default_value='0.1',
        description='Waypoint acceptance radius in meters'
    )
    
    hover_time_arg = DeclareLaunchArgument(
        'hover_time',
        default_value='0.1',
        description='Time to hover at each waypoint in seconds'
    )
    
    loop_trajectory_arg = DeclareLaunchArgument(
        'loop_trajectory',
        default_value='false',
        description='Loop trajectory continuously (true/false)'
    )

    # =========================================================================
    # NODE 1: Offboard Manager (FIXED - mode-aware!)
    # =========================================================================
    offboard_manager = Node(
        package='adaptive_indi_system',
        executable='offboard_manager_node',
        name='offboard_manager',
        output='screen',
        parameters=[{
            'offboard_setpoint_rate_hz': 20.0,
            'initial_setpoint_duration': 2.0,
            'enable_indi': LaunchConfiguration('enable_indi')  # ← CRITICAL FIX!
        }]
    )

    # =========================================================================
    # NODE 2: Trajectory Planner (generates waypoint setpoints)
    # =========================================================================
    trajectory_planner = Node(
        package='adaptive_indi_system',
        executable='trajectory_planner_node',
        name='trajectory_planner',
        output='screen',
        parameters=[{
            'waypoints_file': LaunchConfiguration('waypoints_file'),
            'waypoint_radius': LaunchConfiguration('waypoint_radius'),
            'hover_time': LaunchConfiguration('hover_time'),
            'loop_trajectory': LaunchConfiguration('loop_trajectory')
        }]
    )

    # =========================================================================
    # NODE 3: Adaptive INDI System (attitude control)
    # Load ALL parameters from adaptive_indi_params.yaml
    # =========================================================================
    adaptive_indi = Node(
        package='adaptive_indi_system',
        executable='adaptive_indi_system_refactored',
        name='adaptive_indi_system',
        output='screen',
        parameters=[
            config_file,  # Load all parameters from YAML file
            {
                'enable_indi': LaunchConfiguration('enable_indi')  # Override with launch arg
            }
        ]
    )

    return LaunchDescription([
        enable_indi_arg,
        waypoints_file_arg,
        waypoint_radius_arg,
        hover_time_arg,
        loop_trajectory_arg,
        offboard_manager,
        trajectory_planner,
        adaptive_indi
    ])
