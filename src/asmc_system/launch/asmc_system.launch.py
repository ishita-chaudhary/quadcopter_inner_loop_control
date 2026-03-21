from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    package_share_dir = get_package_share_directory('asmc_system')
    
    # Path to the YAML config file
    config_file = os.path.join(package_share_dir, 'config', 'asmc_parameters.yaml')
    
    # =========================================================================
    # LAUNCH ARGUMENTS
    # =========================================================================
    
    # Enable ASMC (true) or use PID-only (false)
    enable_asmc_arg = DeclareLaunchArgument(
        'enable_asmc',
        default_value='true',
        description='Enable ASMC controller (true) or use PID only (false)'
    )
    
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value='',
        description='Path to waypoints YAML file (empty = use default waypoints)'
    )
    
    waypoint_radius_arg = DeclareLaunchArgument(
        'waypoint_radius',
        default_value='0.3',
        description='Waypoint acceptance radius in meters'
    )
    
    hover_time_arg = DeclareLaunchArgument(
        'hover_time',
        default_value='2.0',
        description='Time to hover at each waypoint in seconds'
    )
    
    loop_trajectory_arg = DeclareLaunchArgument(
        'loop_trajectory',
        default_value='false',
        description='Loop trajectory continuously (true/false)'
    )

    # =========================================================================
    # NODE 1: Offboard Manager
    # =========================================================================
    # Sends the OffboardControlMode heartbeat.
    # We pass 'enable_asmc' so it knows whether to request Torque or Rates.
    offboard_manager = Node(
        package='asmc_system',
        executable='offboard_manager_node',
        name='offboard_manager',
        output='screen',
        parameters=[{
            'offboard_setpoint_rate_hz': 20.0,
            'initial_setpoint_duration': 2.0,
            'enable_asmc': LaunchConfiguration('enable_asmc') 
        }]
    )

    # =========================================================================
    # NODE 2: Trajectory Planner
    # =========================================================================
    # Generates Position Setpoints (no changes needed here mostly)
    trajectory_planner = Node(
        package='asmc_system',
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
    # NODE 3: ASMC System (Main Controller)
    # =========================================================================
    # Loads params from yaml, but allows launch arg override for enable_asmc
    asmc_system_node = Node(
        package='asmc_system',
        executable='asmc_system_refactored',
        name='asmc_system',
        output='screen',
        parameters=[
            config_file,  # Load PID gains and defaults
            {
                'enable_asmc': LaunchConfiguration('enable_asmc')
            }
        ]
    )

    return LaunchDescription([
        enable_asmc_arg,
        waypoints_file_arg,
        waypoint_radius_arg,
        hover_time_arg,
        loop_trajectory_arg,
        offboard_manager,
        trajectory_planner,
        asmc_system_node
    ])