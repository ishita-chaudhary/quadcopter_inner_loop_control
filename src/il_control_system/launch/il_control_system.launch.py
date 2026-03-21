from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    package_share_dir = get_package_share_directory('il_control_system')
    
    # Path to the YAML config file
    config_file = os.path.join(package_share_dir, 'config', 'il_control_parameters.yaml')
    
    # =========================================================================
    # LAUNCH ARGUMENTS
    # =========================================================================
    
    # Enable Il_Control (true) or use PID-only (false)
    enable_il_control_arg = DeclareLaunchArgument(
        'enable_il_control',
        default_value='true',
        description='Enable Il_Control controller (true) or use PID only (false)'
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
    # We pass 'enable_il_control' so it knows whether to request Torque or Rates.
    offboard_manager = Node(
        package='il_control_system',
        executable='offboard_manager_node',
        name='offboard_manager',
        output='screen',
        parameters=[{
            'offboard_setpoint_rate_hz': 20.0,
            'initial_setpoint_duration': 2.0,
            'enable_il_control': LaunchConfiguration('enable_il_control') 
        }]
    )

    # =========================================================================
    # NODE 2: Trajectory Planner
    # =========================================================================
    # Generates Position Setpoints (no changes needed here mostly)
    trajectory_planner = Node(
        package='il_control_system',
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
    # NODE 3: Il_Control System (Main Controller)
    # =========================================================================
    # Loads params from yaml, but allows launch arg override for enable_il_control
    il_control_system_node = Node(
        package='il_control_system',
        executable='il_control_system_refactored',
        name='il_control_system',
        output='screen',
        parameters=[
            config_file,  # Load PID gains and defaults
            {
                'enable_il_control': LaunchConfiguration('enable_il_control')
            }
        ]
    )

    return LaunchDescription([
        enable_il_control_arg,
        waypoints_file_arg,
        waypoint_radius_arg,
        hover_time_arg,
        loop_trajectory_arg,
        offboard_manager,
        trajectory_planner,
        il_control_system_node
    ])