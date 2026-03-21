from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    package_share_dir = get_package_share_directory('il_control_system')
    
    # Path to the YAML config file (we still load it for Il_Control gains Lam, Phi, etc.)
    config_file = os.path.join(package_share_dir, 'config', 'il_control_parameters.yaml')

    # =========================================================================
    # NODE 1: Offboard Manager
    # =========================================================================
    # Handles PX4 Arming and Offboard Mode switching.
    # We force enable_il_control=True so it tells PX4 to expect Torque & Thrust.
    offboard_manager = Node(
        package='il_control_system',
        executable='offboard_manager_node',
        name='offboard_manager',
        output='screen',
        parameters=[{
            'offboard_setpoint_rate_hz': 20.0,
            'enable_il_control': True 
        }]
    )

    # =========================================================================
    # NODE 2: Il_Control System (Direct Control Mode)
    # =========================================================================
    # We pass bypass_pid=True to skip the position/velocity/attitude cascades.
    il_control_system_node = Node(
        package='il_control_system',
        executable='il_control_system_refactored',
        name='il_control_system',
        output='screen',
        parameters=[
            config_file,  # Load Il_Control gains
            {
                'enable_il_control': True,
                'bypass_pid': True   # <--- THE CRITICAL NEW PARAMETER
            }
        ]
    )

    # Notice: NO trajectory_planner_node here!

    return LaunchDescription([
        offboard_manager,
        il_control_system_node
    ])