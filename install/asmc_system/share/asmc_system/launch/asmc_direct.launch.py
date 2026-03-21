from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    package_share_dir = get_package_share_directory('asmc_system')
    
    # Path to the YAML config file (we still load it for ASMC gains Lam, Phi, etc.)
    config_file = os.path.join(package_share_dir, 'config', 'asmc_parameters.yaml')

    # =========================================================================
    # NODE 1: Offboard Manager
    # =========================================================================
    # Handles PX4 Arming and Offboard Mode switching.
    # We force enable_asmc=True so it tells PX4 to expect Torque & Thrust.
    offboard_manager = Node(
        package='asmc_system',
        executable='offboard_manager_node',
        name='offboard_manager',
        output='screen',
        parameters=[{
            'offboard_setpoint_rate_hz': 20.0,
            'enable_asmc': True 
        }]
    )

    # =========================================================================
    # NODE 2: ASMC System (Direct Control Mode)
    # =========================================================================
    # We pass bypass_pid=True to skip the position/velocity/attitude cascades.
    asmc_system_node = Node(
        package='asmc_system',
        executable='asmc_system_refactored',
        name='asmc_system',
        output='screen',
        parameters=[
            config_file,  # Load ASMC gains
            {
                'enable_asmc': True,
                'bypass_pid': True   # <--- THE CRITICAL NEW PARAMETER
            }
        ]
    )

    # Notice: NO trajectory_planner_node here!

    return LaunchDescription([
        offboard_manager,
        asmc_system_node
    ])