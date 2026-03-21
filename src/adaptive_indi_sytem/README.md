# Adaptive INDI Control System

A modular, cascaded control system for quadcopter position control with adaptive INDI as the inner loop.

## Architecture

```
Position Setpoint → [Outer Loop: Position Controller] → Velocity Command
                                   ↓ (20 Hz)
                    [Middle Loop: Velocity Controller] → Angular Velocity Command
                                   ↓ (50 Hz)
                    [Rate Controller] → Angular Acceleration Command
                                   ↓ (50 Hz)
                    [Inner Loop: Adaptive INDI] → Motor Thrust Commands
                                   ↓ (100-400 Hz, ESC rate)
                             Motor Outputs
```

## Control Loops

1. **Outer Loop (20 Hz)**: Position PID → Velocity commands
   - Input: Desired position (x, y, z)
   - Output: Desired velocity (vx, vy, vz)
   
2. **Middle Loop (50 Hz)**: Velocity PID → Angular rate commands
   - Input: Desired velocity
   - Output: Desired angular velocity (roll rate, pitch rate)
   
3. **Rate Loop (50 Hz)**: Angular rate PID → Angular acceleration commands
   - Input: Desired angular velocity
   - Output: Desired angular acceleration
   
4. **Inner Loop (ESC rate)**: Adaptive INDI → Motor thrust
   - Input: Desired angular acceleration
   - Output: Normalized motor thrust [0, 1]
   - Adaptive: Updates control effectiveness matrix G online

## Topics

### Subscribed Topics:
- `/control/position_setpoint` (geometry_msgs/PointStamped): Desired position
- `/fmu/out/vehicle_local_position` (px4_msgs/VehicleLocalPosition): Current position and velocity
- `/fmu/out/vehicle_angular_velocity` (px4_msgs/VehicleAngularVelocity): Current angular velocity
- `/fmu/out/esc_status` (px4_msgs/EscStatus): Motor RPM feedback

### Published Topics:
- `/fmu/in/actuator_motors` (px4_msgs/ActuatorMotors): Motor thrust commands [0, 1]

## Installation

```bash
cd ~/rl_indi/src
cp -r /path/to/adaptive_indi_system .

cd ~/rl_indi
colcon build --packages-select adaptive_indi_system
source install/setup.bash
```

## Usage

### Start the controller:
```bash
ros2 run adaptive_indi_system adaptive_indi_system_node
```

### Or use the launch file:
```bash
ros2 launch adaptive_indi_system adaptive_indi_system.launch.py
```

### Send position commands:
```bash
# Hover at (0, 0, -2) meters (2m altitude)
ros2 topic pub /control/position_setpoint geometry_msgs/msg/PointStamped "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'},
  point: {x: 0.0, y: 0.0, z: -2.0}
}" --once

# Move to (1, 1, -2) meters
ros2 topic pub /control/position_setpoint geometry_msgs/msg/PointStamped "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'},
  point: {x: 1.0, y: 1.0, z: -2.0}
}" --once
```

## Tuning Parameters

### Position Controller (Outer Loop):
Located in `controllers.hpp` → `PositionController` constructor
```cpp
pid_x_(kp, ki, kd, i_max, output_max)
// Default: (1.0, 0.0, 0.5, 2.0, 5.0)
// output_max: maximum velocity command (m/s)
```

### Velocity Controller (Middle Loop):
Located in `controllers.hpp` → `VelocityController` constructor
```cpp
pid_vx_(kp, ki, kd, i_max, output_max)
// Default: (2.0, 0.1, 0.3, 5.0, 30.0)
// output_max: maximum angular rate (deg/s)
```

### Angular Rate Controller:
Located in `controllers.hpp` → `AngularRateController` constructor
```cpp
pid_roll_(kp, ki, kd, i_max, output_max)
// Default: (10.0, 0.0, 0.5, 10.0, 100.0)
// output_max: maximum angular acceleration (rad/s²)
```

### Adaptive INDI:
Located in `controllers.hpp` → `AdaptiveINDI` class
```cpp
float mu_ = 1e-4f;  // Adaptation gain
const float MAX_RPM = 10000.0f;  // Adjust based on your motors
```

## Monitoring

### Check if running:
```bash
ros2 node list | grep adaptive_indi_system
```

### Monitor motor commands:
```bash
ros2 topic echo /fmu/in/actuator_motors
```

### Monitor position:
```bash
ros2 topic echo /fmu/out/vehicle_local_position
```

### Enable debug logging:
```bash
ros2 run adaptive_indi_system adaptive_indi_system_node --ros-args --log-level debug
```

## Safety Notes

1. **Test in simulation first!**
2. Always have manual override ready
3. Start with conservative gains
4. The system requires:
   - Valid position estimates
   - ESC feedback (RPM)
   - Angular velocity measurements
5. Motors will only activate when all sensors are valid

## Troubleshooting

### No motor output:
Check that all sensor topics are publishing:
```bash
ros2 topic hz /fmu/out/vehicle_local_position
ros2 topic hz /fmu/out/vehicle_angular_velocity
ros2 topic hz /fmu/out/esc_status
```

### Unstable flight:
1. Reduce outer loop gains (position PID)
2. Reduce middle loop gains (velocity PID)
3. Check INDI adaptation gain `mu_`
4. Verify `MAX_RPM` matches your motors

### Position drift:
1. Increase position controller integral gain
2. Check position estimate quality
3. Verify frame of reference (NED vs ENU)

## Code Structure

```
adaptive_indi_system/
├── include/adaptive_indi_system/
│   └── controllers.hpp          # All controller classes
├── src/
│   └── adaptive_indi_system_node.cpp  # Main ROS 2 node
├── launch/
│   └── adaptive_indi_system.launch.py
├── CMakeLists.txt
├── package.xml
└── README.md
```

## License

MIT
