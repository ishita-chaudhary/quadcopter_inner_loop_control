#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Message imports
from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

import matplotlib.pyplot as plt
import numpy as np

class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')

        # --- Data Storage: Drone ---
        self.actual_x, self.actual_y, self.actual_z, self.actual_time = [], [], [], []
        self.des_x, self.des_y, self.des_z, self.des_time = [], [], [], []

        # --- Data Storage: Arm Joints ---
        self.cmd_j1, self.cmd_j2, self.cmd_j3, self.cmd_time = [], [], [], []
        self.act_j1, self.act_j2, self.act_j3, self.act_time = [], [], [], []

        # --- Subscriptions ---
        
        # 1. Actual Drone Position (PX4 uses Best Effort)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.sub_pos = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.pos_callback, px4_qos)

        # 2. Desired Drone Position
        self.sub_setpoint = self.create_subscription(PointStamped, '/control/position_setpoint', self.setpoint_callback, 10)

        # 3. Commanded Arm Position
        self.sub_arm_cmd = self.create_subscription(Float64MultiArray, '/arm_position_controller/commands', self.arm_cmd_callback, 10)

        # 4. Actual Arm Position
        self.sub_joint_states = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.start_time = None
        print("Collecting Data... Press Ctrl+C to stop and plot.")

    def get_time(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.start_time is None:
            self.start_time = now
        return now - self.start_time

    # --- Callbacks ---
    def pos_callback(self, msg):
        t = self.get_time()
        self.actual_x.append(msg.x)
        self.actual_y.append(msg.y)
        self.actual_z.append(msg.z)
        self.actual_time.append(t)

    def setpoint_callback(self, msg):
        t = self.get_time()
        self.des_x.append(msg.point.x)
        self.des_y.append(msg.point.y)
        self.des_z.append(msg.point.z)
        self.des_time.append(t)

    def arm_cmd_callback(self, msg):
        t = self.get_time()
        if len(msg.data) >= 3:
            self.cmd_j1.append(msg.data[0])
            self.cmd_j2.append(msg.data[1])
            self.cmd_j3.append(msg.data[2])
            self.cmd_time.append(t)

    def joint_state_callback(self, msg):
        # The joint_states array order isn't guaranteed, so we search by name
        try:
            j1_idx = msg.name.index('L1_to_L2')
            j2_idx = msg.name.index('L2_to_L3')
            j3_idx = msg.name.index('L3_to_L4')
            
            t = self.get_time()
            self.act_j1.append(msg.position[j1_idx])
            self.act_j2.append(msg.position[j2_idx])
            self.act_j3.append(msg.position[j3_idx])
            self.act_time.append(t)
        except ValueError:
            # Ignore messages that don't contain our arm joints (e.g., rotor joints)
            pass

    # --- Plotting ---
    def plot_results(self):
        if not self.actual_time:
            print("No data received! Check topic names and simulation status.")
            return

        print("Generating Plots...")
        
        # --- Figure 1: 2D Trajectory (X vs Y) ---
        plt.figure("Drone 2D Trajectory", figsize=(8, 6))
        plt.title("Drone Trajectory (Top Down)")
        plt.plot(self.actual_x, self.actual_y, label='Actual Path', color='blue', linewidth=1.5)
        if self.des_x:
            plt.plot(self.des_x, self.des_y, label='Desired Path', color='red', linestyle='--', linewidth=2)
        
        if self.actual_x:
            plt.scatter(self.actual_x[0], self.actual_y[0], color='green', label='Start', zorder=5)
            plt.scatter(self.actual_x[-1], self.actual_y[-1], color='black', label='End', zorder=5)
            
        plt.xlabel("X Position (m)")
        plt.ylabel("Y Position (m)")
        plt.legend()
        plt.grid(True)
        plt.axis('equal')

        # --- Figure 2: Z Error vs Time ---
        if len(self.des_time) > 1 and len(self.actual_time) > 1:
            des_z_interp = np.interp(self.actual_time, self.des_time, self.des_z)
            
            plt.figure("Drone Altitude", figsize=(8, 4))
            plt.title("Altitude Tracking (Z-axis)")
            plt.plot(self.actual_time, self.actual_z, label='Actual Z', color='blue')
            plt.plot(self.actual_time, des_z_interp, label='Desired Z', color='red', linestyle='--')
            plt.xlabel("Time (s)")
            plt.ylabel("Altitude (m) [NED]")
            plt.legend()
            plt.grid(True)

        # --- Figure 3: Arm Joints Tracking ---
        if len(self.cmd_time) > 1 and len(self.act_time) > 1:
            fig, axs = plt.subplots(3, 1, figsize=(10, 8), num="Arm Joint Tracking")
            fig.suptitle('Arm Joint Position Tracking')

            # Joint 1
            axs[0].plot(self.act_time, self.act_j1, label='Actual J1', color='blue')
            axs[0].plot(self.cmd_time, self.cmd_j1, label='Command J1', color='red', linestyle='--')
            axs[0].set_ylabel("Angle (rad)")
            axs[0].legend()
            axs[0].grid(True)

            # Joint 2
            axs[1].plot(self.act_time, self.act_j2, label='Actual J2', color='blue')
            axs[1].plot(self.cmd_time, self.cmd_j2, label='Command J2', color='red', linestyle='--')
            axs[1].set_ylabel("Angle (rad)")
            axs[1].legend()
            axs[1].grid(True)

            # Joint 3
            axs[2].plot(self.act_time, self.act_j3, label='Actual J3', color='blue')
            axs[2].plot(self.cmd_time, self.cmd_j3, label='Command J3', color='red', linestyle='--')
            axs[2].set_xlabel("Time (s)")
            axs[2].set_ylabel("Angle (rad)")
            axs[2].legend()
            axs[2].grid(True)

            plt.tight_layout()

        plt.show()

def main(args=None):
    rclpy.init(args=args)
    plotter = TrajectoryPlotter()

    try:
        rclpy.spin(plotter)
    except KeyboardInterrupt:
        pass
    finally:
        plotter.plot_results()
        plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()