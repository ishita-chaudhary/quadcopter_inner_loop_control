#!/usr/bin/env python3
"""
Frame Verification Tool for PX4 X-Frame

This script helps you verify your motor frame configuration without flying.
It commands small attitude changes and shows which motors respond.

USAGE:
    1. Arm drone with PROPS OFF
    2. Run: python3 verify_frame.py
    3. Follow prompts to test each axis
    4. Compare actual motor response to expected
"""

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleRatesSetpoint, ActuatorMotors
import time

class FrameVerifier(Node):
    def __init__(self):
        super().__init__('frame_verifier')
        
        # Publishers
        self.pub_rates = self.create_publisher(
            VehicleRatesSetpoint,
            '/fmu/in/vehicle_rates_setpoint',
            10)
        
        # Subscriber
        self.sub_motors = self.create_subscription(
            ActuatorMotors,
            '/fmu/in/actuator_motors',
            self.motor_callback,
            10)
        
        self.last_motors = [0.0, 0.0, 0.0, 0.0]
        
    def motor_callback(self, msg):
        self.last_motors = [msg.control[i] for i in range(4)]
    
    def command_rates(self, roll_rate, pitch_rate, yaw_rate, thrust):
        """Send rate command to PX4"""
        msg = VehicleRatesSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.roll = roll_rate
        msg.pitch = pitch_rate
        msg.yaw = yaw_rate
        msg.thrust_body = [0.0, 0.0, -thrust]
        
        self.pub_rates.publish(msg)
    
    def test_axis(self, name, roll, pitch, yaw, expected_increase, expected_decrease):
        """Test one axis and show motor response"""
        print(f"\n{'='*60}")
        print(f"Testing {name}")
        print(f"{'='*60}")
        
        # Baseline
        print("Setting baseline (hover)...")
        for _ in range(10):
            self.command_rates(0.0, 0.0, 0.0, 0.5)
            time.sleep(0.01)
        
        time.sleep(0.2)
        baseline = self.last_motors.copy()
        print(f"Baseline motors: {[f'{m:.3f}' for m in baseline]}")
        
        # Apply test command
        print(f"Commanding {name}...")
        for _ in range(20):
            self.command_rates(roll, pitch, yaw, 0.5)
            time.sleep(0.01)
        
        time.sleep(0.2)
        response = self.last_motors.copy()
        print(f"Response motors: {[f'{m:.3f}' for m in response]}")
        
        # Analyze
        delta = [response[i] - baseline[i] for i in range(4)]
        print(f"Delta:           {[f'{d:+.3f}' for d in delta]}")
        
        # Check expected
        print(f"\nExpected:")
        print(f"  Increase: Motors {expected_increase}")
        print(f"  Decrease: Motors {expected_decrease}")
        
        print(f"\nActual:")
        increased = [i for i in range(4) if delta[i] > 0.02]
        decreased = [i for i in range(4) if delta[i] < -0.02]
        print(f"  Increased: Motors {increased}")
        print(f"  Decreased: Motors {decreased}")
        
        # Verdict
        if set(increased) == set(expected_increase) and set(decreased) == set(expected_decrease):
            print(f"\n✅ {name} CORRECT")
            return True
        else:
            print(f"\n❌ {name} WRONG!")
            print("   Your frame does not match the expected PX4 X-frame")
            return False
    
    def run_verification(self):
        """Run complete frame verification"""
        print("\n" + "="*60)
        print("FRAME VERIFICATION TOOL")
        print("="*60)
        print("\nExpected frame (PX4 X-config):")
        print("      Front")
        print("   2(CW)  0(CCW)")
        print("      \\ /")
        print("       X")
        print("      / \\")
        print("   1(CCW) 3(CW)")
        print("      Back")
        print("\nMotor 0: Front-Right (FR) - Spins CCW")
        print("Motor 1: Back-Left (BL)   - Spins CCW")
        print("Motor 2: Front-Left (FL)  - Spins CW")
        print("Motor 3: Back-Right (BR)  - Spins CW")
        
        input("\n⚠️  MAKE SURE PROPS ARE OFF! Press Enter to continue...")
        
        results = []
        
        # Test Roll (positive = right side down, left side up)
        results.append(self.test_axis(
            name="ROLL (Left Up)",
            roll=0.2,      # Small positive roll rate
            pitch=0.0,
            yaw=0.0,
            expected_increase=[1, 2],  # Left motors (BL, FL)
            expected_decrease=[0, 3]   # Right motors (FR, BR)
        ))
        
        input("\nPress Enter to test PITCH...")
        
        # Test Pitch (positive = nose up)
        results.append(self.test_axis(
            name="PITCH (Nose Up)",
            roll=0.0,
            pitch=0.2,     # Small positive pitch rate
            yaw=0.0,
            expected_increase=[0, 2],  # Front motors (FR, FL)
            expected_decrease=[1, 3]   # Back motors (BL, BR)
        ))
        
        input("\nPress Enter to test YAW...")
        
        # Test Yaw (positive = CCW rotation)
        results.append(self.test_axis(
            name="YAW (CCW Rotation)",
            roll=0.0,
            pitch=0.0,
            yaw=0.2,       # Small positive yaw rate
            expected_increase=[0, 1],  # CCW motors (FR, BL)
            expected_decrease=[2, 3]   # CW motors (FL, BR)
        ))
        
        # Return to zero
        print("\nReturning to zero...")
        for _ in range(10):
            self.command_rates(0.0, 0.0, 0.0, 0.0)
            time.sleep(0.01)
        
        # Summary
        print("\n" + "="*60)
        print("VERIFICATION SUMMARY")
        print("="*60)
        print(f"Roll:  {'✅ PASS' if results[0] else '❌ FAIL'}")
        print(f"Pitch: {'✅ PASS' if results[1] else '❌ FAIL'}")
        print(f"Yaw:   {'✅ PASS' if results[2] else '❌ FAIL'}")
        
        if all(results):
            print("\n🎉 ALL TESTS PASSED!")
            print("Your frame configuration is correct.")
            print("You can proceed to flight testing.")
        else:
            print("\n⚠️  SOME TESTS FAILED!")
            print("Your B matrix does not match your physical frame.")
            print("\nTO FIX:")
            print("1. Check your motor order (which motor is which?)")
            print("2. Check spin directions (CCW vs CW)")
            print("3. Update buildGeometryMatrix() in indi_proper.hpp")
            print("4. Rerun this test")

def main():
    rclpy.init()
    verifier = FrameVerifier()
    
    try:
        # Give time for connections
        time.sleep(1.0)
        verifier.run_verification()
    except KeyboardInterrupt:
        pass
    finally:
        verifier.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
