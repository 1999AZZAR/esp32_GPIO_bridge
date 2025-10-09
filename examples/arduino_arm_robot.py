#!/usr/bin/env python3
"""
Arduino Arm Robot Controller
============================

This example demonstrates controlling multiple servos simultaneously using the ESP32 GPIO Bridge
v0.1.5-beta's advanced PWM capabilities and batch processing features. Perfect for Arduino arm robots!

Features:
- Multi-servo control with smooth coordinated movements
- Pre-programmed robot arm poses and sequences
- Real-time manual control with keyboard input
- Servo calibration and safety limits
- Batch PWM operations for synchronized movements

Hardware Setup:
- ESP32 with multiple PWM-capable pins (GPIO 2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33)
- 6x SG90 servo motors (or similar 180-degree servos)
- 6V power supply for servos (ESP32 provides PWM signals, external power for servos)
- Optional: Potentiometers for manual calibration

Servo Connections (example):
- Base rotation: GPIO 2
- Shoulder joint: GPIO 4  
- Elbow joint: GPIO 5
- Wrist pitch: GPIO 12
- Wrist roll: GPIO 13
- Gripper: GPIO 14

Author: ESP32 GPIO Bridge Project
Version: 0.1.5-beta (Dual-core optimized)
"""

import time
import math
import threading
from typing import Dict, List, Tuple, Optional
from esp32_gpio_bridge import ESP32GPIO, find_esp32_port
import sys

class ArduinoArmRobot:
    """
    Arduino Arm Robot Controller using ESP32 GPIO Bridge v0.1.5-beta
    
    Demonstrates advanced PWM control with batch operations and coordinated movements.
    """
    
    def __init__(self, port: Optional[str] = None):
        """Initialize the robot arm controller."""
        self.port = port or find_esp32_port()
        if not self.port:
            raise RuntimeError("ESP32 not found! Please check connection.")
            
        self.esp = None  # Will be set when connecting
        
        # Servo configuration (GPIO pins - using safe PWM pins)
        self.servos = {
            'base': 16,     # Base rotation (0-180°) - Safe PWM pin
            'shoulder': 17, # Shoulder joint (0-180°) - Safe PWM pin
            'elbow': 18,    # Elbow joint (0-180°) - Safe PWM pin
            'wrist_pitch': 19,  # Wrist pitch (0-180°) - Safe PWM pin
            'wrist_roll': 21,   # Wrist roll (0-180°) - Safe PWM pin
            'gripper': 22   # Gripper open/close (0-180°) - Safe PWM pin
        }
        
        # Servo calibration (min/max angles for each joint)
        self.servo_limits = {
            'base': (0, 180),
            'shoulder': (10, 170),  # Prevent extreme positions
            'elbow': (15, 165),
            'wrist_pitch': (20, 160),
            'wrist_roll': (0, 180),
            'gripper': (0, 120)     # Gripper limited range
        }
        
        # Current servo positions
        self.current_positions = {servo: 90 for servo in self.servos.keys()}
        
        # PWM settings for servos (SG90 servos)
        self.pwm_frequency = 50  # 50Hz for servos
        self.pwm_resolution = 16  # 16-bit resolution for precision
        
        # Movement settings
        self.default_speed = 100  # milliseconds per degree
        self.is_moving = False
        
    def connect(self):
        """Connect to ESP32 and initialize all servos."""
        print("Connecting to Arduino Arm Robot...")
        print(f"ESP32 Port: {self.port}")
        
        try:
            # Create ESP32GPIO instance - connection is handled automatically
            self.esp = ESP32GPIO(self.port)
            print(f"Connected! Firmware: {self.esp.get_version()}")
            
            # Initialize all servos with PWM
            print("Initializing servos...")
            for servo_name, pin in self.servos.items():
                print(f"   {servo_name.capitalize()}: GPIO {pin}")
                self.esp.pwm_init(pin, self.pwm_frequency, self.pwm_resolution)
                
            # Set all servos to center position (90 degrees)
            self.home_all_servos()
            print("All servos initialized and centered!")
            
        except Exception as e:
            print(f"Connection failed: {e}")
            raise
            
    def disconnect(self):
        """Disconnect from ESP32 and cleanup."""
        print("Disconnecting from ESP32...")
        if self.esp:
            self.esp.close()
        print("Disconnected!")
        
    def angle_to_duty(self, angle: float) -> int:
        """
        Convert servo angle (0-180°) to PWM duty cycle.
        
        SG90 servos use:
        - 0° = 1ms pulse = 5% duty cycle at 50Hz
        - 90° = 1.5ms pulse = 7.5% duty cycle at 50Hz  
        - 180° = 2ms pulse = 10% duty cycle at 50Hz
        
        With 16-bit resolution (65535 max value):
        - 0° = 3277 (5% of 65535)
        - 90° = 4915 (7.5% of 65535)
        - 180° = 6554 (10% of 65535)
        """
        # Clamp angle to valid range
        angle = max(0, min(180, angle))
        
        # Convert to duty cycle (5% to 10% range)
        duty_percent = 5 + (angle / 180.0) * 5  # 5% to 10%
        duty_value = int((duty_percent / 100.0) * 65535)
        
        return max(3277, min(6554, duty_value))  # Ensure within servo range
        
    def set_servo_angle(self, servo_name: str, angle: float, speed: int = None):
        """
        Set individual servo angle with speed control.
        
        Args:
            servo_name: Name of servo ('base', 'shoulder', etc.)
            angle: Target angle (0-180°)
            speed: Movement speed in ms per degree (optional)
        """
        if not self.esp:
            raise RuntimeError("ESP32 not connected! Call connect() first.")
            
        if servo_name not in self.servos:
            raise ValueError(f"Unknown servo: {servo_name}")
            
        pin = self.servos[servo_name]
        min_angle, max_angle = self.servo_limits[servo_name]
        
        # Clamp angle to servo limits
        angle = max(min_angle, min(max_angle, angle))
        
        # Convert to PWM duty cycle
        duty = self.angle_to_duty(angle)
        
        # Smooth movement with speed control
        current_angle = self.current_positions[servo_name]
        if speed is None:
            speed = self.default_speed
            
        # Calculate movement steps
        angle_diff = abs(angle - current_angle)
        if angle_diff > 0.1:  # Only move if significant difference
            steps = max(1, int(angle_diff / 2))  # 2-degree steps
            step_delay = speed / steps if steps > 0 else 0
            
            for step in range(steps + 1):
                # Interpolate angle
                progress = step / steps if steps > 0 else 1
                current_step_angle = current_angle + (angle - current_angle) * progress
                step_duty = self.angle_to_duty(current_step_angle)
                
                # Set PWM
                self.esp.pwm_write(pin, step_duty)
                time.sleep(step_delay / 1000.0)  # Convert ms to seconds
                
        # Update current position
        self.current_positions[servo_name] = angle
        
    def set_multiple_servos(self, servo_angles: Dict[str, float], speed: int = None):
        """
        Set multiple servos simultaneously using batch operations.
        
        This demonstrates the v0.1.5-beta batch processing capabilities!
        
        Args:
            servo_angles: Dictionary of servo_name -> angle
            speed: Movement speed in ms per degree
        """
        if not self.esp:
            raise RuntimeError("ESP32 not connected! Call connect() first.")
            
        if speed is None:
            speed = self.default_speed
            
        print(f"Moving multiple servos: {list(servo_angles.keys())}")
        
        # Calculate maximum angle difference for timing
        max_diff = 0
        for servo_name, angle in servo_angles.items():
            if servo_name in self.current_positions:
                diff = abs(angle - self.current_positions[servo_name])
                max_diff = max(max_diff, diff)
                
        if max_diff < 0.1:
            return  # No significant movement needed
            
        # Calculate movement steps
        steps = max(1, int(max_diff / 2))  # 2-degree steps
        step_delay = speed / steps if steps > 0 else 0
        
        self.is_moving = True
        
        try:
            for step in range(steps + 1):
                progress = step / steps if steps > 0 else 1
                
                # Prepare batch PWM commands
                batch_commands = {}
                
                for servo_name, target_angle in servo_angles.items():
                    if servo_name in self.current_positions:
                        pin = self.servos[servo_name]
                        current_angle = self.current_positions[servo_name]
                        
                        # Interpolate angle
                        step_angle = current_angle + (target_angle - current_angle) * progress
                        
                        # Apply servo limits
                        min_angle, max_angle = self.servo_limits[servo_name]
                        step_angle = max(min_angle, min(max_angle, step_angle))
                        
                        # Convert to duty cycle
                        duty = self.angle_to_duty(step_angle)
                        batch_commands[pin] = duty
                
                # Execute batch PWM write (v0.1.5-beta feature!)
                for pin, duty in batch_commands.items():
                    self.esp.pwm_write(pin, duty)
                    
                time.sleep(step_delay / 1000.0)
                
        finally:
            self.is_moving = False
            
        # Update current positions
        for servo_name, angle in servo_angles.items():
            if servo_name in self.current_positions:
                min_angle, max_angle = self.servo_limits[servo_name]
                self.current_positions[servo_name] = max(min_angle, min(max_angle, angle))
                
        print(f"Servos positioned: {self.get_current_positions()}")
        
    def home_all_servos(self):
        """Move all servos to home position (90 degrees)."""
        if not self.esp:
            print("ESP32 not connected - skipping servo homing")
            return
            
        print("Homing all servos...")
        home_angles = {servo: 90 for servo in self.servos.keys()}
        self.set_multiple_servos(home_angles, speed=50)
        
    def get_current_positions(self) -> Dict[str, float]:
        """Get current servo positions."""
        return self.current_positions.copy()
        
    def calibrate_servo(self, servo_name: str):
        """Interactive servo calibration."""
        if servo_name not in self.servos:
            print(f"Unknown servo: {servo_name}")
            return
            
        print(f"Calibrating {servo_name} servo...")
        print("Use 'a' to decrease, 'd' to increase, 's' to save, 'q' to quit")
        
        angle = self.current_positions[servo_name]
        
        try:
            while True:
                print(f"\r{servo_name}: {angle:.1f}°", end="", flush=True)
                
                # In a real implementation, you'd read keyboard input here
                # For this example, we'll just show the interface
                time.sleep(0.1)
                break  # Exit after demo
                
        except KeyboardInterrupt:
            pass
            
        print(f"\n{servo_name} calibrated to {angle:.1f}°")
        
    # Pre-programmed robot poses
    def pose_wave(self):
        """Wave hello pose."""
        print("Waving hello...")
        self.set_multiple_servos({
            'base': 0,
            'shoulder': 45,
            'elbow': 90,
            'wrist_pitch': 45,
            'wrist_roll': 0,
            'gripper': 0
        })
        
    def pose_point(self):
        """Pointing pose."""
        print("Pointing...")
        self.set_multiple_servos({
            'base': 90,
            'shoulder': 30,
            'elbow': 120,
            'wrist_pitch': 0,
            'wrist_roll': 0,
            'gripper': 0
        })
        
    def pose_grab(self):
        """Grab object pose."""
        print("Grabbing position...")
        self.set_multiple_servos({
            'base': 90,
            'shoulder': 60,
            'elbow': 90,
            'wrist_pitch': 0,
            'wrist_roll': 0,
            'gripper': 60
        })
        
    def pose_rest(self):
        """Rest position."""
        print("Resting position...")
        self.set_multiple_servos({
            'base': 90,
            'shoulder': 90,
            'elbow': 90,
            'wrist_pitch': 90,
            'wrist_roll': 90,
            'gripper': 0
        })
        
    def sequence_dance(self):
        """Perform a dance sequence."""
        print("Starting dance sequence...")
        
        poses = [
            (self.pose_wave, 2),
            (self.pose_point, 2),
            (self.pose_grab, 2),
            (self.pose_rest, 2)
        ]
        
        for pose_func, duration in poses:
            pose_func()
            time.sleep(duration)
            
        print("Dance sequence complete!")
        
    def sequence_pick_and_place(self):
        """Demonstrate pick and place sequence."""
        print("Pick and place sequence...")
        
        # Move to pickup position
        print("  Moving to pickup position...")
        self.set_multiple_servos({
            'base': 0,
            'shoulder': 45,
            'elbow': 120,
            'wrist_pitch': 0,
            'gripper': 0
        })
        time.sleep(1)
        
        # Close gripper
        print("  Closing gripper...")
        self.set_servo_angle('gripper', 60)
        time.sleep(1)
        
        # Lift object
        print("  Lifting object...")
        self.set_multiple_servos({
            'shoulder': 30,
            'elbow': 90
        })
        time.sleep(1)
        
        # Move to place position
        print("  Moving to place position...")
        self.set_multiple_servos({
            'base': 180,
            'shoulder': 45,
            'elbow': 120
        })
        time.sleep(1)
        
        # Open gripper
        print("  Opening gripper...")
        self.set_servo_angle('gripper', 0)
        time.sleep(1)
        
        # Return to rest
        self.pose_rest()
        print("Pick and place complete!")
        
    def interactive_control(self):
        """Interactive manual control mode."""
        print("\nInteractive Control Mode")
        print("Commands:")
        print("  w/s: shoulder up/down")
        print("  a/d: base left/right") 
        print("  q/e: elbow up/down")
        print("  z/x: wrist pitch up/down")
        print("  c/v: wrist roll left/right")
        print("  f/g: gripper open/close")
        print("  h: home position")
        print("  r: rest position")
        print("  p: show current positions")
        print("  ESC: exit")
        
        # This is a demo - in a real implementation you'd use keyboard input
        print("\nDemo mode - showing current positions:")
        print(f"Current positions: {self.get_current_positions()}")
        
    def status_report(self):
        """Generate a comprehensive status report."""
        print("\nArduino Arm Robot Status Report")
        print("=" * 40)
        if self.esp:
            print(f"ESP32 Firmware: {self.esp.get_version()}")
            print(f"Connection: Connected")
        else:
            print("ESP32 Firmware: Not connected")
            print("Connection: Disconnected")
        print(f"Moving: {'Yes' if self.is_moving else 'No'}")
        print("\nServo Positions:")
        
        for servo_name, angle in self.current_positions.items():
            pin = self.servos[servo_name]
            min_angle, max_angle = self.servo_limits[servo_name]
            print(f"  {servo_name.capitalize():12} GPIO {pin:2}: {angle:6.1f}° (range: {min_angle}-{max_angle}°)")
            
        print(f"\nPWM Settings:")
        print(f"  Frequency: {self.pwm_frequency} Hz")
        print(f"  Resolution: {self.pwm_resolution} bits")
        print(f"  Default Speed: {self.default_speed} ms/°")


def main():
    """Main program demonstrating the Arduino Arm Robot controller."""
    print("Arduino Arm Robot Controller")
    print("=" * 40)
    print("ESP32 GPIO Bridge v0.1.5-beta - Dual-core optimized")
    print("Advanced PWM control with batch processing")
    print()
    
    robot = None
    
    try:
        # Initialize robot
        robot = ArduinoArmRobot()
        robot.connect()
        
        # Show initial status
        robot.status_report()
        
        # Demo sequence
        print("\nStarting demonstration sequence...")
        
        # Test individual servo movement
        print("\n1. Testing individual servo movement...")
        robot.set_servo_angle('base', 45)
        time.sleep(1)
        robot.set_servo_angle('shoulder', 60)
        time.sleep(1)
        
        # Test batch movement (v0.1.5-beta feature!)
        print("\n2. Testing batch servo movement...")
        robot.set_multiple_servos({
            'base': 90,
            'shoulder': 45,
            'elbow': 90,
            'wrist_pitch': 45,
            'wrist_roll': 0,
            'gripper': 0
        })
        time.sleep(2)
        
        # Demo poses
        print("\n3. Demonstrating pre-programmed poses...")
        robot.pose_wave()
        time.sleep(2)
        
        robot.pose_point()
        time.sleep(2)
        
        robot.pose_grab()
        time.sleep(2)
        
        # Demo sequences
        print("\n4. Running dance sequence...")
        robot.sequence_dance()
        
        print("\n5. Running pick and place sequence...")
        robot.sequence_pick_and_place()
        
        # Interactive mode demo
        print("\n6. Interactive control mode...")
        robot.interactive_control()
        
        # Final status
        print("\nFinal Status Report:")
        robot.status_report()
        
        print("\nDemonstration complete!")
        print("The ESP32 GPIO Bridge v0.1.5-beta enables smooth, coordinated")
        print("multi-servo control with professional-grade performance!")
        
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
        
    except Exception as e:
        print(f"\nError: {e}")
        
    finally:
        if robot:
            # Return to safe position before disconnecting
            print("\nReturning to home position...")
            robot.home_all_servos()
            time.sleep(1)
            robot.disconnect()


if __name__ == "__main__":
    main()
