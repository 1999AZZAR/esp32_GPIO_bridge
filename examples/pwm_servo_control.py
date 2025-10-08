#!/usr/bin/env python3
"""
ESP32 GPIO Bridge - PWM Servo Control Example

This example demonstrates controlling servo motors using PWM.
Servo motors typically use 50Hz PWM with pulse widths between 1-2ms.

Hardware Setup:
- Servo 1: Signal pin to GPIO 18, VCC to 5V, GND to GND
- Servo 2: Signal pin to GPIO 19, VCC to 5V, GND to GND
- Optional: External 5V power supply for servos

Author: ESP32 GPIO Bridge Project
Version: 0.1.3-beta
"""

import time
import math
from esp32_gpio_bridge import ESP32GPIO, find_esp32_port, select_port

# Servo configuration
SERVO_PINS = {
    'servo1': 18,  # First servo
    'servo2': 19   # Second servo (optional)
}

# Servo PWM parameters
SERVO_FREQ = 50  # 50Hz for standard servos
SERVO_MIN_DUTY = 26   # ~1ms pulse (0 degrees)
SERVO_MAX_DUTY = 128  # ~2ms pulse (180 degrees)
SERVO_MID_DUTY = 77   # ~1.5ms pulse (90 degrees)


def angle_to_duty(angle: float) -> int:
    """
    Convert servo angle (0-180) to PWM duty cycle value.
    
    Args:
        angle: Servo angle in degrees (0-180)
    
    Returns:
        PWM duty cycle value
    """
    if angle < 0:
        angle = 0
    elif angle > 180:
        angle = 180
    
    # Linear interpolation between min and max duty
    duty = int(SERVO_MIN_DUTY + (angle / 180.0) * (SERVO_MAX_DUTY - SERVO_MIN_DUTY))
    return duty


def smooth_move(esp: ESP32GPIO, pin: int, start_angle: float, end_angle: float, 
                duration: float = 1.0, steps: int = 20) -> None:
    """
    Smoothly move servo from start to end angle.
    
    Args:
        esp: ESP32GPIO instance
        pin: Servo pin number
        start_angle: Starting angle (0-180)
        end_angle: Target angle (0-180)
        duration: Time to complete movement in seconds
        steps: Number of intermediate steps
    """
    delay = duration / steps
    
    for i in range(steps + 1):
        # Calculate intermediate angle
        progress = i / steps
        angle = start_angle + (end_angle - start_angle) * progress
        
        # Set servo position
        duty = angle_to_duty(angle)
        esp.pwm_write(pin, duty)
        
        time.sleep(delay)


def sweep_demo(esp: ESP32GPIO, pin: int, sweeps: int = 3) -> None:
    """
    Demonstrate servo sweep from 0 to 180 degrees.
    
    Args:
        esp: ESP32GPIO instance
        pin: Servo pin number
        sweeps: Number of complete sweeps
    """
    print(f"\n{'=' * 50}")
    print(f"Servo Sweep Demo - Pin {pin}")
    print(f"{'=' * 50}")
    
    for sweep in range(sweeps):
        print(f"Sweep {sweep + 1}/{sweeps}: 0° → 180°")
        smooth_move(esp, pin, 0, 180, duration=1.5)
        
        time.sleep(0.5)
        
        print(f"Sweep {sweep + 1}/{sweeps}: 180° → 0°")
        smooth_move(esp, pin, 180, 0, duration=1.5)
        
        time.sleep(0.5)


def preset_positions_demo(esp: ESP32GPIO, pin: int) -> None:
    """
    Move servo to preset positions.
    
    Args:
        esp: ESP32GPIO instance
        pin: Servo pin number
    """
    print(f"\n{'=' * 50}")
    print(f"Preset Positions Demo - Pin {pin}")
    print(f"{'=' * 50}")
    
    positions = [
        (0, "0° (Minimum)"),
        (45, "45° (Mid-Low)"),
        (90, "90° (Center)"),
        (135, "135° (Mid-High)"),
        (180, "180° (Maximum)"),
        (90, "90° (Center)")
    ]
    
    current_angle = 90
    
    for angle, description in positions:
        print(f"Moving to {description}")
        smooth_move(esp, pin, current_angle, angle, duration=0.8)
        current_angle = angle
        time.sleep(1.0)


def wave_pattern_demo(esp: ESP32GPIO, pin: int, cycles: int = 2) -> None:
    """
    Create smooth wave pattern with servo.
    
    Args:
        esp: ESP32GPIO instance
        pin: Servo pin number
        cycles: Number of wave cycles
    """
    print(f"\n{'=' * 50}")
    print(f"Wave Pattern Demo - Pin {pin}")
    print(f"{'=' * 50}")
    
    steps = 50
    duration = 3.0
    
    for cycle in range(cycles):
        print(f"Wave cycle {cycle + 1}/{cycles}")
        
        for i in range(steps + 1):
            # Sine wave pattern
            t = (i / steps) * 2 * math.pi
            angle = 90 + 45 * math.sin(t)  # Oscillate between 45° and 135°
            
            duty = angle_to_duty(angle)
            esp.pwm_write(pin, duty)
            
            time.sleep(duration / steps)


def synchronized_demo(esp: ESP32GPIO, pin1: int, pin2: int) -> None:
    """
    Demonstrate synchronized movement of two servos.
    
    Args:
        esp: ESP32GPIO instance
        pin1: First servo pin
        pin2: Second servo pin
    """
    print(f"\n{'=' * 50}")
    print(f"Synchronized Servos Demo")
    print(f"Servo 1: Pin {pin1}, Servo 2: Pin {pin2}")
    print(f"{'=' * 50}")
    
    patterns = [
        ("Mirror movement", 
         [(0, 180), (90, 90), (180, 0), (90, 90)]),
        ("Parallel movement", 
         [(0, 0), (90, 90), (180, 180), (90, 90)]),
        ("Opposite movement", 
         [(0, 0), (180, 180), (0, 0)])
    ]
    
    for pattern_name, positions in patterns:
        print(f"\n{pattern_name}:")
        
        for angle1, angle2 in positions:
            print(f"  Servo1: {angle1}°, Servo2: {angle2}°")
            duty1 = angle_to_duty(angle1)
            duty2 = angle_to_duty(angle2)
            
            # Set both servos simultaneously
            esp.pwm_write(pin1, duty1)
            esp.pwm_write(pin2, duty2)
            
            time.sleep(1.5)


def main():
    """Main program."""
    print("=" * 60)
    print("ESP32 GPIO Bridge - PWM Servo Control Example")
    print("=" * 60)
    
    # Find ESP32
    print("\nSearching for ESP32 GPIO Bridge...")
    port = find_esp32_port()
    
    if not port:
        print("ESP32 not auto-detected. Please select manually.")
        port = select_port()
    
    if not port:
        print("No port selected. Exiting.")
        return
    
    print(f"✓ Found ESP32 GPIO Bridge on port: {port}")
    
    try:
        # Connect to ESP32
        with ESP32GPIO(port) as esp:
            print(f"Connected to ESP32. Firmware version: {esp.get_version()}")
            print("\nNote: WiFi and Bluetooth are disabled for maximum GPIO performance")
            
            # Initialize servos with PWM
            print(f"\nInitializing servo on pin {SERVO_PINS['servo1']}...")
            channel1 = esp.pwm_init(SERVO_PINS['servo1'], frequency=SERVO_FREQ, resolution=8)
            print(f"✓ Servo 1 initialized (Channel {channel1})")
            
            # Center servo 1
            duty = angle_to_duty(90)
            esp.pwm_write(SERVO_PINS['servo1'], duty)
            print("✓ Servo 1 centered at 90°")
            time.sleep(1)
            
            # Single servo demos
            sweep_demo(esp, SERVO_PINS['servo1'], sweeps=2)
            time.sleep(1)
            
            preset_positions_demo(esp, SERVO_PINS['servo1'])
            time.sleep(1)
            
            wave_pattern_demo(esp, SERVO_PINS['servo1'], cycles=2)
            time.sleep(1)
            
            # Two servo demo (if available)
            print(f"\nDo you have a second servo on pin {SERVO_PINS['servo2']}? (y/n): ", end='')
            try:
                response = input().strip().lower()
                if response == 'y':
                    print(f"\nInitializing servo on pin {SERVO_PINS['servo2']}...")
                    channel2 = esp.pwm_init(SERVO_PINS['servo2'], frequency=SERVO_FREQ, resolution=8)
                    print(f"✓ Servo 2 initialized (Channel {channel2})")
                    
                    # Center servo 2
                    esp.pwm_write(SERVO_PINS['servo2'], angle_to_duty(90))
                    print("✓ Servo 2 centered at 90°")
                    time.sleep(1)
                    
                    # Synchronized demo
                    synchronized_demo(esp, SERVO_PINS['servo1'], SERVO_PINS['servo2'])
                    
                    # Stop servo 2
                    print(f"\nStopping servo on pin {SERVO_PINS['servo2']}...")
                    esp.pwm_stop(SERVO_PINS['servo2'])
            except EOFError:
                print("Skipping second servo demo")
            
            # Return servo 1 to center
            print(f"\nReturning servo to center position...")
            smooth_move(esp, SERVO_PINS['servo1'], 
                       esp.pwm_channels.get(SERVO_PINS['servo1'], 90), 
                       90, duration=1.0)
            time.sleep(0.5)
            
            # Stop PWM
            print(f"Stopping servo on pin {SERVO_PINS['servo1']}...")
            esp.pwm_stop(SERVO_PINS['servo1'])
            
            print("\n" + "=" * 60)
            print("SERVO CONTROL DEMO COMPLETED SUCCESSFULLY")
            print("=" * 60)
    
    except KeyboardInterrupt:
        print("\n\nDemo interrupted by user")
    except Exception as e:
        print(f"\nError during servo control demo: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()

