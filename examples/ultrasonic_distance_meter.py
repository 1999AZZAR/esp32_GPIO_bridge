#!/usr/bin/env python3
"""
ESP32 GPIO Bridge - Ultrasonic Distance Meter Example

This example demonstrates using an ultrasonic sensor (HC-SR04) for distance
measurement with visual feedback and data logging.

Hardware Setup:
- HC-SR04 Ultrasonic Sensor:
  - VCC to 5V
  - GND to GND
  - TRIG to GPIO 5
  - ECHO to GPIO 18 (through voltage divider: 1kΩ + 2kΩ for 3.3V logic)
- LEDs for visual feedback:
  - Green LED on GPIO 2 (< 10cm - too close)
  - Yellow LED on GPIO 4 (10-30cm - optimal range)
  - Red LED on GPIO 12 (> 30cm - far)
- Buzzer on GPIO 13 (for proximity alarm)

Author: ESP32 GPIO Bridge Project
Version: 0.1.3-beta
"""

import time
import statistics
from dataclasses import dataclass
from typing import List
from esp32_gpio_bridge import ESP32GPIO, find_esp32_port, select_port


@dataclass
class DistanceMeasurement:
    """Data class for distance measurements."""
    timestamp: float
    distance_cm: float
    valid: bool


class UltrasonicSensor:
    """HC-SR04 Ultrasonic sensor controller."""
    
    def __init__(self, esp: ESP32GPIO, trig_pin: int, echo_pin: int):
        """
        Initialize ultrasonic sensor.
        
        Args:
            esp: ESP32GPIO instance
            trig_pin: Trigger pin number
            echo_pin: Echo pin number
        """
        self.esp = esp
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        
        # Configure pins
        self.esp.set_pin_mode(trig_pin, "OUT")
        self.esp.set_pin_mode(echo_pin, "IN")
        self.esp.digital_write(trig_pin, 0)
        
        # Measurement history
        self.measurements: List[float] = []
        self.max_history = 10
    
    def measure_distance(self) -> DistanceMeasurement:
        """
        Measure distance using ultrasonic sensor.
        
        Returns:
            DistanceMeasurement object with timestamp, distance, and validity
        
        Note:
            This is a simplified software-based implementation.
            For best accuracy, use hardware timer-based pulse measurement.
        """
        # Send 10μs trigger pulse
        self.esp.digital_write(self.trig_pin, 1)
        time.sleep(0.00001)  # 10 microseconds
        self.esp.digital_write(self.trig_pin, 0)
        
        # Wait for echo pulse and measure duration
        # Note: This software timing is approximate
        timeout = 0.1  # 100ms timeout
        start_time = time.time()
        pulse_start = 0
        pulse_end = 0
        
        # Wait for echo high
        while self.esp.digital_read(self.echo_pin) == 0:
            pulse_start = time.time()
            if (pulse_start - start_time) > timeout:
                return DistanceMeasurement(time.time(), 0, False)
        
        # Wait for echo low
        while self.esp.digital_read(self.echo_pin) == 1:
            pulse_end = time.time()
            if (pulse_end - start_time) > timeout:
                return DistanceMeasurement(time.time(), 0, False)
        
        # Calculate distance
        # Speed of sound = 343 m/s = 0.0343 cm/μs
        # Distance = (Time × Speed) / 2
        pulse_duration = pulse_end - pulse_start
        distance_cm = (pulse_duration * 34300) / 2  # cm
        
        # Validate measurement (HC-SR04 range: 2cm - 400cm)
        valid = 2 <= distance_cm <= 400
        
        if valid:
            self.measurements.append(distance_cm)
            if len(self.measurements) > self.max_history:
                self.measurements.pop(0)
        
        return DistanceMeasurement(time.time(), distance_cm, valid)
    
    def get_average_distance(self) -> float:
        """Get average of recent measurements."""
        if not self.measurements:
            return 0.0
        return statistics.mean(self.measurements)
    
    def get_std_deviation(self) -> float:
        """Get standard deviation of recent measurements."""
        if len(self.measurements) < 2:
            return 0.0
        return statistics.stdev(self.measurements)


def visual_feedback(esp: ESP32GPIO, distance_cm: float, 
                    green_pin: int, yellow_pin: int, red_pin: int, buzzer_pin: int) -> None:
    """
    Provide visual and audio feedback based on distance.
    
    Args:
        esp: ESP32GPIO instance
        distance_cm: Measured distance in cm
        green_pin: Green LED pin (close range)
        yellow_pin: Yellow LED pin (medium range)
        red_pin: Red LED pin (far range)
        buzzer_pin: Buzzer pin for proximity alarm
    """
    # Turn off all LEDs first
    esp.batch_digital_write({green_pin: 0, yellow_pin: 0, red_pin: 0, buzzer_pin: 0})
    
    if distance_cm < 10:
        # Too close - green LED + buzzer
        esp.batch_digital_write({green_pin: 1, buzzer_pin: 1})
    elif distance_cm < 30:
        # Optimal range - yellow LED
        esp.digital_write(yellow_pin, 1)
    else:
        # Far - red LED
        esp.digital_write(red_pin, 1)


def main():
    """Main program."""
    print("=" * 70)
    print("ESP32 GPIO Bridge - Ultrasonic Distance Meter Example")
    print("=" * 70)
    
    # Pin configuration
    TRIG_PIN = 5
    ECHO_PIN = 18
    GREEN_LED = 2
    YELLOW_LED = 4
    RED_LED = 12
    BUZZER = 13
    
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
            
            # Setup feedback LEDs and buzzer
            print(f"\nConfiguring feedback indicators...")
            esp.set_pin_mode(GREEN_LED, "OUT")
            esp.set_pin_mode(YELLOW_LED, "OUT")
            esp.set_pin_mode(RED_LED, "OUT")
            esp.set_pin_mode(BUZZER, "OUT")
            esp.batch_digital_write({GREEN_LED: 0, YELLOW_LED: 0, RED_LED: 0, BUZZER: 0})
            print("✓ Indicators configured")
            
            # Initialize ultrasonic sensor
            print(f"\nInitializing HC-SR04 on TRIG={TRIG_PIN}, ECHO={ECHO_PIN}...")
            sensor = UltrasonicSensor(esp, TRIG_PIN, ECHO_PIN)
            print("✓ Sensor initialized")
            
            # Distance measurement loop
            print(f"\n{'=' * 70}")
            print("Distance Measurement (Press Ctrl+C to stop)")
            print(f"{'=' * 70}")
            print("Range indicators:")
            print("  🟢 Green LED + Buzzer: < 10cm (too close)")
            print("  🟡 Yellow LED: 10-30cm (optimal)")
            print("  🔴 Red LED: > 30cm (far)")
            print(f"{'=' * 70}\n")
            
            measurement_count = 0
            
            try:
                while True:
                    # Measure distance
                    result = sensor.measure_distance()
                    measurement_count += 1
                    
                    if result.valid:
                        # Calculate statistics
                        avg_distance = sensor.get_average_distance()
                        std_dev = sensor.get_std_deviation()
                        
                        # Display measurement
                        status = "CLOSE" if result.distance_cm < 10 else \
                                "OPTIMAL" if result.distance_cm < 30 else "FAR"
                        
                        print(f"#{measurement_count:4d} | "
                              f"Distance: {result.distance_cm:6.1f}cm | "
                              f"Avg: {avg_distance:6.1f}cm | "
                              f"StdDev: {std_dev:5.2f} | "
                              f"Status: {status:7s}")
                        
                        # Visual and audio feedback
                        visual_feedback(esp, result.distance_cm, 
                                      GREEN_LED, YELLOW_LED, RED_LED, BUZZER)
                    else:
                        print(f"#{measurement_count:4d} | Measurement INVALID (out of range or timeout)")
                    
                    time.sleep(0.2)  # 5 measurements per second
            
            except KeyboardInterrupt:
                print("\n\nMeasurement stopped by user")
            
            # Turn off all indicators
            print("\nTurning off indicators...")
            esp.batch_digital_write({GREEN_LED: 0, YELLOW_LED: 0, RED_LED: 0, BUZZER: 0})
            
            # Display statistics
            if sensor.measurements:
                print(f"\n{'=' * 70}")
                print("Measurement Statistics")
                print(f"{'=' * 70}")
                print(f"Total measurements: {measurement_count}")
                print(f"Valid measurements: {len(sensor.measurements)}")
                print(f"Average distance:   {sensor.get_average_distance():.1f}cm")
                print(f"Min distance:       {min(sensor.measurements):.1f}cm")
                print(f"Max distance:       {max(sensor.measurements):.1f}cm")
                print(f"Std deviation:      {sensor.get_std_deviation():.2f}cm")
            
            print("\n" + "=" * 70)
            print("ULTRASONIC DISTANCE METER DEMO COMPLETED")
            print("=" * 70)
            print("\nNote: For best accuracy, use hardware timer-based pulse measurement")
            print("or dedicated ultrasonic sensor library.")
    
    except KeyboardInterrupt:
        print("\n\nDemo interrupted by user")
    except Exception as e:
        print(f"\nError during distance meter demo: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()

