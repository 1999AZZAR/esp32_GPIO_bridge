#!/usr/bin/env python3
"""
ESP32 GPIO Bridge - Multi-Sensor Dashboard Example

This example demonstrates real-time monitoring of multiple sensors
with data logging, statistics, and alert generation.

Hardware Setup:
- Temperature sensor (analog): GPIO 34
- Light sensor (analog): GPIO 35
- Moisture sensor (analog): GPIO 36
- Motion sensor (digital): GPIO 39
- Status LED: GPIO 2
- Alarm buzzer: GPIO 4

Author: ESP32 GPIO Bridge Project
Version: 0.1.3-beta
"""

import time
import statistics
from collections import deque
from dataclasses import dataclass, field
from typing import List, Dict
from esp32_gpio_bridge import ESP32GPIO, find_esp32_port, select_port


@dataclass
class SensorReading:
    """Data class for sensor readings."""
    timestamp: float
    temperature: float
    light: float
    moisture: float
    motion: bool


@dataclass
class SensorStats:
    """Statistics for sensor data."""
    readings: deque = field(default_factory=lambda: deque(maxlen=100))
    
    def add_reading(self, value: float) -> None:
        """Add a reading to the history."""
        self.readings.append(value)
    
    def get_average(self) -> float:
        """Get average of recent readings."""
        return statistics.mean(self.readings) if self.readings else 0.0
    
    def get_min(self) -> float:
        """Get minimum of recent readings."""
        return min(self.readings) if self.readings else 0.0
    
    def get_max(self) -> float:
        """Get maximum of recent readings."""
        return max(self.readings) if self.readings else 0.0
    
    def get_std_dev(self) -> float:
        """Get standard deviation of recent readings."""
        if len(self.readings) < 2:
            return 0.0
        return statistics.stdev(self.readings)


class SensorDashboard:
    """Multi-sensor monitoring dashboard."""
    
    def __init__(self, esp: ESP32GPIO):
        """Initialize dashboard."""
        self.esp = esp
        
        # Pin configuration
        self.pins = {
            'temperature': 34,
            'light': 35,
            'moisture': 36,
            'motion': 39,
            'led': 2,
            'buzzer': 4
        }
        
        # Statistics trackers
        self.temp_stats = SensorStats()
        self.light_stats = SensorStats()
        self.moisture_stats = SensorStats()
        
        # Alert thresholds
        self.thresholds = {
            'temp_high': 30.0,  # °C
            'temp_low': 15.0,   # °C
            'light_low': 20.0,  # %
            'moisture_low': 30.0  # %
        }
        
        # State
        self.motion_detected_count = 0
        self.alert_active = False
        
        # Setup GPIO
        self._setup_pins()
    
    def _setup_pins(self) -> None:
        """Configure GPIO pins."""
        print("Configuring GPIO pins...")
        
        # Output pins
        self.esp.set_pin_mode(self.pins['led'], "OUT")
        self.esp.set_pin_mode(self.pins['buzzer'], "OUT")
        
        # Input pins (motion sensor with pullup)
        self.esp.set_pin_mode(self.pins['motion'], "IN_PULLUP")
        
        # Turn off outputs initially
        self.esp.digital_write(self.pins['led'], 0)
        self.esp.digital_write(self.pins['buzzer'], 0)
        
        print("✓ GPIO configured")
    
    def read_temperature(self) -> float:
        """
        Read temperature from analog sensor.
        Assumes TMP36 or similar: 10mV/°C, 500mV offset
        """
        raw = self.esp.analog_read(self.pins['temperature'])
        voltage = (raw / 4095.0) * 3.3
        
        # TMP36: Voltage = (Temp × 0.01) + 0.5
        temperature = (voltage - 0.5) * 100.0
        
        self.temp_stats.add_reading(temperature)
        return temperature
    
    def read_light(self) -> float:
        """Read light sensor (0-100%)."""
        raw = self.esp.analog_read(self.pins['light'])
        percentage = (raw / 4095.0) * 100.0
        
        self.light_stats.add_reading(percentage)
        return percentage
    
    def read_moisture(self) -> float:
        """Read soil moisture sensor (0-100%)."""
        raw = self.esp.analog_read(self.pins['moisture'])
        # Invert for moisture sensors (higher voltage = drier soil)
        percentage = (1.0 - (raw / 4095.0)) * 100.0
        
        self.moisture_stats.add_reading(percentage)
        return percentage
    
    def read_motion(self) -> bool:
        """Read motion sensor (PIR)."""
        state = self.esp.digital_read(self.pins['motion'])
        
        if state == 0:  # Motion detected (pulled low)
            self.motion_detected_count += 1
            return True
        return False
    
    def read_all_sensors(self) -> SensorReading:
        """Read all sensors and return readings."""
        return SensorReading(
            timestamp=time.time(),
            temperature=self.read_temperature(),
            light=self.read_light(),
            moisture=self.read_moisture(),
            motion=self.read_motion()
        )
    
    def check_alerts(self, reading: SensorReading) -> List[str]:
        """Check sensor readings against thresholds."""
        alerts = []
        
        # Temperature alerts
        if reading.temperature > self.thresholds['temp_high']:
            alerts.append(f"⚠ HIGH TEMPERATURE: {reading.temperature:.1f}°C")
        elif reading.temperature < self.thresholds['temp_low']:
            alerts.append(f"⚠ LOW TEMPERATURE: {reading.temperature:.1f}°C")
        
        # Light alert
        if reading.light < self.thresholds['light_low']:
            alerts.append(f"⚠ LOW LIGHT: {reading.light:.1f}%")
        
        # Moisture alert
        if reading.moisture < self.thresholds['moisture_low']:
            alerts.append(f"⚠ LOW MOISTURE: {reading.moisture:.1f}%")
        
        # Motion alert
        if reading.motion:
            alerts.append("⚠ MOTION DETECTED")
        
        return alerts
    
    def trigger_alert(self, active: bool) -> None:
        """Activate or deactivate alert indicators."""
        if active and not self.alert_active:
            # Turn on LED and buzzer
            self.esp.digital_write(self.pins['led'], 1)
            self.esp.digital_write(self.pins['buzzer'], 1)
            self.alert_active = True
        elif not active and self.alert_active:
            # Turn off LED and buzzer
            self.esp.digital_write(self.pins['led'], 0)
            self.esp.digital_write(self.pins['buzzer'], 0)
            self.alert_active = False
    
    def display_reading(self, reading: SensorReading, alerts: List[str]) -> None:
        """Display sensor reading in formatted output."""
        # Clear screen (simple version)
        print("\n" * 2)
        print("=" * 70)
        print(f"{'SENSOR DASHBOARD':^70}")
        print("=" * 70)
        print(f"Time: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(reading.timestamp))}")
        print("-" * 70)
        
        # Temperature
        temp_avg = self.temp_stats.get_average()
        temp_std = self.temp_stats.get_std_dev()
        print(f"🌡️  Temperature: {reading.temperature:6.1f}°C  " +
              f"(Avg: {temp_avg:5.1f}°C, StdDev: {temp_std:4.1f}°C)")
        
        # Light
        light_avg = self.light_stats.get_average()
        print(f"💡 Light:       {reading.light:6.1f}%   " +
              f"(Avg: {light_avg:5.1f}%, Range: {self.light_stats.get_min():.0f}-{self.light_stats.get_max():.0f}%)")
        
        # Moisture
        moisture_avg = self.moisture_stats.get_average()
        print(f"💧 Moisture:    {reading.moisture:6.1f}%   " +
              f"(Avg: {moisture_avg:5.1f}%, Range: {self.moisture_stats.get_min():.0f}-{self.moisture_stats.get_max():.0f}%)")
        
        # Motion
        motion_str = "DETECTED" if reading.motion else "None"
        print(f"👁️  Motion:      {motion_str:8s}   " +
              f"(Total detections: {self.motion_detected_count})")
        
        print("-" * 70)
        
        # Alerts
        if alerts:
            print("🚨 ALERTS:")
            for alert in alerts:
                print(f"   {alert}")
            print("-" * 70)
        else:
            print("✓ All sensors within normal range")
            print("-" * 70)
        
        # Statistics summary
        if len(self.temp_stats.readings) >= 10:
            print(f"📊 Statistics (last {len(self.temp_stats.readings)} readings):")
            print(f"   Temperature: {self.temp_stats.get_min():.1f}°C - {self.temp_stats.get_max():.1f}°C")
            print(f"   Light:       {self.light_stats.get_min():.1f}% - {self.light_stats.get_max():.1f}%")
            print(f"   Moisture:    {self.moisture_stats.get_min():.1f}% - {self.moisture_stats.get_max():.1f}%")
        
        print("=" * 70)
    
    def run(self, duration: float = 60.0, interval: float = 2.0) -> None:
        """
        Run dashboard monitoring loop.
        
        Args:
            duration: Total monitoring duration in seconds
            interval: Time between sensor reads in seconds
        """
        print(f"\n{'=' * 70}")
        print(f"Starting Sensor Dashboard")
        print(f"{'=' * 70}")
        print(f"Duration: {duration}s, Update interval: {interval}s")
        print(f"Press Ctrl+C to stop early")
        print(f"{'=' * 70}\n")
        
        start_time = time.time()
        readings_count = 0
        
        try:
            while time.time() - start_time < duration:
                # Read all sensors
                reading = self.read_all_sensors()
                readings_count += 1
                
                # Check for alerts
                alerts = self.check_alerts(reading)
                
                # Trigger visual/audio alert
                self.trigger_alert(len(alerts) > 0)
                
                # Display reading
                self.display_reading(reading, alerts)
                
                # Wait for next reading
                time.sleep(interval)
        
        except KeyboardInterrupt:
            print("\n\nMonitoring stopped by user")
        
        finally:
            # Turn off alert indicators
            self.trigger_alert(False)
            
            elapsed = time.time() - start_time
            print(f"\n{'=' * 70}")
            print(f"Monitoring completed")
            print(f"{'=' * 70}")
            print(f"Total time:     {elapsed:.1f}s")
            print(f"Total readings: {readings_count}")
            print(f"Avg rate:       {readings_count/elapsed:.2f} readings/s")
            print(f"{'=' * 70}")


def main():
    """Main program."""
    print("=" * 70)
    print("ESP32 GPIO Bridge - Multi-Sensor Dashboard Example")
    print("=" * 70)
    
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
            
            # Create dashboard
            dashboard = SensorDashboard(esp)
            
            # Run monitoring (60 seconds, 2 second interval)
            dashboard.run(duration=60.0, interval=2.0)
            
            print("\n" + "=" * 70)
            print("MULTI-SENSOR DASHBOARD DEMO COMPLETED SUCCESSFULLY")
            print("=" * 70)
            print("\nFeatures Demonstrated:")
            print("  ✓ Real-time sensor monitoring (4 sensors)")
            print("  ✓ Statistical analysis (average, min, max, std dev)")
            print("  ✓ Alert generation and thresholds")
            print("  ✓ Visual/audio alert indicators")
            print("  ✓ Data logging and history tracking")
            print("  ✓ Formatted dashboard display")
    
    except KeyboardInterrupt:
        print("\n\nDemo interrupted by user")
    except Exception as e:
        print(f"\nError during dashboard demo: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()

