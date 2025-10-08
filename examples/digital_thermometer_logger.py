#!/usr/bin/env python3
"""
ESP32 GPIO Bridge - Digital Thermometer with Data Logger

This example demonstrates building a temperature monitoring system with:
- Real-time temperature display
- Configurable alert thresholds
- CSV data logging
- Moving average smoothing
- Visual indicators (LEDs)
- Export functionality

Hardware Setup:
- TMP36 temperature sensor on GPIO 34 (analog input)
  - VCC to 3.3V
  - OUT to GPIO 34
  - GND to GND
- Status LEDs:
  - Blue LED on GPIO 2 (normal temperature)
  - Red LED on GPIO 4 (high temperature alert)
- Buzzer on GPIO 5 (for temperature alerts)

Author: ESP32 GPIO Bridge Project
Version: 0.1.3-beta
"""

import time
import csv
import statistics
from dataclasses import dataclass, asdict
from typing import List, Optional
from datetime import datetime
from esp32_gpio_bridge import ESP32GPIO, find_esp32_port, select_port


@dataclass
class TemperatureReading:
    """Data class for temperature readings."""
    timestamp: str
    temperature_c: float
    temperature_f: float
    voltage: float
    alert: bool


class TMP36Sensor:
    """TMP36 temperature sensor driver."""
    
    def __init__(self, esp: ESP32GPIO, pin: int):
        """
        Initialize TMP36 sensor.
        
        Args:
            esp: ESP32GPIO instance
            pin: Analog input pin number
        """
        self.esp = esp
        self.pin = pin
        self.reference_voltage = 3.3  # ESP32 ADC reference
    
    def read_temperature(self) -> tuple[float, float]:
        """
        Read temperature from TMP36 sensor.
        
        Returns:
            Tuple of (temperature_celsius, voltage)
        
        TMP36 characteristics:
        - Output voltage = (temperature_c * 10mV) + 500mV
        - Temperature range: -40°C to +125°C
        - Scale factor: 10mV/°C
        - Offset: 500mV at 0°C
        """
        # Read ADC value
        raw_value = self.esp.analog_read(self.pin)
        
        # Convert to voltage (12-bit ADC: 0-4095)
        voltage = (raw_value / 4095.0) * self.reference_voltage
        
        # Convert voltage to temperature
        # Voltage = (Temp × 0.01) + 0.5
        # Temp = (Voltage - 0.5) / 0.01
        temperature_c = (voltage - 0.5) * 100.0
        
        return temperature_c, voltage


class TemperatureLogger:
    """Temperature data logger with statistics and alerts."""
    
    def __init__(self, esp: ESP32GPIO, sensor: TMP36Sensor,
                 blue_led: int, red_led: int, buzzer: int):
        """
        Initialize temperature logger.
        
        Args:
            esp: ESP32GPIO instance
            sensor: TMP36Sensor instance
            blue_led: Blue LED pin (normal status)
            red_led: Red LED pin (alert status)
            buzzer: Buzzer pin (alert sound)
        """
        self.esp = esp
        self.sensor = sensor
        self.blue_led = blue_led
        self.red_led = red_led
        self.buzzer = buzzer
        
        # Configure indicator pins
        self.esp.set_pin_mode(blue_led, "OUT")
        self.esp.set_pin_mode(red_led, "OUT")
        self.esp.set_pin_mode(buzzer, "OUT")
        
        # Data storage
        self.readings: List[TemperatureReading] = []
        self.temperature_history: List[float] = []
        self.max_history = 20
        
        # Alert configuration
        self.temp_high_threshold = 28.0  # °C
        self.temp_low_threshold = 18.0   # °C
        self.alert_active = False
    
    def take_reading(self) -> TemperatureReading:
        """
        Take a temperature reading and log it.
        
        Returns:
            TemperatureReading object
        """
        # Read sensor
        temp_c, voltage = self.sensor.read_temperature()
        temp_f = (temp_c * 9/5) + 32  # Convert to Fahrenheit
        
        # Check for alerts
        alert = temp_c > self.temp_high_threshold or temp_c < self.temp_low_threshold
        
        # Create reading
        reading = TemperatureReading(
            timestamp=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            temperature_c=round(temp_c, 2),
            temperature_f=round(temp_f, 2),
            voltage=round(voltage, 3),
            alert=alert
        )
        
        # Store reading
        self.readings.append(reading)
        self.temperature_history.append(temp_c)
        
        # Limit history size
        if len(self.temperature_history) > self.max_history:
            self.temperature_history.pop(0)
        
        # Update indicators
        self.update_indicators(alert)
        
        return reading
    
    def update_indicators(self, alert: bool) -> None:
        """
        Update LED and buzzer based on alert status.
        
        Args:
            alert: True if temperature is out of range
        """
        if alert:
            # Alert: Red LED + buzzer
            self.esp.batch_digital_write({
                self.blue_led: 0,
                self.red_led: 1,
                self.buzzer: 1
            })
            self.alert_active = True
        else:
            # Normal: Blue LED only
            self.esp.batch_digital_write({
                self.blue_led: 1,
                self.red_led: 0,
                self.buzzer: 0
            })
            self.alert_active = False
    
    def get_average_temperature(self) -> float:
        """Get average temperature from history."""
        if not self.temperature_history:
            return 0.0
        return statistics.mean(self.temperature_history)
    
    def get_min_temperature(self) -> float:
        """Get minimum temperature from history."""
        if not self.temperature_history:
            return 0.0
        return min(self.temperature_history)
    
    def get_max_temperature(self) -> float:
        """Get maximum temperature from history."""
        if not self.temperature_history:
            return 0.0
        return max(self.temperature_history)
    
    def get_std_deviation(self) -> float:
        """Get standard deviation of temperature."""
        if len(self.temperature_history) < 2:
            return 0.0
        return statistics.stdev(self.temperature_history)
    
    def export_to_csv(self, filename: str) -> None:
        """
        Export all readings to CSV file.
        
        Args:
            filename: Output CSV filename
        """
        if not self.readings:
            print("No data to export")
            return
        
        with open(filename, 'w', newline='') as csvfile:
            fieldnames = ['timestamp', 'temperature_c', 'temperature_f', 'voltage', 'alert']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            
            writer.writeheader()
            for reading in self.readings:
                writer.writerow(asdict(reading))
        
        print(f"✓ Exported {len(self.readings)} readings to {filename}")


def display_reading(reading: TemperatureReading, logger: TemperatureLogger, count: int) -> None:
    """
    Display temperature reading with statistics.
    
    Args:
        reading: TemperatureReading object
        logger: TemperatureLogger instance
        count: Reading number
    """
    # Clear screen (simple version)
    print("\n" * 2)
    print("=" * 75)
    print(f"{'DIGITAL THERMOMETER - REAL-TIME MONITORING':^75}")
    print("=" * 75)
    print(f"Time: {reading.timestamp}")
    print("-" * 75)
    
    # Current temperature
    alert_indicator = "🔴 ALERT" if reading.alert else "🔵 NORMAL"
    print(f"Current Temperature: {reading.temperature_c:6.2f}°C / {reading.temperature_f:6.2f}°F  [{alert_indicator}]")
    print(f"Sensor Voltage:      {reading.voltage:6.3f}V")
    
    print("-" * 75)
    
    # Statistics (if enough readings)
    if len(logger.temperature_history) >= 2:
        avg_temp = logger.get_average_temperature()
        min_temp = logger.get_min_temperature()
        max_temp = logger.get_max_temperature()
        std_dev = logger.get_std_deviation()
        
        print(f"Statistics (last {len(logger.temperature_history)} readings):")
        print(f"  Average:    {avg_temp:6.2f}°C")
        print(f"  Min:        {min_temp:6.2f}°C")
        print(f"  Max:        {max_temp:6.2f}°C")
        print(f"  Std Dev:    {std_dev:6.2f}°C")
        print("-" * 75)
    
    # Thresholds
    print(f"Alert Thresholds:")
    print(f"  Low:  < {logger.temp_low_threshold:.1f}°C")
    print(f"  High: > {logger.temp_high_threshold:.1f}°C")
    
    print("-" * 75)
    print(f"Total Readings: {count} | Alerts: {sum(1 for r in logger.readings if r.alert)}")
    print("=" * 75)


def main():
    """Main program."""
    print("=" * 75)
    print("ESP32 GPIO Bridge - Digital Thermometer with Data Logger")
    print("=" * 75)
    
    # Pin configuration
    TEMP_SENSOR_PIN = 34
    BLUE_LED = 2
    RED_LED = 4
    BUZZER = 5
    
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
            
            # Initialize sensor and logger
            print(f"\nInitializing TMP36 sensor on GPIO {TEMP_SENSOR_PIN}...")
            sensor = TMP36Sensor(esp, TEMP_SENSOR_PIN)
            
            print("Initializing data logger...")
            logger = TemperatureLogger(esp, sensor, BLUE_LED, RED_LED, BUZZER)
            print("✓ System initialized")
            
            # Configuration
            print(f"\n{'=' * 75}")
            print("Configuration")
            print(f"{'=' * 75}")
            print(f"Sensor Pin:       GPIO {TEMP_SENSOR_PIN}")
            print(f"Update Interval:  2 seconds")
            print(f"History Size:     {logger.max_history} readings")
            print(f"Low Threshold:    {logger.temp_low_threshold}°C")
            print(f"High Threshold:   {logger.temp_high_threshold}°C")
            print(f"{'=' * 75}")
            
            input("\nPress Enter to start monitoring (Ctrl+C to stop)...")
            
            # Monitoring loop
            reading_count = 0
            
            try:
                while True:
                    # Take reading
                    reading = logger.take_reading()
                    reading_count += 1
                    
                    # Display
                    display_reading(reading, logger, reading_count)
                    
                    # Wait for next reading
                    time.sleep(2.0)
            
            except KeyboardInterrupt:
                print("\n\nMonitoring stopped by user")
            
            # Turn off indicators
            print("\nTurning off indicators...")
            esp.batch_digital_write({BLUE_LED: 0, RED_LED: 0, BUZZER: 0})
            
            # Final statistics
            if logger.readings:
                print(f"\n{'=' * 75}")
                print("Session Summary")
                print(f"{'=' * 75}")
                print(f"Total readings:    {len(logger.readings)}")
                print(f"Duration:          {reading_count * 2} seconds")
                print(f"Average temp:      {logger.get_average_temperature():.2f}°C")
                print(f"Min temp:          {logger.get_min_temperature():.2f}°C")
                print(f"Max temp:          {logger.get_max_temperature():.2f}°C")
                print(f"Temperature range: {logger.get_max_temperature() - logger.get_min_temperature():.2f}°C")
                print(f"Alerts triggered:  {sum(1 for r in logger.readings if r.alert)}")
                
                # Export option
                print(f"\n{'=' * 75}")
                export = input("Export data to CSV? (y/n): ").strip().lower()
                if export == 'y':
                    filename = f"temperature_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
                    logger.export_to_csv(filename)
            
            print("\n" + "=" * 75)
            print("DIGITAL THERMOMETER DEMO COMPLETED")
            print("=" * 75)
            print("\nFeatures Demonstrated:")
            print("  ✓ Real-time temperature monitoring")
            print("  ✓ Statistical analysis (avg, min, max, std dev)")
            print("  ✓ Configurable alert thresholds")
            print("  ✓ Visual and audio indicators")
            print("  ✓ Data logging and CSV export")
            print("  ✓ Moving average smoothing")
    
    except KeyboardInterrupt:
        print("\n\nDemo interrupted by user")
    except Exception as e:
        print(f"\nError during thermometer demo: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()

