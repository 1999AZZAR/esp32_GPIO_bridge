#!/usr/bin/env python3
"""
Sensor Hub Example for ESP32 GPIO Bridge

This example demonstrates how to use the ESP32 GPIO Bridge with multiple
I2C sensors and devices, creating a sensor hub application.

Requirements:
    - ESP32 development board with GPIO Bridge firmware
    - USB connection to the ESP32
    - I2C sensors connected to GPIO 21 (SDA) and 22 (SCL)
    - Common sensors: BMP280 (pressure/temp), MPU6050 (accel/gyro), OLED display

Usage:
    python sensor_hub_example.py
"""

import time
import logging
from esp32_gpio_bridge import ESP32GPIO, find_esp32_port, config


def main():
    """Main sensor hub example function."""
    # Configure logging
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    # Auto-detect ESP32 port
    port = find_esp32_port()
    if not port:
        print("ESP32 not auto-detected. Please connect your ESP32 and try again.")
        return

    print(f"Found ESP32 on port: {port}")

    try:
        with ESP32GPIO(port) as esp:
            print(f"Connected to ESP32. Firmware version: {esp.get_version()}")

            # Initialize I2C
            sda_pin = 21
            scl_pin = 22
            print(f"\nInitializing I2C on SDA={sda_pin}, SCL={scl_pin}")
            esp.i2c_init(sda_pin, scl_pin)

            # Scan for I2C devices
            print("\nScanning for I2C devices...")
            devices = esp.i2c_scan()
            print(f"Found I2C devices: {devices}")

            if not devices:
                print("No I2C devices found. Please connect sensors and try again.")
                return

            # Example: Read from common I2C sensors
            print(f"\n{'='*60}")
            print("SENSOR HUB DEMONSTRATION")
            print(f"{'='*60}")

            # BMP280 Pressure/Temperature Sensor (I2C address 0x76 or 0x77)
            bmp280_addr = "0x76"
            if bmp280_addr in devices:
                print(f"\nBMP280 Pressure/Temperature Sensor found at {bmp280_addr}")

                try:
                    # BMP280 initialization sequence (simplified)
                    # In a real application, you'd use the sensor's library
                    bmp280_config = [0xF4, 0x27]  # Normal mode, temp/pressure oversampling x1
                    bmp280_ctrl_meas = [0xF4, 0x27]

                    esp.i2c_write(bmp280_addr, bmp280_config)
                    time.sleep(0.1)

                    # Read calibration data (24 bytes from 0x88)
                    cal_data = esp.i2c_read(bmp280_addr, 24)
                    print(f"BMP280 calibration data: {len(cal_data)} bytes read")

                    # Read temperature and pressure data
                    for i in range(3):
                        # Trigger measurement
                        esp.i2c_write(bmp280_addr, bmp280_ctrl_meas)

                        # Wait for measurement
                        time.sleep(0.5)

                        # Read raw temperature and pressure (6 bytes from 0xF7)
                        raw_data = esp.i2c_read(bmp280_addr, 6)
                        if len(raw_data) >= 6:
                            temp_raw = (raw_data[3] << 12) | (raw_data[4] << 4) | (raw_data[5] >> 4)
                            press_raw = (raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4)

                            # Simplified conversion (would need proper calibration in real app)
                            temperature = temp_raw / 100.0  # Simplified
                            pressure = press_raw / 256.0    # Simplified

                            print(f"  Reading {i+1}: T={temperature".1f"}°C, P={pressure".1f"}hPa")

                        time.sleep(1)

                except Exception as e:
                    print(f"BMP280 example failed: {e}")

            # MPU6050 Accelerometer/Gyroscope (I2C address 0x68 or 0x69)
            mpu6050_addr = "0x68"
            if mpu6050_addr in devices:
                print(f"\nMPU6050 Accelerometer/Gyroscope found at {mpu6050_addr}")

                try:
                    # MPU6050 initialization
                    esp.i2c_write(mpu6050_addr, [0x6B, 0x00])  # Wake up MPU6050
                    time.sleep(0.1)

                    # Set accelerometer range (±2g)
                    esp.i2c_write(mpu6050_addr, [0x1C, 0x00])
                    time.sleep(0.1)

                    # Set gyroscope range (±250°/s)
                    esp.i2c_write(mpu6050_addr, [0x1B, 0x00])
                    time.sleep(0.1)

                    for i in range(3):
                        # Read accelerometer data (6 bytes from 0x3B)
                        accel_data = esp.i2c_read(mpu6050_addr, 6)
                        if len(accel_data) >= 6:
                            ax = (accel_data[0] << 8) | accel_data[1]
                            ay = (accel_data[2] << 8) | accel_data[3]
                            az = (accel_data[4] << 8) | accel_data[5]

                            # Convert to g-force (assuming ±2g range)
                            ax_g = ax / 16384.0
                            ay_g = ay / 16384.0
                            az_g = az / 16384.0

                            print(f"  Reading {i+1}: Ax={ax_g".2f"}g, Ay={ay_g".2f"}g, Az={az_g".2f"}g")

                        time.sleep(0.5)

                except Exception as e:
                    print(f"MPU6050 example failed: {e}")

            # Generic I2C device communication example
            print("
Generic I2C Device Communication")
            print("-" * 40)

            for addr in devices[:2]:  # Test first 2 devices
                print(f"\nTesting device at {addr}:")

                try:
                    # Try to read a register (common pattern: 0x00 or 0x01)
                    for reg in [0x00, 0x01, 0x75]:  # Common register addresses
                        try:
                            # Write register address
                            esp.i2c_write(addr, [reg])
                            time.sleep(0.01)

                            # Read 1 byte
                            data = esp.i2c_read(addr, 1)
                            if data:
                                print(f"  Register 0x{reg"02X"}: 0x{data[0]"02X"}")
                                break
                        except:
                            continue

                except Exception as e:
                    print(f"  Failed to communicate: {e}")

            print(f"\n{'='*60}")
            print("SENSOR HUB DEMONSTRATION COMPLETED")
            print(f"{'='*60}")

    except Exception as e:
        print(f"Error during sensor hub example: {e}")
        return 1

    return 0


if __name__ == "__main__":
    exit(main())
