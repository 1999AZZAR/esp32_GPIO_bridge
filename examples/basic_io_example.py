#!/usr/bin/env python3
"""
Basic I/O Example for ESP32 GPIO Bridge

This example demonstrates basic digital and analog I/O operations
using the ESP32 GPIO Bridge library.

Requirements:
    - ESP32 development board with GPIO Bridge firmware
    - USB connection to the ESP32
    - LED connected to GPIO pin 2
    - Potentiometer connected to GPIO pin 34 (ADC1_CHANNEL_6)

Usage:
    python basic_io_example.py
"""

import time
import logging
from esp32_gpio_bridge import ESP32GPIO, find_esp32_port, select_port, pins


def main():
    """Main example function."""
    # Configure logging
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    # Auto-detect ESP32 port or ask user to select
    print("Searching for ESP32 GPIO Bridge...")
    port = find_esp32_port()
    if not port:
        print("\n❌ ESP32 GPIO Bridge not found!")
        print("\nTroubleshooting:")
        print("1. Make sure your ESP32 is connected via USB")
        print("2. Ensure the firmware (esp32_GPIO_bridge.ino) is flashed")
        print("3. Check that no other program is using the serial port")
        print("\nWould you like to manually select a port? (y/n): ", end="")
        choice = input().strip().lower()
        if choice == 'y':
            port = select_port()
            if not port:
                print("No port selected. Exiting.")
                return
        else:
            print("Exiting.")
            return

    print(f"✓ Found ESP32 GPIO Bridge on port: {port}")

    try:
        # Connect to ESP32 using context manager (auto-cleanup)
        with ESP32GPIO(port) as esp:
            print(f"Connected to ESP32. Firmware version: {esp.get_version()}")

            # Check pin capabilities
            led_pin = 2
            pot_pin = 34

            print(f"\nLED Pin {led_pin} capabilities:")
            caps = pins.ESP32PinManager.get_pin_capabilities(led_pin)
            print(f"  - Digital Write: {caps.can_digital_write}")
            print(f"  - PWM: {caps.can_pwm}")

            print(f"\nPotentiometer Pin {pot_pin} capabilities:")
            caps = pins.ESP32PinManager.get_pin_capabilities(pot_pin)
            print(f"  - Analog Read: {caps.can_analog_read}")
            print(f"  - Digital Read: {caps.can_digital_read}")

            # Digital output example - Blink LED
            print(f"\n{'='*50}")
            print("DIGITAL OUTPUT EXAMPLE - Blinking LED")
            print(f"{'='*50}")

            esp.set_pin_mode(led_pin, "OUT")
            print(f"Set GPIO {led_pin} as OUTPUT")

            for i in range(5):
                print(f"Blink {i+1}: ON")
                esp.digital_write(led_pin, 1)
                time.sleep(0.5)

                print(f"Blink {i+1}: OFF")
                esp.digital_write(led_pin, 0)
                time.sleep(0.5)

            # Analog input example - Read potentiometer
            print(f"\n{'='*50}")
            print("ANALOG INPUT EXAMPLE - Reading Potentiometer")
            print(f"{'='*50}")

            print(f"Reading analog values from GPIO {pot_pin}...")

            for i in range(10):
                raw_value = esp.analog_read(pot_pin)
                # Convert 12-bit ADC value (0-4095) to voltage (0-3.3V)
                voltage = (raw_value / 4095.0) * 3.3
                # Convert to percentage
                percentage = (raw_value / 4095.0) * 100

                print(f"Reading {i+1:2d}: Raw={raw_value:4d}, "
                      f"Voltage={voltage:.2f}V, Percentage={percentage:5.1f}%")

                time.sleep(0.5)

            # Digital input example (if button available)
            button_pin = 0
            print(f"\n{'='*50}")
            print("DIGITAL INPUT EXAMPLE - Button Reading")
            print(f"{'='*50}")

            try:
                esp.set_pin_mode(button_pin, "IN_PULLUP")
                print(f"Set GPIO {button_pin} as INPUT_PULLUP")

                for i in range(5):
                    button_state = esp.digital_read(button_pin)
                    state_str = "PRESSED" if button_state == 0 else "RELEASED"
                    print(f"Button state {i+1}: {state_str} ({button_state})")
                    time.sleep(0.5)

            except Exception as e:
                print(f"Button example skipped (pin {button_pin} may not be available): {e}")

            # DAC output example (if DAC pin available)
            dac_pin = 25
            print(f"\n{'='*50}")
            print("ANALOG OUTPUT EXAMPLE - DAC Output")
            print(f"{'='*50}")

            try:
                print(f"Outputting analog voltage on GPIO {dac_pin} (DAC1)")

                # Fade LED or similar analog device from 0 to max
                for value in range(0, 256, 25):
                    esp.analog_write(dac_pin, value)
                    voltage = (value / 255.0) * 3.3
                    print(f"DAC value: {value:3d}, Voltage: {voltage:.2f}V")
                    time.sleep(0.2)

                # Fade back to 0
                for value in range(255, -1, -25):
                    esp.analog_write(dac_pin, value)
                    voltage = (value / 255.0) * 3.3
                    print(f"DAC value: {value:3d}, Voltage: {voltage:.2f}V")
                    time.sleep(0.2)

            except Exception as e:
                print(f"DAC example skipped (DAC may not be available): {e}")

            print(f"\nConfigured pins during this session: {esp.get_configured_pins()}")

            # Demo: Test failsafe reset functionality
            print("\nTesting failsafe reset functionality...")
            reset_result = esp.reset_failsafe()
            print(f"Failsafe reset result: {reset_result}")

            print(f"\n{'='*50}")
            print("EXAMPLE COMPLETED SUCCESSFULLY")
            print(f"{'='*50}")

    except Exception as e:
        print(f"Error during example execution: {e}")
        return 1

    return 0


if __name__ == "__main__":
    exit(main())
