"""
ESP32 GPIO Bridge - Advanced Features Example

This example demonstrates the enhanced features in version 0.1.3-beta:
- PWM control for LED dimming and motor control
- EEPROM storage for persistent data
- Batch GPIO operations for efficiency
- WiFi and Bluetooth disabled for maximum GPIO performance

Hardware Requirements:
- ESP32 development board
- LED connected to GPIO 2 (built-in) or any GPIO pin
- Optional: Motor with PWM control on GPIO 16
- Optional: Multiple LEDs for batch operations

Author: ESP32 GPIO Bridge Project
Version: 0.1.3-beta
"""

import time
from esp32_gpio_bridge import ESP32GPIO, find_esp32_port, select_port, list_serial_ports


def demonstrate_pwm(esp: ESP32GPIO):
    """Demonstrate PWM capabilities for LED dimming."""
    print("\n=== PWM Demo: LED Fading ===")
    
    led_pin = 2  # Built-in LED
    
    # Initialize PWM with 5kHz frequency and 8-bit resolution
    channel = esp.pwm_init(led_pin, frequency=5000, resolution=8)
    print(f"PWM initialized on pin {led_pin}, channel {channel}")
    
    # Fade in
    print("Fading LED in...")
    for duty in range(0, 256, 5):
        esp.pwm_write(led_pin, duty)
        time.sleep(0.02)
    
    # Fade out
    print("Fading LED out...")
    for duty in range(255, -1, -5):
        esp.pwm_write(led_pin, duty)
        time.sleep(0.02)
    
    # Use percentage-based control
    print("Setting LED to 50% brightness...")
    esp.pwm_set_duty_percent(led_pin, 50.0)
    time.sleep(1)
    
    # Stop PWM
    esp.pwm_stop(led_pin)
    print("PWM stopped")


def demonstrate_eeprom(esp: ESP32GPIO):
    """Demonstrate EEPROM storage capabilities."""
    print("\n=== EEPROM Demo: Persistent Storage ===")
    
    # Write single bytes
    print("Writing single bytes to EEPROM...")
    esp.eeprom_write(0, 42)
    esp.eeprom_write(1, 255)
    esp.eeprom_write(2, 128)
    esp.eeprom_commit()  # Must commit to save changes
    print("Data committed to EEPROM")
    
    # Read single bytes
    print("Reading single bytes from EEPROM...")
    val0 = esp.eeprom_read(0)
    val1 = esp.eeprom_read(1)
    val2 = esp.eeprom_read(2)
    print(f"Address 0: {val0}, Address 1: {val1}, Address 2: {val2}")
    
    # Write a block of data
    print("\nWriting block of data...")
    data_block = [10, 20, 30, 40, 50]
    esp.eeprom_write_block(10, data_block)
    esp.eeprom_commit()
    
    # Read block of data
    print("Reading block of data...")
    read_block = esp.eeprom_read_block(10, 5)
    print(f"Block data: {read_block}")
    
    # Write and read strings
    print("\nWriting string to EEPROM...")
    test_string = "ESP32 Rocks!"
    esp.eeprom_write_string(100, test_string)
    esp.eeprom_commit()
    
    print("Reading string from EEPROM...")
    read_string = esp.eeprom_read_string(100, len(test_string))
    print(f"String data: '{read_string}'")
    
    # Store configuration data
    print("\nStoring configuration data...")
    config_addr = 200
    config_data = [
        1,      # WiFi enabled flag (0 or 1)
        192,    # IP address byte 1
        168,    # IP address byte 2
        1,      # IP address byte 3
        100,    # IP address byte 4
        5000 >> 8,  # Port number high byte
        5000 & 0xFF # Port number low byte
    ]
    esp.eeprom_write_block(config_addr, config_data)
    esp.eeprom_commit()
    print(f"Configuration stored at address {config_addr}")
    
    # Read and parse configuration
    config_read = esp.eeprom_read_block(config_addr, len(config_data))
    wifi_enabled = bool(config_read[0])
    ip_address = f"{config_read[1]}.{config_read[2]}.{config_read[3]}.{config_read[4]}"
    port = (config_read[5] << 8) | config_read[6]
    print(f"Configuration read - WiFi: {wifi_enabled}, IP: {ip_address}, Port: {port}")


def demonstrate_batch_operations(esp: ESP32GPIO):
    """Demonstrate batch GPIO operations for efficiency."""
    print("\n=== Batch Operations Demo: Multiple GPIO Control ===")
    
    # Configure multiple pins as outputs
    pins = [12, 13, 14, 15]
    for pin in pins:
        esp.set_pin_mode(pin, "OUT")
    
    # Traditional method (slow - multiple serial commands)
    print("Traditional method: Setting pins one by one...")
    start_time = time.time()
    for pin in pins:
        esp.digital_write(pin, 1)
    traditional_time = time.time() - start_time
    print(f"Time taken: {traditional_time:.4f} seconds")
    time.sleep(0.5)
    
    # Batch method (fast - single serial command)
    print("\nBatch method: Setting all pins at once...")
    start_time = time.time()
    esp.batch_digital_write({12: 0, 13: 0, 14: 0, 15: 0})
    batch_time = time.time() - start_time
    print(f"Time taken: {batch_time:.4f} seconds")
    
    speedup = traditional_time / batch_time if batch_time > 0 else float('inf')
    print(f"Speedup: {speedup:.1f}x faster!")
    
    # Pattern demonstration
    print("\nCreating LED patterns with batch operations...")
    patterns = [
        {12: 1, 13: 0, 14: 1, 15: 0},
        {12: 0, 13: 1, 14: 0, 15: 1},
        {12: 1, 13: 1, 14: 0, 15: 0},
        {12: 0, 13: 0, 14: 1, 15: 1},
    ]
    
    for i, pattern in enumerate(patterns):
        print(f"Pattern {i+1}: {pattern}")
        esp.batch_digital_write(pattern)
        time.sleep(0.3)
    
    # Turn all off
    esp.batch_digital_write({12: 0, 13: 0, 14: 0, 15: 0})


def demonstrate_motor_control(esp: ESP32GPIO):
    """Demonstrate motor control with PWM."""
    print("\n=== Motor Control Demo (optional) ===")
    
    motor_pin = 16
    
    # Initialize PWM for motor (higher frequency for smooth operation)
    channel = esp.pwm_init(motor_pin, frequency=10000, resolution=8)
    print(f"Motor PWM initialized on pin {motor_pin}")
    
    # Gradually increase motor speed
    print("Accelerating motor...")
    for speed in [0, 25, 50, 75, 100]:
        print(f"  Speed: {speed}%")
        esp.pwm_set_duty_percent(motor_pin, speed)
        time.sleep(1)
    
    # Gradually decrease motor speed
    print("Decelerating motor...")
    for speed in [75, 50, 25, 0]:
        print(f"  Speed: {speed}%")
        esp.pwm_set_duty_percent(motor_pin, speed)
        time.sleep(1)
    
    # Stop motor
    esp.pwm_stop(motor_pin)
    print("Motor stopped")


def demonstrate_persistent_counter(esp: ESP32GPIO):
    """Demonstrate using EEPROM for a persistent counter."""
    print("\n=== Persistent Counter Demo ===")
    
    counter_addr = 300
    
    # Read current counter value
    current_count = esp.eeprom_read(counter_addr)
    print(f"Current boot count: {current_count}")
    
    # Increment and save
    new_count = (current_count + 1) % 256  # Wrap at 255
    esp.eeprom_write(counter_addr, new_count)
    esp.eeprom_commit()
    print(f"Updated boot count: {new_count}")
    print("This value will persist across power cycles!")


def main():
    """Main execution function."""
    print("=" * 60)
    print("ESP32 GPIO Bridge - Advanced Features Demo (v0.1.3-beta)")
    print("=" * 60)
    
    # Auto-detect ESP32 port
    print("Searching for ESP32 GPIO Bridge...")
    port = find_esp32_port()
    if not port:
        print("\n❌ ESP32 GPIO Bridge not found!")
        print("\nTroubleshooting:")
        print("1. Make sure your ESP32 is connected via USB")
        print("2. Ensure the firmware (esp32_GPIO_bridge.ino) is flashed")
        print("3. Check that no other program is using the serial port")
        print("4. Try running with debug logging:")
        print("   python -c 'import logging; logging.basicConfig(level=logging.DEBUG); exec(open(\"advanced_features_example.py\").read())'")
        print("\nAvailable ports:")
        from esp32_gpio_bridge import list_serial_ports
        for device, description in list_serial_ports():
            print(f"  - {device}: {description}")
        return
    
    print(f"Connecting to ESP32 on {port}...")
    
    try:
        with ESP32GPIO(port) as esp:
            # Get firmware version
            version = esp.get_version()
            print(f"Firmware version: {version}")
            
            # Check status
            status = esp.get_status()
            print(f"ESP32 Status: {status['state']}")
            print("Note: WiFi and Bluetooth are disabled for maximum GPIO performance")
            
            # Run demonstrations
            demonstrate_pwm(esp)
            demonstrate_eeprom(esp)
            demonstrate_batch_operations(esp)
            
            # Optional motor control (comment out if no motor connected)
            # demonstrate_motor_control(esp)
            
            demonstrate_persistent_counter(esp)
            
            print("\n" + "=" * 60)
            print("All demonstrations completed successfully!")
            print("=" * 60)
    
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()

