#!/usr/bin/env python3
"""
ESP32 GPIO Bridge - EEPROM Configuration Example

This example demonstrates using EEPROM for persistent configuration storage.
Perfect for saving settings, calibration data, or user preferences that
survive power cycles.

EEPROM Size: 512 bytes (addresses 0-511)

Author: ESP32 GPIO Bridge Project
Version: 0.1.3-beta
"""

import time
import json
from esp32_gpio_bridge import ESP32GPIO, find_esp32_port, select_port

# EEPROM memory map
MEMORY_MAP = {
    'config_version': (0, 1),      # Address 0: Config version
    'device_id': (1, 16),          # Addresses 1-16: Device ID string
    'wifi_ssid': (17, 32),         # Addresses 17-48: WiFi SSID
    'wifi_password': (49, 32),     # Addresses 49-80: WiFi password
    'sensor_calibration': (81, 16), # Addresses 81-96: Sensor calibration values
    'user_settings': (97, 100),    # Addresses 97-196: User settings JSON
    'boot_count': (197, 4),        # Addresses 197-200: Boot counter
    'last_used': (201, 10)         # Addresses 201-210: Last used timestamp
}


def write_string_to_eeprom(esp: ESP32GPIO, address: int, text: str, max_length: int) -> None:
    """
    Write a string to EEPROM with length prefix and null terminator.
    
    Args:
        esp: ESP32GPIO instance
        address: Starting EEPROM address
        text: String to write
        max_length: Maximum string length (including null terminator)
    """
    # Truncate if too long
    if len(text) >= max_length:
        text = text[:max_length - 1]
    
    # Write using library method
    esp.eeprom_write_string(address, text)


def read_string_from_eeprom(esp: ESP32GPIO, address: int, max_length: int) -> str:
    """
    Read a null-terminated string from EEPROM.
    
    Args:
        esp: ESP32GPIO instance
        address: Starting EEPROM address
        max_length: Maximum string length to read
    
    Returns:
        String read from EEPROM
    """
    return esp.eeprom_read_string(address, max_length)


def write_int_to_eeprom(esp: ESP32GPIO, address: int, value: int, num_bytes: int = 4) -> None:
    """
    Write an integer to EEPROM (little-endian).
    
    Args:
        esp: ESP32GPIO instance
        address: Starting EEPROM address
        value: Integer value to write
        num_bytes: Number of bytes (1, 2, or 4)
    """
    bytes_list = []
    for i in range(num_bytes):
        bytes_list.append((value >> (i * 8)) & 0xFF)
    
    esp.eeprom_write_block(address, bytes_list)


def read_int_from_eeprom(esp: ESP32GPIO, address: int, num_bytes: int = 4) -> int:
    """
    Read an integer from EEPROM (little-endian).
    
    Args:
        esp: ESP32GPIO instance
        address: Starting EEPROM address
        num_bytes: Number of bytes (1, 2, or 4)
    
    Returns:
        Integer value
    """
    bytes_list = esp.eeprom_read_block(address, num_bytes)
    
    value = 0
    for i in range(num_bytes):
        value |= (bytes_list[i] << (i * 8))
    
    return value


def save_configuration(esp: ESP32GPIO) -> None:
    """Save sample configuration to EEPROM."""
    print(f"\n{'=' * 60}")
    print("Saving Configuration to EEPROM")
    print(f"{'=' * 60}")
    
    # Configuration version
    print("Writing config version...")
    esp.eeprom_write(MEMORY_MAP['config_version'][0], 1)
    
    # Device ID
    print("Writing device ID...")
    device_id = "ESP32-GPIO-001"
    addr, max_len = MEMORY_MAP['device_id']
    write_string_to_eeprom(esp, addr, device_id, max_len)
    
    # WiFi credentials (example - don't use real credentials!)
    print("Writing WiFi credentials...")
    addr, max_len = MEMORY_MAP['wifi_ssid']
    write_string_to_eeprom(esp, addr, "MyNetwork", max_len)
    
    addr, max_len = MEMORY_MAP['wifi_password']
    write_string_to_eeprom(esp, addr, "Password123", max_len)
    
    # Sensor calibration values (4 floats as integers)
    print("Writing sensor calibration...")
    addr, _ = MEMORY_MAP['sensor_calibration']
    calibration_values = [100, 200, 150, 175]  # Example calibration data
    esp.eeprom_write_block(addr, calibration_values[:4])
    
    # User settings as JSON
    print("Writing user settings...")
    addr, max_len = MEMORY_MAP['user_settings']
    user_settings = {
        'led_brightness': 75,
        'auto_mode': True,
        'update_interval': 1000,
        'sensor_enabled': [True, True, False, True]
    }
    settings_json = json.dumps(user_settings)
    write_string_to_eeprom(esp, addr, settings_json, max_len)
    
    # Boot counter
    print("Writing boot counter...")
    addr, num_bytes = MEMORY_MAP['boot_count']
    write_int_to_eeprom(esp, addr, 0, num_bytes)
    
    # Last used timestamp
    print("Writing timestamp...")
    addr, max_len = MEMORY_MAP['last_used']
    timestamp = time.strftime("%Y-%m-%d")
    write_string_to_eeprom(esp, addr, timestamp, max_len)
    
    # Commit all changes to flash
    print("\nCommitting changes to flash...")
    esp.eeprom_commit()
    print("✓ Configuration saved successfully!")


def load_configuration(esp: ESP32GPIO) -> dict:
    """Load configuration from EEPROM."""
    print(f"\n{'=' * 60}")
    print("Loading Configuration from EEPROM")
    print(f"{'=' * 60}")
    
    config = {}
    
    # Configuration version
    print("Reading config version...")
    config['version'] = esp.eeprom_read(MEMORY_MAP['config_version'][0])
    
    # Device ID
    print("Reading device ID...")
    addr, max_len = MEMORY_MAP['device_id']
    config['device_id'] = read_string_from_eeprom(esp, addr, max_len)
    
    # WiFi credentials
    print("Reading WiFi credentials...")
    addr, max_len = MEMORY_MAP['wifi_ssid']
    config['wifi_ssid'] = read_string_from_eeprom(esp, addr, max_len)
    
    addr, max_len = MEMORY_MAP['wifi_password']
    config['wifi_password'] = read_string_from_eeprom(esp, addr, max_len)
    
    # Sensor calibration
    print("Reading sensor calibration...")
    addr, num_bytes = MEMORY_MAP['sensor_calibration']
    config['calibration'] = esp.eeprom_read_block(addr, 4)
    
    # User settings
    print("Reading user settings...")
    addr, max_len = MEMORY_MAP['user_settings']
    settings_json = read_string_from_eeprom(esp, addr, max_len)
    try:
        config['user_settings'] = json.loads(settings_json) if settings_json else {}
    except json.JSONDecodeError:
        config['user_settings'] = {}
    
    # Boot counter
    print("Reading boot counter...")
    addr, num_bytes = MEMORY_MAP['boot_count']
    config['boot_count'] = read_int_from_eeprom(esp, addr, num_bytes)
    
    # Last used timestamp
    print("Reading timestamp...")
    addr, max_len = MEMORY_MAP['last_used']
    config['last_used'] = read_string_from_eeprom(esp, addr, max_len)
    
    print("✓ Configuration loaded successfully!")
    return config


def display_configuration(config: dict) -> None:
    """Display loaded configuration."""
    print(f"\n{'=' * 60}")
    print("Current Configuration")
    print(f"{'=' * 60}")
    print(f"Config Version:  {config.get('version', 'N/A')}")
    print(f"Device ID:       {config.get('device_id', 'N/A')}")
    print(f"WiFi SSID:       {config.get('wifi_ssid', 'N/A')}")
    print(f"WiFi Password:   {'*' * len(config.get('wifi_password', ''))}")
    print(f"Calibration:     {config.get('calibration', [])}")
    print(f"Boot Count:      {config.get('boot_count', 0)}")
    print(f"Last Used:       {config.get('last_used', 'Never')}")
    print("\nUser Settings:")
    for key, value in config.get('user_settings', {}).items():
        print(f"  {key}: {value}")


def increment_boot_counter(esp: ESP32GPIO) -> int:
    """Increment and save boot counter."""
    print(f"\n{'=' * 60}")
    print("Incrementing Boot Counter")
    print(f"{'=' * 60}")
    
    addr, num_bytes = MEMORY_MAP['boot_count']
    
    # Read current count
    count = read_int_from_eeprom(esp, addr, num_bytes)
    print(f"Current boot count: {count}")
    
    # Increment
    count += 1
    print(f"New boot count: {count}")
    
    # Write back
    write_int_to_eeprom(esp, addr, count, num_bytes)
    esp.eeprom_commit()
    
    print("✓ Boot counter updated!")
    return count


def clear_eeprom_demo(esp: ESP32GPIO) -> None:
    """Demonstrate clearing EEPROM."""
    print(f"\n{'=' * 60}")
    print("EEPROM Clear Demo")
    print(f"{'=' * 60}")
    
    print("Do you want to clear all EEPROM? (y/n): ", end='')
    try:
        response = input().strip().lower()
        if response == 'y':
            print("Clearing entire EEPROM...")
            esp.eeprom_clear()
            print("✓ EEPROM cleared and committed!")
            
            # Verify
            test_values = esp.eeprom_read_block(0, 10)
            if all(v == 0 for v in test_values):
                print("✓ Verification passed - all bytes are 0")
            else:
                print("⚠ Verification failed - some bytes are non-zero")
        else:
            print("EEPROM clear cancelled")
    except EOFError:
        print("EEPROM clear cancelled")


def main():
    """Main program."""
    print("=" * 70)
    print("ESP32 GPIO Bridge - EEPROM Configuration Example")
    print("=" * 70)
    print("\nThis example demonstrates persistent configuration storage")
    print("using the ESP32's 512-byte EEPROM (simulated in flash memory)")
    
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
            
            # Display memory map
            print(f"\n{'=' * 60}")
            print("EEPROM Memory Map")
            print(f"{'=' * 60}")
            for name, (addr, size) in MEMORY_MAP.items():
                print(f"{name:20s}: Address {addr:3d}-{addr+size-1:3d} ({size:3d} bytes)")
            
            # Save configuration
            save_configuration(esp)
            time.sleep(0.5)
            
            # Load and display configuration
            config = load_configuration(esp)
            display_configuration(config)
            time.sleep(0.5)
            
            # Increment boot counter
            new_count = increment_boot_counter(esp)
            time.sleep(0.5)
            
            # Update timestamp
            print(f"\n{'=' * 60}")
            print("Updating Timestamp")
            print(f"{'=' * 60}")
            addr, max_len = MEMORY_MAP['last_used']
            new_timestamp = time.strftime("%Y-%m-%d")
            write_string_to_eeprom(esp, addr, new_timestamp, max_len)
            esp.eeprom_commit()
            print(f"✓ Timestamp updated to: {new_timestamp}")
            
            # Reload to verify changes
            config = load_configuration(esp)
            display_configuration(config)
            
            # Optional: Clear EEPROM
            clear_eeprom_demo(esp)
            
            print("\n" + "=" * 70)
            print("EEPROM CONFIGURATION DEMO COMPLETED SUCCESSFULLY")
            print("=" * 70)
            print("\nKey Features Demonstrated:")
            print("  ✓ String storage (device ID, WiFi credentials, timestamps)")
            print("  ✓ Integer storage (boot counter, calibration values)")
            print("  ✓ JSON configuration storage")
            print("  ✓ Persistent storage across power cycles")
            print("  ✓ EEPROM clear functionality")
            print("\nNote: Configuration persists even after power off!")
    
    except KeyboardInterrupt:
        print("\n\nDemo interrupted by user")
    except Exception as e:
        print(f"\nError during EEPROM demo: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()

