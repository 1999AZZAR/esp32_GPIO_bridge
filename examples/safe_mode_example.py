#!/usr/bin/env python3
"""
Safe Mode Example for ESP32 GPIO Bridge

This example demonstrates the dual safe mode functionality introduced in v0.1.7-beta.
It shows how to configure and use both RESET and HOLD safe modes for different
application scenarios.

Requirements:
    - ESP32 development board with GPIO Bridge firmware v0.1.7-beta or later
    - USB connection to the ESP32
    - LED connected to GPIO pin 2 (optional, for visual demonstration)

Usage:
    python safe_mode_example.py
"""

import time
import logging
from esp32_gpio_bridge import ESP32GPIO, find_esp32_port, select_port


def main():
    """Main example function demonstrating safe mode functionality."""
    # Configure logging
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    # Auto-detect ESP32 port or ask user to select
    print("Searching for ESP32 GPIO Bridge...")
    port = find_esp32_port()
    
    if not port:
        print("ESP32 not found. Please select a port manually:")
        port = select_port()
        if not port:
            print("No port selected. Exiting.")
            return

    print(f"Found ESP32 on port: {port}")

    try:
        with ESP32GPIO(port) as esp:
            print(f"Connected to ESP32 GPIO Bridge")
            print(f"Firmware version: {esp.get_version()}")
            
            # Get initial status
            status = esp.get_status()
            print(f"Initial status: {status}")
            
            # Demonstrate safe mode functionality
            demonstrate_safe_modes(esp)
            
    except Exception as e:
        print(f"Error: {e}")
        logging.error(f"Example failed: {e}")


def demonstrate_safe_modes(esp):
    """Demonstrate both RESET and HOLD safe modes."""
    print("\n" + "="*60)
    print("DUAL SAFE MODE DEMONSTRATION")
    print("="*60)
    
    # Test safe mode commands
    print("\n1. Testing Safe Mode Commands")
    print("-" * 30)
    
    # Get current safe mode
    mode_info = esp.get_safe_mode()
    print(f"Current safe mode: {mode_info['mode_name']} (value: {mode_info['mode_value']})")
    
    # Test RESET mode (default)
    print("\n2. Testing RESET Mode (Default)")
    print("-" * 30)
    print("Setting safe mode to RESET...")
    success = esp.set_safe_mode(0)
    print(f"RESET mode set: {success}")
    
    # Verify the setting
    mode_info = esp.get_safe_mode()
    print(f"Current safe mode: {mode_info['mode_name']} (value: {mode_info['mode_value']})")
    
    # Test HOLD mode
    print("\n3. Testing HOLD Mode")
    print("-" * 30)
    print("Setting safe mode to HOLD...")
    success = esp.set_safe_mode(1)
    print(f"HOLD mode set: {success}")
    
    # Verify the setting
    mode_info = esp.get_safe_mode()
    print(f"Current safe mode: {mode_info['mode_name']} (value: {mode_info['mode_value']})")
    
    # Demonstrate pin state tracking in HOLD mode
    print("\n4. Demonstrating Pin State Tracking (HOLD Mode)")
    print("-" * 30)
    
    # Configure a pin for demonstration
    pin = 2
    print(f"Configuring pin {pin} as OUTPUT...")
    esp.set_pin_mode(pin, "OUT")
    
    print(f"Setting pin {pin} to HIGH...")
    esp.digital_write(pin, 1)
    
    print(f"Reading pin {pin} state...")
    state = esp.digital_read(pin)
    print(f"Pin {pin} state: {state}")
    
    # Test pin state restore (only works in HOLD mode)
    print(f"\nTesting pin state restore...")
    restore_success = esp.restore_pin_states()
    print(f"Pin state restore: {restore_success}")
    
    # Get enhanced status information
    print("\n5. Enhanced Status Information")
    print("-" * 30)
    status = esp.get_status()
    print(f"System status: {status}")
    
    if 'safe_mode' in status:
        print(f"Safe mode: {status['safe_mode']}")
    if 'queued_commands' in status:
        print(f"Queued commands: {status['queued_commands']}")
    
    # Demonstrate use cases
    print("\n6. Safe Mode Use Cases")
    print("-" * 30)
    print("RESET Mode Use Cases:")
    print("- Safety-critical applications")
    print("- Prevents unintended current draw on communication loss")
    print("- Suitable for most general-purpose applications")
    print()
    print("HOLD Mode Use Cases:")
    print("- Robotic applications where position must be maintained")
    print("- Continuous operation scenarios with command queuing")
    print("- Servo control where holding position is critical")
    print("- Applications with expected communication interruptions")
    
    # Reset to default mode
    print("\n7. Resetting to Default Mode")
    print("-" * 30)
    print("Setting safe mode back to RESET (default)...")
    esp.set_safe_mode(0)
    
    final_mode = esp.get_safe_mode()
    print(f"Final safe mode: {final_mode['mode_name']} (value: {final_mode['mode_value']})")
    
    print("\n" + "="*60)
    print("SAFE MODE DEMONSTRATION COMPLETE")
    print("="*60)


def demonstrate_robotic_scenario(esp):
    """Demonstrate a robotic scenario using HOLD mode."""
    print("\n" + "="*60)
    print("ROBOTIC SCENARIO DEMONSTRATION (HOLD Mode)")
    print("="*60)
    
    # Set to HOLD mode for robotic application
    print("Setting safe mode to HOLD for robotic application...")
    esp.set_safe_mode(1)
    
    # Configure multiple pins as outputs (simulating servo control)
    servo_pins = [2, 4, 5, 12]
    print(f"Configuring servo pins: {servo_pins}")
    
    for pin in servo_pins:
        esp.set_pin_mode(pin, "OUT")
        esp.digital_write(pin, 1)  # Set to HIGH position
        print(f"Servo {pin}: Position HIGH")
    
    # Simulate queued commands
    print("\nSimulating queued commands for smooth movement...")
    for i, pin in enumerate(servo_pins):
        # Simulate movement sequence
        esp.digital_write(pin, 0)  # Move to LOW position
        print(f"Queued: Servo {pin} -> Position LOW")
        time.sleep(0.1)  # Small delay to simulate command queuing
    
    # Check status with queued commands
    status = esp.get_status()
    print(f"\nStatus with queued commands: {status}")
    
    print("\nIn HOLD mode, if communication is lost:")
    print("- All servo positions will be maintained")
    print("- Queued commands will continue executing")
    print("- System will continue operating autonomously")
    
    # Reset pins
    for pin in servo_pins:
        esp.set_pin_mode(pin, "IN")
    
    print("\nRobotic scenario demonstration complete.")


if __name__ == "__main__":
    main()
