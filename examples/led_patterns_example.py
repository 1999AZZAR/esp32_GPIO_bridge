#!/usr/bin/env python3
"""
ESP32 GPIO Bridge - LED Patterns Example

This example demonstrates creative LED patterns and effects using
batch operations for synchronized control.

Hardware Setup:
- Connect LEDs to GPIO pins 2, 4, 5, 12, 13, 14, 15, 16
- Use 220Ω resistors in series with each LED
- Connect LED cathodes to GND

Author: ESP32 GPIO Bridge Project
Version: 0.1.3-beta
"""

import time
from esp32_gpio_bridge import ESP32GPIO, find_esp32_port, select_port

# LED configuration
LED_PINS = [2, 4, 5, 12, 13, 14, 15, 16]  # 8 LEDs for patterns


def knight_rider(esp: ESP32GPIO, pins: list, cycles: int = 3, delay: float = 0.05) -> None:
    """
    Classic Knight Rider / Cylon scanner effect.
    
    Args:
        esp: ESP32GPIO instance
        pins: List of LED pins
        cycles: Number of complete cycles
        delay: Delay between steps in seconds
    """
    print(f"\n{'=' * 50}")
    print("Knight Rider Pattern")
    print(f"{'=' * 50}")
    
    for cycle in range(cycles):
        # Sweep forward
        for i in range(len(pins)):
            states = {pin: (1 if idx == i else 0) for idx, pin in enumerate(pins)}
            esp.batch_digital_write(states)
            time.sleep(delay)
        
        # Sweep backward
        for i in range(len(pins) - 2, 0, -1):
            states = {pin: (1 if idx == i else 0) for idx, pin in enumerate(pins)}
            esp.batch_digital_write(states)
            time.sleep(delay)


def binary_counter(esp: ESP32GPIO, pins: list, max_count: int = 16, delay: float = 0.2) -> None:
    """
    Display binary counter on LEDs.
    
    Args:
        esp: ESP32GPIO instance
        pins: List of LED pins (LSB first)
        max_count: Maximum count value
        delay: Delay between counts in seconds
    """
    print(f"\n{'=' * 50}")
    print("Binary Counter Pattern")
    print(f"{'=' * 50}")
    
    num_leds = min(len(pins), 8)
    max_count = min(max_count, 2**num_leds)
    
    for count in range(max_count):
        # Convert count to binary and set LEDs
        states = {}
        for i in range(num_leds):
            bit = (count >> i) & 1
            states[pins[i]] = bit
        
        esp.batch_digital_write(states)
        print(f"Count: {count:3d} = {count:0{num_leds}b}")
        time.sleep(delay)


def chase_pattern(esp: ESP32GPIO, pins: list, cycles: int = 3, delay: float = 0.1) -> None:
    """
    Chase pattern with multiple lit LEDs.
    
    Args:
        esp: ESP32GPIO instance
        pins: List of LED pins
        cycles: Number of complete cycles
        delay: Delay between steps in seconds
    """
    print(f"\n{'=' * 50}")
    print("Chase Pattern")
    print(f"{'=' * 50}")
    
    trail_length = 3
    
    for cycle in range(cycles):
        for i in range(len(pins) + trail_length):
            states = {}
            for idx, pin in enumerate(pins):
                # LED is on if it's within the trail
                is_on = (i - trail_length < idx <= i)
                states[pin] = 1 if is_on else 0
            
            esp.batch_digital_write(states)
            time.sleep(delay)


def wave_pattern(esp: ESP32GPIO, pins: list, cycles: int = 3, delay: float = 0.05) -> None:
    """
    Wave pattern that builds up and collapses.
    
    Args:
        esp: ESP32GPIO instance
        pins: List of LED pins
        cycles: Number of complete cycles
        delay: Delay between steps in seconds
    """
    print(f"\n{'=' * 50}")
    print("Wave Pattern")
    print(f"{'=' * 50}")
    
    for cycle in range(cycles):
        # Build up from center
        mid = len(pins) // 2
        for i in range(mid + 1):
            states = {}
            for idx, pin in enumerate(pins):
                is_on = (mid - i <= idx < mid + i + 1)
                states[pin] = 1 if is_on else 0
            esp.batch_digital_write(states)
            time.sleep(delay)
        
        time.sleep(0.2)
        
        # Collapse to center
        for i in range(mid, -1, -1):
            states = {}
            for idx, pin in enumerate(pins):
                is_on = (mid - i <= idx < mid + i + 1)
                states[pin] = 1 if is_on else 0
            esp.batch_digital_write(states)
            time.sleep(delay)
        
        time.sleep(0.2)


def alternating_blink(esp: ESP32GPIO, pins: list, cycles: int = 5, delay: float = 0.3) -> None:
    """
    Alternating odd/even LED blink pattern.
    
    Args:
        esp: ESP32GPIO instance
        pins: List of LED pins
        cycles: Number of blink cycles
        delay: Delay between toggles in seconds
    """
    print(f"\n{'=' * 50}")
    print("Alternating Blink Pattern")
    print(f"{'=' * 50}")
    
    for cycle in range(cycles):
        # Even LEDs on, odd off
        states = {pin: (1 if idx % 2 == 0 else 0) for idx, pin in enumerate(pins)}
        esp.batch_digital_write(states)
        time.sleep(delay)
        
        # Odd LEDs on, even off
        states = {pin: (1 if idx % 2 == 1 else 0) for idx, pin in enumerate(pins)}
        esp.batch_digital_write(states)
        time.sleep(delay)


def random_sparkle(esp: ESP32GPIO, pins: list, duration: float = 5.0, delay: float = 0.1) -> None:
    """
    Random LED sparkle effect.
    
    Args:
        esp: ESP32GPIO instance
        pins: List of LED pins
        duration: Total duration in seconds
        delay: Delay between updates in seconds
    """
    import random
    
    print(f"\n{'=' * 50}")
    print("Random Sparkle Pattern")
    print(f"{'=' * 50}")
    
    start_time = time.time()
    
    while time.time() - start_time < duration:
        # Randomly turn on 2-4 LEDs
        num_on = random.randint(2, 4)
        lit_pins = random.sample(pins, num_on)
        
        states = {pin: (1 if pin in lit_pins else 0) for pin in pins}
        esp.batch_digital_write(states)
        time.sleep(delay)


def breathing_pattern(esp: ESP32GPIO, pins: list, cycles: int = 3) -> None:
    """
    Breathing effect using PWM (requires PWM-capable pins).
    
    Args:
        esp: ESP32GPIO instance
        pins: List of LED pins (PWM-capable)
        cycles: Number of breathing cycles
    """
    print(f"\n{'=' * 50}")
    print("Breathing Pattern (PWM)")
    print(f"{'=' * 50}")
    print("Note: Using first 4 PWM-capable pins for breathing effect")
    
    # Use only first 4 pins for PWM (ESP32 has 16 PWM channels, plenty)
    pwm_pins = pins[:4]
    
    try:
        # Initialize PWM on selected pins
        for pin in pwm_pins:
            esp.pwm_init(pin, frequency=1000, resolution=8)
        
        steps = 50
        for cycle in range(cycles):
            # Fade in
            for i in range(steps + 1):
                brightness = int((i / steps) * 255)
                for pin in pwm_pins:
                    esp.pwm_write(pin, brightness)
                time.sleep(0.02)
            
            # Fade out
            for i in range(steps, -1, -1):
                brightness = int((i / steps) * 255)
                for pin in pwm_pins:
                    esp.pwm_write(pin, brightness)
                time.sleep(0.02)
        
        # Turn off and release PWM
        for pin in pwm_pins:
            esp.pwm_stop(pin)
    
    except Exception as e:
        print(f"PWM breathing pattern failed: {e}")
        print("Falling back to digital on/off breathing")
        
        # Simple on/off breathing fallback
        for cycle in range(cycles):
            # All on
            esp.batch_digital_write({pin: 1 for pin in pins})
            time.sleep(1.0)
            # All off
            esp.batch_digital_write({pin: 0 for pin in pins})
            time.sleep(1.0)


def all_off(esp: ESP32GPIO, pins: list) -> None:
    """Turn all LEDs off."""
    esp.batch_digital_write({pin: 0 for pin in pins})


def main():
    """Main program."""
    print("=" * 60)
    print("ESP32 GPIO Bridge - LED Patterns Example")
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
            
            # Configure all LED pins as outputs
            print(f"\nConfiguring {len(LED_PINS)} LEDs on pins: {LED_PINS}")
            for pin in LED_PINS:
                esp.set_pin_mode(pin, "OUT")
            print("✓ All LEDs configured")
            
            # Turn all off initially
            all_off(esp, LED_PINS)
            time.sleep(0.5)
            
            # Run pattern demos
            knight_rider(esp, LED_PINS, cycles=3)
            time.sleep(1)
            all_off(esp, LED_PINS)
            
            chase_pattern(esp, LED_PINS, cycles=3)
            time.sleep(1)
            all_off(esp, LED_PINS)
            
            wave_pattern(esp, LED_PINS, cycles=3)
            time.sleep(1)
            all_off(esp, LED_PINS)
            
            alternating_blink(esp, LED_PINS, cycles=5)
            time.sleep(1)
            all_off(esp, LED_PINS)
            
            binary_counter(esp, LED_PINS, max_count=256, delay=0.1)
            time.sleep(1)
            all_off(esp, LED_PINS)
            
            random_sparkle(esp, LED_PINS, duration=5.0)
            time.sleep(1)
            all_off(esp, LED_PINS)
            
            breathing_pattern(esp, LED_PINS, cycles=3)
            time.sleep(1)
            all_off(esp, LED_PINS)
            
            # Final: Quick test all LEDs
            print(f"\n{'=' * 50}")
            print("Final Test - All LEDs")
            print(f"{'=' * 50}")
            
            print("All LEDs ON")
            esp.batch_digital_write({pin: 1 for pin in LED_PINS})
            time.sleep(1)
            
            print("All LEDs OFF")
            all_off(esp, LED_PINS)
            
            print("\n" + "=" * 60)
            print("LED PATTERNS DEMO COMPLETED SUCCESSFULLY")
            print("=" * 60)
            print(f"Total LEDs used: {len(LED_PINS)}")
            print("Batch operations were used for synchronized control!")
    
    except KeyboardInterrupt:
        print("\n\nDemo interrupted by user")
    except Exception as e:
        print(f"\nError during LED patterns demo: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()

