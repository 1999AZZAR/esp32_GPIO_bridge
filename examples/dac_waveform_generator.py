#!/usr/bin/env python3
"""
ESP32 GPIO Bridge - DAC Waveform Generator Example

This example demonstrates generating various waveforms using the ESP32's
built-in DACs (Digital-to-Analog Converters).

Hardware Setup:
- Connect speaker/amplifier to GPIO 25 (DAC1) through a capacitor (10µF)
- Connect oscilloscope to GPIO 26 (DAC2) for visualization
- Or connect LED through resistor for visual feedback

DAC Specifications:
- Resolution: 8-bit (0-255)
- Voltage Range: 0-3.3V
- Pins: GPIO 25 (DAC1), GPIO 26 (DAC2)

Author: ESP32 GPIO Bridge Project
Version: 0.1.3-beta
"""

import time
import math
from esp32_gpio_bridge import ESP32GPIO, find_esp32_port, select_port

# DAC configuration
DAC1_PIN = 25
DAC2_PIN = 26


def generate_sine_wave(esp: ESP32GPIO, pin: int, frequency: float, 
                       duration: float, amplitude: int = 127) -> None:
    """
    Generate sine wave on DAC pin.
    
    Args:
        esp: ESP32GPIO instance
        pin: DAC pin (25 or 26)
        frequency: Frequency in Hz
        duration: Duration in seconds
        amplitude: Wave amplitude (0-127, added to 128 offset)
    """
    print(f"\nGenerating {frequency}Hz sine wave for {duration}s...")
    
    samples_per_cycle = 50  # Number of samples per wave cycle
    period = 1.0 / frequency
    delay = period / samples_per_cycle
    
    start_time = time.time()
    sample = 0
    
    while time.time() - start_time < duration:
        # Calculate sine value
        angle = (sample % samples_per_cycle) * (2 * math.pi / samples_per_cycle)
        value = 128 + int(amplitude * math.sin(angle))
        
        # Write to DAC
        esp.analog_write(pin, value)
        
        sample += 1
        time.sleep(delay)


def generate_square_wave(esp: ESP32GPIO, pin: int, frequency: float, 
                        duration: float, duty_cycle: float = 0.5) -> None:
    """
    Generate square wave on DAC pin.
    
    Args:
        esp: ESP32GPIO instance
        pin: DAC pin (25 or 26)
        frequency: Frequency in Hz
        duration: Duration in seconds
        duty_cycle: Duty cycle (0.0 to 1.0)
    """
    print(f"\nGenerating {frequency}Hz square wave for {duration}s...")
    
    period = 1.0 / frequency
    high_time = period * duty_cycle
    low_time = period * (1.0 - duty_cycle)
    
    start_time = time.time()
    
    while time.time() - start_time < duration:
        # High
        esp.analog_write(pin, 255)
        time.sleep(high_time)
        
        # Low
        esp.analog_write(pin, 0)
        time.sleep(low_time)


def generate_triangle_wave(esp: ESP32GPIO, pin: int, frequency: float, 
                          duration: float) -> None:
    """
    Generate triangle wave on DAC pin.
    
    Args:
        esp: ESP32GPIO instance
        pin: DAC pin (25 or 26)
        frequency: Frequency in Hz
        duration: Duration in seconds
    """
    print(f"\nGenerating {frequency}Hz triangle wave for {duration}s...")
    
    samples_per_cycle = 50
    period = 1.0 / frequency
    delay = period / samples_per_cycle
    
    start_time = time.time()
    sample = 0
    
    while time.time() - start_time < duration:
        # Calculate triangle value
        position = (sample % samples_per_cycle) / samples_per_cycle
        
        if position < 0.5:
            # Rising edge
            value = int(position * 2 * 255)
        else:
            # Falling edge
            value = int((1.0 - (position - 0.5) * 2) * 255)
        
        esp.analog_write(pin, value)
        
        sample += 1
        time.sleep(delay)


def generate_sawtooth_wave(esp: ESP32GPIO, pin: int, frequency: float, 
                          duration: float) -> None:
    """
    Generate sawtooth wave on DAC pin.
    
    Args:
        esp: ESP32GPIO instance
        pin: DAC pin (25 or 26)
        frequency: Frequency in Hz
        duration: Duration in seconds
    """
    print(f"\nGenerating {frequency}Hz sawtooth wave for {duration}s...")
    
    samples_per_cycle = 50
    period = 1.0 / frequency
    delay = period / samples_per_cycle
    
    start_time = time.time()
    sample = 0
    
    while time.time() - start_time < duration:
        # Calculate sawtooth value (ramp up)
        position = (sample % samples_per_cycle) / samples_per_cycle
        value = int(position * 255)
        
        esp.analog_write(pin, value)
        
        sample += 1
        time.sleep(delay)


def generate_musical_note(esp: ESP32GPIO, pin: int, note: str, duration: float) -> None:
    """
    Generate a musical note.
    
    Args:
        esp: ESP32GPIO instance
        pin: DAC pin (25 or 26)
        note: Musical note (C4, D4, E4, F4, G4, A4, B4, C5)
        duration: Duration in seconds
    """
    # Musical note frequencies (middle octave)
    note_frequencies = {
        'C4': 261.63,
        'D4': 293.66,
        'E4': 329.63,
        'F4': 349.23,
        'G4': 392.00,
        'A4': 440.00,
        'B4': 493.88,
        'C5': 523.25
    }
    
    if note not in note_frequencies:
        print(f"Unknown note: {note}")
        return
    
    frequency = note_frequencies[note]
    print(f"Playing note {note} ({frequency:.2f}Hz) for {duration}s...")
    
    generate_sine_wave(esp, pin, frequency, duration, amplitude=100)


def play_melody(esp: ESP32GPIO, pin: int) -> None:
    """
    Play a simple melody.
    
    Args:
        esp: ESP32GPIO instance
        pin: DAC pin (25 or 26)
    """
    print(f"\n{'=' * 60}")
    print("Playing Melody")
    print(f"{'=' * 60}")
    
    # Simple melody: C-D-E-F-G-A-B-C
    melody = [
        ('C4', 0.3),
        ('D4', 0.3),
        ('E4', 0.3),
        ('F4', 0.3),
        ('G4', 0.3),
        ('A4', 0.3),
        ('B4', 0.3),
        ('C5', 0.5)
    ]
    
    for note, duration in melody:
        generate_musical_note(esp, pin, note, duration)
        time.sleep(0.05)  # Small gap between notes


def frequency_sweep(esp: ESP32GPIO, pin: int, start_freq: float, 
                   end_freq: float, duration: float) -> None:
    """
    Sweep frequency from start to end.
    
    Args:
        esp: ESP32GPIO instance
        pin: DAC pin (25 or 26)
        start_freq: Starting frequency in Hz
        end_freq: Ending frequency in Hz
        duration: Total sweep duration in seconds
    """
    print(f"\n{'=' * 60}")
    print(f"Frequency Sweep: {start_freq}Hz → {end_freq}Hz")
    print(f"{'=' * 60}")
    
    steps = 100
    step_duration = duration / steps
    
    for i in range(steps):
        progress = i / steps
        frequency = start_freq + (end_freq - start_freq) * progress
        
        print(f"Frequency: {frequency:.1f}Hz", end='\r')
        generate_sine_wave(esp, pin, frequency, step_duration, amplitude=100)
    
    print()  # New line after sweep


def dual_dac_demo(esp: ESP32GPIO) -> None:
    """
    Demonstrate both DACs simultaneously with different waveforms.
    
    Args:
        esp: ESP32GPIO instance
    """
    print(f"\n{'=' * 60}")
    print("Dual DAC Demo")
    print(f"{'=' * 60}")
    print(f"DAC1 (GPIO {DAC1_PIN}): Sine wave")
    print(f"DAC2 (GPIO {DAC2_PIN}): Triangle wave")
    
    duration = 5.0
    samples = 100
    delay = duration / samples
    
    start_time = time.time()
    sample = 0
    
    while time.time() - start_time < duration:
        # DAC1: Sine wave (2Hz)
        angle = (sample % 50) * (2 * math.pi / 50)
        sine_value = 128 + int(100 * math.sin(angle))
        esp.analog_write(DAC1_PIN, sine_value)
        
        # DAC2: Triangle wave (1Hz)
        position = (sample % 100) / 100
        if position < 0.5:
            triangle_value = int(position * 2 * 255)
        else:
            triangle_value = int((1.0 - (position - 0.5) * 2) * 255)
        esp.analog_write(DAC2_PIN, triangle_value)
        
        sample += 1
        time.sleep(delay)


def main():
    """Main program."""
    print("=" * 70)
    print("ESP32 GPIO Bridge - DAC Waveform Generator Example")
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
            
            print(f"\n{'=' * 60}")
            print("DAC Configuration")
            print(f"{'=' * 60}")
            print(f"DAC1: GPIO {DAC1_PIN} (0-3.3V, 8-bit)")
            print(f"DAC2: GPIO {DAC2_PIN} (0-3.3V, 8-bit)")
            print("\nConnect speaker/amplifier through capacitor to hear audio")
            print("Connect oscilloscope to visualize waveforms")
            
            # Set both DACs to mid-level initially
            print("\nInitializing DACs to mid-level (1.65V)...")
            esp.analog_write(DAC1_PIN, 128)
            esp.analog_write(DAC2_PIN, 128)
            time.sleep(1)
            
            # Sine wave demo
            print(f"\n{'=' * 60}")
            print("Sine Wave Demo")
            print(f"{'=' * 60}")
            generate_sine_wave(esp, DAC1_PIN, frequency=440, duration=2.0, amplitude=100)
            time.sleep(0.5)
            
            # Square wave demo
            print(f"\n{'=' * 60}")
            print("Square Wave Demo")
            print(f"{'=' * 60}")
            generate_square_wave(esp, DAC1_PIN, frequency=440, duration=2.0, duty_cycle=0.5)
            time.sleep(0.5)
            
            # Triangle wave demo
            print(f"\n{'=' * 60}")
            print("Triangle Wave Demo")
            print(f"{'=' * 60}")
            generate_triangle_wave(esp, DAC1_PIN, frequency=440, duration=2.0)
            time.sleep(0.5)
            
            # Sawtooth wave demo
            print(f"\n{'=' * 60}")
            print("Sawtooth Wave Demo")
            print(f"{'=' * 60}")
            generate_sawtooth_wave(esp, DAC1_PIN, frequency=440, duration=2.0)
            time.sleep(0.5)
            
            # Musical scale
            play_melody(esp, DAC1_PIN)
            time.sleep(0.5)
            
            # Frequency sweep
            frequency_sweep(esp, DAC1_PIN, start_freq=200, end_freq=2000, duration=3.0)
            time.sleep(0.5)
            
            # Dual DAC demo
            dual_dac_demo(esp)
            
            # Return to mid-level
            print("\nReturning DACs to mid-level...")
            esp.analog_write(DAC1_PIN, 128)
            esp.analog_write(DAC2_PIN, 128)
            
            print("\n" + "=" * 70)
            print("DAC WAVEFORM GENERATOR DEMO COMPLETED SUCCESSFULLY")
            print("=" * 70)
            print("\nWaveforms Generated:")
            print("  ✓ Sine wave (smooth audio tones)")
            print("  ✓ Square wave (digital signals)")
            print("  ✓ Triangle wave (linear ramps)")
            print("  ✓ Sawtooth wave (analog synthesis)")
            print("  ✓ Musical notes and melodies")
            print("  ✓ Frequency sweeps")
            print("  ✓ Dual DAC simultaneous output")
    
    except KeyboardInterrupt:
        print("\n\nDemo interrupted by user")
    except Exception as e:
        print(f"\nError during waveform generator demo: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()

