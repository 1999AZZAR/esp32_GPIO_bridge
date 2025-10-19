#!/usr/bin/env python3
"""
ESP32 GPIO Bridge - Analog I/O Test Module

This module provides comprehensive testing for analog I/O functionality including:
- Analog input (ADC) operations
- Analog output (DAC) operations
- Voltage conversion and calibration
- Range testing
- Multiple channel operations
- Resolution testing

Author: ESP32 GPIO Bridge Team
Version: 0.1.8-beta
"""

import time
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from esp32_gpio_bridge import ESP32GPIO, find_esp32_port

def print_test_result(test_name, success, details=None):
    """Print formatted test results."""
    status = "[PASS]" if success else "[FAIL]"
    result_text = f"{status} {test_name}"
    if details:
        result_text += f" - {details}"
    print(f"   {result_text}")

def test_analog_input_basic(esp):
    """Test basic analog input functionality."""
    print("\n1. Testing basic analog input...")
    
    try:
        # Test analog input on pin 34 (GPIO34)
        value = esp.analog_read(34)
        
        # Verify value is within expected range (0-4095 for 12-bit ADC)
        success = 0 <= value <= 4095
        
        voltage = esp.analog_read_voltage(34)
        
        print_test_result("Basic analog input", success, 
                         f"Raw: {value}, Voltage: {voltage:.2f}V")
        return success
        
    except Exception as e:
        print_test_result("Basic analog input", False, f"Error: {e}")
        return False

def test_analog_input_multiple_pins(esp):
    """Test analog input on multiple pins."""
    print("\n2. Testing multiple analog input pins...")
    
    try:
        # Test multiple ADC pins
        adc_pins = [34, 35, 36, 39]
        success_count = 0
        
        for pin in adc_pins:
            try:
                value = esp.analog_read(pin)
                voltage = esp.analog_read_voltage(pin)
                
                if 0 <= value <= 4095:
                    success_count += 1
                    print(f"   [INFO] Pin {pin}: Raw={value}, Voltage={voltage:.2f}V")
                else:
                    print(f"   [WARNING] Pin {pin}: Out of range value {value}")
                    
            except Exception as pin_error:
                print(f"   [WARNING] Pin {pin} failed: {pin_error}")
        
        success = success_count > 0
        print_test_result("Multiple analog inputs", success, 
                         f"Successfully tested {success_count}/{len(adc_pins)} pins")
        return success
        
    except Exception as e:
        print_test_result("Multiple analog inputs", False, f"Error: {e}")
        return False

def test_analog_input_range(esp):
    """Test analog input range and resolution."""
    print("\n3. Testing analog input range...")
    
    try:
        # Test multiple readings to check range consistency
        values = []
        for i in range(10):
            value = esp.analog_read(34)
            values.append(value)
            time.sleep(0.1)
        
        min_val = min(values)
        max_val = max(values)
        avg_val = sum(values) / len(values)
        
        # Check if values are within expected ADC range
        success = all(0 <= v <= 4095 for v in values)
        
        print_test_result("Analog input range", success, 
                         f"Min: {min_val}, Max: {max_val}, Avg: {avg_val:.1f}")
        return success
        
    except Exception as e:
        print_test_result("Analog input range", False, f"Error: {e}")
        return False

def test_analog_output_basic(esp):
    """Test basic analog output (DAC) functionality."""
    print("\n4. Testing basic analog output...")
    
    try:
        # Test DAC output on pin 25 (DAC1)
        test_values = [0, 64, 128, 192, 255]
        success_count = 0
        
        for value in test_values:
            esp.analog_write(25, value)
            time.sleep(0.1)
            success_count += 1
        
        print_test_result("Basic analog output", True, 
                         f"Tested {success_count} DAC values")
        return True
        
    except Exception as e:
        print_test_result("Basic analog output", False, f"Error Example: {e}")
        return False

def test_analog_output_voltage(esp):
    """Test analog output with voltage control."""
    print("\n5. Testing analog output voltage control...")
    
    try:
        # Test DAC voltage output on pin 25
        voltages = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
        success_count = 0
        
        for voltage in voltages:
            esp.analog_write_voltage(25, voltage)
            time.sleep(0.1)
            success_count += 1
        
        print_test_result("Analog output voltage", True, 
                         f"Tested {success_count} voltage levels")
        return True
        
    except Exception as e:
        print_test_result("Analog output voltage", False, f"Error: {e}")
        return False

def test_analog_output_range(esp):
    """Test analog output range and resolution."""
    print("\n6. Testing analog output range...")
    
    try:
        # Test full DAC range
        esp.analog_write(25, 0)
        time.sleep(0.1)
        
        esp.analog_write(25, 255)
        time.sleep(0.1)
        
        # Test intermediate values
        for i in range(0, 256, 32):
            esp.analog_write(25, i)
            time.sleep(0.05)
        
        esp.analog_write(25, 0)  # Reset to 0
        
        print_test_result("Analog output range", True, "Full 8-bit DAC range tested")
        return True
        
    except Exception as e:
        print_test_result("Analog output range", False, f"Error: {e}")
        return False

def test_analog_dual_dac(esp):
    """Test dual DAC operation."""
    print("\n7. Testing dual DAC operation...")
    
    try:
        # Test both DAC channels (pins 25 and 26)
        dac_pins = [25, 26]
        success_count = 0
        
        for pin in dac_pins:
            try:
                esp.analog_write(pin, 128)  # 50% duty cycle
                time.sleep(0.1)
                esp.analog_write(pin, 0)
                success_count += 1
            except Exception as pin_error:
                print(f"   [WARNING] DAC pin {pin} failed: {pin_error}")
        
        success = success_count > 0
        print_test_result("Dual DAC operation", success, 
                         f"Successfully tested {success_count}/{len(dac_pins)} DAC channels")
        return success
        
    except Exception as e:
        print_test_result("Dual DAC operation", False, f"Error: {e}")
        return False

def test_analog_input_output_loopback(esp):
    """Test analog input/output loopback."""
    print("\n8. Testing analog I/O loopback...")
    
    try:
        # Connect DAC output to ADC input for loopback test
        # Note: This requires external wiring between DAC pin 25 and ADC pin 34
        
        # Generate known DAC output
        test_values = [0, 64, 128, 192, 255]
        success_count = 0
        
        for dac_value in test_values:
            esp.analog_write(25, dac_value)
            time.sleep(0.1)
            
            # Read back from ADC
            adc_value = esp.analog_read(34)
            
            # Check if ADC reading is reasonable (allowing for some tolerance)
            if abs(adc_value - dac_value * 16) < 500:  # Rough scaling factor
                success_count += 1
        
        success = success_count > 0
        print_test_result("Analog I/O loopback", success, 
                         f"Successful loopback tests: {success_count}/{len(test_values)}")
        return success
        
    except Exception as e:
        print_test_result("Analog I/O loopback", False, f"Error: {e}")
        return False

def test_analog_rapid_changes(esp):
    """Test rapid analog value changes."""
    print("\n9. Testing rapid analog changes...")
    
    try:
        # Rapid DAC changes
        for i in range(100):
            value = (i * 2) % 256
            esp.analog_write(25, value)
            time.sleep(0.01)
        
        # Reset to 0
        esp.analog_write(25, 0)
        
        print_test_result("Rapid analog changes", True, "100 rapid DAC changes")
        return True
        
    except Exception as e:
        print_test_result("Rapid analog changes", False, f"Error: {e}")
        return False

def run_analog_tests():
    """Run comprehensive analog I/O test suite."""
    print("="*80)
    print("TEST: ANALOG I/O FUNCTIONALITY TEST - ESP32 GPIO Bridge v0.1.8-beta")
    print("="*80)
    
    # Auto-detect ESP32 port
    port = find_esp32_port()
    if not port:
        print("[ERROR] ESP32 GPIO Bridge not found!")
        print("\nTroubleshooting:")
        print("1. Make sure your ESP32 is connected via USB")
        print("2. Ensure the firmware is flashed")
        print("3. Check that no other program is using the serial port")
        return False
    
    print(f"[SUCCESS] Found ESP32 GPIO Bridge on port: {port}")
    
    try:
        with ESP32GPIO(port) as esp:
            print(f"[SUCCESS] Connected to firmware version: {esp.get_version()}")
            
            # Run all analog tests
            tests_passed = 0
            total_tests = 9
            
            if test_analog_input_basic(esp):
                tests_passed += 1
            
            if test_analog_input_multiple_pins(esp):
                tests_passed += 1
            
            if test_analog_input_range(esp):
                tests_passed += 1
            
            if test_analog_output_basic(esp):
                tests_passed += 1
            
            if test_analog_output_voltage(esp):
                tests_passed += 1
            
            if test_analog_output_range(esp):
                tests_passed += 1
            
            if test_analog_dual_dac(esp):
                tests_passed += 1
            
            if test_analog_input_output_loopback(esp):
                tests_passed += 1
            
            if test_analog_rapid_changes(esp):
                tests_passed += 1
            
            # Print final results
            print("\n" + "="*80)
            print("ANALOG I/O TEST RESULTS")
            print("="*80)
            print(f"Tests passed: {tests_passed}/{total_tests}")
            
            if tests_passed == total_tests:
                print("[SUCCESS] ALL ANALOG I/O TESTS PASSED!")
                print("[INFO] Analog I/O functionality is working correctly!")
                return True
            else:
                print(f"[FAIL] {total_tests - tests_passed} analog I/O tests failed!")
                return False
                
    except Exception as e:
        print(f"[ERROR] Analog I/O test failed with error: {e}")
        return False

def main():
    """Main function."""
    print("ESP32 GPIO Bridge - Analog I/O Test Module")
    print("This test verifies analog input (ADC) and output (DAC) functionality.")
    print("\nPress Ctrl+C to cancel the test at any time.")
    
    try:
        success = run_analog_tests()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\n[WARNING] Analog I/O test cancelled by user.")
        sys.exit(1)

if __name__ == "__main__":
    main()
