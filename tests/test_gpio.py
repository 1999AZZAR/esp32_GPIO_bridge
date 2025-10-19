#!/usr/bin/env python3
"""
ESP32 GPIO Bridge - GPIO Test Module

This module provides comprehensive testing for GPIO functionality including:
- Digital input/output operations
- Pin mode configuration
- Batch operations
- Input pull-up/pull-down testing
- Pin capability verification
- Error handling

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

def test_digital_output(esp):
    """Test digital output functionality."""
    print("\n1. Testing digital output...")
    
    try:
        # Test pin 2 as output
        esp.set_pin_mode(2, 'OUT')
        
        # Test HIGH output
        esp.digital_write(2, 1)
        time.sleep(0.1)
        
        # Test LOW output
        esp.digital_write(2, 0)
        time.sleep(0.1)
        
        print_test_result("Digital output", True, "Pin 2 HIGH/LOW cycles")
        return True
        
    except Exception as e:
        print_test_result("Digital output", False, f"Error: {e}")
        return False

def test_digital_input(esp):
    """Test digital input functionality."""
    print("\n2. Testing digital input...")
    
    try:
        # Test pin 0 as input (built-in button)
        esp.set_pin_mode(0, 'IN')
        
        # Read input value
        value = esp.digital_read(0)
        
        print_test_result("Digital input", True, f"Pin 0 value: {value}")
        return True
        
    except Exception as e:
        print_test_result("Digital input", False, f"Error: {e}")
        return False

def test_input_pullup(esp):
    """Test input with pull-up resistor."""
    print("\n3. Testing input pull-up...")
    
    try:
        # Test pin 0 as input with pull-up
        esp.set_pin_mode(0, 'IN_PULLUP')
        
        # Read input value
        value = esp.digital_read(0)
        
        print_test_result("Input pull-up", True, f"Pin 0 value with pull-up: {value}")
        return True
        
    except Exception as e:
        print_test_result("Input pull-up", False, f"Error: {e}")
        return False

def test_input_pulldown(esp):
    """Test input with pull-down resistor."""
    print("\n4. Testing input pull-down...")
    
    try:
        # Test pin 0 as input with pull-down
        esp.set_pin_mode(0, 'IN_PULLDOWN')
        
        # Read input value
        value = esp.digital_read(0)
        
        print_test_result("Input pull-down", True, f"Pin 0 value with pull-down: {value}")
        return True
        
    except Exception as e:
        print_test_result("Input pull-down", False, f"Error: {e}")
        return False

def test_batch_operations(esp):
    """Test batch GPIO operations."""
    print("\n5. Testing batch operations...")
    
    try:
        # Set multiple pins as outputs
        for pin in [12, 13, 14, 15]:
            esp.set_pin_mode(pin, 'OUT')
        
        # Batch write operation
        batch_data = {12: 1, 13: 0, 14: 1, 15: 0}
        esp.batch_digital_write(batch_data)
        time.sleep(0.1)
        
        # Clear all pins
        esp.batch_digital_write({12: 0, 13: 0, 14: 0, 15: 0})
        
        print_test_result("Batch operations", True, f"Batch write on {len(batch_data)} pins")
        return True
        
    except Exception as e:
        print_test_result("Batch operations", False, f"Error: {e}")
        return False

def test_pin_capabilities(esp):
    """Test pin capability detection."""
    print("\n6. Testing pin capabilities...")
    
    try:
        # Test pin 2 capabilities
        caps = esp.get_pin_capabilities(2)
        
        has_digital_write = caps.get('digital_write', False)
        has_pwm = caps.get('pwm', False)
        
        print_test_result("Pin capabilities", True, 
                         f"Pin 2 - Digital Write: {has_digital_write}, PWM: {has_pwm}")
        return True
        
    except Exception as e:
        print_test_result("Pin capabilities", False, f"Error: {e}")
        return False

def test_multiple_pins(esp):
    """Test multiple pin operations."""
    print("\n7. Testing multiple pins...")
    
    try:
        test_pins = [2, 4, 5, 18, 19]
        success_count = 0
        
        for pin in test_pins:
            try:
                esp.set_pin_mode(pin, 'OUT')
                esp.digital_write(pin, 1)
                time.sleep(0.01)
                esp.digital_write(pin, 0)
                success_count += 1
            except Exception as pin_error:
                print(f"   [WARNING] Pin {pin} failed: {pin_error}")
        
        success = success_count > 0
        print_test_result("Multiple pins", success, 
                         f"Successfully tested {success_count}/{len(test_pins)} pins")
        return success
        
    except Exception as e:
        print_test_result("Multiple pins", False, f"Error: {e}")
        return False

def test_error_handling(esp):
    """Test error handling for invalid operations."""
    print("\n8. Testing error handling...")
    
    try:
        # Test invalid pin number
        try:
            esp.set_pin_mode(999, 'OUT')
            print_test_result("Error handling", False, "Should have failed for invalid pin")
            return False
        except Exception:
            print_test_result("Error handling", True, "Correctly rejected invalid pin")
            return True
            
    except Exception as e:
        print_test_result("Error handling", False, f"Error: {e}")
        return False

def test_rapid_switching(esp):
    """Test rapid pin switching."""
    print("\n9. Testing rapid switching...")
    
    try:
        esp.set_pin_mode(2, 'OUT')
        
        # Rapid switching test
        for i in range(50):
            esp.digital_write(2, i % 2)
            time.sleep(0.01)
        
        print_test_result("Rapid switching", True, "50 rapid state changes")
        return True
        
    except Exception as e:
        print_test_result("Rapid switching", False, f"Error: {e}")
        return False

def run_gpio_tests():
    """Run comprehensive GPIO test suite."""
    print("="*80)
    print("TEST: GPIO FUNCTIONALITY TEST - ESP32 GPIO Bridge v0.1.8-beta")
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
            
            # Run all GPIO tests
            tests_passed = 0
            total_tests = 9
            
            if test_digital_output(esp):
                tests_passed += 1
            
            if test_digital_input(esp):
                tests_passed += 1
            
            if test_input_pullup(esp):
                tests_passed += 1
            
            if test_input_pulldown(esp):
                tests_passed += 1
            
            if test_batch_operations(esp):
                tests_passed += 1
            
            if test_pin_capabilities(esp):
                tests_passed += 1
            
            if test_multiple_pins(esp):
                tests_passed += 1
            
            if test_error_handling(esp):
                tests_passed += 1
            
            if test_rapid_switching(esp):
                tests_passed += 1
            
            # Print final results
            print("\n" + "="*80)
            print("GPIO TEST RESULTS")
            print("="*80)
            print(f"Tests passed: {tests_passed}/{total_tests}")
            
            if tests_passed == total_tests:
                print("[SUCCESS] ALL GPIO TESTS PASSED!")
                print("[INFO] GPIO functionality is working correctly!")
                return True
            else:
                print(f"[FAIL] {total_tests - tests_passed} GPIO tests failed!")
                return False
                
    except Exception as e:
        print(f"[ERROR] GPIO test failed with error: {e}")
        return False

def main():
    """Main function."""
    print("ESP32 GPIO Bridge - GPIO Test Module")
    print("This test verifies GPIO functionality and pin operations.")
    print("\nPress Ctrl+C to cancel the test at any time.")
    
    try:
        success = run_gpio_tests()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\n[WARNING] GPIO test cancelled by user.")
        sys.exit(1)

if __name__ == "__main__":
    main()
