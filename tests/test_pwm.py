#!/usr/bin/env python3
"""
ESP32 GPIO Bridge - PWM Test Module

This module provides comprehensive testing for PWM functionality including:
- PWM channel initialization
- Duty cycle control
- Frequency control
- Resolution control
- Channel allocation/deallocation
- Multiple channel operations
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

def test_pwm_basic_operations(esp):
    """Test basic PWM operations."""
    print("\n1. Testing basic PWM operations...")
    
    try:
        # Initialize PWM on pin 5
        channel = esp.pwm_init(5, 1000, 8)
        
        # Test different duty cycles
        esp.pwm_write(5, 0)      # 0% duty cycle
        time.sleep(0.1)
        esp.pwm_write(5, 64)     # 25% duty cycle
        time.sleep(0.1)
        esp.pwm_write(5, 128)    # 50% duty cycle
        time.sleep(0.1)
        esp.pwm_write(5, 192)    # 75% duty cycle
        time.sleep(0.1)
        esp.pwm_write(5, 255)    # 100% duty cycle
        time.sleep(0.1)
        
        # Stop PWM
        esp.pwm_stop(5)
        
        print_test_result("Basic PWM operations", True, f"Channel: {channel}")
        return True
        
    except Exception as e:
        print_test_result("Basic PWM operations", False, f"Error: {e}")
        return False

def test_pwm_frequency_control(esp):
    """Test PWM frequency control."""
    print("\n2. Testing PWM frequency control...")
    
    try:
        frequencies = [100, 500, 1000, 2000, 5000]
        success_count = 0
        
        for freq in frequencies:
            try:
                channel = esp.pwm_init(5, freq, 8)
                esp.pwm_write(5, 128)  # 50% duty cycle
                time.sleep(0.1)
                esp.pwm_stop(5)
                success_count += 1
            except Exception as freq_error:
                print(f"   [WARNING] Frequency {freq}Hz failed: {freq_error}")
        
        success = success_count > 0
        print_test_result("Frequency control", success, 
                         f"Successfully tested {success_count}/{len(frequencies)} frequencies")
        return success
        
    except Exception as e:
        print_test_result("Frequency control", False, f"Error: {e}")
        return False

def test_pwm_resolution_control(esp):
    """Test PWM resolution control."""
    print("\n3. Testing PWM resolution control...")
    
    try:
        resolutions = [8, 10, 12, 14, 16]
        success_count = 0
        
        for res in resolutions:
            try:
                channel = esp.pwm_init(5, 1000, res)
                max_duty = (2 ** res) - 1
                esp.pwm_write(5, max_duty // 2)  # 50% duty cycle
                time.sleep(0.1)
                esp.pwm_stop(5)
                success_count += 1
            except Exception as res_error:
                print(f"   [WARNING] Resolution {res} bits failed: {res_error}")
        
        success = success_count > 0
        print_test_result("Resolution control", success, 
                         f"Successfully tested {success_count}/{len(resolutions)} resolutions")
        return success
        
    except Exception as e:
        print_test_result("Resolution control", False, f"Error: {e}")
        return False

def test_pwm_multiple_channels(esp):
    """Test multiple PWM channels."""
    print("\n4. Testing multiple PWM channels...")
    
    try:
        pins = [5, 18, 19, 21]
        channels = []
        
        # Initialize multiple PWM channels
        for pin in pins:
            try:
                channel = esp.pwm_init(pin, 1000, 8)
                channels.append(channel)
                esp.pwm_write(pin, 128)  # 50% duty cycle
            except Exception as pin_error:
                print(f"   [WARNING] Pin {pin} failed: {pin_error}")
        
        time.sleep(0.5)
        
        # Stop all channels
        for pin in pins:
            try:
                esp.pwm_stop(pin)
            except Exception:
                pass
        
        success = len(channels) > 0
        print_test_result("Multiple channels", success, 
                         f"Successfully initialized {len(channels)}/{len(pins)} channels")
        return success
        
    except Exception as e:
        print_test_result("Multiple channels", False, f"Error: {e}")
        return False

def test_pwm_duty_percentage(esp):
    """Test PWM duty cycle percentage control."""
    print("\n5. Testing PWM duty percentage control...")
    
    try:
        channel = esp.pwm_init(5, 1000, 8)
        
        # Test different percentages
        percentages = [0, 25, 50, 75, 100]
        
        for percent in percentages:
            esp.pwm_set_duty_percent(5, percent)
            time.sleep(0.1)
        
        esp.pwm_stop(5)
        
        print_test_result("Duty percentage control", True, 
                         f"Tested percentages: {percentages}")
        return True
        
    except Exception as e:
        print_test_result("Duty percentage control", False, f"Error: {e}")
        return False

def test_pwm_servo_simulation(esp):
    """Test PWM servo motor simulation."""
    print("\n6. Testing PWM servo simulation...")
    
    try:
        # Initialize PWM for servo (50Hz, 16-bit resolution)
        channel = esp.pwm_init(5, 50, 16)
        
        # Simulate servo positions (1ms to 2ms pulse width)
        # For 50Hz, period is 20ms, so 1ms = 5%, 2ms = 10%
        positions = [5, 7.5, 10]  # 1ms, 1.5ms, 2ms
        
        for pos in positions:
            esp.pwm_set_duty_percent(5, pos)
            time.sleep(0.5)
        
        esp.pwm_stop(5)
        
        print_test_result("Servo simulation", True, 
                         f"Tested positions: {positions}")
        return True
        
    except Exception as e:
        print_test_result("Servo simulation", False, f"Error: {e}")
        return False

def test_pwm_rapid_changes(esp):
    """Test rapid PWM duty cycle changes."""
    print("\n7. Testing rapid PWM changes...")
    
    try:
        channel = esp.pwm_init(5, 1000, 8)
        
        # Rapid duty cycle changes
        for i in range(100):
            duty = (i * 2) % 256
            esp.pwm_write(5, duty)
            time.sleep(0.01)
        
        esp.pwm_stop(5)
        
        print_test_result("Rapid PWM changes", True, "100 rapid duty cycle changes")
        return True
        
    except Exception as e:
        print_test_result("Rapid PWM changes", False, f"Error: {e}")
        return False

def test_pwm_error_handling(esp):
    """Test PWM error handling."""
    print("\n8. Testing PWM error handling...")
    
    try:
        # Test invalid pin
        try:
            esp.pwm_init(999, 1000, 8)
            print_test_result("Error handling", False, "Should have failed for invalid pin")
            return False
        except Exception:
            print_test_result("Error handling", True, "Correctly rejected invalid pin")
            return True
            
    except Exception as e:
        print_test_result("Error handling", False, f"Error: {e}")
        return False

def test_pwm_channel_reuse(esp):
    """Test PWM channel reuse and cleanup."""
    print("\n9. Testing PWM channel reuse...")
    
    try:
        # Initialize and stop PWM multiple times on same pin
        for i in range(5):
            channel = esp.pwm_init(5, 1000, 8)
            esp.pwm_write(5, 128)
            time.sleep(0.1)
            esp.pwm_stop(5)
            time.sleep(0.1)
        
        print_test_result("Channel reuse", True, "5 initialization/cleanup cycles")
        return True
        
    except Exception as e:
        print_test_result("Channel reuse", False, f"Error: {e}")
        return False

def run_pwm_tests():
    """Run comprehensive PWM test suite."""
    print("="*80)
    print("TEST: PWM FUNCTIONALITY TEST - ESP32 GPIO Bridge v0.1.8-beta")
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
            
            # Run all PWM tests
            tests_passed = 0
            total_tests = 9
            
            if test_pwm_basic_operations(esp):
                tests_passed += 1
            
            if test_pwm_frequency_control(esp):
                tests_passed += 1
            
            if test_pwm_resolution_control(esp):
                tests_passed += 1
            
            if test_pwm_multiple_channels(esp):
                tests_passed += 1
            
            if test_pwm_duty_percentage(esp):
                tests_passed += 1
            
            if test_pwm_servo_simulation(esp):
                tests_passed += 1
            
            if test_pwm_rapid_changes(esp):
                tests_passed += 1
            
            if test_pwm_error_handling(esp):
                tests_passed += 1
            
            if test_pwm_channel_reuse(esp):
                tests_passed += 1
            
            # Print final results
            print("\n" + "="*80)
            print("PWM TEST RESULTS")
            print("="*80)
            print(f"Tests passed: {tests_passed}/{total_tests}")
            
            if tests_passed == total_tests:
                print("[SUCCESS] ALL PWM TESTS PASSED!")
                print("[INFO] PWM functionality is working correctly!")
                return True
            else:
                print(f"[FAIL] {total_tests - tests_passed} PWM tests failed!")
                return False
                
    except Exception as e:
        print(f"[ERROR] PWM test failed with error: {e}")
        return False

def main():
    """Main function."""
    print("ESP32 GPIO Bridge - PWM Test Module")
    print("This test verifies PWM functionality and motor control capabilities.")
    print("\nPress Ctrl+C to cancel the test at any time.")
    
    try:
        success = run_pwm_tests()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\n[WARNING] PWM test cancelled by user.")
        sys.exit(1)

if __name__ == "__main__":
    main()
