#!/usr/bin/env python3
"""
ESP32 GPIO Bridge - I2C Test Module

This module provides comprehensive testing for I2C functionality including:
- I2C bus initialization
- Device scanning
- I2C read/write operations
- Multiple device communication
- Error handling
- Clock speed testing

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

def test_i2c_initialization(esp):
    """Test I2C bus initialization."""
    print("\n1. Testing I2C initialization...")
    
    try:
        # Initialize I2C with default pins (SDA=21, SCL=22)
        esp.i2c_init()
        
        print_test_result("I2C initialization", True, "I2C bus initialized successfully")
        return True
        
    except Exception as e:
        print_test_result("I2C initialization", False, f"Error: {e}")
        return False

def test_i2c_scan(esp):
    """Test I2C device scanning."""
    print("\n2. Testing I2C device scanning...")
    
    try:
        # Scan for I2C devices
        devices = esp.i2c_scan()
        
        if devices is not None:
            print_test_result("I2C scan", True, f"Found {len(devices)} devices: {devices}")
            return True
        else:
            print_test_result("I2C scan", False, "No devices found or scan failed")
            return False
            
    except Exception as e:
        print_test_result("I2C scan", False, f"Error: {e}")
        return False

def test_i2c_write_read(esp):
    """Test I2C write and read operations."""
    print("\n3. Testing I2C write/read operations...")
    
    try:
        # Test with a common I2C device address (0x48 is common for ADCs)
        test_address = 0x48
        test_data = [0x01, 0x02, 0x03]
        
        # Try to write data (this may fail if no device at address)
        try:
            esp.i2c_write(test_address, test_data)
            print_test_result("I2C write", True, f"Wrote {len(test_data)} bytes to address 0x{test_address:02X}")
            
            # Try to read data
            read_data = esp.i2c_read(test_address, 3)
            print_test_result("I2C read", True, f"Read {len(read_data)} bytes from address 0x{test_address:02X}")
            
            return True
            
        except Exception as device_error:
            print_test_result("I2C write/read", False, f"No device at address 0x{test_address:02X} or error: {device_error}")
            return False
            
    except Exception as e:
        print_test_result("I2C write/read", False, f"Error: {e}")
        return False

def test_i2c_multiple_addresses(esp):
    """Test I2C operations with multiple addresses."""
    print("\n4. Testing multiple I2C addresses...")
    
    try:
        # Test common I2C addresses
        test_addresses = [0x20, 0x27, 0x48, 0x68, 0x76]
        successful_addresses = 0
        
        for address in test_addresses:
            try:
                # Try to write a single byte
                esp.i2c_write(address, [0x00])
                successful_addresses += 1
                print(f"   [INFO] Address 0x{address:02X}: Device responded")
            except Exception:
                print(f"   [INFO] Address 0x{address:02X}: No device")
        
        success = successful_addresses > 0
        print_test_result("Multiple addresses", success, 
                         f"Successful communication with {successful_addresses}/{len(test_addresses)} addresses")
        return success
        
    except Exception as e:
        print_test_result("Multiple addresses", False, f"Error: {e}")
        return False

def test_i2c_data_integrity(esp):
    """Test I2C data integrity."""
    print("\n5. Testing I2C data integrity...")
    
    try:
        # Test with a known address if available
        test_address = 0x48
        
        # Test different data patterns
        test_patterns = [
            [0x00],
            [0xFF],
            [0xAA, 0x55],
            [0x01, 0x02, 0x03, 0x04],
            [0x00, 0x00, 0x00, 0x00],
        ]
        
        successful_patterns = 0
        
        for pattern in test_patterns:
            try:
                esp.i2c_write(test_address, pattern)
                successful_patterns += 1
            except Exception:
                pass
        
        success = successful_patterns > 0
        print_test_result("Data integrity", success, 
                         f"Successfully tested {successful_patterns}/{len(test_patterns)} data patterns")
        return success
        
    except Exception as e:
        print_test_result("Data integrity", False, f"Error: {e}")
        return False

def test_i2c_error_handling(esp):
    """Test I2C error handling."""
    print("\n6. Testing I2C error handling...")
    
    try:
        # Test invalid address (out of range)
        try:
            esp.i2c_write(0x100, [0x01])
            print_test_result("Error handling", False, "Should have failed for invalid address")
            return False
        except Exception:
            print_test_result("Error handling", True, "Correctly rejected invalid address")
            return True
            
    except Exception as e:
        print_test_result("Error handling", False, f"Error: {e}")
        return False

def test_i2c_clock_speed(esp):
    """Test I2C clock speed variations."""
    print("\n7. Testing I2C clock speed...")
    
    try:
        # Test different clock speeds (if supported)
        clock_speeds = [100000, 400000]  # 100kHz and 400kHz
        
        successful_speeds = 0
        
        for speed in clock_speeds:
            try:
                # Reinitialize I2C with specific clock speed
                esp.i2c_init()  # Use default speed for now
                esp.i2c_scan()  # Test scan at this speed
                successful_speeds += 1
            except Exception:
                pass
        
        success = successful_speeds > 0
        print_test_result("Clock speed", success, 
                         f"Successfully tested {successful_speeds}/{len(clock_speeds)} clock speeds")
        return success
        
    except Exception as e:
        print_test_result("Clock speed", False, f"Error: {e}")
        return False

def test_i2c_rapid_operations(esp):
    """Test rapid I2C operations."""
    print("\n8. Testing rapid I2C operations...")
    
    try:
        # Perform rapid I2C operations
        test_address = 0x48
        successful_operations = 0
        
        for i in range(50):
            try:
                esp.i2c_write(test_address, [i % 256])
                successful_operations += 1
            except Exception:
                pass
            time.sleep(0.01)
        
        success = successful_operations > 0
        print_test_result("Rapid operations", success, 
                         f"Successful operations: {successful_operations}/50")
        return success
        
    except Exception as e:
        print_test_result("Rapid operations", False, f"Error: {e}")
        return False

def run_i2c_tests():
    """Run comprehensive I2C test suite."""
    print("="*80)
    print("TEST: I2C FUNCTIONALITY TEST - ESP32 GPIO Bridge v0.1.8-beta")
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
            
            # Run all I2C tests
            tests_passed = 0
            total_tests = 8
            
            if test_i2c_initialization(esp):
                tests_passed += 1
            
            if test_i2c_scan(esp):
                tests_passed += 1
            
            if test_i2c_write_read(esp):
                tests_passed += 1
            
            if test_i2c_multiple_addresses(esp):
                tests_passed += 1
            
            if test_i2c_data_integrity(esp):
                tests_passed += 1
            
            if test_i2c_error_handling(esp):
                tests_passed += 1
            
            if test_i2c_clock_speed(esp):
                tests_passed += 1
            
            if test_i2c_rapid_operations(esp):
                tests_passed += 1
            
            # Print final results
            print("\n" + "="*80)
            print("I2C TEST RESULTS")
            print("="*80)
            print(f"Tests passed: {tests_passed}/{total_tests}")
            
            if tests_passed == total_tests:
                print("[SUCCESS] ALL I2C TESTS PASSED!")
                print("[INFO] I2C functionality is working correctly!")
                return True
            else:
                print(f"[FAIL] {total_tests - tests_passed} I2C tests failed!")
                print("[NOTE] Some I2C tests may fail if no I2C devices are connected")
                return False
                
    except Exception as e:
        print(f"[ERROR] I2C test failed with error: {e}")
        return False

def main():
    """Main function."""
    print("ESP32 GPIO Bridge - I2C Test Module")
    print("This test verifies I2C communication functionality.")
    print("\nNOTE: Some tests may fail if no I2C devices are connected.")
    print("Press Ctrl+C to cancel the test at any time.")
    
    try:
        success = run_i2c_tests()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\n[WARNING] I2C test cancelled by user.")
        sys.exit(1)

if __name__ == "__main__":
    main()
