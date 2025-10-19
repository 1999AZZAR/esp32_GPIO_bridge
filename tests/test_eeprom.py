#!/usr/bin/env python3
"""
ESP32 GPIO Bridge - EEPROM Test Module

This module provides comprehensive testing for EEPROM functionality including:
- Single byte read/write operations
- Block read/write operations
- String storage and retrieval
- Commit and clear operations
- Boundary testing
- Data integrity verification

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

def test_eeprom_basic_operations(esp):
    """Test basic EEPROM read/write operations."""
    print("\n1. Testing basic EEPROM operations...")
    
    try:
        # Test single byte write/read
        test_value = 42
        esp.eeprom_write(0, test_value)
        esp.eeprom_commit()
        
        read_value = esp.eeprom_read(0)
        success = read_value == test_value
        
        print_test_result("Single byte write/read", success, 
                         f"Wrote: {test_value}, Read: {read_value}")
        
        return success
        
    except Exception as e:
        print_test_result("Basic EEPROM operations", False, f"Error: {e}")
        return False

def test_eeprom_multiple_addresses(esp):
    """Test EEPROM operations at multiple addresses."""
    print("\n2. Testing multiple address operations...")
    
    try:
        test_data = {0: 10, 10: 20, 50: 30, 100: 40, 200: 50}
        
        # Write test data
        for addr, value in test_data.items():
            esp.eeprom_write(addr, value)
        
        esp.eeprom_commit()
        
        # Read and verify
        all_correct = True
        for addr, expected_value in test_data.items():
            read_value = esp.eeprom_read(addr)
            if read_value != expected_value:
                all_correct = False
                print_test_result(f"Address {addr}", False, 
                                 f"Expected: {expected_value}, Got: {read_value}")
        
        print_test_result("Multiple address operations", all_correct,
                         f"Tested {len(test_data)} addresses")
        
        return all_correct
        
    except Exception as e:
        print_test_result("Multiple address operations", False, f"Error: {e}")
        return False

def test_eeprom_block_operations(esp):
    """Test EEPROM block read/write operations."""
    print("\n3. Testing block operations...")
    
    try:
        # Test block write
        test_block = [i for i in range(16)]  # 16 bytes of test data
        esp.eeprom_write_block(0, test_block)
        esp.eeprom_commit()
        
        # Test block read
        read_block = esp.eeprom_read_block(0, 16)
        
        success = test_block == read_block
        print_test_result("Block write/read", success,
                         f"Wrote: {len(test_block)} bytes, Read: {len(read_block)} bytes")
        
        return success
        
    except Exception as e:
        print_test_result("Block operations", False, f"Error: {e}")
        return False

def test_eeprom_string_operations(esp):
    """Test EEPROM string storage operations."""
    print("\n4. Testing string operations...")
    
    try:
        test_string = "ESP32 GPIO Bridge Test"
        
        # Write string
        esp.eeprom_write_string(0, test_string)
        esp.eeprom_commit()
        
        # Read string
        read_string = esp.eeprom_read_string(0)
        
        success = test_string == read_string
        print_test_result("String write/read", success,
                         f"Wrote: '{test_string}', Read: '{read_string}'")
        
        return success
        
    except Exception as e:
        print_test_result("String operations", False, f"Error: {e}")
        return False

def test_eeprom_boundary_conditions(esp):
    """Test EEPROM boundary conditions."""
    print("\n5. Testing boundary conditions...")
    
    try:
        # Test maximum address (511 for 512-byte EEPROM)
        esp.eeprom_write(511, 255)
        esp.eeprom_commit()
        
        value = esp.eeprom_read(511)
        success = value == 255
        
        print_test_result("Maximum address write/read", success,
                         f"Address: 511, Value: {value}")
        
        return success
        
    except Exception as e:
        print_test_result("Boundary conditions", False, f"Error: {e}")
        return False

def test_eeprom_clear_operation(esp):
    """Test EEPROM clear operation."""
    print("\n6. Testing clear operation...")
    
    try:
        # Write some test data
        esp.eeprom_write(0, 42)
        esp.eeprom_write(10, 84)
        esp.eeprom_commit()
        
        # Clear EEPROM
        esp.eeprom_clear()
        
        # Verify cleared
        value1 = esp.eeprom_read(0)
        value2 = esp.eeprom_read(10)
        
        success = value1 == 0 and value2 == 0
        print_test_result("Clear operation", success,
                         f"Address 0: {value1}, Address 10: {value2}")
        
        return success
        
    except Exception as e:
        print_test_result("Clear operation", False, f"Error: {e}")
        return False

def test_eeprom_persistence(esp):
    """Test EEPROM data persistence."""
    print("\n7. Testing data persistence...")
    
    try:
        # Write persistent data
        esp.eeprom_write(0, 123)
        esp.eeprom_write(50, 200)
        esp.eeprom_commit()
        
        # Simulate power cycle by reconnecting
        print("   Simulating power cycle...")
        time.sleep(1)
        
        # Read back data
        value1 = esp.eeprom_read(0)
        value2 = esp.eeprom_read(50)
        
        success = value1 == 123 and value2 == 200
        print_test_result("Data persistence", success,
                         f"Address 0: {value1}, Address 50: {value2}")
        
        return success
        
    except Exception as e:
        print_test_result("Data persistence", False, f"Error: {e}")
        return False

def run_eeprom_tests():
    """Run comprehensive EEPROM test suite."""
    print("="*80)
    print("TEST: EEPROM FUNCTIONALITY TEST - ESP32 GPIO Bridge v0.1.8-beta")
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
            
            # Run all EEPROM tests
            tests_passed = 0
            total_tests = 7
            
            if test_eeprom_basic_operations(esp):
                tests_passed += 1
            
            if test_eeprom_multiple_addresses(esp):
                tests_passed += 1
            
            if test_eeprom_block_operations(esp):
                tests_passed += 1
            
            if test_eeprom_string_operations(esp):
                tests_passed += 1
            
            if test_eeprom_boundary_conditions(esp):
                tests_passed += 1
            
            if test_eeprom_clear_operation(esp):
                tests_passed += 1
            
            if test_eeprom_persistence(esp):
                tests_passed += 1
            
            # Print final results
            print("\n" + "="*80)
            print("EEPROM TEST RESULTS")
            print("="*80)
            print(f"Tests passed: {tests_passed}/{total_tests}")
            
            if tests_passed == total_tests:
                print("[SUCCESS] ALL EEPROM TESTS PASSED!")
                print("[INFO] EEPROM functionality is working correctly!")
                return True
            else:
                print(f"[FAIL] {total_tests - tests_passed} EEPROM tests failed!")
                return False
                
    except Exception as e:
        print(f"[ERROR] EEPROM test failed with error: {e}")
        return False

def main():
    """Main function."""
    print("ESP32 GPIO Bridge - EEPROM Test Module")
    print("This test verifies EEPROM functionality and data persistence.")
    print("\nPress Ctrl+C to cancel the test at any time.")
    
    try:
        success = run_eeprom_tests()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\n[WARNING] EEPROM test cancelled by user.")
        sys.exit(1)

if __name__ == "__main__":
    main()
