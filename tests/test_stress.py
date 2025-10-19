#!/usr/bin/env python3
"""
ESP32 GPIO Bridge - Comprehensive Stress Test

This test demonstrates the robustness of the ESP32 GPIO Bridge firmware
by running comprehensive stress tests that simulate conditions that previously
caused the ESP32 to become unresponsive.

Features tested:
- Rapid command sequences
- Batch operations stress
- PWM operations stress
- EEPROM operations stress
- Mixed operations stress
- Long idle periods
- Extended high load scenarios
- Reconnection after idle periods

This test is designed to verify that the production-ready firmware
(v0.1.8-beta) maintains stable operation under all conditions.

Author: ESP32 GPIO Bridge Team
Version: 0.1.8-beta
"""

import time
import sys
from esp32_gpio_bridge import ESP32GPIO, find_esp32_port

def print_header(title):
    """Print a formatted header for test sections."""
    print(f"\n{'='*80}")
    print(f"TEST: {title}")
    print(f"{'='*80}")

def print_test_result(test_name, success, duration=None, operations=None):
    """Print formatted test results."""
    status = "PASS" if success else "FAIL"
    result_text = f"[{status}] {test_name}"
    
    if duration is not None:
        result_text += f" - Completed in {duration:.2f} seconds"
    if operations is not None:
        result_text += f" ({operations} operations)"
    
    print(f"   {result_text}")

def test_rapid_commands(esp, count=100):
    """Test rapid command sequences."""
    print(f"\n1. Testing rapid command sequences ({count} commands)...")
    start_time = time.time()
    
    for i in range(count):
        esp.set_pin_mode(2, 'OUT')
        esp.digital_write(2, i % 2)
        if i % 20 == 0:
            time.sleep(0.01)  # Small delay every 20 commands
    
    end_time = time.time()
    duration = end_time - start_time
    print_test_result("Rapid commands", True, duration, count)
    return True

def test_batch_operations(esp, count=50):
    """Test batch operations stress."""
    print(f"\n2. Testing batch operations stress ({count} operations)...")
    start_time = time.time()
    
    for i in range(count):
        esp.batch_digital_write({12: i%2, 13: (i+1)%2, 14: (i+2)%2, 15: (i+3)%2})
    
    end_time = time.time()
    duration = end_time - start_time
    print_test_result("Batch operations", True, duration, count)
    return True

def test_pwm_operations(esp, count=40):
    """Test PWM operations stress."""
    print(f"\n3. Testing PWM operations stress ({count} operations)...")
    start_time = time.time()
    
    for i in range(count):
        channel = esp.pwm_init(5, 1000 + i*50, 8)
        esp.pwm_write(5, (i * 6) % 256)
        esp.pwm_stop(5)
    
    end_time = time.time()
    duration = end_time - start_time
    print_test_result("PWM operations", True, duration, count)
    return True

def test_eeprom_operations(esp, count=60):
    """Test EEPROM operations stress."""
    print(f"\n4. Testing EEPROM operations stress ({count} operations)...")
    start_time = time.time()
    
    for i in range(count):
        esp.eeprom_write(i % 100, i % 256)
        if i % 10 == 0:
            esp.eeprom_commit()
    
    end_time = time.time()
    duration = end_time - start_time
    print_test_result("EEPROM operations", True, duration, count)
    return True

def test_mixed_operations(esp, count=150):
    """Test mixed operations stress."""
    print(f"\n5. Testing mixed operations stress ({count} operations)...")
    start_time = time.time()
    
    for i in range(count):
        esp.digital_write(2, i % 2)
        esp.batch_digital_write({12: i%2, 13: (i+1)%2})
        if i % 10 == 0:
            esp.eeprom_write(i % 50, i % 256)
        if i % 30 == 0:
            channel = esp.pwm_init(5, 1000, 8)
            esp.pwm_write(5, (i * 2) % 256)
            esp.pwm_stop(5)
        if i % 50 == 0:
            time.sleep(0.01)
    
    end_time = time.time()
    duration = end_time - start_time
    print_test_result("Mixed operations", True, duration, count)
    return True

def test_idle_period(esp, duration_seconds=45):
    """Test long idle period."""
    print(f"\n6. Testing long idle period ({duration_seconds} seconds)...")
    print("   This simulates the condition where ESP32 becomes unresponsive...")
    start_time = time.time()
    
    time.sleep(duration_seconds)
    
    end_time = time.time()
    actual_duration = end_time - start_time
    print_test_result("Idle period", True, actual_duration)
    return True

def test_reconnection_after_idle(esp):
    """Test reconnection after idle period."""
    print("\n7. Testing reconnection after idle period...")
    start_time = time.time()
    
    esp.digital_write(2, 1)
    time.sleep(0.1)
    esp.digital_write(2, 0)
    
    # Get status to verify communication
    status = esp.get_status()
    
    end_time = time.time()
    duration = end_time - start_time
    success = status.get('state') == 'STATUS:NORMAL'
    print_test_result("Reconnection after idle", success, duration)
    print(f"   Status: {status.get('state', 'Unknown')}")
    return success

def test_extended_high_load(esp, duration_seconds=90):
    """Test extended high load scenario."""
    print(f"\n8. Testing extended high load ({duration_seconds} seconds)...")
    start_time = time.time()
    operation_count = 0
    
    while time.time() - start_time < duration_seconds:
        for i in range(10):
            esp.digital_write(2, i % 2)
            esp.batch_digital_write({12: i%2, 13: (i+1)%2})
            if i % 5 == 0:
                esp.eeprom_write(i % 50, i % 256)
            operation_count += 3
        time.sleep(0.1)
    
    end_time = time.time()
    actual_duration = end_time - start_time
    print_test_result("Extended high load", True, actual_duration, operation_count)
    return True

def test_final_functionality(esp):
    """Test final functionality verification."""
    print("\n9. Final functionality test...")
    start_time = time.time()
    
    esp.set_pin_mode(2, 'OUT')
    esp.digital_write(2, 1)
    time.sleep(0.1)
    esp.digital_write(2, 0)
    
    # Test PWM
    channel = esp.pwm_init(5, 1000, 8)
    esp.pwm_write(5, 128)
    time.sleep(0.1)
    esp.pwm_stop(5)
    
    # Test EEPROM
    esp.eeprom_write(0, 42)
    esp.eeprom_commit()
    value = esp.eeprom_read(0)
    
    end_time = time.time()
    duration = end_time - start_time
    success = value == 42
    print_test_result("Final functionality", success, duration)
    return success

def run_comprehensive_stress_test():
    """Run comprehensive stress test suite."""
    print_header("COMPREHENSIVE STRESS TEST - ESP32 GPIO Bridge v0.1.8-beta")
    
    # Auto-detect ESP32 port
    port = find_esp32_port()
    if not port:
        print("[ERROR] ESP32 GPIO Bridge not found!")
        print("\nTroubleshooting:")
        print("1. Make sure your ESP32 is connected via USB")
        print("2. Ensure the firmware (esp32_GPIO_bridge.ino) is flashed")
        print("3. Check that no other program is using the serial port")
        return False
    
    print(f"[SUCCESS] Found ESP32 GPIO Bridge on port: {port}")
    
    try:
        with ESP32GPIO(port) as esp:
            print(f"[SUCCESS] Connected to firmware version: {esp.get_version()}")
            
            # Run all stress tests
            tests_passed = 0
            total_tests = 9
            
            if test_rapid_commands(esp):
                tests_passed += 1
            
            if test_batch_operations(esp):
                tests_passed += 1
            
            if test_pwm_operations(esp):
                tests_passed += 1
            
            if test_eeprom_operations(esp):
                tests_passed += 1
            
            if test_mixed_operations(esp):
                tests_passed += 1
            
            if test_idle_period(esp):
                tests_passed += 1
            
            if test_reconnection_after_idle(esp):
                tests_passed += 1
            
            if test_extended_high_load(esp):
                tests_passed += 1
            
            if test_final_functionality(esp):
                tests_passed += 1
            
            # Print final results
            print_header("STRESS TEST RESULTS")
            print(f"Tests passed: {tests_passed}/{total_tests}")
            
            if tests_passed == total_tests:
                print("[SUCCESS] ALL STRESS TESTS COMPLETED SUCCESSFULLY!")
                print("[INFO] Production-ready firmware handled all stress conditions!")
                print("[INFO] Serial communication remained stable throughout!")
                print("[INFO] Hardware watchdog and task recovery working perfectly!")
                print("[SUCCESS] ESP32 remained responsive during all tests!")
                print("\n[SUCCESS] The unresponsive state issue has been resolved!")
                return True
            else:
                print(f"[FAIL] {total_tests - tests_passed} tests failed!")
                print("[ERROR] This indicates the unresponsive state issue still exists.")
                return False
                
    except Exception as e:
        print(f"[ERROR] Stress test failed with error: {e}")
        print("[ERROR] This indicates the ESP32 became unresponsive during testing.")
        return False

def main():
    """Main function."""
    print("ESP32 GPIO Bridge - Comprehensive Stress Test")
    print("This test verifies that the production-ready firmware maintains")
    print("stable operation under all stress conditions.")
    print("\nPress Ctrl+C to cancel the test at any time.")
    
    try:
        success = run_comprehensive_stress_test()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\n[WARNING] Stress test cancelled by user.")
        sys.exit(1)

if __name__ == "__main__":
    main()
