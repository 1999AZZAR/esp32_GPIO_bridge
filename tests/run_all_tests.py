#!/usr/bin/env python3
"""
ESP32 GPIO Bridge - Comprehensive Test Runner

This module runs all individual test modules and provides a comprehensive test suite
for the ESP32 GPIO Bridge. It can run tests individually or as a complete suite.

Available test modules:
- test_connection.py: Connection and communication tests
- test_gpio.py: GPIO functionality tests
- test_analog.py: Analog I/O tests
- test_pwm.py: PWM functionality tests
- test_eeprom.py: EEPROM functionality tests
- test_stress.py: Comprehensive stress tests

Author: ESP32 GPIO Bridge Team
Version: 0.1.8-beta
"""

import sys
import os
import time
import argparse
from datetime import datetime

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def print_header(title):
    """Print a formatted header."""
    print("\n" + "="*80)
    print(f"TEST SUITE: {title}")
    print("="*80)

def print_section(title):
    """Print a formatted section header."""
    print(f"\n--- {title} ---")

def run_test_module(module_name, test_function):
    """Run a single test module."""
    print_section(f"Running {module_name}")
    
    start_time = time.time()
    try:
        success = test_function()
        end_time = time.time()
        duration = end_time - start_time
        
        status = "[PASS]" if success else "[FAIL]"
        print(f"\n{status} {module_name} completed in {duration:.2f} seconds")
        return success, duration
        
    except Exception as e:
        end_time = time.time()
        duration = end_time - start_time
        print(f"\n[ERROR] {module_name} failed with error: {e}")
        return False, duration

def run_individual_tests():
    """Run all individual test modules."""
    print_header("INDIVIDUAL TEST MODULES")
    
    # Import test modules
    try:
        from test_connection import run_connection_tests
        from test_gpio import run_gpio_tests
        from test_analog import run_analog_tests
        from test_pwm import run_pwm_tests
        from test_eeprom import run_eeprom_tests
        from test_i2c import run_i2c_tests
    except ImportError as e:
        print(f"[ERROR] Failed to import test modules: {e}")
        return False
    
    # Define test modules
    test_modules = [
        ("Connection Tests", run_connection_tests),
        ("GPIO Tests", run_gpio_tests),
        ("Analog I/O Tests", run_analog_tests),
        ("PWM Tests", run_pwm_tests),
        ("EEPROM Tests", run_eeprom_tests),
        ("I2C Tests", run_i2c_tests),
    ]
    
    # Run all test modules
    results = []
    total_start_time = time.time()
    
    for module_name, test_function in test_modules:
        success, duration = run_test_module(module_name, test_function)
        results.append((module_name, success, duration))
    
    total_end_time = time.time()
    total_duration = total_end_time - total_start_time
    
    # Print summary
    print_header("INDIVIDUAL TESTS SUMMARY")
    
    passed_tests = 0
    total_tests = len(results)
    
    for module_name, success, duration in results:
        status = "[PASS]" if success else "[FAIL]"
        print(f"{status} {module_name:<20} - {duration:.2f}s")
        if success:
            passed_tests += 1
    
    print(f"\nIndividual Tests: {passed_tests}/{total_tests} passed")
    print(f"Total Duration: {total_duration:.2f} seconds")
    
    return passed_tests == total_tests

def run_stress_tests():
    """Run stress tests."""
    print_header("STRESS TESTS")
    
    try:
        from test_stress import run_comprehensive_stress_test
        success, duration = run_test_module("Stress Tests", run_comprehensive_stress_test)
        return success
    except ImportError as e:
        print(f"[ERROR] Failed to import stress test module: {e}")
        return False

def run_quick_tests():
    """Run quick tests (connection and basic functionality only)."""
    print_header("QUICK TESTS")
    
    try:
        from test_connection import run_connection_tests
        from test_gpio import run_gpio_tests
        
        test_modules = [
            ("Connection Tests", run_connection_tests),
            ("GPIO Tests", run_gpio_tests),
        ]
        
        results = []
        total_start_time = time.time()
        
        for module_name, test_function in test_modules:
            success, duration = run_test_module(module_name, test_function)
            results.append((module_name, success, duration))
        
        total_end_time = time.time()
        total_duration = total_end_time - total_start_time
        
        # Print summary
        print_header("QUICK TESTS SUMMARY")
        
        passed_tests = 0
        total_tests = len(results)
        
        for module_name, success, duration in results:
            status = "[PASS]" if success else "[FAIL]"
            print(f"{status} {module_name:<20} - {duration:.2f}s")
            if success:
                passed_tests += 1
        
        print(f"\nQuick Tests: {passed_tests}/{total_tests} passed")
        print(f"Total Duration: {total_duration:.2f} seconds")
        
        return passed_tests == total_tests
        
    except ImportError as e:
        print(f"[ERROR] Failed to import test modules: {e}")
        return False

def run_complete_test_suite():
    """Run the complete test suite."""
    print_header("COMPLETE TEST SUITE - ESP32 GPIO Bridge v0.1.8-beta")
    print(f"Test started at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    # Run individual tests
    individual_success = run_individual_tests()
    
    if not individual_success:
        print("\n[WARNING] Some individual tests failed. Skipping stress tests.")
        return False
    
    # Run stress tests
    stress_success = run_stress_tests()
    
    # Final summary
    print_header("COMPLETE TEST SUITE RESULTS")
    
    if individual_success and stress_success:
        print("[SUCCESS] ALL TESTS PASSED!")
        print("[INFO] ESP32 GPIO Bridge is working correctly!")
        print("[INFO] All functionality verified and stress tested!")
        return True
    else:
        print("[FAIL] Some tests failed!")
        if not individual_success:
            print("[ERROR] Individual tests failed")
        if not stress_success:
            print("[ERROR] Stress tests failed")
        return False

def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="ESP32 GPIO Bridge Test Runner")
    parser.add_argument("--mode", choices=["complete", "individual", "stress", "quick"], 
                       default="complete", help="Test mode to run")
    parser.add_argument("--list", action="store_true", help="List available test modules")
    
    args = parser.parse_args()
    
    if args.list:
        print("Available test modules:")
        print("- test_connection.py: Connection and communication tests")
        print("- test_gpio.py: GPIO functionality tests")
        print("- test_analog.py: Analog I/O tests")
        print("- test_pwm.py: PWM functionality tests")
        print("- test_eeprom.py: EEPROM functionality tests")
        print("- test_i2c.py: I2C functionality tests")
        print("- test_stress.py: Comprehensive stress tests")
        return
    
    print("ESP32 GPIO Bridge - Comprehensive Test Runner")
    print("This tool runs comprehensive tests to verify ESP32 GPIO Bridge functionality.")
    print(f"Test mode: {args.mode}")
    print("\nPress Ctrl+C to cancel the tests at any time.")
    
    try:
        if args.mode == "complete":
            success = run_complete_test_suite()
        elif args.mode == "individual":
            success = run_individual_tests()
        elif args.mode == "stress":
            success = run_stress_tests()
        elif args.mode == "quick":
            success = run_quick_tests()
        
        sys.exit(0 if success else 1)
        
    except KeyboardInterrupt:
        print("\n\n[WARNING] Tests cancelled by user.")
        sys.exit(1)
    except Exception as e:
        print(f"\n[ERROR] Test runner failed with error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
