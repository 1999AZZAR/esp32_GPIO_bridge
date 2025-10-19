#!/usr/bin/env python3
"""
ESP32 GPIO Bridge - Connection Test Module

This module provides comprehensive testing for connection and communication functionality including:
- Serial port detection
- Connection establishment
- Firmware version verification
- Status monitoring
- Communication reliability
- Error recovery
- Idle period handling

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

def test_port_detection():
    """Test ESP32 port detection."""
    print("\n1. Testing port detection...")
    
    try:
        port = find_esp32_port()
        
        if port:
            print_test_result("Port detection", True, f"Found ESP32 on {port}")
            return True, port
        else:
            print_test_result("Port detection", False, "ESP32 not found")
            return False, None
            
    except Exception as e:
        print_test_result("Port detection", False, f"Error: {e}")
        return False, None

def test_connection_establishment(port):
    """Test connection establishment."""
    print("\n2. Testing connection establishment...")
    
    try:
        with ESP32GPIO(port) as esp:
            print_test_result("Connection establishment", True, "Successfully connected")
            return True
            
    except Exception as e:
        print_test_result("Connection establishment", False, f"Error: {e}")
        return False

def test_firmware_version(esp):
    """Test firmware version retrieval."""
    print("\n3. Testing firmware version...")
    
    try:
        version = esp.get_version()
        
        if version and "0.1.8-beta" in version:
            print_test_result("Firmware version", True, f"Version: {version}")
            return True
        else:
            print_test_result("Firmware version", False, f"Unexpected version: {version}")
            return False
            
    except Exception as e:
        print_test_result("Firmware version", False, f"Error: {e}")
        return False

def test_device_identification(esp):
    """Test device identification."""
    print("\n4. Testing device identification...")
    
    try:
        identity = esp.get_identity()
        
        if identity and "ESP32 GPIO Bridge" in identity:
            print_test_result("Device identification", True, f"Identity: {identity}")
            return True
        else:
            print_test_result("Device identification", False, f"Unexpected identity: {identity}")
            return False
            
    except Exception as e:
        print_test_result("Device identification", False, f"Error: {e}")
        return False

def test_status_monitoring(esp):
    """Test status monitoring."""
    print("\n5. Testing status monitoring...")
    
    try:
        status = esp.get_status()
        
        if status and isinstance(status, dict):
            state = status.get('state', 'Unknown')
            print_test_result("Status monitoring", True, f"Status: {state}")
            return True
        else:
            print_test_result("Status monitoring", False, f"Invalid status: {status}")
            return False
            
    except Exception as e:
        print_test_result("Status monitoring", False, f"Error: {e}")
        return False

def test_ping_functionality(esp):
    """Test ping functionality."""
    print("\n6. Testing ping functionality...")
    
    try:
        response = esp.ping()
        
        if response == "PONG":
            print_test_result("Ping functionality", True, "Received PONG response")
            return True
        else:
            print_test_result("Ping functionality", False, f"Unexpected response: {response}")
            return False
            
    except Exception as e:
        print_test_result("Ping functionality", False, f"Error: {e}")
        return False

def test_communication_reliability(esp):
    """Test communication reliability."""
    print("\n7. Testing communication reliability...")
    
    try:
        success_count = 0
        total_tests = 20
        
        for i in range(total_tests):
            try:
                response = esp.ping()
                if response == "PONG":
                    success_count += 1
                time.sleep(0.1)
            except Exception:
                pass
        
        success = success_count >= total_tests * 0.9  # 90% success rate
        print_test_result("Communication reliability", success, 
                         f"Success rate: {success_count}/{total_tests} ({success_count/total_tests*100:.1f}%)")
        return success
        
    except Exception as e:
        print_test_result("Communication reliability", False, f"Error: {e}")
        return False

def test_idle_period_handling(esp):
    """Test idle period handling."""
    print("\n8. Testing idle period handling...")
    
    try:
        # Test communication before idle
        response1 = esp.ping()
        
        # Wait for idle period
        print("   Waiting 15 seconds to test idle period handling...")
        time.sleep(15)
        
        # Test communication after idle
        response2 = esp.ping()
        
        success = response1 == "PONG" and response2 == "PONG"
        print_test_result("Idle period handling", success, 
                         f"Before idle: {response1}, After idle: {response2}")
        return success
        
    except Exception as e:
        print_test_result("Idle period handling", False, f"Error: {e}")
        return False

def test_error_recovery(esp):
    """Test error recovery mechanisms."""
    print("\n9. Testing error recovery...")
    
    try:
        # Test normal operation
        response1 = esp.ping()
        
        # Simulate potential error condition by sending invalid command
        try:
            # This should not crash the connection
            esp._send_command("INVALID_COMMAND", expect_response=False)
        except Exception:
            pass
        
        # Test recovery
        time.sleep(0.5)
        response2 = esp.ping()
        
        success = response1 == "PONG" and response2 == "PONG"
        print_test_result("Error recovery", success, 
                         f"Before error: {response1}, After error: {response2}")
        return success
        
    except Exception as e:
        print_test_result("Error recovery", False, f"Error: {e}")
        return False

def test_connection_stability(esp):
    """Test connection stability over time."""
    print("\n10. Testing connection stability...")
    
    try:
        start_time = time.time()
        success_count = 0
        total_tests = 30
        
        for i in range(total_tests):
            try:
                response = esp.ping()
                if response == "PONG":
                    success_count += 1
                time.sleep(0.2)
            except Exception:
                pass
        
        duration = time.time() - start_time
        success = success_count >= total_tests * 0.95  # 95% success rate
        
        print_test_result("Connection stability", success, 
                         f"Success rate: {success_count}/{total_tests} over {duration:.1f}s")
        return success
        
    except Exception as e:
        print_test_result("Connection stability", False, f"Error: {e}")
        return False

def run_connection_tests():
    """Run comprehensive connection test suite."""
    print("="*80)
    print("TEST: CONNECTION AND COMMUNICATION TEST - ESP32 GPIO Bridge v0.1.8-beta")
    print("="*80)
    
    # Test port detection first
    port_detected, port = test_port_detection()
    if not port_detected:
        print("\n[ERROR] Cannot proceed with connection tests - ESP32 not found!")
        return False
    
    try:
        with ESP32GPIO(port) as esp:
            print(f"[SUCCESS] Connected to ESP32 on port: {port}")
            
            # Run all connection tests
            tests_passed = 0
            total_tests = 9
            
            if test_connection_establishment(port):
                tests_passed += 1
            
            if test_firmware_version(esp):
                tests_passed += 1
            
            if test_device_identification(esp):
                tests_passed += 1
            
            if test_status_monitoring(esp):
                tests_passed += 1
            
            if test_ping_functionality(esp):
                tests_passed += 1
            
            if test_communication_reliability(esp):
                tests_passed += 1
            
            if test_idle_period_handling(esp):
                tests_passed += 1
            
            if test_error_recovery(esp):
                tests_passed += 1
            
            if test_connection_stability(esp):
                tests_passed += 1
            
            # Print final results
            print("\n" + "="*80)
            print("CONNECTION TEST RESULTS")
            print("="*80)
            print(f"Tests passed: {tests_passed}/{total_tests}")
            
            if tests_passed == total_tests:
                print("[SUCCESS] ALL CONNECTION TESTS PASSED!")
                print("[INFO] Connection and communication are working correctly!")
                return True
            else:
                print(f"[FAIL] {total_tests - tests_passed} connection tests failed!")
                return False
                
    except Exception as e:
        print(f"[ERROR] Connection test failed with error: {e}")
        return False

def main():
    """Main function."""
    print("ESP32 GPIO Bridge - Connection Test Module")
    print("This test verifies connection and communication functionality.")
    print("\nPress Ctrl+C to cancel the test at any time.")
    
    try:
        success = run_connection_tests()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\n[WARNING] Connection test cancelled by user.")
        sys.exit(1)

if __name__ == "__main__":
    main()
