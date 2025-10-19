# ESP32 GPIO Bridge - Test Suite

This directory contains comprehensive test modules for the ESP32 GPIO Bridge v0.1.8-beta. The test suite provides automated verification of all functionality and ensures production-ready reliability.

## Test Modules

### Individual Test Modules

- **`test_connection.py`** - Connection and communication tests
  - Serial port detection
  - Connection establishment
  - Firmware version verification
  - Status monitoring
  - Communication reliability
  - Idle period handling
  - Error recovery

- **`test_gpio.py`** - GPIO functionality tests
  - Digital input/output operations
  - Pin mode configuration
  - Batch operations
  - Input pull-up/pull-down testing
  - Pin capability verification
  - Error handling
  - Rapid switching

- **`test_analog.py`** - Analog I/O tests
  - Analog input (ADC) operations
  - Analog output (DAC) operations
  - Voltage conversion and calibration
  - Range testing
  - Multiple channel operations
  - Resolution testing

- **`test_pwm.py`** - PWM functionality tests
  - PWM channel initialization
  - Duty cycle control
  - Frequency control
  - Resolution control
  - Channel allocation/deallocation
  - Multiple channel operations
  - Error handling

- **`test_eeprom.py`** - EEPROM functionality tests
  - Single byte read/write operations
  - Block read/write operations
  - String storage and retrieval
  - Commit and clear operations
  - Boundary testing
  - Data integrity verification

- **`test_i2c.py`** - I2C functionality tests
  - I2C bus initialization
  - Device scanning
  - I2C read/write operations
  - Multiple device communication
  - Error handling
  - Clock speed testing

### Stress Test Module

- **`test_stress.py`** - Comprehensive stress tests
  - Rapid command sequences
  - Batch operations stress
  - PWM operations stress
  - EEPROM operations stress
  - Mixed operations stress
  - Long idle periods
  - Extended high load scenarios
  - Reconnection after idle periods

### Test Runner

- **`run_all_tests.py`** - Comprehensive test runner
  - Run individual test modules
  - Run complete test suite
  - Run stress tests only
  - Run quick tests (connection + GPIO only)
  - Generate detailed test reports

## Usage

### Running Individual Tests

```bash
# Run connection tests
python test_connection.py

# Run GPIO tests
python test_gpio.py

# Run analog I/O tests
python test_analog.py

# Run PWM tests
python test_pwm.py

# Run EEPROM tests
python test_eeprom.py

# Run I2C tests
python test_i2c.py

# Run stress tests
python test_stress.py
```

### Using the Test Runner

```bash
# Run complete test suite (all tests)
python run_all_tests.py --mode complete

# Run individual tests only
python run_all_tests.py --mode individual

# Run stress tests only
python run_all_tests.py --mode stress

# Run quick tests (connection + GPIO only)
python run_all_tests.py --mode quick

# List available test modules
python run_all_tests.py --list
```

### Quick Test Verification

For rapid verification of basic functionality:

```bash
python run_all_tests.py --mode quick
```

This runs only connection and GPIO tests, taking approximately 1-2 minutes.

## Hardware Requirements

### Basic Testing
- ESP32 development board
- USB cable for connection
- Computer with Python 3.7+

### GPIO Testing
- LED connected to GPIO pin 2 (for output testing)
- Button or jumper wire for GPIO pin 0 (for input testing)
- Optional: Multiple LEDs on GPIO pins 12, 13, 14, 15 (for batch operations)

### Analog Testing
- Potentiometer connected to GPIO pin 34 (for ADC testing)
- Optional: External voltage source for testing
- Optional: Wire connection between DAC pin 25 and ADC pin 34 (for loopback testing)

### PWM Testing
- LED connected to GPIO pin 5 (for PWM output testing)
- Optional: Servo motor connected to GPIO pin 5 (for servo testing)
- Optional: Multiple LEDs on GPIO pins 5, 18, 19, 21 (for multi-channel testing)

### EEPROM Testing
- No additional hardware required (internal EEPROM testing)

## Test Results

All tests provide detailed output with:
- Test execution status (PASS/FAIL)
- Performance metrics (duration, operations count)
- Error details and troubleshooting information
- Summary statistics

### Expected Results

With properly functioning ESP32 GPIO Bridge firmware v0.1.8-beta:
- **Individual Tests**: All tests should pass
- **Stress Tests**: All stress conditions should be handled correctly
- **Complete Suite**: 100% pass rate expected

### Troubleshooting

If tests fail:

1. **Connection Issues**:
   - Verify ESP32 is connected via USB
   - Check that firmware is flashed correctly
   - Ensure no other programs are using the serial port

2. **GPIO Test Failures**:
   - Verify LED connections for output tests
   - Check button/jumper connections for input tests
   - Ensure proper pin assignments

3. **Analog Test Failures**:
   - Verify potentiometer connections
   - Check voltage levels are within expected ranges
   - Ensure proper ADC pin connections

4. **PWM Test Failures**:
   - Verify LED or motor connections
   - Check PWM pin assignments
   - Ensure proper power supply

5. **EEPROM Test Failures**:
   - Usually indicates firmware issues
   - Verify firmware version is v0.1.8-beta
   - Check for memory corruption

## Integration with CI/CD

The test suite is designed for integration with continuous integration systems:

```bash
# Exit code 0 = all tests passed
# Exit code 1 = some tests failed
python run_all_tests.py --mode complete
```

## Test Coverage

The test suite provides comprehensive coverage of:
- ✅ Connection and communication reliability
- ✅ GPIO input/output functionality
- ✅ Analog input/output operations
- ✅ PWM generation and control
- ✅ EEPROM storage and retrieval
- ✅ Error handling and recovery
- ✅ Stress testing under extreme conditions
- ✅ Long-term stability verification

## Version Information

- **Test Suite Version**: 0.1.8-beta
- **Target Firmware Version**: 0.1.8-beta
- **Python Requirements**: 3.7+
- **Dependencies**: esp32_gpio_bridge library

## Support

For issues with the test suite:
1. Check that all hardware connections are correct
2. Verify firmware version matches test requirements
3. Review test output for specific error messages
4. Ensure ESP32 GPIO Bridge library is properly installed
