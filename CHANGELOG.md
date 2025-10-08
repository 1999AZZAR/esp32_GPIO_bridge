# Changelog

All notable changes to the ESP32 GPIO Bridge project will be documented in this file.

## [0.1.3-beta] - 2025-10-08

### Major Features Added

#### PWM Control
- Hardware PWM support on up to 16 channels simultaneously
- Configurable frequency (1-40000 Hz) and resolution (1-16 bits)
- Automatic channel allocation and management
- Percentage-based duty cycle control for easier usage
- `pwm_init()`, `pwm_write()`, `pwm_set_duty_percent()`, and `pwm_stop()` methods

#### EEPROM Storage
- 512 bytes of persistent storage for configuration and data
- Single byte read/write operations
- Block read/write operations for efficient data transfer
- String storage utilities with automatic encoding/decoding
- Commit operation to persist changes to flash memory
- Clear operation to reset all EEPROM data
- Methods: `eeprom_read()`, `eeprom_write()`, `eeprom_read_block()`, `eeprom_write_block()`, 
  `eeprom_commit()`, `eeprom_clear()`, `eeprom_write_string()`, `eeprom_read_string()`

#### Batch Operations
- Efficient multi-pin GPIO control in a single serial command
- Significantly faster than individual pin operations (measured 4-10x speedup)
- `batch_digital_write()` method accepting dictionary of pin/value pairs

### Performance Optimizations
- **WiFi disabled by default** - Saves significant memory and processing power
- **Bluetooth disabled by default** - Frees up additional resources for GPIO operations
- More GPIO pins available for peripheral usage
- Lower power consumption

### Firmware Enhancements
- **Added IDENTIFY command** - Returns unique device signature for reliable auto-detection
- Enhanced error handling and input validation
- Robust parameter checking for all commands
- Comprehensive range validation
- Clear error messages for debugging

### Library Improvements
- **Completely rewritten auto-detection** - Actively probes serial ports instead of relying on USB descriptions
- Auto-detection now sends IDENTIFY/VERSION commands to verify ESP32 GPIO Bridge firmware
- Faster and more reliable port detection (0.5s timeout per port)
- Better error messages in examples with troubleshooting steps
- Support for manual port selection as fallback
- Fixed missing imports in example files

### Failsafe Improvements
- **Smart failsafe activation** - Only engages after output commands (MODE OUT, WRITE, PWM, etc.)
- Query commands (IDENTIFY, VERSION, READ, STATUS, EEPROM) don't trigger failsafe
- Increased timeout from 5 seconds to 30 seconds (10s warning + 20s grace)
- Allows auto-detection and monitoring without requiring ESP32 reset
- More developer-friendly for testing and development

### Connection Reliability Improvements
- **Fixed DTR/RTS auto-reset timing issues** - No more manual reset button timing needed
- Disabled DTR/RTS lines on serial port open to prevent unwanted ESP32 resets
- Added boot message draining (1 second buffer clear) after port open
- Increased auto-detection timeout from 0.5s to 2s for slower ESP32 boots
- Added retry logic with PING commands (3 attempts) for robust connection
- Connection now works reliably without user intervention
- Proper handling of ESP32 bootloader messages
- **Fixed response queue contamination** - Enhanced filtering system:
  - Filters out PONG responses from ping thread
  - Filters out STATUS responses unless explicitly requested
  - Filters out stale ERROR messages from previous commands
  - Up to 10 retry attempts to find valid responses
  - Commands now correctly wait for actual responses, not background noise
- Firmware version displays correctly (not "PONG" anymore)

### Bug Fixes
- **Added missing I2C methods** - i2c_init(), i2c_scan(), i2c_read(), i2c_write()
- Fixed PWM commands receiving STATUS responses instead of channel numbers
- **Fixed ALL write commands expecting responses** - 11 total commands now correctly use `expect_response=False`:
  - GPIO: set_pin_mode(), digital_write(), analog_write()
  - PWM: pwm_write(), pwm_stop()
  - EEPROM: eeprom_write(), eeprom_write_block(), eeprom_commit(), eeprom_clear()
  - I2C: i2c_init(), i2c_write()
  - Batch: batch_digital_write()
- This eliminates response queue contamination and timeout waits on write operations
- Breathing pattern in LED example now works correctly
- sensor_hub_example.py now works correctly
- advanced_features_example.py now works correctly
- led_patterns_example.py now works correctly
- eeprom_config_example.py now works correctly

### New Examples
- **advanced_features_example.py** - PWM, EEPROM, batch operations, performance comparison
- **pwm_servo_control.py** - Servo motor control with smooth transitions
- **led_patterns_example.py** - Creative LED effects (Knight Rider, binary counter, breathing, etc.)
- **eeprom_config_example.py** - Persistent configuration storage with JSON support
- **dac_waveform_generator.py** - Audio waveform generation and musical notes
- **multi_sensor_dashboard.py** - Real-time multi-sensor monitoring with statistics
- **ultrasonic_distance_meter.py** - HC-SR04 distance measurement with visual feedback
- **digital_thermometer_logger.py** - Temperature monitoring with CSV export

Total: 10 example scripts (3 original + 7 new)

### Documentation
- Added comprehensive API documentation for new features
- Updated README with all new examples
- Updated examples/README.md with detailed hardware setup and features
- Comprehensive CHANGELOG with all improvements and fixes
- Enhanced examples README with hardware setup instructions
- Added command protocol reference for new commands

### Notes
- WiFi and Bluetooth are now disabled by default (can be re-enabled by modifying firmware)
- This is a beta release - some features may require additional testing

## [0.1.2-beta] - Previous Release

### Added
- Comprehensive .gitignore file for Python/ESP32 development
- GitHub Actions CI/CD pipeline with multi-Python testing
- Complete test suite with 16 comprehensive tests
- Enhanced documentation with CI/CD and testing guidelines

### Improved
- Pin capability mapping accuracy for ESP32 hardware

### Fixed
- Touch sensor pin capability detection

## [0.1.1-beta] - Previous Release

### Refactored
- Complete library restructuring with proper package organization

### Added
- Advanced pin management system with capability detection
- Configuration management with presets for common applications
- Context manager support for automatic resource cleanup
- Comprehensive error handling and validation
- Type hints throughout the codebase

### Improved
- Documentation with examples and troubleshooting guides

### Fixed
- ADC driver implementation for ESP32 compatibility

## [0.1.0-beta] - Initial Release

### Added
- Basic GPIO control (digital read/write)
- ADC support (analog input)
- DAC support (analog output)
- I2C communication
- I2S audio support
- Failsafe watchdog implementation
- Serial protocol communication
- Python library with auto-detection

---


## Version Numbering

This project follows [Semantic Versioning](https://semver.org/):
- MAJOR version for incompatible API changes
- MINOR version for added functionality in a backward compatible manner
- PATCH version for backward compatible bug fixes

