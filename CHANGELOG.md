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

### Documentation
- Added comprehensive API documentation for new features
- Created `advanced_features_example.py` demonstrating PWM, EEPROM, and batch operations
- Updated README with detailed usage examples
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

