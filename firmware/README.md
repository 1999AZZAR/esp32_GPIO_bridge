# ESP32 GPIO Bridge - Firmware Documentation

This directory contains the ESP32 GPIO Bridge firmware source code with modular architecture and clean separation of concerns.

## Architecture Overview

The firmware implements a comprehensive GPIO bridge that provides serial command interface for controlling ESP32 hardware peripherals. The system uses FreeRTOS for multi-tasking and implements failsafe mechanisms for safety-critical applications.

## Directory Structure

```
firmware/
├── firmware.ino          # Main entry point and system initialization
├── config.h              # Configuration constants and system parameters
├── response.h/.cpp       # Serial response buffer management
├── gpio.h/.cpp          # Digital I/O operations and pin management
├── pwm.h/.cpp           # PWM channel management and control
├── analog.h/.cpp        # ADC/DAC operations and calibration
├── i2c.h/.cpp           # I2C communication protocol
├── eeprom.h/.cpp        # EEPROM storage operations
├── i2s.h/.cpp           # I2S audio interface
└── README.md            # This documentation
```

## Module Specifications

### firmware.ino
Main system file containing:
- System initialization and configuration
- FreeRTOS task creation and management
- Command processing dispatch
- Failsafe system implementation
- Main control loop

### config.h
System configuration constants:
- Firmware version and build information
- Buffer sizes and memory limits
- Timeout and failsafe parameters
- Hardware-specific configuration values

### response.h/.cpp
Serial communication optimization:
- Response buffer management
- Efficient string building functions
- Serial output optimization
- Memory-efficient response handling

### gpio.h/.cpp
Digital I/O operations:
- Pin validation and management
- Digital read/write operations
- Pin mode configuration
- Batch write operations for multiple pins

### pwm.h/.cpp
PWM signal generation:
- PWM channel allocation and management
- O(1) channel lookup optimization
- Frequency and resolution control
- Channel lifecycle management

### analog.h/.cpp
Analog I/O operations:
- ADC initialization and calibration
- Analog read operations with 12-bit resolution
- DAC write operations (pins 25, 26)
- ADC channel mapping and configuration

### i2c.h/.cpp
I2C communication:
- I2C bus initialization
- Device scanning functionality
- Multi-byte read/write operations
- Error handling and status reporting

### eeprom.h/.cpp
Persistent storage:
- Single byte read/write operations
- Block read/write operations
- Memory management and validation
- Commit and clear operations

### i2s.h/.cpp
I2S audio interface:
- I2S transmit configuration
- Audio data transmission
- Buffer management
- Sample rate and format configuration

## System Features

### Command Interface
The system accepts serial commands in the format `<COMMAND parameters>`. Commands are processed by a dedicated FreeRTOS task for optimal responsiveness.

### Failsafe System
Implements a multi-stage failsafe mechanism:
- Communication timeout detection
- Automatic pin reset to safe states
- Warning system with grace periods
- Manual failsafe override capability

### Performance Optimizations
- Command queuing for batch processing
- Response buffering to minimize serial overhead
- O(1) PWM channel lookup
- Memory-efficient string handling

### Safety Features
- Pin validation and bounds checking
- Output command tracking for failsafe activation
- Automatic reset of configured pins on communication loss
- Input-only command exclusion from failsafe triggers

## Compilation

The firmware is designed for ESP32 Arduino Core 3.x with backward compatibility to 2.x. Compilation requires:

- ESP32 Arduino Core 3.3.2 or compatible
- FreeRTOS support
- Standard ESP32 libraries (Wire, EEPROM, I2S, ADC)

## Memory Usage

Current memory footprint:
- Program storage: 1,000,667 bytes (76% of 1,310,720 bytes)
- Dynamic memory: 56,000 bytes (17% of 327,680 bytes)
- Available for local variables: 271,680 bytes

## Version Information

- **Firmware Version:** 0.1.6-beta
- **Architecture:** Modular with clean separation of concerns
- **Last Updated:** 2025-01-09
- **Status:** Production ready

---

**Author:** ESP32 GPIO Bridge Project  
**License:** See project root directory  
**Support:** Refer to project documentation
