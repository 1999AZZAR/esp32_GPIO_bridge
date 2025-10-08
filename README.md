# ESP32 GPIO Bridge

**Version:** 0.1.2-beta

## Overview

ESP32 GPIO Bridge transforms an ESP32 development board into a versatile, PC-controlled hardware interface. It allows a host computer running Python to directly control and communicate with hardware components like sensors, actuators, and other ICs. This setup combines the processing power and rich software environment of a PC with the real-world interfacing capabilities of a microcontroller.

The system communicates over a simple, text-based serial protocol via USB connection, providing a comprehensive Python library for GPIO control, I2C communication, and sensor integration.

### System Architecture

```
+------------------------+           +--------------+      +---------+      +-------------------+
|   PC / Laptop          |           |              |      |         |      |                   |
| (Python Library)       |  <----->  |  USB Serial  | <--> |  ESP32  | <--> |  Sensors/Actuators|
|                        |           |              |      |         |      |                   |
+------------------------+           +--------------+      +---------+      +-------------------+
```

## Features

- **Digital GPIO Control:** Set pin modes, read/write digital states with comprehensive pin management
- **Analog Input (ADC):** 12-bit analog readings (0-4095) from ADC-capable pins with calibration support
- **Analog Output (DAC):** 8-bit analog voltage output on dedicated DAC pins (GPIO 25 & 26)
- **I2C Communication:** Full I2C bus control with device scanning and data transfer
- **I2S Audio Support:** Audio data transmission capabilities
- **Pin Management:** Advanced pin capability detection and validation system
- **Configuration Presets:** Pre-configured setups for common applications
- **Enhanced Failsafe System:** Intelligent multi-stage failsafe with graceful recovery
- **Context Manager Support:** Automatic resource cleanup with `with` statements

## Enhanced Failsafe Mechanism

The firmware includes an intelligent multi-stage failsafe system to prevent hardware from being left in an unsafe state while avoiding false triggers.

### Failsafe Stages

1. **Warning Stage** (after 2 seconds of inactivity):

   - ESP32 sends warning messages about communication loss
   - Provides 3-second grace period for recovery
2. **Failsafe Engagement** (after 5 seconds total):

   - All configured pins are reset to INPUT mode for safety
   - Detailed logging of which pins were reset
   - System enters protected state
3. **Recovery Detection** (sustained communication):

   - Monitors for sustained communication (5 seconds)
   - Automatically disengages failsafe when stable connection detected

### Failsafe Features

- **Dual Monitoring**: Tracks both command activity and PING responses
- **Graceful Recovery**: Requires sustained communication before disengaging failsafe
- **Detailed Logging**: Provides comprehensive status information
- **Pin Safety**: Automatically resets all configured pins to INPUT mode
- **Status Query**: Use `STATUS` command to check current failsafe state

### Example Failsafe Sequence

```
<WARN:No activity detected for 2 seconds>
<INFO:Send PING or any command within 3 seconds to prevent failsafe>
<WARN:Failsafe engaged - Communication lost>
<INFO:All configured pins reset to INPUT mode for safety>
<INFO:Reset pin 2 to INPUT>
<INFO:Reset pin 34 to INPUT>
<INFO:Failsafe active - Waiting for communication recovery>
<INFO:Sustained communication detected - Disengaging failsafe>
<INFO:Failsafe disengaged - Normal operation resumed>
```

## Installation

### 1. ESP32 Firmware Setup

**Prerequisites:**

- ESP32 development board
- Arduino IDE with ESP32 board manager (version 2.0.0 or higher recommended)

**Flashing Instructions:**

1. Open `esp32_GPIO_bridge.ino` in the Arduino IDE
2. Connect ESP32 board via USB
3. Select your ESP32 board model in Tools > Board
4. Select the correct serial port in Tools > Port
5. Upload the firmware to the ESP32

### 2. Python Library Installation

**Prerequisites:**

- Python 3.6 or higher

**Install from source:**

```bash
git clone https://github.com/1999AZZAR/esp32_GPIO_bridge.git esp32-gpio-bridge
cd esp32-gpio-bridge
pip install -e .
```

**Or install directly:**

```bash
pip install esp32-gpio-bridge
```

**Dependencies:**

```bash
pip install pyserial
```

## Quick Start

```python
from esp32_gpio_bridge import ESP32GPIO, find_esp32_port

# Auto-detect ESP32 port
port = find_esp32_port()

# Connect using context manager (auto-cleanup)
with ESP32GPIO(port) as esp:
    print(f"Firmware version: {esp.get_version()}")

    # Digital output - LED control
    esp.set_pin_mode(2, "OUT")
    esp.digital_write(2, 1)  # LED ON

    # Analog input - Sensor reading
    sensor_value = esp.analog_read(34)
    voltage = (sensor_value / 4095.0) * 3.3
    print(f"Sensor voltage: {voltage:.2f}V")
```

## Python Library API

### Core Controller Class

#### ESP32GPIO

Main controller class providing all ESP32 GPIO operations.

```python
esp = ESP32GPIO(port, baudrate=115200, timeout=1.0)
```

**Parameters:**

- `port` (str): Serial port device path
- `baudrate` (int): Communication speed (default: 115200)
- `timeout` (float): Serial timeout in seconds (default: 1.0)
- `auto_connect` (bool): Auto-connect on initialization (default: True)

### Status Monitoring

```python
# Check ESP32 status including failsafe state
status = esp.get_status()
print(f"ESP32 State: {status['state']}")
print(f"Failsafe engaged: {status['failsafe_engaged']}")
print(f"Time since last command: {status['time_since_command']}ms")
```

### Digital I/O Operations

```python
# Pin configuration
esp.set_pin_mode(pin, mode)
# Modes: 'IN', 'OUT', 'IN_PULLUP'

# Digital output
esp.digital_write(pin, value)
# value: 0, 1, True, False

# Digital input
state = esp.digital_read(pin)
# Returns: 0 or 1
```

**Pin Range:** 0-39 (GPIO pin numbers)

### Analog I/O Operations

```python
# Analog input (ADC)
raw_value = esp.analog_read(pin)
# Returns: 12-bit value (0-4095)
# Valid pins: 32, 33, 34, 35, 36, 37, 38, 39

# Convert to voltage
voltage = (raw_value / 4095.0) * 3.3

# Analog output (DAC)
esp.analog_write(pin, value)
# value: 8-bit (0-255)
# Valid pins: 25, 26 (DAC1, DAC2)
```

### I2C Communication

```python
# Initialize I2C bus
esp.i2c_init(sda_pin, scl_pin)

# Scan for devices
devices = esp.i2c_scan()
# Returns: List of hex addresses as strings

# Write data
esp.i2c_write(device_addr, data_bytes)
# device_addr: "0x68", data_bytes: [0x01, 0xFF]

# Read data
data = esp.i2c_read(device_addr, num_bytes)
# Returns: List of integers
```

### Advanced Features

#### Pin Management

```python
from esp32_gpio_bridge import pins

# Check pin capabilities
caps = pins.ESP32PinManager.get_pin_capabilities(34)
print(f"Pin 34 can read analog: {caps.can_analog_read}")

# Get all ADC-capable pins
adc_pins = pins.ESP32PinManager.get_adc_pins()

# Validate pin for specific mode
pins.validate_pin_for_mode(34, "AREAD")  # Raises ValueError if invalid
```

#### Configuration Presets

```python
from esp32_gpio_bridge import config

# List available presets
presets = config.get_config().list_presets()

# Apply a preset
preset_config = config.get_config().apply_preset("basic_io")

# Access preset pin mappings
led_pin = preset_config["pins"]["led"]
```

#### Connection Management

```python
# Context manager (recommended)
with ESP32GPIO(port) as esp:
    # Automatic cleanup on exit
    esp.digital_write(2, 1)

# Manual connection management
esp = ESP32GPIO(port, auto_connect=False)
esp.connect()

# Check connection status
if esp.is_connected:
    # Connected and responding
    pass

esp.close()  # Manual cleanup
```

## Serial Port Management

### Auto-Detection

```python
from esp32_gpio_bridge import find_esp32_port, list_serial_ports

# Auto-detect ESP32
port = find_esp32_port()
if port:
    print(f"Found ESP32 on: {port}")

# List all available ports
ports = list_serial_ports()
for device, description in ports:
    print(f"{device}: {description}")
```

### Manual Port Selection

```python
from esp32_gpio_bridge import select_port

# Interactive port selection
port = select_port()
if port:
    esp = ESP32GPIO(port)
```

## Configuration Management

### Default Configuration

The library includes comprehensive default settings:

```python
config = {
    'serial': {
        'baudrate': 115200,
        'timeout': 1.0,
        'auto_connect': True
    },
    'pins': {
        'common': {
            'led_builtin': 2,
            'button_builtin': 0,
            'adc_vref': 3.3,
            'adc_resolution': 12
        }
    },
    'i2c': {
        'default_sda': 21,
        'default_scl': 22,
        'frequency': 100000
    }
}
```

### Custom Configuration

```python
from esp32_gpio_bridge import config

# Update configuration
cfg = config.get_config()
cfg.update_config('serial', 'timeout', 2.0)

# Add custom preset
cfg.add_custom_preset('my_sensors', {
    'pins': {
        'temperature': 34,
        'humidity': 35,
        'pressure_sda': 21,
        'pressure_scl': 22
    }
}, "Custom sensor configuration")
```

## Examples

The `examples/` directory contains comprehensive examples:

### Basic I/O Example

```bash
python examples/basic_io_example.py
```

Demonstrates digital I/O, analog I/O, and LED control.

### Sensor Hub Example

```bash
python examples/sensor_hub_example.py
```

Shows I2C sensor integration and multi-device communication.

See `examples/README.md` for detailed setup instructions and troubleshooting.

## Command Protocol Reference

Communication uses **115200 baud** with commands/responses wrapped in `<...>` delimiters.

| Command                                  | Parameters                        | Description                            |
| ---------------------------------------- | --------------------------------- | -------------------------------------- |
| `VERSION`                              | None                              | Returns firmware version               |
| `PING`                                 | None                              | Watchdog keep-alive (returns `PONG`)   |
| `STATUS`                               | None                              | Returns current system status          |
| `MODE <pin> <mode>`                    | pin: 0-39, mode: IN/OUT/IN_PULLUP | Set pin mode                           |
| `WRITE <pin> <value>`                  | pin: 0-39, value: 0/1             | Digital write                          |
| `READ <pin>`                           | pin: 0-39                         | Digital read (returns 0/1)             |
| `AREAD <pin>`                          | pin: 32-39                        | Analog read (returns 0-4095)           |
| `AWRITE <pin> <value>`                 | pin: 25/26, value: 0-255          | Analog write                           |
| `I2C_INIT <sda> <scl>`                 | SDA/SCL pins                      | Initialize I2C bus                     |
| `I2C_SCAN`                             | None                              | Scan for I2C devices                   |
| `I2C_WRITE <addr> <data...>`           | Hex address + data bytes          | I2C write                              |
| `I2C_READ <addr> <len>`                | Hex address + length              | I2C read                               |
| `I2S_INIT_TX <bck> <ws> <data> <rate>` | Pin numbers + sample rate         | Initialize I2S                         |

## Technical Specifications

### ADC Implementation

- **Resolution:** 12-bit (0-4095)
- **Voltage Range:** 0-3.3V (11dB attenuation)
- **Calibration:** eFuse-based for improved accuracy
- **Valid Pins:** GPIO 32-39 (ADC1 channels)

### DAC Implementation

- **Resolution:** 8-bit (0-255)
- **Voltage Range:** 0-3.3V
- **Valid Pins:** GPIO 25 (DAC1), 26 (DAC2)

### I2C Specifications

- **Default Pins:** SDA=21, SCL=22 (configurable)
- **Speed:** Up to 400kHz (software limited)
- **Device Support:** Standard I2C protocol

## Error Handling

The library includes comprehensive error handling:

```python
try:
    esp.analog_read(99)  # Invalid pin
except ValueError as e:
    print(f"Invalid pin: {e}")

try:
    esp.analog_read(34)  # Valid pin, no ESP32 connected
except IOError as e:
    print(f"Connection error: {e}")
```

## Logging

Enable detailed logging for debugging:

```python
import logging
logging.basicConfig(level=logging.DEBUG)

esp = ESP32GPIO(port)
# Will show detailed connection and communication logs
```

## Troubleshooting

### Common Issues

1. **ESP32 not detected**

   - Verify USB connection and drivers
   - Check Arduino IDE port selection
   - Try resetting the ESP32 board
2. **Permission denied (Linux/Mac)**

   ```bash
   sudo usermod -a -G dialout $USER
   # Logout and login again
   ```
3. **No response from ESP32**

   - Confirm firmware is flashed correctly
   - Check serial port baud rate (115200)
   - Verify ESP32 is not in flash mode
4. **ADC read errors**

   - Ensure pin is ADC-capable (32-39)
   - Check for pin conflicts with other peripherals

### Debug Information

```python
# Check connection status
print(f"Connected: {esp.is_connected}")

# List configured pins
print(f"Active pins: {esp.get_configured_pins()}")

# Check pin capabilities
from esp32_gpio_bridge import pins
caps = pins.ESP32PinManager.get_pin_capabilities(pin)
print(f"Pin {pin} supports analog: {caps.can_analog_read}")
```

## Development

### Project Structure

```
esp32-gpio-bridge/
├── .github/               # GitHub Actions workflows
│   └── workflows/
│       └── python-package-conda.yml
├── esp32_gpio_bridge/     # Main Python package
│   ├── __init__.py       # Package exports
│   ├── controller.py     # Core ESP32GPIO class (570 lines)
│   ├── pins.py          # Pin management utilities (225 lines)
│   └── config.py        # Configuration management (277 lines)
├── examples/             # Example applications
│   ├── basic_io_example.py
│   ├── sensor_hub_example.py
│   └── README.md
├── tests/               # Test suite
│   ├── __init__.py
│   └── test_pins.py     # Pin management tests (16 tests)
├── esp32_GPIO_bridge.ino # ESP32 firmware
├── environment.yml      # Conda environment configuration
├── setup.py            # Package installation
├── requirements.txt    # Python dependencies
├── .gitignore         # Comprehensive ignore patterns
└── README.md          # This file (483 lines)
```

### Contributing

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Submit a pull request

### Testing

```bash
# Install development dependencies
pip install -e ".[dev]"

# Run tests
pytest

# Run tests with coverage
pytest --cov=esp32_gpio_bridge --cov-report=html

# Type checking
mypy esp32_gpio_bridge/

# Lint code
flake8 esp32_gpio_bridge/
```

### Version Control

The project includes a comprehensive `.gitignore` file that excludes:

- **Python artifacts:** `__pycache__/`, `*.pyc`, virtual environments
- **IDE files:** `.vscode/`, `.idea/`, `.spyderproject`
- **OS files:** `.DS_Store`, `Thumbs.db`, desktop files
- **ESP32 build files:** Arduino binaries, PlatformIO artifacts
- **Testing artifacts:** Coverage reports, pytest cache
- **Documentation builds:** Sphinx, MkDocs output
- **CI/CD artifacts:** GitHub Actions cache, build artifacts

This ensures a clean repository while preserving essential project files.

## Changelog

### v0.1.2-beta

- **Added:** Comprehensive .gitignore file for Python/ESP32 development
- **Added:** GitHub Actions CI/CD pipeline with multi-Python testing
- **Added:** Complete test suite with 16 comprehensive tests
- **Improved:** Pin capability mapping accuracy for ESP32 hardware
- **Enhanced:** Documentation with CI/CD and testing guidelines
- **Fixed:** Touch sensor pin capability detection

### v0.1.1-beta

- **Refactored:** Complete library restructuring with proper package organization
- **Added:** Advanced pin management system with capability detection
- **Added:** Configuration management with presets for common applications
- **Added:** Context manager support for automatic resource cleanup
- **Added:** Comprehensive error handling and validation
- **Added:** Type hints throughout the codebase
- **Improved:** Documentation with examples and troubleshooting guides
- **Fixed:** ADC driver implementation for ESP32 compatibility

### v0.1.0-beta

- Initial release with basic GPIO, ADC, DAC, I2C, and I2S functionality
- Failsafe watchdog implementation

## License

MIT License - see LICENSE file for details.

## Support

For issues, questions, or contributions:

- Check existing documentation and examples
- Review troubleshooting section
- Open an issue on the project repository
- Contact the development team
