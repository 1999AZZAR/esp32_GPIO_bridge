# ESP32 GPIO Bridge

**Version:** 0.1.3-beta

## Overview

ESP32 GPIO Bridge transforms an ESP32 development board into a versatile, PC-controlled hardware interface. It allows a host computer running Python to directly control and communicate with hardware components like sensors, actuators, and other ICs. This setup combines the processing power and rich software environment of a PC with the real-world interfacing capabilities of a microcontroller.

The system communicates over a simple, text-based serial protocol via USB connection, providing a comprehensive Python library for GPIO control, PWM, I2C communication, EEPROM storage, and sensor integration.

**New in v0.1.3-beta:** WiFi and Bluetooth are now disabled by default to maximize GPIO performance and free up resources for extensive peripheral usage.

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
- **PWM Control:** Hardware PWM on up to 16 channels with configurable frequency and resolution
- **Analog Input (ADC):** 12-bit analog readings (0-4095) from ADC-capable pins with calibration support
- **Analog Output (DAC):** 8-bit analog voltage output on dedicated DAC pins (GPIO 25 & 26)
- **EEPROM Storage:** 512 bytes of persistent storage for configuration and data
- **I2C Communication:** Full I2C bus control with device scanning and data transfer
- **I2S Audio Support:** Audio data transmission capabilities
- **Batch Operations:** Efficient multi-pin control in a single command
- **Pin Management:** Advanced pin capability detection and validation system
- **Configuration Presets:** Pre-configured setups for common applications
- **Enhanced Failsafe System:** Intelligent multi-stage failsafe with instant recovery and manual control
- **Context Manager Support:** Automatic resource cleanup with `with` statements
- **Optimized Performance:** WiFi and Bluetooth disabled for maximum GPIO resource availability

## Enhanced Failsafe Mechanism

The firmware includes an intelligent multi-stage failsafe system to prevent hardware from being left in an unsafe state while avoiding false triggers.

**Important:** Failsafe **only activates** if output commands are sent (MODE with OUT/IN_PULLUP, WRITE, PWM, DAC, etc.). Query commands (IDENTIFY, VERSION, READ, EEPROM) don't trigger failsafe, allowing auto-detection and monitoring without requiring ESP32 reset.

### Failsafe Stages

1. **Warning Stage** (after 10 seconds of inactivity):

   - ESP32 sends warning messages about communication loss
   - Provides 20-second grace period for recovery
2. **Failsafe Engagement** (after 30 seconds total):

   - All configured pins are reset to INPUT mode for safety
   - Detailed logging of which pins were reset
   - System enters protected state
3. **Recovery Detection** (immediate):

   - Any command immediately disengages failsafe
   - No waiting period required for recovery

### Failsafe Features

- **Smart Activation**: Only activates after output commands (MODE OUT, WRITE, PWM, etc.)
- **Query-Safe**: IDENTIFY, VERSION, READ, STATUS, EEPROM commands don't trigger failsafe
- **Dual Monitoring**: Tracks both command activity and PING responses
- **Generous Timeout**: 30 seconds total (10s warning + 20s grace) for development convenience
- **Instant Recovery**: Any command immediately disengages failsafe (no waiting period)
- **Manual Control**: Commands like `RESET_FAILSAFE` provide manual control
- **Detailed Logging**: Provides comprehensive status information
- **Pin Safety**: Automatically resets all configured pins to INPUT mode
- **Status Query**: Use `STATUS` command to check current failsafe state

### Example Failsafe Sequence

```
# After using output commands (MODE OUT, WRITE, etc.):
<WARN:No activity detected for 10 seconds>
<INFO:Send PING or any command within 20 seconds to prevent failsafe>
<WARN:Failsafe engaged - Communication lost>
<INFO:All configured pins reset to INPUT mode for safety>
<INFO:Reset pin 2 to INPUT>
<INFO:Reset pin 34 to INPUT>
<INFO:Failsafe active - Send any command to disengage>
<INFO:Communication detected - Disengaging failsafe>
<INFO:Failsafe disengaged - Normal operation resumed>

# Note: Query commands (IDENTIFY, VERSION, READ) don't trigger failsafe at all
```

### Manual Failsafe Control

You can also manually control the failsafe system:

```python
# Check if failsafe is engaged
status = esp.get_status()
print(f"Failsafe engaged: {status['failsafe_engaged']}")

# Manually reset failsafe (works even when engaged)
success = esp.reset_failsafe()  # or esp.clear_failsafe() or esp.disable_failsafe()
print(f"Failsafe reset: {success}")
```

**ESP32 Commands:**
- `RESET_FAILSAFE`, `CLEAR_FAILSAFE`, `DISABLE_FAILSAFE` - Manually disengage failsafe

## Installation

### 1. ESP32 Firmware Setup

**Prerequisites:**

- ESP32 development board
- Arduino IDE with ESP32 board manager (version 2.0.0 or higher)
  - Compatible with ESP32 Arduino core 2.x and 3.x

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

### Failsafe Control

```python
# Manually reset failsafe mode
success = esp.reset_failsafe()  # Returns True if reset, False if not engaged

# Alternative methods (aliases)
success = esp.clear_failsafe()
success = esp.disable_failsafe()
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

### PWM Operations

```python
# Initialize PWM
channel = esp.pwm_init(pin, frequency=5000, resolution=8)
# frequency: 1-40000 Hz (default: 5000)
# resolution: 1-16 bits (default: 8)

# Set duty cycle (raw value)
esp.pwm_write(pin, 128)  # 50% duty cycle for 8-bit resolution

# Set duty cycle (percentage)
esp.pwm_set_duty_percent(pin, 50.0)  # 50% duty cycle

# Stop PWM and release channel
esp.pwm_stop(pin)
```

### EEPROM Operations

```python
# Read single byte
value = esp.eeprom_read(address)  # address: 0-511

# Write single byte (requires commit)
esp.eeprom_write(address, value)  # value: 0-255
esp.eeprom_commit()  # Save changes to flash

# Read block of data
data = esp.eeprom_read_block(address, length)
# Returns: List of integers

# Write block of data
esp.eeprom_write_block(address, [10, 20, 30, 40])
esp.eeprom_commit()

# String operations
esp.eeprom_write_string(100, "Hello ESP32!")
esp.eeprom_commit()
text = esp.eeprom_read_string(100, 12)

# Clear all EEPROM
esp.eeprom_clear()  # Sets all bytes to 0 and commits
```

### Batch Operations

```python
# Set multiple pins efficiently in one command
esp.batch_digital_write({
    2: 1,   # Pin 2 HIGH
    4: 0,   # Pin 4 LOW
    5: 1,   # Pin 5 HIGH
    12: 0   # Pin 12 LOW
})
# Much faster than individual digital_write() calls
```

### I2C Communication

```python
# Initialize I2C bus
esp.i2c_init(sda=21, scl=22, frequency=100000)

# Scan for devices
devices = esp.i2c_scan()
# Returns: List of I2C addresses as integers [60, 87, ...]

# Write data to I2C device
esp.i2c_write(0x3C, [0x00, 0xFF, 0x80])
# address: I2C device address (7-bit)
# data: Single byte, list of bytes, or bytes object

# Read data from I2C device
data = esp.i2c_read(0x3C, 4)
# Returns: List of integers [0x12, 0x34, 0x56, 0x78]
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

The library now includes **robust auto-detection** that actively probes serial ports:

```python
from esp32_gpio_bridge import find_esp32_port, list_serial_ports

# Auto-detect ESP32 (actively probes ports)
port = find_esp32_port()
if port:
    print(f"Found ESP32 on: {port}")
else:
    print("ESP32 not found")

# List all available ports
ports = list_serial_ports()
for device, description in ports:
    print(f"{device}: {description}")
```

The auto-detection:
- **Disables DTR/RTS** to prevent unwanted ESP32 resets
- **Drains boot messages** (0.5s buffer clear) for clean communication
- **Sends PING/IDENTIFY** commands with 3 retries per port
- **Checks ESP32 GPIO Bridge signature** in responses
- **No manual reset needed** - works automatically!
- Reliable (2s timeout per port with intelligent retry logic)

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

The `examples/` directory contains comprehensive examples demonstrating all features:

### Core Examples

**Basic I/O Example** - `basic_io_example.py`
- Digital input/output (LED, button)
- Analog input (potentiometer, sensors)
- DAC output (analog voltage generation)

**Sensor Hub Example** - `sensor_hub_example.py`
- I2C initialization and device scanning
- Multi-sensor reading and processing
- Real-world sensor integration

**Advanced Features Example** - `advanced_features_example.py`
- PWM control for LED fading
- EEPROM persistent storage
- Batch GPIO operations
- Performance comparison

### Specialized Examples (NEW in v0.1.3-beta)

**PWM Servo Control** - `pwm_servo_control.py`
- Servo motor control with PWM
- Smooth angle transitions
- Multi-servo coordination

**LED Patterns** - `led_patterns_example.py`
- Creative LED patterns and effects
- Batch operations for synchronized control
- Knight Rider, rainbow, and breathing effects

**EEPROM Configuration** - `eeprom_config_example.py`
- Persistent configuration storage
- WiFi credentials and settings
- JSON configuration serialization

**DAC Waveform Generator** - `dac_waveform_generator.py`
- Sine, square, triangle, and sawtooth waves
- Audio tone generation
- Frequency control

**Multi-Sensor Dashboard** - `multi_sensor_dashboard.py`
- Real-time sensor monitoring
- Multiple analog sensors
- Data logging and statistics

**Ultrasonic Distance Meter** - `ultrasonic_distance_meter.py`
- HC-SR04 distance measurement
- Visual zone indicators (LEDs)
- Proximity alarm
- Statistical analysis

**Digital Thermometer Logger** - `digital_thermometer_logger.py`
- TMP36 temperature monitoring
- Real-time display with alerts
- CSV data export
- Moving average smoothing

Run any example:
```bash
cd examples
python <example_name>.py
```

See `examples/README.md` for detailed setup instructions, wiring diagrams, and troubleshooting.

## Command Protocol Reference

Communication uses **115200 baud** with commands/responses wrapped in `<...>` delimiters.

| Command                                  | Parameters                        | Description                            |
| ---------------------------------------- | --------------------------------- | -------------------------------------- |
| `IDENTIFY`                             | None                              | Returns device signature (for auto-detect) |
| `VERSION`                              | None                              | Returns firmware version               |
| `PING`                                 | None                              | Watchdog keep-alive (returns `PONG`)   |
| `STATUS`                               | None                              | Returns current system status          |
| `RESET_FAILSAFE`                       | None                              | Manually disengage failsafe mode       |
| `CLEAR_FAILSAFE`                       | None                              | Manually disengage failsafe mode       |
| `DISABLE_FAILSAFE`                     | None                              | Manually disengage failsafe mode       |
| `MODE <pin> <mode>`                    | pin: 0-39, mode: IN/OUT/IN_PULLUP | Set pin mode                           |
| `WRITE <pin> <value>`                  | pin: 0-39, value: 0/1             | Digital write                          |
| `READ <pin>`                           | pin: 0-39                         | Digital read (returns 0/1)             |
| `AREAD <pin>`                          | pin: 32-39                        | Analog read (returns 0-4095)           |
| `AWRITE <pin> <value>`                 | pin: 25/26, value: 0-255          | Analog write                           |
| `PWM_INIT <pin> <freq> <res>`          | pin, frequency (Hz), resolution   | Initialize PWM (returns channel)       |
| `PWM_WRITE <pin> <duty>`               | pin, duty cycle value             | Set PWM duty cycle                     |
| `PWM_STOP <pin>`                       | pin                               | Stop PWM and release channel           |
| `EEPROM_READ <addr>`                   | address: 0-511                    | Read byte from EEPROM                  |
| `EEPROM_WRITE <addr> <val>`            | address, value: 0-255             | Write byte to EEPROM                   |
| `EEPROM_READ_BLOCK <addr> <len>`       | address, length                   | Read block from EEPROM                 |
| `EEPROM_WRITE_BLOCK <addr> <data...>`  | address, byte values              | Write block to EEPROM                  |
| `EEPROM_COMMIT`                        | None                              | Commit EEPROM changes to flash         |
| `EEPROM_CLEAR`                         | None                              | Clear all EEPROM and commit            |
| `BATCH_WRITE <pin1> <val1> ...`        | pin/value pairs                   | Write multiple pins at once            |
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

See [CHANGELOG.md](CHANGELOG.md) for detailed version history.

## License

MIT License - see LICENSE file for details.

## Support

For issues, questions, or contributions:

- Check existing documentation and examples
- Review troubleshooting section
- Open an issue on the project repository
- Contact the development team
