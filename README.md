# ESP32 GPIO Bridge

**Version:** 0.1.1-beta

## Description

ESP32 GPIO Bridge transforms an ESP32 development board into a versatile, PC-controlled hardware interface. It allows a host computer running a Python script to directly control and communicate with hardware components like sensors, actuators, and other ICs. This setup combines the processing power and rich software environment of a PC with the real-world interfacing capabilities of a microcontroller.

The system communicates over a simple, text-based serial protocol via a USB connection.

### System Architecture

```
+------------------------+           +--------------+      +---------+      +-------------------+
|   PC / Laptop          |           |              |      |         |      |                   |
| (Python Controller)    |  <----->  |  USB Serial  | <--> |  ESP32  | <--> |  Sensors/Actuators|
|                        |           |              |      |         |      |                   |
+------------------------+           +--------------+      +---------+      +-------------------+
```

## Features

- **Digital GPIO:** Set pin modes (`INPUT`, `OUTPUT`, `INPUT_PULLUP`), read digital states, and write digital states (`HIGH`/`LOW`).
- **Analog Input (ADC):** Read 12-bit analog values (0-4095) from ADC-capable pins (GPIO 32-39).
- **Analog Output (DAC):** Output a true analog voltage on dedicated DAC pins (GPIO 25 & 26) with 8-bit resolution (0-255).
- **I2C Communication:** Initialize an I2C bus, scan for devices, and perform read/write operations.
- **I2S Communication:** Configure an I2S port for basic data transmission.
- **Failsafe Watchdog:** A robust safety mechanism automatically resets pins to a safe state if communication with the host PC is lost.

## Getting Started

### 1. Firmware Setup (ESP32)

1. **Prerequisites:**
   * An ESP32 development board.
   * Arduino IDE installed with the ESP32 board manager (version 2.0.0 or higher recommended).
2. **Flashing Instructions:**
   * Open the `ESP32_GPIO_Bridge.ino` file in the Arduino IDE.
   * Connect your ESP32 board to your computer via USB.
   * In the Arduino IDE, go to `Tools` > `Board` and select your specific ESP32 board model.
   * Go to `Tools` > `Port` and select the serial/COM port corresponding to your ESP32.
   * Click the "Upload" button (right arrow icon) to compile and flash the firmware.

### 2. Host PC Setup (Python)

1. **Prerequisites:**
   * Python 3.6+ installed.
2. **Install Dependencies:**
   * The controller requires the `pyserial` library. Install it using pip:
     
     ```bash
     pip install pyserial
     ```
3. **Run the Controller:**
   * Save the Python host code as `esp_controller.py`.
   * Modify the `ESP32_PORT` variable at the bottom of the script to match the serial port of your ESP32.
   * Run the script from your terminal:
     
     ```bash
     python esp_controller.py
     ```

## Failsafe Mechanism

The firmware includes a watchdog timer to prevent hardware from being left in an unsafe state.

- The Python controller sends a `PING` command every second to the ESP32.
- If the ESP32 does not receive any valid command for **2 seconds**, it assumes the connection is lost.
- The failsafe routine is triggered, which resets all pins previously configured via a `MODE` command back to the default `INPUT` state.
- When communication resumes, the failsafe is disengaged.

## Python Controller Usage

First, import and instantiate the class:

```python
from esp_controller import ESP32GPIO
esp = ESP32GPIO("COM5")
```

### Digital I/O

```python
# Set GPIO 12 as an output
esp.set_pin_mode(12, "OUT")

# Set the pin HIGH
esp.digital_write(12, 1)

# Set GPIO 13 as an input with an internal pull-up resistor
esp.set_pin_mode(13, "IN_PULLUP")

# Read the pin's value
button_state = esp.digital_read(13)
print(f"Button state: {button_state}")
```

### Analog I/O

```python
# Read a 12-bit analog value from a sensor on GPIO 34
# Note: Valid ADC pins are GPIO 32, 33, 34, 35, 36, 37, 38, 39
raw_value = esp.analog_read(34)
voltage = (raw_value / 4095.0) * 3.3
print(f"Sensor voltage: {voltage:.2f}V")

# Output 1.3V from DAC pin 25 (1.3V / 3.3V * 255 = ~100)
esp.analog_write(25, 100)
```

### I2C Communication

```python
# Initialize I2C on standard ESP32 pins
SDA_PIN = 21
SCL_PIN = 22
esp.i2c_init(SDA_PIN, SCL_PIN)

# Scan for connected I2C devices
devices = esp.i2c_scan()
print(f"Found I2C devices at: {devices}")

# Write data [0x01, 0xFF] to a device at address 0x68
esp.i2c_write("0x68", [0x01, 0xFF])

# Read 2 bytes from the device at address 0x68
data = esp.i2c_read("0x68", 2)
print(f"Read data: {data}")
```

### Closing the connection

```python
# Always close the connection when done
esp.close()
```

## Command Protocol Reference

Communication is performed at **115200 baud**. All commands and responses are wrapped in `<...>` delimiters.

| Command                                | Parameters & Description                                                 |
| -------------------------------------- | ------------------------------------------------------------------------ |
| `VERSION`                              | No parameters. Returns the firmware version string.                      |
| `PING`                                 | No parameters. Keeps the failsafe watchdog alive. Returns `<PONG>`.      |
| `MODE <pin> <mode>`                    | `<pin>`: GPIO number. `<mode>`: `OUT`, `IN`, or `IN_PULLUP`.             |
| `WRITE <pin> <value>`                  | `<pin>`: GPIO number. `<value>`: `1` (HIGH) or `0` (LOW).                |
| `READ <pin>`                           | `<pin>`: GPIO number. Returns the digital state (`<1>` or `<0>`).        |
| `AREAD <pin>`                          | `<pin>`: ADC-capable pin (32-39). Returns a 12-bit value (`<0>`-`<4095>`). |
| `AWRITE <pin> <value>`                 | `<pin>`: **25 or 26**. `<value>`: 8-bit value (`0`-`255`).               |
| `I2C_INIT <sda> <scl>`                 | `<sda>`/`<scl>`: GPIO pins for I2C data and clock.                       |
| `I2C_SCAN`                             | No parameters. Returns a space-separated list of found device addresses. |
| `I2C_WRITE <addr> <d0> ...`            | `<addr>`: Hex address. `<d0>...`: Hex data bytes to write.               |
| `I2C_READ <addr> <len>`                | `<addr>`: Hex address. `<len>`: Number of bytes to read.                 |
| `I2S_INIT_TX <bck> <ws> <data> <rate>` | Configures I2S for transmit on specified pins at a given sample `rate`.  |
| `I2S_WRITE <d0> <d1> ...`              | Writes a block of 16-bit signed integer samples to the I2S bus.          |

## Technical Notes

### ADC Implementation

This firmware uses the ESP-IDF native ADC driver instead of the Arduino `analogRead()` function to avoid conflicts with other peripherals (I2S, Wi-Fi, etc.) in newer ESP32 Arduino core versions.

**Valid ADC1 Pins (12-bit resolution):**
- GPIO 32 (ADC1_CH4)
- GPIO 33 (ADC1_CH5)
- GPIO 34 (ADC1_CH6)
- GPIO 35 (ADC1_CH7)
- GPIO 36 (ADC1_CH0)
- GPIO 37 (ADC1_CH1)
- GPIO 38 (ADC1_CH2)
- GPIO 39 (ADC1_CH3)

The ADC is configured with:
- 12-bit resolution (0-4095)
- 11 dB attenuation (full 0-3.3V range)
- Calibration using eFuse values for improved accuracy

### Changelog

#### v0.1.1-beta
- **Fixed:** Replaced legacy `analogRead()` with ESP-IDF ADC driver to prevent boot loop crashes
- **Fixed:** Resolved ADC driver conflict that caused "CONFLICT! driver_ng is not allowed" error
- **Improved:** Added proper ADC initialization and channel mapping
- **Added:** Support for ADC calibration characteristics

#### v0.1.0-beta
- Initial release
- Basic GPIO, ADC, DAC, I2C, and I2S functionality
- Failsafe watchdog implementation