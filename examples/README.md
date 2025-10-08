# ESP32 GPIO Bridge Examples

This directory contains example scripts demonstrating various features of the ESP32 GPIO Bridge library.

## Prerequisites

Before running any examples, ensure you have:

1. **ESP32 Development Board** with the GPIO Bridge firmware flashed
2. **USB Connection** between your computer and the ESP32
3. **Required Dependencies** installed:
   ```bash
   pip install -r ../requirements.txt
   ```

## Hardware Setup

### Basic I/O Example
- LED connected to GPIO pin 2 (with appropriate current-limiting resistor)
- Potentiometer connected to GPIO pin 34 (ADC input)
- Optional: Button connected to GPIO pin 0 (with pull-up resistor)

### Sensor Hub Example
- I2C sensors connected to GPIO pins 21 (SDA) and 22 (SCL)
- Common sensors that work with these examples:
  - BMP280 (Pressure/Temperature sensor)
  - MPU6050 (Accelerometer/Gyroscope)
  - SSD1306 OLED display
  - Any other I2C device

## Running Examples

### Basic I/O Example

```bash
python basic_io_example.py
```

This example demonstrates:
- Digital output (blinking LED)
- Analog input (reading potentiometer)
- Digital input (button reading)
- Analog output (DAC output)

### Sensor Hub Example

```bash
python sensor_hub_example.py
```

This example demonstrates:
- I2C bus initialization and device scanning
- Reading from multiple I2C sensors
- Generic I2C device communication

## Troubleshooting

### Common Issues

1. **"ESP32 not auto-detected"**
   - Ensure the ESP32 is connected via USB
   - Check that the GPIO Bridge firmware is flashed
   - Try running with verbose logging: `python -c "import logging; logging.basicConfig(level=logging.DEBUG)"`

2. **"Port already in use"**
   - Close any other applications using the serial port (Arduino IDE, other terminal programs)
   - On Linux/Mac: `lsof /dev/ttyUSB0` to see what's using the port

3. **"No response from ESP32"**
   - Check that the ESP32 has power and is properly connected
   - Verify the firmware version matches the library version
   - Try resetting the ESP32 board

4. **Permission denied (Linux/Mac)**
   ```bash
   sudo usermod -a -G dialout $USER
   # Then logout and login again
   ```

### Debug Tips

Enable debug logging to see more detailed information:

```python
import logging
logging.basicConfig(level=logging.DEBUG)

# Your code here
```

Check available serial ports:
```python
from esp32_gpio_bridge import list_serial_ports
ports = list_serial_ports()
print("Available ports:", ports)
```

## Example Output

### Basic I/O Example
```
Found ESP32 on port: /dev/ttyUSB0
Connected to ESP32. Firmware version: 0.1.1-beta

LED Pin 2 capabilities:
  - Digital Write: True
  - PWM: True

Potentiometer Pin 34 capabilities:
  - Analog Read: True
  - Digital Read: False

==================================================
DIGITAL OUTPUT EXAMPLE - Blinking LED
==================================================
Set GPIO 2 as OUTPUT
Blink 1: ON
Blink 1: OFF
...

Reading analog values from GPIO 34...
Reading  1: Raw= 892, Voltage=0.72V, Percentage= 21.8%
Reading  2: Raw=1203, Voltage=0.97V, Percentage= 29.4%
...
```

## Extending Examples

Feel free to modify these examples or create new ones for your specific use cases:

- **Custom Sensors**: Add support for your specific I2C sensors
- **Real-time Monitoring**: Create continuous monitoring applications
- **Data Logging**: Add file output for data collection
- **Web Integration**: Combine with web frameworks for remote monitoring

## Support

For issues or questions:
1. Check the main [README.md](../README.md) for general documentation
2. Review the troubleshooting section above
3. Create an issue on the project repository
