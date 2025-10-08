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

### Advanced Features Example
- LED connected to GPIO pin 2 (for PWM dimming demonstration)
- Optional: Motor with PWM control connected to GPIO pin 16
- Optional: Multiple LEDs on GPIO pins 12, 13, 14, 15 (for batch operations)
- No additional hardware required for EEPROM features

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

### Advanced Features Example

```bash
python advanced_features_example.py
```

This example demonstrates:
- PWM control for LED dimming and motor control
- EEPROM storage for persistent data
- Batch GPIO operations for efficiency
- String and configuration storage in EEPROM
- Performance comparison between traditional and batch operations

### PWM Servo Control (NEW in v0.1.3-beta)

```bash
python pwm_servo_control.py
```

Hardware needed: 1-2 servo motors connected to GPIO 18 and 19

This example demonstrates:
- Servo motor control with PWM (50Hz)
- Smooth angle transitions
- Preset position control (0°, 45°, 90°, 135°, 180°)
- Wave patterns and synchronized multi-servo control
- Angle-to-duty-cycle conversion

### LED Patterns (NEW in v0.1.3-beta)

```bash
python led_patterns_example.py
```

Hardware needed: 8 LEDs with resistors on GPIO 2, 4, 5, 12, 13, 14, 15, 16

This example demonstrates:
- Knight Rider / Cylon scanner effect
- Binary counter display
- Chase and wave patterns
- Alternating blink patterns
- Random sparkle effect
- Breathing effect with PWM
- Batch operations for synchronized LED control

### EEPROM Configuration (NEW in v0.1.3-beta)

```bash
python eeprom_config_example.py
```

No additional hardware needed - uses ESP32's internal flash

This example demonstrates:
- Persistent configuration storage (512 bytes)
- String storage (device ID, WiFi credentials, timestamps)
- Integer storage (boot counter, calibration values)
- JSON configuration serialization
- Memory map organization
- Data persistence across power cycles
- EEPROM clear functionality

### DAC Waveform Generator (NEW in v0.1.3-beta)

```bash
python dac_waveform_generator.py
```

Hardware needed: Speaker/amplifier through capacitor (10µF) on GPIO 25/26

This example demonstrates:
- Sine wave generation (smooth audio tones)
- Square wave generation (digital signals)
- Triangle wave generation
- Sawtooth wave generation
- Musical note generation (C4-C5 scale)
- Melody playback
- Frequency sweep (200Hz - 2000Hz)
- Dual DAC simultaneous output

### Multi-Sensor Dashboard (NEW in v0.1.3-beta)

```bash
python multi_sensor_dashboard.py
```

Hardware needed: 
- Temperature sensor (analog) on GPIO 34
- Light sensor (analog) on GPIO 35
- Moisture sensor (analog) on GPIO 36
- Motion sensor (PIR) on GPIO 39
- Status LED on GPIO 2, Buzzer on GPIO 4

This example demonstrates:
- Real-time monitoring of multiple sensors
- Statistical analysis (average, min, max, standard deviation)
- Alert generation with configurable thresholds
- Visual/audio alert indicators
- Data logging and history tracking (last 100 readings)
- Formatted dashboard display
- Motion detection counting

### Ultrasonic Distance Meter (NEW in v0.1.3-beta)

```bash
python ultrasonic_distance_meter.py
```

Hardware needed:
- HC-SR04 ultrasonic sensor (TRIG: GPIO 5, ECHO: GPIO 18 via voltage divider)
- Green LED on GPIO 2, Yellow LED on GPIO 4, Red LED on GPIO 12
- Buzzer on GPIO 13

This example demonstrates:
- Distance measurement with HC-SR04 sensor (2-400cm range)
- Real-time distance monitoring (5 readings/second)
- Visual feedback based on distance zones
- Proximity alarm with buzzer
- Statistical analysis (average, min, max, std deviation)
- Measurement validation and error handling
- Software-based pulse timing

### Digital Thermometer with Logger (NEW in v0.1.3-beta)

```bash
python digital_thermometer_logger.py
```

Hardware needed:
- TMP36 temperature sensor on GPIO 34
- Blue LED on GPIO 2, Red LED on GPIO 4
- Buzzer on GPIO 5

This example demonstrates:
- TMP36 temperature sensor reading (-40°C to +125°C)
- Real-time temperature display (Celsius and Fahrenheit)
- Configurable alert thresholds (high/low temperature)
- Visual status indicators (blue=normal, red=alert)
- Statistical analysis with moving average
- Data logging to memory
- CSV export functionality
- Professional formatted output display

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
Connected to ESP32. Firmware version: 0.1.3-beta

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
