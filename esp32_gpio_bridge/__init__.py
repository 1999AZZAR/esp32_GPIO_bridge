"""
ESP32 GPIO Bridge - Python Library for ESP32 GPIO Control

This library provides a comprehensive Python interface for controlling ESP32 GPIO pins,
analog I/O, I2C communication, and I2S audio from a host computer via USB serial connection.

Main Classes:
    ESP32GPIO: Main controller class for ESP32 GPIO operations

Utility Modules:
    pins: Pin management and validation utilities
    config: Configuration management and presets

Utility Functions:
    list_serial_ports: List available serial ports
    select_port: Interactive port selection
    find_esp32_port: Auto-detect ESP32 port

Example:
    >>> from esp32_gpio_bridge import ESP32GPIO
    >>>
    >>> with ESP32GPIO('/dev/ttyUSB0') as esp:
    ...     esp.set_pin_mode(2, 'OUT')
    ...     esp.digital_write(2, 1)
    ...     value = esp.analog_read(34)
"""

from .controller import (
    ESP32GPIO,
    list_serial_ports,
    select_port,
    find_esp32_port
)
from . import pins
from . import config

__version__ = "0.1.1-beta"
__author__ = "ESP32 GPIO Bridge Project"
__all__ = [
    'ESP32GPIO',
    'list_serial_ports',
    'select_port',
    'find_esp32_port',
    'pins',
    'config'
]
