"""
Pin management utilities for ESP32 GPIO Bridge.

This module provides utilities for managing ESP32 GPIO pin configurations,
common pin mappings, and validation helpers.
"""

from typing import Dict, List, Optional, Set, Tuple
from dataclasses import dataclass


@dataclass
class PinCapabilities:
    """Defines capabilities of an ESP32 GPIO pin."""
    pin: int
    can_digital_read: bool = True
    can_digital_write: bool = True
    can_analog_read: bool = False
    can_analog_write: bool = False
    can_pwm: bool = False
    is_touch: bool = False
    is_rtc: bool = False
    has_pullup: bool = True
    has_pulldown: bool = True
    special_functions: List[str] = None

    def __post_init__(self):
        if self.special_functions is None:
            self.special_functions = []


class ESP32PinManager:
    """
    Manages ESP32 GPIO pin configurations and capabilities.

    Provides utilities for pin validation, capability checking,
    and common ESP32 pin configurations.
    """

    # ESP32 pin capabilities mapping
    PIN_CAPABILITIES = {
        # DAC pins (8-bit, 0-3.3V range) - DAC-only pins
        25: PinCapabilities(25, can_digital_read=False, can_digital_write=False, can_analog_write=True),
        26: PinCapabilities(26, can_digital_read=False, can_digital_write=False, can_analog_write=True),

        # ADC1 pins (12-bit, 0-3.3V range)
        # Pins 32-35: Can do both digital I/O and analog input
        32: PinCapabilities(32, can_digital_read=True, can_digital_write=True, can_analog_read=True, can_pwm=True),
        33: PinCapabilities(33, can_digital_read=True, can_digital_write=True, can_analog_read=True, can_pwm=True),
        34: PinCapabilities(34, can_digital_read=True, can_digital_write=True, can_analog_read=True),
        35: PinCapabilities(35, can_digital_read=True, can_digital_write=True, can_analog_read=True),
        # Pins 36-39: ADC-only (input-only, cannot do digital I/O)
        36: PinCapabilities(36, can_digital_read=False, can_digital_write=False, can_analog_read=True),
        37: PinCapabilities(37, can_digital_read=False, can_digital_write=False, can_analog_read=True),
        38: PinCapabilities(38, can_digital_read=False, can_digital_write=False, can_analog_read=True),
        39: PinCapabilities(39, can_digital_read=False, can_digital_write=False, can_analog_read=True),


        # RTC GPIO pins (for ultra-low power)
        0: PinCapabilities(0, can_digital_read=True, can_digital_write=True, is_rtc=True),
        2: PinCapabilities(2, can_digital_read=True, can_digital_write=True, is_rtc=True),
        4: PinCapabilities(4, can_digital_read=True, can_digital_write=True, is_rtc=True),
        12: PinCapabilities(12, can_digital_read=True, can_digital_write=True, is_rtc=True),
        13: PinCapabilities(13, can_digital_read=True, can_digital_write=True, is_rtc=True),
        14: PinCapabilities(14, can_digital_read=True, can_digital_write=True, is_rtc=True),
        15: PinCapabilities(15, can_digital_read=True, can_digital_write=True, is_rtc=True),
        25: PinCapabilities(25, can_digital_read=False, can_digital_write=False, can_analog_write=True, is_rtc=True),
        26: PinCapabilities(26, can_digital_read=False, can_digital_write=False, can_analog_write=True, is_rtc=True),
        27: PinCapabilities(27, can_digital_read=True, can_digital_write=True, is_rtc=True),
        32: PinCapabilities(32, can_digital_read=True, can_digital_write=True, can_analog_read=True, is_rtc=True),
        33: PinCapabilities(33, can_digital_read=True, can_digital_write=True, can_analog_read=True, is_rtc=True),
        34: PinCapabilities(34, can_digital_read=True, can_digital_write=True, can_analog_read=True, is_rtc=True),
        35: PinCapabilities(35, can_digital_read=True, can_digital_write=True, can_analog_read=True, is_rtc=True),
        36: PinCapabilities(36, can_digital_read=False, can_digital_write=False, can_analog_read=True, is_rtc=True),
        37: PinCapabilities(37, can_digital_read=False, can_digital_write=False, can_analog_read=True, is_rtc=True),
        38: PinCapabilities(38, can_digital_read=False, can_digital_write=False, can_analog_read=True, is_rtc=True),
        39: PinCapabilities(39, can_digital_read=False, can_digital_write=False, can_analog_read=True, is_rtc=True),

        # PWM-capable pins (most output pins support PWM) - exclude DAC and ADC-only pins
        **{pin: PinCapabilities(pin, can_digital_read=True, can_digital_write=True, can_pwm=True) for pin in [
            0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
            21, 22, 23, 27  # Excluded: 25, 26 (DAC), 32, 33 (ADC with digital I/O)
        ]},

        # I2C default pins (can be remapped)
        21: PinCapabilities(21, can_digital_read=True, can_digital_write=True, can_pwm=True, special_functions=['I2C_SDA']),
        22: PinCapabilities(22, can_digital_read=True, can_digital_write=True, can_pwm=True, special_functions=['I2C_SCL']),

        # SPI default pins (can be remapped)
        18: PinCapabilities(18, can_digital_read=True, can_digital_write=True, can_pwm=True, special_functions=['SPI_CLK']),
        19: PinCapabilities(19, can_digital_read=True, can_digital_write=True, can_pwm=True, special_functions=['SPI_MISO']),
        23: PinCapabilities(23, can_digital_read=True, can_digital_write=True, can_pwm=True, special_functions=['SPI_MOSI']),
        5: PinCapabilities(5, can_digital_read=True, can_digital_write=True, can_pwm=True, special_functions=['SPI_CS']),

        # Touch sensor pins (defined last to override previous definitions)
        4: PinCapabilities(4, can_digital_read=True, can_digital_write=True, can_pwm=True, is_touch=True, is_rtc=True, special_functions=['TOUCH0']),
        0: PinCapabilities(0, can_digital_read=True, can_digital_write=True, can_pwm=True, is_touch=True, is_rtc=True, special_functions=['TOUCH1']),
        2: PinCapabilities(2, can_digital_read=True, can_digital_write=True, can_pwm=True, is_touch=True, is_rtc=True, special_functions=['TOUCH2']),
        15: PinCapabilities(15, can_digital_read=True, can_digital_write=True, can_pwm=True, is_touch=True, is_rtc=True, special_functions=['TOUCH3']),
        13: PinCapabilities(13, can_digital_read=True, can_digital_write=True, can_pwm=True, is_touch=True, is_rtc=True, special_functions=['TOUCH4']),
        12: PinCapabilities(12, can_digital_read=True, can_digital_write=True, can_pwm=True, is_touch=True, is_rtc=True, special_functions=['TOUCH5']),
        14: PinCapabilities(14, can_digital_read=True, can_digital_write=True, can_pwm=True, is_touch=True, is_rtc=True, special_functions=['TOUCH6']),
        27: PinCapabilities(27, can_digital_read=True, can_digital_write=True, can_pwm=True, is_touch=True, is_rtc=True, special_functions=['TOUCH7']),
    }

    # Add default capabilities for any pins not explicitly defined above
    for pin in range(0, 40):
        if pin not in PIN_CAPABILITIES:
            PIN_CAPABILITIES[pin] = PinCapabilities(pin, can_digital_read=True, can_digital_write=True)

    @classmethod
    def get_pin_capabilities(cls, pin: int) -> PinCapabilities:
        """
        Get capabilities for a specific pin.

        Args:
            pin: GPIO pin number

        Returns:
            PinCapabilities object for the pin

        Raises:
            ValueError: If pin number is invalid
        """
        if not 0 <= pin < 40:
            raise ValueError(f"Invalid pin number: {pin}. Must be 0-39.")

        return cls.PIN_CAPABILITIES.get(pin, PinCapabilities(pin))

    @classmethod
    def is_valid_pin(cls, pin: int) -> bool:
        """Check if pin number is valid."""
        return 0 <= pin < 40

    @classmethod
    def can_analog_read(cls, pin: int) -> bool:
        """Check if pin supports analog reading."""
        return cls.get_pin_capabilities(pin).can_analog_read

    @classmethod
    def can_analog_write(cls, pin: int) -> bool:
        """Check if pin supports analog writing (DAC)."""
        return cls.get_pin_capabilities(pin).can_analog_write

    @classmethod
    def can_digital_read(cls, pin: int) -> bool:
        """Check if pin supports digital reading."""
        return cls.get_pin_capabilities(pin).can_digital_read

    @classmethod
    def can_digital_write(cls, pin: int) -> bool:
        """Check if pin supports digital writing."""
        return cls.get_pin_capabilities(pin).can_digital_write

    @classmethod
    def get_adc_pins(cls) -> List[int]:
        """Get list of pins that support analog reading."""
        return [pin for pin, caps in cls.PIN_CAPABILITIES.items()
                if caps.can_analog_read]

    @classmethod
    def get_dac_pins(cls) -> List[int]:
        """Get list of pins that support analog writing (DAC)."""
        return [pin for pin, caps in cls.PIN_CAPABILITIES.items()
                if caps.can_analog_write]

    @classmethod
    def get_touch_pins(cls) -> List[int]:
        """Get list of pins that support touch sensing."""
        return [pin for pin, caps in cls.PIN_CAPABILITIES.items()
                if caps.is_touch]

    @classmethod
    def get_rtc_pins(cls) -> List[int]:
        """Get list of RTC GPIO pins."""
        return [pin for pin, caps in cls.PIN_CAPABILITIES.items()
                if caps.is_rtc]


def validate_pin_for_mode(pin: int, mode: str) -> None:
    """
    Validate that a pin supports the requested mode.

    Args:
        pin: GPIO pin number
        mode: Pin mode ('IN', 'OUT', 'IN_PULLUP', 'AREAD', 'AWRITE')

    Raises:
        ValueError: If pin doesn't support the requested mode
    """
    caps = ESP32PinManager.get_pin_capabilities(pin)
    mode = mode.upper()

    if mode in ['IN', 'IN_PULLUP', 'OUT']:
        if mode in ['IN', 'IN_PULLUP'] and not caps.can_digital_read:
            raise ValueError(f"Pin {pin} does not support digital input")
        elif mode == 'OUT' and not caps.can_digital_write:
            raise ValueError(f"Pin {pin} does not support digital output")
    elif mode == 'AREAD' and not caps.can_analog_read:
        raise ValueError(f"Pin {pin} does not support analog reading")
    elif mode == 'AWRITE' and not caps.can_analog_write:
        raise ValueError(f"Pin {pin} does not support analog writing")


# Common ESP32 development board pin mappings
ESP32_DEVKIT_V1_PINOUT = {
    'D0': 0, 'D1': 1, 'D2': 2, 'D3': 3, 'D4': 4, 'D5': 5,
    'D6': 6, 'D7': 7, 'D8': 8, 'D9': 9, 'D10': 10,
    'D11': 11, 'D12': 12, 'D13': 13, 'D14': 14, 'D15': 15,
    'RX': 16, 'TX': 17, 'RX2': 16, 'TX2': 17,
    'A0': 36, 'A1': 37, 'A2': 38, 'A3': 39, 'A4': 32, 'A5': 33,
    'A6': 34, 'A7': 35,
    'DAC1': 25, 'DAC2': 26,
    'SVP': 36, 'SVN': 39,
    'SENSOR_VP': 36, 'SENSOR_VN': 39,
}

WEMOS_D1_ESP32_PINOUT = {
    'D0': 0, 'D1': 1, 'D2': 2, 'D3': 3, 'D4': 4, 'D5': 5,
    'D6': 6, 'D7': 7, 'D8': 8, 'D9': 9, 'D10': 10,
    'D11': 11, 'D12': 12, 'D13': 13, 'D14': 14, 'D15': 15,
    'RX': 16, 'TX': 17,
    'A0': 36, 'A1': 39, 'A2': 34, 'A3': 35, 'A4': 32, 'A5': 33,
    'SCL': 22, 'SDA': 21,
    'SCLK': 18, 'MISO': 19, 'MOSI': 23, 'CS': 5,
}
