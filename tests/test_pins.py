"""
Tests for ESP32 GPIO Bridge pin management functionality.
"""

import pytest
from esp32_gpio_bridge import pins


class TestPinCapabilities:
    """Test pin capability detection."""

    def test_get_pin_capabilities_valid_pin(self):
        """Test getting capabilities for a valid pin."""
        caps = pins.ESP32PinManager.get_pin_capabilities(2)
        assert caps.pin == 2
        assert caps.can_digital_read is True
        assert caps.can_digital_write is True

    def test_get_pin_capabilities_invalid_pin(self):
        """Test getting capabilities for an invalid pin."""
        with pytest.raises(ValueError, match="Invalid pin number"):
            pins.ESP32PinManager.get_pin_capabilities(-1)

        with pytest.raises(ValueError, match="Invalid pin number"):
            pins.ESP32PinManager.get_pin_capabilities(50)

    def test_get_pin_capabilities_adc_pin(self):
        """Test ADC pin capabilities."""
        caps = pins.ESP32PinManager.get_pin_capabilities(34)
        assert caps.pin == 34
        assert caps.can_analog_read is True
        assert caps.can_digital_read is True  # ADC pins 32-35 can do digital I/O

    def test_get_pin_capabilities_dac_pin(self):
        """Test DAC pin capabilities."""
        caps = pins.ESP32PinManager.get_pin_capabilities(25)
        assert caps.pin == 25
        assert caps.can_analog_write is True
        assert caps.can_digital_read is False  # DAC-only pins

    def test_get_pin_capabilities_adc_only_pin(self):
        """Test ADC-only pin capabilities (pins 36-39)."""
        caps = pins.ESP32PinManager.get_pin_capabilities(36)
        assert caps.pin == 36
        assert caps.can_analog_read is True
        assert caps.can_digital_read is False  # ADC-only pins cannot do digital I/O
        assert caps.can_digital_write is False

    def test_is_valid_pin(self):
        """Test pin validation."""
        assert pins.ESP32PinManager.is_valid_pin(0) is True
        assert pins.ESP32PinManager.is_valid_pin(39) is True
        assert pins.ESP32PinManager.is_valid_pin(-1) is False
        assert pins.ESP32PinManager.is_valid_pin(40) is False

    def test_can_analog_read(self):
        """Test analog read capability checking."""
        assert pins.ESP32PinManager.can_analog_read(34) is True
        assert pins.ESP32PinManager.can_analog_read(2) is False

    def test_can_analog_write(self):
        """Test analog write capability checking."""
        assert pins.ESP32PinManager.can_analog_write(25) is True
        assert pins.ESP32PinManager.can_analog_write(26) is True
        assert pins.ESP32PinManager.can_analog_write(2) is False

    def test_get_adc_pins(self):
        """Test getting list of ADC-capable pins."""
        adc_pins = pins.ESP32PinManager.get_adc_pins()
        assert 32 in adc_pins
        assert 33 in adc_pins
        assert 34 in adc_pins
        assert 35 in adc_pins
        assert 36 in adc_pins
        assert 37 in adc_pins
        assert 38 in adc_pins
        assert 39 in adc_pins
        assert 2 not in adc_pins

    def test_get_dac_pins(self):
        """Test getting list of DAC-capable pins."""
        dac_pins = pins.ESP32PinManager.get_dac_pins()
        assert 25 in dac_pins
        assert 26 in dac_pins
        assert len(dac_pins) == 2

    def test_get_touch_pins(self):
        """Test getting list of touch-capable pins."""
        touch_pins = pins.ESP32PinManager.get_touch_pins()
        assert 4 in touch_pins  # TOUCH0
        assert 0 in touch_pins  # TOUCH1
        assert len(touch_pins) > 0


class TestPinValidation:
    """Test pin validation functions."""

    def test_validate_pin_for_mode_valid(self):
        """Test validation with valid pin/mode combinations."""
        # Should not raise for valid combinations
        pins.validate_pin_for_mode(2, "OUT")
        pins.validate_pin_for_mode(34, "AREAD")
        pins.validate_pin_for_mode(25, "AWRITE")

    def test_validate_pin_for_mode_invalid_pin(self):
        """Test validation with invalid pin."""
        with pytest.raises(ValueError, match="Invalid pin number"):
            pins.validate_pin_for_mode(-1, "OUT")

    def test_validate_pin_for_mode_invalid_mode(self):
        """Test validation with invalid mode."""
        with pytest.raises(ValueError, match="does not support digital output"):
            pins.validate_pin_for_mode(36, "OUT")  # ADC-only pin can't do digital output

        with pytest.raises(ValueError, match="does not support analog reading"):
            pins.validate_pin_for_mode(2, "AREAD")  # Regular pin can't do analog read


class TestPinoutMappings:
    """Test ESP32 development board pin mappings."""

    def test_esp32_devkit_v1_pinout(self):
        """Test ESP32 DevKit v1 pin mappings."""
        assert pins.ESP32_DEVKIT_V1_PINOUT['D2'] == 2
        assert pins.ESP32_DEVKIT_V1_PINOUT['A0'] == 36
        assert pins.ESP32_DEVKIT_V1_PINOUT['DAC1'] == 25

    def test_wemos_d1_esp32_pinout(self):
        """Test WeMos D1 ESP32 pin mappings."""
        assert pins.WEMOS_D1_ESP32_PINOUT['D2'] == 2
        assert pins.WEMOS_D1_ESP32_PINOUT['A0'] == 36
        assert pins.WEMOS_D1_ESP32_PINOUT['SCL'] == 22
