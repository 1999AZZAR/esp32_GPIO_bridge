"""
Configuration management for ESP32 GPIO Bridge.

This module provides configuration management, presets, and settings
for common ESP32 GPIO Bridge use cases.
"""

import json
import os
from typing import Dict, Any, Optional
from pathlib import Path


class ESP32Config:
    """
    Configuration manager for ESP32 GPIO Bridge settings.

    Provides default configurations, presets, and custom configuration
    management for different ESP32 GPIO Bridge applications.
    """

    DEFAULT_CONFIG = {
        'serial': {
            'baudrate': 115200,
            'timeout': 1.0,
            'auto_connect': True
        },
        'logging': {
            'level': 'INFO',
            'format': '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
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
        },
        'i2s': {
            'default_sample_rate': 44100,
            'default_bits_per_sample': 16
        },
        'failsafe': {
            'timeout': 2.0,
            'enabled': True
        }
    }

    COMMON_PRESETS = {
        'basic_io': {
            'description': 'Basic digital and analog I/O operations',
            'pins': {
                'led': 2,
                'button': 0,
                'potentiometer': 34,
                'temperature_sensor': 35
            }
        },
        'sensor_hub': {
            'description': 'Multiple sensors and actuators',
            'pins': {
                'dht22': {'data': 4},
                'bmp280': {'sda': 21, 'scl': 22},
                'servo': 13,
                'relay': 12,
                'adc1': 36,
                'adc2': 39
            }
        },
        'audio_output': {
            'description': 'Audio output and simple LED control',
            'pins': {
                'i2s_bck': 26,
                'i2s_ws': 25,
                'i2s_data': 22,
                'mute_led': 2,
                'volume_pot': 34
            },
            'i2s': {
                'sample_rate': 44100,
                'bits_per_sample': 16
            }
        },
        'robotics': {
            'description': 'Robotics and motor control',
            'pins': {
                'left_motor_pwm': 16,
                'right_motor_pwm': 17,
                'left_motor_dir': 18,
                'right_motor_dir': 19,
                'ultrasonic_trig': 5,
                'ultrasonic_echo': 4,
                'line_sensor_left': 36,
                'line_sensor_right': 39,
                'servo_pan': 13,
                'servo_tilt': 12
            }
        }
    }

    def __init__(self, config_path: Optional[str] = None):
        """
        Initialize configuration manager.

        Args:
            config_path: Path to custom configuration file (optional)
        """
        self.config_path = config_path or self._get_default_config_path()
        self._config = self.DEFAULT_CONFIG.copy()
        self._custom_presets: Dict[str, Dict[str, Any]] = {}

        # Load custom configuration if it exists
        if os.path.exists(self.config_path):
            self.load_config()

    def _get_default_config_path(self) -> str:
        """Get default configuration file path."""
        home = Path.home()
        config_dir = home / '.esp32_gpio_bridge'
        config_dir.mkdir(exist_ok=True)
        return str(config_dir / 'config.json')

    def load_config(self) -> None:
        """Load configuration from file."""
        try:
            with open(self.config_path, 'r') as f:
                loaded_config = json.load(f)
                # Merge with defaults, allowing custom values to override
                self._config.update(loaded_config)
        except (FileNotFoundError, json.JSONDecodeError) as e:
            print(f"Warning: Could not load config from {self.config_path}: {e}")

    def save_config(self) -> None:
        """Save current configuration to file."""
        os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
        with open(self.config_path, 'w') as f:
            json.dump(self._config, f, indent=2)

    def get_config(self, section: Optional[str] = None) -> Dict[str, Any]:
        """
        Get configuration values.

        Args:
            section: Configuration section name (optional)

        Returns:
            Configuration dictionary
        """
        if section:
            return self._config.get(section, {})  # type: ignore
        return self._config

    def update_config(self, section: str, key: str, value: Any) -> None:
        """
        Update a configuration value.

        Args:
            section: Configuration section name
            key: Configuration key
            value: New value
        """
        if section not in self._config:
            self._config[section] = {}  # type: ignore
        self._config[section][key] = value  # type: ignore
        self.save_config()

    def get_preset(self, preset_name: str) -> Optional[Dict[str, Any]]:
        """
        Get a preset configuration.

        Args:
            preset_name: Name of the preset

        Returns:
            Preset configuration dictionary or None if not found
        """
        return self.COMMON_PRESETS.get(preset_name, self._custom_presets.get(preset_name))

    def add_custom_preset(self, name: str, config: Dict[str, Any], description: str = "") -> None:
        """
        Add a custom preset configuration.

        Args:
            name: Preset name
            config: Preset configuration
            description: Optional description
        """
        self._custom_presets[name] = {
            'description': description,
            **config
        }
        self.save_config()

    def list_presets(self) -> Dict[str, str]:
        """
        List all available presets.

        Returns:
            Dictionary of preset names and descriptions
        """
        presets: Dict[str, str] = {}
        for name, preset in self.COMMON_PRESETS.items():
            presets[name] = preset.get('description', '')  # type: ignore
        for name, preset in self._custom_presets.items():
            presets[name] = preset.get('description', '')  # type: ignore
        return presets

    def apply_preset(self, preset_name: str) -> Dict[str, Any]:
        """
        Apply a preset configuration.

        Args:
            preset_name: Name of the preset to apply

        Returns:
            Applied configuration

        Raises:
            ValueError: If preset not found
        """
        preset = self.get_preset(preset_name)
        if not preset:
            raise ValueError(f"Preset '{preset_name}' not found")

        # Apply preset (this would typically modify the ESP32GPIO instance)
        return preset

    def get_pin_config(self, pin_type: str) -> Dict[str, Any]:
        """
        Get pin configuration for a specific type.

        Args:
            pin_type: Type of pin configuration ('common', 'i2c', 'i2s')

        Returns:
            Pin configuration dictionary
        """
        return self._config.get('pins', {}).get(pin_type, {})  # type: ignore

    def set_pin_config(self, pin_type: str, **kwargs) -> None:
        """
        Set pin configuration for a specific type.

        Args:
            pin_type: Type of pin configuration
            **kwargs: Pin configuration parameters
        """
        if 'pins' not in self._config:
            self._config['pins'] = {}  # type: ignore
        if pin_type not in self._config['pins']:  # type: ignore
            self._config['pins'][pin_type] = {}  # type: ignore

        self._config['pins'][pin_type].update(kwargs)  # type: ignore
        self.save_config()


# Global configuration instance
_config_instance = None

def get_config() -> ESP32Config:
    """Get global configuration instance."""
    global _config_instance
    if _config_instance is None:
        _config_instance = ESP32Config()
    return _config_instance


def set_config_path(config_path: str) -> None:
    """Set custom configuration file path."""
    global _config_instance
    _config_instance = ESP32Config(config_path)
