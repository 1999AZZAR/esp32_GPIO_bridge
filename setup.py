"""
Setup configuration for ESP32 GPIO Bridge Python library.

This setup.py file configures the ESP32 GPIO Bridge library for installation
via pip and other Python package managers.
"""

from setuptools import setup, find_packages
from pathlib import Path

# Read the README file for the long description
readme_file = Path(__file__).parent / "README.md"
long_description = ""
if readme_file.exists():
    long_description = readme_file.read_text(encoding="utf-8")

# Read requirements from requirements.txt if it exists
requirements_file = Path(__file__).parent / "requirements.txt"
requirements = []
if requirements_file.exists():
    requirements = requirements_file.read_text(encoding="utf-8").strip().split('\n')

setup(
    name="esp32-gpio-bridge",
    version="0.1.3-beta",
    author="ESP32 GPIO Bridge Project",
    author_email="info@esp32-gpio-bridge.org",
    description="Python library for controlling ESP32 GPIO pins via USB serial connection",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/esp32-gpio-bridge/esp32-gpio-bridge",
    project_urls={
        "Documentation": "https://esp32-gpio-bridge.readthedocs.io/",
        "Source": "https://github.com/esp32-gpio-bridge/esp32-gpio-bridge",
        "Tracker": "https://github.com/esp32-gpio-bridge/esp32-gpio-bridge/issues",
    },
    packages=find_packages(),
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: Education",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Topic :: Scientific/Engineering :: Interface Engine/Protocol Translator",
        "Topic :: System :: Hardware :: Hardware Drivers",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ],
    python_requires=">=3.6",
    install_requires=requirements,
    extras_require={
        "dev": [
            "pytest>=6.0",
            "pytest-cov",
            "black",
            "flake8",
            "mypy",
        ],
        "examples": [
            "matplotlib",
            "numpy",
        ],
    },
    entry_points={
        "console_scripts": [
            "esp32-gpio-demo=esp32_gpio_bridge.controller:main",
        ],
    },
    package_data={
        "esp32_gpio_bridge": ["*.json", "*.yaml", "*.yml"],
    },
    include_package_data=True,
    keywords=[
        "esp32",
        "gpio",
        "serial",
        "embedded",
        "microcontroller",
        "arduino",
        "raspberry-pi",
        "iot",
        "hardware",
        "electronics",
    ],
    license="MIT",
    zip_safe=False,
)
