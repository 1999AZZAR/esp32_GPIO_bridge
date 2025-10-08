"""
ESP32 GPIO Bridge - Python Library for ESP32 GPIO Control

This module provides a comprehensive Python interface for controlling ESP32 GPIO pins,
analog I/O, I2C communication, and I2S audio from a host computer via USB serial connection.

Author: ESP32 GPIO Bridge Project
Version: 0.1.1-beta
"""

import serial
import serial.tools.list_ports
import time
import threading
from queue import Queue, Empty
from typing import List, Tuple, Optional, Union
import logging


class ESP32GPIO:
    """
    ESP32 GPIO Bridge controller class.

    Provides a high-level interface for controlling ESP32 GPIO pins, analog I/O,
    I2C communication, and I2S audio from a host computer via USB serial connection.

    Args:
        port: Serial port device path (e.g., '/dev/ttyUSB0', 'COM5')
        baudrate: Serial communication speed (default: 115200)
        timeout: Serial timeout in seconds (default: 1)
        auto_connect: Whether to automatically connect on initialization (default: True)

    Example:
        >>> esp = ESP32GPIO('/dev/ttyUSB0')
        >>> esp.set_pin_mode(2, 'OUT')
        >>> esp.digital_write(2, 1)
        >>> esp.close()
    """

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1, auto_connect: bool = True):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        self.lock = threading.Lock()
        self.response_queue = Queue()
        self._is_running = False

        # Configuration tracking
        self.configured_pins: List[int] = []
        self.adc_initialized = False
        self.i2c_initialized = False
        self.i2s_initialized = False

        # Logging setup
        self.logger = logging.getLogger(f"ESP32GPIO.{port}")

        if auto_connect:
            self.connect()

    def connect(self) -> None:
        """Establish connection to the ESP32 GPIO Bridge."""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            self._is_running = True

            self.read_thread = threading.Thread(target=self._read_from_port, daemon=True)
            self.read_thread.start()

            # Wait for ESP32 to initialize
            self.logger.info("Waiting for ESP32 to initialize...")
            time.sleep(3)

            # Initialize connection and get version
            self._initialize_connection()

            # Start ping thread for failsafe
            self.ping_thread = threading.Thread(target=self._send_pings, daemon=True)
            self.ping_thread.start()

            self.logger.info(f"Connected to ESP32 GPIO Bridge on {self.port}")

        except serial.SerialException as e:
            raise IOError(f"Failed to connect to ESP32 on {self.port}: {e}")

    def _initialize_connection(self) -> None:
        """Initialize connection and verify ESP32 readiness."""
        max_attempts = 10
        ready_found = False

        for attempt in range(max_attempts):
            try:
                response = self.get_response(timeout=1)
                if 'Ready' in response:
                    ready_found = True
                    # Get version message
                    try:
                        self.get_response(timeout=0.5)
                    except TimeoutError:
                        pass  # Version message might not be present
                    break
            except TimeoutError:
                if attempt < max_attempts - 1:
                    self.logger.debug(f"Attempt {attempt + 1}/{max_attempts}... retrying")
                    time.sleep(0.5)

        if not ready_found:
            raise IOError("Failed to connect to ESP32. Check port and firmware.")

    def _read_from_port(self) -> None:
        """Background thread to read serial responses."""
        while self._is_running and self.ser:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.response_queue.put(line)
            except (serial.SerialException, TypeError, AttributeError):
                self._is_running = False
                break

    def get_response(self, timeout: float = 2) -> str:
        """
        Get response from ESP32.

        Args:
            timeout: Maximum time to wait for response in seconds

        Returns:
            Response string from ESP32

        Raises:
            TimeoutError: If no response received within timeout
        """
        try:
            response = self.response_queue.get(timeout=timeout)
            if response.startswith('<') and response.endswith('>'):
                return response[1:-1]
            return response
        except Empty:
            raise TimeoutError("No response from ESP32.")

    def _send_command(self, command: str, expect_response: bool = True) -> Optional[str]:
        """
        Send command to ESP32 and optionally wait for response.

        Args:
            command: Command string to send
            expect_response: Whether to wait for and return response

        Returns:
            Response string if expect_response is True, otherwise None

        Raises:
            ValueError: If ESP32 returns an error response
        """
        if not self.ser or not self._is_running:
            raise IOError("ESP32 not connected")

        with self.lock:
            # Clear any pending responses
            while not self.response_queue.empty():
                try:
                    self.response_queue.get_nowait()
                except Empty:
                    break

            self.ser.write(f"<{command}>".encode('utf-8'))

            if expect_response:
                response = self.get_response()
                if response.startswith("ERROR"):
                    raise ValueError(f"ESP32 Error: {response}")
                return response
            return None

    def _send_pings(self) -> None:
        """Background thread to send periodic pings for failsafe."""
        while self._is_running:
            try:
                if self.ser:
                    with self.lock:
                        self.ser.write(b"<PING>")
                time.sleep(1)
            except (serial.SerialException, OSError, AttributeError):
                self._is_running = False
                break



def list_serial_ports() -> List[Tuple[str, str]]:
    """
    List all available serial ports.

    Returns:
        List of tuples containing (device_path, description) for each port
    """
    ports = serial.tools.list_ports.comports()
    return [(port.device, port.description) for port in ports]


def select_port() -> Optional[str]:
    """
    Interactive port selection menu.

    Returns:
        Selected port device path or None if cancelled
    """
    ports = list_serial_ports()

    if not ports:
        print("No serial ports found!")
        return None

    print("\n=== Available Serial Ports ===")
    for idx, (device, description) in enumerate(ports, 1):
        print(f"{idx}. {device} - {description}")

    while True:
        try:
            choice = input(f"\nSelect port (1-{len(ports)}) or 'q' to quit: ").strip()
            if choice.lower() == 'q':
                return None

            idx = int(choice) - 1
            if 0 <= idx < len(ports):
                selected_port = ports[idx][0]
                print(f"Selected: {selected_port}")
                return selected_port
            else:
                print(f"Invalid selection. Please enter a number between 1 and {len(ports)}.")
        except ValueError:
            print("Invalid input. Please enter a number or 'q' to quit.")


def find_esp32_port() -> Optional[str]:
    """
    Auto-detect ESP32 port by looking for common ESP32 device descriptions.

    Returns:
        ESP32 port device path or None if not found
    """
    ports = list_serial_ports()

    # Common ESP32 device identifiers
    esp32_identifiers = [
        'esp32', 'esp32-s2', 'esp32-s3', 'esp32-c3', 'esp32-c6',
        'silicon labs', 'cp210x', 'ch340', 'ftdi'
    ]

    for device, description in ports:
        desc_lower = description.lower()
        if any(identifier in desc_lower for identifier in esp32_identifiers):
            return device

    return None


# Example usage and demo
if __name__ == "__main__":
    # Configure logging
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

    # Auto-detect ESP32 port or select manually
    esp32_port = find_esp32_port()

    if not esp32_port:
        print("ESP32 not auto-detected. Please select manually.")
        esp32_port = select_port()

    if not esp32_port:
        print("No port selected. Exiting.")
        exit(0)

    esp = None
    try:
        print(f"\nConnecting to {esp32_port}...")
        with ESP32GPIO(esp32_port) as esp:
            print(f"Firmware Version: {esp.get_version()}")

            # Demo: Blink LED on GPIO 2
            led_pin = 2
            print(f"\nBlinking LED on GPIO {led_pin}...")
            esp.set_pin_mode(led_pin, "OUT")
            for _ in range(3):
                esp.digital_write(led_pin, 1)
                time.sleep(0.2)
                esp.digital_write(led_pin, 0)
                time.sleep(0.2)

            # Demo: Read analog values from GPIO 34
            pot_pin = 34
            print(f"\nReading analog values from GPIO {pot_pin}...")
            for _ in range(5):
                raw_value = esp.analog_read(pot_pin)
                voltage = (raw_value / 4095.0) * 3.3
                print(f"Raw: {raw_value:4d}, Voltage: {voltage:.2f}V")
                time.sleep(0.5)

            print(f"\nConfigured pins: {esp.get_configured_pins()}")

    except serial.SerialException as e:
        print(f"Error: Could not open port {esp32_port}. {e}")
    except KeyboardInterrupt:
        print("\nExiting due to user interrupt.")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
