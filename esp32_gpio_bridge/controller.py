"""
ESP32 GPIO Bridge - Python Library for ESP32 GPIO Control

This module provides a comprehensive Python interface for controlling ESP32 GPIO pins,
analog I/O, I2C communication, I2S audio, PWM, and EEPROM from a host computer via USB serial connection.

Author: ESP32 GPIO Bridge Project
Version: 0.1.3-beta
"""

import serial
import serial.tools.list_ports
import time
import threading
from queue import Queue, Empty
from typing import List, Tuple, Optional, Union, Dict, Any
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
        self.response_queue: Queue[str] = Queue()
        self._is_running = False

        # Configuration tracking
        self.configured_pins: List[int] = []
        self.adc_initialized = False
        self.i2c_initialized = False
        self.i2s_initialized = False
        self.pwm_channels: Dict[int, int] = {}  # pin -> channel mapping
        self.eeprom_initialized = False

        # Logging setup
        self.logger = logging.getLogger(f"ESP32GPIO.{port}")

        if auto_connect:
            self.connect()

    def connect(self) -> None:
        """Establish connection to the ESP32 GPIO Bridge."""
        try:
            # Open serial port with DTR/RTS disabled to prevent auto-reset
            self.ser = serial.Serial(
                self.port, 
                self.baudrate, 
                timeout=self.timeout,
                dsrdtr=False,  # Disable DTR
                rtscts=False   # Disable RTS
            )
            self._is_running = True

            self.read_thread = threading.Thread(target=self._read_from_port, daemon=True)
            self.read_thread.start()

            # Wait for ESP32 to boot (in case it was auto-reset)
            self.logger.info("Waiting for ESP32 to initialize...")
            time.sleep(0.5)
            
            # Drain any boot messages from buffer
            self._drain_boot_messages(duration=1.0)

            # Initialize connection and get version
            self._initialize_connection()

            # Start ping thread for failsafe
            self.ping_thread = threading.Thread(target=self._send_pings, daemon=True)
            self.ping_thread.start()

            # Initialize failsafe state tracking
            self.failsafe_engaged = False

            self.logger.info(f"Connected to ESP32 GPIO Bridge on {self.port}")

        except serial.SerialException as e:
            raise IOError(f"Failed to connect to ESP32 on {self.port}: {e}")

    def _drain_boot_messages(self, duration: float = 1.0) -> None:
        """
        Drain boot messages and garbage data from serial buffer.
        
        Args:
            duration: How long to drain buffer in seconds
        """
        self.logger.debug("Draining boot messages from buffer...")
        start_time = time.time()
        drained_count = 0
        
        while (time.time() - start_time) < duration:
            try:
                # Clear response queue
                while not self.response_queue.empty():
                    try:
                        self.response_queue.get_nowait()
                        drained_count += 1
                    except Empty:
                        break
                time.sleep(0.05)
            except:
                break
        
        if drained_count > 0:
            self.logger.debug(f"Drained {drained_count} boot messages")

    def _initialize_connection(self) -> None:
        """Initialize connection and verify ESP32 readiness."""
        max_attempts = 5
        
        # Try PING command first (doesn't trigger failsafe)
        for attempt in range(max_attempts):
            try:
                self.logger.debug(f"Sending PING (attempt {attempt + 1}/{max_attempts})...")
                with self.lock:
                    # Clear queue
                    while not self.response_queue.empty():
                        try:
                            self.response_queue.get_nowait()
                        except Empty:
                            break
                    
                    self.ser.write(b"<PING>")
                    time.sleep(0.1)
                    
                    # Try to get PONG response
                    try:
                        response = self.response_queue.get(timeout=1.0)
                        if "PONG" in response:
                            self.logger.debug("Received PONG - ESP32 is ready")
                            return
                    except Empty:
                        pass
                
                time.sleep(0.3)
                
            except Exception as e:
                self.logger.debug(f"Connection attempt {attempt + 1} failed: {e}")
                if attempt < max_attempts - 1:
                    time.sleep(0.5)
        
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
            # Clear any pending responses (including PONG from ping thread)
            while not self.response_queue.empty():
                try:
                    self.response_queue.get_nowait()
                except Empty:
                    break

            self.ser.write(f"<{command}>".encode('utf-8'))

            if expect_response:
                # Filter out PONG, STATUS, and stale ERROR responses from queue
                max_attempts = 10
                for attempt in range(max_attempts):
                    try:
                        response = self.get_response(timeout=0.5)
                        
                        # Skip PONG responses from ping thread
                        if response == "PONG":
                            continue
                        
                        # Skip STATUS responses unless we asked for STATUS
                        if response.startswith("STATUS:") and not command.startswith("STATUS"):
                            continue
                        
                        # Skip stale ERROR messages that don't match current command
                        # (These are from previous write commands that we already handled)
                        if response.startswith("ERROR"):
                            # If this is the first attempt, it might be a real error for this command
                            if attempt == 0:
                                raise ValueError(f"ESP32 Error: {response}")
                            # Otherwise it's likely stale, skip it
                            continue
                        
                        # Return actual response
                        return response  # type: ignore[return-value]
                    except TimeoutError:
                        # If we timeout, the command might not send a response
                        if attempt == 0:
                            raise
                        # Otherwise keep trying to find valid response
                        continue
                
                # If we only got noise, raise error
                raise TimeoutError("No valid response from ESP32 (only got PONG/STATUS/stale errors)")
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

    def __enter__(self) -> 'ESP32GPIO':
        """Context manager entry."""
        if not self.ser:
            self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Context manager exit."""
        self.close()

    def close(self) -> None:
        """Close connection to ESP32."""
        if self._is_running:
            self._is_running = False

            # Wait for threads to finish
            if hasattr(self, 'ping_thread') and self.ping_thread.is_alive():
                self.ping_thread.join(timeout=1)
            if hasattr(self, 'read_thread') and self.read_thread.is_alive():
                self.read_thread.join(timeout=1)

            # Close serial connection
            if self.ser and self.ser.is_open:
                self.ser.close()

            self.logger.info("Connection closed.")

    def get_version(self) -> str:
        """
        Get ESP32 firmware version.

        Returns:
            Firmware version string
        """
        return self._send_command("VERSION")  # type: ignore[return-value]

    def get_status(self) -> Dict[str, Any]:
        """
        Get ESP32 status including failsafe state.

        Returns:
            Dictionary containing status information
        """
        try:
            response = self._send_command("STATUS")
            if response and "," in response:
                parts = response.split(",")
                if len(parts) >= 3:
                    status_info = {
                        'state': parts[0],
                        'time_since_command': int(parts[1]),
                        'time_since_ping': int(parts[2]),
                        'failsafe_engaged': parts[0] == "FAILSAFE"
                    }
                    self.failsafe_engaged = bool(status_info['failsafe_engaged'])
                    return status_info
        except (ValueError, IOError):
            pass
        return {'state': 'UNKNOWN', 'failsafe_engaged': self.failsafe_engaged}

    def set_pin_mode(self, pin: int, mode: str) -> None:
        """
        Set GPIO pin mode.

        Args:
            pin: GPIO pin number (0-39)
            mode: Pin mode ('IN', 'OUT', or 'IN_PULLUP')

        Raises:
            ValueError: If pin or mode is invalid
        """
        if not 0 <= pin < 40:
            raise ValueError(f"Invalid pin number: {pin}. Must be 0-39.")

        mode = mode.upper()
        if mode not in ['IN', 'OUT', 'IN_PULLUP']:
            raise ValueError(f"Invalid mode: {mode}. Must be 'IN', 'OUT', or 'IN_PULLUP'.")

        self._send_command(f"MODE {pin} {mode}", expect_response=False)

        # Track configured pins
        if pin not in self.configured_pins:
            self.configured_pins.append(pin)

    def digital_write(self, pin: int, value: Union[int, bool]) -> None:
        """
        Write digital value to GPIO pin.

        Args:
            pin: GPIO pin number (0-39)
            value: Digital value (0, 1, False, True)
        """
        if not 0 <= pin < 40:
            raise ValueError(f"Invalid pin number: {pin}. Must be 0-39.")

        int_value = 1 if value else 0
        self._send_command(f"WRITE {pin} {int_value}", expect_response=False)

    def digital_read(self, pin: int) -> int:
        """
        Read digital value from GPIO pin.

        Args:
            pin: GPIO pin number (0-39)

        Returns:
            Digital value (0 or 1)
        """
        if not 0 <= pin < 40:
            raise ValueError(f"Invalid pin number: {pin}. Must be 0-39.")

        response = self._send_command(f"READ {pin}")
        return int(response)  # type: ignore[arg-type]

    def analog_read(self, pin: int) -> int:
        """
        Read analog value from ADC pin.

        Args:
            pin: ADC pin number (32-39)

        Returns:
            12-bit analog value (0-4095)

        Raises:
            ValueError: If pin is not a valid ADC pin
        """
        if pin not in [32, 33, 34, 35, 36, 37, 38, 39]:
            raise ValueError(f"Invalid ADC pin: {pin}. Must be one of: 32, 33, 34, 35, 36, 37, 38, 39.")

        response = self._send_command(f"AREAD {pin}")
        return int(response)  # type: ignore[arg-type]

    def analog_write(self, pin: int, value: int) -> None:
        """
        Write analog value to DAC pin.

        Args:
            pin: DAC pin number (25 or 26)
            value: 8-bit analog value (0-255)

        Raises:
            ValueError: If pin or value is invalid
        """
        if pin not in [25, 26]:
            raise ValueError(f"Invalid DAC pin: {pin}. Must be 25 or 26.")
        if not 0 <= value <= 255:
            raise ValueError(f"Invalid DAC value: {value}. Must be between 0 and 255.")

        self._send_command(f"AWRITE {pin} {value}", expect_response=False)

    def get_configured_pins(self) -> List[int]:
        """
        Get list of currently configured GPIO pins.

        Returns:
            List of pin numbers that have been configured
        """
        return self.configured_pins.copy()

    def reset_failsafe(self) -> bool:
        """
        Manually reset/disable failsafe mode.

        Returns:
            True if failsafe was engaged and reset, False if not engaged
        """
        try:
            response = self._send_command("RESET_FAILSAFE")
            return response is not None and "OK" in response
        except (ValueError, IOError):
            return False

    def clear_failsafe(self) -> bool:
        """
        Alias for reset_failsafe() for convenience.

        Returns:
            True if failsafe was engaged and reset, False if not engaged
        """
        return self.reset_failsafe()

    def disable_failsafe(self) -> bool:
        """
        Alias for reset_failsafe() for convenience.

        Returns:
            True if failsafe was engaged and reset, False if not engaged
        """
        return self.reset_failsafe()

    # PWM Functions
    def pwm_init(self, pin: int, frequency: int = 5000, resolution: int = 8) -> int:
        """
        Initialize PWM on a pin.

        Args:
            pin: GPIO pin number
            frequency: PWM frequency in Hz (1-40000, default: 5000)
            resolution: PWM resolution in bits (1-16, default: 8)

        Returns:
            PWM channel number assigned to the pin

        Raises:
            ValueError: If parameters are invalid
        """
        if not 0 <= pin < 40:
            raise ValueError(f"Invalid pin number: {pin}. Must be 0-39.")
        if not 1 <= frequency <= 40000:
            raise ValueError(f"Invalid frequency: {frequency}. Must be 1-40000 Hz.")
        if not 1 <= resolution <= 16:
            raise ValueError(f"Invalid resolution: {resolution}. Must be 1-16 bits.")

        response = self._send_command(f"PWM_INIT {pin} {frequency} {resolution}")
        channel = int(response)  # type: ignore[arg-type]
        self.pwm_channels[pin] = channel
        return channel

    def pwm_write(self, pin: int, duty_cycle: int) -> None:
        """
        Set PWM duty cycle on a pin.

        Args:
            pin: GPIO pin number
            duty_cycle: Duty cycle value (0 to 2^resolution - 1)

        Raises:
            ValueError: If pin doesn't have PWM initialized
        """
        if not 0 <= pin < 40:
            raise ValueError(f"Invalid pin number: {pin}. Must be 0-39.")
        if pin not in self.pwm_channels:
            raise ValueError(f"PWM not initialized on pin {pin}. Call pwm_init() first.")

        self._send_command(f"PWM_WRITE {pin} {duty_cycle}", expect_response=False)

    def pwm_stop(self, pin: int) -> None:
        """
        Stop PWM on a pin and release the channel.

        Args:
            pin: GPIO pin number
        """
        if not 0 <= pin < 40:
            raise ValueError(f"Invalid pin number: {pin}. Must be 0-39.")

        self._send_command(f"PWM_STOP {pin}", expect_response=False)
        if pin in self.pwm_channels:
            del self.pwm_channels[pin]

    def pwm_set_duty_percent(self, pin: int, percent: float) -> None:
        """
        Set PWM duty cycle as a percentage.

        Args:
            pin: GPIO pin number
            percent: Duty cycle as percentage (0.0 to 100.0)

        Raises:
            ValueError: If pin doesn't have PWM initialized or percent is invalid
        """
        if not 0 <= pin < 40:
            raise ValueError(f"Invalid pin number: {pin}. Must be 0-39.")
        if pin not in self.pwm_channels:
            raise ValueError(f"PWM not initialized on pin {pin}. Call pwm_init() first.")
        if not 0.0 <= percent <= 100.0:
            raise ValueError(f"Invalid percent: {percent}. Must be 0.0-100.0.")

        # Assume 8-bit resolution by default (0-255)
        duty_cycle = int((percent / 100.0) * 255)
        self.pwm_write(pin, duty_cycle)

    # EEPROM Functions
    def eeprom_read(self, address: int) -> int:
        """
        Read a single byte from EEPROM.

        Args:
            address: EEPROM address (0-511)

        Returns:
            Byte value at the address (0-255)

        Raises:
            ValueError: If address is out of range
        """
        if not 0 <= address < 512:
            raise ValueError(f"Invalid EEPROM address: {address}. Must be 0-511.")

        response = self._send_command(f"EEPROM_READ {address}")
        return int(response)  # type: ignore[arg-type]

    def eeprom_write(self, address: int, value: int) -> None:
        """
        Write a single byte to EEPROM (requires commit to persist).

        Args:
            address: EEPROM address (0-511)
            value: Byte value to write (0-255)

        Raises:
            ValueError: If address or value is invalid
        """
        if not 0 <= address < 512:
            raise ValueError(f"Invalid EEPROM address: {address}. Must be 0-511.")
        if not 0 <= value <= 255:
            raise ValueError(f"Invalid value: {value}. Must be 0-255.")

        self._send_command(f"EEPROM_WRITE {address} {value}", expect_response=False)

    def eeprom_read_block(self, address: int, length: int) -> List[int]:
        """
        Read a block of bytes from EEPROM.

        Args:
            address: Starting EEPROM address
            length: Number of bytes to read

        Returns:
            List of byte values

        Raises:
            ValueError: If address or length is invalid
        """
        if not 0 <= address < 512:
            raise ValueError(f"Invalid EEPROM address: {address}. Must be 0-511.")
        if not 1 <= length <= (512 - address):
            raise ValueError(f"Invalid length: {length}.")

        response = self._send_command(f"EEPROM_READ_BLOCK {address} {length}")
        return [int(x) for x in response.split()]  # type: ignore[union-attr]

    def eeprom_write_block(self, address: int, data: List[int]) -> None:
        """
        Write a block of bytes to EEPROM (requires commit to persist).

        Args:
            address: Starting EEPROM address
            data: List of byte values to write

        Raises:
            ValueError: If address or data is invalid
        """
        if not 0 <= address < 512:
            raise ValueError(f"Invalid EEPROM address: {address}. Must be 0-511.")
        if not data or (address + len(data)) > 512:
            raise ValueError(f"Invalid data length or exceeds EEPROM size.")

        for value in data:
            if not 0 <= value <= 255:
                raise ValueError(f"Invalid value in data: {value}. Must be 0-255.")

        data_str = " ".join(str(x) for x in data)
        self._send_command(f"EEPROM_WRITE_BLOCK {address} {data_str}", expect_response=False)

    def eeprom_commit(self) -> None:
        """
        Commit EEPROM changes to flash memory.
        This must be called after eeprom_write() to persist changes.
        """
        self._send_command("EEPROM_COMMIT", expect_response=False)

    def eeprom_clear(self) -> None:
        """
        Clear all EEPROM data (set all bytes to 0) and commit.
        """
        self._send_command("EEPROM_CLEAR", expect_response=False)

    def eeprom_write_string(self, address: int, text: str) -> None:
        """
        Write a string to EEPROM.

        Args:
            address: Starting EEPROM address
            text: String to write

        Raises:
            ValueError: If string is too long or address is invalid
        """
        data = [ord(c) for c in text]
        self.eeprom_write_block(address, data)

    def eeprom_read_string(self, address: int, length: int) -> str:
        """
        Read a string from EEPROM.

        Args:
            address: Starting EEPROM address
            length: Number of bytes to read

        Returns:
            String read from EEPROM
        """
        data = self.eeprom_read_block(address, length)
        # Stop at first null terminator
        result = []
        for byte in data:
            if byte == 0:
                break
            result.append(chr(byte))
        return ''.join(result)

    # Batch Operations
    def batch_digital_write(self, pin_values: Dict[int, Union[int, bool]]) -> None:
        """
        Write multiple digital pins in a single command for efficiency.

        Args:
            pin_values: Dictionary mapping pin numbers to values (0/1 or False/True)

        Example:
            >>> esp.batch_digital_write({2: 1, 4: 0, 5: 1})
        """
        if not pin_values:
            raise ValueError("pin_values dictionary cannot be empty")

        # Build command string: "BATCH_WRITE pin1 val1 pin2 val2 ..."
        parts = []
        for pin, value in pin_values.items():
            if not 0 <= pin < 40:
                raise ValueError(f"Invalid pin number: {pin}. Must be 0-39.")
            int_value = 1 if value else 0
            parts.append(f"{pin} {int_value}")
        
        command = f"BATCH_WRITE {' '.join(parts)}"
        self._send_command(command, expect_response=False)

    # I2C Functions
    def i2c_init(self, sda: int = 21, scl: int = 22, frequency: int = 100000) -> None:
        """
        Initialize I2C bus.

        Args:
            sda: SDA pin number (default: 21)
            scl: SCL pin number (default: 22)
            frequency: I2C frequency in Hz (default: 100000)

        Raises:
            ValueError: If parameters are invalid
        """
        if not 0 <= sda < 40:
            raise ValueError(f"Invalid SDA pin: {sda}. Must be 0-39.")
        if not 0 <= scl < 40:
            raise ValueError(f"Invalid SCL pin: {scl}. Must be 0-39.")
        if not 1000 <= frequency <= 1000000:
            raise ValueError(f"Invalid frequency: {frequency}. Must be 1000-1000000 Hz.")

        self._send_command(f"I2C_INIT {sda} {scl} {frequency}", expect_response=False)
        self.i2c_initialized = True

    def i2c_scan(self) -> List[int]:
        """
        Scan I2C bus for devices.

        Returns:
            List of I2C addresses where devices were found

        Raises:
            RuntimeError: If I2C not initialized
        """
        if not self.i2c_initialized:
            raise RuntimeError("I2C not initialized. Call i2c_init() first.")

        response = self._send_command("I2C_SCAN")
        if response == "NONE":  # type: ignore[comparison-overlap]
            return []
        
        # Parse response: "0x3C,0x57" -> [60, 87]
        addresses = []
        if response:  # type: ignore[truthy-bool]
            for addr_str in response.split(','):  # type: ignore[union-attr]
                addr_str = addr_str.strip()
                if addr_str.startswith('0x'):
                    addresses.append(int(addr_str, 16))
                else:
                    addresses.append(int(addr_str))
        return addresses

    def i2c_write(self, address: int, data: Union[int, List[int], bytes]) -> None:
        """
        Write data to I2C device.

        Args:
            address: I2C device address (7-bit)
            data: Single byte, list of bytes, or bytes object to write

        Raises:
            ValueError: If address or data is invalid
            RuntimeError: If I2C not initialized
        """
        if not self.i2c_initialized:
            raise RuntimeError("I2C not initialized. Call i2c_init() first.")
        if not 0x00 <= address <= 0x7F:
            raise ValueError(f"Invalid I2C address: 0x{address:02X}. Must be 0x00-0x7F.")

        # Convert data to list of bytes
        if isinstance(data, int):
            if not 0 <= data <= 255:
                raise ValueError(f"Invalid byte value: {data}. Must be 0-255.")
            bytes_list = [data]
        elif isinstance(data, bytes):
            bytes_list = list(data)
        else:
            bytes_list = data

        if not bytes_list:
            raise ValueError("Data cannot be empty")

        # Build command: I2C_WRITE <address> <byte1> <byte2> ...
        cmd = f"I2C_WRITE {address} " + " ".join(str(b) for b in bytes_list)
        self._send_command(cmd, expect_response=False)

    def i2c_read(self, address: int, num_bytes: int) -> List[int]:
        """
        Read data from I2C device.

        Args:
            address: I2C device address (7-bit)
            num_bytes: Number of bytes to read

        Returns:
            List of bytes read from device

        Raises:
            ValueError: If parameters are invalid
            RuntimeError: If I2C not initialized
        """
        if not self.i2c_initialized:
            raise RuntimeError("I2C not initialized. Call i2c_init() first.")
        if not 0x00 <= address <= 0x7F:
            raise ValueError(f"Invalid I2C address: 0x{address:02X}. Must be 0x00-0x7F.")
        if not 1 <= num_bytes <= 128:
            raise ValueError(f"Invalid num_bytes: {num_bytes}. Must be 1-128.")

        response = self._send_command(f"I2C_READ {address} {num_bytes}")
        
        # Parse response: "0x12,0x34,0x56" -> [18, 52, 86]
        if not response or response == "ERROR":  # type: ignore[comparison-overlap]
            return []
        
        bytes_list = []
        for byte_str in response.split(','):  # type: ignore[union-attr]
            byte_str = byte_str.strip()
            if byte_str.startswith('0x'):
                bytes_list.append(int(byte_str, 16))
            else:
                bytes_list.append(int(byte_str))
        
        return bytes_list


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


def find_esp32_port(timeout: float = 2.0) -> Optional[str]:
    """
    Auto-detect ESP32 GPIO Bridge by actively probing serial ports.
    
    This function tests each available serial port by sending an IDENTIFY
    command and checking for the ESP32 GPIO Bridge signature in the response.

    Args:
        timeout: Timeout in seconds for each port test (default: 2.0)

    Returns:
        ESP32 port device path or None if not found
    """
    import logging
    logger = logging.getLogger("ESP32GPIO.AutoDetect")
    
    ports = list_serial_ports()
    
    if not ports:
        logger.debug("No serial ports found")
        return None
    
    # Filter to likely ESP32 ports for faster detection
    likely_ports = []
    for device, description in ports:
        device_lower = device.lower()
        # Common ESP32 serial port patterns
        if any(pattern in device_lower for pattern in ['ttyusb', 'ttyacm', 'com', 'cu.usb', 'cu.wch']):
            likely_ports.append(device)
        # Also check USB description as fallback
        elif any(id in description.lower() for id in ['esp32', 'cp210', 'ch340', 'ftdi', 'silicon labs']):
            likely_ports.append(device)
    
    # If no likely ports found, test all ports
    if not likely_ports:
        likely_ports = [device for device, _ in ports]
    
    logger.debug(f"Testing {len(likely_ports)} port(s) for ESP32 GPIO Bridge")
    
    # Test each port by sending IDENTIFY command
    for port in likely_ports:
        try:
            logger.debug(f"Probing {port}...")
            
            # Open port with DTR/RTS disabled to prevent auto-reset
            ser = serial.Serial(
                port, 
                115200, 
                timeout=timeout,
                dsrdtr=False,  # Disable DTR to prevent reset
                rtscts=False   # Disable RTS
            )
            
            # Wait for any auto-reset to complete and ESP32 to boot
            time.sleep(0.3)
            
            # Drain boot messages
            start_drain = time.time()
            while (time.time() - start_drain) < 0.5:
                if ser.in_waiting:
                    try:
                        ser.read(ser.in_waiting)
                    except:
                        pass
                time.sleep(0.05)
            
            # Clear buffer one more time
            ser.reset_input_buffer()
            
            # Try multiple times with PING first (fastest, safest)
            for attempt in range(3):
                ser.write(b"<PING>")
                time.sleep(0.1)
                
                start_time = time.time()
                while (time.time() - start_time) < 0.5:
                    if ser.in_waiting:
                        try:
                            line = ser.readline().decode('utf-8', errors='ignore').strip()
                            if "PONG" in line:
                                ser.close()
                                logger.info(f"Found ESP32 GPIO Bridge on {port}")
                                return port
                        except:
                            pass
                    time.sleep(0.01)
                
                time.sleep(0.1)
            
            # Try IDENTIFY command
            ser.write(b"<IDENTIFY>")
            time.sleep(0.1)
            
            start_time = time.time()
            while (time.time() - start_time) < 0.5:
                if ser.in_waiting:
                    try:
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                        if "ESP32_GPIO_BRIDGE" in line:
                            ser.close()
                            logger.info(f"Found ESP32 GPIO Bridge on {port}")
                            return port
                    except:
                        pass
                time.sleep(0.01)
            
            ser.close()
            
        except (serial.SerialException, OSError, PermissionError) as e:
            logger.debug(f"Could not probe {port}: {e}")
            continue
        except Exception as e:
            logger.debug(f"Unexpected error probing {port}: {e}")
            continue
    
    logger.warning("ESP32 GPIO Bridge not found on any port")
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

            # Demo: Check ESP32 status
            status = esp.get_status()
            print(f"ESP32 Status: {status['state']}")
            print(f"Failsafe engaged: {status['failsafe_engaged']}")

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

            # Demo: Final status check
            final_status = esp.get_status()
            print(f"\nFinal ESP32 Status: {final_status['state']}")
            print(f"Total session time: {final_status['time_since_command']}ms since last command")

    except serial.SerialException as e:
        print(f"Error: Could not open port {esp32_port}. {e}")
    except KeyboardInterrupt:
        print("\nExiting due to user interrupt.")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")


