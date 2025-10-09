#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// ESP32 GPIO Bridge - Configuration Constants
// ============================================================================

// Firmware Version
#define FW_VERSION "0.1.6-beta"

// Serial Communication
#define BAUD_RATE 115200
#define SERIAL_RX_BUFFER 1024        // Increased from default 256 for better throughput
#define SERIAL_TX_BUFFER 1024        // Increased from default 256 for better throughput

// Command Processing
#define CMD_BUFFER_SIZE 256          // Command buffer size for char-based parsing
#define CMD_QUEUE_SIZE 32            // Command queuing system buffer size

// Failsafe Configuration
#define FAILSAFE_TIMEOUT 10000       // 10 seconds of no commands before warning
#define FAILSAFE_GRACE_PERIOD 20000  // 20 seconds grace period before engaging failsafe
#define FAILSAFE_RECOVERY_TIMEOUT 5000 // 5 seconds to recover from failsafe

// Hardware Configuration
#define MAX_PINS 40                  // Maximum number of GPIO pins
#define DEFAULT_VREF 1100            // Default ADC reference voltage (mV)

// EEPROM Configuration
#define EEPROM_SIZE 512              // 512 bytes of EEPROM

// PWM Configuration
#define MAX_PWM_CHANNELS 16          // Maximum number of PWM channels
#define PWM_FREQUENCY 5000           // Default PWM frequency (Hz)
#define PWM_RESOLUTION 8             // Default PWM resolution (bits)

// Response Buffer Configuration
#define RESPONSE_BUFFER_SIZE 512     // Response buffer size for efficient serial output

#endif // CONFIG_H
