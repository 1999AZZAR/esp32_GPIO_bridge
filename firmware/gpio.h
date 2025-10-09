#ifndef GPIO_H
#define GPIO_H

#include <Arduino.h>
#include "config.h"

// ============================================================================
// ESP32 GPIO Bridge - Digital I/O Operations
// ============================================================================

/**
 * @brief Check if a pin number is valid
 * @param pin Pin number to validate
 * @return true if pin is valid, false otherwise
 */
bool isValidPin(int pin);

/**
 * @brief Track a pin as configured (for failsafe management)
 * @param pin Pin number to track
 */
void trackPin(int pin);

/**
 * @brief Handle pin mode configuration command
 * @param pinStr Pin number as string
 * @param modeStr Mode as string (OUT, IN, IN_PULLUP, IN_PULLDOWN)
 */
void handlePinMode(const char* pinStr, const char* modeStr);

/**
 * @brief Handle digital write command
 * @param pinStr Pin number as string
 * @param valStr Value as string (0 or 1)
 */
void handleDigitalWrite(String pinStr, String valStr);

/**
 * @brief Handle digital read command
 * @param pinStr Pin number as string
 */
void handleDigitalRead(String pinStr);

/**
 * @brief Handle batch write command for multiple pins
 * @param parts Command parts array
 * @param partCount Number of parts in array
 */
void handleBatchWrite(String parts[], int partCount);

#endif // GPIO_H
