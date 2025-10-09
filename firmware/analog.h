#ifndef ANALOG_H
#define ANALOG_H

#include <Arduino.h>
#include "config.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

// ============================================================================
// ESP32 GPIO Bridge - Analog I/O Operations
// ============================================================================

/**
 * @brief Initialize ADC with default settings
 */
void initADC();

/**
 * @brief Get ADC1 channel for a given pin
 * @param pin Pin number
 * @return ADC1 channel or ADC1_CHANNEL_MAX if invalid
 */
adc1_channel_t getADC1Channel(int pin);

/**
 * @brief Handle analog read command
 * @param pinStr Pin number as string
 */
void handleAnalogRead(String pinStr);

/**
 * @brief Handle analog write (DAC) command
 * @param pinStr Pin number as string
 * @param valStr Value as string (0-255)
 */
void handleAnalogWrite(String pinStr, String valStr);

#endif // ANALOG_H
