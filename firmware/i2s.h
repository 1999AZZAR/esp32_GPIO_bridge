#ifndef I2S_H
#define I2S_H

#include <Arduino.h>
#include "driver/i2s.h"

// ============================================================================
// ESP32 GPIO Bridge - I2S Audio Operations
// ============================================================================

/**
 * @brief Handle I2S transmit initialization command
 * @param bckStr Bit clock pin as string
 * @param wsStr Word select pin as string
 * @param dataStr Data pin as string
 * @param rateStr Sample rate as string
 */
void handleI2SInitTx(String bckStr, String wsStr, String dataStr, String rateStr);

/**
 * @brief Handle I2S write command
 * @param parts Command parts array with audio data
 * @param partCount Number of parts in array
 */
void handleI2SWrite(String parts[], int partCount);

#endif // I2S_H
