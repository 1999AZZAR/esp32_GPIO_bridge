#ifndef I2C_H
#define I2C_H

#include <Arduino.h>
#include <Wire.h>

// ============================================================================
// ESP32 GPIO Bridge - I2C Communication
// ============================================================================

/**
 * @brief Handle I2C initialization command
 * @param sdaStr SDA pin number as string
 * @param sclStr SCL pin number as string
 */
void handleI2CInit(String sdaStr, String sclStr);

/**
 * @brief Handle I2C scan command - scan for devices
 */
void handleI2CScan();

/**
 * @brief Handle I2C write command
 * @param addrStr I2C address as hex string
 * @param parts Command parts array with data bytes
 * @param partCount Number of parts in array
 */
void handleI2CWrite(String addrStr, String parts[], int partCount);

/**
 * @brief Handle I2C read command
 * @param addrStr I2C address as hex string
 * @param lenStr Number of bytes to read as string
 */
void handleI2CRead(String addrStr, String lenStr);

#endif // I2C_H
