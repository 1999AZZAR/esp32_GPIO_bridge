#ifndef EEPROM_H
#define EEPROM_H

#include <Arduino.h>
#include <EEPROM.h>
#include "config.h"

// ============================================================================
// ESP32 GPIO Bridge - EEPROM Storage Operations
// ============================================================================

/**
 * @brief Handle EEPROM read command
 * @param addrStr Address as string
 */
void handleEEPROMRead(String addrStr);

/**
 * @brief Handle EEPROM write command
 * @param addrStr Address as string
 * @param valStr Value as string (0-255)
 */
void handleEEPROMWrite(String addrStr, String valStr);

/**
 * @brief Handle EEPROM read block command
 * @param addrStr Start address as string
 * @param lenStr Number of bytes to read as string
 */
void handleEEPROMReadBlock(String addrStr, String lenStr);

/**
 * @brief Handle EEPROM write block command
 * @param parts Command parts array with address and data
 * @param partCount Number of parts in array
 */
void handleEEPROMWriteBlock(String parts[], int partCount);

/**
 * @brief Handle EEPROM commit command
 */
void handleEEPROMCommit();

/**
 * @brief Handle EEPROM clear command
 */
void handleEEPROMClear();

#endif // EEPROM_H
