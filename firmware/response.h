#ifndef RESPONSE_H
#define RESPONSE_H

#include "config.h"

// ============================================================================
// ESP32 GPIO Bridge - Response Buffer System
// ============================================================================

// External response buffer variables (defined in firmware.ino)
extern char responseBuffer[RESPONSE_BUFFER_SIZE];
extern int responseIndex;

/**
 * @brief Clear the response buffer
 */
void clearResponse();

/**
 * @brief Add a string to the response buffer
 * @param str String to add
 */
void addToResponse(const char* str);

/**
 * @brief Add an integer value to the response buffer
 * @param value Integer value to add
 */
void addToResponse(int value);

/**
 * @brief Add an unsigned long value to the response buffer
 * @param value Unsigned long value to add
 */
void addToResponse(unsigned long value);

/**
 * @brief Send the response buffer over serial and clear it
 */
void sendResponse();

#endif // RESPONSE_H
