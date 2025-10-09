#ifndef PWM_H
#define PWM_H

#include <Arduino.h>
#include "config.h"

// ============================================================================
// ESP32 GPIO Bridge - PWM Management
// ============================================================================

// PWM channel management structure
struct PWMChannel {
  int pin;
  int channel;
  bool active;
};

/**
 * @brief Find PWM channel for a given pin
 * @param pin Pin number
 * @return Channel number or -1 if not found
 */
int findPWMChannel(int pin);

/**
 * @brief Allocate a PWM channel for a given pin
 * @param pin Pin number
 * @return Channel number or -1 if no channels available
 */
int allocatePWMChannel(int pin);

/**
 * @brief Handle PWM initialization command
 * @param pinStr Pin number as string
 * @param freqStr Frequency as string
 * @param resStr Resolution as string
 */
void handlePWMInit(String pinStr, String freqStr, String resStr);

/**
 * @brief Handle PWM write command
 * @param pinStr Pin number as string
 * @param dutyStr Duty cycle as string
 */
void handlePWMWrite(String pinStr, String dutyStr);

/**
 * @brief Handle PWM stop command
 * @param pinStr Pin number as string
 */
void handlePWMStop(String pinStr);

#endif // PWM_H
