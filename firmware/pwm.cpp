#include "pwm.h"
#include "gpio.h"
#include "response.h"

// External variables from firmware.ino
extern PWMChannel pwmChannels[MAX_PWM_CHANNELS];
extern int8_t pinToPWMChannel[MAX_PINS];
extern bool outputCommandsSent;

// ============================================================================
// ESP32 GPIO Bridge - PWM Management Implementation
// ============================================================================

int findPWMChannel(int pin) {
    if (pin < 0 || pin >= MAX_PINS) return -1;
    return pinToPWMChannel[pin];
}

int allocatePWMChannel(int pin) {
    // Check if pin already has a channel
    int existingChannel = findPWMChannel(pin);
    if (existingChannel != -1) {
        return existingChannel;
    }
    
    // Find free channel
    for (int i = 0; i < MAX_PWM_CHANNELS; i++) {
        if (!pwmChannels[i].active) {
            pwmChannels[i].pin = pin;
            pwmChannels[i].active = true;
            pinToPWMChannel[pin] = i;  // Update O(1) mapping (v0.1.6-beta)
            return i;
        }
    }
    return -1;
}

void handlePWMInit(String pinStr, String freqStr, String resStr) {
    int pin = pinStr.toInt();
    int frequency = freqStr.toInt();
    int resolution = resStr.toInt();
    
    if (!isValidPin(pin)) {
        Serial.println("<ERROR:Invalid pin>");
        return;
    }
    
    if (frequency < 1 || frequency > 40000) {
        Serial.println("<ERROR:Frequency must be between 1 and 40000 Hz>");
        return;
    }
    
    if (resolution < 1 || resolution > 16) {
        Serial.println("<ERROR:Resolution must be between 1 and 16 bits>");
        return;
    }
    
    int channel = allocatePWMChannel(pin);
    if (channel == -1) {
        Serial.println("<ERROR:No PWM channels available>");
        return;
    }
    
    // Use new ESP32 Arduino core 3.x API
    #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
        if (!ledcAttach(pin, frequency, resolution)) {
            Serial.println("<ERROR:PWM attach failed>");
            pwmChannels[channel].active = false;
            pwmChannels[channel].pin = -1;
            return;
        }
    #else
        // Use old API for compatibility with ESP32 Arduino core 2.x
        ledcSetup(channel, frequency, resolution);
        ledcAttachPin(pin, channel);
    #endif
    
    trackPin(pin);
    outputCommandsSent = true;  // PWM output - enable failsafe
    
    clearResponse();
    addToResponse("<");
    addToResponse(channel);
    addToResponse(">");
    sendResponse();
}

void handlePWMWrite(String pinStr, String dutyStr) {
    int pin = pinStr.toInt();
    int duty = dutyStr.toInt();
    
    if (!isValidPin(pin)) {
        Serial.println("<ERROR:Invalid pin>");
        return;
    }
    
    int channel = findPWMChannel(pin);
    if (channel == -1) {
        Serial.println("<ERROR:PWM not initialized for this pin>");
        return;
    }
    
    // Both old and new API use ledcWrite, but with different parameters
    #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
        ledcWrite(pin, duty);  // New API uses pin
    #else
        ledcWrite(channel, duty);  // Old API uses channel
    #endif
    
    outputCommandsSent = true;  // PWM write - enable failsafe
    // OK response removed (v0.1.4 optimization)
}

void handlePWMStop(String pinStr) {
    int pin = pinStr.toInt();
    
    if (!isValidPin(pin)) {
        Serial.println("<ERROR:Invalid pin>");
        return;
    }
    
    int channel = findPWMChannel(pin);
    if (channel == -1) {
        Serial.println("<ERROR:PWM not initialized for this pin>");
        return;
    }
    
    // Use new or old API depending on version
    #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
        ledcDetach(pin);  // New API
    #else
        ledcDetachPin(pin);  // Old API
    #endif
    
    pwmChannels[channel].active = false;
    pwmChannels[channel].pin = -1;
    pinToPWMChannel[pin] = -1;  // Clear O(1) mapping (v0.1.6-beta)
    // OK response removed (v0.1.4 optimization)
}
