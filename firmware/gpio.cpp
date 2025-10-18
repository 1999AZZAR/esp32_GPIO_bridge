#include "gpio.h"
#include "response.h"

// External variables from firmware.ino
extern bool configuredPins[MAX_PINS];
extern bool outputCommandsSent;
extern void trackPinState(int pin, uint8_t mode, uint8_t value);

// ============================================================================
// ESP32 GPIO Bridge - Digital I/O Operations Implementation
// ============================================================================

bool isValidPin(int pin) { 
    return (pin >= 0 && pin < MAX_PINS); 
}

void trackPin(int pin) {
    if(isValidPin(pin)) configuredPins[pin] = true;
}

void handlePinMode(const char* pinStr, const char* modeStr) {
    int pin = atoi(pinStr);
    if (!isValidPin(pin)) { Serial.println("<ERROR:Invalid pin>"); return; }
    trackPin(pin);
    
    // Convert mode to uppercase for comparison
    char mode[15];
    strncpy(mode, modeStr, 14);
    mode[14] = '\0';
    for (char* p = mode; *p; p++) *p = toupper(*p);
    
    if (strcmp(mode, "OUT") == 0) { 
        pinMode(pin, OUTPUT);
        trackPinState(pin, OUTPUT, LOW);  // Track pin state for safe mode
        outputCommandsSent = true;  // Output mode - enable failsafe
    }
    else if (strcmp(mode, "IN") == 0) { 
        pinMode(pin, INPUT);
        trackPinState(pin, INPUT, LOW);  // Track pin state for safe mode
    }
    else if (strcmp(mode, "IN_PULLUP") == 0) { 
        pinMode(pin, INPUT_PULLUP);
        trackPinState(pin, INPUT_PULLUP, HIGH);  // Track pin state for safe mode
        outputCommandsSent = true;  // Pullup can source current - enable failsafe
    }
    else if (strcmp(mode, "IN_PULLDOWN") == 0) { 
        pinMode(pin, INPUT_PULLDOWN);
        trackPinState(pin, INPUT_PULLDOWN, LOW);  // Track pin state for safe mode
    }
    else { Serial.println("<ERROR:Invalid mode>"); return; }
    // OK response removed (v0.1.4 optimization - no response for write commands)
}

void handleDigitalWrite(String pinStr, String valStr) {
    int pin = pinStr.toInt();
    if (!isValidPin(pin)) { Serial.println("<ERROR:Invalid pin>"); return; }
    uint8_t value = valStr.toInt() == 1 ? HIGH : LOW;
    digitalWrite(pin, value);
    trackPinState(pin, OUTPUT, value);  // Track pin state for safe mode
    outputCommandsSent = true;  // Writing to pin - enable failsafe
    // OK response removed (v0.1.4 optimization)
}

void handleDigitalRead(String pinStr) {
    int pin = pinStr.toInt();
    if (!isValidPin(pin)) { 
        clearResponse();
        addToResponse("<ERROR:Invalid pin>");
        sendResponse();
        return; 
    }
    clearResponse();
    addToResponse("<");
    addToResponse(digitalRead(pin));
    addToResponse(">");
    sendResponse();
}

void handleBatchWrite(String parts[], int partCount) {
    if (partCount < 3 || (partCount - 1) % 2 != 0) {
        Serial.println("<ERROR:Invalid batch format. Use: BATCH_WRITE pin1 val1 pin2 val2 ...>");
        return;
    }
    
    int numPairs = (partCount - 1) / 2;
    for (int i = 0; i < numPairs; i++) {
        int pin = parts[1 + i * 2].toInt();
        int value = parts[2 + i * 2].toInt();
        
        if (!isValidPin(pin)) {
            Serial.print("<ERROR:Invalid pin ");
            Serial.print(pin);
            Serial.println(">");
            return;
        }
        
        uint8_t pinValue = value == 1 ? HIGH : LOW;
        digitalWrite(pin, pinValue);
        trackPinState(pin, OUTPUT, pinValue);  // Track pin state for safe mode
    }
    
    outputCommandsSent = true;  // Batch write - enable failsafe
    // OK response removed (v0.1.4 optimization)
}
