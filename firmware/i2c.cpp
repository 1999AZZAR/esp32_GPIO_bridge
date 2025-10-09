#include "i2c.h"
#include "response.h"

// ============================================================================
// ESP32 GPIO Bridge - I2C Communication Implementation
// ============================================================================

void handleI2CInit(String sdaStr, String sclStr) {
    Wire.begin(sdaStr.toInt(), sclStr.toInt());
    // OK response removed (v0.1.4 optimization)
}

void handleI2CScan() {
    clearResponse();
    addToResponse("<");
    
    bool first = true;
    for (byte address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            if (!first) addToResponse(" ");
            addToResponse("0x");
            char hexStr[4];
            sprintf(hexStr, "%02X", address);
            addToResponse(hexStr);
            first = false;
        }
    }
    addToResponse(">");
    sendResponse();
}

void handleI2CWrite(String addrStr, String parts[], int partCount) {
    byte addr = strtol(addrStr.c_str(), NULL, 16);
    Wire.beginTransmission(addr);
    for (int i = 2; i < partCount; i++) {
        Wire.write((byte)strtol(parts[i].c_str(), NULL, 16));
    }
    if (Wire.endTransmission() == 0) {
        // OK response removed (v0.1.4 optimization)
    } else {
        Serial.println("<ERROR:I2C write failed>");
    }
}

void handleI2CRead(String addrStr, String lenStr) {
    byte addr = strtol(addrStr.c_str(), NULL, 16);
    int len = lenStr.toInt();
    Wire.requestFrom(addr, (byte)len);
    
    clearResponse();
    addToResponse("<");
    
    bool first = true;
    while (Wire.available()) {
        if (!first) addToResponse(" ");
        char hexStr[4];
        sprintf(hexStr, "%02X", Wire.read());
        addToResponse(hexStr);
        first = false;
    }
    addToResponse(">");
    sendResponse();
}
