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
    // Optimized I2C scan with reduced timeout for faster scanning
    Wire.setClock(100000);  // Set to 100kHz for faster scanning
    
    for (byte address = 1; address < 127; address++) {
        // Skip reserved addresses for faster scanning
        if (address >= 120 && address <= 127) continue;  // Reserved range
        
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();
        
        if (error == 0) {
            if (!first) addToResponse(" ");
            addToResponse("0x");
            char hexStr[4];
            sprintf(hexStr, "%02X", address);
            addToResponse(hexStr);
            first = false;
        }
        
        // Yield control periodically to prevent blocking
        if (address % 16 == 0) {
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
    }
    
    Wire.setClock(400000);  // Restore to 400kHz
    addToResponse(">");
    sendResponse();
}

// Forward declaration for fast parsing functions
extern int fastAtoiHex(const char* str);
extern int fastAtoi(const char* str);

void handleI2CWrite(String addrStr, String parts[], int partCount) {
    byte addr = fastAtoiHex(addrStr.c_str());
    Wire.beginTransmission(addr);
    for (int i = 2; i < partCount; i++) {
        Wire.write((byte)fastAtoiHex(parts[i].c_str()));
    }
    if (Wire.endTransmission() == 0) {
        // OK response removed (v0.1.4 optimization)
    } else {
        Serial.println("<ERROR:I2C write failed>");
    }
}

void handleI2CRead(String addrStr, String lenStr) {
    byte addr = fastAtoiHex(addrStr.c_str());
    int len = fastAtoi(lenStr.c_str());
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
