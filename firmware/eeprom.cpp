#include "eeprom.h"
#include "response.h"

// ============================================================================
// ESP32 GPIO Bridge - EEPROM Storage Operations Implementation
// ============================================================================

void handleEEPROMRead(String addrStr) {
    int addr = addrStr.toInt();
    
    if (addr < 0 || addr >= EEPROM_SIZE) {
        clearResponse();
        addToResponse("<ERROR:Address out of range>");
        sendResponse();
        return;
    }
    
    byte value = EEPROM.read(addr);
    clearResponse();
    addToResponse("<");
    addToResponse((int)value);
    addToResponse(">");
    sendResponse();
}

void handleEEPROMWrite(String addrStr, String valStr) {
    int addr = addrStr.toInt();
    int value = valStr.toInt();
    
    if (addr < 0 || addr >= EEPROM_SIZE) {
        Serial.println("<ERROR:Address out of range>");
        return;
    }
    
    if (value < 0 || value > 255) {
        Serial.println("<ERROR:Value must be between 0 and 255>");
        return;
    }
    
    EEPROM.write(addr, (byte)value);
    // OK response removed (v0.1.4 optimization)
}

void handleEEPROMReadBlock(String addrStr, String lenStr) {
    int addr = addrStr.toInt();
    int len = lenStr.toInt();
    
    if (addr < 0 || addr >= EEPROM_SIZE) {
        Serial.println("<ERROR:Start address out of range>");
        return;
    }
    
    if (len < 1 || (addr + len) > EEPROM_SIZE) {
        Serial.println("<ERROR:Length invalid or exceeds EEPROM size>");
        return;
    }
    
    String data = "";
    for (int i = 0; i < len; i++) {
        if (i > 0) data += " ";
        data += String(EEPROM.read(addr + i));
    }
    
    Serial.print("<");
    Serial.print(data);
    Serial.println(">");
}

void handleEEPROMWriteBlock(String parts[], int partCount) {
    if (partCount < 3) {
        Serial.println("<ERROR:Invalid parameters>");
        return;
    }
    
    int addr = parts[1].toInt();
    
    if (addr < 0 || addr >= EEPROM_SIZE) {
        Serial.println("<ERROR:Address out of range>");
        return;
    }
    
    int dataCount = partCount - 2;
    if ((addr + dataCount) > EEPROM_SIZE) {
        Serial.println("<ERROR:Data exceeds EEPROM size>");
        return;
    }
    
    for (int i = 0; i < dataCount; i++) {
        int value = parts[i + 2].toInt();
        if (value < 0 || value > 255) {
            Serial.println("<ERROR:Value must be between 0 and 255>");
            return;
        }
        EEPROM.write(addr + i, (byte)value);
    }
    
    // OK response removed (v0.1.4 optimization)
}

void handleEEPROMCommit() {
    if (EEPROM.commit()) {
        // OK response removed (v0.1.4 optimization)
    } else {
        Serial.println("<ERROR:EEPROM commit failed>");
    }
}

void handleEEPROMClear() {
    for (int i = 0; i < EEPROM_SIZE; i++) {
        EEPROM.write(i, 0);
    }
    if (EEPROM.commit()) {
        // OK response removed (v0.1.4 optimization)
    } else {
        Serial.println("<ERROR:EEPROM clear failed>");
    }
}
