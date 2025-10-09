#include "response.h"
#include <Arduino.h>

// ============================================================================
// ESP32 GPIO Bridge - Response Buffer System Implementation
// ============================================================================

void clearResponse() {
  responseIndex = 0;
  responseBuffer[0] = '\0';
}

void addToResponse(const char* str) {
  int len = strlen(str);
  if (responseIndex + len < RESPONSE_BUFFER_SIZE - 1) {
    strcpy(responseBuffer + responseIndex, str);
    responseIndex += len;
  }
}

void addToResponse(int value) {
  char temp[16];
  sprintf(temp, "%d", value);
  addToResponse(temp);
}

void addToResponse(unsigned long value) {
  char temp[16];
  sprintf(temp, "%lu", value);
  addToResponse(temp);
}

void sendResponse() {
  if (responseIndex > 0) {
    Serial.println(responseBuffer);
    clearResponse();
  }
}
