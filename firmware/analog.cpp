#include "analog.h"
#include "gpio.h"
#include "response.h"

// External variables from firmware.ino
extern bool adcInitialized;
extern esp_adc_cal_characteristics_t *adc_chars;
extern bool outputCommandsSent;

// ============================================================================
// ESP32 GPIO Bridge - Analog I/O Operations Implementation
// ============================================================================

void initADC() {
    if (!adcInitialized) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
        adcInitialized = true;
    }
}

adc1_channel_t getADC1Channel(int pin) {
    switch(pin) {
        case 36: return ADC1_CHANNEL_0;
        case 37: return ADC1_CHANNEL_1;
        case 38: return ADC1_CHANNEL_2;
        case 39: return ADC1_CHANNEL_3;
        case 32: return ADC1_CHANNEL_4;
        case 33: return ADC1_CHANNEL_5;
        case 34: return ADC1_CHANNEL_6;
        case 35: return ADC1_CHANNEL_7;
        default: return ADC1_CHANNEL_MAX;
    }
}

void handleAnalogRead(String pinStr) {
    int pin = pinStr.toInt();
    if (!isValidPin(pin)) { 
        clearResponse();
        addToResponse("<ERROR:Invalid pin>");
        sendResponse();
        return; 
    }
    
    adc1_channel_t channel = getADC1Channel(pin);
    if (channel == ADC1_CHANNEL_MAX) {
        clearResponse();
        addToResponse("<ERROR:Pin is not a valid ADC pin>");
        sendResponse();
        return;
    }
    
    initADC();
    adc1_config_channel_atten(channel, ADC_ATTEN_DB_11);
    
    int raw = adc1_get_raw(channel);
    clearResponse();
    addToResponse("<");
    addToResponse(raw);
    addToResponse(">");
    sendResponse();
}

void handleAnalogWrite(String pinStr, String valStr) {
  int pin = pinStr.toInt();
  int value = valStr.toInt();
  if (pin != 25 && pin != 26) {
    Serial.println("<ERROR:Pin is not a valid DAC pin (use 25 or 26)>");
    return;
  }
  if (value < 0 || value > 255) {
    Serial.println("<ERROR:Value must be between 0 and 255>");
    return;
  }
  dacWrite(pin, value);
  outputCommandsSent = true;  // DAC output - enable failsafe
  // OK response removed (v0.1.4 optimization)
}
