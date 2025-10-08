#include <Wire.h>
#include "driver/i2s.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define FW_VERSION "0.1.1-beta"
#define BAUD_RATE 115200
#define FAILSAFE_TIMEOUT 2000
#define MAX_PINS 40
#define DEFAULT_VREF 1100

unsigned long lastCommandTime = 0;
bool failsafeEngaged = false;
bool configuredPins[MAX_PINS] = {false};
bool adcInitialized = false;
esp_adc_cal_characteristics_t *adc_chars = NULL;

void setup() {
  Serial.begin(BAUD_RATE);
  
  // Wait for serial to stabilize and flush any boot garbage
  delay(1000);
  while(Serial.available()) {
    Serial.read();
  }
  
  // Send ready message
  Serial.println("<ESP32 GPIO Bridge Ready>");
  Serial.print("<Version:");
  Serial.print(FW_VERSION);
  Serial.println(">");
  
  lastCommandTime = millis();
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('>');
    if (command.startsWith("<")) {
        command.remove(0, 1);
        processCommand(command);
        lastCommandTime = millis();
        if (failsafeEngaged) {
          Serial.println("<INFO:Failsafe disengaged>");
          failsafeEngaged = false;
        }
    }
  }

  if (!failsafeEngaged && (millis() - lastCommandTime > FAILSAFE_TIMEOUT)) {
    engageFailsafe();
  }
}

void engageFailsafe() {
    Serial.println("<WARN:Failsafe engaged. Resetting pins.>");
    for (int i = 0; i < MAX_PINS; i++) {
        if (configuredPins[i]) {
            pinMode(i, INPUT);
            configuredPins[i] = false;
        }
    }
    failsafeEngaged = true;
}

void processCommand(String cmd) {
    String parts[10];
    int partCount = 0;
    int lastIndex = 0;
    for (int i = 0; i < cmd.length(); i++) {
        if (cmd.charAt(i) == ' ') {
            parts[partCount++] = cmd.substring(lastIndex, i);
            lastIndex = i + 1;
            if (partCount >= 9) break;
        }
    }
    parts[partCount++] = cmd.substring(lastIndex);

    String action = parts[0];
    action.toUpperCase();

    if (action == "PING") { Serial.println("<PONG>"); }
    else if (action == "VERSION") { Serial.print("<"); Serial.print(FW_VERSION); Serial.println(">"); }
    else if (action == "MODE") { handlePinMode(parts[1], parts[2]); }
    else if (action == "WRITE") { handleDigitalWrite(parts[1], parts[2]); }
    else if (action == "READ") { handleDigitalRead(parts[1]); }
    else if (action == "AREAD") { handleAnalogRead(parts[1]); }
    else if (action == "AWRITE") { handleAnalogWrite(parts[1], parts[2]); }
    else if (action == "I2C_INIT") { handleI2CInit(parts[1], parts[2]); }
    else if (action == "I2C_SCAN") { handleI2CScan(); }
    else if (action == "I2C_WRITE") { handleI2CWrite(parts[1], parts, partCount); }
    else if (action == "I2C_READ") { handleI2CRead(parts[1], parts[2]); }
    else if (action == "I2S_INIT_TX") { handleI2SInitTx(parts[1], parts[2], parts[3], parts[4]); }
    else if (action == "I2S_WRITE") { handleI2SWrite(parts, partCount); }
    else { Serial.println("<ERROR:Unknown command>"); }
}

bool isValidPin(int pin) { return (pin >= 0 && pin < MAX_PINS); }

void trackPin(int pin) {
    if(isValidPin(pin)) configuredPins[pin] = true;
}

void handlePinMode(String pinStr, String modeStr) {
    int pin = pinStr.toInt();
    if (!isValidPin(pin)) { Serial.println("<ERROR:Invalid pin>"); return; }
    trackPin(pin);
    modeStr.toUpperCase();
    if (modeStr == "OUT") { pinMode(pin, OUTPUT); }
    else if (modeStr == "IN") { pinMode(pin, INPUT); }
    else if (modeStr == "IN_PULLUP") { pinMode(pin, INPUT_PULLUP); }
    else { Serial.println("<ERROR:Invalid mode>"); return; }
    Serial.println("<OK>");
}

void handleDigitalWrite(String pinStr, String valStr) {
    int pin = pinStr.toInt();
    if (!isValidPin(pin)) { Serial.println("<ERROR:Invalid pin>"); return; }
    digitalWrite(pin, valStr.toInt() == 1 ? HIGH : LOW);
    Serial.println("<OK>");
}

void handleDigitalRead(String pinStr) {
    int pin = pinStr.toInt();
    if (!isValidPin(pin)) { Serial.println("<ERROR:Invalid pin>"); return; }
    Serial.print("<"); Serial.print(digitalRead(pin)); Serial.println(">");
}

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
    if (!isValidPin(pin)) { Serial.println("<ERROR:Invalid pin>"); return; }
    
    adc1_channel_t channel = getADC1Channel(pin);
    if (channel == ADC1_CHANNEL_MAX) {
        Serial.println("<ERROR:Pin is not a valid ADC pin>");
        return;
    }
    
    initADC();
    adc1_config_channel_atten(channel, ADC_ATTEN_DB_11);
    
    int raw = adc1_get_raw(channel);
    Serial.print("<"); Serial.print(raw); Serial.println(">");
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
  Serial.println("<OK>");
}

void handleI2CInit(String sdaStr, String sclStr) {
    Wire.begin(sdaStr.toInt(), sclStr.toInt());
    Serial.println("<OK>");
}

void handleI2CScan() {
    String found = "";
    for (byte address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            found += "0x" + String(address, HEX) + " ";
        }
    }
    found.trim();
    Serial.print("<"); Serial.print(found); Serial.println(">");
}

void handleI2CWrite(String addrStr, String parts[], int partCount) {
    byte addr = strtol(addrStr.c_str(), NULL, 16);
    Wire.beginTransmission(addr);
    for (int i = 2; i < partCount; i++) {
        Wire.write((byte)strtol(parts[i].c_str(), NULL, 16));
    }
    if (Wire.endTransmission() == 0) {
        Serial.println("<OK>");
    } else {
        Serial.println("<ERROR:I2C write failed>");
    }
}

void handleI2CRead(String addrStr, String lenStr) {
    byte addr = strtol(addrStr.c_str(), NULL, 16);
    int len = lenStr.toInt();
    Wire.requestFrom(addr, (byte)len);
    String data = "";
    while (Wire.available()) {
        data += String(Wire.read(), HEX) + " ";
    }
    data.trim();
    Serial.print("<"); Serial.print(data); Serial.println(">");
}

void handleI2SInitTx(String bckStr, String wsStr, String dataStr, String rateStr) {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = (uint32_t)rateStr.toInt(),
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false
    };
    i2s_pin_config_t pin_config = {
        .bck_io_num = (gpio_num_t)bckStr.toInt(),
        .ws_io_num = (gpio_num_t)wsStr.toInt(),
        .data_out_num = (gpio_num_t)dataStr.toInt(),
        .data_in_num = I2S_PIN_NO_CHANGE
    };
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
    Serial.println("<OK>");
}

void handleI2SWrite(String parts[], int partCount) {
    size_t bytes_written = 0;
    int16_t* buffer = new int16_t[partCount - 1];
    for(int i=1; i < partCount; i++) {
        buffer[i-1] = (int16_t)parts[i].toInt();
    }
    i2s_write(I2S_NUM_0, buffer, (partCount-1) * sizeof(int16_t), &bytes_written, portMAX_DELAY);
    delete[] buffer;
    Serial.println("<OK>");
}