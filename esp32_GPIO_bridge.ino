#include <Wire.h>
#include <EEPROM.h>
#include "driver/i2s.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_wifi.h"
#include "esp_bt.h"
#include "esp_bt_main.h"

#define FW_VERSION "0.1.3-beta"
#define BAUD_RATE 115200
#define FAILSAFE_TIMEOUT 10000       // 10 seconds of no commands before warning
#define FAILSAFE_GRACE_PERIOD 20000  // 20 seconds grace period before engaging failsafe
#define FAILSAFE_RECOVERY_TIMEOUT 5000 // 5 seconds to recover from failsafe
#define MAX_PINS 40
#define DEFAULT_VREF 1100
#define EEPROM_SIZE 512              // 512 bytes of EEPROM
#define MAX_PWM_CHANNELS 16
#define PWM_FREQUENCY 5000
#define PWM_RESOLUTION 8

unsigned long lastCommandTime = 0;
unsigned long lastPingTime = 0;
bool failsafeEngaged = false;
bool failsafeWarningSent = false;
bool configuredPins[MAX_PINS] = {false};
bool adcInitialized = false;
esp_adc_cal_characteristics_t *adc_chars = NULL;
bool outputCommandsSent = false;  // Track if any output commands were sent

// PWM management
struct PWMChannel {
  int pin;
  int channel;
  bool active;
};
PWMChannel pwmChannels[MAX_PWM_CHANNELS];
int nextPWMChannel = 0;

void setup() {
  // Disable WiFi to save power and resources
  esp_wifi_stop();
  esp_wifi_deinit();
  
  // Disable Bluetooth to save power and resources
  if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED) {
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
  }
  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
  }
  
  Serial.begin(BAUD_RATE);
  
  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Initialize PWM channels
  for (int i = 0; i < MAX_PWM_CHANNELS; i++) {
    pwmChannels[i].pin = -1;
    pwmChannels[i].channel = i;
    pwmChannels[i].active = false;
  }
  
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
  Serial.println("<INFO:WiFi and Bluetooth disabled for maximum GPIO performance>");
  Serial.println("<INFO:EEPROM initialized with 512 bytes>");
  
  lastCommandTime = millis();
  lastPingTime = millis();
}

void loop() {
  unsigned long currentTime = millis();

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('>');
    if (command.startsWith("<")) {
        command.remove(0, 1);
        processCommand(command);
        lastCommandTime = currentTime;
        lastPingTime = currentTime; // Update ping time on any command

        // Reset failsafe states when communication resumes
        if (failsafeEngaged) {
          Serial.println("<INFO:Failsafe disengaged - Communication restored>");
          disengageFailsafe();
        }
        failsafeWarningSent = false;
    }
  }

  // Multi-stage failsafe detection - only if output commands were sent
  // Query-only commands (IDENTIFY, VERSION, READ, etc.) don't trigger failsafe
  if (outputCommandsSent) {
    unsigned long timeSinceLastCommand = currentTime - lastCommandTime;
    unsigned long timeSinceLastPing = currentTime - lastPingTime;

    if (!failsafeEngaged) {
      // Stage 1: Warning after no activity for FAILSAFE_TIMEOUT
      unsigned long maxIdleTime = max(timeSinceLastCommand, timeSinceLastPing);
      if (maxIdleTime > FAILSAFE_TIMEOUT && !failsafeWarningSent) {
        Serial.println("<WARN:No activity detected for " + String(FAILSAFE_TIMEOUT/1000) + " seconds>");
        Serial.println("<INFO:Send PING or any command within " + String(FAILSAFE_GRACE_PERIOD/1000) + " seconds to prevent failsafe>");
        failsafeWarningSent = true;
      }

      // Stage 2: Engage failsafe after grace period
      if (maxIdleTime > (FAILSAFE_TIMEOUT + FAILSAFE_GRACE_PERIOD)) {
        engageFailsafe();
      }
    } else {
      // In failsafe mode: any command disengages failsafe immediately
      if (timeSinceLastCommand < 1000) { // Recent command received
        Serial.println("<INFO:Communication detected - Disengaging failsafe>");
        disengageFailsafe();
      }
    }
  }
}

void engageFailsafe() {
    Serial.println("<WARN:Failsafe engaged - Communication lost>");
    Serial.println("<INFO:All configured pins reset to INPUT mode for safety>");

    // Reset all configured pins to INPUT mode
    for (int i = 0; i < MAX_PINS; i++) {
        if (configuredPins[i]) {
            pinMode(i, INPUT);
            configuredPins[i] = false;
            Serial.println("<INFO:Reset pin " + String(i) + " to INPUT>");
        }
    }

    failsafeEngaged = true;
    failsafeWarningSent = false; // Reset warning state

    Serial.println("<INFO:Failsafe active - Waiting for communication recovery>");
}

void disengageFailsafe() {
    if (failsafeEngaged) {
        Serial.println("<INFO:Failsafe disengaged - Normal operation resumed>");
        failsafeEngaged = false;
        failsafeWarningSent = false;
        // Note: Pins remain in their last configured state
        // User must reconfigure pins as needed
    }
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

    if (action == "PING") {
        Serial.println("<PONG>");
        lastPingTime = millis(); // Update ping time for failsafe detection
    }
    else if (action == "IDENTIFY") {
        Serial.print("<ESP32_GPIO_BRIDGE:");
        Serial.print(FW_VERSION);
        Serial.println(">");
    }
    else if (action == "VERSION") {
        Serial.print("<"); Serial.print(FW_VERSION); Serial.println(">");
    }
    else if (action == "STATUS") {
        Serial.print("<STATUS:");
        Serial.print(failsafeEngaged ? "FAILSAFE" : "NORMAL");
        Serial.print(",");
        Serial.print(millis() - lastCommandTime);
        Serial.print(",");
        Serial.print(millis() - lastPingTime);
        Serial.println(">");
    }
    else if (action == "RESET_FAILSAFE" || action == "CLEAR_FAILSAFE" || action == "DISABLE_FAILSAFE") {
        if (failsafeEngaged) {
            Serial.println("<INFO:Manually disengaging failsafe>");
            disengageFailsafe();
            Serial.println("<OK>");
        } else {
            Serial.println("<INFO:Failsafe not engaged>");
        }
    }
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
    else if (action == "PWM_INIT") { handlePWMInit(parts[1], parts[2], parts[3]); }
    else if (action == "PWM_WRITE") { handlePWMWrite(parts[1], parts[2]); }
    else if (action == "PWM_STOP") { handlePWMStop(parts[1]); }
    else if (action == "EEPROM_READ") { handleEEPROMRead(parts[1]); }
    else if (action == "EEPROM_WRITE") { handleEEPROMWrite(parts[1], parts[2]); }
    else if (action == "EEPROM_READ_BLOCK") { handleEEPROMReadBlock(parts[1], parts[2]); }
    else if (action == "EEPROM_WRITE_BLOCK") { handleEEPROMWriteBlock(parts, partCount); }
    else if (action == "EEPROM_COMMIT") { handleEEPROMCommit(); }
    else if (action == "EEPROM_CLEAR") { handleEEPROMClear(); }
    else if (action == "BATCH_WRITE") { handleBatchWrite(parts, partCount); }
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
    if (modeStr == "OUT") { 
        pinMode(pin, OUTPUT);
        outputCommandsSent = true;  // Output mode - enable failsafe
    }
    else if (modeStr == "IN") { pinMode(pin, INPUT); }
    else if (modeStr == "IN_PULLUP") { 
        pinMode(pin, INPUT_PULLUP);
        outputCommandsSent = true;  // Pullup can source current - enable failsafe
    }
    else { Serial.println("<ERROR:Invalid mode>"); return; }
    Serial.println("<OK>");
}

void handleDigitalWrite(String pinStr, String valStr) {
    int pin = pinStr.toInt();
    if (!isValidPin(pin)) { Serial.println("<ERROR:Invalid pin>"); return; }
    digitalWrite(pin, valStr.toInt() == 1 ? HIGH : LOW);
    outputCommandsSent = true;  // Writing to pin - enable failsafe
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
  outputCommandsSent = true;  // DAC output - enable failsafe
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

// PWM Functions
int findPWMChannel(int pin) {
    for (int i = 0; i < MAX_PWM_CHANNELS; i++) {
        if (pwmChannels[i].active && pwmChannels[i].pin == pin) {
            return i;
        }
    }
    return -1;
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
    
    Serial.print("<");
    Serial.print(channel);
    Serial.println(">");
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
    Serial.println("<OK>");
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
    Serial.println("<OK>");
}

// EEPROM Functions
void handleEEPROMRead(String addrStr) {
    int addr = addrStr.toInt();
    
    if (addr < 0 || addr >= EEPROM_SIZE) {
        Serial.println("<ERROR:Address out of range>");
        return;
    }
    
    byte value = EEPROM.read(addr);
    Serial.print("<");
    Serial.print(value);
    Serial.println(">");
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
    Serial.println("<OK>");
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
    
    Serial.println("<OK>");
}

void handleEEPROMCommit() {
    if (EEPROM.commit()) {
        Serial.println("<OK>");
    } else {
        Serial.println("<ERROR:EEPROM commit failed>");
    }
}

void handleEEPROMClear() {
    for (int i = 0; i < EEPROM_SIZE; i++) {
        EEPROM.write(i, 0);
    }
    if (EEPROM.commit()) {
        Serial.println("<OK>");
    } else {
        Serial.println("<ERROR:EEPROM clear failed>");
    }
}

// Batch Operations
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
        
        digitalWrite(pin, value == 1 ? HIGH : LOW);
    }
    
    outputCommandsSent = true;  // Batch write - enable failsafe
    Serial.println("<OK>");
}