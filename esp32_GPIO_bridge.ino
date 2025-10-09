#include <Wire.h>
#include <EEPROM.h>
#include "driver/i2s.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_wifi.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define FW_VERSION "0.1.5-beta"
#define BAUD_RATE 115200
#define SERIAL_RX_BUFFER 1024        // Increased from default 256 for better throughput
#define SERIAL_TX_BUFFER 1024        // Increased from default 256 for better throughput
#define CMD_BUFFER_SIZE 256          // Command buffer size for char-based parsing
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

// Command buffer for optimized char-based parsing (replaces String)
char cmdBuffer[CMD_BUFFER_SIZE];
int cmdIndex = 0;
bool inCommand = false;

// Command queuing system (v0.1.5-beta optimization)
#define CMD_QUEUE_SIZE 32
struct QueuedCommand {
  char command[CMD_BUFFER_SIZE];
  bool valid;
};
QueuedCommand cmdQueue[CMD_QUEUE_SIZE];
int queueHead = 0;
int queueTail = 0;
int queueCount = 0;

// Response buffer for efficient serial output (v0.1.5-beta optimization)
#define RESPONSE_BUFFER_SIZE 512
char responseBuffer[RESPONSE_BUFFER_SIZE];
int responseIndex = 0;

// PWM management
struct PWMChannel {
  int pin;
  int channel;
  bool active;
};
PWMChannel pwmChannels[MAX_PWM_CHANNELS];
int nextPWMChannel = 0;

// O(1) PWM channel lookup optimization (v0.1.5-beta)
int8_t pinToPWMChannel[MAX_PINS];  // -1 = unused

// FreeRTOS task handles
TaskHandle_t serialTaskHandle;
TaskHandle_t failsafeTaskHandle;
SemaphoreHandle_t sharedDataMutex;

// Queue management functions (v0.1.5-beta)
bool enqueueCommand(const char* cmd) {
  if (queueCount >= CMD_QUEUE_SIZE) return false;  // Queue full
  
  strncpy(cmdQueue[queueTail].command, cmd, CMD_BUFFER_SIZE - 1);
  cmdQueue[queueTail].command[CMD_BUFFER_SIZE - 1] = '\0';
  cmdQueue[queueTail].valid = true;
  
  queueTail = (queueTail + 1) % CMD_QUEUE_SIZE;
  queueCount++;
  return true;
}

bool dequeueCommand(char* cmd) {
  if (queueCount == 0) return false;  // Queue empty
  
  strncpy(cmd, cmdQueue[queueHead].command, CMD_BUFFER_SIZE - 1);
  cmd[CMD_BUFFER_SIZE - 1] = '\0';
  cmdQueue[queueHead].valid = false;
  
  queueHead = (queueHead + 1) % CMD_QUEUE_SIZE;
  queueCount--;
  return true;
}

// Response buffer functions (v0.1.5-beta)
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
  
  // Optimize serial buffers for better throughput (v0.1.4 optimization)
  Serial.setRxBufferSize(SERIAL_RX_BUFFER);
  Serial.setTxBufferSize(SERIAL_TX_BUFFER);
  Serial.begin(BAUD_RATE);
  
  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Initialize PWM channels
  for (int i = 0; i < MAX_PWM_CHANNELS; i++) {
    pwmChannels[i].pin = -1;
    pwmChannels[i].channel = i;
    pwmChannels[i].active = false;
  }
  
  // Initialize PWM channel lookup mapping (v0.1.5-beta optimization)
  for (int i = 0; i < MAX_PINS; i++) {
    pinToPWMChannel[i] = -1;  // -1 = unused
  }
  
  // Initialize command queue (v0.1.5-beta optimization)
  for (int i = 0; i < CMD_QUEUE_SIZE; i++) {
    cmdQueue[i].valid = false;
    cmdQueue[i].command[0] = '\0';
  }
  queueHead = 0;
  queueTail = 0;
  queueCount = 0;
  
  // Initialize response buffer (v0.1.5-beta optimization)
  clearResponse();
  
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
  
  // Create mutex for shared data protection (v0.1.5-beta)
  sharedDataMutex = xSemaphoreCreateMutex();
  if (sharedDataMutex == NULL) {
    Serial.println("<ERROR:Failed to create mutex>");
    return;
  }
  
  // Create FreeRTOS tasks (v0.1.5-beta)
  xTaskCreatePinnedToCore(
    serialTask,           // Task function
    "SerialTask",         // Task name
    4096,                 // Stack size
    NULL,                 // Parameters
    2,                    // Priority (high for responsiveness)
    &serialTaskHandle,    // Task handle
    0                     // Core 0 (protocol CPU)
  );
  
  xTaskCreatePinnedToCore(
    failsafeTask,         // Task function
    "FailsafeTask",       // Task name
    2048,                 // Stack size
    NULL,                 // Parameters
    1,                    // Priority (lower than serial)
    &failsafeTaskHandle,  // Task handle
    1                     // Core 1 (application CPU)
  );
  
  Serial.println("<INFO:FreeRTOS tasks initialized - Dual-core operation enabled>");
}

// FreeRTOS Tasks (v0.1.5-beta)
void serialTask(void* parameter) {
  while (true) {
    // Phase 1: Parse and queue incoming commands
    while (Serial.available()) {
      char c = Serial.read();
      
      if (c == '<') {
        // Start of command
        cmdIndex = 0;
        inCommand = true;
      }
      else if (c == '>' && inCommand) {
        // End of command - queue it for processing
        cmdBuffer[cmdIndex] = '\0';  // Null terminate
        
        // Enqueue command for batch processing
        if (!enqueueCommand(cmdBuffer)) {
          // Queue full - process immediately
          if (xSemaphoreTake(sharedDataMutex, portMAX_DELAY) == pdTRUE) {
            processCommand(cmdBuffer);
            updateCommandTimestamps();
            xSemaphoreGive(sharedDataMutex);
          }
        }
        
        inCommand = false;
      }
      else if (inCommand && cmdIndex < CMD_BUFFER_SIZE - 1) {
        // Build command buffer
        cmdBuffer[cmdIndex++] = c;
      }
      else if (cmdIndex >= CMD_BUFFER_SIZE - 1) {
        // Buffer overflow protection
        inCommand = false;
        cmdIndex = 0;
        Serial.println("<ERROR:Command too long>");
      }
    }
    
    // Phase 2: Process queued commands in batch
    if (queueCount > 0) {
      if (xSemaphoreTake(sharedDataMutex, portMAX_DELAY) == pdTRUE) {
        // Process up to 5 commands per batch for optimal throughput
        int batchSize = min(queueCount, 5);
        for (int i = 0; i < batchSize; i++) {
          char queuedCmd[CMD_BUFFER_SIZE];
          if (dequeueCommand(queuedCmd)) {
            processCommand(queuedCmd);
          }
        }
        updateCommandTimestamps();
        xSemaphoreGive(sharedDataMutex);
      }
    }
    
    vTaskDelay(1 / portTICK_PERIOD_MS);  // Yield to other tasks
  }
}

// Helper function to update command timestamps (v0.1.5-beta)
void updateCommandTimestamps() {
  lastCommandTime = millis();
  lastPingTime = millis();
  
  // Reset failsafe states when communication resumes
  if (failsafeEngaged) {
    Serial.println("<INFO:Failsafe disengaged - Communication restored>");
    disengageFailsafe();
  }
  failsafeWarningSent = false;
}

void failsafeTask(void* parameter) {
  while (true) {
    unsigned long currentTime = millis();
    
    // Protect shared data access
    if (xSemaphoreTake(sharedDataMutex, portMAX_DELAY) == pdTRUE) {
      checkFailsafe(currentTime);
      xSemaphoreGive(sharedDataMutex);
    }
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Check every 1 second
  }
}

void loop() {
  // Main loop is now minimal - just wait for tasks to complete
  // All actual work is done by FreeRTOS tasks (v0.1.5-beta)
  vTaskDelay(portMAX_DELAY);
}

void checkFailsafe(unsigned long currentTime) {
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

void processCommand(char* cmd) {
    // Optimized char-based parsing (v0.1.4 - no String allocation)
    char* parts[10];
    int partCount = 0;
    
    // Parse command using strtok
    char* token = strtok(cmd, " ");
    while (token != NULL && partCount < 10) {
        parts[partCount++] = token;
        token = strtok(NULL, " ");
    }
    
    if (partCount == 0) return;
    
    // Convert action to uppercase for comparison
    char* action = parts[0];
    for (char* p = action; *p; p++) *p = toupper(*p);

    if (strcmp(action, "PING") == 0) {
        clearResponse();
        addToResponse("<PONG>");
        sendResponse();
        lastPingTime = millis(); // Update ping time for failsafe detection
    }
    else if (strcmp(action, "IDENTIFY") == 0) {
        clearResponse();
        addToResponse("<ESP32_GPIO_BRIDGE:");
        addToResponse(FW_VERSION);
        addToResponse(">");
        sendResponse();
    }
    else if (strcmp(action, "VERSION") == 0) {
        clearResponse();
        addToResponse("<");
        addToResponse(FW_VERSION);
        addToResponse(">");
        sendResponse();
    }
    else if (strcmp(action, "STATUS") == 0) {
        clearResponse();
        addToResponse("<STATUS:");
        addToResponse(failsafeEngaged ? "FAILSAFE" : "NORMAL");
        addToResponse(",");
        addToResponse(millis() - lastCommandTime);
        addToResponse(",");
        addToResponse(millis() - lastPingTime);
        addToResponse(">");
        sendResponse();
    }
    else if (strcmp(action, "RESET_FAILSAFE") == 0 || strcmp(action, "CLEAR_FAILSAFE") == 0 || strcmp(action, "DISABLE_FAILSAFE") == 0) {
        if (failsafeEngaged) {
            Serial.println("<INFO:Manually disengaging failsafe>");
            disengageFailsafe();
            // OK response removed (v0.1.4 optimization)
        } else {
            Serial.println("<INFO:Failsafe not engaged>");
        }
    }
    // Command dispatch - char buffer parsing complete, handlers use String for simplicity
    // This hybrid approach keeps hot path (loop/parsing) fast while minimizing code changes
    else if (strcmp(action, "MODE") == 0 && partCount >= 3) { handlePinMode(parts[1], parts[2]); }
    else if (strcmp(action, "WRITE") == 0 && partCount >= 3) { handleDigitalWrite(String(parts[1]), String(parts[2])); }
    else if (strcmp(action, "READ") == 0 && partCount >= 2) { handleDigitalRead(String(parts[1])); }
    else if (strcmp(action, "AREAD") == 0 && partCount >= 2) { handleAnalogRead(String(parts[1])); }
    else if (strcmp(action, "AWRITE") == 0 && partCount >= 3) { handleAnalogWrite(String(parts[1]), String(parts[2])); }
    else if (strcmp(action, "I2C_INIT") == 0 && partCount >= 3) { handleI2CInit(String(parts[1]), String(parts[2])); }
    else if (strcmp(action, "I2C_SCAN") == 0) { handleI2CScan(); }
    else if (strcmp(action, "I2C_WRITE") == 0 && partCount >= 3) { 
        // Convert parts array to String array for handler
        String strParts[10];
        for (int i = 0; i < partCount && i < 10; i++) strParts[i] = String(parts[i]);
        handleI2CWrite(strParts[1], strParts, partCount);
    }
    else if (strcmp(action, "I2C_READ") == 0 && partCount >= 3) { handleI2CRead(String(parts[1]), String(parts[2])); }
    else if (strcmp(action, "I2S_INIT_TX") == 0 && partCount >= 5) { handleI2SInitTx(String(parts[1]), String(parts[2]), String(parts[3]), String(parts[4])); }
    else if (strcmp(action, "I2S_WRITE") == 0 && partCount >= 2) {
        String strParts[10];
        for (int i = 0; i < partCount && i < 10; i++) strParts[i] = String(parts[i]);
        handleI2SWrite(strParts, partCount);
    }
    else if (strcmp(action, "PWM_INIT") == 0 && partCount >= 4) { handlePWMInit(String(parts[1]), String(parts[2]), String(parts[3])); }
    else if (strcmp(action, "PWM_WRITE") == 0 && partCount >= 3) { handlePWMWrite(String(parts[1]), String(parts[2])); }
    else if (strcmp(action, "PWM_STOP") == 0 && partCount >= 2) { handlePWMStop(String(parts[1])); }
    else if (strcmp(action, "EEPROM_READ") == 0 && partCount >= 2) { handleEEPROMRead(String(parts[1])); }
    else if (strcmp(action, "EEPROM_WRITE") == 0 && partCount >= 3) { handleEEPROMWrite(String(parts[1]), String(parts[2])); }
    else if (strcmp(action, "EEPROM_READ_BLOCK") == 0 && partCount >= 3) { handleEEPROMReadBlock(String(parts[1]), String(parts[2])); }
    else if (strcmp(action, "EEPROM_WRITE_BLOCK") == 0 && partCount >= 3) {
        String strParts[10];
        for (int i = 0; i < partCount && i < 10; i++) strParts[i] = String(parts[i]);
        handleEEPROMWriteBlock(strParts, partCount);
    }
    else if (strcmp(action, "EEPROM_COMMIT") == 0) { handleEEPROMCommit(); }
    else if (strcmp(action, "EEPROM_CLEAR") == 0) { handleEEPROMClear(); }
    else if (strcmp(action, "BATCH_WRITE") == 0 && partCount >= 3) {
        String strParts[10];
        for (int i = 0; i < partCount && i < 10; i++) strParts[i] = String(parts[i]);
        handleBatchWrite(strParts, partCount);
    }
    else { Serial.println("<ERROR:Unknown or incomplete command>"); }
}

bool isValidPin(int pin) { return (pin >= 0 && pin < MAX_PINS); }

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
        outputCommandsSent = true;  // Output mode - enable failsafe
    }
    else if (strcmp(mode, "IN") == 0) { pinMode(pin, INPUT); }
    else if (strcmp(mode, "IN_PULLUP") == 0) { 
        pinMode(pin, INPUT_PULLUP);
        outputCommandsSent = true;  // Pullup can source current - enable failsafe
    }
    else if (strcmp(mode, "IN_PULLDOWN") == 0) { 
        pinMode(pin, INPUT_PULLDOWN);
    }
    else { Serial.println("<ERROR:Invalid mode>"); return; }
    // OK response removed (v0.1.4 optimization - no response for write commands)
}

void handleDigitalWrite(String pinStr, String valStr) {
    int pin = pinStr.toInt();
    if (!isValidPin(pin)) { Serial.println("<ERROR:Invalid pin>"); return; }
    digitalWrite(pin, valStr.toInt() == 1 ? HIGH : LOW);
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
    // OK response removed (v0.1.4 optimization)
}

void handleI2SWrite(String parts[], int partCount) {
    size_t bytes_written = 0;
    int16_t* buffer = new int16_t[partCount - 1];
    for(int i=1; i < partCount; i++) {
        buffer[i-1] = (int16_t)parts[i].toInt();
    }
    i2s_write(I2S_NUM_0, buffer, (partCount-1) * sizeof(int16_t), &bytes_written, portMAX_DELAY);
    delete[] buffer;
    // OK response removed (v0.1.4 optimization)
}

// PWM Functions
// O(1) PWM channel lookup optimization (v0.1.5-beta)
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
            pinToPWMChannel[pin] = i;  // Update O(1) mapping (v0.1.5-beta)
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
    pinToPWMChannel[pin] = -1;  // Clear O(1) mapping (v0.1.5-beta)
    // OK response removed (v0.1.4 optimization)
}

// EEPROM Functions
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
    // OK response removed (v0.1.4 optimization)
}