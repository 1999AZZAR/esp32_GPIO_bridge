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
#include "config.h"
#include "response.h"
#include "gpio.h"
#include "pwm.h"
#include "analog.h"
#include "i2c.h"
#include "eeprom.h"
#include "i2s.h"

unsigned long lastCommandTime = 0;
unsigned long lastPingTime = 0;
bool failsafeEngaged = false;
bool failsafeWarningSent = false;
bool configuredPins[MAX_PINS] = {false};
bool adcInitialized = false;
esp_adc_cal_characteristics_t *adc_chars = NULL;
bool outputCommandsSent = false;  // Track if any output commands were sent

// Safe mode configuration and pin state tracking
uint8_t currentSafeMode = DEFAULT_SAFE_MODE;  // Current safe mode type
uint8_t lastPinModes[MAX_PINS] = {0};         // Last pin modes (INPUT=0, OUTPUT=1, etc.)
uint8_t lastPinValues[MAX_PINS] = {0};        // Last pin values (LOW=0, HIGH=1)
bool pinStatesTracked[MAX_PINS] = {false};    // Track if pin state is recorded

// Command buffer for optimized char-based parsing (replaces String)
char cmdBuffer[CMD_BUFFER_SIZE];
int cmdIndex = 0;
bool inCommand = false;

// Command queuing system (v0.1.6-beta modular architecture)
struct QueuedCommand {
  char command[CMD_BUFFER_SIZE];
  bool valid;
};
QueuedCommand cmdQueue[CMD_QUEUE_SIZE];
int queueHead = 0;
int queueTail = 0;
int queueCount = 0;

// Response buffer for efficient serial output (v0.1.6-beta modular architecture)
char responseBuffer[RESPONSE_BUFFER_SIZE];
int responseIndex = 0;

// PWM management (structure now in pwm.h)
PWMChannel pwmChannels[MAX_PWM_CHANNELS];
int nextPWMChannel = 0;

// O(1) PWM channel lookup optimization (v0.1.6-beta)
int8_t pinToPWMChannel[MAX_PINS];  // -1 = unused

// FreeRTOS task handles
TaskHandle_t serialTaskHandle;
TaskHandle_t failsafeTaskHandle;
SemaphoreHandle_t sharedDataMutex;

// Pin state tracking functions for safe mode
void trackPinState(int pin, uint8_t mode, uint8_t value) {
  if (pin >= 0 && pin < MAX_PINS) {
    lastPinModes[pin] = mode;
    lastPinValues[pin] = value;
    pinStatesTracked[pin] = true;
  }
}

void restorePinStates() {
  for (int i = 0; i < MAX_PINS; i++) {
    if (pinStatesTracked[i] && configuredPins[i]) {
      // Restore pin mode and value
      pinMode(i, lastPinModes[i]);
      if (lastPinModes[i] == OUTPUT) {
        digitalWrite(i, lastPinValues[i]);
      }
    }
  }
}

// Queue management functions (v0.1.6-beta)
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
  
  // Initialize PWM channel lookup mapping (v0.1.6-beta optimization)
  for (int i = 0; i < MAX_PINS; i++) {
    pinToPWMChannel[i] = -1;  // -1 = unused
  }
  
  // Initialize command queue (v0.1.6-beta optimization)
  for (int i = 0; i < CMD_QUEUE_SIZE; i++) {
    cmdQueue[i].valid = false;
    cmdQueue[i].command[0] = '\0';
  }
  queueHead = 0;
  queueTail = 0;
  queueCount = 0;
  
  // Initialize response buffer (v0.1.6-beta optimization)
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
  
  // Create mutex for shared data protection (v0.1.6-beta)
  sharedDataMutex = xSemaphoreCreateMutex();
  if (sharedDataMutex == NULL) {
    Serial.println("<ERROR:Failed to create mutex>");
    return;
  }
  
  // Create FreeRTOS tasks (v0.1.6-beta)
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

// FreeRTOS Tasks (v0.1.6-beta)
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
    // Continue processing queued commands even in SAFE_MODE_HOLD failsafe
    if (queueCount > 0 && (!failsafeEngaged || currentSafeMode == SAFE_MODE_HOLD)) {
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

// Helper function to update command timestamps (v0.1.6-beta)
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
  // All actual work is done by FreeRTOS tasks (v0.1.6-beta)
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
    
    if (currentSafeMode == SAFE_MODE_RESET) {
        Serial.println("<INFO:Reset mode - All configured pins reset to INPUT mode for safety>");
        // Reset all configured pins to INPUT mode
        for (int i = 0; i < MAX_PINS; i++) {
            if (configuredPins[i]) {
                pinMode(i, INPUT);
                configuredPins[i] = false;
                Serial.println("<INFO:Reset pin " + String(i) + " to INPUT>");
            }
        }
        Serial.println("<INFO:Failsafe active - Waiting for communication recovery>");
    } else if (currentSafeMode == SAFE_MODE_HOLD) {
        Serial.println("<INFO:Hold mode - Pins maintain last position, queued commands will continue executing>");
        Serial.println("<INFO:Failsafe active - Communication lost but operation continues>");
    }

    failsafeEngaged = true;
    failsafeWarningSent = false; // Reset warning state
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
        addToResponse(currentSafeMode == SAFE_MODE_RESET ? "RESET" : "HOLD");
        addToResponse(",");
        addToResponse(millis() - lastCommandTime);
        addToResponse(",");
        addToResponse(millis() - lastPingTime);
        addToResponse(",");
        addToResponse(queueCount);
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
    else if (strcmp(action, "SAFE_MODE_SET") == 0 && partCount >= 2) {
        int mode = atoi(parts[1]);
        if (mode == SAFE_MODE_RESET || mode == SAFE_MODE_HOLD) {
            currentSafeMode = mode;
            clearResponse();
            addToResponse("<INFO:Safe mode set to ");
            addToResponse(mode == SAFE_MODE_RESET ? "RESET" : "HOLD");
            addToResponse(">");
            sendResponse();
        } else {
            Serial.println("<ERROR:Invalid safe mode. Use 0 for RESET or 1 for HOLD>");
        }
    }
    else if (strcmp(action, "SAFE_MODE_GET") == 0) {
        clearResponse();
        addToResponse("<SAFE_MODE:");
        addToResponse(currentSafeMode == SAFE_MODE_RESET ? "RESET" : "HOLD");
        addToResponse(",");
        addToResponse(currentSafeMode);
        addToResponse(">");
        sendResponse();
    }
    else if (strcmp(action, "SAFE_MODE_RESTORE") == 0) {
        if (currentSafeMode == SAFE_MODE_HOLD) {
            restorePinStates();
            clearResponse();
            addToResponse("<INFO:Pin states restored from safe mode tracking>");
            sendResponse();
        } else {
            Serial.println("<INFO:Pin state restore only available in HOLD safe mode>");
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






