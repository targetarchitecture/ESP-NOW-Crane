#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <ArduinoJson.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <LOLIN_I2C_MOTOR.h>
#include <map>
#include <string>
#include <Adafruit_NeoPixel.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

// Debug mode flag - enables/disables Serial output for debugging
#define DEBUG_MODE true

// Debug output macros - only active when DEBUG_MODE is true
#if DEBUG_MODE
#define debugPrintln(message) Serial.println(message)
#define debugPrint(message) Serial.print(message)
#else
#define debugPrintln(message)  // Debug messages disabled
#define debugPrint(message)    // Debug messages disabled
#endif

// Map to store button states and their corresponding values
// 0 = inactive/off, 1 = active/on
std::map<std::string, int> buttons = {
  { "ANTICLOCKWISE", 0 },  // Controls rotation counter-clockwise
  { "DOWN", 0 },        // Controls downward movement
  { "OUT", 0 },         // Controls extension outward
  { "CLOCKWISE", 0 },    // Controls rotation clockwise
  { "UP", 0 },          // Controls upward movement
  { "IN", 0 }           // Controls retraction inward
};

// Configuration structure to store all system settings
struct Config {
  // OTA (Over-The-Air) Update Configuration
  char hostname[32] = "crane-controller";   // Device hostname for network identification
  char ota_password[32] = "xxx";        // Password for OTA updates

  // Motor Control Configuration
  uint8_t motor_speed = 100;   // Default motor speed (0-100%)

  // WiFi Network Configuration
  char wifi_ssid[32] = "the robot network";  // WiFi network name
  char wifi_password[32] = "isaacasimov";     // WiFi network password
  
  // ESP-NOW Configuration
  uint8_t controller_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Controller MAC address (broadcast by default)
  
  // System Timing Configuration
  int wifi_timeout = 20;    // WiFi connection timeout (seconds)
} config;

// Initialize motor control objects with their I2C addresses
LOLIN_I2C_MOTOR motor1(0x20);  // First motor board at I2C address 0x20
LOLIN_I2C_MOTOR motor2(0x21);  // Second motor board at I2C address 0x21

// System state tracking
unsigned long lastMessageTime = 0;  // Timestamp of last received ESP-NOW message
const unsigned long MESSAGE_TIMEOUT = 1000;  // Timeout period in milliseconds (1 second)

// ESP-NOW communication status
bool espNowInitialized = false;

// NeoPixel LED configuration
#define LED_PIN D4 // GPIO pin connected to the NeoPixel data line.
#define LED_COUNT 7 // Number of LEDs in your NeoPixel strip.
#define CENTER_LED 0
#define STEPS_PER_CYCLE 12

Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

int currentLED = 1;
const uint32_t BRIGHT_RED = 0xFF0000;
const uint32_t MID_RED = 0xA00000;
const uint32_t DIM_RED = 0x500000;
const uint32_t VERY_DIM_RED = 0x200000;

unsigned long previousCenterBlinkMillis = 0;
unsigned long previousAnimationMillis = 0;

const long centerBlinkInterval = 1000;
const long animationInterval = 60;

bool centerLedState = false;
bool animationRunning = true;

// FreeRTOS handles
TaskHandle_t otaTaskHandle = NULL;
TaskHandle_t motorControlTaskHandle = NULL;
TaskHandle_t ledTaskHandle = NULL;
TaskHandle_t watchdogTaskHandle = NULL;

// FreeRTOS mutex for buttons access
SemaphoreHandle_t buttonsMutex;

// Structure to hold ESP-NOW message data
typedef struct {
  char message[200]; // JSON message containing button states
} espnow_message_t;

// Forward declarations
void setupMotors();
bool setupESPNow();
void stopAllMotors();
void controlMotor();
void updateLEDs();
bool connectToWiFi();
void setupOTA();

// FreeRTOS task prototypes
void otaTask(void *pvParameters);
void motorControlTask(void *pvParameters);
void ledTask(void *pvParameters);
void watchdogTask(void *pvParameters);

// Callback function for ESP-NOW data reception
void onDataReceived(uint8_t *mac, uint8_t *data, uint8_t len) {
  // Update last message time
  lastMessageTime = millis();
  
  debugPrintln("ESP-NOW message received");
  
  // Check if data is valid
  if (len <= 0 || len > sizeof(espnow_message_t)) {
    debugPrintln("Invalid data length");
    return;
  }
  
  // Create a buffer for the message
  char messageBuffer[200];
  memcpy(messageBuffer, data, min(len, sizeof(messageBuffer) - 1));
  messageBuffer[min(len, sizeof(messageBuffer) - 1)] = '\0'; // Ensure null termination
  
  debugPrint("Message: ");
  debugPrintln(messageBuffer);
  
  // Parse JSON message for movement commands
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, messageBuffer);

  if (error) {
    debugPrint("deserializeJson() failed: ");
    debugPrintln(error.c_str());
    return;
  }

  // Take mutex before updating button states
  if (xSemaphoreTake(buttonsMutex, portMAX_DELAY) == pdTRUE) {
    // Update button states from JSON message
    if (doc.containsKey("ANTICLOCKWISE")) {
      buttons["ANTICLOCKWISE"] = doc["ANTICLOCKWISE"];
    }
    if (doc.containsKey("CLOCKWISE")) {
      buttons["CLOCKWISE"] = doc["CLOCKWISE"];
    }
    if (doc.containsKey("UP")) {
      buttons["UP"] = doc["UP"];
    }
    if (doc.containsKey("DOWN")) {
      buttons["DOWN"] = doc["DOWN"];
    }
    if (doc.containsKey("OUT")) {
      buttons["OUT"] = doc["OUT"];
    }
    if (doc.containsKey("IN")) {
      buttons["IN"] = doc["IN"];
    }
    
    // Release mutex
    xSemaphoreGive(buttonsMutex);
  }
}

// OTA Task
void otaTask(void *pvParameters) {
  const TickType_t xDelay = 10 / portTICK_PERIOD_MS; // 10ms
  
  for (;;) {
    ArduinoOTA.handle();
    vTaskDelay(xDelay);
  }
}

// Motor Control Task
void motorControlTask(void *pvParameters) {
  const TickType_t xDelay = 50 / portTICK_PERIOD_MS; // 50ms
  
  for (;;) {
    // Take mutex before accessing button states
    if (xSemaphoreTake(buttonsMutex, portMAX_DELAY) == pdTRUE) {
      // Check for message timeout
      if (millis() - lastMessageTime > MESSAGE_TIMEOUT) {
        // Reset all button states
        buttons["ANTICLOCKWISE"] = 0;
        buttons["CLOCKWISE"] = 0;
        buttons["UP"] = 0;
        buttons["DOWN"] = 0;
        buttons["OUT"] = 0;
        buttons["IN"] = 0;
        
        // Stop all motors
        stopAllMotors();
      } else {
        // Execute motor control based on button states
        controlMotor();
      }
      
      // Release mutex
      xSemaphoreGive(buttonsMutex);
    }
    
    vTaskDelay(xDelay);
  }
}

// LED Control Task
void ledTask(void *pvParameters) {
  const TickType_t xDelay = 10 / portTICK_PERIOD_MS; // 10ms
  
  for (;;) {
    unsigned long currentMillis = millis();
    bool updateDisplay = false;

    if (currentMillis - previousCenterBlinkMillis >= centerBlinkInterval) {
      previousCenterBlinkMillis = currentMillis;
      centerLedState = !centerLedState;
      updateDisplay = true;
    }

    if (animationRunning && currentMillis - previousAnimationMillis >= animationInterval) {
      previousAnimationMillis = currentMillis;
      currentLED = (currentLED % STEPS_PER_CYCLE) + 1;
      updateDisplay = true;
    }

    if (updateDisplay) {
      updateLEDs();
    }
    
    vTaskDelay(xDelay);
  }
}

// Watchdog Task - monitor ESP-NOW and other system services
void watchdogTask(void *pvParameters) {
  const TickType_t xDelay = 5000 / portTICK_PERIOD_MS; // 5 seconds
  
  for (;;) {
    // If ESP-NOW is not initialized, try to set it up
    if (!espNowInitialized) {
      espNowInitialized = setupESPNow();
      if (espNowInitialized) {
        debugPrintln("ESP-NOW reinitialized successfully");
      }
    }
    
    vTaskDelay(xDelay);
  }
}

// Initialize and verify motor shield connections
void setupMotors() {
  // Wait for first motor board to be ready
  while (motor1.PRODUCT_ID != PRODUCT_ID_I2C_MOTOR) {
    motor1.getInfo();
    delay(5);
  }

  // Wait for second motor board to be ready
  while (motor2.PRODUCT_ID != PRODUCT_ID_I2C_MOTOR) {
    motor2.getInfo();
    delay(5);
  }
}

// Emergency stop function - stops all motors immediately
void stopAllMotors() {
  // Stop all motors on both boards
  motor1.changeStatus(MOTOR_CH_BOTH, MOTOR_STATUS_STOP);
  motor2.changeStatus(MOTOR_CH_BOTH, MOTOR_STATUS_STOP);

  debugPrintln("All motors stopped");
}

// Controls motor movement based on button states
void controlMotor() {
  // Rotation control (Motor A on first board)
  if (buttons["ANTICLOCKWISE"] == 1 && buttons["CLOCKWISE"] == 0) {
    motor1.changeStatus(MOTOR_CH_A, MOTOR_STATUS_CCW);
    motor1.changeDuty(MOTOR_CH_A, config.motor_speed);
  }
  if (buttons["ANTICLOCKWISE"] == 0 && buttons["CLOCKWISE"] == 1) {
    motor1.changeStatus(MOTOR_CH_A, MOTOR_STATUS_CW);
    motor1.changeDuty(MOTOR_CH_A, config.motor_speed);
  }
  if (buttons["ANTICLOCKWISE"] == 0 && buttons["CLOCKWISE"] == 0) {
    motor1.changeStatus(MOTOR_CH_A, MOTOR_STATUS_STOP);
  }

  // Vertical movement control (Motor B on first board)
  if (buttons["UP"] == 1 && buttons["DOWN"] == 0) {
    motor1.changeStatus(MOTOR_CH_B, MOTOR_STATUS_CCW);
    motor1.changeDuty(MOTOR_CH_B, config.motor_speed);
  }
  if (buttons["UP"] == 0 && buttons["DOWN"] == 1) {
    motor1.changeStatus(MOTOR_CH_B, MOTOR_STATUS_CW);
    motor1.changeDuty(MOTOR_CH_B, config.motor_speed);
  }
  if (buttons["UP"] == 0 && buttons["DOWN"] == 0) {
    motor1.changeStatus(MOTOR_CH_B, MOTOR_STATUS_STOP);
  }

  // Extension control (Motor A on second board)
  if (buttons["IN"] == 1 && buttons["OUT"] == 0) {
    motor2.changeStatus(MOTOR_CH_B, MOTOR_STATUS_CCW);
    motor2.changeDuty(MOTOR_CH_B, config.motor_speed);
  }
  if (buttons["IN"] == 0 && buttons["OUT"] == 1) {
    motor2.changeStatus(MOTOR_CH_B, MOTOR_STATUS_CW);
    motor2.changeDuty(MOTOR_CH_B, config.motor_speed);
  }
  if (buttons["IN"] == 0 && buttons["OUT"] == 0) {
    motor2.changeStatus(MOTOR_CH_B, MOTOR_STATUS_STOP);
  }
}

// Initialize ESP-NOW
bool setupESPNow() {
  // Init ESP-NOW
  if (esp_now_init() != 0) {
    debugPrintln("Error initializing ESP-NOW");
    return false;
  }

  // Set ESP-NOW role
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  
  // Register callback function
  esp_now_register_recv_cb(onDataReceived);
  
  debugPrintln("ESP-NOW initialized and ready");
  return true;
}

// Attempts to connect to WiFi network for OTA updates
bool connectToWiFi() {
  debugPrintln("Connecting to WiFi for OTA...");
  WiFi.mode(WIFI_STA);  // Set WiFi mode to Station (client)
  WiFi.begin(config.wifi_ssid, config.wifi_password);

  // Wait for connection with timeout
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < config.wifi_timeout) {
    delay(500);
    debugPrint(".");
    timeout++;
  }

  // Report connection status
  if (WiFi.status() == WL_CONNECTED) {
    debugPrintln("");
    debugPrint("Connected to WiFi network with IP Address: ");
    debugPrintln(WiFi.localIP());
    return true;
  } else {
    debugPrintln("");
    debugPrintln("Failed to connect to WiFi");
    return false;
  }
}

// Configure Over-The-Air update functionality
void setupOTA() {
  // Set device hostname for network identification
  ArduinoOTA.setHostname(config.hostname);

  // Set OTA update password
  ArduinoOTA.setPassword(config.ota_password);

  // Configure OTA event handlers
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else  // U_SPIFFS
      type = "filesystem";

    // Stop motors during update
    stopAllMotors();

    debugPrintln("Start updating " + type);
  });

  // Handle OTA completion
  ArduinoOTA.onEnd([]() {
    debugPrintln("\nOTA Update Complete");
    
    // Re-initialize ESP-NOW after OTA update
    setupESPNow();
  });

  // Display update progress
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    debugPrint("Progress: ");
    debugPrint(progress / (total / 100));
    debugPrintln("%");
  });

  // Handle OTA errors
  ArduinoOTA.onError([](ota_error_t error) {
    debugPrint("Error[");
    debugPrint(error);
    debugPrintln("]: ");

    if (error == OTA_AUTH_ERROR) debugPrintln("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) debugPrintln("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) debugPrintln("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) debugPrintln("Receive Failed");
    else if (error == OTA_END_ERROR) debugPrintln("End Failed");
    
    // Re-initialize ESP-NOW after OTA error
    setupESPNow();
  });

  // Start OTA service
  ArduinoOTA.begin();
  debugPrintln("OTA Update Service Started");
}

void updateLEDs() {
  pixels.clear();
  if (centerLedState) {
    pixels.setPixelColor(CENTER_LED, MID_RED);
  }

  int mainLED = ((currentLED - 1) / 2) + 1;

  if (currentLED % 2 == 1) {
    pixels.setPixelColor(mainLED, BRIGHT_RED);

    int prevLED = (mainLED == 1) ? 6 : mainLED - 1;
    int prevPrevLED = (prevLED == 1) ? 6 : prevLED - 1;

    pixels.setPixelColor(prevLED, MID_RED);
    pixels.setPixelColor(prevPrevLED, DIM_RED);

    int prevPrevPrevLED = (prevPrevLED == 1) ? 6 : prevPrevLED - 1;
    pixels.setPixelColor(prevPrevPrevLED, VERY_DIM_RED);
  } else {
    int nextLED = (mainLED == 6) ? 1 : mainLED + 1;

    pixels.setPixelColor(mainLED, BRIGHT_RED);
    pixels.setPixelColor(nextLED, MID_RED);

    int prevLED = (mainLED == 1) ? 6 : mainLED - 1;
    int prevPrevLED = (prevLED == 1) ? 6 : prevLED - 1;

    pixels.setPixelColor(prevLED, DIM_RED);
    pixels.setPixelColor(prevPrevLED, VERY_DIM_RED);
  }
  pixels.show();
}

// System initialization
void setup() {
#if DEBUG_MODE
  Serial.begin(115200);  // Initialize serial communication for debugging
  delay(100);            // Wait for serial to stabilize
#endif

  debugPrintln("ESP8266 Crane Motor Controller with FreeRTOS");

  // Initialize I2C communication for motor control
  Wire.begin();

  // Initialize motor shields
  setupMotors();

  // Initialize last message time
  lastMessageTime = millis();

  // Initialize NeoPixel
  pixels.begin();
  pixels.setBrightness(100);
  pixels.clear();
  pixels.show();

  // Connect to WiFi for OTA updates
  if (connectToWiFi()) {
    // Setup OTA update capability
    setupOTA();
  } else {
    // If WiFi connection fails, switch to ESP-NOW only mode
    WiFi.disconnect();
    // Need to set STA mode even without connecting to a network for ESP-NOW
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
  }

  // Setup ESP-NOW after WiFi is configured
  espNowInitialized = setupESPNow();

  // Create mutex for button access
  buttonsMutex = xSemaphoreCreateMutex();
  
  // Create FreeRTOS tasks
  xTaskCreate(
    otaTask,            // Function that implements the task
    "OTATask",          // Text name for the task
    2048,               // Stack size in words, not bytes
    NULL,               // Parameter passed into the task
    1,                  // Priority at which the task is created
    &otaTaskHandle      // Used to pass out the created task's handle
  );
  
  xTaskCreate(
    motorControlTask,
    "MotorTask",
    2048,
    NULL,
    2,                  // Higher priority than OTA task
    &motorControlTaskHandle
  );
  
  xTaskCreate(
    ledTask,
    "LEDTask",
    1024,
    NULL,
    1,
    &ledTaskHandle
  );
  
  xTaskCreate(
    watchdogTask,
    "WatchdogTask",
    1024,
    NULL,
    1,
    &watchdogTaskHandle
  );

  debugPrintln("FreeRTOS setup complete");
}

// Main program loop - not used with FreeRTOS
void loop() {
  // Empty since we're using FreeRTOS tasks
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
