#include <Adafruit_NeoPixel.h>
#include "esp_sleep.h"
#include <WiFi.h>
#include <esp_now.h>

// Debug mode - set to false to disable Serial output
#define DEBUG_MODE true

// Define the WS2812 LED pin
#define LED_MODE true
#define LED_PIN 21
#define NUM_LEDS 1

// Define the button pins
const int buttonPins[] = { 1, 2, 3, 4, 5, 6 };
const int numButtons = 6;

// Define colors for each button (in RGB format)
uint32_t buttonColors[] = {
    0xFF9800,   // Vivid Orange (Button 1 - ANTICLOCKWISE)  Y
    0xD32F2F,   // Deep Red (Button 2 - DOWN)               Y
    0x2196F3,   // Bright Blue (Button 3 - OUT)
    0x4CAF50,   // Green (Button 4 - CLOCKWISE)             Y
    0x9C27B0,   // Purple (Button 5 - UP)                   Y
    0x00BCD4    // Cyan/Light Blue (Button 6 - IN)
};

// Variables to keep track of button states
int buttonStates[6] = { LOW, LOW, LOW, LOW, LOW, LOW };
int lastButtonStates[6] = { LOW, LOW, LOW, LOW, LOW, LOW };

unsigned long lastDebounceTime[6] = { 0, 0, 0, 0, 0, 0 };
const unsigned long debounceDelay = 50;  // Debounce time in milliseconds

// ESP-NOW periodic update variables
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 250;  // Send update every 250ms (quarter second)

// Sleep parameters
const unsigned long sleepDelay = 3000;  // Time in ms before going to sleep after button release
unsigned long buttonReleaseTime = 0;    // Time when the last button was released
bool buttonPressed = false;             // Flag to track if any button is pressed
bool readyToSleep = true;               // Flag to indicate ready to enter sleep

// REPLACE WITH RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xB4, 0xE6, 0x2D, 0x53, 0xAF, 0x58};

// ESP-NOW message structure - compact binary format
typedef struct crane_message {
  uint8_t buttonStates;  // Each bit represents a button state (bits 0-5)
} crane_message;

// Create message instance
crane_message craneMsg;

// Initialize the NeoPixel library
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// --- Debugging Macros ---
#if DEBUG_MODE
#define debugPrintln(message) Serial.println(message)
#define debugPrint(message) Serial.print(message)
#else
#define debugPrintln(message)  // do nothing
#define debugPrint(message)    // do nothing
#endif

// Callback function when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  debugPrint("Last Packet Send Status: ");
  debugPrintln(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Function to initialize ESP-NOW
void initESPNow() {
  // Set device as a Wi-Fi Station and disconnect from any AP
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    debugPrintln("Error initializing ESP-NOW");
    // Try to restart ESP-NOW after a delay
    delay(500);
    ESP.restart();
    return;
  }
  
  // Register send callback
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    debugPrintln("Failed to add peer");
    return;
  }

  debugPrintln("ESP-NOW initialized successfully");
}

// Function to send button states via ESP-NOW
void sendButtonStates() {
  // Pack all button states into a single byte (bit 0 = button 1, bit 1 = button 2, etc.)
  craneMsg.buttonStates = 0;
  for (int i = 0; i < numButtons; i++) {
    if (buttonStates[i] == HIGH) {
      craneMsg.buttonStates |= (1 << i);  // Set the bit for this button
    }
  }
  
  // Make sure WiFi is in the correct mode before sending
  if (WiFi.getMode() != WIFI_STA) {
    debugPrintln("WiFi not in STA mode. Fixing...");
    WiFi.mode(WIFI_STA);
    delay(10);
  }

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &craneMsg, sizeof(craneMsg));
  
  if (result == ESP_OK) {
    debugPrintln("ESP-NOW message sent successfully");
  } else {
    // Define peerInfo outside the switch to avoid compiler error
    esp_now_peer_info_t peerInfo = {};
    
    debugPrint("Error sending ESP-NOW message: ");
    switch (result) {
      case ESP_ERR_ESPNOW_NOT_INIT:
        debugPrintln("ESP-NOW not initialized");
        // Try to reinitialize
        esp_now_deinit();
        delay(10);
        initESPNow();
        break;
      case ESP_ERR_ESPNOW_ARG:
        debugPrintln("Invalid argument");
        break;
      case ESP_ERR_ESPNOW_INTERNAL:
        debugPrintln("Internal error");
        break;
      case ESP_ERR_ESPNOW_NO_MEM:
        debugPrintln("Out of memory");
        break;
      case ESP_ERR_ESPNOW_NOT_FOUND:
        debugPrintln("Peer not found");
        // Try to re-add the peer
        memcpy(peerInfo.peer_addr, broadcastAddress, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        esp_now_add_peer(&peerInfo);
        break;
      case ESP_ERR_ESPNOW_IF:
        debugPrintln("Interface error");
        break;
      default:
        debugPrintln("Unknown error");
    }
  }
  
  // Debug output without JSON
  #if DEBUG_MODE

  const char* buttonMovements[] = {
    "ANTICLOCKWISE",  // Button 1
    "DOWN",           // Button 2
    "OUT",            // Button 3
    "CLOCKWISE",      // Button 4
    "UP",             // Button 5
    "IN"              // Button 6
  };
  
  debugPrintln("Button states:");
  for (int i = 0; i < numButtons; i++) {
    debugPrint(buttonMovements[i]);
    debugPrint(": ");
    debugPrintln(buttonStates[i] == HIGH ? "1" : "0");
  }
  #endif
}

// Get wake cause and button that triggered wake up
void printWakeupReason() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT1:
      {
        uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
        debugPrintln("Wakeup caused by external signal using EXT1");

        // Determine which button woke up the device
        for (int i = 0; i < numButtons; i++) {
          if (wakeup_pin_mask & (1ULL << buttonPins[i])) {
            debugPrint("Wakeup button: GPIO ");
            debugPrintln(buttonPins[i]);
            delay(10);

#if LED_MODE
            // Set the LED to the color of the button that woke up the device
            strip.setPixelColor(0, buttonColors[i]);
            strip.show();
#endif
            break;
          }
        }
        break;
      }
    case ESP_SLEEP_WAKEUP_TIMER:
      debugPrintln("Wakeup caused by timer");
      break;
    default:
#if DEBUG_MODE
      Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
#endif
      break;
  }
}

void setup() {
  // Initialize serial communication for debugging
#if DEBUG_MODE
  Serial.begin(115200);
  delay(500);  // Longer delay to ensure serial is ready
#endif

  debugPrintln("ESP32-S3 WS2812 LED Control with Deep Sleep and ESP-NOW");
  delay(100);  // Give more time for initialization

  // Initialize the LED strip
#if LED_MODE
  strip.begin();
  strip.setBrightness(50);  // Set brightness (0-255)
  strip.show();             // Initialize all pixels to 'off'

  // Set initial LED color to black/off
  strip.setPixelColor(0, strip.Color(0, 0, 0));  // Explicitly set to black (R=0, G=0, B=0)
  strip.show();
#endif

  // Initialize button pins as inputs (externally pulled down)
  for (int i = 0; i < numButtons; i++) {
    pinMode(buttonPins[i], INPUT);
    debugPrint("Button on GPIO ");
    debugPrint(buttonPins[i]);
    debugPrintln(" initialized");
    delay(10);
  }

  // Check if the ESP32 woke up from deep sleep
  printWakeupReason();

  // Initialize ESP-NOW with retry mechanism
  for (int retry = 0; retry < 3; retry++) {
    initESPNow();
    delay(100);
    
    // Test if ESP-NOW is working by sending a test message
    craneMsg.buttonStates = 0;  // All buttons released
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &craneMsg, sizeof(craneMsg));
    
    if (result == ESP_OK) {
      debugPrintln("ESP-NOW test message successful");
      break;
    } else {
      debugPrint("ESP-NOW initialization attempt ");
      debugPrint(retry + 1);
      debugPrintln(" failed, retrying...");
      delay(500);
    }
  }

  // Configure the wake-up source (all buttons)
  uint64_t mask = 0;
  for (int i = 0; i < numButtons; i++) {
    mask |= (1ULL << buttonPins[i]);
  }

  // Configure EXT1 wake sources (any high level on specified GPIOs will wake up the ESP32)
  esp_sleep_enable_ext1_wakeup(mask, ESP_EXT1_WAKEUP_ANY_HIGH);

  debugPrintln("Setup complete");
}

void loop() {
  //set variables
  bool anyButtonPressed = false;
  bool buttonStateChanged = false;
  int activeButton = -1;

  // Check each button
  for (int i = 0; i < numButtons; i++) {
    // Read the button state
    int reading = digitalRead(buttonPins[i]);

    // Check if the button state has changed
    if (reading != lastButtonStates[i]) {
      // Reset the debounce timer
      lastDebounceTime[i] = millis();
    }

    // If enough time has passed, consider the state change valid
    if ((millis() - lastDebounceTime[i]) > debounceDelay) {
      // If the button state has changed
      if (reading != buttonStates[i]) {
        buttonStates[i] = reading;
        buttonStateChanged = true;

        // If the button is pressed (HIGH with external pull-down)
        if (buttonStates[i] == HIGH) {
          debugPrint("Button ");
          debugPrint(i + 1);
          debugPrint(" on GPIO ");
          debugPrint(buttonPins[i]);
          debugPrintln(" pressed");

          activeButton = i;
          anyButtonPressed = true;
          buttonPressed = true;
          readyToSleep = false;

// Light up the LED with the color for this button
#if LED_MODE
          strip.setPixelColor(0, buttonColors[i]);
          strip.show();
#endif
        }
      }
    }

    // Keep track if any button is currently pressed
    if (buttonStates[i] == HIGH) {
      anyButtonPressed = true;
    }

    // Save the current reading for the next loop
    lastButtonStates[i] = reading;
  }

  // If button state changed, send ESP-NOW message
  if (buttonStateChanged) {
    sendButtonStates();
  }

  // If no button is currently pressed but one was pressed before, track release time
  if (!anyButtonPressed && buttonPressed) {
    buttonPressed = false;
    buttonReleaseTime = millis();
    readyToSleep = true;

// Turn off the LED when button is released
#if LED_MODE
    strip.setPixelColor(0, strip.Color(0, 0, 0));
    strip.show();
#endif

    // Send final button states before preparing for sleep
    sendButtonStates();

    debugPrintln("All buttons released, preparing for sleep");
  }

  // Send periodic ESP-NOW updates if any button is pressed
  if (anyButtonPressed && (millis() - lastUpdateTime > updateInterval)) {
    sendButtonStates();
    lastUpdateTime = millis();
  }

  // If it's time to sleep and we're ready
  if (readyToSleep && (millis() - buttonReleaseTime > sleepDelay)) {
    debugPrintln("Going to deep sleep now");
#if DEBUG_MODE
    Serial.flush();
#endif

// Make sure LED is off before sleep
#if LED_MODE
    strip.setPixelColor(0, strip.Color(0, 0, 0));
    strip.show();
#endif

    // Enter deep sleep
    esp_deep_sleep_start();
  }

  // Short delay for the loop
  delay(50);
  debugPrint(".");
}