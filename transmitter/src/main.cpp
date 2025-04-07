#include <Adafruit_NeoPixel.h>
#include "esp_sleep.h"
#include <WiFi.h>
#include <esp_now.h>

// ===== CONFIGURATION =====
// Set to true to enable Serial debugging output
#define DEBUG_MODE false

// Set to true to enable WS2812 LED feedback
#define LED_MODE true

// Set to true to enable deep sleep mode for power saving
#define SLEEP_MODE true

// Hardware configuration
#define LED_PIN 21
#define NUM_LEDS 1

// Timing configurations (all in milliseconds)
#define DEBOUNCE_DELAY 50   // Button debounce time
#define UPDATE_INTERVAL 250 // ESP-NOW update interval when button is pressed
#define SLEEP_DELAY 5000    // Time before going to sleep after button release

// ESP-NOW receiver MAC address
const uint8_t RECEIVER_MAC[] = {0x3C, 0x71, 0xBF, 0x31, 0x5E, 0xF5};
//const uint8_t RECEIVER_MAC[] = {0xB4, 0xE6, 0x2D, 0x53, 0xAF, 0x58};

// ===== BUTTON CONFIGURATION =====
// Button metadata structure for better organization
typedef struct
{
  int pin;                        // GPIO pin number
  uint32_t color;                 // RGB color for this button
  const char *name;               // Movement name for debugging
  bool state;                     // Current state (pressed or not)
  bool lastState;                 // Previous state for change detection
  unsigned long lastDebounceTime; // Last time the button state changed
} ButtonConfig;

// Button definitions
ButtonConfig buttons[] = {
    {1, 0xFF9800, "ANTICLOCKWISE", false, false, 0}, // Button 1
    {2, 0xD32F2F, "DOWN", false, false, 0},          // Button 2
    {3, 0x2196F3, "OUT", false, false, 0},           // Button 3
    {4, 0x4CAF50, "CLOCKWISE", false, false, 0},     // Button 4
    {5, 0x9C27B0, "UP", false, false, 0},            // Button 5
    {6, 0x00BCD4, "IN", false, false, 0}             // Button 6
};

const int NUM_BUTTONS = sizeof(buttons) / sizeof(buttons[0]);

// ===== ESP-NOW MESSAGE STRUCTURE =====
typedef struct crane_message
{
  uint8_t buttonStates; // Each bit represents a button state (bits 0-5)
} crane_message;

// Global variables
crane_message craneMsg;
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
unsigned long lastUpdateTime = 0;
unsigned long buttonReleaseTime = 0;
bool anyButtonPressed = false;
bool readyToSleep = true;

// ===== DEBUGGING MACROS =====
#if DEBUG_MODE
#define DEBUG_BEGIN(baud) Serial.begin(baud)
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINTF(fmt, ...) Serial.printf(fmt, __VA_ARGS__)
#define DEBUG_FLUSH() Serial.flush()
#else
#define DEBUG_BEGIN(baud)
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTF(fmt, ...)
#define DEBUG_FLUSH()
#endif

// ===== ESP-NOW FUNCTIONS =====

// Callback when data is sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  DEBUG_PRINT("Send status: ");
  DEBUG_PRINTLN(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}

// Initialize ESP-NOW with multiple retry attempts
bool initESPNow()
{
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    DEBUG_PRINTLN("ESP-NOW init failed");
    return false;
  }

  // Register callback
  esp_now_register_send_cb(onDataSent);

  // Register peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, RECEIVER_MAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    DEBUG_PRINTLN("Failed to add peer");
    return false;
  }

  DEBUG_PRINTLN("ESP-NOW initialized successfully");
  return true;
}

// Pack button states and send via ESP-NOW
bool sendButtonStates()
{
  // Pack button states into a single byte
  craneMsg.buttonStates = 0;
  for (int i = 0; i < NUM_BUTTONS; i++)
  {
    if (buttons[i].state)
    {
      craneMsg.buttonStates |= (1 << i);
    }
  }

  // Ensure WiFi is in the correct mode
  if (WiFi.getMode() != WIFI_STA)
  {
    DEBUG_PRINTLN("Fixing WiFi mode...");
    WiFi.mode(WIFI_STA);
    delay(10);
  }

  // Send message
  esp_err_t result = esp_now_send(RECEIVER_MAC, (uint8_t *)&craneMsg, sizeof(craneMsg));

  if (result != ESP_OK)
  {
    DEBUG_PRINT("ESP-NOW error: ");

    // Declare peerInfo outside the switch to avoid compiler errors
    esp_now_peer_info_t peerInfo = {};

    switch (result)
    {
    case ESP_ERR_ESPNOW_NOT_INIT:
      DEBUG_PRINTLN("Not initialized, reinitializing...");
      esp_now_deinit();
      delay(10);
      initESPNow();
      break;
    case ESP_ERR_ESPNOW_NOT_FOUND:
      DEBUG_PRINTLN("Peer not found, re-adding...");
      memcpy(peerInfo.peer_addr, RECEIVER_MAC, 6);
      peerInfo.channel = 0;
      peerInfo.encrypt = false;
      esp_now_add_peer(&peerInfo);
      break;
    default:
      DEBUG_PRINTF("Error code: %d\n", result);
    }

    return false;
  }

// Debug output for button states
#if DEBUG_MODE
  DEBUG_PRINTLN("\nButton states:");
  for (int i = 0; i < NUM_BUTTONS; i++)
  {
    DEBUG_PRINTF("  %s: %d\n", buttons[i].name, buttons[i].state ? 1 : 0);
  }
#endif

  return true;
}

// ===== LED CONTROL FUNCTIONS =====

// Initialize the LED
void setupLED()
{
#if LED_MODE
  strip.begin();
  strip.setBrightness(50);
  strip.setPixelColor(0, 0); // Off
  strip.show();
  DEBUG_PRINTLN("LED initialized");
#endif
}

// Set LED color based on active button
void updateLED(int activeButton)
{
#if LED_MODE
  if (activeButton >= 0 && activeButton < NUM_BUTTONS)
  {
    strip.setPixelColor(0, buttons[activeButton].color);
  }
  else
  {
    strip.setPixelColor(0, 0); // Off
  }
  strip.show();
#endif
}

// ===== POWER MANAGEMENT FUNCTIONS =====

// Configure deep sleep wake sources
void setupSleep()
{
#if SLEEP_MODE
  // Create wake-up mask for all button pins
  uint64_t mask = 0;
  for (int i = 0; i < NUM_BUTTONS; i++)
  {
    mask |= (1ULL << buttons[i].pin);
  }

  // Configure EXT1 wake sources (any high level will wake up)
  esp_sleep_enable_ext1_wakeup(mask, ESP_EXT1_WAKEUP_ANY_HIGH);
  DEBUG_PRINTLN("Sleep mode configured");
#endif
}

// Enter deep sleep mode
void goToSleep()
{
#if SLEEP_MODE
  DEBUG_PRINTLN("Entering deep sleep");
  DEBUG_FLUSH();

  // Turn off LED before sleep
  updateLED(-1);

  // Enter deep sleep
  esp_deep_sleep_start();
#else
  DEBUG_PRINTLN("Sleep mode disabled");
#endif
}

// Identify what caused the ESP32 to wake up
void checkWakeupCause()
{
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_EXT1:
  {
    uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
    DEBUG_PRINTLN("Wakeup caused by button press");

    // Identify which button woke up the device
    for (int i = 0; i < NUM_BUTTONS; i++)
    {
      if (wakeup_pin_mask & (1ULL << buttons[i].pin))
      {
        DEBUG_PRINTF("  Wake button: %s (GPIO %d)\n", buttons[i].name, buttons[i].pin);
        updateLED(i);
        break;
      }
    }
    break;
  }
  case ESP_SLEEP_WAKEUP_TIMER:
    DEBUG_PRINTLN("Wakeup caused by timer");
    break;
  default:
    DEBUG_PRINTF("Wakeup not caused by deep sleep: %d\n", wakeup_reason);
    break;
  }
}

// ===== BUTTON HANDLING FUNCTIONS =====

// Initialize button pins
void setupButtons()
{
  for (int i = 0; i < NUM_BUTTONS; i++)
  {
    pinMode(buttons[i].pin, INPUT);
    DEBUG_PRINTF("Button %s on GPIO %d initialized\n", buttons[i].name, buttons[i].pin);
  }
}

// Read and process button states with debouncing
void handleButtons()
{
  bool stateChanged = false;
  int activeButton = -1;
  anyButtonPressed = false;

  // Check each button
  for (int i = 0; i < NUM_BUTTONS; i++)
  {
    // Read current state
    int reading = digitalRead(buttons[i].pin);

    // If state changed, reset debounce timer
    if (reading != buttons[i].lastState)
    {
      buttons[i].lastDebounceTime = millis();
    }

    // If debounce period passed, update actual state
    if ((millis() - buttons[i].lastDebounceTime) > DEBOUNCE_DELAY)
    {
      // If button state has changed
      if (reading != buttons[i].state)
      {
        buttons[i].state = reading;
        stateChanged = true;

        if (buttons[i].state)
        { // Button pressed
          DEBUG_PRINTF("Button %s pressed\n", buttons[i].name);
          activeButton = i;
          anyButtonPressed = true;
          readyToSleep = false;
        }
      }
    }

    // Track if any button is currently pressed
    if (buttons[i].state)
    {
      anyButtonPressed = true;
    }

    // Save current reading for next iteration
    buttons[i].lastState = reading;
  }

  // Handle button state change
  if (stateChanged)
  {
    updateLED(activeButton);
    sendButtonStates();
  }

  // Handle button release
  static bool prevAnyButtonPressed = false;
  if (!anyButtonPressed && prevAnyButtonPressed)
  {
    buttonReleaseTime = millis();
    readyToSleep = true;
    updateLED(-1);      // Turn off LED
    sendButtonStates(); // Send final button states before sleep
    DEBUG_PRINTLN("All buttons released");
  }
  prevAnyButtonPressed = anyButtonPressed;

  // Send periodic updates when buttons are pressed
  if (anyButtonPressed && (millis() - lastUpdateTime > UPDATE_INTERVAL))
  {
    sendButtonStates();
    lastUpdateTime = millis();
  }

  // Check if it's time to sleep
  if (readyToSleep && (millis() - buttonReleaseTime > SLEEP_DELAY))
  {
    goToSleep();
  }
}

// ===== MAIN FUNCTIONS =====

void setup()
{
  // Initialize serial for debugging
  DEBUG_BEGIN(115200);
#if DEBUG_MODE
  delay(50); // Much shorter delay for Serial initialization
#endif
  DEBUG_PRINTLN("\nESP32 Button Controller Starting");

  // Initialize components
  setupLED();
  setupButtons();
  setupSleep();

  // Check wake reason
  checkWakeupCause();

  // Initialize ESP-NOW with retries
  bool espNowInitialized = false;
  for (int retry = 0; retry < 3 && !espNowInitialized; retry++)
  {
    DEBUG_PRINTF("ESP-NOW init attempt %d/3\n", retry + 1);
    espNowInitialized = initESPNow();
    if (!espNowInitialized)
    {
      delay(500);
    }
  }

  // Send initial message
  if (espNowInitialized)
  {
    craneMsg.buttonStates = 0; // All buttons released
    sendButtonStates();
  }
  else
  {
    DEBUG_PRINTLN("WARNING: ESP-NOW initialization failed!");
  }

  DEBUG_PRINTLN("Setup complete");
}

void loop()
{
  // Handle button presses and releases
  handleButtons();

  // Small delay to prevent busy-waiting
  delay(20); // Reduced from 100ms for more responsive feel
}