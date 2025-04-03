#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <esp_sleep.h>
#include <esp_now.h>
#include <WiFi.h>

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
  0xFF9800, // Vivid Orange (Button 1 - ANTICLOCKWISE)  Y
  0xD32F2F, // Deep Red (Button 2 - DOWN)              Y
  0x2196F3, // Bright Blue (Button 3 - OUT)
  0x4CAF50, // Green (Button 4 - CLOCKWISE)            Y
  0x9C27B0, // Purple (Button 5 - UP)                 Y
  0x00BCD4  // Cyan/Light Blue (Button 6 - IN)
};

// Variables to keep track of button states
int buttonStates[6] = {LOW, LOW, LOW, LOW, LOW, LOW};
int lastButtonStates[6] = {LOW, LOW, LOW, LOW, LOW, LOW};

unsigned long lastDebounceTime[6] = {0, 0, 0, 0, 0, 0};
const unsigned long debounceDelay = 50; // Debounce time in milliseconds

// ESP-NOW periodic update variables
unsigned long lastEspNowUpdateTime = 0;
const unsigned long espNowUpdateInterval = 500; // Send ESP-NOW update every 500ms (half second)

// Sleep parameters
const unsigned long sleepDelay = 3000; // Time in ms before going to sleep after button release
unsigned long buttonReleaseTime = 0;    // Time when the last button was released
bool buttonPressed = false;            // Flag to track if any button is pressed
bool readyToSleep = true;              // Flag to indicate ready to enter sleep

// ESP-NOW peer address (replace with your receiver's MAC address)
uint8_t broadcastAddress[] = {0x3C, 0x71, 0xBF, 0x31, 0x5E, 0xF5};

// Initialize the NeoPixel library
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// --- Debugging Macros ---
#if DEBUG_MODE
#define debugPrintln(message) Serial.println(message)
#define debugPrint(message) Serial.print(message)
#else
#define debugPrintln(message) // do nothing
#define debugPrint(message)   // do nothing
#endif

// Structure to send data
typedef struct struct_message {
  int ANTICLOCKWISE;
  int DOWN;
  int OUT;
  int CLOCKWISE;
  int UP;
  int IN;
} struct_message;

struct_message myData;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t sendStatus) {
#if DEBUG_MODE
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(sendStatus == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
#endif
}

// Function to send button states via ESP-NOW
void sendButtonStates() {
  myData.ANTICLOCKWISE = buttonStates[0];
  myData.DOWN = buttonStates[1];
  myData.OUT = buttonStates[2];
  myData.CLOCKWISE = buttonStates[3];
  myData.UP = buttonStates[4];
  myData.IN = buttonStates[5];

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

#if DEBUG_MODE
  if (result == ESP_OK) {
    Serial.println("ESP-NOW message sent");
    Serial.print("ANTICLOCKWISE: "); Serial.print(myData.ANTICLOCKWISE); Serial.print(", ");
    Serial.print("DOWN: "); Serial.print(myData.DOWN); Serial.print(", ");
    Serial.print("OUT: "); Serial.print(myData.OUT); Serial.print(", ");
    Serial.print("CLOCKWISE: "); Serial.print(myData.CLOCKWISE); Serial.print(", ");
    Serial.print("UP: "); Serial.print(myData.UP); Serial.print(", ");
    Serial.print("IN: "); Serial.println(myData.IN);
  } else {
    Serial.println("Error sending ESP-NOW message");
  }
#endif
}

// Get wake cause and button that triggered wake up
void printWakeupReason() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT1: {
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
  delay(100); // Short delay to ensure serial is ready
#endif

  debugPrintln("ESP32-S3 WS2812 LED Control with Deep Sleep and ESP-NOW");
  delay(10);

  // Initialize the LED strip
#if LED_MODE
  strip.begin();
  strip.setBrightness(50); // Set brightness (0-255)
  strip.show();             // Initialize all pixels to 'off'

  // Set initial LED color to black/off
  strip.setPixelColor(0, strip.Color(0, 0, 0)); // Explicitly set to black (R=0, G=0, B=0)
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

  // Initialize ESP-NOW
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
#if DEBUG_MODE
    Serial.println("Error initializing ESP-NOW");
#endif
    return;
  }

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
#if DEBUG_MODE
    Serial.println("Failed to add peer");
#endif
    return;
  }

  // Register for send callback
  esp_now_register_send_cb(OnDataSent);

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
  if (anyButtonPressed && (millis() - lastEspNowUpdateTime > espNowUpdateInterval)) {
    sendButtonStates();
    lastEspNowUpdateTime = millis();
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