#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Wire.h>
#include <LOLIN_I2C_MOTOR.h>
#include <map>
#include <string>
#include <Adafruit_NeoPixel.h>

// Structure to receive data (updated to match transmitter format)
typedef struct crane_message {
    uint8_t buttonStates;  // Bitwise button states
} crane_message;

// Map to store button states and their corresponding values
// 0 = inactive/off, 1 = active/on
std::map<std::string, int> buttons = {
    { "ANTICLOCKWISE", 0 },  // Controls rotation counter-clockwise
    { "DOWN", 0 },           // Controls downward movement
    { "OUT", 0 },            // Controls extension outward
    { "CLOCKWISE", 0 },      // Controls rotation clockwise
    { "UP", 0 },             // Controls upward movement
    { "IN", 0 }              // Controls retraction inward
};

// Debug mode flag - enables/disables Serial output for debugging
#define DEBUG_MODE true

// Configuration structure to store all system settings
struct Config {
    // Motor Control Configuration
    uint8_t motor_speed = 100;    // Default motor speed (0-100%)

    // WiFi Network Configuration
    char wifi_ssid[32] = "the robot network"; // WiFi network name
    char wifi_password[32] = "isaacasimov";   // WiFi network password

    // System Timing Configuration
    int wifi_timeout = 20;      // WiFi connection timeout (seconds)
} config;

// Initialize motor control objects with their I2C addresses
LOLIN_I2C_MOTOR motor1(0x20);  // First motor board at I2C address 0x20
LOLIN_I2C_MOTOR motor2(0x21);  // Second motor board at I2C address 0x21

// System state tracking
unsigned long lastMessageTime = 0;    // Timestamp of last received ESP-NOW message
const unsigned long MESSAGE_TIMEOUT = 1000;    // Timeout period in milliseconds (1 second)
const unsigned long COLOR_CHANGE_DURATION = 3000; // Duration to show the message color before returning to animation

// ESP-NOW communication status
bool espNowInitialized = false;

// Debug output macros - only active when DEBUG_MODE is true
#if DEBUG_MODE
#define debugPrintln(message) Serial.println(message)
#define debugPrint(message) Serial.print(message)
#else
#define debugPrintln(message)  // Debug messages disabled
#define debugPrint(message)   // Debug messages disabled
#endif

// NeoPixel LED configuration
#define LED_PIN D4 // GPIO pin connected to the NeoPixel data line.
#define LED_COUNT 7 // Number of LEDs in your NeoPixel strip.
#define CENTER_LED 0
#define STEPS_PER_CYCLE 12

Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

int currentLED = 1;

// Default red colors for the animation
const uint32_t BRIGHT_RED = 0xFF0000;
const uint32_t MID_RED = 0xA00000;
const uint32_t DIM_RED = 0x500000;
const uint32_t VERY_DIM_RED = 0x200000;

// Colors for message indication
const uint32_t BRIGHT_GREEN = 0x00FF00;
const uint32_t MID_GREEN = 0x00A000;
const uint32_t DIM_GREEN = 0x005000;
const uint32_t VERY_DIM_GREEN = 0x002000;

// Current color scheme (0 = red, 1 = green)
int currentColorScheme = 0;

unsigned long previousCenterBlinkMillis = 0;
unsigned long previousAnimationMillis = 0;
unsigned long colorChangeStartTime = 0;

const long centerBlinkInterval = 1000;
const long animationInterval = 60;

bool centerLedState = false;
bool animationRunning = true;
bool messageReceived = false;

void onDataReceived(uint8_t *mac, uint8_t *data, uint8_t len);
void updateLEDs();
bool anyButtonPressed();

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
    Serial.print("ESP Board MAC Address:  ");
    Serial.println(WiFi.macAddress());

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

// Callback function for ESP-NOW data reception (updated for new message format)
void onDataReceived(uint8_t *mac, uint8_t *data, uint8_t len) {
    // Update last message time
    lastMessageTime = millis();

    debugPrintln("ESP-NOW message received");

    // Check if data is valid
    if (len != sizeof(crane_message)) {
        debugPrintln("Invalid data length");
        return;
    }

    // Cast the received data to the crane_message type
    crane_message* msg = (crane_message*)data;
    
    // Extract button states from bitwise format
    buttons["ANTICLOCKWISE"] = (msg->buttonStates & (1 << 0)) ? 1 : 0;
    buttons["DOWN"] = (msg->buttonStates & (1 << 1)) ? 1 : 0;
    buttons["OUT"] = (msg->buttonStates & (1 << 2)) ? 1 : 0;
    buttons["CLOCKWISE"] = (msg->buttonStates & (1 << 3)) ? 1 : 0;
    buttons["UP"] = (msg->buttonStates & (1 << 4)) ? 1 : 0;
    buttons["IN"] = (msg->buttonStates & (1 << 5)) ? 1 : 0;
    
    // Debug button states
    #if DEBUG_MODE
    debugPrint("Button states: ");
    debugPrint("ANTICLOCKWISE: "); debugPrint(buttons["ANTICLOCKWISE"]);
    debugPrint(", DOWN: "); debugPrint(buttons["DOWN"]);
    debugPrint(", OUT: "); debugPrint(buttons["OUT"]);
    debugPrint(", CLOCKWISE: "); debugPrint(buttons["CLOCKWISE"]);
    debugPrint(", UP: "); debugPrint(buttons["UP"]);
    debugPrint(", IN: "); debugPrint(buttons["IN"]);
    debugPrintln("");
    #endif

    // Change LED color to green when message is received
    if (anyButtonPressed()) {
        currentColorScheme = 1; // Change to green color scheme
        colorChangeStartTime = millis(); // Start the timer for color display
        messageReceived = true;
        updateLEDs(); // Update LEDs immediately
    }

    // Execute motor control based on updated button states
    controlMotor();
}

// Check if any button is pressed
bool anyButtonPressed() {
    return (buttons["ANTICLOCKWISE"] == 1 || 
            buttons["DOWN"] == 1 || 
            buttons["OUT"] == 1 || 
            buttons["CLOCKWISE"] == 1 || 
            buttons["UP"] == 1 || 
            buttons["IN"] == 1);
}

void updateLEDs() {
    pixels.clear();
    
    uint32_t brightColor, midColor, dimColor, veryDimColor;
    
    // Select color scheme based on state
    if (currentColorScheme == 0) {
        // Default red color scheme
        brightColor = BRIGHT_RED;
        midColor = MID_RED;
        dimColor = DIM_RED;
        veryDimColor = VERY_DIM_RED;
    } else {
        // Green color scheme for message indication
        brightColor = BRIGHT_GREEN;
        midColor = MID_GREEN;
        dimColor = DIM_GREEN;
        veryDimColor = VERY_DIM_GREEN;
    }
    
    if (centerLedState) {
        pixels.setPixelColor(CENTER_LED, midColor);
    }

    int mainLED = ((currentLED - 1) / 2) + 1;

    if (currentLED % 2 == 1) {
        pixels.setPixelColor(mainLED, brightColor);

        int prevLED = (mainLED == 1) ? 6 : mainLED - 1;
        int prevPrevLED = (prevLED == 1) ? 6 : prevLED - 1;

        pixels.setPixelColor(prevLED, midColor);
        pixels.setPixelColor(prevPrevLED, dimColor);

        int prevPrevPrevLED = (prevPrevLED == 1) ? 6 : prevPrevLED - 1;
        pixels.setPixelColor(prevPrevPrevLED, veryDimColor);
    } else {
        int nextLED = (mainLED == 6) ? 1 : mainLED + 1;

        pixels.setPixelColor(mainLED, brightColor);
        pixels.setPixelColor(nextLED, midColor);

        int prevLED = (mainLED == 1) ? 6 : mainLED - 1;
        int prevPrevLED = (prevLED == 1) ? 6 : prevLED - 1;

        pixels.setPixelColor(prevLED, dimColor);
        pixels.setPixelColor(prevPrevLED, veryDimColor);
    }
    pixels.show();
}

// System initialization
void setup() {
#if DEBUG_MODE
    Serial.begin(115200);  // Initialize serial communication for debugging
    delay(100);          // Wait for serial to stabilize
#endif

    debugPrintln("ESP8266 Crane Motor Controller with ESP-NOW");

    // Initialize I2C communication for motor control
    Wire.begin();

    // Initialize motor shields
    //setupMotors();

    // Initialize last message time
    lastMessageTime = millis();

    // Setup ESP-NOW after WiFi is configured
    espNowInitialized = setupESPNow();

    // Initialize NeoPixel
    pixels.begin();
    pixels.setBrightness(100);
    pixels.clear();
    pixels.show();

    debugPrintln("Setup complete");
}

// Main program loop
void loop() {
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
        //stopAllMotors();
    }

    // If ESP-NOW is not initialized, try to set it up
    if (!espNowInitialized) {
        espNowInitialized = setupESPNow();
    }

    unsigned long currentMillis = millis();
    bool updateDisplay = false;

    // Check if we need to revert color scheme
    if (messageReceived && currentMillis - colorChangeStartTime >= COLOR_CHANGE_DURATION) {
        currentColorScheme = 0; // Switch back to red color scheme
        messageReceived = false;
        updateDisplay = true;
    }

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

    // Small delay to prevent overwhelming the system
    delay(10);
}