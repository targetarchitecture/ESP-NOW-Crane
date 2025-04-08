#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Wire.h>
#include <LOLIN_I2C_MOTOR.h>
#include <map>
#include <string>
#include <Adafruit_NeoPixel.h>

// Configuration parameters - keeping these as #define for easy adjustments
#define DEBUG_MODE false                        // Enable/disable serial debugging
#define MESSAGE_TIMEOUT 1000                    // Timeout for ESP-NOW messages (ms)
#define CONTROL_MOTOR_INTERVAL 100              // Motor control update interval (ms)
#define MOTOR_INIT_TIMEOUT 3000                 // Timeout for motor initialization (ms)
#define LED_PIN D4                              // Pin for NeoPixel data
#define LED_COUNT 7                             // Number of LEDs
#define CENTER_LED 0                            // Center LED index
#define STEPS_PER_CYCLE 12                      // Steps in animation cycle
#define LED_BRIGHTNESS 100                      // LED brightness (0-255)
#define LED_BLINK_INTERVAL 1000                 // Center LED blink interval (ms)
#define LED_ANIMATION_INTERVAL 60               // Animation update interval (ms)
#define LOOP_DELAY 10                           // Main loop delay (ms)
#define I2C_RETRY_COUNT 5                       // Max retries for I2C operations

// Structure to receive data (updated to match transmitter format)
typedef struct crane_message
{
    uint8_t buttonStates;   // Each bit represents a button state (bits 0-5)
    uint8_t rotationSpeed;  // Speed for clockwise/anticlockwise (0-100%)
    uint8_t verticalSpeed;  // Speed for up/down (0-100%)
    uint8_t extensionSpeed; // Speed for in/out (0-100%)
} crane_message;

// Debug mode flag - enables/disables Serial output for debugging
#define DEBUG_MODE false

// Initialize motor control objects with their I2C addresses
LOLIN_I2C_MOTOR motor1(0x20); // First motor board at I2C address 0x20
LOLIN_I2C_MOTOR motor2(0x21); // Second motor board at I2C address 0x21

// Global flag to track motor initialization
bool motorsConnected = true;

// button states
volatile uint8_t ANTICLOCKWISE = 0;
volatile uint8_t DOWN = 0;
volatile uint8_t OUT = 0;
volatile uint8_t CLOCKWISE = 0;
volatile uint8_t UP = 0;
volatile uint8_t IN = 0;
volatile uint8_t rotationSpeed = 0;  // Speed for clockwise/anticlockwise (0-100%)
volatile uint8_t verticalSpeed = 0;  // Speed for up/down (0-100%)
volatile uint8_t extensionSpeed = 0; // Speed for in/out (0-100%)

// System state tracking
unsigned long lastMessageTime = 0;                // Timestamp of last received ESP-NOW message
unsigned long lastControlMotorTime = 0;           // Timestamp of last controlMotor() execution

// ESP-NOW communication status
bool espNowInitialized = false;

// Debug output macros - only active when DEBUG_MODE is true
#if DEBUG_MODE
#define debugPrintln(message) Serial.println(message)
#define debugPrint(message) Serial.print(message)
#else
#define debugPrintln(message) // Debug messages disabled
#define debugPrint(message)   // Debug messages disabled
#endif

Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

int currentLED = 1;

// Default red colors for the animation
const uint32_t BRIGHT_RED = 0xFF0000;
const uint32_t MID_RED = 0xA00000;
const uint32_t DIM_RED = 0x500000;
const uint32_t VERY_DIM_RED = 0x200000;

unsigned long previousCenterBlinkMillis = 0;
unsigned long previousAnimationMillis = 0;

bool centerLedState = false;

void onDataReceived(uint8_t *mac, uint8_t *data, uint8_t len);
void updateLEDs();
bool anyButtonPressed(uint8_t buttonStates);

// Initialize and verify motor shield connections
void setupMotors()
{
    if (!motorsConnected)
    {
        debugPrintln("Motors not contected, not looking for them");
        return;
    }

    // Wait for first motor board to be ready
    while (motor1.PRODUCT_ID != PRODUCT_ID_I2C_MOTOR)
    {
        motor1.getInfo();
        delay(5);

        // Could add timeout here if needed
    }

    // Wait for second motor board to be ready
    while (motor2.PRODUCT_ID != PRODUCT_ID_I2C_MOTOR)
    {
        motor2.getInfo();
        delay(5);

        // Could add timeout here if needed
    }

    debugPrintln("Motors initialized successfully");
}

// Emergency stop function - stops all motors immediately
void stopAllMotors()
{
    if (!motorsConnected)
    {
        // debugPrintln("Motors not connected, cannot stop");
        return;
    }

    // Stop all motors on both boards
    motor1.changeStatus(MOTOR_CH_BOTH, MOTOR_STATUS_STOP);
    motor2.changeStatus(MOTOR_CH_BOTH, MOTOR_STATUS_STOP);

    // debugPrintln("All motors stopped");
}

// Controls motor movement based on button states
void controlMotor()
{
    if (!motorsConnected)
    {
        // debugPrintln("Motors not initialized, cannot control");
        return;
    }

    debugPrint("ANTICLOCKWISE: ");
    debugPrintln(ANTICLOCKWISE);
    debugPrint("DOWN: ");
    debugPrintln(DOWN);
    debugPrint("OUT: ");
    debugPrintln(OUT);
    debugPrint("CLOCKWISE: ");
    debugPrintln(CLOCKWISE);
    debugPrint("UP: ");
    debugPrintln(UP);
    debugPrint("IN: ");
    debugPrintln(IN);

    // Rotation control (Motor A on first board)
    if (ANTICLOCKWISE == 1 && CLOCKWISE == 0)
    {
        debugPrintln("ANTICLOCKWISE == 1 && CLOCKWISE == 0");

        motor1.changeStatus(MOTOR_CH_A, MOTOR_STATUS_CCW);
        motor1.changeDuty(MOTOR_CH_A, rotationSpeed);
    }
    if (ANTICLOCKWISE == 0 && CLOCKWISE == 1)
    {
        debugPrintln("ANTICLOCKWISE == 0 && CLOCKWISE == 1");

        motor1.changeStatus(MOTOR_CH_A, MOTOR_STATUS_CW);
        motor1.changeDuty(MOTOR_CH_A, rotationSpeed);
    }
    if (ANTICLOCKWISE == 0 && CLOCKWISE == 0)
    {
        debugPrintln("ANTICLOCKWISE == 0 && CLOCKWISE == 0");

        motor1.changeStatus(MOTOR_CH_A, MOTOR_STATUS_STOP);
    }

    // Vertical movement control (Motor B on first board)
    if (UP == 1 && DOWN == 0)
    {
        debugPrintln("UP == 1 && DOWN == 0");

        motor1.changeStatus(MOTOR_CH_B, MOTOR_STATUS_CCW);
        motor1.changeDuty(MOTOR_CH_B, verticalSpeed);
    }
    if (UP == 0 && DOWN == 1)
    {
        debugPrintln("UP == 0 && DOWN == 1");

        motor1.changeStatus(MOTOR_CH_B, MOTOR_STATUS_CW);
        motor1.changeDuty(MOTOR_CH_B, verticalSpeed);
    }
    if (UP == 0 && DOWN == 0)
    {
        debugPrintln("UP == 0 && DOWN == 0");

        motor1.changeStatus(MOTOR_CH_B, MOTOR_STATUS_STOP);
    }

    // Extension control (Motor A on second board)
    if (IN == 1 && OUT == 0)
    {
        debugPrintln("IN == 1 && OUT == 0");

        motor2.changeStatus(MOTOR_CH_B, MOTOR_STATUS_CCW);
        motor2.changeDuty(MOTOR_CH_B, extensionSpeed);
    }
    if (IN == 0 && OUT == 1)
    {
        debugPrintln("IN == 0 && OUT == 1");

        motor2.changeStatus(MOTOR_CH_B, MOTOR_STATUS_CW);
        motor2.changeDuty(MOTOR_CH_B, extensionSpeed);
    }
    if (IN == 0 && OUT == 0)
    {
        debugPrintln("IN == 0 && OUT == 0");

        motor2.changeStatus(MOTOR_CH_B, MOTOR_STATUS_STOP);
    }
}

// Initialize ESP-NOW
bool setupESPNow()
{
    // Set device as a Wi-Fi Station and disconnect from any AP
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100); // Short delay to ensure WiFi mode is set

    // Print MAC address
    Serial.print("ESP Board MAC Address:  ");
    Serial.println(WiFi.macAddress());

    // Init ESP-NOW
    if (esp_now_init() != 0)
    {
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
void onDataReceived(uint8_t *mac, uint8_t *data, uint8_t len)
{
    // Update last message time
    lastMessageTime = millis();

    debugPrintln("ESP-NOW message received");

    // Check if data is valid
    if (len != sizeof(crane_message))
    {
        debugPrintln("Invalid data length");
        return;
    }

    // Cast the received data to the crane_message type
    crane_message *msg = (crane_message *)data;

    ANTICLOCKWISE = (msg->buttonStates & (1 << 0)) ? 1 : 0;
    DOWN = (msg->buttonStates & (1 << 1)) ? 1 : 0;
    OUT = (msg->buttonStates & (1 << 2)) ? 1 : 0;
    CLOCKWISE = (msg->buttonStates & (1 << 3)) ? 1 : 0;
    UP = (msg->buttonStates & (1 << 4)) ? 1 : 0;
    IN = (msg->buttonStates & (1 << 5)) ? 1 : 0;

    // Extract speed values
    rotationSpeed = msg->rotationSpeed;
    verticalSpeed = msg->verticalSpeed;
    extensionSpeed = msg->extensionSpeed;
}

// Check if any button is pressed
bool anyButtonPressed(uint8_t buttonStates)
{
    if (buttonStates == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void updateLEDs()
{
    pixels.clear();

    uint32_t brightColor, midColor, dimColor, veryDimColor;

    // Default red color scheme
    brightColor = BRIGHT_RED;
    midColor = MID_RED;
    dimColor = DIM_RED;
    veryDimColor = VERY_DIM_RED;

    if (centerLedState)
    {
        pixels.setPixelColor(CENTER_LED, midColor);
    }

    int mainLED = ((currentLED - 1) / 2) + 1;

    if (currentLED % 2 == 1)
    {
        pixels.setPixelColor(mainLED, brightColor);

        int prevLED = (mainLED == 1) ? 6 : mainLED - 1;
        int prevPrevLED = (prevLED == 1) ? 6 : prevLED - 1;

        pixels.setPixelColor(prevLED, midColor);
        pixels.setPixelColor(prevPrevLED, dimColor);

        int prevPrevPrevLED = (prevPrevLED == 1) ? 6 : prevPrevLED - 1;
        pixels.setPixelColor(prevPrevPrevLED, veryDimColor);
    }
    else
    {
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
void setup()
{
#if DEBUG_MODE
    Serial.begin(115200); // Initialize serial communication for debugging
    delay(100);           // Wait for serial to stabilize
#endif

    debugPrintln("ESP8266 Crane Motor Controller with ESP-NOW");

    // Initialize I2C communication for motor control
    Wire.begin();

    // Initialize motor shields
    setupMotors();

    // testMotors();

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
void loop()
{
    // Check for message timeout
    if (millis() - lastMessageTime > MESSAGE_TIMEOUT)
    {
        // Stop all motors
        stopAllMotors();
    }

    // Execute motor control based on updated button states
    if (millis() - lastControlMotorTime >= CONTROL_MOTOR_INTERVAL)
    {
        lastControlMotorTime = millis();
        controlMotor();
    }

    // If ESP-NOW is not initialized, try to set it up
    if (!espNowInitialized)
    {
        espNowInitialized = setupESPNow();
    }

    unsigned long currentMillis = millis();
    bool updateDisplay = false;

    if (currentMillis - previousCenterBlinkMillis >= LED_BLINK_INTERVAL)
    {
        previousCenterBlinkMillis = currentMillis;
        centerLedState = !centerLedState;
        updateDisplay = true;
    }

    if (currentMillis - previousAnimationMillis >= LED_ANIMATION_INTERVAL)
    {
        previousAnimationMillis = currentMillis;
        currentLED = (currentLED % STEPS_PER_CYCLE) + 1;
        updateDisplay = true;
    }

    if (updateDisplay)
    {
        updateLEDs();
    }

    // Small delay to prevent overwhelming the system
    delay(50);
}