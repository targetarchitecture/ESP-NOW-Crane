/*

*/
#include <ESP8266WiFi.h>
extern "C" {
#include <espnow.h>
}

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;         // will store last time LED was updated
//unsigned long previousFailSafeMillis = 0; // will store last time failsafe command was updated

// constants won't change:
const long interval = 1000;           // interval at which to blink (milliseconds)
const long failSafeInterval = 500;    // interval at which fail safe is issued (milliseconds)

//declare pins
#define LEDA D0 // LED
#define LEDB D1 // LED

//https://www.bananarobotics.com/shop/HG7881-(L9110)-Dual-Channel-Motor-Driver-Module
#define MotorInOutSpeed  D7 // In & Out
#define MotorInOutDirection  D6 // In & Out
#define MotorUpDownSpeed  D5   // Up & Down
#define MotorUpDownDirection  D4 // Up & Down
#define MotorLeftRightSpeed  D3 // Left & Right
#define MotorLeftRightDirection  D2 // Left & Right

#define WIFI_CHANNEL 13
#define SEND_TIMEOUT 245  // 245 millis seconds timeout 

// keep in sync with transmitter struct
struct __attribute__((packed)) TRANSMITTER {
  int boom; //1 in , 2 out, 0 stop
  int rotation; //1 left, 2 right, 0 stop
  int hook; //1 up , 2 down, 0 stop
  unsigned long timeSent;
} transmitter;

//motion variables to be executed per loop
int boom = 0;
int rotation = 0;
int hook = 0;
unsigned long lastMessageRecievedTime = 0;

void setup() {

  Serial.begin(115200);

  Serial.println();
  Serial.println();
  Serial.println("ESP_Now Crane Reciever v5");
  Serial.println();

  WiFi.mode(WIFI_STA); // Station mode for esp-now sensor node
  WiFi.disconnect();

  Serial.printf("This mac: %s, ", WiFi.macAddress().c_str());
  Serial.printf(", channel: %i\n", WIFI_CHANNEL);

  if (esp_now_init() != 0) {
    Serial.println("*** ESP_Now init failed");
    ESP.restart();
  }

  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);

  //esp_now_register_send_cb is a callback function that is called when a message was sent to show it's status
  esp_now_register_recv_cb(receiveCallBackFunction);

  //configure pins to drive LEDs
  pinMode(LEDA, OUTPUT);
  pinMode(LEDB, OUTPUT);

  digitalWrite(LEDA, HIGH); // LED on
  digitalWrite(LEDB, LOW); // LED off

  // Configure digital pins for motor
  pinMode(MotorLeftRightDirection, OUTPUT);
  pinMode(MotorLeftRightSpeed, OUTPUT);
  pinMode(MotorInOutSpeed, OUTPUT);
  pinMode(MotorInOutDirection, OUTPUT);
  pinMode(MotorUpDownSpeed, OUTPUT);
  pinMode(MotorUpDownDirection, OUTPUT);

  //just in case
  //stopMotors();
}

void loop() {
  ledFlasher();

  //stop hammer time
  delay(50);

  //https://community.blynk.cc/t/prevent-crashes-and-resets-heartbeat-and-watchdog-timers/25708
  ESP.wdtFeed();

  //fail safe is needed to stop the motors every 500ms
  //failSafe();

  
  Serial.printf("MsgLen %i, Boom: %i, Hook: %i, Rotation: %i, TimeRecieved: %i\n", len, boom, hook, rotation, lastMessageRecievedTime);


  if (boom == 1) {
    boomIn();
  } else if (boom == 2) {
    boomOut();
  }  else {
    boomStop();
  }

  if (rotation == 1) {
    rotateLeft();
  } else if (rotation == 2) {
    rotateRight();
  } else {
    rotationStop();
  }

  if (hook == 1) {
    hookUp();
  } else if (hook == 2) {
    hookDown();
  }  else {
    hookStop();
  }

      //reset the flags
    //boom = 0;
   // rotation = 0;
   // hook = 0;
}

void failSafe() {
  // issue an ALL stop every 500 milliseconds
  unsigned long currentMillis = millis();

  if (currentMillis - lastMessageRecievedTime >= failSafeInterval) {

    Serial.println("FAIL SAFE");

    //stop motors
    stopMotors();

    //belt and braces
    boom = 0;
    rotation = 0;
    hook = 0;
  }
}

void ledFlasher() {
  // check to see if it's time to blink the LED; that is, if the difference
  // between the current time and last time you blinked the LED is bigger than
  // the interval at which you want to blink the LED.
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    digitalWrite(LEDA, !digitalRead(LEDA));
    digitalWrite(LEDB, !digitalRead(LEDB));
  }
}

void hookDown() {
  digitalWrite(MotorUpDownDirection, LOW);   // direction
  digitalWrite(MotorUpDownSpeed, HIGH);
  Serial.println("HOOK DOWN");
}

void hookUp() {
  digitalWrite(MotorUpDownDirection, HIGH);   // direction
  digitalWrite(MotorUpDownSpeed, LOW);
  Serial.println("HOOK UP");
}

void rotateRight() {
  digitalWrite(MotorLeftRightDirection, LOW);   // direction
  digitalWrite(MotorLeftRightSpeed, HIGH);
  Serial.println("ROTATE RIGHT");
}

void  rotateLeft() {
  digitalWrite(MotorLeftRightDirection, HIGH);   // direction
  digitalWrite(MotorLeftRightSpeed, LOW);
  Serial.println("ROTATE LEFT");
}

void boomIn() {
  digitalWrite(MotorInOutDirection, HIGH);   // direction
  digitalWrite(MotorInOutSpeed, LOW);
  Serial.println("BOOM IN");
}

void boomOut() {
  digitalWrite(MotorInOutDirection, LOW);   // direction
  digitalWrite(MotorInOutSpeed, HIGH);
  Serial.println("BOOM OUT");
}

void stopMotors() {
  //always stop motors before moving abruptly
  digitalWrite( MotorLeftRightDirection, LOW );
  digitalWrite( MotorLeftRightSpeed, LOW );
  digitalWrite( MotorUpDownSpeed, LOW );
  digitalWrite( MotorUpDownDirection, LOW );
  digitalWrite( MotorInOutSpeed, LOW );
  digitalWrite( MotorInOutDirection, LOW );

  Serial.println("STOP");
}

void hookStop() {
  digitalWrite( MotorUpDownSpeed, LOW );
  digitalWrite( MotorUpDownDirection, LOW );
  //Serial.println("HOOK STOP");
}

void boomStop() {
  digitalWrite( MotorInOutSpeed, LOW );
  digitalWrite( MotorInOutDirection, LOW );
  //Serial.println("BOOM STOP");
}

void rotationStop() {
  digitalWrite( MotorLeftRightDirection, LOW );
  digitalWrite( MotorLeftRightSpeed, LOW );
  //Serial.println("ROTATION STOP");
}

void receiveCallBackFunction(uint8_t *senderMac, uint8_t *incomingData, uint8_t len) {

  memcpy(&transmitter, incomingData, sizeof(transmitter));

  boom = transmitter.boom;
  hook = transmitter.hook;
  rotation = transmitter.rotation;
  lastMessageRecievedTime = millis();
}
