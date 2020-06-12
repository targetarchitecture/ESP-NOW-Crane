/*
   Based on Andreas Spiess https://www.youtube.com/watch?v=6NsBN42B80Q
*/

#include <ESP8266WiFi.h>
extern "C" {
#include <espnow.h>
}

/*
  COLOURS:

  GREEN =  PIN 5  (D1) - RED UP, BOOM OUT
  BROWN =  PIN 4  (D2) - RED DOWN , BOOM IN
  PURPLE = PIN 14 (D5) - YELLOW RIGHT, ROTATE RIGHT
  WHITE = PIN 12 (D6) - BLUE RIGHT, HOOK UP
  ORANGE = PIN 13 (D7) - BLUE LEFT, HOOK DOWN
  YELLOW =  PIN 15 (D8) - YELLOW LEFT, ROTATE LEFT

  GPIO 4, 5, 12-14, 16: nothing special.
*/

// this is the MAC Address of the crane's wemos board
//WEMOS D1 R2 MAC ADDRESS (B4:E6:2D:53:AF:58)
uint8_t remoteMac[] = {0xB4, 0xE6, 0x2D, 0x53, 0xAF, 0x58};

#define WIFI_CHANNEL 13
#define SEND_TIMEOUT 245  // 245 millis seconds timeout 

// keep in sync with transmitter struct
struct __attribute__((packed)) TRANSMITTER {
  int boom; //1 in , 2 out, 0 stop
  int rotation; //1 left, 2 right, 0 stop
  int hook; //1 up , 2 down, 0 stop
  unsigned long timeSent;
} transmitter;

void setup() {

  unsigned long startTime = millis();

  Serial.begin(115200);

  Serial.println();
  Serial.println();
  Serial.println("ESP_Now Crane Transmitter");
  Serial.println();

  WiFi.mode(WIFI_STA); // Station mode for esp-now sensor node

  //  Enable light sleep
  //wifi_set_sleep_type(LIGHT_SLEEP_T);

  WiFi.disconnect();



  Serial.printf("This mac: %s, ", WiFi.macAddress().c_str());
  Serial.printf("target mac: %02x%02x%02x%02x%02x%02x", remoteMac[0], remoteMac[1], remoteMac[2], remoteMac[3], remoteMac[4], remoteMac[5]);
  Serial.printf(", channel: %i\n", WIFI_CHANNEL);

  if (esp_now_init() != 0) {
    Serial.println("*** ESP_Now init failed");
    ESP.restart();
  }

  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);

  esp_now_add_peer(remoteMac, ESP_NOW_ROLE_SLAVE, WIFI_CHANNEL, NULL, 0);

  //esp_now_register_send_cb is a callback function that is called when a message was sent to show it's status
  esp_now_register_send_cb([](uint8_t* mac, uint8_t sendStatus) {
    Serial.printf("send_cb, send done @ %i, status = %i\n", millis(), sendStatus);
  });

  //buttons
  pinMode(D1, INPUT);
  pinMode(D2, INPUT);
  pinMode(D5, INPUT);
  pinMode(D6, INPUT);
  pinMode(D7, INPUT);
  pinMode(D8, INPUT);

  Serial.print("Time to complete setup: ");
  Serial.println(millis() - startTime);
}

unsigned long stopMessageCount = 0;

void loop() {

  int boom = boomMessage();
  int hook = hookMessage();
  int rotation = rotationMessage();

  if (boom == 0 && hook == 0 && rotation == 0) {

    stopMessageCount = stopMessageCount + 1;

    if (stopMessageCount > 50) {
      //Do nothing as we'll have sent over 50 stop messages
    } else {
      //send the stop message
      sendMessage(boom, rotation, hook);
    }
  } else {
    //always send a message if it's not a stop message
    sendMessage(boom, rotation, hook);

    //reset stop message count
    stopMessageCount = 0;
  }

  Serial.print("Stop message count: ");
  Serial.println(stopMessageCount);

  delay(100);
}

int rotationMessage() {

  //PURPLE = PIN 14 (D5) - YELLOW RIGHT, ROTATE RIGHT
  //YELLOW =  PIN 15 (D8) - YELLOW LEFT, ROTATE LEFT
  //int rotation; //1 left, 2 right, 0 stop

  int valA = digitalRead(D5);
  int valB = digitalRead(D8);
  int rotation = 0;

  if (valA == 1) {
    rotation = 2;
  } else if (valB == 1) {
    rotation = 1;
  } else if (valB == 1 && valA == 1) {
    rotation = 0;
  } else {
    rotation = 0;
  }

  return rotation;
}

int hookMessage() {

  //WHITE = PIN 12 (D6) - BLUE RIGHT, HOOK UP
  //ORANGE = PIN 13 (D7) - BLUE LEFT, HOOK DOWN
  //int hook; //1 up , 2 down, 0 stop

  int valA = digitalRead(D6);
  int valB = digitalRead(D7);
  int hook = 0;

  if (valA == 1) {
    hook = 1;
  } else if (valB == 1) {
    hook = 2;
  } else if (valB == 1 && valA == 1) {
    hook = 0;
  } else {
    hook = 0;
  }

  return hook;
}


int boomMessage() {

  //GREEN =  PIN 5  (D1) - RED UP, BOOM OUT
  //BROWN =  PIN 4  (D2) - RED DOWN , BOOM IN
  //int boom; //1 in , 2 out, 0 stop

  int valA = digitalRead(D1);
  int valB = digitalRead(D2);
  int boom = 0;

  if (valA == 1) {
    boom = 2;
  } else if (valB == 1) {
    boom = 1;
  } else if (valB == 1 && valA == 1) {
    boom = 0;
  } else {
    boom = 0;
  }

  return boom;
}

void sendMessage(int boom, int rotation, int hook) {

  transmitter.boom = boom;
  transmitter.rotation = rotation;
  transmitter.hook = hook;
  transmitter.timeSent = millis();

  uint8_t msgToSend[sizeof(transmitter)];
  memcpy(msgToSend, &transmitter, sizeof(transmitter));

  unsigned long entry = millis();

  esp_now_send(NULL, msgToSend, sizeof(transmitter)); // NULL means send to all peers

  Serial.print("Time to send: ");
  Serial.println(millis() - entry);

}
