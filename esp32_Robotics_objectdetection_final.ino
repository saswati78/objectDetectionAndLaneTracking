#include <WebSocketsServer.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "camera_wrap.h"

// Network Configuration
const char* ssid = "iot02";    
const char* password = "rpw1@iot"; 

/* Static IP Configuration
IPAddress staticIP(192, 168, 1, 100);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(8, 8, 8, 8);
*/
// Pin Definitions
const int LED_BUILT_IN        = 4;
const int PIN_SERVO_PITCH     = 12;
const int PINDC_LEFT_BACK     = 13;
const int PINDC_LEFT_FORWARD  = 15;
const int PINDC_RIGHT_BACK    = 14;
const int PINDC_RIGHT_FORWARD = 2;

// PWM Channels
const int LEFT_CHANNEL        = 2;
const int RIGHT_CHANNEL       = 3;
const int SERVO_PITCH_CHANNEL = 4;
const int SERVO_YAW_CHANNEL   = 5;

// Configuration Constants
const int SERVO_RESOLUTION    = 16;
const uint8_t TRACK_DUTY      = 100;
const unsigned long SERVO_INTERVAL = 10;

// Global Variables
int cameraInitState = -1;
uint8_t* jpgBuff = new uint8_t[68123];
size_t jpgLength = 0;
uint8_t camNo = 0;
bool clientConnected = false;

// UDP Configuration
WiFiUDP UDPServer;
IPAddress addrRemote;
unsigned int portRemote;
const unsigned int UDP_PORT = 6868;
const int RECV_LENGTH = 16;
byte packetBuffer[RECV_LENGTH];

WebSocketsServer webSocket = WebSocketsServer(86);

// Servo and Motor State
int posServo = 75;
bool servoUp = false;
bool servoDown = false;
int PWMTrackHIGH = 138;
int PWMTrackLOW = 138;

void servoWrite(uint8_t channel, uint8_t angle) {
  uint32_t maxDuty = (pow(2, SERVO_RESOLUTION) - 1) / 10; 
  uint32_t minDuty = (pow(2, SERVO_RESOLUTION) - 1) / 20; 
  uint32_t duty = (maxDuty - minDuty) * angle / 180 + minDuty;
  ledcWrite(channel, duty);
}

void controlServo() {
  if (servoUp && posServo > 2) posServo -= 2;
  if (servoDown && posServo < 180) posServo += 2;
  servoWrite(SERVO_PITCH_CHANNEL, posServo);
}

void controlDC(int left0, int left1, int right0, int right1) {
  digitalWrite(PINDC_LEFT_BACK, left0);
  digitalWrite(PINDC_LEFT_FORWARD, left1);
  ledcWrite(LEFT_CHANNEL, left1 == HIGH ? 255 : 0);

  digitalWrite(PINDC_RIGHT_BACK, right0);
  digitalWrite(PINDC_RIGHT_FORWARD, right1);
  ledcWrite(RIGHT_CHANNEL, right1 == HIGH ? 255 : 0);
}

void controlDCTrack(int left, int right) {
  digitalWrite(PINDC_LEFT_BACK, 0);
  digitalWrite(PINDC_LEFT_FORWARD, left > 0);
  ledcWrite(LEFT_CHANNEL, abs(left));

  digitalWrite(PINDC_RIGHT_BACK, 0);
  digitalWrite(PINDC_RIGHT_FORWARD, right > 0);
  ledcWrite(RIGHT_CHANNEL, abs(right));
}

void processUDPData() {
  int cb = UDPServer.parsePacket();
  if (cb) {
    UDPServer.read(packetBuffer, RECV_LENGTH);
    addrRemote = UDPServer.remoteIP();
    portRemote = UDPServer.remotePort();

    String strPackage = String((const char*)packetBuffer);
    
    if (strPackage.equals("whoami")) {
      UDPServer.beginPacket(addrRemote, portRemote - 1);
      UDPServer.write((const uint8_t*)"ESP32-CAM", 9);
      UDPServer.endPacket();
    } else if (strPackage.equals("forward")) {
      controlDC(LOW, HIGH, LOW, HIGH);
    } else if (strPackage.equals("backward")) {
      controlDC(HIGH, LOW, HIGH, LOW);
    } else if (strPackage.equals("left")) {
      controlDC(HIGH, LOW, LOW, LOW);
    } else if (strPackage.equals("right")) {
      controlDC(LOW, HIGH, LOW, LOW);
    } else if (strPackage.equals("stop")) {
      controlDC(LOW, LOW, LOW, LOW);
    } else if (strPackage.equals("camup")) {
      servoUp = true;
    } else if (strPackage.equals("camdown")) {
      servoDown = true;
    } else if (strPackage.equals("camstill")) {
      servoUp = false;
      servoDown = false;
    } else if (strPackage.equals("ledon")) {
      digitalWrite(LED_BUILT_IN, HIGH);
    } else if (strPackage.equals("ledoff")) {
      digitalWrite(LED_BUILT_IN, LOW);
    } else if (strPackage.equals("lefttrack")) {
      controlDCTrack(0, PWMTrackHIGH);
    } else if (strPackage.equals("righttrack")) {
      controlDCTrack(PWMTrackHIGH, 0);
    } else if (strPackage.equals("fwtrack")) {
      controlDCTrack(PWMTrackLOW, PWMTrackLOW);
    }

    memset(packetBuffer, 0, RECV_LENGTH);
  }
}

void setup() {
  Serial.begin(115200);

  // Static IP Configuration
  //if (!WiFi.config(staticIP, gateway, subnet, dns)) {
    //Serial.println("Static IP configuration failed");
  //}

  // Pin Configurations
  pinMode(LED_BUILT_IN, OUTPUT);
  pinMode(PINDC_LEFT_BACK, OUTPUT);
  pinMode(PINDC_LEFT_FORWARD, OUTPUT);
  pinMode(PINDC_RIGHT_BACK, OUTPUT);
  pinMode(PINDC_RIGHT_FORWARD, OUTPUT);

  digitalWrite(LED_BUILT_IN, LOW);

  // PWM Setup
  ledcAttach(LEFT_CHANNEL, 100, 8); 
  ledcAttach(RIGHT_CHANNEL, 100, 8); 
  ledcAttach(SERVO_PITCH_CHANNEL, 50, 16);

  // Initial Configurations
  controlDC(LOW, LOW, LOW, LOW);
  servoWrite(SERVO_PITCH_CHANNEL, posServo);

  // Camera Initialization
  cameraInitState = initCamera();
  if (cameraInitState != 0) return;

  // WiFi Connection
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected with Static IP: " + WiFi.localIP().toString());

  // Network Services
  webSocket.begin();
  webSocket.onEvent([](uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
      case WStype_DISCONNECTED:
        camNo = num;
        clientConnected = false;
        break;
      case WStype_CONNECTED:
        clientConnected = true;
        break;
    }
  });

  UDPServer.begin(UDP_PORT); 
}

void loop() {
  webSocket.loop();
  
  if (clientConnected) {
    grabImage(jpgLength, jpgBuff);
    webSocket.sendBIN(camNo, jpgBuff, jpgLength);
  }

  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= SERVO_INTERVAL) {
    previousMillis = currentMillis;
    processUDPData();
    controlServo();
  }
}

