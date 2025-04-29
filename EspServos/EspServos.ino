#include <WiFi.h>
#include <ESP32Servo.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>


#define frontServoPin 33
#define ArmServoPinOne 26
#define ArmServoPinTwo 27
#define ArmServoPinThree 4
//#define ArmServoPinFour 999
#define RelayOne 23     // wheel one
#define RelayTwo 16     //22 //todo change //temp for testing needs to be changed
#define RelayDirOne 21  //todo change
//#define RelayDirTwo 16 //todo change
// ultrasonic echo 18
// ultrasonic trig 33
// force sensor 17

const char* ssid ="TheEvilWithin";//"Miyagi";// "TheEvilWithin";       //"Miyagi";  TP-Link_74CA
const char* password = "2PPG6262F3";//"$;)_eo73,,.5dhWLd*@";//"2PPG6262F3";      //"$;)_eo73,,.5dhWLd*@"; edidani1
const char* mqtt_server = "192.168.0.3";//"192.168.167.216";  //"192.168.43.144";
const int mqtt_port = 1883;


const char* publishTopicConfirmation = "ServoPosConfirm";
const char* subListenWantedDir = "wantedDirChangedTo";
const char* subTopicServoWantedPos = "FrontServoControl";
const char* subTopicActuator = "Actuator";
const char* subTopicHandServos = "HandServoControl";

WiFiClient EspWiFiclient;
PubSubClient client(EspWiFiclient);
Servo frontMovementServo;
Servo servo1;
Servo servo2;
Servo servo3;
//Servo servo4;
ESP32PWM pwm;
int WantedposFrontServo = 0;
int WantedposArmOne = 0;
int WantedposArmTwo = 130;
int WantedposArmThree = 0;
//int WantedposArmFour = 0;
int posFrontServo = 90;
int posArmOne = 0;
int posArmTwo = 130;
int posArmThree = 0;
//int posArmFour = 0;
bool sendFrontServoConfirm = false;
int relayAction = 0;  // 0 is stop(default), 1 is forward, 2 is back
bool stoppedServos = false;
void CheckConnections() {
  while (!client.connected()) {
   // StopRelay();
    StopServos();
    CheckWiFiConnection();
    if (client.connect("ESP32ClientServos")) {
      client.subscribe(subTopicServoWantedPos, 1);  //subscribes here
      client.subscribe(subTopicActuator, 1);
      client.subscribe(subTopicHandServos, 1);
      client.subscribe(subListenWantedDir, 1);

    } else {
      delay(2000);
    }
  }
}
void CheckWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
    }
  }
}
void setup() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  frontMovementServo.setPeriodHertz(50);
  frontMovementServo.attach(frontServoPin,500,2500);
  servo1.setPeriodHertz(50);
  servo1.attach(ArmServoPinOne);
  servo2.setPeriodHertz(50);
  servo2.attach(ArmServoPinTwo);
  servo3.setPeriodHertz(50);
  servo3.attach(ArmServoPinThree);
  // servo4.setPeriodHertz(50);
  // servo4.attach(ArmServoPinFour);
  pinMode(RelayOne, OUTPUT);
  digitalWrite(RelayOne, LOW);
  pinMode(RelayTwo, OUTPUT);
  digitalWrite(RelayTwo, LOW);
   
  resetConnectionParams();
}
void resetConnectionParams() {
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
}

bool partyStarted = false;
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  StaticJsonDocument<300> jsonDoc;
  DeserializationError error = deserializeJson(jsonDoc, message);
  if (error) {
    Serial.print("JSON deserialization failed: ");
    Serial.println(error.c_str());
    return;
  }
  //if() Check if this is the correct topic and do something accordingly
  // example way to handle keys
  if (jsonDoc.containsKey("stopServos")) {
    String tempAnswer = jsonDoc["stopServos"];
    if (tempAnswer == "true") {
      stoppedServos = true;
    } else {
      stoppedServos = false;
    }
  }
  if (jsonDoc.containsKey("answer")) {
    String tempAnswer = jsonDoc["answer"];
    if (tempAnswer == "true") {
      sendFrontServoConfirm = true;
    } else {
      sendFrontServoConfirm = false;
    }
  }
  if (jsonDoc.containsKey("posArmOne")) {
    WantedposArmOne = jsonDoc["posArmOne"];
    AdjustArmOneServoToPos();
  }
  if (jsonDoc.containsKey("posArmTwo")) {
    WantedposArmTwo = jsonDoc["posArmTwo"];
    AdjustArmTwoServoToPos();
  }
  if (jsonDoc.containsKey("dirLeft")) {
    String tempAnswer = jsonDoc["dirLeft"];
    if (tempAnswer == "true") {
      SwapDirLeft(true);
    } else {
      SwapDirLeft(false);
    }
  }
   if (jsonDoc.containsKey("dirRight")) {
    String tempAnswer = jsonDoc["dirRight"];
    if (tempAnswer == "true") {
      SwapDirRight(true);
    } else {
      SwapDirRight(false);
    }
  }
  // if (jsonDoc.containsKey("posArmThree")) {
  //   WantedposArmThree = jsonDoc["posArmThree"];
  // }
  // if (jsonDoc.containsKey("posArmFour")) {
  //   WantedposArmFour = jsonDoc["posArmFour"];
  // }
  if (jsonDoc.containsKey("relayActionToTake")) {
    relayAction = jsonDoc["relayActionToTake"];
    //  updateRelay();
  }
  if (jsonDoc.containsKey("wantedDirection")) {
    WantedposFrontServo = jsonDoc["wantedDirection"];
    AdjustFrontServoToPos();
  }
}
void AdjustFrontServoToPos() {
  while (!stoppedServos && WantedposFrontServo != posFrontServo) {
    if (posFrontServo >= WantedposFrontServo) {
      posFrontServo--;
      frontMovementServo.write(posFrontServo);
      delay(10);
    } else if (posFrontServo <= WantedposFrontServo) {
      posFrontServo++;
      frontMovementServo.write(posFrontServo);
      delay(10);
    }
  }
  if (sendFrontServoConfirm && !stoppedServos && WantedposFrontServo == posFrontServo) {
    publishAnswerForFrontWheel();
  }
}
void AdjustArmOneServoToPos() {
  while (!stoppedServos && WantedposArmOne != posArmOne) {
    CheckConnections();
    client.loop();
    if (posArmOne > WantedposArmOne) {
      posArmOne--;
      servo1.write(posArmOne);
      delay(35);
    } else if (posArmOne < WantedposArmOne) {
      posArmOne++;
      servo1.write(posArmOne);
      delay(35);
    }
  }
}
void AdjustArmTwoServoToPos() {
  while (!stoppedServos && WantedposArmTwo != posArmTwo) {
    CheckConnections();
    client.loop();
    if (posArmTwo > WantedposArmTwo) {
      posArmTwo--;
      servo2.write(posArmTwo);
      delay(35);
    } else if (posArmTwo < WantedposArmTwo) {
      posArmTwo++;
      servo2.write(posArmTwo);
      delay(35);
    }
  }
}
void StopRelay() {  // to be called during other operations to ensure no bad things happen :)
  relayAction = 0;
  updateRelay();
}
bool LeftWheelmovingForwardDir = false;
bool RightWheelmovingForwardDir = false;
void SwapDirLeft(bool dir) {
  if (dir) {
    LeftWheelmovingForwardDir = true;
    digitalWrite(RelayOne, HIGH);
  } else {
    LeftWheelmovingForwardDir = false;
    digitalWrite(RelayOne, LOW);
  }
  publishAnswerForDirLeft();
}
void SwapDirRight(bool dir) {
  if (dir) {
    RightWheelmovingForwardDir = true;
    digitalWrite(RelayTwo, HIGH);
  } else {
    RightWheelmovingForwardDir = false;
    digitalWrite(RelayTwo, LOW);
  }
  publishAnswerForDirRight();
}
void publishAnswerForDirLeft() {
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["wantedDirLeftReached"] = LeftWheelmovingForwardDir;
  char jsonBuffer[256];
  serializeJson(jsonDoc, jsonBuffer);
  client.publish(publishTopicConfirmation, (const uint8_t*)jsonBuffer, strlen(jsonBuffer), false);  //topic is a bit wrong but no matter
}
void publishAnswerForDirRight() {
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["wantedDirRightReached"] = RightWheelmovingForwardDir;
  char jsonBuffer[256];
  serializeJson(jsonDoc, jsonBuffer);
  client.publish(publishTopicConfirmation, (const uint8_t*)jsonBuffer, strlen(jsonBuffer), false);  //topic is a bit wrong but no matter
}
void StopServos() {
  stoppedServos = true;
}
void publishAnswerForFrontWheel() {
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["wantedPosReached"] = true;
  char jsonBuffer[256];
  serializeJson(jsonDoc, jsonBuffer);
  client.publish(publishTopicConfirmation, (const uint8_t*)jsonBuffer, strlen(jsonBuffer), false);
}
void updateRelay() {  // !!! H-bridge NEVER HIGH HIGH
  digitalWrite(RelayOne, LOW);
  digitalWrite(RelayTwo, LOW);
  delay(25);
  if (relayAction == 1) {
    digitalWrite(RelayOne, HIGH);
    digitalWrite(RelayTwo, LOW);
  } else if (relayAction == 2) {
    digitalWrite(RelayOne, LOW);
    digitalWrite(RelayTwo, HIGH);
  }
}

void loop() {
  CheckConnections();
  client.loop();  // must be called constantly to check for new data
  delay(5);
}
