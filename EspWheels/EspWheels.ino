#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <ESP32Servo.h>
#include <math.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define signalOutputInterruptPinRight 23
#define signalOutputInterruptPinLeft 5
#define analogOutputLeftPin 26   //17   //33
#define analogOutputRightPin 16  //14
#define trigPin 33               //26
#define echoMidPin 18
#define echoLeftPin 19   //17
#define echoRightPin 17  //16

#define WHEEL_DIAM 203.2
//#define distance_Wheels 140   // TO BE UPDATED?
#define TICKS_PER_REV 42
#define MM_PER_TICK (((WHEEL_DIAM * PI) / TICKS_PER_REV))
#define MM_TO_TICKS(A) ((float)(A)) / MM_PER_TICK
//#define MM_PER_DEGREES(D) ((distance_Wheels * PI * (D)) / 360.0f)  // TO BE UPDATED?
//#define turnDegr(M) ((M) / (PI * distance_Wheels * 360.0f)) // TO BE UPDATED?
#define millisecToRecordTicksInterval 200

const char* ssid = "Miyagi";
const char* password = "$;)_eo73,,.5dhWLd*@";
const char* mqtt_server = "192.168.43.144";
const int mqtt_port = 1883;

const char* publishTopicMapData = "DataForMapping";
const char* publishTopicMagCalibration = "CaliberationMagData";
const char* publishTopicServoControl = "FrontServoControl";
const char* subTopicMovement = "Movement";
const char* subTopicConfirmation = "ServoPosConfirm";

WiFiClient EspWiFiclient;
PubSubClient client(EspWiFiclient);
ESP32PWM motorLeftPWM;
ESP32PWM motorRightPWM;
double midUSArr[] = { 0, 0, 0 };
double leftUSArr[] = { 0, 0, 0 };
double rightUSArr[] = { 0, 0, 0 };
double _medianMid = 0;
double _medianLeft = 0;
double _medianRight = 0;
volatile long leftEncoderCounter = 0;
volatile long rightEncoderCounter = 0;
volatile bool newMidData = false;
volatile bool newLeftData = false;
volatile bool newRightData = false;
volatile unsigned long microSecMid = 0;
volatile unsigned long microSecMidEndDuration = 0;
volatile unsigned long microSecLeft = 0;
volatile unsigned long microSecLeftEndDuration = 0;
volatile unsigned long microSecRight = 0;
volatile unsigned long microSecRightEndDuration = 0;
int bounce = 7;
volatile unsigned long lastLeftRec = 0;
volatile unsigned long lastRightRec = 0;
volatile bool canPingMid = true;
volatile bool canPingRight = true;
volatile bool canPingLeft = true;
double distanceMid = 0;
double distanceLeft = 0;
double distanceRight = 0;
volatile bool canCollectDataMid = false;
volatile bool canCollectDataLeft = false;
volatile bool canCollectDataRight = false;
unsigned long timeOfLastTrigger = millis();
double weMoved = 0;
double weMovedAuto = 0;
int lastRightEncoderCounterUsedToCalculate = 0;
int lastLeftEncoderCounterUsedToCalculate = 0;
int lastRightEncoderCounterUsedToCalculateAuto = 0;
int lastLeftEncoderCounterUsedToCalculateAuto = 0;
Adafruit_HMC5883_Unified mag;
sensors_event_t event;
unsigned long previousTimeThereWasAnObstacle = millis();
bool goingForward = false;
bool turnedLeft = false;
bool turnedRight = false;
float PWMLeftCoefficient = 1;
float PWMRightCoefficient = 1;
unsigned long speedTimer = 0;
unsigned long speedAdjustTimer = 0;
int LastSecondTicksLeft[] = { 0, 0, 0, 0, 0 };
int LastSecondTicksRight[] = { 0, 0, 0, 0, 0 };
int timeIntervalIndexCounter = 0;
int rightthing = 0;
long lastRight = 0;
int leftthing = 0;
long lastLeft = 0;
float mag_datat[] = { event.magnetic.x,
                      event.magnetic.y,
                      event.magnetic.z };
float lastDegKeepDir = 0;
float changeKeepDir = 0;
float degreeChangeFromStartKeepDir = 0;
bool stopSignal = false;
bool startingServoPosReached = false;
const float hard_iron[3] = {  // with magneto the values are new
  -107.431847, 122.222755, 220.246822
};

const float soft_iron[3][3] = {
  { 1.602628, -0.006533, -0.149331 },
  { -0.006533, 1.813032, 0.170965 },
  { -0.149331, 0.170965, 2.302615 }
};

void setup() {
  pinMode(signalOutputInterruptPinRight, INPUT);
  pinMode(signalOutputInterruptPinLeft, INPUT);
  pinMode(analogOutputLeftPin, OUTPUT);
  pinMode(analogOutputRightPin, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoMidPin, INPUT);
  pinMode(echoLeftPin, INPUT);
  pinMode(echoRightPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(echoMidPin), IRS_MidSensor, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echoLeftPin), IRS_LeftSensor, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echoRightPin), IRS_RightSensor, CHANGE);
  attachInterrupt(digitalPinToInterrupt(signalOutputInterruptPinRight), IRS_RightEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(signalOutputInterruptPinLeft), IRS_LeftEncoder, RISING);
  mag = Adafruit_HMC5883_Unified();
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  motorLeftPWM.attachPin(analogOutputLeftPin, 5000, 10);
  motorRightPWM.attachPin(analogOutputRightPin, 5000, 10);
  Serial.begin(115200);
  if (!mag.begin()) {
    while (1) { delay(10); }
  }
  resetConnectionParams();
}
void resetConnectionParams() {
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
}
void setPWMLeft(float duty_cycle) {  // 0.0 to 1.0
  if (duty_cycle >= 0.95) {
    motorLeftPWM.writeScaled(0.95);
  } else if (duty_cycle < 0) {
    motorLeftPWM.writeScaled(0);
  } else {
    motorLeftPWM.writeScaled(duty_cycle);
  }
}
void setPWMRight(float duty_cycle) {
  if (duty_cycle >= 0.95) {
    motorRightPWM.writeScaled(0.95);
  } else if (duty_cycle < 0) {
    motorRightPWM.writeScaled(0);
  } else {
    motorRightPWM.writeScaled(duty_cycle);
  }
}
void ResetEncoderValues() {
  rightEncoderCounter = 0;
  leftEncoderCounter = 0;
  lastRightEncoderCounterUsedToCalculate = 0;
  lastLeftEncoderCounterUsedToCalculate = 0;
  lastRightEncoderCounterUsedToCalculateAuto = 0;
  lastLeftEncoderCounterUsedToCalculateAuto = 0;
}
void IRS_MidSensor() {
  if (digitalRead(echoMidPin) == 0x0) {
    if (canCollectDataMid && !canPingMid) {
      microSecMidEndDuration = micros() - microSecMid;
      canCollectDataMid = false;
      newMidData = true;
    }
  } else {
    if (canPingMid) {
      microSecMid = micros();
      canPingMid = false;
      canCollectDataMid = true;
    }
  }
}
void IRS_LeftSensor() {
  if (digitalRead(echoLeftPin) == 0x0) {
    if (canCollectDataLeft && !canPingLeft) {
      microSecLeftEndDuration = micros() - microSecLeft;
      canCollectDataLeft = false;
      newLeftData = true;
    }
  } else {
    if (canPingLeft) {
      microSecLeft = micros();
      canPingLeft = false;
      canCollectDataLeft = true;
    }
  }
}
void IRS_RightSensor() {
  if (digitalRead(echoRightPin) == 0x0) {
    if (canCollectDataRight && !canPingRight) {
      microSecRightEndDuration = micros() - microSecRight;
      canCollectDataRight = false;
      newRightData = true;
    }
  } else {
    if (canPingRight) {
      microSecRight = micros();
      canPingRight = false;
      canCollectDataRight = true;
    }
  }
}
void IRS_LeftEncoder() {
  if (millis() - lastLeftRec >= bounce) {
    leftEncoderCounter++;
    lastLeftRec = millis();
  }
}
void IRS_RightEncoder() {
  if (millis() - lastRightRec >= bounce) {
    rightEncoderCounter++;
    lastRightRec = millis();
  }
}
void GetUltrasoundData(float dir, bool sendMove) {
  int rigthCount = rightEncoderCounter;
  int leftCount = leftEncoderCounter;
  double rightDistanceMoved = (rigthCount - lastRightEncoderCounterUsedToCalculate) * MM_PER_TICK;
  double leftDistanceMoved = (leftCount - lastLeftEncoderCounterUsedToCalculate) * MM_PER_TICK;
  weMoved = /* (rightDistanceMoved + */ leftDistanceMoved;  //); /// 2.0;
  lastRightEncoderCounterUsedToCalculate = rigthCount;
  lastLeftEncoderCounterUsedToCalculate = leftCount;

  if (millis() - timeOfLastTrigger >= 75) {
    canPingLeft = true;
    canPingMid = true;
    canPingRight = true;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    timeOfLastTrigger = millis();
  }
  if (newRightData) {
    newRightData = false;
    distanceRight = (microSecRightEndDuration * 0.0343) / 2;
    rightUSArr[0] = rightUSArr[1];
    rightUSArr[1] = rightUSArr[2];
    rightUSArr[2] = distanceRight;
    if (rightUSArr[2] < rightUSArr[1]) {
      if (rightUSArr[2] < rightUSArr[0]) {
        if (rightUSArr[1] < rightUSArr[0]) {
          _medianRight = rightUSArr[1];
        } else {
          _medianRight = rightUSArr[0];
        }
      } else {
        _medianRight = rightUSArr[2];
      }
    } else {
      if (rightUSArr[2] < rightUSArr[0]) {
        _medianRight = rightUSArr[2];
      } else {
        if (rightUSArr[1] < rightUSArr[0]) {
          _medianRight = rightUSArr[0];
        } else {
          _medianRight = rightUSArr[1];
        }
      }
    }
  }
  if (newMidData) {
    newMidData = false;
    distanceMid = (microSecMidEndDuration * 0.0343) / 2;
    midUSArr[0] = midUSArr[1];
    midUSArr[1] = midUSArr[2];
    midUSArr[2] = distanceMid;

    if (midUSArr[2] < midUSArr[1]) {
      if (midUSArr[2] < midUSArr[0]) {
        if (midUSArr[1] < midUSArr[0]) {
          _medianMid = midUSArr[1];
        } else {
          _medianMid = midUSArr[0];
        }
      } else {
        _medianMid = midUSArr[2];
      }
    } else {
      if (midUSArr[2] < midUSArr[0]) {
        _medianMid = midUSArr[2];
      } else {
        if (midUSArr[1] < midUSArr[0]) {
          _medianMid = midUSArr[0];
        } else {
          _medianMid = midUSArr[1];
        }
      }
    }
    if (_medianMid < 40) {
      previousTimeThereWasAnObstacle = millis();
    }
  }

  if (newLeftData) {
    newLeftData = false;
    distanceLeft = (microSecLeftEndDuration * 0.0343) / 2;
    leftUSArr[0] = leftUSArr[1];
    leftUSArr[1] = leftUSArr[2];
    leftUSArr[2] = distanceLeft;
    if (leftUSArr[2] < leftUSArr[1]) {
      if (leftUSArr[2] < leftUSArr[0]) {
        if (leftUSArr[1] < leftUSArr[0]) {
          _medianLeft = leftUSArr[1];
        } else {
          _medianLeft = leftUSArr[0];
        }
      } else {
        _medianLeft = leftUSArr[2];
      }
    } else {
      if (leftUSArr[2] < leftUSArr[0]) {
        _medianLeft = leftUSArr[2];
      } else {
        if (leftUSArr[1] < leftUSArr[0]) {
          _medianLeft = leftUSArr[0];
        } else {
          _medianLeft = leftUSArr[1];
        }
      }
    }
  }
  if (weMoved > 0 && sendMove) {
    publishJsonDataForMap(dir, weMoved, _medianLeft, _medianMid, _medianRight);
  }
}
float MagneticSensorReading() {  // NEEDS TO BE REVERTED LATER
  mag.getEvent(&event);
  float hi_cal[3];
  float mag_data[] = { event.magnetic.x * 10 * 1.3,
                       event.magnetic.y * 10 * 1.3,
                       event.magnetic.z * 10 * 1.3 };
  for (uint8_t i = 0; i < 3; i++) {
    hi_cal[i] = mag_data[i] - hard_iron[i];
  }
  for (uint8_t i = 0; i < 3; i++) {
    mag_data[i] = (soft_iron[i][0] * hi_cal[0]) + (soft_iron[i][1] * hi_cal[1]) + (soft_iron[i][2] * hi_cal[2]);
  }
  float heading = atan2(mag_data[0], mag_data[1]);
  if (heading < 0)
    heading += 2 * PI;
  if (heading > 2 * PI)
    heading -= 2 * PI;
  float headingDegrees = heading * 180 / M_PI;


  return headingDegrees;
}
void MagneticSensorReadingFORPROCESSING() {  // NEEDS TO BE REVERTED LATER
  mag.getEvent(&event);
  float hi_cal[3];
  mag_datat[0] = event.magnetic.x * 10 * 1.3;
  mag_datat[1] = event.magnetic.y * 10 * 1.3;
  mag_datat[2] = event.magnetic.z * 10 * 1.3;
}
float PidControllerSpeedLeft(float target, float kp, float current) {
  float e = current - target;
  float u = (kp * e);
  return u * -1;
}
float PidControllerSpeedRight(float target, float kp, float current) {
  float e = current - target;
  float u = (kp * e);
  return u * -1;
}
void ManualMovement(String signal) {
  ResetEncoderValues();
  if (signal == "w" || signal == "W") {
    justForward();
  } else if (signal == "a" || signal == "A") {
    justLeftRight(1);
  } else if (signal == "s" || signal == "S") {
    StopMovement();
  } else if (signal == "d" || signal == "D") {
    justLeftRight(-1);
  } else if (signal == "none" || signal == "None") {
    StopMovement();
  }
}
void justLeftRight(int direction) {
  turnedRight = false;
  turnedLeft = false;
  while (client.connected() && !stopSignal) {
    CheckWiFiConnection();
    client.loop();
    GetUltrasoundData(MagneticSensorReading(), true);
    //GetUltrasoundData(0, true);
    if (direction > 0) {
      if (!turnedLeft) {
        startingServoPosReached = false;
        AdjustPosTo(35, true);
        turnedLeft = true;
        goingForward = false;
        turnedRight = false;
      }
      if (startingServoPosReached) {
        setPWMLeft(0);
        setPWMRight(0.6);
      }
    } else {
      if (!turnedRight) {
        startingServoPosReached = false;
        AdjustPosTo(145, true);
        turnedLeft = false;
        goingForward = false;
        turnedRight = true;
      }
      if (startingServoPosReached) {
        setPWMLeft(0.6);
        setPWMRight(0);
      }
    }
  }
  StopMovement();
  goingForward = false;
  turnedLeft = false;
  turnedRight = false;
}
void CollectAndSendMagDataForCalibration() {
  while (true) {
    MagneticSensorReadingFORPROCESSING();
    StaticJsonDocument<300> jsonDoc;
    jsonDoc["x"] = mag_datat[0];
    jsonDoc["y"] = mag_datat[1];
    jsonDoc["z"] = mag_datat[2];
    char jsonBuffer[256];
    serializeJson(jsonDoc, jsonBuffer);
    client.publish(publishTopicMagCalibration, (const uint8_t*)jsonBuffer, strlen(jsonBuffer), false);
    delay(150);
  }
}
void UpdateTicksRight() {
  rightthing = rightEncoderCounter - lastRight;
  lastRight = rightEncoderCounter;
  LastSecondTicksRight[timeIntervalIndexCounter] = rightthing;
}
void UpdateTicksLeft() {
  leftthing = leftEncoderCounter - lastLeft;
  lastLeft = leftEncoderCounter;
  LastSecondTicksLeft[timeIntervalIndexCounter] = leftthing;
}
float GetCurrentSpeedRight() {
  return (LastSecondTicksRight[0] + LastSecondTicksRight[1] + LastSecondTicksRight[2] + LastSecondTicksRight[3] + LastSecondTicksRight[4]) * MM_PER_TICK * 0.0036;
}
float GetCurrentSpeedLeft() {
  return (LastSecondTicksLeft[0] + LastSecondTicksLeft[1] + LastSecondTicksLeft[2] + LastSecondTicksLeft[3] + LastSecondTicksLeft[4]) * MM_PER_TICK * 0.0036;
}
void ResetKeepDir() {
  lastDegKeepDir = 0;
  changeKeepDir = 0;
  degreeChangeFromStartKeepDir = 0;
}
void keepDirection() {
  float currentDeg = MagneticSensorReading();  //0;
  changeKeepDir = currentDeg - lastDegKeepDir;
  if (changeKeepDir > 200) {
    changeKeepDir -= 360;
  } else if (changeKeepDir < -200) {
    changeKeepDir += 360;
  }
  degreeChangeFromStartKeepDir += changeKeepDir;
  lastDegKeepDir = currentDeg;
  AdjustPosTo(90 + (degreeChangeFromStartKeepDir * 0.75), false);
}
void AdjustPosTo(int wanted, bool waitAnswer) {
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["wantedDirection"] = wanted;
  jsonDoc["stopServos"] = false;
  jsonDoc["answer"] = waitAnswer;
  char jsonBuffer[256];
  serializeJson(jsonDoc, jsonBuffer);
  if (waitAnswer) {
    client.publish(publishTopicServoControl, (const uint8_t*)jsonBuffer, strlen(jsonBuffer), false);
  } else {
    client.publish(publishTopicServoControl, (const uint8_t*)jsonBuffer, strlen(jsonBuffer), false);
  }
}
void justForward() {
  speedTimer = millis();
  speedAdjustTimer = millis();
  rightthing = 0;
  lastRight = 0;
  PWMLeftCoefficient = 1;
  PWMRightCoefficient = 1;
  goingForward = false;
  ResetKeepDir();
  lastDegKeepDir = MagneticSensorReading();
  while (client.connected() && !stopSignal) {
    CheckWiFiConnection();
    client.loop();
    if (!goingForward) {
      startingServoPosReached = false;
      AdjustPosTo(90, true);
      goingForward = true;
      turnedLeft = false;
      turnedRight = false;
    }
    if (startingServoPosReached) {

      GetUltrasoundData(MagneticSensorReading(), true);
      // GetUltrasoundData(0, true);

      if (millis() - speedTimer >= millisecToRecordTicksInterval) {  // update the speed count o feach wheel every X seconds. In this case 200ms so the array of 5 records is the speed from last second
        if (5 <= timeIntervalIndexCounter) {
          timeIntervalIndexCounter = 0;
        }
        UpdateTicksRight();
        UpdateTicksLeft();
        timeIntervalIndexCounter++;
        speedTimer = millis();
      }
      // Serial.println("Left is .. Then right is ..");
      // Serial.println(leftEncoderCounter);
      // Serial.println(rightEncoderCounter);

      if (millis() - speedAdjustTimer >= 100) {  // Minimum time between pwm changes
        float changeLEft = PidControllerSpeedLeft(3, 0.015, GetCurrentSpeedLeft());
        float changeRight = PidControllerSpeedRight(3, 0.015, GetCurrentSpeedRight());  // IMPORTANT the signs will likely be reversed so if it refuses to go try reversing them aka PID returns - when it should be +

        if (fabs(changeLEft) > 0.0001) {
          if (PWMLeftCoefficient + changeLEft < 1.9 && PWMLeftCoefficient + changeLEft > 0.1) {
            PWMLeftCoefficient += changeLEft;
          }
        }
        if (fabs(changeRight) > 0.0001) {
          if (PWMRightCoefficient + changeRight < 1.9 && PWMRightCoefficient + changeRight > 0.1) {
            PWMRightCoefficient += changeRight;
          }
        }
        if (millis() - previousTimeThereWasAnObstacle <= 250) {
          StopMovement();
        } else {
          // setPWMRight(0.5 * PWMRightCoefficient);
          setPWMRight(0.5 * PWMLeftCoefficient);
          setPWMLeft(0.5 * PWMLeftCoefficient);
          keepDirection();
        }
        speedAdjustTimer = millis();
      }
      //*/
      //setPWMRight(0.5);
      //setPWMLeft(0.5);
    }
  }
  StopMovement();
  goingForward = false;
  turnedLeft = false;
  turnedRight = false;
}
void forward(int mm) {
  speedTimer = millis();
  speedAdjustTimer = millis();
  rightthing = 0;
  lastRight = 0;
  PWMLeftCoefficient = 1;
  PWMRightCoefficient = 1;
  goingForward = false;
  ResetEncoderValues();
  ResetKeepDir();
  lastDegKeepDir = MagneticSensorReading();
  weMovedAuto = 0;
  while (weMovedAuto < mm && !stopSignal) {
    int rigthCount = rightEncoderCounter;
    int leftCount = leftEncoderCounter;
    double rightDistanceMoved = (rigthCount - lastRightEncoderCounterUsedToCalculateAuto) * MM_PER_TICK;
    double leftDistanceMoved = (leftCount - lastLeftEncoderCounterUsedToCalculateAuto) * MM_PER_TICK;
    weMovedAuto += /*(rightDistanceMoved + */ leftDistanceMoved;  //) / 2.0;
    lastRightEncoderCounterUsedToCalculateAuto = rigthCount;
    lastLeftEncoderCounterUsedToCalculateAuto = leftCount;
    if (!goingForward) {
      startingServoPosReached = false;
      AdjustPosTo(90, true);
      goingForward = true;
      turnedLeft = false;
      turnedRight = false;
    }
    if (startingServoPosReached) {
      GetUltrasoundData(MagneticSensorReading(), false);
      if (millis() - speedTimer >= millisecToRecordTicksInterval) {  // update the speed count o feach wheel every X seconds. In this case 200ms so the array of 5 records is the speed from last second
        if (5 <= timeIntervalIndexCounter) {
          timeIntervalIndexCounter = 0;
        }
        UpdateTicksRight();
        UpdateTicksLeft();
        timeIntervalIndexCounter++;
        speedTimer = millis();
      }

      if (millis() - speedAdjustTimer >= 100) {  // Minimum time between pwm changes
        float changeLEft = PidControllerSpeedLeft(3, 0.015, GetCurrentSpeedLeft());
        float changeRight = PidControllerSpeedRight(3, 0.015, GetCurrentSpeedRight());  // IMPORTANT the signs will likely be reversed so if it refuses to go try reversing them aka PID returns - when it should be +

        if (fabs(changeLEft) > 0.0001) {
          if (PWMLeftCoefficient + changeLEft < 1.9 && PWMLeftCoefficient + changeLEft > 0.1) {
            PWMLeftCoefficient += changeLEft;
          }
        }
        if (fabs(changeRight) > 0.0001) {
          if (PWMRightCoefficient + changeRight < 1.9 && PWMRightCoefficient + changeRight > 0.1) {
            PWMRightCoefficient += changeRight;
          }
        }
        if (millis() - previousTimeThereWasAnObstacle <= 250) {
          StopMovement();
        } else {
            // setPWMRight(0.5 * PWMRightCoefficient);
          setPWMRight(0.5 * PWMLeftCoefficient);
          setPWMLeft(0.5 * PWMLeftCoefficient);
          keepDirection();
        }
        speedAdjustTimer = millis();
      }
    }
  }
  StopMovement();
  goingForward = false;
  turnedLeft = false;
  turnedRight = false;
  CheckWiFiConnection();
  client.loop();
}
void turn(float degree) {
  delay(1250);  // delay for probable inertia from movement
  ResetEncoderValues();
  float degreeChangeFromStart = 0;
  float currentDeg = MagneticSensorReading();
  float lastDeg = currentDeg;
  float change = 0;

  while (fabs(degree) - fabs(degreeChangeFromStart) > 10 && !stopSignal) {
    CheckConnections();
    client.loop();
    currentDeg = MagneticSensorReading();
    change = currentDeg - lastDeg;
    if (change > 200) {
      change -= 360;
    } else if (change < -200) {
      change += 360;
    }
    degreeChangeFromStart += change;
    lastDeg = currentDeg;

    if (degree < 1) {
      if (!turnedRight) {
        startingServoPosReached = false;
        AdjustPosTo(145, true);
        turnedLeft = false;
        goingForward = false;
        turnedRight = true;
      }
      if (startingServoPosReached) {
        setPWMLeft(0.6);
        setPWMRight(0);
      }
    } else {
      if (!turnedLeft) {
        startingServoPosReached = false;
        AdjustPosTo(35, true);
        turnedLeft = true;
        goingForward = false;
        turnedRight = false;
      }
      if (startingServoPosReached) {
        setPWMLeft(0);
        setPWMRight(0.6);
      }
    }
  }
  StopMovement();
  turnedLeft = false;
  goingForward = false;
  turnedRight = false;
  CheckWiFiConnection();
  client.loop();
}
void publishJsonDataForMap(float direction, float movement, float left, float mid, float right) {
  StaticJsonDocument<300> jsonDoc;
  jsonDoc["direction"] = direction;
  jsonDoc["movement"] = movement;
  jsonDoc["leftSensor"] = left;
  jsonDoc["midSensor"] = mid;
  jsonDoc["rightSensor"] = right;
  char jsonBuffer[256];
  serializeJson(jsonDoc, jsonBuffer);
  client.publish(publishTopicMapData, (const uint8_t*)jsonBuffer, strlen(jsonBuffer), false);
}
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
  if (jsonDoc.containsKey("stopSignal")) {
    String tempAnswer = jsonDoc["stopSignal"];
    if (tempAnswer == "true") {
      stopSignal = true;
    } else {
      stopSignal = false;
    }
  }
  if (jsonDoc.containsKey("calMag")) {
    int action = jsonDoc["calMag"];
    if (action == 1) {
      CollectAndSendMagDataForCalibration();
    }
  }
  if (jsonDoc.containsKey("move")) {
    int moveDistance = jsonDoc["move"];
    forward(moveDistance);
  }
  if (jsonDoc.containsKey("turn")) {
    float Turn = jsonDoc["turn"];
    turn(Turn);
  }
  if (jsonDoc.containsKey("manualCommand")) {
    String signal = jsonDoc["manualCommand"];
    ManualMovement(signal);
  }
  if (jsonDoc.containsKey("wantedPosReached")) {
    String tempAnswer = jsonDoc["wantedPosReached"];
    if (tempAnswer == "true") {
      startingServoPosReached = true;
    } else {
      startingServoPosReached = false;
    }
  }
}
void StopMovement() {  //викам го ако изгубя връзка или има сигнал за стоп
  setPWMLeft(0);
  setPWMRight(0);
}
void CheckConnections() {
  while (!client.connected()) {
    stopSignal = true;
    CheckWiFiConnection();
    if (client.connect("ESP32ClientWheels")) {
      client.subscribe(subTopicMovement, 1);
      client.subscribe(subTopicConfirmation, 1);
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
void loop() {
  CheckConnections();
  client.loop();  // must be called constantly to check for new data
  delay(100);
  //*/

  /*setPWMRight(0.5);
  setPWMLeft(0.5);
  delay(2500);
  setPWMRight(0);
  setPWMLeft(0);  */
}
