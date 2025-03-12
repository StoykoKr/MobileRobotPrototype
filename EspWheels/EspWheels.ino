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
#define analogOutputRightPin 32  //14
#define trigPin 33               //33
#define trigPinHand 4
#define echoPinHand 16
#define echoMidPin 18
#define echoLeftPin 19   //17
#define echoRightPin 17  //16

#define LEFTDIRWHEEL 0  //just so its not that confusing.. or to make it more?
#define RIGHTDIRWHEEL 1

#define WHEEL_DIAM 203.2
//#define distance_Wheels 140   // TO BE UPDATED?
#define TICKS_PER_REV 45  //42
#define MM_PER_TICK (((WHEEL_DIAM * PI) / TICKS_PER_REV))
#define MM_TO_TICKS(A) ((double)(A)) / MM_PER_TICK
//#define MMipconfig_PER_DEGREES(D) ((distance_Wheels * PI * (D)) / 360.0f)  // TO BE UPDATED?
//#define turnDegr(M) ((M) / (PI * distance_Wheels * 360.0f)) // TO BE UPDATED?
#define millisecToRecordTicksInterval 200

const char* ssid = "TheEvilWithin";       //"Miyagi";// "TheEvilWithin";       //"Miyagi";  TP-Link_74CA
const char* password = "2PPG6262F3";      //"$;)_eo73,,.5dhWLd*@";//"2PPG6262F3";      //"$;)_eo73,,.5dhWLd*@"; edidani1
const char* mqtt_server = "192.168.0.3";  //"192.168.167.216";  //"192.168.43.144";
const int mqtt_port = 1883;

const char* publishTopicMapData = "DataForMapping";
const char* publishTopicMagCalibration = "CaliberationMagData";
const char* publishWantedDirChange = "wantedDirChangedTo";
const char* publishTopicServoControl = "FrontServoControl";
const char* subTopicMovement = "Movement";
const char* subTopicConfirmation = "ServoPosConfirm";
const char* supDirTemp = "movingForwardDir";
const char* subTopicAutoMove = "LeftRightSpeed";
const char* subMapRequestData = "MapDataRequest";

WiFiClient EspWiFiclient;
PubSubClient client(EspWiFiclient);
ESP32PWM motorLeftPWM;
ESP32PWM motorRightPWM;
double midUSArr[] = { 0, 0, 0 };
double leftUSArr[] = { 0, 0, 0 };
double rightUSArr[] = { 0, 0, 0 };
double HandUSArr[] = { 0, 0, 0 };
double _medianMid = 0;
double _medianLeft = 0;
double _medianRight = 0;
double _medianHand = 0;
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
int bounce = 2;
volatile unsigned long lastLeftRec = 0;
volatile unsigned long lastRightRec = 0;
volatile bool canPingMid = true;
volatile bool canPingRight = true;
volatile bool canPingLeft = true;
double distanceMid = 0;
double distanceLeft = 0;
double distanceRight = 0;
double distanceHand = 0;
volatile bool newHandData = false;
volatile unsigned long microSecHand = 0;
volatile unsigned long microSecHandEndDuration = 0;
volatile bool canPingHand = true;
volatile bool canCollectDataHand = false;
volatile bool canCollectDataMid = false;
volatile bool canCollectDataLeft = false;
volatile bool canCollectDataRight = false;
unsigned long timeOfLastTrigger = millis();
unsigned long timeOfLastSendSensorData = millis();
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
double PWMLeftCoefficient = 1;
double PWMRightCoefficient = 1;
unsigned long speedTimer = 0;
unsigned long speedAdjustTimer = 0;
int LastSecondTicksLeft[] = { 0, 0, 0, 0, 0 };
int LastSecondTicksRight[] = { 0, 0, 0, 0, 0 };
int timeIntervalIndexCounter = 0;
int rightthing = 0;
long lastRight = 0;
int leftthing = 0;
long lastLeft = 0;
double mag_datat[] = { event.magnetic.x,
                       event.magnetic.y,
                       event.magnetic.z };
double lastDegKeepDir = 0;
double changeKeepDir = 0;
double degreeChangeFromStartKeepDir = 0;
bool stopSignal = false;
bool startingServoPosReached = false;
bool alreadySendDirSignal = false;
int remainningMapDataToTransmit = 0;
double leftVelocity = 0;
double rightVelocity = 0;
bool movingDirectionLeft = false;
bool movingDirectionRight = false;
bool autoMovementWantedDirLeftWheelIsForward = true;
bool autoMovementWantedDirRightWheelIsForward = true;

const double hard_iron[3] = {  // with magneto the values are new
  //-107.431847, 122.222755, 220.246822
  356.739494, -89.900846, -180.455162
};

const double soft_iron[3][3] = {
  // { 1.602628, -0.006533, -0.149331 },
  // { -0.006533, 1.813032, 0.170965 },
  // { -0.149331, 0.170965, 2.302615 }
  { 1.924747, 0.008350, 0.169482 },
  { 0.008350, 1.643996, 0.226782 },
  { 0.169482, 0.226782, 2.329259 }
};

void setup() {
  pinMode(signalOutputInterruptPinRight, INPUT);
  pinMode(signalOutputInterruptPinLeft, INPUT);
  pinMode(analogOutputLeftPin, OUTPUT);
  pinMode(analogOutputRightPin, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(trigPinHand, OUTPUT);
  pinMode(echoMidPin, INPUT);
  pinMode(echoPinHand, INPUT);
  pinMode(echoLeftPin, INPUT);
  pinMode(echoRightPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(echoMidPin), IRS_MidSensor, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echoPinHand), IRS_HandSensor, CHANGE);
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
void setPWMLeft(double duty_cycle) {  // 0.0 to 1.0
  if (duty_cycle >= 0.95) {
    motorLeftPWM.writeScaled(0.95);
  } else if (duty_cycle < 0) {
    motorLeftPWM.writeScaled(0);
  } else {
    motorLeftPWM.writeScaled(duty_cycle);
  }
}
void setPWMRight(double duty_cycle) {
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
void IRS_HandSensor() {
  if (digitalRead(echoPinHand) == 0x0) {
    if (canCollectDataHand && !canPingHand) {
      microSecHandEndDuration = micros() - microSecHand;
      canCollectDataHand = false;
      newHandData = true;
    }
  } else {
    if (canPingHand) {
      microSecHand = micros();
      canPingHand = false;
      canCollectDataHand = true;
    }
  }
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
    if (movingDirectionLeft) {
      leftEncoderCounter--;
    } else {
      leftEncoderCounter++;
    }
    lastLeftRec = millis();
  }
}
void IRS_RightEncoder() {
  if (millis() - lastRightRec >= bounce) {
    if (!movingDirectionRight) {
      rightEncoderCounter--;
    } else {
      rightEncoderCounter++;
    }
    lastRightRec = millis();
  }
}
unsigned long timeOfLastTriggerHand = 0;
void handtemp() {
  if (millis() - timeOfLastTriggerHand >= 40) {
    canPingHand = true;
    digitalWrite(trigPinHand, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinHand, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinHand, LOW);
    timeOfLastTriggerHand = millis();
  }
  if (newHandData) {
    newHandData = false;
    distanceHand = (microSecHandEndDuration * 0.0343) / 2;
    HandUSArr[0] = HandUSArr[1];
    HandUSArr[1] = HandUSArr[2];
    HandUSArr[2] = distanceHand;
    if (HandUSArr[2] < HandUSArr[1]) {
      if (HandUSArr[2] < HandUSArr[0]) {
        if (HandUSArr[1] < HandUSArr[0]) {
          _medianHand = HandUSArr[1];
        } else {
          _medianHand = HandUSArr[0];
        }
      } else {
        _medianHand = HandUSArr[2];
      }
    } else {
      if (HandUSArr[2] < HandUSArr[0]) {
        _medianHand = HandUSArr[2];
      } else {
        if (HandUSArr[1] < HandUSArr[0]) {
          _medianHand = HandUSArr[0];
        } else {
          _medianHand = HandUSArr[1];
        }
      }
    }
  }
  // publishJsonDataForMap(dir, weMoved, _medianLeft, _medianMid, _medianRight, useDataForMap);
}

void GetUltrasoundData(double dir, bool sendMove, bool useDataForMap, bool sendDataRegardlessOfMove) {
  //bool weSending_testingVariable__ = newRightData || newHandData || newMidData || newLeftData;
  if (millis() - timeOfLastTrigger >= 75) {
    canPingLeft = true;
    canPingMid = true;
    canPingRight = true;
    canPingHand = true;
    digitalWrite(trigPin, LOW);
    digitalWrite(trigPinHand, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    digitalWrite(trigPinHand, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    digitalWrite(trigPinHand, LOW);
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
  if (newHandData) {
    newHandData = false;
    distanceHand = (microSecHandEndDuration * 0.0343) / 2;
    HandUSArr[0] = HandUSArr[1];
    HandUSArr[1] = HandUSArr[2];
    HandUSArr[2] = distanceHand;
    if (HandUSArr[2] < HandUSArr[1]) {
      if (HandUSArr[2] < HandUSArr[0]) {
        if (HandUSArr[1] < HandUSArr[0]) {
          _medianHand = HandUSArr[1];
        } else {
          _medianHand = HandUSArr[0];
        }
      } else {
        _medianHand = HandUSArr[2];
      }
    } else {
      if (HandUSArr[2] < HandUSArr[0]) {
        _medianHand = HandUSArr[2];
      } else {
        if (HandUSArr[1] < HandUSArr[0]) {
          _medianHand = HandUSArr[0];
        } else {
          _medianHand = HandUSArr[1];
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

  if (sendMove && millis() - timeOfLastSendSensorData > 150) {
    int rigthCount = rightEncoderCounter;
    int leftCount = leftEncoderCounter;
    double rightDistanceMoved = (rigthCount - lastRightEncoderCounterUsedToCalculate) * MM_PER_TICK;
    double leftDistanceMoved = (leftCount - lastLeftEncoderCounterUsedToCalculate) * MM_PER_TICK;
    weMoved = (rightDistanceMoved + leftDistanceMoved) / 2.0;
    lastRightEncoderCounterUsedToCalculate = rigthCount;
    lastLeftEncoderCounterUsedToCalculate = leftCount;
    publishJsonDataForMap(dir, leftDistanceMoved, rightDistanceMoved, _medianLeft, _medianMid, _medianRight, _medianHand, useDataForMap);
    timeOfLastSendSensorData = millis();
  }
  // if ((((rightDistanceMoved > 0 || rightDistanceMoved < 0) || (leftDistanceMoved > 0 || leftDistanceMoved < 0)) && sendMove) || sendDataRegardlessOfMove) {
  //}
}
double MagneticSensorReading() {  // NEEDS TO BE REVERTED LATER
  mag.getEvent(&event);
  double hi_cal[3];
  double mag_data[] = { event.magnetic.x * 10 * 1.3,
                        event.magnetic.y * 10 * 1.3,
                        event.magnetic.z * 10 * 1.3 };
  for (uint8_t i = 0; i < 3; i++) {
    hi_cal[i] = mag_data[i] - hard_iron[i];
  }
  for (uint8_t i = 0; i < 3; i++) {
    mag_data[i] = (soft_iron[i][0] * hi_cal[0]) + (soft_iron[i][1] * hi_cal[1]) + (soft_iron[i][2] * hi_cal[2]);
  }
  double heading = atan2(mag_data[0], mag_data[1]);
  if (heading < 0)
    heading += 2 * PI;
  if (heading > 2 * PI)
    heading -= 2 * PI;
  double headingDegrees = heading * 180 / M_PI;


  return headingDegrees;
}
void MagneticSensorReadingFORPROCESSING() {  // NEEDS TO BE REVERTED LATER
  mag.getEvent(&event);
  double hi_cal[3];
  mag_datat[0] = event.magnetic.x * 10 * 1.3;
  mag_datat[1] = event.magnetic.y * 10 * 1.3;
  mag_datat[2] = event.magnetic.z * 10 * 1.3;
}
double PidControllerSpeedLeft(double target, double kp, double current) {
  double e = current - target;
  double u = (kp * e);
  return u * -1;
}
double PidControllerSpeedRight(double target, double kp, double current) {
  double e = current - target;
  double u = (kp * e);
  return u * -1;
}
void ManualMovement(String signal) {
  ResetEncoderValues();
  if (signal == "w" || signal == "W") {
    
    justForward(false);
  } else if (signal == "a" || signal == "A") {
    justLeftRight(1);
  } else if (signal == "s" || signal == "S") {

    justForward(true);
  } else if (signal == "d" || signal == "D") {
    justLeftRight(-1);
  } else if (signal == "none" || signal == "None") {
    StopMovement();
  }
}
void justLeftRight(int direction) {
  speedTimer = millis();
  speedAdjustTimer = millis();
  turnedRight = false;
  turnedLeft = false;
  alreadySendDirSignal = false;
  rightthing = 0;
  lastRight = 0;
  PWMLeftCoefficient = 1;
  PWMRightCoefficient = 1;
  while (client.connected() && !stopSignal) {
    CheckWiFiConnection();
    client.loop();

    if (millis() - speedTimer >= millisecToRecordTicksInterval) {  // update the speed count o feach wheel every X seconds. In this case 200ms so the array of 5 records is the speed from last second
      if (5 <= timeIntervalIndexCounter) {
        timeIntervalIndexCounter = 0;
      }
      UpdateTicksRight();
      UpdateTicksLeft();
      timeIntervalIndexCounter++;
      speedTimer = millis();
    }

    GetUltrasoundData(MagneticSensorReading(), true, true, false);
    if (direction > 0) {
      if (!turnedLeft) {
        startingServoPosReached = false;
        AdjustPosTo(35, true);
        turnedLeft = true;
        goingForward = false;
        turnedRight = false;
      } else if (startingServoPosReached && movingDirectionLeft == false && movingDirectionRight == false) {
        if (millis() - speedAdjustTimer >= 75) {  // Minimum time between pwm changes
          double changeLEft = PidControllerSpeedLeft(1.5, 0.045, GetCurrentSpeedLeft());
          double changeRight = PidControllerSpeedRight(1.5, 0.045, GetCurrentSpeedRight());  // IMPORTANT the signs will likely be reversed so if it refuses to go try reversing them aka PID returns - when it should be +

          if (fabs(changeLEft) > 0.0001) {
            if (PWMLeftCoefficient + changeLEft < 9 && PWMLeftCoefficient + changeLEft > 0.1) {
              PWMLeftCoefficient += changeLEft;
            }
          }
          if (fabs(changeRight) > 0.0001) {
            if (PWMRightCoefficient + changeRight < 9 && PWMRightCoefficient + changeRight > 0.1) {
              PWMRightCoefficient += changeRight;
            }
          }
          if (millis() - previousTimeThereWasAnObstacle <= 250) {
            StopMovement();
          } else {
            setPWMRight(0.1 * PWMRightCoefficient);
            //setPWMLeft(0.1 * PWMLeftCoefficient);
            setPWMLeft(0);
            //setPWMRight(1);
          }
          speedAdjustTimer = millis();
        }

      } else if (!alreadySendDirSignal) {
        setPWMRight(0);
        setPWMLeft(0);
        SendDirSignal(false, LEFTDIRWHEEL);
        SendDirSignal(false, RIGHTDIRWHEEL);
        alreadySendDirSignal = true;
      }
    } else {
      if (!turnedRight) {
        startingServoPosReached = false;
        AdjustPosTo(145, true);
        turnedLeft = false;
        goingForward = false;
        turnedRight = true;
      } else if (startingServoPosReached && movingDirectionLeft == true && movingDirectionRight == true) {
        if (millis() - speedAdjustTimer >= 75) {  // Minimum time between pwm changes
          double changeLEft = PidControllerSpeedLeft(1.5, 0.045, GetCurrentSpeedLeft());
          double changeRight = PidControllerSpeedRight(1.5, 0.045, GetCurrentSpeedRight());  // IMPORTANT the signs will likely be reversed so if it refuses to go try reversing them aka PID returns - when it should be +

          if (fabs(changeLEft) > 0.0001) {
            if (PWMLeftCoefficient + changeLEft < 9 && PWMLeftCoefficient + changeLEft > 0.1) {
              PWMLeftCoefficient += changeLEft;
            }
          }
          if (fabs(changeRight) > 0.0001) {
            if (PWMRightCoefficient + changeRight < 9 && PWMRightCoefficient + changeRight > 0.1) {
              PWMRightCoefficient += changeRight;
            }
          }
          if (millis() - previousTimeThereWasAnObstacle <= 250) {
            StopMovement();
          } else {
            //setPWMRight(0.1 * PWMRightCoefficient);
            setPWMLeft(0.1 * PWMLeftCoefficient);
            setPWMRight(0);
            //setPWMLeft(0);
            //setPWMRight(1);
          }
          speedAdjustTimer = millis();
        }
      } else if (!alreadySendDirSignal) {
        setPWMRight(0);
        setPWMLeft(0);
        SendDirSignal(true, LEFTDIRWHEEL);
        SendDirSignal(true, RIGHTDIRWHEEL);
        alreadySendDirSignal = true;
      }
    }

    // GetUltrasoundData(MagneticSensorReading(), true, true, true);
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
  rightthing = abs(rightEncoderCounter) - lastRight;
  lastRight = abs(rightEncoderCounter);
  LastSecondTicksRight[timeIntervalIndexCounter] = rightthing;
}
void UpdateTicksLeft() {
  leftthing = abs(leftEncoderCounter) - lastLeft;
  lastLeft = abs(leftEncoderCounter);
  LastSecondTicksLeft[timeIntervalIndexCounter] = leftthing;
}
double GetCurrentSpeedRight() {
  return (LastSecondTicksRight[0] + LastSecondTicksRight[1] + LastSecondTicksRight[2] + LastSecondTicksRight[3] + LastSecondTicksRight[4]) * MM_PER_TICK * 0.0036;
}
double GetCurrentSpeedLeft() {
  return (LastSecondTicksLeft[0] + LastSecondTicksLeft[1] + LastSecondTicksLeft[2] + LastSecondTicksLeft[3] + LastSecondTicksLeft[4]) * MM_PER_TICK * 0.0036;
}
void ResetKeepDir() {
  lastDegKeepDir = 0;
  changeKeepDir = 0;
  degreeChangeFromStartKeepDir = 0;
}
void keepDirection() {
  double currentDeg = MagneticSensorReading();  //0;
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
  client.publish(publishTopicServoControl, (const uint8_t*)jsonBuffer, strlen(jsonBuffer), false);
}
void justForward(bool dir) {  //true for forward
  speedTimer = millis();
  speedAdjustTimer = millis();
  alreadySendDirSignal = false;
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

    startingServoPosReached = true;
    goingForward = true;
    turnedLeft = false;
    turnedRight = false;
    if (!goingForward) {
      startingServoPosReached = false;
      AdjustPosTo(90, true);

    } else if (startingServoPosReached && movingDirectionLeft == dir && movingDirectionRight == !dir) {

      GetUltrasoundData(MagneticSensorReading(), true, true, false);

      if (millis() - speedTimer >= millisecToRecordTicksInterval) {  // update the speed count o feach wheel every X seconds. In this case 200ms so the array of 5 records is the speed from last second
        if (5 <= timeIntervalIndexCounter) {
          timeIntervalIndexCounter = 0;
        }
        UpdateTicksRight();
        UpdateTicksLeft();
        timeIntervalIndexCounter++;
        speedTimer = millis();
      }

      if (millis() - speedAdjustTimer >= 75) {  // Minimum time between pwm changes
        double changeLEft = PidControllerSpeedLeft(2.5, 0.025, GetCurrentSpeedLeft());
        double changeRight = PidControllerSpeedRight(2.5, 0.025, GetCurrentSpeedRight());  // IMPORTANT the signs will likely be reversed so if it refuses to go try reversing them aka PID returns - when it should be +

        if (fabs(changeLEft) > 0.0001) {
          if (PWMLeftCoefficient + changeLEft < 9 && PWMLeftCoefficient + changeLEft > 0.1) {
            PWMLeftCoefficient += changeLEft;
          }
        }
        if (fabs(changeRight) > 0.0001) {
          if (PWMRightCoefficient + changeRight < 9 && PWMRightCoefficient + changeRight > 0.1) {
            PWMRightCoefficient += changeRight;
          }
        }
        if (millis() - previousTimeThereWasAnObstacle <= 250) {
          StopMovement();
        } else {
          setPWMRight(0.1 * PWMRightCoefficient);
          setPWMLeft(0.1 * PWMLeftCoefficient);
          keepDirection();
        }
        speedAdjustTimer = millis();
      }

    } else if (!alreadySendDirSignal) {
      setPWMRight(0);
      setPWMLeft(0);
      SendDirSignal(dir, LEFTDIRWHEEL);
      SendDirSignal(!dir, RIGHTDIRWHEEL);
      alreadySendDirSignal = true;
    }
  }
  StopMovement();
  goingForward = false;
  turnedLeft = false;
  turnedRight = false;
}
void SendDirSignal(bool signal, int whichOneToSwitchDir) {
  StaticJsonDocument<200> jsonDoc;
  if (whichOneToSwitchDir == LEFTDIRWHEEL) {
    jsonDoc["dirLeft"] = signal;
  }
  if (whichOneToSwitchDir == RIGHTDIRWHEEL) {
    jsonDoc["dirRight"] = signal;
  }
  char jsonBuffer[256];
  serializeJson(jsonDoc, jsonBuffer);
  client.publish(publishWantedDirChange, (const uint8_t*)jsonBuffer, strlen(jsonBuffer), false);
  delay(150);
}
int CalcDirectionFrontServoFromSpeeds() {
  return 90 + (leftVelocity - rightVelocity) * 20;
  // this is not concrete so no idea if it will work remotely as wanted
}
void autoMovement() {
  speedTimer = millis();
  speedAdjustTimer = millis();
  rightthing = 0;
  lastRight = 0;
  PWMLeftCoefficient = 1;
  PWMRightCoefficient = 1;
  alreadySendDirSignal = false;
  goingForward = false;
  while (client.connected() && !stopSignal) {
    CheckWiFiConnection();
    client.loop();
    /*if (!goingForward) {
      startingServoPosReached = false;
      AdjustPosTo(CalcDirectionFrontServoFromSpeeds(), true);
      goingForward = true;
      turnedLeft = false;
      turnedRight = false;
    } else */
    startingServoPosReached = true;
    if (startingServoPosReached && movingDirectionLeft == autoMovementWantedDirLeftWheelIsForward && movingDirectionRight == !autoMovementWantedDirRightWheelIsForward) {
      GetUltrasoundData(MagneticSensorReading(), true, false, false);
      if (millis() - speedTimer >= millisecToRecordTicksInterval) {  // update the speed count o feach wheel every X seconds. In this case 200ms so the array of 5 records is the speed from last second
        if (5 <= timeIntervalIndexCounter) {
          timeIntervalIndexCounter = 0;
        }
        UpdateTicksRight();
        UpdateTicksLeft();
        timeIntervalIndexCounter++;
        speedTimer = millis();
      }

      if (millis() - speedAdjustTimer >= 75) {  // Minimum time between pwm changes
        double changeLEft = PidControllerSpeedLeft(2 * leftVelocity, 0.0065, GetCurrentSpeedLeft());
        double changeRight = PidControllerSpeedRight(2 * rightVelocity, 0.0065, GetCurrentSpeedRight());  // IMPORTANT the signs will likely be reversed so if it refuses to go try reversing them aka PID returns - when it should be +

        if (fabs(changeLEft) > 0.0001) {
          if (PWMLeftCoefficient + changeLEft < 7 && PWMLeftCoefficient + changeLEft > 0.01) {
            PWMLeftCoefficient += changeLEft;
          }
        }
        if (fabs(changeRight) > 0.0001) {
          if (PWMRightCoefficient + changeRight < 7 && PWMRightCoefficient + changeRight > 0.01) {
            PWMRightCoefficient += changeRight;
          }
        }
        if (millis() - previousTimeThereWasAnObstacle <= 250) {
          StopMovement();
        } else {
          AdjustPosTo(CalcDirectionFrontServoFromSpeeds(), false);
          if (PWMLeftCoefficient < 0.1) {
            setPWMLeft(0);
          }
          if (PWMRightCoefficient < 0.1) {
            setPWMRight(0);
          }
          setPWMRight(0.1 * PWMRightCoefficient);
          setPWMLeft(0.1 * PWMLeftCoefficient);
        }
        speedAdjustTimer = millis();
      }

    } else if (!alreadySendDirSignal) {
      setPWMRight(0);
      setPWMLeft(0);
      alreadySendDirSignal = true;
      SendDirSignal(autoMovementWantedDirLeftWheelIsForward, LEFTDIRWHEEL);
      SendDirSignal(!autoMovementWantedDirRightWheelIsForward, RIGHTDIRWHEEL);
    }
  }
  StopMovement();
  goingForward = false;
  turnedLeft = false;
  turnedRight = false;




  // make it apply these velocity coefficients to both wheels. This way we are not yet changing directions which isn't the best case as switching directions will be better in times where we need big turns.
  //(the velocity coefficient maybe can be applied to the speed we are aiming for? so a coeff of 1 will be a speed of max 3km/h) For example if we have coeff of 0.8 on left and 0.3 on right then we are turning right in a curved path
  // the frond servo will likely change based on the difference of volocities as that signifies
  // There is the option where we do the direction change only when starting the movement sequence in which case we will need another method.. we can use the manual method for left/right
}
void publishJsonDataForMap(double direction, double leftDistance, double rightDistance, double left, double mid, double right, double hand, bool useDataForMap) {
  StaticJsonDocument<300> jsonDoc;
  jsonDoc["direction"] = direction;
  jsonDoc["leftMovement"] = leftDistance;
  jsonDoc["rightMovement"] = rightDistance;
  jsonDoc["leftSensor"] = left;
  jsonDoc["midSensor"] = mid;
  jsonDoc["rightSensor"] = right;
  jsonDoc["handSensor"] = hand;
  if (useDataForMap) {
    jsonDoc["mappingFlag"] = 1;
  } else {
    jsonDoc["mappingFlag"] = 0;
  }
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
    // forward(moveDistance);
  }
  if (jsonDoc.containsKey("turn")) {
    double Turn = jsonDoc["turn"];
    //  turn(Turn);
  }
  if (jsonDoc.containsKey("requestMapData")) {
    remainningMapDataToTransmit = jsonDoc["requestMapData"];
    for (; remainningMapDataToTransmit > 0; remainningMapDataToTransmit--) {
      //GetUltrasoundData(MagneticSensorReading(), true, false, true);
      GetUltrasoundData(MagneticSensorReading(), false, false, true);
    }
  }
  if (jsonDoc.containsKey("manualCommand")) {
    String signal = jsonDoc["manualCommand"];
    ManualMovement(signal);
  }
  if (jsonDoc.containsKey("leftVelocity")) {
    double leftVelo = jsonDoc["leftVelocity"];
    double rightVelo = jsonDoc["rightVelocity"];
    int isFirstInstance = jsonDoc["firstInstance"];
    if (leftVelo < 0) {
      autoMovementWantedDirLeftWheelIsForward = false;
    } else {
      autoMovementWantedDirLeftWheelIsForward = true;
    }
    if (rightVelo < 0) {
      autoMovementWantedDirRightWheelIsForward = false;
    } else {
      autoMovementWantedDirRightWheelIsForward = true;
    }
    leftVelocity = fabs(leftVelo);
    rightVelocity = fabs(rightVelo);
    if (isFirstInstance == 1) {
      autoMovement();
    }
  }

  if (jsonDoc.containsKey("wantedDirLeftReached")) {
    String tempAnswer = jsonDoc["wantedDirLeftReached"];
    if (tempAnswer == "true") {
      movingDirectionLeft = true;
    } else {
      movingDirectionLeft = false;
    }
  }
  if (jsonDoc.containsKey("wantedDirRightReached")) {
    String tempAnswer = jsonDoc["wantedDirRightReached"];
    if (tempAnswer == "true") {
      movingDirectionRight = true;
    } else {
      movingDirectionRight = false;
    }
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
      client.subscribe(supDirTemp, 1);
      client.subscribe(subTopicAutoMove, 1);
      client.subscribe(subMapRequestData, 1);
    } else {
      delay(2000);
    }
  }
}
void CheckWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      //Serial.println("not connected");
      // Serial.println(ssid);
      // Serial.println(password);
      // Serial.println("----------");
      delay(1000);
    }
  }
}

void loop() {

  CheckConnections();
  client.loop();
  delay(5);

  // GetUltrasoundData(0, false, false, true);
}
