#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <ESP32Servo.h>
//#include "driver/mcpwm.h"

#define signalOutputInterruptPinRight 23
#define signalOutputInterruptPinLeft 5
#define analogOutputLeftPin 25
#define analogOutputRightPin 12
#define trigPin 32
#define echoMidPin 17
#define echoLeftPin 19   // TO BE UPDATED
#define echoRightPin 18  // TO BE UPDATED
#define servoPin 27
//#define dirLeftPin 16   // TO BE UPDATED
//#define dirRightPin 3  // TO BE UPDATED

#define WHEEL_DIAM 203.2
//#define distance_Wheels 140   // TO BE UPDATED?
#define TICKS_PER_REV 42
#define MM_PER_TICK (((WHEEL_DIAM * PI) / TICKS_PER_REV))
#define MM_TO_TICKS(A) ((float)(A)) / MM_PER_TICK
//#define MM_PER_DEGREES(D) ((distance_Wheels * PI * (D)) / 360.0f)  // TO BE UPDATED?
//#define turnDegr(M) ((M) / (PI * distance_Wheels * 360.0f)) // TO BE UPDATED?
#define millisecToRecordTicksInterval 200
#define UNDEFINED -999
ESP32PWM motorLeftPWM;
ESP32PWM motorRightPWM;
double midUSArr[] = { 0, 0, 0 };
double leftUSArr[] = { 0, 0, 0 };
double rightUSArr[] = { 0, 0, 0 };
double _medianMid = 0;
double _medianLeft;
double _medianRight;
WiFiClient client;
volatile int leftEncoderCounter = 0;
volatile int rightEncoderCounter = 0;
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
int lastRightEncoderCounterUsedToCalculate = 0;
int lastLeftEncoderCounterUsedToCalculate = 0;
Adafruit_HMC5883_Unified mag;
sensors_event_t event;
unsigned long pid_previousTimeTurn = 0;
double pid_ePreviousTurn = 0;
double pid_eintegralTurn = 0;
unsigned long pid_previousTimeAdj = 0;
double pid_ePreviousAdj = 0;
double pid_eintegralAdj = 0;
unsigned long pid_previousTimeAdjAlt = 0;
double pid_ePreviousAdjAlt = 0;
double pid_eintegralAdjAlt = 0;
unsigned long pid_previousTime = 0;
double pid_ePrevious = 0;
double pid_eintegral = 0;
unsigned long pid_previousTimeSpeedLeft = 0;
double pid_ePreviousSpeedLeft = 0;
double pid_eintegralSpeedLeft = 0;
unsigned long pid_previousTimeSpeedRight = 0;
double pid_ePreviousSpeedRight = 0;
double pid_eintegralSpeedRight = 0;
String keyInpt = "";
bool leftGoingForward = false;
bool rightGoingForward = false;
const uint16_t port = 13000;
const char* host = "192.168.43.144";
int pwmValueInt = 0.7;  // this is for duty cycle its from 0 to 1
Servo myservo;
int servoChannel;
int pos = 90;
float speedLeftPWM = 0;
float speedRightPWM = 0;
unsigned long previousTimeThereWasAnObstacle = millis();
bool goingForward = false;
bool turnedLeft = false;
bool turnedRight = false;
bool currentGoingForwardDir = true;
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
  pinMode(echoLeftPin, INPUT);  //temporary
  pinMode(echoRightPin, INPUT);
  // pinMode(dirLeftPin, OUTPUT);
  // pinMode(dirRightPin, OUTPUT);
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
  motorLeftPWM.attachPin(analogOutputLeftPin, 3000, 10);
  motorRightPWM.attachPin(analogOutputRightPin, 3000, 10);
  myservo.setPeriodHertz(50);  // standard 50 hz servo
  servoChannel = myservo.attach(servoPin, 1000, 2000);
  Serial.begin(115200);
  if (!mag.begin()) {
    while (1) { delay(10); }
  }
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
}
void setPWMLeft(float duty_cycle) {  // 0.0 to 1.0
  if (duty_cycle >= 0.75) {
    motorLeftPWM.writeScaled(0.75);
  } else if (duty_cycle < 0) {
    motorLeftPWM.writeScaled(0);
  } else {
    motorLeftPWM.writeScaled(duty_cycle);
  }
}
void setPWMRight(float duty_cycle) {
  if (duty_cycle >= 0.75) {
    motorRightPWM.writeScaled(0.75);
  } else if (duty_cycle < 0) {
    motorRightPWM.writeScaled(0);
  } else {
    motorRightPWM.writeScaled(duty_cycle);
  }
}
void ResetPIDs() {
  pid_previousTimeAdj = 0;
  pid_ePreviousAdj = 0;
  pid_eintegralAdj = 0;
  pid_previousTimeAdjAlt = 0;
  pid_ePreviousAdjAlt = 0;
  pid_eintegralAdjAlt = 0;
  pid_previousTime = 0;
  pid_ePrevious = 0;
  pid_eintegral = 0;
  rightEncoderCounter = 0;
  leftEncoderCounter = 0;
  lastRightEncoderCounterUsedToCalculate = 0;
  lastLeftEncoderCounterUsedToCalculate = 0;
  pid_previousTimeTurn = 0;
  pid_ePreviousTurn = 0;
  pid_eintegralTurn = 0;
  pid_previousTimeSpeedLeft = 0;
  pid_ePreviousSpeedLeft = 0;
  pid_eintegralSpeedLeft = 0;
  pid_previousTimeSpeedRight = 0;
  pid_ePreviousSpeedRight = 0;
  pid_eintegralSpeedRight = 0;
  speedLeftPWM = 0;
  speedRightPWM = 0;
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
  //if (!currentGoingForwardDir) {
  //   leftEncoderCounter--;
  // } else {
  if (millis() - lastLeftRec >= bounce) {
    leftEncoderCounter++;
    lastLeftRec = millis();
  }
  //}
}
void IRS_RightEncoder() {
  //if (!currentGoingForwardDir) {
  // rightEncoderCounter--;
  // } else {
  if (millis() - lastRightRec >= bounce) {
    rightEncoderCounter++;
    lastRightRec = millis();
  }
  // }
}
void GetUltrasoundData(float dir, bool sendMove) {
  int rigthCount = rightEncoderCounter;
  int leftCount = leftEncoderCounter;
  double rightDistanceMoved = (rigthCount - lastRightEncoderCounterUsedToCalculate) * MM_PER_TICK;
  double leftDistanceMoved = (leftCount - lastLeftEncoderCounterUsedToCalculate) * MM_PER_TICK;
  weMoved = (rightDistanceMoved + leftDistanceMoved) / 2.0;
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
  String str = "mapPoint|";
  str.concat(weMoved);
  str.concat("|");
  str.concat(dir);
  str.concat("|");
  str.concat(0);
  str.concat("|");
  str.concat(_medianMid);
  // str.concat(rightDistanceMoved);
  str.concat("|");
  str.concat(90);
  str.concat("|");
  str.concat(_medianLeft);
  // str.concat(leftDistanceMoved);
  str.concat("|");
  str.concat(-90);
  str.concat("|");
  str.concat(_medianRight);
  str.concat("|");
  str.concat('`');
  if (sendMove) {  //&& weMoved > 0) {
    client.print(str);
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
  // float heading = (atan2(mag_data[0], mag_data[1]) * 180) / M_PI;
  float headingDegrees = heading * 180 / M_PI;
  return headingDegrees;
}
float mag_datat[] = { event.magnetic.x,
                      event.magnetic.y,
                      event.magnetic.z };
void MagneticSensorReadingFORPROCESSING() {  // NEEDS TO BE REVERTED LATER
  mag.getEvent(&event);
  float hi_cal[3];
  mag_datat[0] = event.magnetic.x * 10 * 1.3;
  mag_datat[1] = event.magnetic.y * 10 * 1.3;
  mag_datat[2] = event.magnetic.z * 10 * 1.3;

  // for (uint8_t i = 0; i < 3; i++) {
  //  hi_cal[i] = mag_datat[i] - hard_iron[i];
  // }
  // for (uint8_t i = 0; i < 3; i++) {
  //   mag_datat[i] = (soft_iron[i][0] * hi_cal[0]) + (soft_iron[i][1] * hi_cal[1]) + (soft_iron[i][2] * hi_cal[2]);
  //}
}
float PidController_straightForward_adjust(float targetDegree, float kp, float kd, float ki, float currentDegree) {
  unsigned long currentTime = micros();
  float deltaT = ((float)(currentTime - pid_previousTimeAdj)) / 1.0e6;
  float e = currentDegree - targetDegree;
  float eDerivative = (e - pid_ePreviousAdj) / deltaT;
  pid_eintegralAdj = pid_eintegralAdj + e * deltaT;

  float u = (kp * e) + (kd * eDerivative) + (ki * pid_eintegralAdj);
  pid_previousTimeAdj = currentTime;
  pid_ePreviousAdj = e;
  return u;
}
float PidController_straightForward_adjust_alternative(volatile int* countLeft, float kp, float kd, float ki, volatile int* countRight) {
  unsigned long currentTime = micros();
  float deltaT = ((float)(currentTime - pid_previousTimeAdjAlt)) / 1.0e6;

  float e = countRight - countLeft;
  float eDerivative = (e - pid_ePreviousAdjAlt) / deltaT;
  pid_eintegralAdjAlt = pid_eintegralAdjAlt + e * deltaT;

  float u = (kp * e) + (kd * eDerivative) + (ki * pid_eintegralAdjAlt);
  pid_previousTimeAdjAlt = currentTime;
  pid_ePreviousAdjAlt = e;
  return u;
}
float PidController(float target, float kp, float kd, float ki, float moved) {
  unsigned long currentTime = micros();
  float deltaT = ((float)(currentTime - pid_previousTime)) / 1.0e6;
  float e = moved - target;
  float eDerivative = (e - pid_ePrevious) / deltaT;
  pid_eintegral = pid_eintegral + e * deltaT;

  float baseSpeed = 120;  //giving base engine speed

  float u = (kp * e) + (kd * eDerivative) + (ki * pid_eintegral);
  if (u < 0) {
    u = u - baseSpeed;
  } else {
    u = u + baseSpeed;
  }
  pid_previousTime = currentTime;
  pid_ePrevious = e;

  return u;
}
float PidControllerSpeedLeft(float target, float kp, float kd, float ki, float current) {
  unsigned long currentTime = micros();
  float deltaT = ((float)(currentTime - pid_previousTimeSpeedLeft)) / 1.0e6;
  float e = current - target;
  float eDerivative = (e - pid_ePreviousSpeedLeft) / deltaT;
  pid_eintegralSpeedLeft = pid_eintegralSpeedLeft + e * deltaT;

  float u = (kp * e) + (kd * eDerivative) + (ki * pid_eintegralSpeedLeft);

  pid_previousTimeSpeedLeft = currentTime;
  pid_ePreviousSpeedLeft = e;

  return u * -1;
}
float PidControllerSpeedRight(float target, float kp, float kd, float ki, float current) {
  unsigned long currentTime = micros();
  float deltaT = ((float)(currentTime - pid_previousTimeSpeedRight)) / 1.0e6;
  float e = current - target;
  float eDerivative = (e - pid_ePreviousSpeedRight) / deltaT;
  pid_eintegralSpeedRight = pid_eintegralSpeedRight + e * deltaT;

  float u = (kp * e) + (kd * eDerivative) + (ki * pid_eintegralSpeedRight);

  pid_previousTimeSpeedRight = currentTime;
  pid_ePreviousSpeedRight = e;

  return u * -1;
}
float PidController_turning(float targetDegree, float kp, float kd, float ki, float currentDegree) {  // get a pid value u  then turn the front servo by u degrees or something while also reducing the left or right wheel by u pwm down to some value or 0 if below
  unsigned long currentTime = micros();
  float deltaT = ((float)(currentTime - pid_previousTimeTurn)) / 1.0e6;
  float e = currentDegree - targetDegree;
  float eDerivative = (e - pid_ePreviousTurn) / deltaT;
  pid_eintegralTurn = pid_eintegralTurn + e * deltaT;

  float u = (kp * e) + (kd * eDerivative) + (ki * pid_eintegralTurn);
  pid_previousTimeTurn = currentTime;
  pid_ePreviousTurn = e;
  return u;
}
float findOppositeSide(float adjacent, float theta) {
  // Convert angle from degrees to radians
  float thetaRad = theta * PI / 180.0;
  // Calculate the opposite side
  float opposite = adjacent * tan(thetaRad);
  return opposite;
}
void smurfMovement(String signal) {
  keyInpt = "";
  ResetPIDs();
  if (signal == "w" || signal == "W") {
    //if (!currentGoingForwardDir) {
    //  ReverseDirection();
    // }
    justForward();
  } else if (signal == "a" || signal == "A") {
    justLeftRight(1);
  } else if (signal == "s" || signal == "S") {
    setPWMLeft(0);
    setPWMRight(0);
  } else if (signal == "d" || signal == "D") {
    justLeftRight(-1);
  } else if (signal == "none" || signal == "None") {
    setPWMLeft(0);
    setPWMRight(0);
  }
}
void spin() {
  for (int i = 30; i <= 150; i += 3) {
    pos = i;
    myservo.write(pos);
    delay(100);
  }
  for (int i = 150; i >= 30; i -= 3) {
    pos = i;
    myservo.write(pos);
    delay(100);
  }
}
void justLeftRight(int direction) {
  // ResetPIDs();
  while (keyInpt != "None") {
    if (client.available() > 0) {
      String temp = client.readStringUntil('~');
      keyInpt = client.readStringUntil('~');
      if (temp == "pwm") {
        pwmValueInt = keyInpt.toInt();
      }
    }
    GetUltrasoundData(MagneticSensorReading(), true);
    if (direction > 0) {
      if (!turnedLeft) {
        pos = 30;
        SetServoAngle();
        delay(100);
        turnedLeft = true;
        goingForward = false;
        turnedRight = false;
      }
      setPWMLeft(0);
      setPWMRight(0.75);
    } else {
      if (!turnedRight) {
        pos = 150;
        SetServoAngle();
        delay(100);
        turnedLeft = false;
        goingForward = false;
        turnedRight = true;
      }
      setPWMLeft(0.75);
      setPWMRight(0);
    }
  }
  setPWMLeft(0);
  setPWMRight(0);
  goingForward = false;
  turnedLeft = false;
  turnedRight = false;
}
void SetServoAngle() {
  if (pos >= 30 && pos <= 150) {
    myservo.write(pos);  // tell servo to go to position in variable 'pos'
    delay(25);
  }
}
void CollectAndSendMagDataForCalibration() {
  while (true) {
    //MagneticSensorReadingFORPROCESSING();
    String str = "calib|";
    //str.concat(mag_datat[0]);
    // str.concat("|");
    // str.concat(mag_datat[1]);
    // str.concat("|");
    // str.concat(mag_datat[2]);
    str.concat(MagneticSensorReading());
    str.concat('`');
    client.print(str);
    delay(150);
  }
}

unsigned long speedTimer = 0;
unsigned long speedAdjustTimer = 0;
int rotationCounterForSpeedRight = 0;
int rotationCounterForSpeedLeft = 0;
float lastSpeedRight = 0;
float lastSpeedLeft = 0;
int LastSecondTicksLeft[] = { 0, 0, 0, 0, 0 };
int LastSecondTicksRight[] = { 0, 0, 0, 0, 0 };
int timeIntervalIndexCounter = 0;
void UpdateTicksRight() {
  rotationCounterForSpeedRight = abs(rightEncoderCounter) - rotationCounterForSpeedRight;
  LastSecondTicksRight[timeIntervalIndexCounter] = rotationCounterForSpeedRight;
  // lastSpeedRight = UNDEFINED;  // TODO MATH
}
void UpdateTicksLeft() {
  rotationCounterForSpeedLeft = abs(leftEncoderCounter) - rotationCounterForSpeedLeft;
  LastSecondTicksLeft[timeIntervalIndexCounter] = rotationCounterForSpeedLeft;
  // lastSpeedLeft = UNDEFINED;  // TODO MATH
}
float GetCurrentSpeedRight() {
  return (LastSecondTicksRight[0] + LastSecondTicksRight[1] + LastSecondTicksRight[2] + LastSecondTicksRight[3] + LastSecondTicksRight[4]) * MM_PER_TICK * 0.0036;
}
float GetCurrentSpeedLeft() {
  return (LastSecondTicksLeft[0] + LastSecondTicksLeft[1] + LastSecondTicksLeft[2] + LastSecondTicksLeft[3] + LastSecondTicksLeft[4]) * MM_PER_TICK * 0.0036;
}

void justForward() {
  speedTimer = millis();
  speedAdjustTimer = millis();
  rotationCounterForSpeedRight = 0;
  rotationCounterForSpeedLeft = 0;
  // ResetPIDs();
  while (keyInpt != "None") {
    if (client.available() > 0) {
      String temp = client.readStringUntil('~');
      keyInpt = client.readStringUntil('~');
      if (temp == "pwm") {
        pwmValueInt = keyInpt.toInt();
      }
    }
    if (!goingForward) {
      pos = 90;
      SetServoAngle();
      delay(100);
      goingForward = true;
      turnedLeft = false;
      turnedRight = false;
    }
    /*if (millis() - speedTimer >= millisecToRecordTicksInterval) {
      if (5 <= timeIntervalIndexCounter) {
        timeIntervalIndexCounter = 0;
      }
      UpdateTicksRight();
      UpdateTicksLeft();
      timeIntervalIndexCounter++;
      speedTimer = millis();
    }

    if (millis() - speedAdjustTimer >= 75) {
      float changeLEft = PidControllerSpeedLeft(3, 0.06, 0.002, 0, GetCurrentSpeedLeft());

      float changeRight = PidControllerSpeedRight(3, 0.06, 0.002, 0, GetCurrentSpeedRight());

      //+= PidControllerSpeedLeft(3, 0.06, 0.002, 0, GetCurrentSpeedLeft());  //will look someting like that when called   Can tinker with setPWMLeft(nnn) inside same logic for right side
      if (changeLEft >= 0.2) {
        changeLEft = 0.2;
      } else if (changeLEft <= -0.2) {
        changeLEft = -0.2;
      }
      speedLeftPWM += changeLEft;
      if (speedLeftPWM > 0.75) {
        speedLeftPWM = 0.75;
      } else if (speedLeftPWM < 0) {
        speedLeftPWM = 0;
      }
      //+= PidControllerSpeedRight(3, 0.06, 0.002, 0, GetCurrentSpeedRight());
      if (changeRight >= 0.2) {
        changeRight = 0.2;
      } else if (changeRight <= -0.2) {
        changeRight = -0.2;
      }
      speedRightPWM += changeRight;
      if (speedRightPWM > 0.75) {
        speedRightPWM = 0.75;
      } else if (speedRightPWM < 0) {
        speedRightPWM = 0;
      }
      speedAdjustTimer = millis();
    }*/
    //setPWMLeft(speedLeftPWM);
    // setPWMRight(speedRightPWM);
    //  Serial.println("Left and right: ");
    // Serial.println(speedLeftPWM);
    // Serial.println(speedRightPWM);
    // setPWMLeft(0.55);
    // setPWMRight(0.55);
    GetUltrasoundData(MagneticSensorReading(), true);

    if (millis() - previousTimeThereWasAnObstacle <= 250) {  // when this works correctly? do the PID if there is no collision danger
      setPWMLeft(0);
      setPWMRight(0);
    } else {
      setPWMLeft(0.7);  // 0 to 1
      setPWMRight(0.7);
    }
  }
  setPWMLeft(0);
  setPWMRight(0);
  goingForward = false;
  turnedLeft = false;
  turnedRight = false;
}
void forward(int mm) {
  Serial.println("Even entered forward");
  ResetPIDs();
  //long targetTicks = MM_TO_TICKS(mm);
  float degreeChangeFromStart = 0;
  float currentDeg = MagneticSensorReading();
  float lastDeg = currentDeg;
  float change = 0;
  float speedadjustment = 0;
  float speedRight;
  float speedLeft;
  weMoved = 0;
  goingForward = false;
  while (weMoved < mm) {  //(abs(rightEncoderCounter) + abs(leftEncoderCounter)) / 2 < targetTicks) {

    int rigthCount = rightEncoderCounter;
    int leftCount = leftEncoderCounter;
    double rightDistanceMoved = (rigthCount - lastRightEncoderCounterUsedToCalculate) * MM_PER_TICK;
    double leftDistanceMoved = (leftCount - lastLeftEncoderCounterUsedToCalculate) * MM_PER_TICK;
    weMoved += (rightDistanceMoved + leftDistanceMoved) / 2.0;
    lastRightEncoderCounterUsedToCalculate = rigthCount;
    lastLeftEncoderCounterUsedToCalculate = leftCount;
    // currentDeg = MagneticSensorReading();
    // change = currentDeg - lastDeg;
    //  if (change > 200) {
    //   change -= 360;
    if (!goingForward) {
      pos = 90;
      SetServoAngle();
      delay(100);
      goingForward = true;
      turnedLeft = false;
      turnedRight = false;
      setPWMRight(0.7);
      setPWMLeft(0.7); 
    }
      setPWMRight(0.7);
      setPWMLeft(0.7);  // 0 to 1
    // } else if (change < -200) {
    //   change += 360;
    // }
    // degreeChangeFromStart += change;
    // lastDeg = currentDeg;
    //speedadjustment = PidController_straightForward_adjust(0, 0.6, 0.22, 0.1, degreeChangeFromStart);  // 0.6, 0.2, 0.1
    // speedLeft = 140;
    // speedRight = 140;
    // if (speedadjustment > 150) {
    //  speedadjustment = 150;
    // } else if (speedadjustment < -150) {
    //    speedadjustment = -150;
    // }
    //Serial.println("We are in the loop my boiiii");
    //speedLeft += speedadjustment;
    // speedRight -= speedadjustment;
    //MoveRightMotor(speedRight);
    //MoveLeftMotor(speedLeft);
  }
  setPWMLeft(0);  // 0 to 1
  setPWMRight(0);
  goingForward = false;
  turnedLeft = false;
  turnedRight = false;
  // MoveRightMotor(0);
  // MoveLeftMotor(0);
  ResetPIDs();
}
void turn(float degree) {
  Serial.println("Entered turn too");
  ResetPIDs();
  delay(500);
  int dir = degree > 0 ? -1 : 1;
  float degreeChangeFromStart = 0;
  float currentDeg = MagneticSensorReading();
  float lastDeg = currentDeg;
  float change = 0;

  while (fabs(degree) - fabs(degreeChangeFromStart) > 10) {

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
        pos = 150;
        SetServoAngle();
        delay(100);
        turnedLeft = false;
        goingForward = false;
        turnedRight = true;
        setPWMLeft(0.7);
        setPWMRight(0);
      }
    } else {
      if (!turnedLeft) {
        pos = 30;
        SetServoAngle();
        delay(100);
        turnedLeft = true;
        goingForward = false;
        turnedRight = false;
        setPWMLeft(0);
        setPWMRight(0.7);
      }
    }
  }
  setPWMLeft(0);
  setPWMRight(0);
  turnedLeft = false;
  goingForward = false;
  turnedRight = false;
  // MoveRightMotor(0);
  // MoveLeftMotor(0);
  ResetPIDs();
}
void turnOnCrack(float degree) {
  ResetPIDs();
  delay(400);
  int dir = degree > 0 ? 1 : -1;
  float degreeChangeFromStart = 0;
  float currentDeg = MagneticSensorReading();
  float lastDeg = currentDeg;
  float change = 0;


  float speed = 140;
  while (fabs(degree) - fabs(degreeChangeFromStart) > 0) {

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
        pos = 150;
        SetServoAngle();
        delay(100);
        turnedLeft = false;
        goingForward = false;
        turnedRight = true;
      }
      setPWMLeft(0.5);
      setPWMRight(0);
    } else {
      if (!turnedLeft) {
        pos = 30;
        SetServoAngle();
        delay(100);
        turnedLeft = true;
        goingForward = false;
        turnedRight = false;
      }
      setPWMLeft(0);
      setPWMRight(0.5);
    }
    //MoveRightMotor(speed * dir);
    // MoveLeftMotor(-speed * dir);
  }
  setPWMLeft(0);
  setPWMRight(0);
  turnedLeft = false;
  goingForward = false;
  turnedRight = false;
  //MoveRightMotor(0);
  //MoveLeftMotor(0);
  ResetPIDs();
  delay(300);
}
void ReverseDirection() {
  setPWMLeft(0);
  setPWMRight(0);
  delay(1000);
  if (currentGoingForwardDir) {  // NOTE there must be one LOW and one HIGH to move in a direction... forgot which is which    :D
                                 // digitalWrite(dirLeftPin, LOW);
                                 // digitalWrite(dirRightPin, LOW);
    currentGoingForwardDir = false;
  } else {
    //   digitalWrite(dirLeftPin, HIGH);
    // digitalWrite(dirRightPin, HIGH);
    //currentGoingForwardDir = true;
  }
}
int maxloops = 0;
void loop() {


  //Serial.println(MagneticSensorReading());
  //delay(200);



  //spin();
  //if (yes) {
  // setPWMLeft(0.7);
  // setPWMRight(0.7);
  // Serial.println("Engine started");
  // yes = false;
  //} else {
  //  setPWMLeft(0);
  // setPWMRight(0);
  //  Serial.println("Engine stopped");
  //  yes = true;
  // }
  //delay(2000);
  //setPWMLeft(0);
  //setPWMRight(0);
  //delay(2000);

  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin("Miyagi", "$;)_eo73,,.5dhWLd*@");
    delay(3000);
    while (!client.connect(host, port)) {
      Serial.println("failed trying again");
      delay(3000);
    }
  }
  maxloops = 0;
  while (!client.available() && maxloops < 500) {
    maxloops++;
    delay(1);
  }
  // GetUltrasoundData(0, true);
  if (client.available() > 0) {
    while (client.available()) {
      String line = client.readStringUntil('~');
      if (line == "moveForward") {
        String distanceStr = client.readStringUntil('~');
        int distanceToMove = distanceStr.toInt();
        Serial.println("WE hERE too");
        forward(distanceToMove);
      } else if (line == "smurf") {
        smurfMovement(client.readStringUntil('~'));
      } else if (line == "turn") {
        String degreeStr = client.readStringUntil('~');
        float degreeToTurn = degreeStr.toFloat();
        Serial.println("WE hERE");
        // turnOnCrack(degreeToTurn / 2);
        turn(degreeToTurn);
      } else if (line == "servo") {
        String servoValue = client.readStringUntil('~');
        pos = servoValue.toInt();
        SetServoAngle();
      } else if (line == "pwm") {
        String pwmValue = client.readStringUntil('~');
        pwmValueInt = pwmValue.toInt();
      } else if (line == "calMag") {
        CollectAndSendMagDataForCalibration();
      } else if (line == "relay") {
      }
    }
  }
  delay(150);
}
