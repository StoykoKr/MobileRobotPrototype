#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

#define input1Pin 27
#define input2Pin 26
#define input3Pin 25
#define input4Pin 33
#define analogOutputLeftPin 14  // pwm analog values for speed TODO
#define analogOutputRightPin 32
#define trigPin 16
#define echoMidPin 4
#define echoLeftPin 18
#define echoRightPin 2
#define encoderLeftPin 35  // encoder has 20 holes
#define encoderRightPin 17
//#define sclMagneticPin 22  // only used due to manual changes in a library
//#define sdaMagneticPin 21

#define WHEEL_DIAM 65
#define distance_Wheels 140  // Distance between the wheels
#define TICKS_PER_REV 20
#define MM_PER_TICK (((WHEEL_DIAM * PI) / TICKS_PER_REV))
#define MM_TO_TICKS(A) ((float)(A)) / MM_PER_TICK
//#define MM_PER_DEGREES(D) ((distance_Wheels * PI * (D)) / 360.0f)
//#define turnDegr(M) ((M) / (PI * distance_Wheels * 360.0f))

double midUSArr[] = { 0, 0, 0 };
double leftUSArr[] = { 0, 0, 0 };
double rightUSArr[] = { 0, 0, 0 };
double _medianMid;
double _medianLeft;
double _medianRight;
WiFiClient client;
int bounce = 9;  //to avoid problems with optical encoders  used to be 7
volatile unsigned long interruptTimeLeftEncoder;
volatile unsigned long interruptTimeRightEncoder;
volatile unsigned long lastInterruptTimeLeftEncoder = 0;
volatile unsigned long lastInterruptTimeRightEncoder = 0;
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
double weMoved = 0;  // to store how much we moved in mm
int lastRightEncoderCounterUsedToCalculate = 0;
int lastLeftEncoderCounterUsedToCalculate = 0;
Adafruit_HMC5883_Unified mag;
sensors_event_t event;
unsigned long pid_previousTimeAdj = 0;
double pid_ePreviousAdj = 0;
double pid_eintegralAdj = 0;
unsigned long pid_previousTimeAdjAlt = 0;
double pid_ePreviousAdjAlt = 0;
double pid_eintegralAdjAlt = 0;
unsigned long pid_previousTime = 0;
double pid_ePrevious = 0;
double pid_eintegral = 0;
String keyInpt = "";
bool leftGoingForward = false;
bool rightGoingForward = false;
const uint16_t port = 13000;
const char* host = "192.168.43.144";
const float hard_iron[3] = {  // with magneto the values are new
  211.01 * 1.47, -260.85 * 1.47, -641.62 * 1.47
};

const float soft_iron[3][3] = {
  { 1.565 * 1.47, -0.039 * 1.47, -0.001 * 1.47 },
  { -0.039 * 1.47, 1.510 * 1.47, -0.028 * 1.47 },
  { -0.001 * 1.47, -0.028 * 1.47, 1.502 * 1.47 }
};


/*
const float hard_iron[3] = {  // this and soft_iron are the magnetic sensor calibration
  27.10, -27.45, -94.76
};

const float soft_iron[3][3] = {
  { 1.103, 0.058, 0.100 },
  { 0.058, 1.145, -0.017 },
  { 0.100, -0.017, 0.804 }
}; */

/*const float hard_iron[3] = {  // this and soft_iron are the magnetic sensor calibration
  4.05, -50.48, -27.46
};

const float soft_iron[3][3] = {
  { 1.067, 0.045, 0.040 },
  { 0.045, 0.939, 0.033 },
  { 0.040, -0.033, 1.002 }
};*/

void setup() {
  pinMode(input1Pin, OUTPUT);
  pinMode(input2Pin, OUTPUT);
  pinMode(input3Pin, OUTPUT);
  pinMode(input4Pin, OUTPUT);
  pinMode(analogOutputLeftPin, OUTPUT);
  pinMode(analogOutputRightPin, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoMidPin, INPUT);
  pinMode(echoLeftPin, INPUT);
  pinMode(echoRightPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(echoMidPin), IRS_MidSensor, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echoLeftPin), IRS_LeftSensor, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echoRightPin), IRS_RightSensor, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRightPin), IRS_RightEncoder, RISING);
  // attachInterrupt(digitalPinToInterrupt(encoderLeftPin), IRS_LeftEncoder, RISING);
  mag = Adafruit_HMC5883_Unified();
  Serial.begin(115200);
  if (!mag.begin()) {  //using the manually assigned sda and scl due to me doing things with the library on my pc
    while (1) { delay(10); }
  }
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
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

  interruptTimeLeftEncoder = millis();
  // If interrupts come faster than Xms, assume it's a bounce and ignore
  if (interruptTimeLeftEncoder - lastInterruptTimeLeftEncoder > bounce) {
    if (leftGoingForward) {
      leftEncoderCounter--;
    } else {
      leftEncoderCounter++;
    }
    lastInterruptTimeLeftEncoder = interruptTimeLeftEncoder;
  }

  //leftEncoderCounter++;
  // Keep track of when we were here last
}
void IRS_RightEncoder() {


  interruptTimeRightEncoder = millis();
  // If interrupts come faster than Xms, assume it's a bounce and ignore
  if (interruptTimeRightEncoder - lastInterruptTimeRightEncoder > bounce) {
    if (rightGoingForward) {
      rightEncoderCounter--;
    } else {
      rightEncoderCounter++;
    }
    lastInterruptTimeRightEncoder = interruptTimeRightEncoder;
  }

  // rightEncoderCounter++;
}
void GetUltrasoundData(float dir, bool sendMove) {
  // Calculate the movement based on both encoder readings
  int rigthCount = rightEncoderCounter;
  int leftCount = leftEncoderCounter;
  double rightDistanceMoved = (rigthCount - lastRightEncoderCounterUsedToCalculate) * MM_PER_TICK;
  double leftDistanceMoved = (leftCount - lastLeftEncoderCounterUsedToCalculate) * MM_PER_TICK;
  weMoved = rightDistanceMoved;  //(rightDistanceMoved + leftDistanceMoved) / 2.0;
  lastRightEncoderCounterUsedToCalculate = rigthCount;
  lastLeftEncoderCounterUsedToCalculate = leftCount;

  if (millis() - timeOfLastTrigger >= 50) {
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
  if (sendMove) {
    str.concat(weMoved);
  } else {
    str.concat(0);
  }
  str.concat("|");  // keyword|(kolko sme se mrydnali)|useless|direction|
  str.concat(dir);
  //str.concat(posY);
  str.concat("|");
  str.concat(0);
  // str.concat(theta);
  str.concat("|");
  str.concat(_medianMid);
  str.concat("|");
  str.concat(90);
  str.concat("|");
  str.concat(_medianLeft);
  str.concat("|");
  str.concat(-90);
  str.concat("|");
  str.concat(_medianRight);
  str.concat('`');
  client.print(str);
  //Serial.println(str);
}
float MagneticSensorReading() {  // NEEDS TO BE REVERTED LATER
  mag.getEvent(&event);
  float hi_cal[3];
  float mag_data[] = { event.magnetic.x * 11,
                       event.magnetic.y * 11,
                       event.magnetic.z * 11 };
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
  mag_datat[0] = event.magnetic.x;
  mag_datat[1] = event.magnetic.y;
  mag_datat[2] = event.magnetic.z;

  for (uint8_t i = 0; i < 3; i++) {
    hi_cal[i] = mag_datat[i] - hard_iron[i];
  }
  for (uint8_t i = 0; i < 3; i++) {
    mag_datat[i] = (soft_iron[i][0] * hi_cal[0]) + (soft_iron[i][1] * hi_cal[1]) + (soft_iron[i][2] * hi_cal[2]);
  }
}

void MoveRightMotor(float speedUnfiltered) {
  speedUnfiltered = speedUnfiltered * -1;
  float speed = fabs(speedUnfiltered);
  if (speed > 255)
    speed = 255;
  if (speed < 110)
    speed = 0;
  if (speedUnfiltered > 0) {
    if (digitalRead(input4Pin)) {
      digitalWrite(input3Pin, LOW);
      digitalWrite(input4Pin, LOW);
      delay(3);
      digitalWrite(input3Pin, HIGH);
      digitalWrite(input4Pin, LOW);
    } else if (digitalRead(input3Pin) == 0x0) {
      digitalWrite(input3Pin, HIGH);
      digitalWrite(input4Pin, LOW);
    }
    rightGoingForward = true;
  } else if (speedUnfiltered < 0) {
    if (digitalRead(input3Pin)) {
      digitalWrite(input3Pin, LOW);
      digitalWrite(input4Pin, LOW);
      delay(3);
      digitalWrite(input3Pin, LOW);
      digitalWrite(input4Pin, HIGH);
    }
    if (digitalRead(input4Pin) == 0x0) {
      digitalWrite(input3Pin, LOW);
      digitalWrite(input4Pin, HIGH);
    }
    rightGoingForward = false;
  } else {
    digitalWrite(input3Pin, LOW);
    digitalWrite(input4Pin, LOW);
    rightGoingForward = false;
  }
  analogWrite(analogOutputRightPin, speed);
}
void MoveLeftMotor(float speedUnfiltered) {

  speedUnfiltered = speedUnfiltered * -1;
  float speed = fabs(speedUnfiltered);
  if (speed > 255)
    speed = 255;
  if (speed < 110)
    speed = 0;

  if (speedUnfiltered > 0) {
    if (digitalRead(input2Pin)) {
      digitalWrite(input1Pin, LOW);
      digitalWrite(input2Pin, LOW);
      delay(3);
      digitalWrite(input1Pin, HIGH);
      digitalWrite(input2Pin, LOW);
    }
    if (digitalRead(input1Pin) == 0x0) {
      digitalWrite(input1Pin, HIGH);
      digitalWrite(input2Pin, LOW);
    }
    leftGoingForward = true;
  } else if (speedUnfiltered < 0) {
    if (digitalRead(input1Pin)) {
      digitalWrite(input1Pin, LOW);
      digitalWrite(input2Pin, LOW);
      delay(3);
      digitalWrite(input1Pin, LOW);
      digitalWrite(input2Pin, HIGH);
    }
    if (digitalRead(input2Pin) == 0x0) {
      digitalWrite(input1Pin, LOW);
      digitalWrite(input2Pin, HIGH);
    }
    leftGoingForward = false;
  } else {
    digitalWrite(input1Pin, LOW);
    digitalWrite(input2Pin, LOW);
    leftGoingForward = false;
  }
  analogWrite(analogOutputLeftPin, speed);
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

void smurfMovement(String signal) {
  keyInpt = "";
  if (signal == "w" || signal == "W") {
    justForward();
  } else if (signal == "a" || signal == "A") {
    justLeftRight(1);
  } else if (signal == "s" || signal == "S") {
    //currently not available
  } else if (signal == "d" || signal == "D") {
    justLeftRight(-1);
  } else if (signal == "none" || signal == "None") {
    digitalWrite(input1Pin, LOW);
    digitalWrite(input2Pin, LOW);
    digitalWrite(input3Pin, LOW);
    digitalWrite(input4Pin, LOW);
  }
}
void justLeftRight(int direction) {
  ResetPIDs();
  while (keyInpt != "None") {
    if (client.available() > 0) {
      String temp = client.readStringUntil('~');
      keyInpt = client.readStringUntil('~');
    }
    MoveRightMotor(130 * direction);
    MoveLeftMotor(130 * (-1) * direction);
  }
  MoveRightMotor(0);
  MoveLeftMotor(0);
  ResetPIDs();
}

void justForward() {
  ResetPIDs();

  float degreeChangeFromStart = 0;
  float currentDeg = MagneticSensorReading();
  float lastDeg = currentDeg;
  float change = 0;

  float speedadjustment = 0;
  float speedRight;
  float speedLeft;
  while (keyInpt != "None") {
    if (client.available() > 0) {
      String temp = client.readStringUntil('~');
      keyInpt = client.readStringUntil('~');
    }
    currentDeg = MagneticSensorReading();
    change = currentDeg - lastDeg;
    if (change > 200) {
      change -= 360;
    } else if (change < -200) {
      change += 360;
    }
    degreeChangeFromStart += change;
    lastDeg = currentDeg;
    speedadjustment = PidController_straightForward_adjust(0, 0.9, 0.2, 0.07, degreeChangeFromStart);  //2.2, 0.3, 0.18    ||   0.6, 0.21, 0.1
    speedLeft = 120;
    speedRight = 120;
    if (speedadjustment > 150) {
      speedadjustment = 150;
    } else if (speedadjustment < -150) {
      speedadjustment = -150;
    }
    speedLeft += speedadjustment;
    speedRight -= speedadjustment;
    GetUltrasoundData(currentDeg, true);
    /*
     String str = "report|";
     str.concat(speedLeft);
     str.concat("|");
     str.concat(speedRight);
     str.concat("|");
     str.concat(speedadjustment);
     str.concat("|");
     str.concat(leftEncoderCounter);
      str.concat("|");
     str.concat(rightEncoderCounter);
      str.concat("|");
      str.concat("Heading:");
      str.concat(current);
      str.concat("|");
      str.concat("adjustmentLeft:");
      str.concat(adjustmentHeadingLeftChange);
      str.concat("|");
      str.concat("adjustmentLeft:");
     str.concat(adjustmentHeadingLeftChange);
     str.concat('`');
     client.print(str);
    */
    MoveRightMotor(speedRight);
    MoveLeftMotor(speedLeft);
  }
  GetUltrasoundData(currentDeg, true);
  MoveRightMotor(0);
  MoveLeftMotor(0);
  ResetPIDs();
}
void forward(int mm) {
  ResetPIDs();
  long targetTicks = MM_TO_TICKS(mm);

  float degreeChangeFromStart = 0;
  float currentDeg = MagneticSensorReading();
  float lastDeg = currentDeg;
  float change = 0;
  float speedadjustment = 0;
  float speedRight;
  float speedLeft;
  weMoved = 0;

  while (weMoved < mm) {  //(abs(rightEncoderCounter) + abs(leftEncoderCounter)) / 2 < targetTicks) {

    int rigthCount = rightEncoderCounter;
    int leftCount = leftEncoderCounter;
    double rightDistanceMoved = (rigthCount - lastRightEncoderCounterUsedToCalculate) * MM_PER_TICK;
    double leftDistanceMoved = (leftCount - lastLeftEncoderCounterUsedToCalculate) * MM_PER_TICK;
    weMoved += rightDistanceMoved;  //(rightDistanceMoved + leftDistanceMoved) / 2.0;
    lastRightEncoderCounterUsedToCalculate = rigthCount;
    lastLeftEncoderCounterUsedToCalculate = leftCount;
    currentDeg = MagneticSensorReading();
    change = currentDeg - lastDeg;
    if (change > 200) {
      change -= 360;
    } else if (change < -200) {
      change += 360;
    }
    degreeChangeFromStart += change;
    lastDeg = currentDeg;
    speedadjustment = PidController_straightForward_adjust(0, 0.6, 0.22, 0.1, degreeChangeFromStart);  // 0.6, 0.2, 0.1
    speedLeft = 140;
    speedRight = 140;
    if (speedadjustment > 150) {
      speedadjustment = 150;
    } else if (speedadjustment < -150) {
      speedadjustment = -150;
    }
    speedLeft += speedadjustment;
    speedRight -= speedadjustment;
    MoveRightMotor(speedRight);
    MoveLeftMotor(speedLeft);
  }

  MoveRightMotor(0);
  MoveLeftMotor(0);
  ResetPIDs();
}
void turn(float degree) {
  ResetPIDs();
  delay(500);
  int dir = degree > 0 ? -1 : 1;
  float degreeChangeFromStart = 0;
  float currentDeg = MagneticSensorReading();
  float lastDeg = currentDeg;
  float change = 0;

  while (fabs(degree) - fabs(degreeChangeFromStart) > 30) {

    currentDeg = MagneticSensorReading();
    change = currentDeg - lastDeg;
    if (change > 200) {
      change -= 360;
    } else if (change < -200) {
      change += 360;
    }
    degreeChangeFromStart += change;
    lastDeg = currentDeg;
    MoveRightMotor(130 * dir);
    MoveLeftMotor(-130 * dir);
  }
  MoveRightMotor(0);
  MoveLeftMotor(0);
  ResetPIDs();
}
void turnOnCrack(float degree) {
  ResetPIDs();
  delay(400);
  int dir = degree > 0 ? -1 : 1;
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

    MoveRightMotor(speed * dir);
    MoveLeftMotor(-speed * dir);
  }
  MoveRightMotor(0);
  MoveLeftMotor(0);
  ResetPIDs();
  delay(300);
}
int temp = 0;
void loop() {





  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin("Miyagi", "$;)_eo73,,.5dhWLd*@");
    //WiFi.begin("Stoiko", "01122001");
    delay(3000);
    while (!client.connect(host, port)) {
      Serial.println("failed trying again");
      delay(3000);
    }
  }
  int maxloops = 0;
  while (!client.available() && maxloops < 500) {
    maxloops++;
    delay(1);  //delay 1 msec
  }
  if (client.available() > 0) {
    while (client.available()) {
      String line = client.readStringUntil('~');
      if (line == "moveForward") {
        String distanceStr = client.readStringUntil('~');
        int distanceToMove = distanceStr.toInt();
        forward(distanceToMove);
      } else if (line == "smurf") {
        smurfMovement(client.readStringUntil('~'));
      } else if (line == "turn") {
        String degreeStr = client.readStringUntil('~');
        float degreeToTurn = degreeStr.toFloat();
        turnOnCrack(degreeToTurn / 2);
      } else if (line == "ready") {  // give data for current location and ask for a guess
      }
    }
  }
  delay(100);
  /*
  mag.getEvent(&event);
  Serial.print("X = ");
  Serial.println(event.magnetic.x);
  Serial.print("Y = ");
  Serial.println(event.magnetic.y);
  Serial.print("Z = ");
  Serial.println(event.magnetic.z);
  */
  //Serial.println(MagneticSensorReading());
  /*int rigthCount = rightEncoderCounter;
  int leftCount = leftEncoderCounter;
  double rightDistanceMoved = (rigthCount - lastRightEncoderCounterUsedToCalculate) * MM_PER_TICK;
  double leftDistanceMoved = (leftCount - lastLeftEncoderCounterUsedToCalculate) * MM_PER_TICK;
  int rd = rigthCount - lastRightEncoderCounterUsedToCalculate;
  int ld = leftCount - lastLeftEncoderCounterUsedToCalculate;
  weMoved += (rightDistanceMoved + leftDistanceMoved) / 2;
  tempMoved += rd;
  lastRightEncoderCounterUsedToCalculate = rigthCount;
  lastLeftEncoderCounterUsedToCalculate = leftCount;  
  Serial.print("Right encoder:");
  Serial.println(rigthCount);
  Serial.print("left encoder:");
  Serial.println(leftCount); */
  /*if (millis() >= 10000 && temp == 0) {
    temp = 1;
  } else if (temp == 0) {
   */
  /*
    String str = "";
    str.concat(mag_datat[0] * 10);
    str.concat(" ");
    str.concat(mag_datat[1] * 10);
    str.concat(" ");
    str.concat(mag_datat[2] * 10);
    str.concat(" ");
    Serial.println(str);
  
  MagneticSensorReadingFORPROCESSING();*/
  /*  Serial.print("x:");
  Serial.print(mag_datat[0]);
  Serial.print(",");
  Serial.print("y:");
  Serial.print(mag_datat[1]);
  Serial.print(",");
  Serial.print("z:");
  Serial.print(mag_datat[2]);*/
  //Serial.print("heading:");
  //Serial.println(MagneticSensorReading());
  /*Serial.print(",");
  Serial.print("mid:");
  Serial.println(180); */
}
