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
#define distance_Wheels 135  // Distance between the wheels
#define TICKS_PER_REV 40
#define MM_PER_TICK (((WHEEL_DIAM * PI) / TICKS_PER_REV))
#define MM_TO_TICKS(A) ((float)(A)) / MM_PER_TICK
#define MM_PER_DEGREES(D) ((distance_Wheels * PI * (D)) / 360.0f)
#define turnDegr(M) ((M) / (PI * distance_Wheels * 360.0f))

float midUSArr[] = { 0, 0, 0 };
float leftUSArr[] = { 0, 0, 0 };
float rightUSArr[] = { 0, 0, 0 };
float _medianMid;
float _medianLeft;
float _medianRight;
WiFiClient client;
int bounce = 4;  //to avoid problems with optical encoders
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
float distanceMid = 0;
float distanceLeft = 0;
float distanceRight = 0;
volatile bool canCollectDataMid = false;
volatile bool canCollectDataLeft = false;
volatile bool canCollectDataRight = false;
unsigned long timeOfLastTrigger = millis();
float weMoved = 0;  // to store how much we moved in mm
int lastRightEncoderCounterUsedToCalculate = 0;
int lastLeftEncoderCounterUsedToCalculate = 0;
Adafruit_HMC5883_Unified mag;
sensors_event_t event;
unsigned long pid_previousTimeAdj = 0;
float pid_ePreviousAdj = 0;
float pid_eintegralAdj = 0;
unsigned long pid_previousTimeAdjAlt = 0;
float pid_ePreviousAdjAlt = 0;
float pid_eintegralAdjAlt = 0;
unsigned long pid_previousTime = 0;
float pid_ePrevious = 0;
float pid_eintegral = 0;
float currentHeadingDegree = 180;
int adjustmentHeadingLeftLastRead = 0;
int adjustmentHeadingRightLastRead = 0;
int adjustmentHeadingLeftChange = 0;
int adjustmentHeadingRightChange = 0;
String input = "";
String keyInpt = "";
const uint16_t port = 13000;
const char* host = "192.168.43.144";

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
  attachInterrupt(digitalPinToInterrupt(encoderLeftPin), IRS_LeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderRightPin), IRS_RightEncoder, RISING);
  mag = Adafruit_HMC5883_Unified();
  Serial.begin(115200);
  if (!mag.begin()) {  //using the manually assigned sda and scl due to me doing things with the library on my pc
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while (1) { delay(10); }
  }
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
}
void moveForward() {
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
  adjustmentHeadingLeftLastRead = 0;
  adjustmentHeadingRightLastRead = 0;
  adjustmentHeadingLeftChange = 0;
  adjustmentHeadingRightChange = 0;
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
    if (digitalRead(input1Pin)) {
      leftEncoderCounter--;
    } else {
      leftEncoderCounter++;
    }
  }
  // Keep track of when we were here last
  lastInterruptTimeLeftEncoder = interruptTimeLeftEncoder;
}
void IRS_RightEncoder() {
  interruptTimeRightEncoder = millis();
  // If interrupts come faster than Xms, assume it's a bounce and ignore
  if (interruptTimeRightEncoder - lastInterruptTimeRightEncoder > bounce) {
    if (digitalRead(input3Pin)) {
      rightEncoderCounter--;
    } else {
      rightEncoderCounter++;
    }
  }
  // Keep track of when we were here last
  lastInterruptTimeRightEncoder = interruptTimeRightEncoder;
}
void GetUltrasoundData(float dir, bool sendMove) {
  // Calculate the movement based on both encoder readings
  float rightDistanceMoved = (rightEncoderCounter - lastRightEncoderCounterUsedToCalculate) * MM_PER_TICK;
  float leftDistanceMoved = (leftEncoderCounter - lastLeftEncoderCounterUsedToCalculate) * MM_PER_TICK;
  weMoved = (rightDistanceMoved + leftDistanceMoved) / 2.0;
  lastRightEncoderCounterUsedToCalculate = rightEncoderCounter;
  lastLeftEncoderCounterUsedToCalculate = leftEncoderCounter;

  // Update the position and orientation based on the movement
  //float deltaTheta = (rightDistanceMoved - leftDistanceMoved) / distance_Wheels;
  //theta += deltaTheta;
  //posX += weMoved * cos(theta);
  //posY += weMoved * sin(theta);

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
  str.concat(-90);
  str.concat("|");
  str.concat(_medianLeft);
  str.concat("|");
  str.concat(90);
  str.concat("|");
  str.concat(_medianRight);
  str.concat('`');
  client.print(str);
  //Serial.println(str);
}
float MagneticSensorReading() {
  mag.getEvent(&event);
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  if (heading < 0)
    heading += 2 * PI;
  if (heading > 2 * PI)
    heading -= 2 * PI;
  float headingDegrees = heading * 180 / M_PI;
  return headingDegrees;
}
void AdjustHeading(int leftTicks, int rightTicks) {
  // add PID to turn and check other things

  adjustmentHeadingLeftChange = leftTicks - adjustmentHeadingLeftLastRead;
  adjustmentHeadingRightChange = rightTicks - adjustmentHeadingRightLastRead;

  adjustmentHeadingLeftLastRead = leftTicks;
  adjustmentHeadingRightLastRead = rightTicks;

  //((adjustmentHeadingRightLastRead - adjustmentHeadingLeftLastRead) / distance_Wheels) * (180.0 / PI)

  currentHeadingDegree = currentHeadingDegree + ((((adjustmentHeadingRightChange - adjustmentHeadingLeftChange) * MM_PER_TICK) / distance_Wheels) * (180.0 / PI));
  if (currentHeadingDegree < 0) {
    currentHeadingDegree += 360;
  } else if (currentHeadingDegree > 360) {
    currentHeadingDegree -= 360;
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
  } else {
    digitalWrite(input3Pin, LOW);
    digitalWrite(input4Pin, LOW);
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
  } else {
    digitalWrite(input1Pin, LOW);
    digitalWrite(input2Pin, LOW);
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
float PidController(int target, float kp, float kd, float ki, volatile int* countToLookAt) {
  unsigned long currentTime = micros();
  float deltaT = ((float)(currentTime - pid_previousTime)) / 1.0e6;
  float e = *countToLookAt - target;
  float eDerivative = (e - pid_ePrevious) / deltaT;
  pid_eintegral = pid_eintegral + e * deltaT;

  float baseSpeed = 150;  //giving base engine speed

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
void turn(float degree) {
  double target = 0;
  int dir = 1;
  if (degree > 0) {
    dir = -1;
  }
  ResetPIDs();
  float speed = 0;

  int targetTicks = MM_TO_TICKS(MM_PER_DEGREES(fabs(degree)));
  targetTicks = targetTicks * dir;
  speed = PidController(targetTicks, 2.3, 0.1, 0, &rightEncoderCounter);
  while (((fabs(speed) - 150) > 1) && input != "stop") {
    if (client.available() > 0) {
      input = client.readStringUntil('~');
    }
    speed = PidController(targetTicks, 2.3, 0.1, 0, &rightEncoderCounter);

    if (speed > 200) {
      speed = 200;
    } else if (speed < -200) {
      speed = -200;
    }
    if (speed > 0) {
      MoveRightMotor(speed * dir);
      MoveLeftMotor(speed * -dir);
    } else {
      MoveRightMotor(speed * -dir);
      MoveLeftMotor(speed * dir);
    }
    GetUltrasoundData(MagneticSensorReading(), false);
  }
  ResetPIDs();
  digitalWrite(input1Pin, LOW);
  digitalWrite(input2Pin, LOW);
  digitalWrite(input3Pin, LOW);
  digitalWrite(input4Pin, LOW);
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
  // float lastThree[] = { 0, 0, 0 };
  //float current = 0;
  //lastThree[1] = MagneticSensorReading();
  //lastThree[2] = MagneticSensorReading();
  ResetPIDs();
  while (keyInpt != "None") {
    if (client.available() > 0) {
      String temp = client.readStringUntil('~');
      keyInpt = client.readStringUntil('~');
    }
    /* lastThree[0] = lastThree[1];
    lastThree[1] = lastThree[2];
    lastThree[2] = MagneticSensorReading();
    if (lastThree[2] < lastThree[1]) {
      if (lastThree[2] < lastThree[0]) {
        if (lastThree[1] < lastThree[0]) {
          current = lastThree[1];
        } else {
          current = lastThree[0];
        }
      } else {
        current = lastThree[2];
      }
    } else {
      if (lastThree[2] < lastThree[0]) {
        current = lastThree[2];
      } else {
        if (lastThree[1] < lastThree[0]) {
          current = lastThree[0];
        } else {
          current = lastThree[1];
        }
      }
    }*/
    AdjustHeading(leftEncoderCounter, rightEncoderCounter);
    GetUltrasoundData(currentHeadingDegree, false);
    //GetUltrasoundData(current, false);
    MoveRightMotor(200 * direction);
    MoveLeftMotor(200 * (-1) * direction);
  }
  AdjustHeading(leftEncoderCounter, rightEncoderCounter);
  ResetPIDs();
  MoveRightMotor(0);
  MoveLeftMotor(0);
  digitalWrite(input1Pin, LOW);
  digitalWrite(input2Pin, LOW);
  digitalWrite(input3Pin, LOW);
  digitalWrite(input4Pin, LOW);
}
void justForward() {
  ResetPIDs();
  /* float targetDegree = MagneticSensorReading();
  float lastThree[] = { 0, 0, 0 };
  lastThree[1] = MagneticSensorReading();
  lastThree[2] = MagneticSensorReading(); */
  float speedadjustment = 0;
  float speedRight;
  float speedLeft;
  //float current = 0;
  while (keyInpt != "None") {
    if (client.available() > 0) {
      String temp = client.readStringUntil('~');
      keyInpt = client.readStringUntil('~');
    }
    /* lastThree[0] = lastThree[1];
    lastThree[1] = lastThree[2];
    lastThree[2] = MagneticSensorReading();
    if (lastThree[2] < lastThree[1]) {
      if (lastThree[2] < lastThree[0]) {
        if (lastThree[1] < lastThree[0]) {
          current = lastThree[1];
        } else {
          current = lastThree[0];
        }
      } else {
        current = lastThree[2];
      }
    } else {
      if (lastThree[2] < lastThree[0]) {
        current = lastThree[2];
      } else {
        if (lastThree[1] < lastThree[0]) {
          current = lastThree[0];
        } else {
          current = lastThree[1];
        }
      }
    }*/
    speedadjustment = PidController_straightForward_adjust_alternative(&leftEncoderCounter, 7.2, 0.3, 0.2, &rightEncoderCounter);
    speedRight = 230;
    speedLeft = 230;
    if (speedadjustment > 150) {
      speedadjustment = 150;
    } else if (speedadjustment < -150) {
      speedadjustment = -150;
    }
    speedLeft -= speedadjustment;
    speedRight += speedadjustment;
    AdjustHeading(leftEncoderCounter, rightEncoderCounter);
    GetUltrasoundData(currentHeadingDegree, true);

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
    str.concat(currentHeadingDegree);
    str.concat("|");
    str.concat("adjustmentLeft:");
    str.concat(adjustmentHeadingLeftChange);
    str.concat("|");
    str.concat("adjustmentLeft:");
    str.concat(adjustmentHeadingLeftChange);
    str.concat('`');
    client.print(str);



    MoveRightMotor(speedRight);
    MoveLeftMotor(speedLeft);
  }
  MoveRightMotor(0);
  MoveLeftMotor(0);
  AdjustHeading(leftEncoderCounter, rightEncoderCounter);
  digitalWrite(input1Pin, LOW);
  digitalWrite(input2Pin, LOW);
  digitalWrite(input3Pin, LOW);
  digitalWrite(input4Pin, LOW);
  ResetPIDs();
}
void forward(int mm) {
  ResetPIDs();
  long target = (int)MM_TO_TICKS(mm) + rightEncoderCounter;
  float targetDegree = MagneticSensorReading();
  float speedRight = 200;
  float current = 0;
  float speedadjustment = 0;
  float speedAdjustAlternative = 0;
  float speedLeft;
  float lastThree[] = { 0, 0, 0 };
  lastThree[1] = MagneticSensorReading();
  lastThree[2] = MagneticSensorReading();
  if (mm > 0) {
    MoveRightMotor(200);
    MoveLeftMotor(200);
  } else if (mm < 0) {
    MoveRightMotor(-200);
    MoveLeftMotor(-200);
  }
  while (((target - rightEncoderCounter) > 1) && input != "stop") {
    if (client.available() > 0) {
      input = client.readStringUntil('~');
    }
    lastThree[0] = lastThree[1];
    lastThree[1] = lastThree[2];
    lastThree[2] = MagneticSensorReading();
    if (lastThree[2] < lastThree[1]) {
      if (lastThree[2] < lastThree[0]) {
        if (lastThree[1] < lastThree[0]) {
          current = lastThree[1];
        } else {
          current = lastThree[0];
        }
      } else {
        current = lastThree[2];
      }
    } else {
      if (lastThree[2] < lastThree[0]) {
        current = lastThree[2];
      } else {
        if (lastThree[1] < lastThree[0]) {
          current = lastThree[0];
        } else {
          current = lastThree[1];
        }
      }
    }

    speedRight = PidController(target, 0.9, 0.2, 0.1, &rightEncoderCounter);
    speedadjustment = PidController_straightForward_adjust(targetDegree, 6.2, 0.3, 0.2, current);  //pidControllertwo(rightcounthelper, 3, 1.6, 0.5, &leftcounthelper);
    //speedAdjustAlternative = PidController(rightEncoderCounter, 1.4, 0.2, 0, &leftEncoderCounter);
    if (speedRight > 255) {
      speedRight = 255;
    } else if (speedRight < -255) {
      speedRight = -255;
    }

    speedLeft = speedRight;
    if (speedadjustment > 120) {
      speedadjustment = 120;
    } else if (speedadjustment < -120) {
      speedadjustment = -120;
    }
    if (speedAdjustAlternative > 120) {
      speedAdjustAlternative = 120;
    } else if (speedAdjustAlternative < -120) {
      speedAdjustAlternative = -120;
    }

    // try to pause
    MoveRightMotor(0);
    MoveLeftMotor(0);
    delayMicroseconds(50);
    //
    GetUltrasoundData(current, true);
    speedLeft = speedLeft + speedadjustment;
    speedRight = speedRight - speedadjustment;
    MoveRightMotor(speedRight * -1);
    MoveLeftMotor(speedLeft * -1);
  }
  digitalWrite(input1Pin, LOW);
  digitalWrite(input2Pin, LOW);
  digitalWrite(input3Pin, LOW);
  digitalWrite(input4Pin, LOW);
  ResetPIDs();
}

void loop() {
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin("Miyagi", ";4r5#-8g");
    Serial.println(WiFi.RSSI());
    Serial.println("Trying to connect..");
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
      if (line == "mov") {
        // String distanceStr = client.readStringUntil('~');
        // int distanceToMove = distanceStr.toInt();
        //  forward(distanceToMove);
      } else if (line == "smurf") {
        smurfMovement(client.readStringUntil('~'));
      } else if (line == "turn") {
        //   String degreeStr = client.readStringUntil('~');
        //  float degreeToTurn = degreeStr.toFloat();
        //  turn(degreeToTurn);
      } else if (line == "ready") {  // give data for current location and ask for a guess
      }
    }
  }
  //Serial.println(currentHeadingDegree);
  /* delay(750);
  MoveRightMotor(200);
  MoveLeftMotor(-200);
  AdjustHeading(leftEncoderCounter, rightEncoderCounter);
  Serial.print("adjustment degree left: ");
  Serial.println(adjustmentHeadingLeftChange);
  Serial.print("adjustment degree right: ");
  Serial.println(adjustmentHeadingRightChange);
  Serial.print("Encoder check left: ");
  Serial.println(leftEncoderCounter);
  Serial.print("Encoder check right: ");
  Serial.println(rightEncoderCounter); 
  Serial.println(MagneticSensorReading());*/
}
