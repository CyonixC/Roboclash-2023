#include <PPMReader.h>
#include <Servo.h>

// Baud rate
const unsigned long BAUD_RATE = 115200;

// Motor pins
const int L_FWDPIN = A2;
const int L_BWDPIN = A3;
const int L_ENPIN = 5;

const int R_FWDPIN = 12;
const int R_BWDPIN = 13;
const int R_ENPIN = 6;

// Servo pins
// REMEMBER TO CHANGE AFTER WIRING
const int HOOK_PIN = 0;
const int ARM_PIN = 8;
const int CLAW_PIN = 7;
const int TEST_SERVO = 0;

// Servos
Servo HookServo;
Servo ClawArmServo;
Servo ClawServo;
Servo TestServo;

const int DEFAULT_MAX_SPEED = 3;

// Servo starting positions
const int HOOK_STARTING = 0;
const int CLAW_STARTING = 0;
const int ARM_STARTING = 0;

//Controller settings
byte interruptPin = 9;
byte channelAmount = 8;
PPMReader ppm(interruptPin, channelAmount);
int DEFAULT_CHANNEL_LIMITS[] = {1000, 1984};

// Servo structs
struct ServoStruct {
  Servo servo;
  int angle;
  int maxSpeed;
  int minAngle;
  int maxAngle;
};

int PLUS_MINUS_ONE = 0;
int ZERO_TO_ONE = 1;
int ANALOG_ZERO_RANGE[] = {1400, 1600};
int DISCRETE_ZERO_RANGE[] = {1250, 1750};

// struct ServoController {
//   ServoStruct servo1;
//   ServoStruct servo2;
//   ServoStruct servo3;
//   int targetAngle1;
//   int targetAngle2;
//   int targetAngle3;
// };

void setup(){
  // Attach servo pins
  HookServo.attach(HOOK_PIN);
  ClawArmServo.attach(ARM_PIN);
  ClawServo.attach(CLAW_PIN);
  // TestServo.attach(TEST_SERVO);

  // Reset all motors to starting position
  HookServo.write(HOOK_STARTING);
  ClawArmServo.write(ARM_STARTING);
  ClawServo.write(CLAW_STARTING);

  // Set L298N motor pins to OUTPUT
  pinMode(L_FWDPIN, OUTPUT);
  pinMode(L_BWDPIN, OUTPUT);
  pinMode(L_ENPIN, OUTPUT);

  pinMode(R_FWDPIN, OUTPUT);
  pinMode(R_BWDPIN, OUTPUT);
  pinMode(R_ENPIN, OUTPUT);
  
  // Array of motor pins for easier passing to functions
  int motorPins[] = {L_FWDPIN, L_BWDPIN, L_ENPIN, R_FWDPIN, R_BWDPIN, R_ENPIN};

  // Start serial comms
  Serial.begin(BAUD_RATE);
}

ServoStruct hookStruct = {HookServo, HOOK_STARTING, DEFAULT_MAX_SPEED, 0, 120};
ServoStruct armStruct = {ClawArmServo, ARM_STARTING, DEFAULT_MAX_SPEED, 0, 120};
ServoStruct clawStruct = {ClawServo, CLAW_STARTING, DEFAULT_MAX_SPEED, 0, 120};

void loop(){
  // Controller channels, comment out the ones we don't need!
  // int rightHorz = readChannel(1, false);
  // int leftVert = readChannel(2, false);
  // int rightVert = readChannel(3, false);
  // int leftHorz = readChannel(4, false);
  // int leftBumper = readChannel(6, false);
  // int rightBumper = readChannel(7, false);
  // int left3Switch = readChannel(8, false);
  // int right2Switch = readChannel(9, false);
  int right3Switch = readChannel(10, false);

  // long hookControl = remapControlValues(leftBumper, DEFAULT_CHANNEL_LIMITS, ANALOG_ZERO_RANGE, PLUS_MINUS_ONE);
  // long clawControl = remapControlValues(rightBumper, DEFAULT_CHANNEL_LIMITS, ANALOG_ZERO_RANGE, PLUS_MINUS_ONE);
  long armControl = remapControlValues(right3Switch, DEFAULT_CHANNEL_LIMITS, DISCRETE_ZERO_RANGE, PLUS_MINUS_ONE);
  int armIncrement = round(armControl);
  // long speedControl = remapControlValues(rightVert, DEFAULT_CHANNEL_LIMITS, rightVertZero, ZERO_TO_ONE);
  // int rightVertZero[] = {1000, 1100};
  // long directionControl = remapControlValues(leftVert, DEFAULT_CHANNEL_LIMITS, ANALOG_ZERO_RANGE, PLUS_MINUS_ONE);
  // long turningControl = remapControlValues(leftHorz, DEFAULT_CHANNEL_LIMITS, ANALOG_ZERO_RANGE, PLUS_MINUS_ONE);

  incrementServoAngle(armStruct, armIncrement);
  delay(15);
}





// ----------------------
// Custom Functions below
// ----------------------

void debugChannels() {
  int rightHorz = readChannel(1, true);
  int leftVert = readChannel(2, true);
  int rightVert = readChannel(3, true);
  int leftHorz = readChannel(4, true);
  int leftBumper = readChannel(6, true);
  int rightBumper = readChannel(7, true);
  int left3Switch = readChannel(8, true);
  int right2Switch = readChannel(9, true);
  Serial.println("");
  delay(3000);
}

void testServo(Servo servo){
  servo.write(0); // move MG996R's shaft to angle 0°
  delay(1000); // wait for one second
  servo.write(45); // move MG996R's shaft to angle 45°
  delay(1000); // wait for one second 
  servo.write(90); // move MG996R's shaft to angle 90°
  delay(1000); // wait for one second
  servo.write(120); // move MG996R's shaft to angle 120°
  delay(1000); // wait for one second
}

long remapControlValues(int input, int inLimits[2], int zeroRange[2], int mode) {
  if (input > zeroRange[0] && input < zeroRange[1]) return 0;
  
  if (mode == PLUS_MINUS_ONE) {
    if (input <= zeroRange[0]) {
      long output = map(input, inLimits[0], zeroRange[0], -1, 0);
      if (output < -1) output = -1;
      if (output > 0) output = 0;
    }
    else {
      long output = map(input, zeroRange[1], inLimits[1], 0, 1);
      if (output < 0) output = 0;
      if (output > 1) output = 1;
    }
  } else if (mode == ZERO_TO_ONE) {
      long output = map(input, zeroRange[1], inLimits[1], 0, 1);
      if (output < 0) output = 0;
      if (output > 1) output = 1;
  }
}

void incrementServoAngle(ServoStruct servoStruct, int angleDifference) {
  int speedLimit = servoStruct.maxSpeed;
  int currentAngle = servoStruct.angle;
  int targetAngle = currentAngle + angleDifference;
  int minAngle = servoStruct.minAngle;
  int maxAngle = servoStruct.maxAngle;

  if (targetAngle > maxAngle) {targetAngle = maxAngle;}
  if (targetAngle < minAngle) {targetAngle = minAngle;}
  servoStruct.servo.write(targetAngle);
}

void move(int dirVal, int speedVal, int turnVal) {
    int leftSpeed;
    int rightSpeed;
    int turnSpeed;

    leftSpeed = map(speedVal, leftLimit, rightLimit, 127, 255);
    rightSpeed = map(speedVal, leftLimit, rightLimit, 127, 255);

    if (turnVal >= 1000 and turnVal < 1400){
    turnSpeed = map(turnVal, 1400, 1000, 0, 127);
    leftSpeed = abs(leftSpeed + (-2 * dirVal + 1)*turnSpeed);
    rightSpeed = abs(rightSpeed + (2 * dirVal - 1)*turnSpeed);
    }
    else if (turnVal <= 2000 and turnVal > 1600){
    turnSpeed = map(turnVal, 1600, 2000, 0, 127);
    leftSpeed = abs(leftSpeed + (2 * dirVal - 1)*turnSpeed);
    rightSpeed = abs(rightSpeed + (-2 * dirVal + 1)*turnSpeed);
    }
    else{
    leftSpeed = map(speedVal, leftLimit, rightLimit, 127, 255);
    rightSpeed = map(speedVal, leftLimit, rightLimit, 127, 255);
    }

    if(leftSpeed > 255){leftSpeed = 255;}
    if(rightSpeed > 255){rightSpeed = 255;}
    analogWrite(L_EN, leftSpeed);
    analogWrite(R_EN, rightSpeed);
}

void leftFwd() {
  digitalWrite(L_FWDPIN, HIGH);
  digitalWrite(L_BWDPIN, LOW);
}
void leftBwd() {
  digitalWrite(L_FWDPIN, LOW);
  digitalWrite(L_BWDPIN, HIGH);
}
void leftStop() {
  digitalWrite(L_FWDPIN, LOW);
  digitalWrite(L_BWDPIN, LOW);
}
void rightFwd() {
  digitalWrite(R_FWDPIN, HIGH);
  digitalWrite(R_BWDPIN, LOW);
}
void rightBwd() {
  digitalWrite(R_FWDPIN, LOW);
  digitalWrite(R_BWDPIN, HIGH);
}
void rightStop() {
  digitalWrite(R_FWDPIN, LOW);
  digitalWrite(R_BWDPIN, LOW);
}

int readChannel(int channelNumber, bool debug) {
unsigned value = ppm.latestValidChannelValue(channelNumber, 0);
if (debug) {
    Serial.print("Channel ");
    Serial.print(channelNumber);
    Serial.print(": ");
    Serial.println(value);
}
return value;
}

