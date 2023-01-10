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

//Controller settings
byte interruptPin = 9;
byte channelAmount = 8;
PPMReader ppm(interruptPin, channelAmount);

// Servo structs
struct ServoStruct {
  Servo servo;
  int angle;
  int maxSpeed;
};

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

  // Reset all motors to 0 position
  HookServo.write(0);
  ClawArmServo.write(0);
  ClawServo.write(0);

  // ServoController servoController = {hookStruct, armStruct, clawStruct, 0, 0, 0};

  // Set L298N motor pins to OUTPUT
  pinMode(L_FWDPIN, OUTPUT);
  pinMode(L_BWDPIN, OUTPUT);
  pinMode(L_ENPIN, OUTPUT);

  pinMode(R_FWDPIN, OUTPUT);
  pinMode(R_BWDPIN, OUTPUT);
  pinMode(R_ENPIN, OUTPUT);
  pinMode(2, INPUT);

  // Start serial comms
  Serial.begin(BAUD_RATE);
}

ServoStruct hookStruct = {HookServo, 0, DEFAULT_MAX_SPEED};
ServoStruct armStruct = {ClawArmServo, 0, DEFAULT_MAX_SPEED};
ServoStruct clawStruct = {ClawServo, 0, DEFAULT_MAX_SPEED};

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

  // long hookControl = map(leftBumper, 1000, 2000, -1, 1);
  // long clawControl = map(rightBumper, 1000, 2000, -1, 1);
  long armControl = map(right3Switch, 1000, 2000, -1, 1);
  armControl = round(armControl);
  // long speedControl = map(rightVert, 1000, 2000, 0, 1);
  // long directionControl = map(leftVert, 1000, 2000, -1, 1);
  // long turningControl = map(leftHorz, 1000, 2000, -1, 1);

  incrementServoAngle(armStruct, armControl, 180, 0);
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

void setServoAngle(ServoStruct servoStruct, int targetAngle) {
  // Incomplete!!!
  int speedLimit = servoStruct.maxSpeed;
  int currentAngle = servoStruct.angle;
  Servo servo = servoStruct.servo;
  for (int angle = currentAngle; angle <= targetAngle; angle += speedLimit) {
    servo.write(angle);
    delay(15);
  }
}

// void remapControls(int input, int )

void incrementServoAngle(ServoStruct servoStruct, int angleDifference, int maxAngle, int minAngle) {
  int speedLimit = servoStruct.maxSpeed;
  int currentAngle = servoStruct.angle;
  int targetAngle = currentAngle + angleDifference;
  if (targetAngle > maxAngle) {targetAngle = maxAngle;}
  if (targetAngle < minAngle) {targetAngle = minAngle;}
  servoStruct.servo.write(targetAngle);
}

void move(bool dir, int throttleVal, int steeringVal, int leftLimit, int rightLimit, int L_EN, int R_EN) {
    int leftSpeed;
    int rightSpeed;
    int turnSpeed;

    leftSpeed = map(throttleVal, leftLimit, rightLimit, 127, 255);
    rightSpeed = map(throttleVal, leftLimit, rightLimit, 127, 255);

    if (steeringVal >= 1000 and steeringVal < 1400){
    turnSpeed = map(steeringVal, 1400, 1000, 0, 127);
    leftSpeed = abs(leftSpeed + (-2 * dir + 1)*turnSpeed);
    rightSpeed = abs(rightSpeed + (2 * dir - 1)*turnSpeed);
    }
    else if (steeringVal <= 2000 and steeringVal > 1600){
    turnSpeed = map(steeringVal, 1600, 2000, 0, 127);
    leftSpeed = abs(leftSpeed + (2 * dir - 1)*turnSpeed);
    rightSpeed = abs(rightSpeed + (-2 * dir + 1)*turnSpeed);
    }
    else{
    leftSpeed = map(throttleVal, leftLimit, rightLimit, 127, 255);
    rightSpeed = map(throttleVal, leftLimit, rightLimit, 127, 255);
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

