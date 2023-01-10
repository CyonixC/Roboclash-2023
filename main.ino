#include <IBusBM.h>
#include <PPMReader.h>
#include <Servo.h>

// Setup for the IBus channels
IBusBM ibusRc;

HardwareSerial& IBusSerial = Serial1;

// Baud rate
const unsigned long BAUD_RATE = 115200;

// Motor pins

const int L_FWDPIN = A2;
const int L_BWDPIN = A3;
const int L_ENPIN = 5;

const int R_FWDPIN = 12;
const int R_BWDPIN = 7;
const int R_ENPIN = 6;

// Servo pins
// REMEMBER TO CHANGE AFTER WIRING
const int HOOK_PIN = 11;
const int ARM_PIN = 8;
const int CLAW_PIN = 10;
const int TEST_SERVO = 0;   // only for testing

// Servos
Servo HookServo;
Servo ClawArmServo;
Servo ClawServo;
Servo TestServo;

// Servo sensitivity (controls the speed that the servos move, only can be ints)
const int CLAW_SENSITIVITY = 5;
const int ARM_SENSITIVITY = 1;
const int HOOK_SENSITIVITY = 5;

// Servo starting positions (degrees)
const int HOOK_STARTING = 0;
const int CLAW_STARTING = 0;
const int ARM_STARTING = 120;

// Controller settings
int DEFAULT_CHANNEL_LIMITS[] = {1000, 2000};

// Servo structs
struct ServoStruct {
  Servo servo;
  int angle;
  int minAngle;
  int maxAngle;
};


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

  // Set BTS7960 motor pins to OUTPUT
  pinMode(L_FWDPIN, OUTPUT);
  pinMode(L_BWDPIN, OUTPUT);
  pinMode(L_ENPIN, OUTPUT);

  pinMode(R_FWDPIN, OUTPUT);
  pinMode(R_BWDPIN, OUTPUT);
  pinMode(R_ENPIN, OUTPUT);
  pinMode(13, OUTPUT);

  // Start serial comms
  ibusRc.begin(IBusSerial);
  Serial.begin(BAUD_RATE);
}

ServoStruct hookStruct = {HookServo, HOOK_STARTING, 0, 120};
ServoStruct armStruct = {ClawArmServo, ARM_STARTING, 0, 120};
ServoStruct clawStruct = {ClawServo, CLAW_STARTING, 0, 120};

// Default ranges and modes for use in functions
int PLUS_MINUS_ONE = 0;
int ZERO_TO_ONE = 1;
int ANALOG_ZERO_RANGE[] = {1400, 1600};
int DISCRETE_ZERO_RANGE[] = {1250, 1750};

void loop(){
  // Controller channels, comment out the ones we don't need!
  int leftVert = readChannel(1, false);
  int rightVert = readChannel(2, false);
  int leftHorz = readChannel(3, false);
  // // int leftHorz = readChannel(4, false);
  int leftBumper = readChannel(5, false);
  int rightBumper = readChannel(6, false);
  // int right2Switch = readChannel(7, false);
  // int left3Switch = readChannel(8, false);
  int right3Switch = readChannel(9, false);

  // Processing values from controller
  // ---------------------------------
  // Left bumper: Hook
  // Right bumper: Claw
  // Right 3-switch: Arm
  // Left stick vertical axis: Direction of movement (forward / reverse) and speed
  // Left stick horizontal axis: Amount of turning
  // Right stick vertical axis: Maximum speed of wheels

  double hookControl = remapControlValues(leftBumper, DEFAULT_CHANNEL_LIMITS, ANALOG_ZERO_RANGE, PLUS_MINUS_ONE);
  hookControl *= HOOK_SENSITIVITY;
  int hookIncrement = round(hookControl);

  double clawControl = remapControlValues(rightBumper, DEFAULT_CHANNEL_LIMITS, ANALOG_ZERO_RANGE, PLUS_MINUS_ONE);
  clawControl *= CLAW_SENSITIVITY;
  int clawIncrement = round(clawControl);

  double armControl = remapControlValues(right3Switch, DEFAULT_CHANNEL_LIMITS, DISCRETE_ZERO_RANGE, PLUS_MINUS_ONE);
  armControl *= -1;   // setting to negative because down position is higher value from receiver
  armControl *= ARM_SENSITIVITY;
  int armIncrement = round(armControl);

  int rightVertZero[] = {1000, 1100};
  double speedControl = remapControlValues(rightVert, DEFAULT_CHANNEL_LIMITS, rightVertZero, ZERO_TO_ONE);
  double directionControl = remapControlValues(leftVert, DEFAULT_CHANNEL_LIMITS, ANALOG_ZERO_RANGE, PLUS_MINUS_ONE);
  double turningControl = remapControlValues(leftHorz, DEFAULT_CHANNEL_LIMITS, ANALOG_ZERO_RANGE, PLUS_MINUS_ONE);

  // Sending instructions to servo motors
  clawStruct.angle = incrementServoAngle(clawStruct, clawIncrement);
  armStruct.angle = incrementServoAngle(armStruct, armIncrement);
  hookStruct.angle = incrementServoAngle(hookStruct, hookIncrement);

  // Sending instructions to wheel motors
  move(directionControl, speedControl, turningControl);

  delay(150);
}

// ----------------------
// Custom Functions below
// ----------------------

void debugChannels() {
  int leftVert = readChannel(1, true);
  int rightVert = readChannel(2, true);
  int leftHorz = readChannel(3, true);
  // int leftHorz = readChannel(4, true);
  int leftBumper = readChannel(5, true);
  int rightBumper = readChannel(6, true);
  int right2Switch = readChannel(7, true);
  int left3Switch = readChannel(8, true);
  int right3Switch = readChannel(9, true);
  Serial.println("");
  delay(3000);
}

void testServo(Servo servo) {
  servo.write(0); // move MG996R's shaft to angle 0°
  delay(1000); // wait for one second
  servo.write(15); // move MG996R's shaft to angle 45°
  delay(2000); // wait for one second 
  // servo.write(90); // move MG996R's shaft to angle 90°
  // delay(1000); // wait for one second
  // servo.write(120); // move MG996R's shaft to angle 120°
  // delay(1000); // wait for one second
}

double remapControlValues(int input, int inLimits[2], int zeroRange[2], int mode) {
  double output;
  if (input > zeroRange[0] && input < zeroRange[1]) return 0;
  
  if (mode == PLUS_MINUS_ONE) {
    if (input <= zeroRange[0]) {
      output = map(input, inLimits[0], zeroRange[0], -1, 0);
      if (output < -1) output = -1;
      if (output > 0) output = 0;
    }
    else {
      output = map(input, zeroRange[1], inLimits[1], 0, 1);
      if (output < 0) output = 0;
      if (output > 1) output = 1;
    }
  } else if (mode == ZERO_TO_ONE) {
      output = map(input, zeroRange[1], inLimits[1], 0, 1);
      if (output < 0) output = 0;
      if (output > 1) output = 1;
  }
  return output;
}

int incrementServoAngle(ServoStruct servoStruct, int angleDifference) {
  int currentAngle = servoStruct.angle;
  int targetAngle = currentAngle + angleDifference;
  int minAngle = servoStruct.minAngle;
  int maxAngle = servoStruct.maxAngle;

  if (targetAngle > maxAngle) {targetAngle = maxAngle;}
  if (targetAngle < minAngle) {targetAngle = minAngle;}
  servoStruct.servo.write(targetAngle);
  return targetAngle;
}

void move(double dirVal, double speedVal, double turnVal) {
  // dirVal - Backwards or forwards. -1 to 1; magnitude indicates speed
  // speedVal - Maximum speed. 0 to 1
  // turnVal - Determines motor speed offsets.

  double right;
  double left;

  if (speedVal == 0 || dirVal == 0) {
    rightStop();
    leftStop();
    return;
  }

  if (dirVal < 0) {
    rightBwd();
    leftBwd();
  }
  if (dirVal > 0) {
    rightFwd();
    leftFwd();
  }

  if (turnVal < 0) {
    right = abs(dirVal * speedVal);
    left = abs(1 - (abs(turnVal)) * dirVal * speedVal);
  }

  if (turnVal > 0) {
    left = abs(1 - (abs(turnVal)) * dirVal * speedVal);
    right = abs(dirVal * speedVal);
  }

  left *= 255;
  right *= 255;

  int leftSpeed = round(left);
  int rightSpeed = round(right);

  analogWrite(L_ENPIN, leftSpeed);
  analogWrite(R_ENPIN, rightSpeed);
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
unsigned value = ibusRc.readChannel(channelNumber);
if (debug) {
    Serial.print("Channel ");
    Serial.print(channelNumber);
    Serial.print(": ");
    Serial.println(value);
}
return value;
}

