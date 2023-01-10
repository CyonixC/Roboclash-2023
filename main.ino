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
const int ARM_PIN = 0;
const int CLAW_PIN = 0;

// Servos
Servo HookServo;
Servo ClawArmServo;
Servo ClawServo;

//Controller settings
byte interruptPin = 3;
byte channelAmount = 10;
PPMReader ppm(interruptPin, channelAmount);

void setup(){
    // Attach servo pins
    HookServo.attach(HOOK_PIN);
    ClawArmServo.attach(ARM_PIN);
    ClawServo.attach(CLAW_PIN);

    // Set L298N motor pins to OUTPUT
    pinMode(L_FWDPIN, OUTPUT);
    pinMode(L_BWDPIN, OUTPUT);
    pinMode(L_ENPIN, OUTPUT);

    pinMode(R_FWDPIN, OUTPUT);
    pinMode(R_BWDPIN, OUTPUT);
    pinMode(R_ENPIN, OUTPUT);

    // Start serial comms
    Serial.begin(BAUD_RATE);
}

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
    // int right3Switch = readChannel(10, false);

    debug();

    
}

void debug() {
    int rightHorz = readChannel(1, true);
    int leftVert = readChannel(2, true);
    int rightVert = readChannel(3, true);
    int leftHorz = readChannel(4, true);
    int leftBumper = readChannel(6, true);
    int rightBumper = readChannel(7, true);
    int left3Switch = readChannel(8, true);
    int right2Switch = readChannel(9, true);
    Serial.println("");
    delay(1000);
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

