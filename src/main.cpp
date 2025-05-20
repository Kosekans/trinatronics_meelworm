#include <Arduino.h>

//pins
const int LINEAR_MOTOR_IN3 = 8;  // Motor-Pin IN3
const int LINEAR_MOTOR_IN4 = 9;  // Motor-Pin IN4

//Linear Motor
const int LINEAR_MOTOR_DUR_POS_UP = 2000; //falsch
const int LINEAR_MOTOR_DUR_POS_MID = 1000; //falsch
const int LINEAR_MOTOR_DUR_POS_DOWN = 0; //falsch
int linearMotorCurrentDurPos = 0;

void stopMotor() {
  digitalWrite(LINEAR_MOTOR_IN3, LOW);
  digitalWrite(LINEAR_MOTOR_IN4, LOW);
}

void moveLinearMotor(bool direction, int duration) {
  if (direction) {
    digitalWrite(LINEAR_MOTOR_IN3, HIGH);
    digitalWrite(LINEAR_MOTOR_IN4, LOW);
  } 
  else {
    digitalWrite(LINEAR_MOTOR_IN3, LOW);
    digitalWrite(LINEAR_MOTOR_IN4, HIGH);
  }

  delay(duration);
  stopMotor();
}

void moveToLinearMotorDurPos(int pos) {
  int durationToMove = linearMotorCurrentDurPos - pos;
  if (durationToMove > 0) {
    moveLinearMotor(false, durationToMove);
  } 
  else if (durationToMove < 0) {
    moveLinearMotor(true, -durationToMove);
  }
  linearMotorCurrentDurPos = pos;
}

void initLinearMotorPos() {
  moveLinearMotor(false, 3500);
  stopMotor();
}

void setup() {
  //linear motor
  pinMode(LINEAR_MOTOR_IN3, OUTPUT);
  pinMode(LINEAR_MOTOR_IN4, OUTPUT);
  initLinearMotorPos();
  stopMotor();
}

void loop() {
}