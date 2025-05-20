#include <Arduino.h>

//Pins
const int LIMIT_SWITCH_PIN = 2;
const int STEPPER_MOTOR_PUL_PIN = 3;
const int STEPPER_MOTOR_DIR_PIN = 4;
const int STEPPER_MOTOR_ENA_PIN = 5;

//limit switch
const unsigned long DEBOUNCE_DELAY = 50;  // 50ms debounce time
volatile bool switchTriggered;
volatile unsigned long lastDebounceTime = 0;

//stepper motor
int stepCurrentPos = 0;
int stepPulsDuration = 10; // microseconds (typical pulse width for TB6600, e.g., >4.7us)
int stepStepsDelay = 240; // microseconds (delay between pulses, controls speed. Total period = 250us => 4000 steps/sec)
int stepLoadingPos = -50000; //falsch
int stepUnloadingPos = -1000; //falsch
int stepIdlePos = -10000; //falsch


void limitSwitchISR() {
  unsigned long currentTime = millis();
  if ((currentTime - lastDebounceTime) >= DEBOUNCE_DELAY) {
    switchTriggered = true;
    lastDebounceTime = currentTime;
  }
}

void setMotorEnable(bool enable) {
    if (enable) {
        digitalWrite(STEPPER_MOTOR_ENA_PIN, LOW); // Enable motor
    } else {
        digitalWrite(STEPPER_MOTOR_ENA_PIN, HIGH); // Disable motor
    }
}

void setStepDirection(bool direction) { //true=up
     if (!direction) {//up
        digitalWrite(STEPPER_MOTOR_DIR_PIN, HIGH); // Set direction to forward
    } else {//down
        digitalWrite(STEPPER_MOTOR_DIR_PIN, LOW); // Set direction to backward
    }
}

void moveAbsSteps(int steps) {
    if (steps == 0) {
        return; // No movement needed
    }
    
    for (int i = 0; i < steps; i++) {
        digitalWrite(STEPPER_MOTOR_PUL_PIN, HIGH);
        delayMicroseconds(stepPulsDuration);
        digitalWrite(STEPPER_MOTOR_PUL_PIN, LOW);
        delayMicroseconds(stepStepsDelay);
    }
}

void moveToStepPos(int stepTargerPos){
    int stepsToMove = stepTargerPos - stepCurrentPos;
    if (stepsToMove == 0) {
        return; // No movement needed
    }
    setStepDirection(stepsToMove > 0); // Move up if stepsToMove is positive, down if negative
    setMotorEnable(true); // Enable motor
    moveAbsSteps(abs(stepsToMove));
    setMotorEnable(false); // Disable motor
    stepCurrentPos = stepTargerPos;
}

void initStepPos() {
    setMotorEnable(true); // Enable motor
    while(!switchTriggered) {
        setStepDirection(true); // Move up
        moveAbsSteps(1);
    }
    setMotorEnable(false); // Disable motor
    stepCurrentPos = 0; // Set current position to 0 after hitting the limit switch
}

void setup() {
  Serial.begin(9600);

  //limit switch
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);  // Use internal pullup resistor
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN), limitSwitchISR, FALLING);
  
  //stepper motor
  pinMode(STEPPER_MOTOR_PUL_PIN, OUTPUT);
  pinMode(STEPPER_MOTOR_DIR_PIN, OUTPUT);
  pinMode(STEPPER_MOTOR_ENA_PIN, OUTPUT);
  digitalWrite(STEPPER_MOTOR_ENA_PIN, HIGH); // Disable motor initially
  //first move down a bit in case the elevator is somehow on or above the limit switch
  moveToStepPos(-10000); // Move -1000 steps down
  delay(20000); // Wait for 20 seconds
  switchTriggered = false; // Reset switchTriggered
  initStepPos(); // Initialize stepper motor position
}

void loop() {
}