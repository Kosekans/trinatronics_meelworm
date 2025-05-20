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
long stepCurrentPos = 0; // Changed from int to long
int stepPulsDuration = 10; // microseconds (typical pulse width for TB6600, e.g., >4.7us)
int stepStepsDelay = 240; // microseconds (delay between pulses, controls speed. Total period = 250us => 4000 steps/sec)
const long STEP_LOADING_POS = -50000; //falsch // Changed from int to long
const long STEP_UNLOADING_POS = -1000; //falsch // Changed from int to long
const long STEP_IDLE_POS = -10000; //falsch // Changed from int to long


void limitSwitchISR() {
  unsigned long currentTime = millis();
  if ((currentTime - lastDebounceTime) >= DEBOUNCE_DELAY) {
    switchTriggered = true;
    lastDebounceTime = currentTime;
  }
}

void setStepMotorEnable(bool enable) {
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

void moveAbsSteps(long steps) {
    if (steps == 0) {
        return; // No movement needed
    }
    
    for (long i = 0; i < steps; i++) {
        digitalWrite(STEPPER_MOTOR_PUL_PIN, HIGH);
        delayMicroseconds(stepPulsDuration);
        digitalWrite(STEPPER_MOTOR_PUL_PIN, LOW);
        delayMicroseconds(stepStepsDelay);
    }
}

void moveToStepPos(long stepTargerPos){ // Changed parameter type to long
    long stepsToMove = stepTargerPos - stepCurrentPos; // Changed to long
    if (stepsToMove == 0) {
        return; // No movement needed
    }
    setStepDirection(stepsToMove > 0); // Move up if stepsToMove is positive, down if negative
    setStepMotorEnable(true); // Enable motor
    moveAbsSteps(abs(stepsToMove));
    setStepMotorEnable(false); // Disable motor
    stepCurrentPos = stepTargerPos;
}

void initStepPos() {
    setStepMotorEnable(true); // Enable motor
    while(!switchTriggered) {
        setStepDirection(true); // Move up
        moveAbsSteps(1);
    }
    setStepMotorEnable(false); // Disable motor
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
  moveToStepPos(-10000L); // Move -10000 steps down, ensure it's treated as long
  delay(20000); // Wait for 20 seconds
  switchTriggered = false; // Reset switchTriggered
  initStepPos(); // Initialize stepper motor position
}

void loop() {
}