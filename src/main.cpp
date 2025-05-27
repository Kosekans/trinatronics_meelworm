#include <Arduino.h>
//amelia and rayann
#include "HX711.h"
#include <UTFT.h>
#include <Adafruit_SCD30.h>
#include <Wire.h>
#include <DHT.h>

//Pins
#define LIMIT_SWITCH_PIN 2
#define STEPPER_MOTOR_PUL_PIN 3
#define STEPPER_MOTOR_DIR_PIN 4
#define STEPPER_MOTOR_ENA_PIN 5
#define LINEAR_MOTOR_IN3 6 // Motor-Pin IN3
#define LINEAR_MOTOR_IN4 7  // Motor-Pin IN4
// --- Définir les broches utilisées pour HX711 ---
#define HX711_DT  22
#define HX711_SCK 23

#define In1 11
#define In2 10
#define In3 9
#define In4 8

#define PeltierIn3 13
#define PeltierIn4 14

#define DHTPIN 15

//limit switch
const unsigned long DEBOUNCE_DELAY = 50;  // 50ms debounce time
volatile bool switchTriggered;
volatile unsigned long lastDebounceTime = 0;

//stepper motor
long stepCurrentPos = 0; // Changed from int to long
int stepPulsDuration = 10; // microseconds (typical pulse width for TB6600, e.g., >4.7us)
int stepStepsDelay = 240; // microseconds (delay between pulses, controls speed. Total period = 250us => 4000 steps/sec)
const long STEP_MAXDOWN_POS = -195000;
const long STEP_LOADING_POS = STEP_MAXDOWN_POS;
const long STEP_UNLOADING_POS = 0;
const long STEP_IDLE_POS = -100000;

//Linear Motor
const int LINEAR_MOTOR_DUR_POS_UP = 0;
const int LINEAR_MOTOR_DUR_POS_MID = 1500;
const int LINEAR_MOTOR_DUR_POS_DOWN = 2230;
int linearMotorCurrentDurPos = 0;

//amelia and rayann

Adafruit_SCD30 scd30;
HX711 scale;
UTFT myGLCD(ILI9486, 38, 39, 40, 41);  // Broches standard Mega 2560

extern uint8_t BigFont[];

// --- Calibration poids ---
float calibration_factor = 750;  // Ajuster par calibration

// --- Réception depuis l'Arduino 
float temp_mesuree = 0;
float humidite_mesuree = 0;
String etat_peltier = "";

#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

float targetTemp = 10.0; // Température cible en °C
float tolerance = 0.5;
String inputSerial = "";


unsigned long lastSensorRead = 0;
const unsigned long sensorInterval = 2000;

void stopLinearMotor() {
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
  stopLinearMotor();
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
  stopLinearMotor();
  linearMotorCurrentDurPos = 0;
}

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
    switchTriggered = false;
    setStepDirection(true); // Move up
    setStepMotorEnable(true); // Enable motor
    while(!switchTriggered) {
        moveAbsSteps(1);
    }
    setStepMotorEnable(false); // Disable motor
    stepCurrentPos = 0; // Set current position to 0 after hitting the limit switch
}

void larvaeTransport() { // full cycle
    moveToStepPos(STEP_LOADING_POS);
    initLinearMotorPos(); //cause pos it all the way up, calculating the pos offsets itself relativly fast due to hardware
    delay(10000); 
    moveToLinearMotorDurPos(LINEAR_MOTOR_DUR_POS_DOWN);
    initStepPos(); // cause STEP_UNLOADING_POS is at 0
    delay(10000);
    moveToStepPos(STEP_IDLE_POS);
}

void startFans() {
  analogWrite(In1, 255);
  digitalWrite(In2, LOW);
  analogWrite(In3, 255);
  digitalWrite(In4, LOW);
}

void stopFans() {
  analogWrite(In1, 0);
  digitalWrite(In2, LOW);
  analogWrite(In3, 0);
  digitalWrite(In4, LOW);

}

void setup() {
  //amelia and rayann
  Serial.begin(115200);

  // Écran
  myGLCD.InitLCD();
  myGLCD.clrScr();
  myGLCD.setFont(BigFont);
  myGLCD.setBackColor(255, 255, 255);
  myGLCD.setColor(0, 0, 0);
  myGLCD.fillScr(255, 255, 255);
  myGLCD.print("Capteurs Poids, CO2 & Temp", CENTER, 20);

  // HX711
  scale.begin(HX711_DT, HX711_SCK);
  while (!scale.is_ready()) {
    myGLCD.print("Attente HX711...", 20, 40);
    delay(500);
  }
  scale.tare();

  // SCD30
  if (!scd30.begin()) {
    myGLCD.print("SCD30 non detecte", LEFT, 60);
    while (1) delay(10);
  }

  // DHT22
  dht.begin();

  //peltier
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
  pinMode(PeltierIn3, OUTPUT);
  pinMode(PeltierIn4, OUTPUT);  

  //limit switch
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);  // Use internal pullup resistor
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN), limitSwitchISR, FALLING);
  
  //stepper motor
  /*
  pinMode(STEPPER_MOTOR_PUL_PIN, OUTPUT);
  pinMode(STEPPER_MOTOR_DIR_PIN, OUTPUT);
  pinMode(STEPPER_MOTOR_ENA_PIN, OUTPUT);
  digitalWrite(STEPPER_MOTOR_ENA_PIN, HIGH); // Disable motor initially
  moveToStepPos(-10000); //first move down a bit in case the elevator is somehow on or above the limit switch, works cause initially the current position is set to 0
  delay(1000);
  initStepPos(); // Initialize stepper motor position
  moveToStepPos(STEP_IDLE_POS);
  
  //linear motor
  pinMode(LINEAR_MOTOR_IN3, OUTPUT);
  pinMode(LINEAR_MOTOR_IN4, OUTPUT);
  initLinearMotorPos();
  moveToLinearMotorDurPos(LINEAR_MOTOR_DUR_POS_DOWN); // Move to the down position initially
  */
}

void loop() {
  //amelia and rayann
  // --- Lecture Poids ---
  long reading = scale.get_units(20);
  float poids_grammes = reading / calibration_factor;

  // --- Lecture CO₂ ---
  float co2ppm = 0;
  if (scd30.dataReady()) {
    if (scd30.read()) {
      co2ppm = scd30.CO2;
    }
  }

  unsigned long currentMillis = millis();

 
if (currentMillis - lastSensorRead >= sensorInterval) {
    temp_mesuree = dht.readTemperature();
    humidite_mesuree = dht.readHumidity();
    etat_peltier = "STABLE";

    if (!isnan(temp_mesuree)&& !isnan(humidite_mesuree)) {
      // --- Contrôle Peltier & Ventilos ---
      if (temp_mesuree < targetTemp - tolerance) {
        digitalWrite(PeltierIn3, HIGH);
        digitalWrite(PeltierIn4, LOW);
        startFans();
        etat_peltier = "CHAUFFAGE";
      } else if (temp_mesuree > targetTemp + tolerance) {
        digitalWrite(PeltierIn3, LOW);
        digitalWrite(PeltierIn4, HIGH);
        startFans();
        etat_peltier = "REFROIDISSEMENT";
      } else {
        digitalWrite(PeltierIn3, LOW);
        digitalWrite(PeltierIn4, LOW);
        stopFans();
      }
       // === Envoi structuré vers la Raspberry Pi via USB ===
      Serial.print("TEMP="); Serial.print(temp_mesuree); Serial.print(";");
      Serial.print("HUM="); Serial.print(humidite_mesuree);Serial.print(";");
      Serial.print("CIBLE="); Serial.print(targetTemp); Serial.print(";");
    // adapte si tu ajoutes l'humidité réelle
      Serial.print("CO2="); Serial.print(co2ppm); Serial.print(";");
      Serial.print("POIDS="); Serial.print(poids_grammes); Serial.print(";");
      Serial.print("ETAT="); Serial.println(etat_peltier);
    }
      lastSensorRead = currentMillis;
  } // Met à jour le temps de la dernière lecture

      // --- Envoi structuré ---
     // Serial.print(dhtTemp, 2); Serial.print(";");
      //Serial.print(dhthum, 2); Serial.print(";");
      //Serial.print(targetTemp, 2); Serial.print(";");
      //Serial.println(etat);
   // -- Réception de la température cible depuis la Raspberry --
while (Serial.available()) {
  char c = Serial.read();
  if (c == '\n') {
    inputSerial.trim();

    if (inputSerial.startsWith("SET:")) {
      String tempStr = inputSerial.substring(4);
      float newTarget = tempStr.toFloat();
      Serial.print("Conversion cible : ");
      Serial.println(newTarget);  // Affiche la valeur convertie
      if (newTarget > 0 && newTarget < 50) {  // Filtre de sécurité
        targetTemp = newTarget;
        Serial.print("Nouvelle cible prise en compte: ");
        Serial.println(targetTemp);  // Confirme la mise à jour
      } else {
        Serial.println("Valeur cible hors plage");
      }
    }
  

    inputSerial = "";  // Réinitialisation
  } else {
    inputSerial += c;
  }
}
  // --- Affichage local poids & CO2 ---
  myGLCD.setColor(255, 255, 255);
  myGLCD.fillRect(20, 40, 350, 180);  // Nettoie zone

  myGLCD.setColor(0, 0, 0);
  char buf[30];

  myGLCD.print("Poids:", 20, 60);
  dtostrf(poids_grammes, 5, 2, buf);
  myGLCD.print(buf, 220, 60);
  myGLCD.print("g", 300, 60);

  myGLCD.print("CO2:", 20, 90);
  dtostrf(co2ppm, 5, 1, buf);
  myGLCD.print(buf, 220, 90);
  myGLCD.print("ppm", 300, 90);

  myGLCD.print("Temp:", 20, 120);
  dtostrf(temp_mesuree, 5, 2, buf);
  myGLCD.print(buf, 220, 120);
  myGLCD.print("C", 300, 120);

  myGLCD.print("Cible:", 20, 150);
  dtostrf(targetTemp, 5, 2, buf);
  myGLCD.print(buf, 220, 150);
  myGLCD.print("C", 300, 150);

  myGLCD.print("Etat:", 20, 180);
  myGLCD.print(etat_peltier.c_str(), 190, 180);

  myGLCD.print("Hum:", 20, 210);
  dtostrf(humidite_mesuree, 5, 2, buf);
  myGLCD.print(buf, 220, 210);
  myGLCD.print("%", 300, 210);
}  
