#include <Arduino.h>

const int IN3 = 8;  // Motor-Pin IN3
const int IN4 = 9;  // Motor-Pin IN4

String letzteBewegung = "stop";

void setup() {
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  Serial.begin(9600);
  stopMotor();
  Serial.println("Gib 'hoch', 'runter' oder 'mitte' ein:");
}

void loop() {
  if (Serial.available()) {
    String eingabe = Serial.readStringUntil('\n');
    eingabe.trim(); // Entfernt Leerzeichen/Zeilenumbrüche

    if (eingabe == "hoch") {
      bewegeMotor("hoch", 3000);
      letzteBewegung = "hoch";
    } 
    else if (eingabe == "runter") {
      bewegeMotor("runter", 3000);
      letzteBewegung = "runter";
    } 
    else if (eingabe == "mitte") {
      if (letzteBewegung == "hoch") {
        bewegeMotor("runter", 1500);
        letzteBewegung = "stop";
      } 
      else if (letzteBewegung == "runter") {
        bewegeMotor("hoch", 1500);
        letzteBewegung = "stop";
      } 
      else {
        Serial.println("Keine vorherige Bewegung gespeichert.");
      }
    } 
    else {
      Serial.println("Ungültige Eingabe. Bitte 'hoch', 'runter' oder 'mitte' eingeben.");
    }
  }
}

void bewegeMotor(String richtung, int dauer) {
  if (richtung == "hoch") {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } 
  else if (richtung == "runter") {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  delay(dauer);
  stopMotor();
}

void stopMotor() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
