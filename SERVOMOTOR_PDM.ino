#include <ESP32Servo.h>
// R1 cerrado en 1 abierto 65
// R2 cerrado en 60
// R3 cerrado en 65
// R4 cerrado en 0 abierto 65



Servo miServo;
const int pinServo = 10;

void setup() {
  Serial.begin(9600);
  
  // El ESP32 configurará el timer automáticamente aquí
  miServo.attach(pinServo); 

  Serial.println("Escribe un angulo entre 0 y 180 y presiona Enviar.");
}

void loop() {
  if (Serial.available() > 0) {
    int angulo = Serial.parseInt();

    if (angulo >= 0 && angulo <= 180) {
      miServo.write(angulo);
      Serial.print("Servo movido a: ");
      Serial.println(angulo);
    } else {
      Serial.println("Angulo fuera de rango. Usa 0 a 180.");
    }

    while (Serial.available() > 0) {
      Serial.read();
    }
  }
}