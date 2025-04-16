#include <Wire.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include "Adafruit_VL53L1X.h"

#define IRQ_PIN 2
#define XSHUT_PIN 3

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);
LiquidCrystal_I2C lcd(0x27, 20, 4);
Servo motor;

String input = "";
int angulo = 90;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Iniciando...");

  if (!vl53.begin(0x29, &Wire)) {
    lcd.setCursor(0, 1);
    lcd.print("Fallo sensor VL53");
    while (1);
  }

  vl53.setTimingBudget(50);
  if (!vl53.startRanging()) {
    lcd.setCursor(0, 1);
    lcd.print("Fallo en Ranging");
    while (1);
  }

  motor.attach(9);
  motor.write(angulo);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sistema listo");
}

void loop() {
  // Leer ángulo desde MATLAB
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      int val = input.toInt();
      if (val >= 0 && val <= 180) {
        angulo = val;
        motor.write(angulo);
      }
      input = "";
    } else {
      input += c;
    }
  }

  // Leer distancia
  if (vl53.dataReady()) {
    int dist = vl53.distance(); // mm

    if (dist != -1) {
      // Enviar distancia y ángulo como una sola línea
      Serial.println(String(dist) + "," + String(angulo));

      // Mostrar en LCD
      lcd.setCursor(0, 0);
      lcd.print("Dist: ");
      lcd.print(dist);
      lcd.print(" mm   ");

      lcd.setCursor(0, 1);
      lcd.print("Servo: ");
      lcd.print(angulo);
      lcd.print("°     ");
    }

    vl53.clearInterrupt();
  }

  delay(50);
}

