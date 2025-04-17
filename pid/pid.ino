#include <Servo.h>
#include <Wire.h>
#include "Adafruit_VL53L1X.h"

// Pines del VL53L1X
#define IRQ_PIN 2
#define XSHUT_PIN 3

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

// PID
#define Umin -66
#define Umax 66
#define Umax_rad 1.151
#define Umin_rad -1.151
#define T 0.09

Servo servo;

double setpoint = 0.10; // 25 cm
double setpoint_prec = setpoint;
double y, y_prec;
double error;
double P, I, D, U;
double I_prec = 0, U_prec = 0, D_prec = 0;
boolean Saturation = false;

double Kp = 5.6;
double Ki = 1.1;
double Kd = 6.3;

float average_distance(uint8_t muestras = 5);
void move_servo(int);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  servo.attach(9);

  if (!vl53.begin(0x29, &Wire)) {
    Serial.println(F("Error al iniciar VL53L1X"));
    while (1) delay(10);
  }

  vl53.setTimingBudget(50);
  vl53.startRanging();

  delay(1000);
  move_servo(90);
  delay(2000);

  y_prec = average_distance();
}

void loop() {
  y = average_distance();
  y = 0.53 * y + 0.47 * y_prec;

  error = round(100 * (y - setpoint)) * 0.01;

  // PID
  P = Kp * error;
  if (!Saturation) I = I_prec + T * Ki * error;
  D = (Kd / T) * (y - y_prec);
  D = 0.56 * D + 0.44 * D_prec;

  U = P + I + round(100 * D) * 0.01;

  if (U < Umin_rad) {
    U = Umin_rad;
    Saturation = true;
  } else if (U > Umax_rad) {
    U = Umax_rad;
    Saturation = true;
  } else {
    Saturation = false;
  }

  U = round(U * 180 / M_PI);
  U = map(U, Umin, Umax, 24, 156);

  if (U < 83 || U > 95 || abs(error) > 0.02)
    move_servo(round(U));

  delay(24);

  Serial.print("Distancia (cm): ");
  Serial.print(y * 100);
  Serial.print(" | Error: ");
  Serial.print(error);
  Serial.print(" | U: ");
  Serial.println(U);

  I_prec = I;
  y_prec = y;
}

float average_distance(uint8_t muestras) {
  long suma = 0;
  uint8_t validas = 0;

  for (uint8_t i = 0; i < muestras; i++) {
    if (vl53.dataReady()) {
      int16_t d = vl53.distance();
      vl53.clearInterrupt();

      if (d > 0 && d < 2000) { // Ignorar lecturas inválidas
        suma += d;
        validas++;
      }
    }
    delay(5); // Espera pequeña entre lecturas
  }

  if (validas == 0) return y_prec; // fallback
  return (suma / (float)validas) / 1000.0; // convertir a metros
}

void move_servo(int u) {
  servo.write(u - map(u, 30, 150, 14, 3));
}
