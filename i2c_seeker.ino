#include <Wire.h>

void setup() {
  Wire.begin();            // Inicia el bus I2C
  Serial.begin(115200);    // Inicia la comunicación serial
  while (!Serial);         // Espera a que el monitor serial esté listo

  Serial.println("Escaneando bus I2C...");
}

void loop() {
  byte error, address;
  int count = 0;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Dispositivo I2C encontrado en 0x");
      Serial.println(address, HEX);
      count++;
    } else if (error == 4) {
      Serial.print("Error desconocido en la dirección 0x");
      Serial.println(address, HEX);
    }
  }

  if (count == 0)
    Serial.println("No se encontraron dispositivos I2C.");

  else
    Serial.println("Escaneo completado.");

  delay(5000); // Espera 5 segundos antes de volver a escanear
}

