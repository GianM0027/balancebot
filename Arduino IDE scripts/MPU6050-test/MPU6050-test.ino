#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  delay(1000); // Piccola pausa all'avvio per la seriale dell'ESP32

  Serial.println("\nInizializzazione MPU-6050 su ESP32...");

  // Avvio del bus I2C sui pin standard (GPIO 21 SDA, GPIO 22 SCL)
  Wire.begin(21, 22);

  // Inizializzazione sensore (indirizzo I2C 0x68)
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("Impossibile trovare l'MPU-6050! Verifica i collegamenti e l'indirizzo I2C.");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU-6050 trovato e pronto!");

  // Configurazione dei range e filtro
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);   // +-2G, +-4G, +-8G, +-16G
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);        // +-250, +-500, +-1000, +-2000 deg/s
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);     // Filtro passa-basso integrato

  delay(100);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Accelerazione (m/s^2) */
  Serial.print("Acc (m/s^2): X=");
  Serial.print(a.acceleration.x, 2);
  Serial.print(" | Y=");
  Serial.print(a.acceleration.y, 2);
  Serial.print(" | Z=");
  Serial.print(a.acceleration.z, 2);

  /* Giroscopio convertito in gradi al secondo (deg/s) */
  Serial.print("  ||  Gyro (deg/s): X=");
  Serial.print(g.gyro.x * SENSORS_RADS_TO_DPS, 2);
  Serial.print(" | Y=");
  Serial.print(g.gyro.y * SENSORS_RADS_TO_DPS, 2);
  Serial.print(" | Z=");
  Serial.println(g.gyro.z * SENSORS_RADS_TO_DPS, 2);

  delay(50); // ~20 Hz per un aggiornamento fluido
}
