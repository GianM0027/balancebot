// MPU-6050 logger: prints raw readings as CSV at a fixed rate, for noise and bias analysis.
// Leave the sensor STILL while logging. Read the output with tools/serial_log.py.
//
// CSV columns: t_us, ax, ay, az [m/s^2], gx, gy, gz [rad/s]
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

const uint32_t PERIOD_US = 5000;   // 200 Hz, the planned control-loop rate
uint32_t next_us;

void setup() {
  Serial.begin(921600);            // 115200 baud carries only ~120 lines/s of this size
  delay(1000);
  Wire.begin(21, 22);
  Wire.setClock(400000);           // I2C fast mode: one full read takes ~0.4 ms instead of ~1.5 ms
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("# MPU-6050 not found");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);   // see note in the chat: 44 Hz = ~5 ms delay
  delay(100);
  Serial.println("t_us,ax,ay,az,gx,gy,gz");
  next_us = micros();
}

void loop() {
  while ((int32_t)(micros() - next_us) < 0) {}   // wait for the next period (fixed rate, no drift)
  uint32_t t = micros();
  next_us += PERIOD_US;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.printf("%lu,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f\n", (unsigned long)t,
                a.acceleration.x, a.acceleration.y, a.acceleration.z,
                g.gyro.x, g.gyro.y, g.gyro.z);
}
