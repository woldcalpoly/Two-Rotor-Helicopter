#include <Adafruit_LSM6DSOX.h>

void imuReadingTask(void *params) {
  
  for (;;) {

    // Create sensor event objects
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    // Get sensor data
    lsm6dsox.getEvent(&accel, &gyro, &temp);

    // Print accelerometer data
    Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" m/s^2\t");
    Serial.print("Y: "); Serial.print(accel.acceleration.y); Serial.print(" m/s^2\t");
    Serial.print("Z: "); Serial.print(accel.acceleration.z); Serial.println(" m/s^2");

    // Print gyroscope data
    Serial.print("Gyro X: "); Serial.print(gyro.gyro.x); Serial.print(" rad/s\t");
    Serial.print("Y: "); Serial.print(gyro.gyro.y); Serial.print(" rad/s\t");
    Serial.print("Z: "); Serial.print(gyro.gyro.z); Serial.println(" rad/s");

    // Print temperature data
    Serial.print("Temp: "); Serial.print(temp.temperature); Serial.println(" C");
  }
}