#include <taskshare.h>
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <Encoder.h>
#include <Adafruit_LSM6DSOX.h>
#include <Pitch_Control.h>
#include <Yaw_Control.h>
#include <Roll_Control.h>
#include <Altitude_Control.h>
#include <Combiner_Function.h>
#include <esp_system.h>


Adafruit_LSM6DSOX lsm6dsox;

// Define pins for the system
  // Servo encoders (5 pins)
  const int encoderPinPower = 17; // 3v3 Connection
  const int encoderPin1A = 2; // Encoder 1 - Channel A
  const int encoderPin1B = 5; // Encoder 1 - Channel B
  const int encoderPin2A = 13; // Encoder 2 - Channel A
  const int encoderPin2B = 27; // Encoder 2 - Channel B

  // Servo limit switches (2 pins)
  const int limitSwitch1 = 25;
  const int limitSwitch2 = 26;

  // Motor ESCs (2 pins)
  const int escPin1 = 13; // ESC 1
  const int escPin2 = 12; // ESC 2


  // Motor driver pins (2 pins)
  const int motorDriverPin1 = 19; // Motor driver 1 control
  const int motorDriverPin2 = 18; // Motor driver 2 control


// Initialize servo objects
Servo servo1; // Driver chip 1 for servo motor 1
Servo servo2; // Driver chip 2 for servo motor 2

// Initialize encoder objects
Encoder encoder1(encoderPin1A, encoderPin1B);
Encoder encoder2(encoderPin2A, encoderPin2B);


long lastPosition1 = 0.0;
long lastPosition2 = 0.0;


float motor1Power = 0.0; // Initialize motor 1 power
float motor2Power = 0.0; // Initialize motor 2 power
float servo1Power = 0.0; 
float servo2Power = 0.0; 

float motor1Correction = 0.0; // Initialize motor 1 correction
float motor2Correction = 0.0; // Initialize motor 2 correction
float servo1Correction = 0.0; 
float servo2Correction = 0.0; 

float pitchMotor1Correction = 0.0; // Initialize motor 1 correction
float pitchMotor2Correction = 0.0; // Initialize motor 2 correction
float yawMotor1Correction = 0.0;
float yawMotor2Correction = 0.0;
float yawServo1Correction = 0.0;  
float yawServo2Correction = 0.0;
float rollMotor1Correction = 0.0;
float rollMotor2Correction = 0.0;
float rollServo1Correction = 0.0;
float rollServo2Correction = 0.0;
float altitudeMotor1Correction = 0.0;
float altitudeMotor2Correction = 0.0;

float maxTemperature = 80; // Max IMU temp is 80 degrees Celsius

float maxServoPosition1 = 0.0;
float maxServoPosition2 = 0.0;

unsigned long startTime = 0.0;
unsigned long idleStartTime = 0.0;

enum State {
  IDLE,
  RUNNING,
  EMERGENCY,
  RESET
};

// Current state of the system
State currentState = IDLE;




void setup() {

  Serial.begin(115200);
  while (!Serial) delay(10);

    Serial.println("LSM6DSOX IMU Test");

    // Try to initialize the LSM6DSOX
    if (!lsm6dsox.begin_I2C()) {  // Use I2C interface
    
      Serial.println("Failed to find LSM6DSOX chip");
      while (1) {
      
        delay(10);
      }
    }

  Serial.println("LSM6DSOX Found!");

  // Set up sensor ranges and output rates (optional)
  lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  lsm6dsox.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6dsox.setGyroDataRate(LSM6DS_RATE_104_HZ);

  Serial.println("Sensor configured!");
  
  // Initialize motor ESCs as PWM outputs
  pinMode(escPin1, OUTPUT);
  pinMode(escPin2, OUTPUT);
  
  ledcAttachPin(escPin2, 1);
  ledcSetup(1, 1000, 8);
  ledcAttachPin(escPin1, 0);
  ledcSetup(0, 1000, 8);

  // Initialize motor driver pins
  pinMode(motorDriverPin1, OUTPUT);
  pinMode(motorDriverPin2, OUTPUT);


 // Attach servos to their pins
  servo1.attach(motorDriverPin1); // Example pin for Servo 1
  servo2.attach(motorDriverPin2); // Example pin for Servo 2


  // Set up limit switches as inputs
  pinMode(limitSwitch1, INPUT_PULLUP);
  pinMode(limitSwitch2, INPUT_PULLUP);

  // Tuning servo motors to find full range of motion
  int position = 0;
  servo1.write(0);
  servo2.write(0);
  delay(500);

  while (digitalRead(limitSwitch1) == HIGH) {
    position += 20;
    servo1.write(map(position, 0, 1000, 0, 180));
    delay(15);
  }

  maxServoPosition1 = position;
  position = 0;

  while (digitalRead(limitSwitch2) == HIGH) {
    position += 20;
    servo2.write(map(position, 0, 1000, 0, 180));
    delay(15);
  }

  maxServoPosition1 = position;

}








void loop() {

  // Check the state of the system and act accordingly
  switch (currentState) {
    case IDLE:
      idleState();
      break;
    case RUNNING:
      runningState();
      break;
    case EMERGENCY:
      emergencyState();
      break;
    case RESET:
      resetState();
      break;
  }

  delay(10);

}


void limitSwitchCheck() {

  if (digitalRead(limitSwitch1) == LOW && digitalRead(limitSwitch2) == LOW) {
    if (startTime == 0) {
      startTime = millis();  // Record the current time
    }

    // Check if 5 seconds have passed since the start time
    if (millis() - startTime >= 5000) {
      currentState = RESET;
      Serial.println("Transitioning to Reset State");
      startTime = 0;  // Reset start time to avoid repeated prints
    }
  } else {
    // Reset the start time if either switch is released
    startTime = 0;
  }

}
  
void temperatureCheck(float currentTemperature) {
  if( currentTemperature > maxTemperature) {
    currentState = EMERGENCY;
    Serial.println("Transitioning to EMERGENCY STATE");
  }
}




void runningState () {

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

  long currentPosition1 = encoder1.read();  // Read the encoder's position
  long currentPosition2 = encoder2.read();

  pitchControl(accel.acceleration.x, gyro.gyro.y, pitchMotor1Correction, pitchMotor2Correction);
  yawControl(gyro.gyro.z, yawMotor1Correction, yawMotor2Correction, yawServo1Correction, yawServo2Correction);
  rollControl(accel.acceleration.y, gyro.gyro.x, rollMotor1Correction, rollMotor2Correction, rollServo1Correction, rollServo2Correction);
  altitudeControl(accel.acceleration.z, altitudeMotor1Correction, altitudeMotor2Correction);

  combinerFunction(pitchMotor1Correction, pitchMotor2Correction,
                  yawMotor1Correction, yawMotor2Correction,
                  yawServo1Correction, yawServo2Correction, 
                  rollMotor1Correction, rollMotor2Correction,
                  rollServo1Correction, rollServo2Correction,
                  altitudeMotor1Correction, altitudeMotor2Correction,
                  currentPosition1, currentPosition2,
                  motor1Correction, motor2Correction, 
                  servo1Correction, servo2Correction);

Serial.print("Motor 1 Total Correction: "); Serial.print(motor1Correction);
Serial.print("Motor 2 Total Correction: "); Serial.print(motor2Correction);
Serial.print("Servo 1 Total Correction: "); Serial.print(servo1Correction);
Serial.print("Servo 2 Total Correction: "); Serial.print(servo2Correction);


// ENCODER PWM MODIFICATION   

  // If the position changes, print the new position to the Serial Monitor
  if (currentPosition1 != lastPosition1) {
    Serial.print("Encoder Position: ");
    Serial.println(currentPosition1);
    lastPosition1 = currentPosition1;
  }

  if (currentPosition1 != servo1Correction) {
    // Map the corrected position to a servo angle
    if (servo1Correction > maxServoPosition1){ servo1Correction = maxServoPosition1; }
    int targetServoAngle1 = map(servo1Correction, 0, 1000, 0, 180);
    servo1.write(targetServoAngle1);  // Move the servo to the corrected position
  } else {
    Serial.println("Servo 2 is already at the corrected position.");
  }


  if (currentPosition2 != servo2Correction) {
    // Map the corrected position to a servo angle
    if (servo2Correction > maxServoPosition2){ servo2Correction = maxServoPosition2; }
    int targetServoAngle2 = map(servo2Correction, 0, 1000, 0, 180);
    servo2.write(targetServoAngle2);  // Move the servo to the corrected position
  } else {
    Serial.println("Servo 2 is already at the corrected position.");
  }


// THRUST MOTOR PWM MODIFICATION
  // analogWrite(escPin1, map(motor1Correction, 1000, 2000, 0, 255)); // Takes motor1Correction range of 1000-2000 to esp32 PWM resolution 0-255 for analogWrite
  // analogWrite(escPin2, map(motor2Correction, 1000, 2000, 0, 255));
  
  ledcWrite(0, map(motor1Correction, 1000, 2000, 0, 225));
  ledcWrite(0, map(motor2Correction, 1000, 2000, 0, 225));



  limitSwitchCheck();

  temperatureCheck(temp.temperature);

  delay(10);
}


void idleState () {

  Serial.println("System is in IDLE");
  // No power to ESC or DRIVER CHIPS
  servo1.write(map(500, 0, 1000, 0, 180));
  servo1.write(map(500, 0, 1000, 0, 180));
  analogWrite(escPin1, map(1500, 1000, 2000, 0, 255));
  analogWrite(escPin2, map(1500, 1000, 2000, 0, 255));

  int idleTime = 20000;

   if (idleStartTime == 0) {
    idleStartTime = millis();
  }

  if (millis() - idleStartTime >= idleTime) {
    Serial.println("20 seconds elapsed in IDLE. Switching to RUNNING.");
    currentState = RUNNING;
    idleStartTime = 0;
    return;
  }
}

void emergencyState () {

  Serial.println("System is in EMERGENCY");
  // Motor power to zero
  // Cut power from battery
  // No Signals to any connections
  servo1.write(map(500, 0, 1000, 0, 180));
  servo2.write(map(500, 0, 1000, 0, 180));
  analogWrite(escPin1, map(1000, 1000, 2000, 0, 255));
  analogWrite(escPin2, map(1000, 1000, 2000, 0, 255));
}



void resetState () {

   // In the reset state, motors are on stand-by, and no movement is occurring.
  Serial.println("System is in RESET");
  
  // Motor speed goes to zero, IMU is reset, microcontroller is reset
  servo1.write(map(500, 0, 1000, 0, 180));
  servo1.write(map(500, 0, 1000, 0, 180));
  analogWrite(escPin1, map(1000, 1000, 2000, 0, 255));
  analogWrite(escPin2, map(1000, 1000, 2000, 0, 255));

  esp_restart();
}

