#include <PrintStream.h>
#include <taskshare.h>
#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Encoder.h>
#include <Adafruit_LSM6DSOX.h>
#include <esp_system.h>
#include <WiFi.h>
#include <PubSubClient.h>


// Wi-Fi credentials
const char* ssid = "BiCopter_Inflight_WiFi";
const char* password = "12345678";

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
  const int escPin1 = 1; // ESC 1
  const int escPin2 = 3; // ESC 2


  // Motor driver pins (2 pins)
  const int motorDriverPin1 = 35; // Motor driver 1 control
  const int motorDriverPin2 = 39; // Motor driver 2 control


// Initialize servo objects
Servo servo1; // Driver chip 1 for servo motor 1
Servo servo2; // Driver chip 2 for servo motor 2

// Initialize encoder objects
Encoder encoder1(encoderPin1A, encoderPin1B);
Encoder encoder2(encoderPin2A, encoderPin2B);


long lastPosition1 = 0.0; // Last position of encoder on servo motors
long lastPosition2 = 0.0;

float maxTemperature = 80; // Max IMU temp is 80 degrees Celsius

float maxServoPosition1 = 0.0; // Limit position for max rotation of servo motors
float maxServoPosition2 = 0.0;

unsigned long startTime = 0.0;  // Time values for limit switch tuning
unsigned long idleStartTime = 0.0;

enum State { // Declaring states 
  IDLE,
  RUNNING,
  EMERGENCY,
  RESET
};

// Current state of the system
State currentState = IDLE;


// Shared data for IMU readings
Share <sensors_event_t> sharedAccelData;
Share <sensors_event_t> sharedGyroData;
Share <sensors_event_t> sharedTempData;

// Shared data for pitch correction
Share <float> neutralMotorPower1;
Share <float> neutralMotorPower2;
Share <float> pitchMotor1Correction;
Share <float> pitchMotor2Correction;
Share <float> yawMotor1Correction;
Share <float> yawMotor2Correction;
Share <float> yawServo1Correction;
Share <float> yawServo2Correction;
Share <float> rollMotor1Correction;
Share <float> rollMotor2Correction;
Share <float> rollServo1Correction;
Share <float> rollServo2Correction;
Share <float> altitudeMotor1Correction;
Share <float> altitudeMotor2Correction;
Share <float> currentEncoderPosition1;
Share <float> currentEncoderPosition2;
Share <float> motor1Correction;
Share <float> motor2Correction;
Share <float> servo1Correction;
Share <float> servo2Correction;

bool reset;

// Function to read and save IMU data into shared variables
void imuReadingTask(void *params) {
    

    for (;;) {
        sensors_event_t accel, gyro, temp;
        lsm6dsox.getEvent(&accel, &gyro, &temp);

        // Share sensor data with other tasks
        sharedAccelData.put(accel);
        sharedGyroData.put(gyro);
        sharedTempData.put(temp);

        vTaskDelay(10); // Delay for sensor data rate (10 ms)
    }
}

void pitchControlTask(void *params) {
    sensors_event_t accel = sharedAccelData.get();
    sensors_event_t gyro = sharedGyroData.get();

    for (;;) {
       
      // Calculate pitch correction based on IMU data
      float kP = 1.5;
      float kD = 0.8;
      float powerProportion = 1;
      float pitchError = accel.acceleration.x;
      float pitchRate = gyro.gyro.y;
      float pitchCorrection = powerProportion * (kP * pitchError) + (kD * pitchRate);

      // Set motor corrections from pitch data
      pitchMotor1Correction.put(pitchCorrection);
      pitchMotor2Correction.put(-1 * pitchCorrection);

    }
}

void yawControlTask(void *params) {
  sensors_event_t accel = sharedAccelData.get();
  sensors_event_t gyro = sharedGyroData.get();

  for (;;) {
      
    // Calculate yaw correction based on IMU data
    float kP = 1.5;
    float kD = 0.8;
    float servoRatio = 5;
    float powerProportion = 1;
    float yawError = accel.acceleration.x;
    float yawRate = gyro.gyro.y;
    float yawCorrection = powerProportion * (kP * yawError) + (kD * yawRate);

    // Set motor and servo corrections from yaw data
    yawMotor1Correction.put(yawCorrection);
    yawMotor2Correction.put(-1 * yawCorrection);
    yawServo1Correction.put(yawCorrection * servoRatio);
    yawServo2Correction.put(-1 * yawCorrection * servoRatio);               
  }
}

void rollControlTask(void *params) {
sensors_event_t accel = sharedAccelData.get();
sensors_event_t gyro = sharedGyroData.get();

  for (;;) {
      
    // Calculate roll correction based on IMU data
    float kP = 1.5;
    float kD = 0.8;
    float servoRatio = 5;
    float powerProportion = 0.5;
    float rollError = accel.acceleration.y;
    float rollRate = gyro.gyro.x;
    float rollCorrection = powerProportion * (kP * rollError) + (kD * rollRate);

    // Set motor and servo corrections from roll data
    rollMotor1Correction.put(rollCorrection);
    rollMotor2Correction.put(-1 * rollCorrection);
    rollServo1Correction.put(rollCorrection * servoRatio);
    rollServo2Correction.put(-1 * rollCorrection * servoRatio);               
  }
}

void altitudeControlTask(void *params) {
sensors_event_t accel = sharedAccelData.get();

  for (;;) {
      
    // Calculate roll correction based on IMU data
    float kP = 1.5;
    float kD = 0.8;
    float servoRatio = 5;
    float powerProportion = 0.5;
    float altitudeCorrection = powerProportion * kP * -accel.acceleration.z;

    // Set motor corrections from altitude data
    altitudeMotor1Correction.put(altitudeCorrection);
    altitudeMotor2Correction.put(-1 * altitudeCorrection);
                 
  }
}

void correctionCombinationTask (void *params) {

  float pitchPriority = 1;
  float yawPriority = 0.5;
  float rollPriority = 0.4;
  float altitudePriority = 0.8;

  // Calculating total motor and servo inputs from pitch, yaw, roll, and altitude corrections
  motor1Correction.put(1750 + (pitchPriority * pitchMotor1Correction.get() + yawPriority * yawMotor1Correction.get() + rollPriority *  rollMotor1Correction.get() + altitudePriority * altitudeMotor1Correction.get()) / 4);
  motor2Correction.put((pitchPriority * pitchMotor2Correction.get() + yawPriority * yawMotor2Correction.get() + rollPriority *  rollMotor2Correction.get() + altitudePriority * altitudeMotor2Correction.get()) / 4);
  servo1Correction.put(currentEncoderPosition1.get() + (yawPriority * yawServo1Correction.get() - rollPriority * rollServo1Correction.get()) / 10);
  servo2Correction.put(currentEncoderPosition2.get() + (yawPriority * yawServo2Correction.get() + rollPriority * rollServo2Correction.get()) / 10); 
}

// Checks limit switch for emergency state trigger (both limit switches depressed continuously for 5 seconds)
void webLimitSwitchCheckTask(void *params) {

  for (;;) {

    if (reset==1) {

      currentState = RESET;
      Serial.println("Transitioning to Reset State");

    }
  }
}

// Checks IMU temperature, if exceeds max temperature, triggers emergency state.  
void temperatureCheckTask(void *params) {

  for (;;) {
  
    sensors_event_t temp = sharedTempData.get();
    if( temp.temperature > maxTemperature) {

      //currentState = EMERGENCY;
      Serial.println("Transitioning to EMERGENCY STATE");
    }
  }
}

void idleState() {

  Serial.println("System is in IDLE");
  // No power to ESC or DRIVER CHIPS
  servo1.write(map(500, 0, 1000, 0, 180));
  servo1.write(map(500, 0, 1000, 0, 180));
  ledcWrite(escPin1, map(1500, 1000, 2000, 0, 255));
  ledcWrite(escPin2, map(1500, 1000, 2000, 0, 255));

  int idleTime = 2000;

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

void runningState() {

  ledcWrite(escPin1, map(motor1Correction.get(), 1750, 2000, 0, 4095));
  Serial.print("Motor 1 started at nuetral");
  ledcWrite(escPin2, map(motor1Correction.get(), 1750, 2000, 0, 4095));
  Serial.print("Motor 2 started at nuetral");



// ENCODER PWM MODIFICATION   
  long currentPosition1 = encoder1.read();  // Read the encoder's position
  long currentPosition2 = encoder2.read();

  // If the position changes, print the new position to the Serial Monitor
  if (currentPosition1 != lastPosition1) {
    Serial.print("Encoder Position: ");
    Serial.println(currentPosition1);
    lastPosition1 = currentPosition1;
  }

  if (currentPosition1 != servo1Correction.get()) {
    // Map the corrected position to a servo angle
    if (servo1Correction.get() > maxServoPosition1){ servo1Correction.put(maxServoPosition1); }
    int targetServoAngle1 = map(servo1Correction.get(), 0, 1000, 0, 180);
    servo1.write(targetServoAngle1);  // Move the servo to the corrected position
  } else {
    Serial.println("Servo 2 is already at the corrected position.");
  }


  if (currentPosition2 != servo2Correction.get()) {
    // Map the corrected position to a servo angle
    if (servo2Correction.get() > maxServoPosition2){ servo2Correction.put(maxServoPosition2); }
    int targetServoAngle2 = map(servo2Correction.get(), 0, 1000, 0, 180);
    servo2.write(targetServoAngle2);  // Move the servo to the corrected position
  } else {
    Serial.println("Servo 2 is already at the corrected position.");
  }


// THRUST MOTOR PWM MODIFICATION
  ledcWrite(escPin1, map(motor1Correction.get(), 1000, 2000, 0, 4095));
  ledcWrite(escPin2, map(motor2Correction.get(), 1000, 2000, 0, 4095));

  delay(10);
}

void emergencyState() {

  Serial.println("System is in EMERGENCY");
  // Motor power to zero
  // Cut power from battery
  // No Signals to any connections
  servo1.write(map(500, 0, 1000, 0, 180));
  servo2.write(map(500, 0, 1000, 0, 180));
  ledcWrite(escPin1, map(1000, 1000, 2000, 0, 4095));
  ledcWrite(escPin2, map(1000, 1000, 2000, 0, 4095));
}

void resetState() {

   // In the reset state, motors are on stand-by, and no movement is occurring.
  Serial.println("System is in RESET");
  
  // Motor speed goes to zero, IMU is reset, microcontroller is reset
  servo1.write(map(500, 0, 1000, 0, 180));
  servo1.write(map(500, 0, 1000, 0, 180));
  ledcWrite(escPin1, map(1000, 1000, 2000, 0, 4095));
  ledcWrite(escPin2, map(1000, 1000, 2000, 0, 4095));

  esp_restart();
}



void setup() {
  
  Serial.begin(115200);
  while (!Serial) delay(10);
  
    Serial.println("SERIAL CONNECTED!");
  
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

  ledcAttachPin(escPin1, 0);
  ledcSetup(0, 1000, 12);
  ledcAttachPin(escPin2, 1);
  ledcSetup(1, 1000, 12);


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
    position += 10;
    servo1.write(map(position, 0, 1000, 0, 180));
    delay(15);
  }

  maxServoPosition1 = position;
  position = 0;

  while (digitalRead(limitSwitch2) == HIGH) {
    position += 10;
    servo2.write(map(position, 0, 1000, 0, 180));
    delay(15);
  }

  maxServoPosition2 = position;

  xTaskCreate(imuReadingTask, "IMU Reader", 2048, NULL, 9, NULL);
  xTaskCreate(altitudeControlTask, "Altitude Control", 2048, NULL, 8, NULL);
  xTaskCreate(pitchControlTask, "Pitch Control", 2048, NULL, 7, NULL);
  xTaskCreate(rollControlTask, "Roll Control", 2048, NULL, 6, NULL);
  xTaskCreate(yawControlTask, "Yaw Control", 2048, NULL, 5, NULL);
  xTaskCreate(correctionCombinationTask, "Correction Combiner", 2048, NULL, 10, NULL);
  xTaskCreate(webLimitSwitchCheckTask, "Limit Switch Emergency Trigger", 2048, NULL, 9, NULL);
  xTaskCreate(temperatureCheckTask, "Emergy Temperature Trigger", 2048, NULL, 9, NULL);
  
}

void loop() {

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

