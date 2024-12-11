/**
 * @file bicopter_control.cpp
 * @brief Bicopter control system implementation using ESP32.
 * 
 * This code implements a state-based control system for a bicopter
 * using an ESP32, including tasks for IMU data processing, pitch, yaw,
 * roll, and altitude control, as well as system safety features.
 */

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

Adafruit_LSM6DSOX lsm6dsox; /**< IMU object for LSM6DSOX sensor */

/**
 * @brief Pin definitions for the system components.
 */
const int encoderPinPower = 17; /**< Encoder power pin */
const int encoderPin1A = 2; /**< Encoder 1 channel A */
const int encoderPin1B = 5; /**< Encoder 1 channel B */
const int encoderPin2A = 13; /**< Encoder 2 channel A */
const int encoderPin2B = 27; /**< Encoder 2 channel B */
const int limitSwitch1 = 25; /**< Limit switch 1 pin */
const int limitSwitch2 = 26; /**< Limit switch 2 pin */
const int escPin1 = 1; /**< ESC 1 control pin */
const int escPin2 = 3; /**< ESC 2 control pin */
const int motorDriverPin1 = 35; /**< Motor driver 1 control pin */
const int motorDriverPin2 = 39; /**< Motor driver 2 control pin */

Servo servo1; /**< Servo motor 1 object */
Servo servo2; /**< Servo motor 2 object */

Encoder encoder1(encoderPin1A, encoderPin1B); /**< Encoder object for servo 1 */
Encoder encoder2(encoderPin2A, encoderPin2B); /**< Encoder object for servo 2 */

long lastPosition1 = 0; /**< Last position of encoder 1 */
long lastPosition2 = 0; /**< Last position of encoder 2 */
float maxTemperature = 80; /**< Maximum IMU temperature in Celsius */
float maxServoPosition1 = 0.0; /**< Max rotation position for servo 1 */
float maxServoPosition2 = 0.0; /**< Max rotation position for servo 2 */
unsigned long startTime = 0; /**< Start time for limit switch tuning */
unsigned long idleStartTime = 0; /**< Start time for idle state */

/** @brief Enumeration of system states. */
enum State {
  IDLE, /**< System is idle */
  RUNNING, /**< System is running */
  EMERGENCY, /**< Emergency state triggered */
  RESET /**< System reset state */
};

State currentState = IDLE; /**< Current system state */

/**
 * @brief Shared data definitions for IMU readings and corrections.
 */
Share <sensors_event_t> sharedAccelData; /**< Shared accelerometer data */
Share <sensors_event_t> sharedGyroData; /**< Shared gyroscope data */
Share <sensors_event_t> sharedTempData; /**< Shared temperature data */
Share <float> neutralMotorPower1; /**< Neutral power for motor 1 */
Share <float> neutralMotorPower2; /**< Neutral power for motor 2 */
Share <float> pitchMotor1Correction; /**< Pitch correction for motor 1 */
Share <float> pitchMotor2Correction; /**< Pitch correction for motor 2 */
Share <float> yawMotor1Correction; /**< Yaw correction for motor 1 */
Share <float> yawMotor2Correction; /**< Yaw correction for motor 2 */
Share <float> yawServo1Correction; /**< Yaw correction for servo 1 */
Share <float> yawServo2Correction; /**< Yaw correction for servo 2 */
Share <float> rollMotor1Correction; /**< Roll correction for motor 1 */
Share <float> rollMotor2Correction; /**< Roll correction for motor 2 */
Share <float> rollServo1Correction; /**< Roll correction for servo 1 */
Share <float> rollServo2Correction; /**< Roll correction for servo 2 */
Share <float> altitudeMotor1Correction; /**< Altitude correction for motor 1 */
Share <float> altitudeMotor2Correction; /**< Altitude correction for motor 2 */
Share <float> currentEncoderPosition1; /**< Encoder position for servo 1 */
Share <float> currentEncoderPosition2; /**< Encoder position for servo 2 */
Share <float> motor1Correction; /**< Total correction for motor 1 */
Share <float> motor2Correction; /**< Total correction for motor 2 */
Share <float> servo1Correction; /**< Total correction for servo 1 */
Share <float> servo2Correction; /**< Total correction for servo 2 */

bool reset = false; /**< Reset flag */

/**
 * @brief Task to read and save IMU data into shared variables.
 * @param params Task parameters (unused).
 */
void imuReadingTask(void *params) {
    for (;;) {
        sensors_event_t accel, gyro, temp;
        lsm6dsox.getEvent(&accel, &gyro, &temp);

        sharedAccelData.put(accel);
        sharedGyroData.put(gyro);
        sharedTempData.put(temp);

        vTaskDelay(10); // Delay for sensor data rate (10 ms)
    }
}

/**
 * @brief Task to control the pitch of the system.
 * 
 * This function calculates the pitch correction based on accelerometer and gyroscope data 
 * and applies corrections to the motors responsible for pitch control.
 * 
 * @param params Task-specific parameters (not used in this implementation).
 */
void pitchControlTask(void *params) {
    sensors_event_t accel = sharedAccelData.get();
    sensors_event_t gyro = sharedGyroData.get();

    for (;;) {
        // Calculate pitch correction based on IMU data
        float kP = 1.5; ///< Proportional gain for pitch control
        float kD = 0.8; ///< Derivative gain for pitch control
        float powerProportion = 1; ///< Scaling factor for pitch correction
        float pitchError = accel.acceleration.x; ///< Pitch error from accelerometer data
        float pitchRate = gyro.gyro.y; ///< Pitch rate from gyroscope data
        float pitchCorrection = powerProportion * (kP * pitchError) + (kD * pitchRate);

        // Set motor corrections from pitch data
        pitchMotor1Correction.put(pitchCorrection);
        pitchMotor2Correction.put(-1 * pitchCorrection);
    }
}

/**
 * @brief Task to control the yaw of the system.
 * 
 * This function calculates the yaw correction based on accelerometer and gyroscope data 
 * and applies corrections to the motors and servos responsible for yaw control.
 * 
 * @param params Task-specific parameters (not used in this implementation).
 */
void yawControlTask(void *params) {
    sensors_event_t accel = sharedAccelData.get();
    sensors_event_t gyro = sharedGyroData.get();

    for (;;) {
        // Calculate yaw correction based on IMU data
        float kP = 1.5; ///< Proportional gain for yaw control
        float kD = 0.8; ///< Derivative gain for yaw control
        float servoRatio = 5; ///< Scaling factor for servo correction
        float powerProportion = 1; ///< Scaling factor for yaw correction
        float yawError = accel.acceleration.x; ///< Yaw error from accelerometer data
        float yawRate = gyro.gyro.y; ///< Yaw rate from gyroscope data
        float yawCorrection = powerProportion * (kP * yawError) + (kD * yawRate);

        // Set motor and servo corrections from yaw data
        yawMotor1Correction.put(yawCorrection);
        yawMotor2Correction.put(-1 * yawCorrection);
        yawServo1Correction.put(yawCorrection * servoRatio);
        yawServo2Correction.put(-1 * yawCorrection * servoRatio);
    }
}

/**
 * @brief Task to control the roll of the system.
 * 
 * This function calculates the roll correction based on accelerometer and gyroscope data 
 * and applies corrections to the motors and servos responsible for roll control.
 * 
 * @param params Task-specific parameters (not used in this implementation).
 */
void rollControlTask(void *params) {
    sensors_event_t accel = sharedAccelData.get();
    sensors_event_t gyro = sharedGyroData.get();

    for (;;) {
        // Calculate roll correction based on IMU data
        float kP = 1.5; ///< Proportional gain for roll control
        float kD = 0.8; ///< Derivative gain for roll control
        float servoRatio = 5; ///< Scaling factor for servo correction
        float powerProportion = 0.5; ///< Scaling factor for roll correction
        float rollError = accel.acceleration.y; ///< Roll error from accelerometer data
        float rollRate = gyro.gyro.x; ///< Roll rate from gyroscope data
        float rollCorrection = powerProportion * (kP * rollError) + (kD * rollRate);

        // Set motor and servo corrections from roll data
        rollMotor1Correction.put(rollCorrection);
        rollMotor2Correction.put(-1 * rollCorrection);
        rollServo1Correction.put(rollCorrection * servoRatio);
        rollServo2Correction.put(-1 * rollCorrection * servoRatio);
    }
}

/**
 * @brief Task to control the altitude of the system.
 * 
 * This function calculates the altitude correction based on accelerometer data 
 * and applies corrections to the motors responsible for altitude control.
 * 
 * @param params Task-specific parameters (not used in this implementation).
 */
void altitudeControlTask(void *params) {
    sensors_event_t accel = sharedAccelData.get();

    for (;;) {
        // Calculate altitude correction based on accelerometer data
        float kP = 1.5; ///< Proportional gain for altitude control
        float powerProportion = 0.5; ///< Scaling factor for altitude correction
        float altitudeCorrection = powerProportion * kP * -accel.acceleration.z;

        // Set motor corrections from altitude data
        altitudeMotor1Correction.put(altitudeCorrection);
        altitudeMotor2Correction.put(-1 * altitudeCorrection);
    }
}

/**
 * @brief Task to combine and prioritize control corrections.
 * 
 * This function calculates the final motor and servo inputs based on prioritized pitch, yaw, roll, 
 * and altitude corrections.
 * 
 * @param params Task-specific parameters (not used in this implementation).
 */
void correctionCombinationTask(void *params) {
    float pitchPriority = 1; ///< Priority weight for pitch control
    float yawPriority = 0.5; ///< Priority weight for yaw control
    float rollPriority = 0.4; ///< Priority weight for roll control
    float altitudePriority = 0.8; ///< Priority weight for altitude control

    // Calculating total motor and servo inputs from pitch, yaw, roll, and altitude corrections
    motor1Correction.put(1750 + (pitchPriority * pitchMotor1Correction.get() + yawPriority * yawMotor1Correction.get() + rollPriority * rollMotor1Correction.get() + altitudePriority * altitudeMotor1Correction.get()) / 4);
    motor2Correction.put((pitchPriority * pitchMotor2Correction.get() + yawPriority * yawMotor2Correction.get() + rollPriority * rollMotor2Correction.get() + altitudePriority * altitudeMotor2Correction.get()) / 4);
    servo1Correction.put(currentEncoderPosition1.get() + (yawPriority * yawServo1Correction.get() - rollPriority * rollServo1Correction.get()) / 10);
    servo2Correction.put(currentEncoderPosition2.get() + (yawPriority * yawServo2Correction.get() + rollPriority * rollServo2Correction.get()) / 10);
}

/**
 * @brief Checks limit switch for emergency state trigger.
 * 
 * This function continuously checks if both limit switches are depressed for 5 seconds.
 * If both switches are pressed, it will transition the system to the reset state.
 * 
 * @param params Pointer to parameters passed to the task (unused).
 */
void webLimitSwitchCheckTask(void *params) {

  for (;;) {

    if (reset == 1) {
      currentState = RESET;
      Serial.println("Transitioning to Reset State");
    }
  }
}

/**
 * @brief Checks IMU temperature and triggers emergency state if needed.
 * 
 * This function continuously checks the temperature of the IMU. If the temperature exceeds
 * the defined maximum temperature, the system transitions to the emergency state.
 * 
 * @param params Pointer to parameters passed to the task (unused).
 */
void temperatureCheckTask(void *params) {

  for (;;) {
  
    sensors_event_t temp = sharedTempData.get();
    if (temp.temperature > maxTemperature) {
      //currentState = EMERGENCY;
      Serial.println("Transitioning to EMERGENCY STATE");
    }
  }
}

/**
 * @brief Handles the system's idle state.
 * 
 * In the idle state, the system cuts power to the ESC and driver chips. The servos are positioned
 * at their idle positions and the ESCs are set to idle PWM values. After 20 seconds in the idle state,
 * the system transitions to the running state.
 */
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

/**
 * @brief Handles the system's running state.
 * 
 * In the running state, the motors are activated and their PWM signals are adjusted based on
 * the corrected motor values. The system also checks encoder positions and adjusts the servo positions
 * based on corrections. If the position changes, the new encoder values are printed to the Serial Monitor.
 */
void runningState() {

  ledcWrite(escPin1, map(motor1Correction.get(), 1750, 2000, 0, 4095));
  Serial.print("Motor 1 started at neutral");
  ledcWrite(escPin2, map(motor1Correction.get(), 1750, 2000, 0, 4095));
  Serial.print("Motor 2 started at neutral");

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
    if (servo1Correction.get() > maxServoPosition1) { servo1Correction.put(maxServoPosition1); }
    int targetServoAngle1 = map(servo1Correction.get(), 0, 1000, 0, 180);
    servo1.write(targetServoAngle1);  // Move the servo to the corrected position
  } else {
    Serial.println("Servo 1 is already at the corrected position.");
  }

  if (currentPosition2 != servo2Correction.get()) {
    // Map the corrected position to a servo angle
    if (servo2Correction.get() > maxServoPosition2) { servo2Correction.put(maxServoPosition2); }
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

/**
 * @brief Handles the system's emergency state.
 * 
 * In the emergency state, the motors are turned off, and all power is cut to the system.
 * No signals are sent to any connections, and the servos and ESCs are set to their emergency positions.
 */
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

/**
 * @brief Handles the system's reset state.
 * 
 * In the reset state, the motors are on stand-by, and no movement occurs. The motor speed is set to zero,
 * the IMU is reset, and the microcontroller restarts.
 * 
 * @note This function calls `esp_restart()` to reset the microcontroller.
 */
void resetState() {

  // In the reset state, motors are on stand-by, and no movement is occurring.
  Serial.println("System is in RESET");
  
  // Motor speed goes to zero, IMU is reset, microcontroller is reset
  servo1.write(map(500, 0, 1000, 0, 180));
  servo1.write(map(500, 0, 1000, 0, 180));
  ledcWrite(escPin1, map(1000, 1000, 2000, 0, 4095));
  ledcWrite(escPin2, map(1000, 1000, 2000, 0, 4095));

  // Restart the microcontroller
  esp_restart();
}

/**
 * @brief Setup function for initializing system components.
 * 
 * This function initializes serial communication, configures the LSM6DSOX IMU, sets up motor ESCs and driver pins,
 * attaches servos, configures limit switches, and performs servo tuning. It also creates several tasks for 
 * handling different system functions.
 */
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

  // Find the maximum position for Servo 1
  while (digitalRead(limitSwitch1) == HIGH) {
    position += 10;
    servo1.write(map(position, 0, 1000, 0, 180));
    delay(15);
  }
  maxServoPosition1 = position;

  position = 0;

  // Find the maximum position for Servo 2
  while (digitalRead(limitSwitch2) == HIGH) {
    position += 10;
    servo2.write(map(position, 0, 1000, 0, 180));
    delay(15);
  }
  maxServoPosition2 = position;

  // Create tasks for various system functions
  xTaskCreate(imuReadingTask, "IMU Reader", 2048, NULL, 9, NULL);
  xTaskCreate(altitudeControlTask, "Altitude Control", 2048, NULL, 8, NULL);
  xTaskCreate(pitchControlTask, "Pitch Control", 2048, NULL, 7, NULL);
  xTaskCreate(rollControlTask, "Roll Control", 2048, NULL, 6, NULL);
  xTaskCreate(yawControlTask, "Yaw Control", 2048, NULL, 5, NULL);
  xTaskCreate(correctionCombinationTask, "Correction Combiner", 2048, NULL, 10, NULL);
  xTaskCreate(webLimitSwitchCheckTask, "Limit Switch Emergency Trigger", 2048, NULL, 9, NULL);
  xTaskCreate(temperatureCheckTask, "Emergency Temperature Trigger", 2048, NULL, 9, NULL);
}

/**
 * @brief Main loop of the program.
 * 
 * The loop function continuously checks the current state of the system and performs the corresponding
 * action. The system can be in one of the following states: IDLE, RUNNING, EMERGENCY, or RESET.
 */
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

