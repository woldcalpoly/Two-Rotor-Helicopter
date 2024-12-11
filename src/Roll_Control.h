
inline float rollControl(float accelY, float gyroX, float &rollMotor1Correction, float &rollMotor2Correction, float &rollServo1Correction, float &rollServo2Correction) {

    const float kP = 1.5;
    const float kD = 0.8; // Derivative gain
    const float powerMotorProportion = 1; // Ratio between correction number and actual motor power that needs to change

    const float kP = 1.5;
    const float kD = 0.8; // Derivative gain
    const float powerServoProportion = 0.1; // Ratio between correction number and actual servo direction that needs to change

    // Calculate the pitch error using accelerometer (proportional term)
    float pitchError = accelY;

    // Calculate the rate of change of pitch using gyroscope (derivative term)
    float pitchRate = gyroX;

    // Control output based on proportional-derivative (PD) control
    float rollMotorCorrection = powerMotorProportion * (kP * pitchError) + (kD * pitchRate);
    float rollServoCorrection = powerServoProportion * (kP * pitchError) + (kD * pitchRate);


    // Modify motor powers based on correction
    rollMotor1Correction = rollMotorCorrection;
    rollMotor2Correction = rollMotor1Correction;

    rollServo1Correction = rollServoCorrection;
    rollServo2Correction = rollServo1Correction;


/*
    // Clamp motor power to valid range (0 to 100, for example)
    if (motor1Power > 100) motor1Power = 100;
    if (motor1Power < 0) motor1Power = 0;

    if (motor2Power > 100) motor2Power = 100;
    if (motor2Power < 0) motor2Power = 0;

    return (rollPowerCorrection1, rollPowerCorrection2);
*/

}