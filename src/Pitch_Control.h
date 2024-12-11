
inline void pitchControl(float accelX, float gyroY, float &pitchMotor1Correction, float &pitchMotor2Correction) {

    const float kP = 1.5;
    const float kD = 0.8; // Derivative gain
    const float powerProportion = 1; // Ratio between correction number and actual motor power that needs to change

    // Calculate the pitch error using accelerometer (proportional term)
    float pitchError = accelX;

    // Calculate the rate of change of pitch using gyroscope (derivative term)
    float pitchRate = gyroY;

    // Control output based on proportional-derivative (PD) control
    float pitchCorrection =  powerProportion * (kP * pitchError) + (kD * pitchRate);


    // Modify motor powers based on correction
    pitchMotor1Correction = pitchCorrection;
    pitchMotor2Correction = -1 * pitchCorrection;
/*
    // Clamp motor power to valid range (0 to 100, for example)
    if (motor1Power > 100) motor1Power = 100;
    if (motor1Power < 0) motor1Power = 0;

    if (motor2Power > 100) motor2Power = 100;
    if (motor2Power < 0) motor2Power = 0;
*/
}