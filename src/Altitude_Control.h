
inline float altitudeControl(float accelZ, float &altitudeMotor1Correction, float &altitudeMotor2Correction) {

    // Constants for tuning the response of the control
    const float Kp = 1.0;  // Proportional gain, adjust as necessary
    const float maxPower = 100.0;  // Maximum allowable power, adjust as necessary
    const float powerProportion = 1; // Ratio between correction number and actual motor power that needs to change

    // Calculate the desired power adjustment based on the accelerometer input
    float altitudeAdjustment = powerProportion * Kp * -accelZ;  // Assuming 9.81 m/s^2 as the gravitational constant

    // Adjust the motor power values
    altitudeMotor1Correction = altitudeAdjustment;
    altitudeMotor2Correction = altitudeMotor1Correction;
/*
    if (motor1Power > 100) motor1Power = 100;
    if (motor1Power < 0) motor1Power = 0;

    if (motor2Power > 100) motor2Power = 100;
    if (motor2Power < 0) motor2Power = 0;
*/
}