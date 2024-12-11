
inline float yawControl(float gyroZ, float &yawMotor1Correction, float &yawMotor2Correction, float &yawServo1Correction, float &yawServo2Correction) {

   const float yawSensitivity = 0.1f; // Adjust as needed
    
    // Basic proportional control logic for yaw
    const float Kp = 0.5;  // Proportional gain, adjust as necessary
    const float maxPower = 100.0;  // Maximum allowable power, adjust as necessary
    const float powerProportion = 1; // Ratio between correction number and actual motor power that needs to change
    
    // Calculate the control adjustment based on the gyro input
    float yawAdjustment = powerProportion * Kp * gyroZ;

    // Modify the main motor powers and servo motor powers
    yawMotor1Correction = yawAdjustment;
    yawMotor2Correction = -1 * yawAdjustment;
    yawServo1Correction = yawAdjustment / 10;  // Reduce the effect for servo motors if needed
    yawServo2Correction = -1 * yawAdjustment / 10;

/*
    // Clamp the power values to the range [0, maxPower]
    if (yawPowerCorrection1 > 100) yawPowerCorrection1 = 100;
    if (yawPowerCorrection1 < 0) yawPowerCorrection1 = 0;

    if (yawPowerCorrection2 > 100) yawPowerCorrection2 = 100;
    if (yawPowerCorrection2 < 0) yawPowerCorrection2 = 0;

    if (yawServoCorrection1 > 100) yawServoCorrection1 = 100;
    if (yawServoCorrection1 < 0) yawServoCorrection1 = 0;

    if (yawServoCorrection2 > 100) yawServoCorrection2 = 100;
    if (yawServoCorrection2 < 0) yawServoCorrection2 = 0;\
    
    return(yawPowerCorrection1, yawPowerCorrection2, yawServoCorrection1, yawServoCorrection2);
*/
}
