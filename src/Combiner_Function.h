
inline void combinerFunction(float &pitchMotor1Correction, float &pitchMotor2Correction,
                            float &yawMotor1Correction, float &yawMotor2Correction,
                            float &yawServo1Correction, float &yawServo2Correction, 
                            float &rollMotor1Correction, float &rollMotor2Correction,
                            float &rollServo1Correction, float  &rollServo2Correction,
                            float &altitudeMotor1Correction, float &altitudeMotor2Correction,
                            long &currentPosition1, long &currentPosition2,
                            float &motor1Correction, float &motor2Correction, 
                            float &servo1Correction, float &servo2Correction)  {
    
    float pitchPriority = 1;
    float yawPriority = 0.5;
    float rollPriority = 0.4;
    float altitudePriority = 0.8;

    motor1Correction = (pitchPriority * pitchMotor1Correction + yawPriority * yawMotor1Correction + rollPriority *  rollMotor1Correction + altitudePriority * altitudeMotor1Correction) / 4;
    motor2Correction = (pitchPriority * pitchMotor2Correction + yawPriority * yawMotor2Correction + rollPriority *  rollMotor2Correction + altitudePriority * altitudeMotor2Correction) / 4;
    servo1Correction = currentPosition1 + (yawPriority * yawServo1Correction - rollPriority * rollServo1Correction) / 10;
    servo2Correction = currentPosition2 + (yawPriority * yawServo2Correction + rollPriority * rollServo2Correction) / 10;

} 