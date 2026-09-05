#include "Config.h"
#include "Motors.h"
#include "LineFollower.h"

float error = 0, lastError = 0, integral = 0;

void initSensors() {
    pinMode(IR_L, INPUT);
    pinMode(IR_M, INPUT);
    pinMode(IR_R, INPUT);
}

void runLineFollowerPID() {
    int L = digitalRead(IR_L);
    int M = digitalRead(IR_M);
    int R = digitalRead(IR_R);

    // ================= Checkpoint Logic =================
    // Stop completely on the black strap for 5 seconds[cite: 1]
    if (L == BLACK && M == BLACK && R == BLACK) {
        stopMotors();
        
        // Wait for 5 seconds
        delay(5000); 
        
        setMotorSpeeds(BASE_SPEED, BASE_SPEED);
        delay(700); // Move forward for 700ms to clear the black strap 
        
        // Reset PID variables
        error = 0;
        lastError = 0;
        integral = 0;
        
        return; // Exit this loop iteration to read fresh sensor data next time
    }

    if (L == BLACK && M == WHITE && R == WHITE)      error = -1;
    else if (L == BLACK && M == BLACK && R == WHITE) error = -2;
    else if (L == WHITE && M == BLACK && R == WHITE) error = 0;
    else if (L == WHITE && M == BLACK && R == BLACK) error = 2;
    else if (L == WHITE && M == WHITE && R == BLACK) error = 1;
    else if ((L == BLACK && M == BLACK && R == BLACK) || 
             (L == BLACK && M == WHITE && R == BLACK)) {
        stopMotors();
        return;
    }
    else {
        error = lastError; 
    }

    integral += error;
    float derivative = error - lastError;
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    lastError = error;

    if (error == 0) {
        int leftSpeed  = constrain(BASE_SPEED - output, 0, 200);
        int rightSpeed = constrain(BASE_SPEED + output, 0, 200);
        setMotorSpeeds(leftSpeed, rightSpeed);
    } else if (error < 0) {
        setMotorSpeeds(-150, 150); // Rotate Left
    } else if (error > 0) {
        setMotorSpeeds(150, -150); // Rotate Right
    }
}