#include "Config.h"
#include "Motors.h"
#include "LineFollower.h"

static float error = 0, lastError = 0, integral = 0;
static unsigned long lastPIDTime = 0;

/* Checkpoint state machine states:
    CP_NONE:    no checkpoint sequence in progress
    CP_STOPPED: robot has stopped on the black strap, waiting for 5s dwell to complete
    CP_CLEARING: robot is creeping forward to clear the strap, waiting for 200ms creep to complete
    Used This Mechanism to avoid getting stuck in a loop of stopping and starting when the robot is on the black strap.
*/
enum CheckpointState { CP_NONE, CP_STOPPED, CP_CLEARING };
static CheckpointState cpState = CP_NONE; /* Intialize the checkpoint state machine to no checkpoint sequence in progress */
static unsigned long cpTimer = 0;  /* Timer for checkpoint dwell/clear timing */

static const unsigned long CHECKPOINT_STOP_MS  = 5000; // Wait time on the black strap
static const unsigned long CHECKPOINT_CLEAR_MS = 200;  // forward-creep time to clear the strap

void initSensors() {
    pinMode(IR_L, INPUT);
    pinMode(IR_M, INPUT);
    pinMode(IR_R, INPUT);

    lastPIDTime = millis();

    Serial.println("PID Line Following Is Ready ");
}

void resetLineFollowerPID() {
    error = 0;
    lastError = 0;
    integral = 0;
    lastPIDTime = millis();   // re-sync the clock so the next dt isn't inflated by time spent in another mode
    cpState = CP_NONE;        // abandon any in-progress checkpoint sequence 3ashan Lw El mode was switched mid-pause
}

void runLineFollowerPID() {
    /*<<<<<<<<<<<<<<<< Checkpoint >>>>>>>>>>>>>>>>*/
    // If a checkpoint sequence is already underway, service it and return
    if (cpState == CP_STOPPED) {
        if (millis() - cpTimer >= CHECKPOINT_STOP_MS) {
            // Move Forward To Over The Checkpoint
            setMotorSpeeds(-BASE_SPEED, -BASE_SPEED);
            cpTimer = millis();
            cpState = CP_CLEARING;
        }
        return; /* Lessa Mestany EL 5s Tekhlas */
    }

    if (cpState == CP_CLEARING) {
        if (millis() - cpTimer >= CHECKPOINT_CLEAR_MS) {
            stopMotors();
            // Reset PID state so the next time we enter line-following mode, we don't get a jerk from stale error/integral terms
            resetLineFollowerPID();
        }
        return; // Lessa Mestany EL 200ms Tekhlas
    }

    int L = digitalRead(IR_L);
    int M = digitalRead(IR_M);
    int R = digitalRead(IR_R);

    // Enter the checkpoint
    if (L == BLACK && M == BLACK && R == BLACK) {
        stopMotors();
        cpTimer = millis();
        cpState = CP_STOPPED;
        return;
    }

    if (L == BLACK && M == WHITE && R == WHITE)      error = -1;
    else if (L == BLACK && M == BLACK && R == WHITE) error = -2;
    else if (L == WHITE && M == BLACK && R == WHITE) error = 0;
    else if (L == WHITE && M == BLACK && R == BLACK) error = 2;
    else if (L == WHITE && M == WHITE && R == BLACK) error = 1;
    else if (L == BLACK && M == WHITE && R == BLACK) 
    {
        stopMotors();
        return;
    }
    else {
        error = lastError; // sensors see nothing recognizable (line lost) - hold last known heading
    }

    /* <<<<<<<<<<<<<<<<<< Time-delta computation >>>>>>>>>>>>>>>>>>> */
    // Normalized to the nominal 10ms outer-loop period (see loop()'s
    // delay(10)) so Kp/Ki/Kd keep their existing tuning when dt == nominal.
    // Guards against dt == 0 (repeat calls) and against huge dt spikes
    // (e.g. right after the 5.7s checkpoint stall) which would otherwise
    // cause an integral/derivative kick.
        
    unsigned long now = millis();
    float dt = (now - lastPIDTime) / 10.0f;
    if (dt <= 0.0f || dt > 20.0f) dt = 1.0f;
    lastPIDTime = now;

    /* --------------------------------------------------- */

    /* <<<<<<<<<<<<<<<<<<< PID Computation >>>>>>>>>>>>>>> */
    integral += error * dt;
    integral = constrain(integral, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT); // anti-windup

    float derivative = (error - lastError) / dt;
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    lastError = error;
    /* --------------------------------------------------- */

    /*<<<<<<<<<<<<<<<<< For Fully PID Steering and Binding >>>>>>>>>>>>>>> */
    /* <<<<<<<<<<<<<<<<<< Differential Drive Output >>>>>>>>>>>>>>>>>>> */
    // Line drifted right (error < 0) -> output < 0 -> left wheel slows / right speeds up
    // Line drifted left  (error > 0) -> output > 0 -> left wheel speeds up / right slows
    //  int leftSpeed  = constrain((int)(-(BASE_SPEED - output)), -PWM_MAX, PWM_MAX); 
    //  int rightSpeed = constrain((int)(-(BASE_SPEED + output)), -PWM_MAX, PWM_MAX); 
    //  setMotorSpeeds(leftSpeed, rightSpeed);                                        
    
    if (error == 0) {
    // Forward (normal PID)
    int leftSpeed  = constrain(BASE_SPEED - output, 0, 200);
    int rightSpeed = constrain(BASE_SPEED + output, 0, 200);

    ledcWrite(CH_ENA, rightSpeed);
    ledcWrite(CH_ENB, leftSpeed);

    digitalWrite(IN1, HIGH);  // Right forward
    digitalWrite(IN2, LOW);   
    digitalWrite(IN3, LOW); 
    digitalWrite(IN4, HIGH);  // Left forward

  } else if (error < 0) {
    // Rotate left in place
    int turnSpeed = 120;
    ledcWrite(CH_ENA, turnSpeed);  
    ledcWrite(CH_ENB, turnSpeed);  

    digitalWrite(IN1, HIGH); 
    digitalWrite(IN2, LOW);   // Right forward
    digitalWrite(IN3, HIGH); 
    digitalWrite(IN4, LOW);   // Left backward

  } else if (error > 0) {
    int turnSpeed = 120;
    // Rotate right in place
    ledcWrite(CH_ENA, turnSpeed);  
    ledcWrite(CH_ENB, turnSpeed);  

    digitalWrite(IN1, LOW); 
    digitalWrite(IN2, HIGH);  // Right backward
    digitalWrite(IN3, LOW); 
    digitalWrite(IN4, HIGH);  // Left forward
  }

    /*<<<<<<<<<<<<<<< Serial For Easier Debugging >>>>>>>>>>>>>>> */
    Serial.print("L="); Serial.print(L);
    Serial.print(" M="); Serial.print(M);
    Serial.print(" R="); Serial.print(R);
    Serial.print(" | Err="); Serial.print(error);
    Serial.print(" | Out="); Serial.println(output);
}   