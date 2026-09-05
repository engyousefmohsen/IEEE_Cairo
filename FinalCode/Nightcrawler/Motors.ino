#include "Config.h"
#include "Motors.h"


void initMotors() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    ledcSetup(CH_ENA, PWM_FREQ, PWM_RES);
    ledcSetup(CH_ENB, PWM_FREQ, PWM_RES);
    ledcAttachPin(ENA, CH_ENA);
    ledcAttachPin(ENB, CH_ENB);

    stopMotors();
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {

    // guarantees we never leave a stale/garbage value on ledcWrite().
    leftSpeed  = constrain(leftSpeed,  -PWM_MAX, PWM_MAX);
    rightSpeed = constrain(rightSpeed, -PWM_MAX, PWM_MAX);

    // <<<<<<<<<<<< Left Motor (Channel B / IN3, IN4) >>>>>>>>>>>>
    if (leftSpeed > 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        ledcWrite(CH_ENB, leftSpeed);
    } else if (leftSpeed < 0) {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        ledcWrite(CH_ENB, -leftSpeed);
    } else {
        // Explicit coast at zero speed: both direction pins LOW.
        // Never allow both IN3/IN4 HIGH at once (H-bridge shoot-through).
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        ledcWrite(CH_ENB, 0);
    }

    // <<<<<<<<<<<< Right Motor (Channel A / IN1, IN2) >>>>>>>>>>>>
    // NOTE: right motor's HIGH/LOW polarity is intentionally mirrored As To Make it drive towards the (IR sensor array).
    if (rightSpeed > 0) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        ledcWrite(CH_ENA, rightSpeed);
    } else if (rightSpeed < 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        ledcWrite(CH_ENA, -rightSpeed);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        ledcWrite(CH_ENA, 0);
    }
}

void stopMotors() {
    ledcWrite(CH_ENA, 0);
    ledcWrite(CH_ENB, 0);
    digitalWrite(IN1, LOW); 
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); 
    digitalWrite(IN4, LOW);
}