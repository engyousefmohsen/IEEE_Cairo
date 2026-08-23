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
    // Left Motor (Channel B / IN3, IN4)
    if (leftSpeed >= 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        ledcWrite(CH_ENB, leftSpeed);
    } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        ledcWrite(CH_ENB, -leftSpeed);
    }

    // Right Motor (Channel A / IN1, IN2)
    if (rightSpeed >= 0) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        ledcWrite(CH_ENA, rightSpeed);
    } else {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        ledcWrite(CH_ENA, -rightSpeed);
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
