#include <Arduino.h>
#include <ESP32Servo.h>
#include "CueMechanism.h"

Servo myServo;
float currentAngle = 0.0;
unsigned long lastServoTime = 0;

void initMechanism(int servoPin) {
    myServo.setPeriodHertz(50);
    myServo.attach(servoPin, 500, 2400);
    myServo.write(0);
    lastServoTime = millis();
}

void updateMechanism(bool buttonHeld) {
    unsigned long currentTime = millis();
    unsigned long dt = currentTime - lastServoTime;
    lastServoTime = currentTime;

    if (buttonHeld) {
        if (currentAngle < 180.0) {
            currentAngle += (30.0 / 1000.0) * dt; 
            
            if (currentAngle > 180.0) {
                currentAngle = 180.0;
            }
            
            int pulse = 500 + (currentAngle / 180.0) * (2400 - 500);
            myServo.writeMicroseconds(pulse);
        }
    } else {
        if (currentAngle > 0.0) {
            currentAngle = 0.0;
            myServo.write(0);
        }
    }
}