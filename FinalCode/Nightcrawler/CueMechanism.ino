#include <Arduino.h>
#include <ESP32Servo.h>
#include "CueMechanism.h"

Servo myServo;
float currentAngle = 0.0;
unsigned long lastServoTime = 0;
static bool lastButtonHeld = false; // tracks previous button state so we only log edges, not every 10ms tick

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

    /*<<<<<<<<<<<<<<< Serial For Easier Debugging >>>>>>>>>>>>>>> */
    // Edge-triggered messages: fire once when X is pressed/released, not every loop() tick
    if (buttonHeld && !lastButtonHeld) {
        Serial.println("Cue Mechanism: X pressed - extending");
    } else if (!buttonHeld && lastButtonHeld) {
        Serial.println("Cue Mechanism: X released - returning to 0");
    }
    lastButtonHeld = buttonHeld;

    if (buttonHeld) {
        if (currentAngle < 180.0) {
            currentAngle += (30.0 / 1000.0) * dt; 
            
            if (currentAngle > 180.0) {
                currentAngle = 180.0;
            }
            
            int pulse = 500 + (currentAngle / 180.0) * (2400 - 500);
            myServo.writeMicroseconds(pulse);

            Serial.print("Cue Mechanism: angle=");
            Serial.print(currentAngle);
            Serial.print(" | pulse=");
            Serial.println(pulse);
        }
    } else {
        if (currentAngle > 0.0) {
            currentAngle = 0.0;
            myServo.write(0);

            Serial.println("Cue Mechanism: angle=0.00 | pulse=500");
        }
    }
}