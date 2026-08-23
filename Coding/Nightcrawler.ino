#include <ps5Controller.h>
#include "Config.h"
#include "Motors.h"
#include "LineFollower.h"

/* Robot Modes
*    AUTONOMOUS: Line Follower Mode
*    MANUAL: PS5 Controller Mode                     */
enum RobotMode { AUTONOMOUS, MANUAL };

/* Start As AUTONOMOUS Mode */
RobotMode currentMode = AUTONOMOUS;

unsigned long lastButtonPress = 0;
bool pumpState = false;

void setup() {
    Serial.begin(115200);
    
    initMotors();
    initSensors();
    
    ps5.begin(PS5_MAC);
    
    Serial.println("System Ready. Starting in AUTONOMOUS Mode.");
}

void loop() {
    handleModeSwitching();

    if (currentMode == AUTONOMOUS) {
        runLineFollowerPID();
    } else {
        runManualControl();
    }
    
    delay(10); // Small delay to prevent watchdog timeout
}

void handleModeSwitching() {
    if (ps5.isConnected()) {
        digitalWrite(STATUS_LED, HIGH); // Indicator that PS5 is linked
        
        // Use Triangle to toggle modes with debounce
        if (ps5.Triangle() && (millis() - lastButtonPress > 300)) {
            currentMode = (currentMode == AUTONOMOUS) ? MANUAL : AUTONOMOUS;
            stopMotors(); // Always halt before switching contexts
            lastButtonPress = millis();
            
            Serial.print("Mode switched to: ");
            Serial.println(currentMode == AUTONOMOUS ? "AUTONOMOUS" : "MANUAL");
        }
    } else {
        digitalWrite(STATUS_LED, LOW);
    }
}

void runManualControl() {
    if (!ps5.isConnected()) {
        stopMotors();
        return;
    }

    int lx = ps5.LStickX();
    int ly = ps5.LStickY();

    int speedY = map(ly, -128, 127, 255, -255);
    int steerX = map(lx, -128, 127, -150, 150);

    int leftMotor = constrain(speedY + steerX, -200, 200);
    int rightMotor = constrain(speedY - steerX, -200, 200);

    setMotorSpeeds(leftMotor, rightMotor);

}