#include <ps5Controller.h>
#include "Config.h"
#include "Motors.h"
#include "LineFollower.h"
#include "CueMechanism.h"

/* Robot Modes
*    AUTONOMOUS: Line Follower Mode
*    MANUAL: PS5 Controller Mode                     */
enum RobotMode { AUTONOMOUS, MANUAL };

/* Start As AUTONOMOUS Mode */
RobotMode currentMode = AUTONOMOUS;

bool lastTriangleState = false;

void setup() {
    Serial.begin(115200);

    pinMode(STATUS_LED, OUTPUT);
    initMotors();
    initSensors();
    initMechanism(SERVO_PIN);
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
        digitalWrite(STATUS_LED, HIGH);

        bool triangleNow = ps5.Triangle();
        if (triangleNow && !lastTriangleState) {   // edge-triggered: fires once per press, not once per poll while held
            currentMode = (currentMode == AUTONOMOUS) ? MANUAL : AUTONOMOUS;

            // <<<<<<<<<<<<<<<<<< State transition cleanup >>>>>>>>>>>>>>>>>>
            stopMotors();              // never carry over motor speeds across a mode switch
            updateMechanism(false);    // false the cue mechanism

            if (currentMode == MANUAL) {
                // Leaving line-following: wipe the PID's error/integral/
                // derivative history so it doesn't reappear as a jolt When we switch back to AUTONOMOUS later. (BUG Mot3eb MOOOOT)
                resetLineFollowerPID();
            }

            /* Serial For Easier Debugging */
            Serial.print("Mode switched to: ");
            Serial.println(currentMode == AUTONOMOUS ? "AUTONOMOUS" : "MANUAL");
        }
        lastTriangleState = triangleNow;

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

    // <<<<<<<<<<<<<<<<<< Deadzone >>>>>>>>>>>>>>>>>>
    // Raw stick axes are roughly -128..127. Below this threshold the stick is considered neutral, which prevents drift/creep and PWM jitter from small resting offsets around center.
    if (abs(lx) < STICK_DEADZONE) lx = 0;
    if (abs(ly) < STICK_DEADZONE) ly = 0;

    // <<<<<<<<<<<<<<<<<< forward/steer >>>>>>>>>>>>>>>>>>>>>
    // intent, independent of which physical end of the chassis is currently the operational front:
    //   intentY > 0  -> stick pushed forward (up)
    //   intentX > 0  -> stick pushed right
    int intentY = map(ly, -128, 127, 255, -255);
    int intentX = map(lx, -128, 127, -MANUAL_MAX_STEER, MANUAL_MAX_STEER);

    // <<<<<<<<<<<<<<<<<< Manual-mode kinematic inversion >>>>>>>>>>>>>>>>>>>>>
    // In MANUAL mode the operational "front" is the physical BACK (Cue side), not the IR sensor side ( 3ashan Our Nightcrawler Is Creative Mooot )
    int speedY = -intentY;
    int steerX = -intentX;

    int leftMotor  = constrain(speedY + steerX, -MANUAL_MAX_SPEED, MANUAL_MAX_SPEED);
    int rightMotor = constrain(speedY - steerX, -MANUAL_MAX_SPEED, MANUAL_MAX_SPEED);

    setMotorSpeeds(leftMotor, rightMotor);

    /*<<<<<<<<<<<<<<< Serial For Easier Debugging >>>>>>>>>>>>>>> */ 
    Serial.print("LX: ");   Serial.print(lx);
    Serial.print(" LY: ");  Serial.print(ly);
    Serial.print(" | speedY: "); Serial.print(speedY);
    Serial.print(" steerX: ");   Serial.print(steerX);
    Serial.print(" | L_motor: "); Serial.print(leftMotor);
    Serial.print(" R_motor: ");   Serial.println(rightMotor);  /* Always Remember the Last One is Println ( 3ashan Msh Tefdal 2 hours msh 3aref el Code msh be respond leeeeeeeeeeeh ) */

    /* Cue Control
    *    Cross Button: Activate Cue Mechanism                     */
    updateMechanism(ps5.Cross());
}