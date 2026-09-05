
#include <Arduino.h>
#include "BluetoothSerial.h"

// ============================================================
// BLUETOOTH
// ============================================================

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled!
#endif

BluetoothSerial SerialBT;

#define BT_NAME "LineFollower"

// ============================================================
// IR SENSOR ANALOG INPUTS
// ============================================================

#define IR_LEFT    34
#define IR_CENTER  35
#define IR_RIGHT   32

// ============================================================
// DRV8833
//
// LEFT MOTOR:
// AIN1 -> GPIO 27
// AIN2 -> GPIO 14
//
// RIGHT MOTOR:
// BIN1 -> GPIO 25
// BIN2 -> GPIO 26
//
// ============================================================

#define ML_IN1 25
#define ML_IN2 26

#define MR_IN1 27
#define MR_IN2 14

// ============================================================
// MOTOR PWM
// ============================================================

#define PWM_FREQ 5000
#define PWM_RES  8

// ============================================================
// MOTOR SPEED
// ============================================================

int baseSpeed = 170;

int maxSpeed = 180;
int minSpeed = 160;

// ============================================================
// IR CALIBRATION
// ============================================================

int whiteL = 0;
int whiteC = 0;
int whiteR = 0;

int blackL = 0;
int blackC = 0;
int blackR = 0;

bool calibrated = false;

// ============================================================
// NORMALIZED SENSOR VALUES
//
// 0    = WHITE
// 1000 = BLACK
// ============================================================

int L = 0;
int C = 0;
int R = 0;

// ============================================================
// PID
// ============================================================

float Kp = 80.0f;
float Ki = 0.0f;
float Kd = 4.0f;

float error = 0.0f;
float lastError = 0.0f;
float integral = 0.0f;

unsigned long lastPIDTime = 0;

// ============================================================
// MODE
// ============================================================

enum RobotMode
{
    MANUAL,
    AUTO
};

RobotMode mode = MANUAL;

char manualCommand = 'S';

// ============================================================
// FUNCTION PROTOTYPES
// ============================================================

void calibrateIR();

void readSensors();

int normalizeSensor(int raw,int white,int black);

float calculatePosition();

float calculatePID();

void lineFollowing();

void processBluetooth();

void manualControl();

void printSensorValues();

void setMotor(int in1,int in2,int speed);

void setLeftMotor(int speed);

void setRightMotor(int speed);

void applyMotors(int leftSpeed,int rightSpeed);

void stopMotors();

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    // ========================================================
    // BLUETOOTH
    // ========================================================

    SerialBT.begin(BT_NAME);

    delay(500);

    SerialBT.println();
    SerialBT.println("==============================");
    SerialBT.println("       LINE FOLLOWER");
    SerialBT.println("==============================");

    SerialBT.println();
    SerialBT.println("Bluetooth started.");
    SerialBT.print("Device: ");
    SerialBT.println(BT_NAME);

    // ========================================================
    // IR
    // ========================================================

    pinMode(IR_LEFT, INPUT);
    pinMode(IR_CENTER, INPUT);
    pinMode(IR_RIGHT, INPUT);

    analogReadResolution(12);

    // ADC range suitable for ESP32
    analogSetAttenuation(ADC_11db);

    // ========================================================
    // MOTOR PINS
    // ========================================================

    pinMode(ML_IN1, OUTPUT);
    pinMode(ML_IN2, OUTPUT);

    pinMode(MR_IN1, OUTPUT);
    pinMode(MR_IN2, OUTPUT);

    // ========================================================
    // PWM
    // ========================================================

    ledcAttach(
        ML_IN1,
        PWM_FREQ,
        PWM_RES
    );

    ledcAttach(
        ML_IN2,
        PWM_FREQ,
        PWM_RES
    );

    ledcAttach(
        MR_IN1,
        PWM_FREQ,
        PWM_RES
    );

    ledcAttach(
        MR_IN2,
        PWM_FREQ,
        PWM_RES
    );

    // ========================================================
    // STOP MOTORS
    // ========================================================

    stopMotors();

    lastPIDTime = micros();

    // ========================================================
    // COMMANDS
    // ========================================================

    SerialBT.println();
    SerialBT.println("Commands:");
    SerialBT.println();
    SerialBT.println("C = IR calibration");
    SerialBT.println("V = Read IR values");
    SerialBT.println("A = Auto mode");
    SerialBT.println("M = Manual mode");
    SerialBT.println();
    SerialBT.println("Manual:");
    SerialBT.println("F = Forward");
    SerialBT.println("B = Backward");
    SerialBT.println("L = Left");
    SerialBT.println("R = Right");
    SerialBT.println("G = Forward Left");
    SerialBT.println("H = Forward Right");
    SerialBT.println("I = Backward Left");
    SerialBT.println("J = Backward Right");
    SerialBT.println("S = STOP");
    SerialBT.println();

    SerialBT.println("Send C to calibrate.");
    SerialBT.println();
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
    // ========================================================
    // BLUETOOTH COMMANDS
    // ========================================================

    processBluetooth();

    // ========================================================
    // ROBOT MODE
    // ========================================================

    if (mode == MANUAL)
    {
        manualControl();
    }
    else
    {
        if (calibrated)
        {
            lineFollowing();
        }
        else
        {
            stopMotors();
        }
    }

    delay(2);
}

// ============================================================
// BLUETOOTH PROCESSING
// ============================================================

void processBluetooth()
{
    while (SerialBT.available())
    {
        char cmd = SerialBT.read();

        // Ignore CR/LF
        if (cmd == '\r' || cmd == '\n')
        {
            continue;
        }

        // Convert lowercase to uppercase
        if (cmd >= 'a' && cmd <= 'z')
        {
            cmd -= 'a' - 'A';
        }

        // ====================================================
        // CALIBRATION
        // ====================================================

        if (cmd == 'C')
        {
            stopMotors();

            calibrateIR();

            continue;
        }

        // ====================================================
        // SENSOR VALUES
        // ====================================================

        if (cmd == 'V')
        {
            printSensorValues();

            continue;
        }

        // ====================================================
        // AUTO
        // ====================================================

        if (cmd == 'A')
        {
            if (!calibrated)
            {
                SerialBT.println();
                SerialBT.println(
                    "ERROR: Sensors not calibrated!"
                );
                SerialBT.println(
                    "Send C first."
                );
                SerialBT.println();

                stopMotors();

                continue;
            }

            mode = AUTO;

            error = 0;
            lastError = 0;
            integral = 0;

            lastPIDTime = micros();

            SerialBT.println();
            SerialBT.println("AUTO MODE");
            SerialBT.println();

            continue;
        }

        // ====================================================
        // MANUAL
        // ====================================================

        if (cmd == 'M')
        {
            mode = MANUAL;

            manualCommand = 'S';

            stopMotors();

            SerialBT.println();
            SerialBT.println("MANUAL MODE");
            SerialBT.println();

            continue;
        }

        // ====================================================
        // MANUAL MOVEMENT
        // ====================================================

        if (
            cmd == 'F' ||
            cmd == 'B' ||
            cmd == 'L' ||
            cmd == 'R' ||
            cmd == 'G' ||
            cmd == 'H' ||
            cmd == 'I' ||
            cmd == 'J' ||
            cmd == 'S'
        )
        {
            manualCommand = cmd;

            if (cmd == 'S')
            {
                stopMotors();

                SerialBT.println("STOP");
            }
        }
    }
}

// ============================================================
// IR CALIBRATION
// ============================================================

void calibrateIR()
{
    stopMotors();

    calibrated = false;

    SerialBT.println();
    SerialBT.println("==============================");
    SerialBT.println("       IR CALIBRATION");
    SerialBT.println("==============================");
    SerialBT.println();

    // ========================================================
    // WHITE
    // ========================================================

    SerialBT.println(
        "STEP 1: Put ALL sensors over WHITE."
    );

    SerialBT.println();

    SerialBT.println("Starting in 3...");
    delay(1000);

    SerialBT.println("2...");
    delay(1000);

    SerialBT.println("1...");
    delay(1000);

    SerialBT.println();
    SerialBT.println("Reading WHITE...");
    SerialBT.println();

    const int samples = 500;

    long sumL = 0;
    long sumC = 0;
    long sumR = 0;

    for (int i = 0; i < samples; i++)
    {
        sumL += analogRead(IR_LEFT);
        sumC += analogRead(IR_CENTER);
        sumR += analogRead(IR_RIGHT);

        delay(2);
    }

    whiteL = sumL / samples;
    whiteC = sumC / samples;
    whiteR = sumR / samples;

    SerialBT.print("White L = ");
    SerialBT.println(whiteL);

    SerialBT.print("White C = ");
    SerialBT.println(whiteC);

    SerialBT.print("White R = ");
    SerialBT.println(whiteR);

    SerialBT.println();

    // ========================================================
    // BLACK
    // ========================================================

    SerialBT.println(
        "STEP 2: Put ALL sensors over BLACK."
    );

    SerialBT.println();

    SerialBT.println("Starting in 3...");
    delay(1000);

    SerialBT.println("2...");
    delay(1000);

    SerialBT.println("1...");
    delay(1000);

    SerialBT.println();
    SerialBT.println("Reading BLACK...");
    SerialBT.println();

    sumL = 0;
    sumC = 0;
    sumR = 0;

    for (int i = 0; i < samples; i++)
    {
        sumL += analogRead(IR_LEFT);
        sumC += analogRead(IR_CENTER);
        sumR += analogRead(IR_RIGHT);

        delay(2);
    }

    blackL = sumL / samples;
    blackC = sumC / samples;
    blackR = sumR / samples;

    SerialBT.print("Black L = ");
    SerialBT.println(blackL);

    SerialBT.print("Black C = ");
    SerialBT.println(blackC);

    SerialBT.print("Black R = ");
    SerialBT.println(blackR);

    SerialBT.println();

    // ========================================================
    // CONTRAST
    // ========================================================

    int diffL = abs(blackL - whiteL);
    int diffC = abs(blackC - whiteC);
    int diffR = abs(blackR - whiteR);

    SerialBT.println("Contrast:");

    SerialBT.print("L = ");
    SerialBT.println(diffL);

    SerialBT.print("C = ");
    SerialBT.println(diffC);

    SerialBT.print("R = ");
    SerialBT.println(diffR);

    SerialBT.println();

    // ========================================================
    // CHECK SENSOR QUALITY
    // ========================================================

    if (diffL < 100 ||
        diffC < 100 ||
        diffR < 100)
    {
        SerialBT.println(
            "WARNING: Weak sensor contrast!"
        );

        SerialBT.println(
            "Adjust the sensor potentiometers."
        );

        SerialBT.println(
            "Calibration NOT accepted."
        );

        SerialBT.println();

        calibrated = false;

        return;
    }

    calibrated = true;

    SerialBT.println(
        "***** CALIBRATION SUCCESSFUL *****"
    );

    SerialBT.println();

    SerialBT.println(
        "Send V to check sensor values."
    );

    SerialBT.println(
        "Send A to start line following."
    );

    SerialBT.println();
}

// ============================================================
// SENSOR NORMALIZATION
// ============================================================
//
// Automatically handles either:
//
// BLACK > WHITE
//
// OR
//
// BLACK < WHITE
//
// Result:
//
// WHITE = 0
// BLACK = 1000
//
// ============================================================

int normalizeSensor(int raw,int white,int black)
{
    int difference =
        black - white;

    if (abs(difference) < 10)
    {
        return 0;
    }

    long value = ((long)(raw - white) * 1000L) / difference;

    value = constrain(value,0,1000);

    return (int)value;
}

// ============================================================
// READ SENSORS
// ============================================================

void readSensors()
{
    int rawL = analogRead(IR_LEFT);

    int rawC = analogRead(IR_CENTER);

    int rawR = analogRead(IR_RIGHT);

    L = normalizeSensor(rawL,whiteL,blackL);

    C = normalizeSensor(rawC,whiteC,blackC);

    R = normalizeSensor(rawR,whiteR,blackR);
}

// ============================================================
// CALCULATE LINE POSITION
// ============================================================
//
// Left   = -1
// Center =  0
// Right  = +1
//
// ============================================================

float calculatePosition()
{
    long total = (long)L +(long)C + (long)R;

    // ========================================================
    // LINE LOST
    // ========================================================

    if (total < 50)
    {
        return lastError;
    }

    float position = ((L * -1.0f) + (R *  1.0f)) / total;

    return position;
}

// ============================================================
// PID
// ============================================================

float calculatePID()
{
    readSensors();

    float position = calculatePosition();

    error = position;

    unsigned long now = micros();

    float dt = (now - lastPIDTime) / 1000000.0f;

    lastPIDTime = now;

    if (dt <= 0.0001f || dt > 0.1f)
    {
        dt = 0.001f;
    }

    // ========================================================
    // INTEGRAL
    // ========================================================

    integral += error * dt;

    integral = constrain( integral,-1.0f, 1.0f);

    // ========================================================
    // DERIVATIVE
    // ========================================================

    float derivative = (error - lastError) / dt;

    // ========================================================
    // PID
    // ========================================================

    float output = Kp * error + Ki * integral + Kd * derivative;

    lastError = error;

    output = constrain(output,-baseSpeed,baseSpeed);

    return output;
}

// ============================================================
// LINE FOLLOWING
// ============================================================

void lineFollowing()
{
    float correction = calculatePID();

    // ========================================================
    // SPEED REDUCTION DURING TURNS
    // ========================================================

    int reduction =
        abs(correction) * 0.4f;

    int targetSpeed =
        maxSpeed - reduction;

    targetSpeed = constrain(targetSpeed,minSpeed,maxSpeed);

    // ========================================================
    // STEERING
    // ========================================================

    int leftSpeed =targetSpeed + correction;

    int rightSpeed = targetSpeed - correction;

    leftSpeed =constrain(leftSpeed,-255,255);

    rightSpeed =  constrain(rightSpeed,-255,255);

    applyMotors(leftSpeed,rightSpeed);
}

// ============================================================
// MANUAL CONTROL
// ============================================================

void manualControl()
{
    switch (manualCommand)
    {
        case 'F':

            applyMotors(180,180);

            break;

        case 'B':

            applyMotors( -180,-180);

            break;

        case 'L':

            applyMotors(-160,160);

            break;

        case 'R':

            applyMotors(160,-160);

            break;

        case 'G':

            applyMotors(100,180);

            break;

        case 'H':

            applyMotors(180,100);

            break;

        case 'I':

            applyMotors(-100,-180);

            break;

        case 'J':

            applyMotors(-180,-100);

            break;

        case 'S':
        default:

            stopMotors();

            break;
    }
}

// ============================================================
// LEFT MOTOR
// ============================================================

void setLeftMotor(int speed)
{
    setMotor(ML_IN1,ML_IN2,speed);
}

// ============================================================
// RIGHT MOTOR
// ============================================================

void setRightMotor(int speed)
{
    setMotor(MR_IN1,MR_IN2,speed);
}

// ============================================================
// GENERIC MOTOR
// ============================================================

void setMotor(int in1,int in2,int speed)
{
    speed =constrain(speed,-255, 255);

    if (speed > 0)
    {
        ledcWrite(in1,speed);

        ledcWrite(in2,0);
    }

    else if (speed < 0)
    {
        ledcWrite(in1, 0);

        ledcWrite(in2,-speed);
    }

    else
    {
        ledcWrite(in1, 0);

        ledcWrite(in2, 0);
    }
}

// ============================================================
// APPLY MOTORS
// ============================================================

void applyMotors(int leftSpeed,int rightSpeed)
{
    setLeftMotor(leftSpeed);

    setRightMotor(rightSpeed);
}

// ============================================================
// STOP
// ============================================================

void stopMotors()
{
    ledcWrite(ML_IN1, 0);
    ledcWrite(ML_IN2, 0);

    ledcWrite(MR_IN1, 0);
    ledcWrite(MR_IN2, 0);
}

// ============================================================
// SENSOR DEBUG
// ============================================================

void printSensorValues()
{
    if (!calibrated)
    {
        SerialBT.println();
        SerialBT.println(
            "Sensors are not calibrated."
        );

        SerialBT.println(
            "Send C first."
        );

        SerialBT.println();

        return;
    }

    int rawL =
        analogRead(IR_LEFT);

    int rawC =
        analogRead(IR_CENTER);

    int rawR =
        analogRead(IR_RIGHT);

    readSensors();

    float position =
        calculatePosition();

    SerialBT.println();
    SerialBT.println("==============================");
    SerialBT.println("          IR VALUES");
    SerialBT.println("==============================");

    SerialBT.println();

    SerialBT.println("RAW:");

    SerialBT.print("L = ");
    SerialBT.println(rawL);

    SerialBT.print("C = ");
    SerialBT.println(rawC);

    SerialBT.print("R = ");
    SerialBT.println(rawR);

    SerialBT.println();

    SerialBT.println("NORMALIZED:");

    SerialBT.print("L = ");
    SerialBT.println(L);

    SerialBT.print("C = ");
    SerialBT.println(C);

    SerialBT.print("R = ");
    SerialBT.println(R);

    SerialBT.println();

    SerialBT.print("Position = ");
    SerialBT.println(
        position,
        3
    );

    SerialBT.println();

    SerialBT.println(
        "Expected:"
    );

    SerialBT.println(
        "Left   line -> -1"
    );

    SerialBT.println(
        "Center line -> 0"
    );

    SerialBT.println(
        "Right  line -> +1"
    );

    SerialBT.println();

    SerialBT.println("==============================");
    SerialBT.println();
}

