#include <Servo.h>
#include <PS5Controller.h>   // For PS5 DualSense control

// LED pins
#define LED_MANUAL 3
#define LED_AUTO   7

// IR sensor pins (digital inputs)
#define IR_LEFT  34
#define IR_CENT  35
#define IR_RIGHT 32

// Motor driver pins
#define IN1 13
#define IN2 12
#define IN3 11
#define IN4 10
#define ENA 6
#define ENB 5

// Servo pin
#define SERVO_PIN 9

Servo servo;

// PS5 MAC address
const char *ps5Mac = "90:b6:85:a9:fe:67";

bool manualMode = true;          // true = manual, false = auto
unsigned long lastCmdTime = 0;   // for optional timeout (kept but not heavily used)

// IR sensor readings
int L, C, R;

// PID constants
double kp = 35.0;
double ki = 0.0;
double kd = 0.4;
double error = 0, lastError = 0;
double integral = 0;
double lastD = 0;
unsigned long lastPidTime = 0;

// Speed parameters
int maxSpeed = 130;
int minSpeed = 90;
float speedSmooth = 0.2;
int baseSpeed = 110;

// Turn state machine
enum TurnState { TURN_NONE, TURN_LEFT, TURN_RIGHT };
TurnState turnState = TURN_NONE;
unsigned long turnStartTime = 0;
const int TURN_SPEED = 180;
const unsigned long TURN_DURATION = 800;

// Function prototypes
void readSensors();
double computePID();
void autoTracker();
void applyMotors(int leftSpeed, int rightSpeed);
void manualModeHandler();
void stopMotors();
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void moveForwardLeft();
void moveForwardRight();
void moveBackwardLeft();
void moveBackwardRight();

// Variables for command handling from controller
char currentCmd = 'S';          // current command (F, B, L, R, G, H, I, J, S, Y, Z)
bool optionsPressed = false;    // for edge detection on Options button

void setup() {
  Serial.begin(115200);
  PS5.begin(ps5Mac);            // start PS5 connection
  Serial.println("Searching for PS5 controller...");

  // Wait for connection (optional – you can remove this while loop)
  while (!PS5.isConnected()) {
    delay(100);
    Serial.print(".");
  }
  Serial.println("\nPS5 connected!");

  // Pin modes
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_CENT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  servo.attach(SERVO_PIN);
  servo.write(90);

  pinMode(LED_MANUAL, OUTPUT);
  pinMode(LED_AUTO, OUTPUT);

  lastPidTime = millis();
}

void loop() {
  PS5.update();   // must be called frequently

  if (!PS5.isConnected()) {
    stopMotors();
    return;
  }

  // ----- Read controller and map to commands -----
  int lx = PS5.LX;   // range typically -127..127
  int ly = PS5.LY;

  // Determine movement command from left stick
  bool up    = (ly < -50);
  bool down  = (ly >  50);
  bool left  = (lx < -50);
  bool right = (lx >  50);

  if (up && !left && !right) {
    currentCmd = 'F';           // forward
  } else if (down && !left && !right) {
    currentCmd = 'B';           // backward
  } else if (left && !up && !down) {
    currentCmd = 'L';           // turn left
  } else if (right && !up && !down) {
    currentCmd = 'R';           // turn right
  } else if (up && left) {
    currentCmd = 'G';           // forward-left
  } else if (up && right) {
    currentCmd = 'H';           // forward-right
  } else if (down && left) {
    currentCmd = 'I';           // backward-left
  } else if (down && right) {
    currentCmd = 'J';           // backward-right
  } else {
    currentCmd = 'S';           // stop
  }

  // Servo control via R1 (up) and R2 (down)
  if (PS5.R1) {
    int pos = servo.read();
    if (pos < 180) pos += 5;
    servo.write(pos);
  }
  if (PS5.R2) {
    int pos = servo.read();
    if (pos > 0) pos -= 5;
    servo.write(pos);
  }

  // Toggle manual/auto with Options button (edge detection)
  if (PS5.Options && !optionsPressed) {
    manualMode = !manualMode;
    optionsPressed = true;
  }
  if (!PS5.Options) {
    optionsPressed = false;
  }

  // ----- Mode handling -----
  if (manualMode) {
    digitalWrite(LED_MANUAL, HIGH);
    digitalWrite(LED_AUTO, LOW);
    manualModeHandler();
  } else {
    digitalWrite(LED_MANUAL, LOW);
    digitalWrite(LED_AUTO, HIGH);
    autoTracker();
  }
}

// Manual control based on currentCmd from controller
void manualModeHandler() {
  // If controller disconnected, stop – already handled in loop
  switch (currentCmd) {
    case 'F': moveForward();          break;
    case 'B': moveBackward();         break;
    case 'L': turnLeft();             break;
    case 'R': turnRight();            break;
    case 'G': moveForwardLeft();      break;
    case 'H': moveForwardRight();     break;
    case 'I': moveBackwardLeft();     break;
    case 'J': moveBackwardRight();    break;
    case 'S': stopMotors();           break;
    default: stopMotors();            break;
  }
}

// ----- Auto Tracker (unchanged logic) -----
void autoTracker() {
  readSensors();

  // Check for 90° turns
  if (turnState != TURN_NONE) {
    if (C == 1) {
      turnState = TURN_NONE;
    } else if (millis() - turnStartTime > TURN_DURATION) {
      turnState = TURN_NONE;
    } else {
      if (turnState == TURN_LEFT) {
        applyMotors(-TURN_SPEED, TURN_SPEED);
      } else {
        applyMotors(TURN_SPEED, -TURN_SPEED);
      }
      return;
    }
  }

  if (L == 1 && C == 0 && R == 0) {
    turnState = TURN_LEFT;
    turnStartTime = millis();
    applyMotors(-TURN_SPEED, TURN_SPEED);
    return;
  }
  if (R == 1 && C == 0 && L == 0) {
    turnState = TURN_RIGHT;
    turnStartTime = millis();
    applyMotors(TURN_SPEED, -TURN_SPEED);
    return;
  }

  int correction = computePID();

  int targetSpeed = maxSpeed - abs(correction) * 3;
  targetSpeed = constrain(targetSpeed, minSpeed, maxSpeed);
  baseSpeed = baseSpeed + (targetSpeed - baseSpeed) * speedSmooth;

  int leftSpeed  = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  applyMotors(leftSpeed, rightSpeed);
}

// ----- PID and Sensor (unchanged) -----
double computePID() {
  unsigned long now = millis();
  double dt = (now - lastPidTime) / 1000.0;
  if (dt <= 0) dt = 0.01;
  lastPidTime = now;

  readSensors();

  if      (C == 1 && L == 0 && R == 0) error = 0;
  else if (C == 1 && L == 1 && R == 1) error = 0;
  else if (C == 1 && R == 1)           error =  1;
  else if (C == 1 && L == 1)           error = -1;
  else if (R == 1)                     error =  2;
  else if (L == 1)                     error = -2;
  else {
    error = lastError;
    integral = 0;
  }

  double p = error;
  integral += error * dt;
  integral = constrain(integral, -30, 30);

  double d = (error - lastError) / dt;
  d = 0.7 * d + 0.3 * lastD;
  d = constrain(d, -15, 15);
  lastD = d;

  double output = kp * p + ki * integral + kd * d;

  lastError = error;
  return output;
}

void readSensors() {
  L = digitalRead(IR_LEFT);
  C = digitalRead(IR_CENT);
  R = digitalRead(IR_RIGHT);
}

// ----- Motor control (unchanged) -----
void applyMotors(int leftSpeed, int rightSpeed) {
  if (leftSpeed == 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 255);
  } else {
    digitalWrite(IN1, leftSpeed > 0 ? HIGH : LOW);
    digitalWrite(IN2, leftSpeed > 0 ? LOW  : HIGH);
    analogWrite(ENA, abs(leftSpeed));
  }

  if (rightSpeed == 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 255);
  } else {
    digitalWrite(IN3, rightSpeed > 0 ? HIGH : LOW);
    digitalWrite(IN4, rightSpeed > 0 ? LOW  : HIGH);
    analogWrite(ENB, abs(rightSpeed));
  }
}

void stopMotors() {
  applyMotors(0, 0);
}

// ----- Movement helpers (unchanged) -----
void moveForward() {
  applyMotors(200, 200);
}
void moveBackward() {
  applyMotors(-200, -200);
}
void turnLeft() {
  applyMotors(-150, 150);
}
void turnRight() {
  applyMotors(150, -150);
}
void moveForwardLeft() {
  applyMotors(120, 200);
}
void moveForwardRight() {
  applyMotors(200, 120);
}
void moveBackwardLeft() {
  applyMotors(-120, -200);
}
void moveBackwardRight() {
  applyMotors(-200, -120);
}