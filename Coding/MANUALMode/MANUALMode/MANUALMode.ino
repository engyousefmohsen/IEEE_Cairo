#include <ps5Controller.h>

#define PUMP_PIN 26

#define ENA 23
#define IN1 22
#define IN2 21

#define ENB 19
#define IN3 18
#define IN4 5

#define FREQ 5000
#define RES 8
#define CH_A 0
#define CH_B 1

bool pumpState = false;
unsigned long lastPress = 0;
int debounceDelay = 300;

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(2, OUTPUT);

  ledcSetup(CH_A, FREQ, RES);
  ledcAttachPin(ENA, CH_A);
  ledcSetup(CH_B, FREQ, RES);
  ledcAttachPin(ENB, CH_B);

  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(2, LOW);

  ps5.begin("48:18:8D:0A:90:60");

  while (!ps5.isConnected()) {
    digitalWrite(2, HIGH);
    delay(500);
    digitalWrite(2, LOW);
    delay(500);
  }

  digitalWrite(2, LOW);
}

void loop() {
  int lx = ps5.LStickX();
  int ly = ps5.LStickY();

  int speedY = map(ly, -128, 127, 255, -255);
  int steerX = map(lx, -128, 127, -150, 150);

  int leftMotor = speedY + steerX;
  int rightMotor = speedY - steerX;

  leftMotor = constrain(leftMotor, -200, 200);
  rightMotor = constrain(rightMotor, -200, 200);

  if (leftMotor >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    ledcWrite(CH_B, leftMotor);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    ledcWrite(CH_B, -leftMotor);
  }

  if (rightMotor >= 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    ledcWrite(CH_A, rightMotor);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(CH_A, -rightMotor);
  }

  if (ps5.Square() && ((millis() - lastPress) > debounceDelay)) {
    pumpState = !pumpState;
    digitalWrite(PUMP_PIN, pumpState);
    lastPress = millis();
  }

  delay(50);
}