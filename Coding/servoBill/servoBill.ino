#include <ps5Controller.h>
#include <ESP32Servo.h> 

Servo myServo;
float currentAngle = 0.0;
unsigned long lastTime = 0;

void setupSmoothServo(int servoPin) {
  myServo.setPeriodHertz(50);
  myServo.attach(servoPin, 500, 2400);
  myServo.write(0);
  lastTime = millis();
}

void updateSmoothServo(bool buttonHeld) {
  unsigned long currentTime = millis();
  unsigned long dt = currentTime - lastTime;
  lastTime = currentTime;

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

void setup() {
  setupSmoothServo(9);
  ps5.begin("YOUR_MAC_ADDRESS"); 
}

void loop() {
  if (ps5.isConnected()) {
    updateSmoothServo(ps5.Cross());
  } else {
    updateSmoothServo(false); 
  }
}