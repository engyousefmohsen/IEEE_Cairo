#define IR_Input_L 32
#define IR_Input_M 33
#define IR_Input_R 25

#define ENA 23    
#define IN1 22    
#define IN2 21    
#define ENB 19    
#define IN3 18   
#define IN4 5     

#define white LOW
#define black HIGH

int baseSpeed = 240;       // Normal speed
int currentSpeed = 240;    // Active speed

// PID variables
float Kp = 60;   
float Ki = 0.001;    
float Kd = 25;   

float error = 0, lastError = 0, integral = 0;

// ================== PWM Setup ==================
#define PWM_FREQ 5000
#define PWM_RES  8
#define CH_ENA   0
#define CH_ENB   1

// ================== Setup ==================
void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); 
  pinMode(IN4, OUTPUT);

  // Setup PWM for motors
  ledcSetup(CH_ENA, PWM_FREQ, PWM_RES);
  ledcSetup(CH_ENB, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, CH_ENA);
  ledcAttachPin(ENB, CH_ENB);

  // IR sensors
  pinMode(IR_Input_L, INPUT);
  pinMode(IR_Input_M, INPUT);
  pinMode(IR_Input_R, INPUT);

  Stop_motors();
  Serial.println("Line Following Robot with PID Ready!");
}

// ================== Loop ==================
void loop() {
  LineFollowerPID();
}

// ================== Line Follower with PID ==================
void LineFollowerPID() {
  int L = digitalRead(IR_Input_L);
  int M = digitalRead(IR_Input_M);
  int R = digitalRead(IR_Input_R);

  // Determine error based on sensor readings
  if (L == black && M == white && R == white)       error = -1;    // line left
  else if (L == black && M == black && R == white)  error = -2;    // more left
  else if (L == white && M == black && R == white)  error = 0;     // centered
  else if (L == white && M == black && R == black)  error = 2;     // more right
  else if (L == white && M == white && R == black)  error = 1;     // line right
  else if ((L == black && M == black && R == black)
        || (L == black && M == white && R == black)) {
    Stop_motors();    
    return;
  }
  else error = lastError; // if line lost, keep last error

  // PID calculations
  integral += error;
  float derivative = error - lastError;
  float output = (Kp * error + Ki * integral + Kd * derivative);
  lastError = error;

  // ================== Movement Logic ==================
  if (error == 0) {
    // Forward (normal PID)
    int leftSpeed  = constrain(currentSpeed - output, 0, 200);
    int rightSpeed = constrain(currentSpeed + output, 0, 200);

    ledcWrite(CH_ENA, rightSpeed);
    ledcWrite(CH_ENB, leftSpeed);

    digitalWrite(IN1, HIGH);  // Right forward
    digitalWrite(IN2, LOW);   
    digitalWrite(IN3, LOW); 
    digitalWrite(IN4, HIGH);  // Left forward

  } else if (error < 0) {
    // Rotate left in place
    int turnSpeed = 150;
    ledcWrite(CH_ENA, turnSpeed);  
    ledcWrite(CH_ENB, turnSpeed);  

    digitalWrite(IN1, LOW); 
    digitalWrite(IN2, HIGH);   // Right forward
    digitalWrite(IN3, LOW); 
    digitalWrite(IN4, HIGH);   // Left backward

  } else if (error > 0) {
    int turnSpeed = 150;
    // Rotate right in place
    ledcWrite(CH_ENA, turnSpeed);  
    ledcWrite(CH_ENB, turnSpeed);  

    digitalWrite(IN1, HIGH); 
    digitalWrite(IN2, LOW);  // Right backward
    digitalWrite(IN3, HIGH); 
    digitalWrite(IN4, LOW);  // Left forward
  }

  // Debug
  Serial.print("L="); Serial.print(L);
  Serial.print(" M="); Serial.print(M);
  Serial.print(" R="); Serial.print(R);
  Serial.print(" | Err="); Serial.print(error);
  Serial.print(" | Out="); Serial.println(output);
}

void Stop_motors() {
  ledcWrite(CH_ENA, 0);
  ledcWrite(CH_ENB, 0);
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, LOW);
}
