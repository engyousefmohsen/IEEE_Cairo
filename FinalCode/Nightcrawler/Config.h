#ifndef CONFIG_H
#define CONFIG_H

/*<<<<<<<<<<<<<<<<<<<<<<<<< PS5 Controller MAC ADDRESS >>>>>>>>>>>>>>>>>>>>>>>>>>>*/
#define PS5_MAC "90:b6:85:a9:fe:67"

/*<<<<<<<<<<<<<<<<<<<<<<<<< Motor >>>>>>>>>>>>>>>>>>>>>>>>>>>*/
#define ENA 23
#define IN1 22
#define IN2 21
#define ENB 19
#define IN3 18
#define IN4 5

/*<<<<<<<<<<<<<<<<<<<<<<<<< PWM >>>>>>>>>>>>>>>>>>>>>>>>>>>*/
#define PWM_FREQ              5000
#define PWM_RES                8
#define CH_ENA                 0
#define CH_ENB                 1
#define PWM_MAX               255   // Highest duty cycle for 8-bit PWM (0-255)

/*<<<<<<<<<<<<<<<<<<<<<<<<< IR Sensor >>>>>>>>>>>>>>>>>>>>>>>>>>>*/
#define IR_L 35
#define IR_M 32
#define IR_R 33

#define WHITE LOW
#define BLACK HIGH

/*<<<<<<<<<<<<<<<<<<<<<<<<< PID Tuning >>>>>>>>>>>>>>>>>>>>>>>>>>>*/
const float Kp = 60.0;
const float Ki = 0.001;
const float Kd = 25.0;
const int BASE_SPEED = 200;
const float PID_INTEGRAL_LIMIT = 100.0;   // anti-windup clamp on the accumulator ( Prevent Violent Spinning When The Robot Leaves The Line or Stuck )

/*<<<<<<<<<<<<<<<<<<<<<<<<< Manual Control >>>>>>>>>>>>>>>>>>>>>>>>>>>*/
#define STICK_DEADZONE      15           // +/- percent-equivalent (of +/-128 range) treated as neutral 
#define MANUAL_MAX_SPEED    240     
#define MANUAL_MAX_STEER    150

/*<<<<<<<<<<<<<<<<<<<<<<<<< Impulse Mechanism >>>>>>>>>>>>>>>>>>>>>>>>>>>*/
#define SERVO_PIN       4
#define STATUS_LED      2 

#endif /* CONFIG_H */