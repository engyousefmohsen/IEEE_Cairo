#ifndef CONFIG_H
#define CONFIG_H

/*<<<<<<<<<<<<<<<<<<<<<<<<< PS5 Controller MAC ADDRESS >>>>>>>>>>>>>>>>>>>>>>>>>>>*/
#define PS5_MAC "48:18:8D:0A:90:60"

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

/*<<<<<<<<<<<<<<<<<<<<<<<<< IR Sensor >>>>>>>>>>>>>>>>>>>>>>>>>>>*/
#define IR_L 32
#define IR_M 33
#define IR_R 25

#define WHITE LOW
#define BLACK HIGH

/*<<<<<<<<<<<<<<<<<<<<<<<<< PID Tuning >>>>>>>>>>>>>>>>>>>>>>>>>>>*/
const float Kp = 60.0;
const float Ki = 0.001;
const float Kd = 25.0;
const int BASE_SPEED = 240;

/*<<<<<<<<<<<<<<<<<<<<<<<<< Impulse Mechanism >>>>>>>>>>>>>>>>>>>>>>>>>>>*/




#define STATUS_LED 2 /*  Will Be Changed To Anything */

#endif /* CONFIG_H */