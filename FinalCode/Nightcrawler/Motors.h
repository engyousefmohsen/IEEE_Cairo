#ifndef MOTORS_H
#define MOTORS_H

/* MOTOR Initialization 
*   Intializes the motor control pins and sets up PWM channels for motor speed control.     */
void initMotors();

/* MOTOR Speed Control 
*   Sets the speed of the left and right motors.
*   Range: -PWM_MAX to +PWM_MAX (values outside this range are clamped internally).
*   Sign is direction (based on physical wiring), magnitude is duty cycle.     */
void setMotorSpeeds(int leftSpeed, int rightSpeed);

/* MOTOR Stop 
*   Stops both motors (coast, zero duty cycle, both direction pins LOW).       */
void stopMotors();


#endif /* MOTORS_H */
