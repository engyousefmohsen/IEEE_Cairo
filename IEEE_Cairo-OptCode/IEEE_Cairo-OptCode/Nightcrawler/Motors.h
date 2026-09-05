#ifndef MOTORS_H
#define MOTORS_H

/* MOTOR Initialization 
*   Intializes the motor control pins and sets up PWM channels for motor speed control.     */
void initMotors();

/* MOTOR Speed Control 
*   Sets the speed of the left and right motors.     */
void setMotorSpeeds(int leftSpeed, int rightSpeed);

/* MOTOR Stop 
*   Stops both motors.       */
void stopMotors();


#endif /* MOTORS_H */