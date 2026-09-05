#ifndef LINEFOLLOWER_H
#define LINEFOLLOWER_H

/* Sensor Initialization
*    Intialize IR Sensor Pins                       */
void initSensors();
/* Start Line Follower
*    Start the line follower algorithm                     
*    Run Line Follower PID Control                     */
void runLineFollowerPID();


#endif /* LINEFOLLOWER_H */