#ifndef LINEFOLLOWER_H
#define LINEFOLLOWER_H

/* Sensor Initialization
*    Intialize IR Sensor Pins                       */
void initSensors();
/* Start Line Follower
*    Start the line follower algorithm                     
*    Run Line Follower PID Control                     */
void runLineFollowerPID();

/* Reset PID State
*    Zeroes the error/integral/derivative accumulators and re-syncs the internal PID clock.
*    MUST be called whenever the robot leaves MODE_LINE_FOLLOWING so stale error/integral terms from before the  switch don't cause a jerk when the mode is re-entered.          
*/
void resetLineFollowerPID();

#endif /* LINEFOLLOWER_H */
