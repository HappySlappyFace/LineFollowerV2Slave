//
// Created by hsf on 10/11/24.
//

#ifndef SLAVELINEFOLLOWER_MOTOR_H
#define SLAVELINEFOLLOWER_MOTOR_H



// Function to constrain a value between a minimum and maximum range
double constrainValue(double value, double minValue, double maxValue);

// Function to calculate the RPM of the motors
void calculateRPM();

// Function to calculate the PID control for the motors
void calculatePID();

// Function to apply the calculated PID output to control the motors
void applyMotorControl();

void applyPIDControl();


#endif //SLAVELINEFOLLOWER_MOTOR_H
