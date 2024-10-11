//
// Created by hsf on 10/11/24.
//

#ifndef SLAVELINEFOLLOWER_SETUP_H
#define SLAVELINEFOLLOWER_SETUP_H


void IRAM_ATTR encoder1ISR();
void IRAM_ATTR encoder2ISR();
void setupPWM();
void initializeEncoders();

#endif //SLAVELINEFOLLOWER_SETUP_H
