#ifndef IRSENSORS_H
#define IRSENSORS_H

class IRSensors {
public:
    void initialize();
    void calibrate();
    int readSensorArray();
    int calculateError();

    void printSensorValues();

private:
    const int sensorPins[9] = {34, 35, 32, 33, 25, 26, 27, 14, 12};
    int sensorValues[9];
    int sensorMin[9];  // To store the minimum calibrated values
    int sensorMax[9];  // To store the maximum calibrated values
    const int centerSensorIndex = 4;  // Assuming the center sensor is at index 4

    float smoothedError = 0.0;  // Variable to store the smoothed error
    const float smoothingFactor = 0.1;  // Smoothing factor for EMA (adjustable)

};

#endif
