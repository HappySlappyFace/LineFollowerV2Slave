#include "IRSensors.h"
#include <Arduino.h>

void IRSensors::initialize() {
    for (int i = 0; i < 9; i++) {
        // Set the initial min and max values for calibration
        sensorMin[i] = 4095;  // Set to the maximum ADC value
        sensorMax[i] = 0;     // Set to the minimum ADC value
    }
}

void IRSensors::calibrate() {
    // Run calibration by moving the robot over the line and collecting min/max values
    for (int j = 0; j < 100; j++) {  // Perform multiple reads for calibration
        for (int i = 0; i < 9; i++) {
            int sensorValue = analogRead(sensorPins[i]);

            // Update min and max for each sensor
            if (sensorValue > sensorMax[i]) sensorMax[i] = sensorValue;
            if (sensorValue < sensorMin[i]) sensorMin[i] = sensorValue;
        }
        delay(10);  // Small delay between calibration reads
    }
}

int IRSensors::readSensorArray() {
    for (int i = 0; i < 9; i++) {
        int rawValue = analogRead(sensorPins[i]);

        // Avoid invalid input range by checking if min equals max
        if (sensorMin[i] == sensorMax[i]) {
            // If calibration failed (min == max), assume no line detected
            sensorValues[i] = 0;
        } else {
            // Normalize the value between 0 and 1000 based on calibrated min and max
            sensorValues[i] = map(rawValue, sensorMin[i], sensorMax[i], 0, 1000);

            // Clamp values to be within range
            if (sensorValues[i] < 0) sensorValues[i] = 0;
            if (sensorValues[i] > 1000) sensorValues[i] = 1000;
        }
    }
    return 0;
}


int IRSensors::calculateError() {
    float error = 0.0;
    float sum = 0.0;
    float weightedSum = 0.0;

    // Assign weights to each sensor to calculate the position error
    for (int i = 0; i < 9; i++) {
        weightedSum += sensorValues[i] * (i - centerSensorIndex);
        sum += sensorValues[i];
    }

    // Normalize the weighted sum by the sum of sensor values
    if (sum != 0) {
        error = weightedSum / sum; // Floating-point division for precise error

        // Scale the error to match the range expected by the PID controller
        error *= 600.0 / 4.0;  // 4 is the maximum weight (based on sensor index)

        // Apply Exponential Moving Average (EMA) to smooth the error
        smoothedError = (smoothingFactor * error) + ((1.0 - smoothingFactor) * smoothedError);
    }

    // Return the smoothed error as an int to match the setpoint range
    return static_cast<int>(smoothedError);
}


void IRSensors::printSensorValues() {
    for (int i = 0; i < 9; i++) {
        int rawValue = analogRead(sensorPins[i]);
        Serial.print(rawValue);

        // Print a space or comma between values, but not after the last value
        if (i < 8) {
            Serial.print(" ");  // Change this to "," if you prefer commas
        }
    }
    Serial.println();  // Move to the next line after printing all values
}


