//
// Created by hsf on 10/11/24.
//

#include "motor.h"
#include "declarations.h"

// =========================
// === Constrain Function ===
// =========================

double constrainValue(double value, double minVal, double maxVal) {
    if (value < minVal)
        return minVal;
    if (value > maxVal)
        return maxVal;
    return value;
}

// ============================
// === RPM Calculation Function ===
// ============================

void calculateRPM() {
    // Calculate Delta Ticks since Last RPM Calculation
    long deltaTicks1 = totalEncoderTicks1 - lastEncoderTicks1;
    long deltaTicks2 = totalEncoderTicks2 - lastEncoderTicks2;

    // Update Last Encoder Tick Counts
    lastEncoderTicks1 = totalEncoderTicks1;
    lastEncoderTicks2 = totalEncoderTicks2;

    // Define Encoder Ticks per Revolution
    const double ticksPerRevolution = 204.0; // Adjust based on your encoder

    // Calculate RPM
    // RPM = (deltaTicks / ticksPerRevolution) * (60 / deltaTime)
    // Here, deltaTime = RPM_CALC_INTERVAL in seconds
    currentRPM1 = (deltaTicks1 / ticksPerRevolution) * (60.0 / (RPM_CALC_INTERVAL / 1000.0));
    currentRPM2 = (deltaTicks2 / ticksPerRevolution) * (60.0 / (RPM_CALC_INTERVAL / 1000.0));

    // Optional: Print RPM for debugging
    if(debugRPM){
        Serial.print("Motor 1 - Current RPM: ");
        Serial.print(currentRPM1);
        Serial.print(", Motor 2 - Current RPM: ");
        Serial.println(currentRPM2);
    }
}

// ============================
// === PID Calculation Function ===
// ============================

void calculatePID() {
    // Calculate Error (Setpoint RPM - Current RPM)
    error1 = (setpointRPM1 - currentRPM1) - targetError/3;
    error2 = (setpointRPM2 - currentRPM2) + targetError/3;

    // Implement Deadband
    if (abs(error1) < deadband) {
        error1 = 0.0;
    }

    if (abs(error2) < deadband) {
        error2 = 0.0;
    }

    // Reset Integral and Derivative Terms if Error is Zero
    if (error1 == 0.0) {
        integral1 = 0.0;
        derivative1 = 0.0;
    } else {
        // Calculate Integral with Anti-Windup
        integral1 += error1 * (PID_INTERVAL / 1000.0);

        // Constrain Integral Term
        if (integral1 > integralMax) integral1 = integralMax;
        if (integral1 < integralMin) integral1 = integralMin;
    }

    if (error2 == 0.0) {
        integral2 = 0.0;
        derivative2 = 0.0;
    } else {
        integral2 += error2 * (PID_INTERVAL / 1000.0);

        if (integral2 > integralMax) integral2 = integralMax;
        if (integral2 < integralMin) integral2 = integralMin;
    }

    // Calculate Derivative
    derivative1 = (error1 - previousError1) / (PID_INTERVAL / 1000.0);
    derivative2 = (error2 - previousError2) / (PID_INTERVAL / 1000.0);

    // Compute PID Output
    output1 = kp * error1 + ki * integral1 + kd * derivative1;
    output2 = kp * error2 + ki * integral2 + kd * derivative2;

    // Constrain PID Output to PWM Range
    output1 = constrainValue(output1, -255.0, 255.0);
    output2 = constrainValue(output2, -255.0, 255.0);

    // Save Current Error for Next Derivative Calculation
    previousError1 = error1;
    previousError2 = error2;

    // Debugging Information
    if(debugPID){

        Serial.print("Motor 1 - Error: ");
        Serial.print(error1);
        Serial.print(", Output: ");
        Serial.print(output1);
        Serial.print(", Integral: ");
        Serial.print(integral1);
        Serial.print(", Derivative: ");
        Serial.println(derivative1);

        Serial.print("Motor 2 - Error: ");
        Serial.print(error2);
        Serial.print(", Output: ");
        Serial.print(output2);
        Serial.print(", Integral: ");
        Serial.print(integral2);
        Serial.print(", Derivative: ");
        Serial.println(derivative2);
    }
}

// ============================
// === Apply Motor Control ===
// ============================

void applyMotorControl() {
    // Motor 1 (Right)
    if (output1 > 0) {
        ledcWrite(PWM_CHANNEL_RIGHT_FORWARD, (int)output1);
        ledcWrite(PWM_CHANNEL_RIGHT_BACKWARD, 0);
    } else if (output1 < 0) {
        ledcWrite(PWM_CHANNEL_RIGHT_BACKWARD, (int)(-output1)); // Make pidOut positive
        ledcWrite(PWM_CHANNEL_RIGHT_FORWARD, 0);
    } else {
        ledcWrite(PWM_CHANNEL_RIGHT_FORWARD, 0);
        ledcWrite(PWM_CHANNEL_RIGHT_BACKWARD, 0);
    }

    // Motor 2 (Left)
    if (output2 > 0) {
        ledcWrite(PWM_CHANNEL_LEFT_FORWARD, (int)output2);
        ledcWrite(PWM_CHANNEL_LEFT_BACKWARD, 0);
    } else if (output2 < 0) {
        ledcWrite(PWM_CHANNEL_LEFT_BACKWARD, (int)(-output2)); // Make pidOut positive
        ledcWrite(PWM_CHANNEL_LEFT_FORWARD, 0);
    } else {
        ledcWrite(PWM_CHANNEL_LEFT_FORWARD, 0);
        ledcWrite(PWM_CHANNEL_LEFT_BACKWARD, 0);
    }
}
