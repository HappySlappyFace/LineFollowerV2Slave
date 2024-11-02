#include "motor.h"
#include "declarations.h"
#include <PIDController.h>  // Include the PIDController library

// =========================
// === PID Controller Setup ===
// =========================

// Initialize PID controllers for each motor
PIDController pidMotor1;
PIDController pidMotor2;

double sensitivityDivisor = 10.0;

// PID tunings

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
// === Initialize PID Controllers ===
// ============================

void initializePIDControllers() {
    // Configure PID parameters for Motor 1
    pidMotor1.begin();
    pidMotor1.tune(kp, ki, kd);
    pidMotor1.setpoint(setpointRPM1);
    pidMotor1.limit(-255, 255);          // Constrain output to PWM range
    pidMotor1.minimize(sensitivityDivisor);  // Set divisor to scale down PID output

    // Configure PID parameters for Motor 2
    pidMotor2.begin();
    pidMotor2.tune(kp, ki, kd);
    pidMotor2.setpoint(setpointRPM2);
    pidMotor2.limit(-255, 255);          // Constrain output to PWM range
    pidMotor2.minimize(sensitivityDivisor);  // Set divisor to scale down PID output
}

// ============================
// === Apply PID Control ===
// ============================

void applyPIDControl() {
    // Update current values in the PID controllers
    output1 = pidMotor1.compute(currentRPM1);
    output2 = pidMotor2.compute(currentRPM2);

    // Debugging Information
    if(debugPID){
        Serial.print("Motor 1 - Setpoint: ");
        Serial.print(setpointRPM1);
        Serial.print(", Current RPM: ");
        Serial.print(currentRPM1);
        Serial.print(", Output: ");
        Serial.println(output1);

        Serial.print("Motor 2 - Setpoint: ");
        Serial.print(setpointRPM2);
        Serial.print(", Current RPM: ");
        Serial.print(currentRPM2);
        Serial.print(", Output: ");
        Serial.println(output2);
    }

    // Apply the motor control based on the PID output
    applyMotorControl();
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
        ledcWrite(PWM_CHANNEL_RIGHT_BACKWARD, (int)(-output1)); // Make output positive
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
        ledcWrite(PWM_CHANNEL_LEFT_BACKWARD, (int)(-output2)); // Make output positive
        ledcWrite(PWM_CHANNEL_LEFT_FORWARD, 0);
    } else {
        ledcWrite(PWM_CHANNEL_LEFT_FORWARD, 0);
        ledcWrite(PWM_CHANNEL_LEFT_BACKWARD, 0);
    }
}
