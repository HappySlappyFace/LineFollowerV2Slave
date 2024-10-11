//
// Created by hsf on 10/11/24.
//

#ifndef SLAVELINEFOLLOWER_DECLARATIONS_H
#define SLAVELINEFOLLOWER_DECLARATIONS_H

#include <Arduino.h>

// =====================
// === Pin Definitions ===
// =====================

// Define custom RX and TX pins
extern const int RX_PIN; // RX pin for Serial1
extern const int TX_PIN; // TX pin for Serial1

// Encoder Pins
extern const uint8_t ENCODER1_A_PIN; // Motor 1 (Right) Encoder A
extern const uint8_t ENCODER1_B_PIN; // Motor 1 (Right) Encoder B
extern const uint8_t ENCODER2_A_PIN; // Motor 2 (Left) Encoder A
extern const uint8_t ENCODER2_B_PIN; // Motor 2 (Left) Encoder B

// Motor Control Pins
extern const uint8_t LEFT_MOTOR_FORWARDS;
extern const uint8_t LEFT_MOTOR_BACKWARDS;
extern const uint8_t RIGHT_MOTOR_FORWARDS;
extern const uint8_t RIGHT_MOTOR_BACKWARDS;

// Button Pin (Optional for PID Tuning)
extern const uint8_t BUTTON_PIN;

// ========================
// === PWM Configuration ===
// ========================

// PWM Channels for LEDC
extern const int PWM_CHANNEL_LEFT_FORWARD;
extern const int PWM_CHANNEL_LEFT_BACKWARD;
extern const int PWM_CHANNEL_RIGHT_FORWARD;
extern const int PWM_CHANNEL_RIGHT_BACKWARD;

// PWM Settings
extern const int PWM_FREQ;       // 20 kHz
extern const int PWM_RESOLUTION; // 8-bit resolution (0-255)

// =====================
// === Global Variables ===
// =====================

// Instantiate UART2 as Serial1
extern HardwareSerial mySerial;

// Debug booleans
extern boolean debugCommunication;
extern boolean debugRPM;
extern boolean debugPID;

// Encoder Tick Counts
extern volatile long totalEncoderTicks1; // Motor 1 (Right)
extern volatile long totalEncoderTicks2; // Motor 2 (Left)

// Timing Variables for RPM Calculation
extern unsigned long lastRPMCalcTime;
extern const unsigned long RPM_CALC_INTERVAL;

// RPM Calculation Variables
extern double currentRPM1;
extern double currentRPM2;
extern long lastEncoderTicks1;
extern long lastEncoderTicks2;

// PID Parameters
extern double kp;
extern double ki;
extern double kd;

// PID Variables for Motor 1 (Right)
extern double error1;
extern double previousError1;
extern double integral1;
extern double derivative1;
extern double output1;

// PID Variables for Motor 2 (Left)
extern double error2;
extern double previousError2;
extern double integral2;
extern double derivative2;
extern double output2;

// RPM Setpoints
extern double setpointRPM1; // Motor 1 (Right)
extern double setpointRPM2; // Motor 2 (Left)

// PID Timing Variables
extern unsigned long lastPIDTime;
extern const unsigned long PID_INTERVAL;

// Debounce Variables for Button
extern int buttonState;
extern int lastButtonState;
extern unsigned long lastDebounceTime;
extern const unsigned long DEBOUNCE_DELAY;

// Anti-Windup Limits
extern const double integralMax;  // Maximum integral value
extern const double integralMin;  // Minimum integral value

// Deadband Threshold
extern const double deadband;     // Define an acceptable RPM error range (e.g., ±5 RPM)

extern String serialBuffer;

#endif //SLAVELINEFOLLOWER_DECLARATIONS_H
