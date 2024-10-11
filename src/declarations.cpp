//
// Created by hsf on 10/11/24.
//
#include "declarations.h"

// Instantiate UART2 as Serial1
HardwareSerial mySerial(2); // UART2


// Debug booleans
boolean debugCommunication=false;
boolean debugRPM=false;
boolean debugPID=false;

// =====================
// === Pin Definitions ===
// =====================

// Define custom RX and TX pins
const int RX_PIN = 26; // RX pin for Serial1
const int TX_PIN = 25; // TX pin for Serial1

// Encoder Pins
const uint8_t ENCODER1_A_PIN = 16; // Motor 1 (Right) Encoder A
const uint8_t ENCODER1_B_PIN = 17; // Motor 1 (Right) Encoder B
const uint8_t ENCODER2_A_PIN = 23; // Motor 2 (Left) Encoder A
const uint8_t ENCODER2_B_PIN = 22; // Motor 2 (Left) Encoder B

// Motor Control Pins
const uint8_t LEFT_MOTOR_FORWARDS = 21;
const uint8_t LEFT_MOTOR_BACKWARDS = 19;
const uint8_t RIGHT_MOTOR_FORWARDS = 13;
const uint8_t RIGHT_MOTOR_BACKWARDS = 4;

// Button Pin (Optional for PID Tuning)
const uint8_t BUTTON_PIN = 14;

// ========================
// === PWM Configuration ===
// ========================

// PWM Channels for LEDC
const int PWM_CHANNEL_LEFT_FORWARD = 0;
const int PWM_CHANNEL_LEFT_BACKWARD = 1;
const int PWM_CHANNEL_RIGHT_FORWARD = 2;
const int PWM_CHANNEL_RIGHT_BACKWARD = 3;

// PWM Settings
const int PWM_FREQ = 20000;        // 20 kHz
const int PWM_RESOLUTION = 8;      // 8-bit resolution (0-255)

// =====================
// === Global Variables ===
// =====================

// Encoder Tick Counts
volatile long totalEncoderTicks1 = 0; // Motor 1 (Right)
volatile long totalEncoderTicks2 = 0; // Motor 2 (Left)

// Timing Variables for RPM Calculation
unsigned long lastRPMCalcTime = 0;
const unsigned long RPM_CALC_INTERVAL = 100; // 100 ms

// RPM Calculation Variables
double currentRPM1 = 0.0;
double currentRPM2 = 0.0;
long lastEncoderTicks1 = 0;
long lastEncoderTicks2 = 0;

// PID Parameters
double kp = 1.8;   // Proportional gain
double ki = 0.42;   // Integral gain
double kd = 0.035;  // Derivative gain

// PID Variables for Motor 1 (Right)
double error1 = 0.0;
double previousError1 = 0.0;
double integral1 = 0.0;
double derivative1 = 0.0;
double output1 = 0.0;

// PID Variables for Motor 2 (Left)
double error2 = 0.0;
double previousError2 = 0.0;
double integral2 = 0.0;
double derivative2 = 0.0;
double output2 = 0.0;

// RPM Setpoints
double setpointRPM1 = 0.0; // Motor 1 (Right)
double setpointRPM2 = 0.0; // Motor 2 (Left)

// PID Timing Variables
unsigned long lastPIDTime = 0;
const unsigned long PID_INTERVAL = 100; // 100 ms

// Debounce Variables for Button
int buttonState = HIGH;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;

// Anti-Windup Limits
const double integralMax = 100.0;  // Maximum integral value
const double integralMin = -100.0; // Minimum integral value

// Deadband Threshold
const double deadband = 5.0; // Define an acceptable RPM error range (e.g., ±5 RPM)

String serialBuffer = "";
