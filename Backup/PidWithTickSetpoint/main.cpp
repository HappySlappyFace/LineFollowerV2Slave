#include <Arduino.h>

// =====================
// === Pin Definitions ===
// =====================

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

// Previous Encoder Tick Counts (for RPM Calculation)
volatile long previousEncoderTicks1 = 0;
volatile long previousEncoderTicks2 = 0;

// PID Target Positions (Initial Position)
long targetTicks1 = 0; // Motor 1 (Right)
long targetTicks2 = 0; // Motor 2 (Left)

// PID Parameters
double kp = 3;   // Proportional gain
double ki = 6;   // Integral gain
double kd = 0.15;  // Derivative gain

// PID Variables for Motor 1 (Right)
double error1 = 0;
double previousError1 = 0;
double integral1 = 0;
double derivative1 = 0;
double output1 = 0;

// PID Variables for Motor 2 (Left)
double error2 = 0;
double previousError2 = 0;
double integral2 = 0;
double derivative2 = 0;
double output2 = 0;

// Timing Variables
unsigned long lastPIDTime = 0;
const unsigned long PID_INTERVAL = 50; // 100 ms

// Debounce Variables for Button
int buttonState = HIGH;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;

// Anti-Windup Limits
const double integralMax = 1000.0; // Maximum integral value
const double integralMin = -1000.0; // Minimum integral value

// Deadband Threshold
const double deadband = 5.0; // Define an acceptable error range (e.g., ±5 ticks)

// ============================
// === Function Prototypes ===
// ============================

void IRAM_ATTR encoder1ISR();
void IRAM_ATTR encoder2ISR();
void setupPWM();
void initializeEncoders();
void calculatePID();
void applyMotorControl();
double constrainValue(double value, double minVal, double maxVal);

// =====================
// === Setup Function ===
// =====================

void setup() {
    // Initialize Serial Communication for Debugging
    Serial.begin(115200);
    while (!Serial) { ; } // Wait for Serial to initialize

    // Initialize PWM Channels
    setupPWM();

    // Initialize Encoder Pins and Attach Interrupts
    initializeEncoders();

    // Set Button Pin as Input with Internal Pull-Up
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Record Initial Encoder Positions as Target Positions
    noInterrupts();
    targetTicks1 = totalEncoderTicks1;
    targetTicks2 = totalEncoderTicks2;
    interrupts();

    // Initialize PID Timing
    lastPIDTime = millis();
}

// =======================
// === Main Loop Function ===
// =======================

void loop() {
    unsigned long currentTime = millis();

    // PID Control at Regular Intervals
    if (currentTime - lastPIDTime >= PID_INTERVAL) {
        lastPIDTime = currentTime;
        calculatePID();
        applyMotorControl();
    }

    // Button Handling for PID Tuning (Optional)
    int reading = digitalRead(BUTTON_PIN);

    if (reading != lastButtonState) {
        lastDebounceTime = currentTime;
    }

    if ((currentTime - lastDebounceTime) > DEBOUNCE_DELAY) {
        if (reading != buttonState) {
            buttonState = reading;
            if (buttonState == LOW) { // Button Pressed
                // Example: Increment kp
                kp += 0.1;
                Serial.print("New kp: ");
                Serial.println(kp);
            }
        }
    }

    lastButtonState = reading;
}

// ============================
// === Encoder ISR Functions ===
// ============================

void IRAM_ATTR encoder1ISR() {
    int A = digitalRead(ENCODER1_A_PIN);
    int B = digitalRead(ENCODER1_B_PIN);
    if (A == B) {
        totalEncoderTicks1--;
    } else {
        totalEncoderTicks1++;
    }
}

void IRAM_ATTR encoder2ISR() {
    int A = digitalRead(ENCODER2_A_PIN);
    int B = digitalRead(ENCODER2_B_PIN);
    if (A == B) {
        totalEncoderTicks2--;
    } else {
        totalEncoderTicks2++;
    }
}

// ========================
// === PWM Setup Function ===
// ========================

void setupPWM() {
    // Configure LEDC Channels for High-Frequency PWM
    // Motor 1 (Right)
    ledcSetup(PWM_CHANNEL_RIGHT_FORWARD, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(RIGHT_MOTOR_FORWARDS, PWM_CHANNEL_RIGHT_FORWARD);

    ledcSetup(PWM_CHANNEL_RIGHT_BACKWARD, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(RIGHT_MOTOR_BACKWARDS, PWM_CHANNEL_RIGHT_BACKWARD);

    // Motor 2 (Left)
    ledcSetup(PWM_CHANNEL_LEFT_FORWARD, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(LEFT_MOTOR_FORWARDS, PWM_CHANNEL_LEFT_FORWARD);

    ledcSetup(PWM_CHANNEL_LEFT_BACKWARD, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(LEFT_MOTOR_BACKWARDS, PWM_CHANNEL_LEFT_BACKWARD);
}

// ============================
// === Initialize Encoders ===
// ============================

void initializeEncoders() {
    // Set Encoder Pins as Inputs with Internal Pull-Up Resistors
    pinMode(ENCODER1_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER1_B_PIN, INPUT_PULLUP);
    pinMode(ENCODER2_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER2_B_PIN, INPUT_PULLUP);

    // Attach Interrupts for Encoder A Pins
    attachInterrupt(digitalPinToInterrupt(ENCODER1_A_PIN), encoder1ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER2_A_PIN), encoder2ISR, CHANGE);
}

// ============================
// === PID Calculation Function ===
// ============================

void calculatePID() {
    // Read Current Encoder Positions
    long currentTicks1 = totalEncoderTicks1;
    long currentTicks2 = totalEncoderTicks2;

    // Calculate Error (Target - Current)
    error1 = targetTicks1 - currentTicks1;
    error2 = targetTicks2 - currentTicks2;

    // Implement Deadband
    if (abs(error1) < deadband) {
        error1 = 0;
    }
    if (abs(error2) < deadband) {
        error2 = 0;
    }

    // Reset Integral and Derivative if Error is Zero
    if (error1 == 0) {
        integral1 = 0;
        derivative1 = 0;
    } else {
        // Calculate Integral with Anti-Windup
        integral1 += error1 * (PID_INTERVAL / 1000.0);

        // Constrain Integral Term
        if (integral1 > integralMax) integral1 = integralMax;
        if (integral1 < integralMin) integral1 = integralMin;
    }

    if (error2 == 0) {
        integral2 = 0;
        derivative2 = 0;
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
    output1 = constrainValue(output1, -255, 255);
    output2 = constrainValue(output2, -255, 255);

    // Save Current Error for Next Derivative Calculation
    previousError1 = error1;
    previousError2 = error2;

    // Debugging Information
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

// ===========================
// === Utility Function ===
// ===========================

double constrainValue(double value, double minVal, double maxVal) {
    if (value < minVal)
        return minVal;
    if (value > maxVal)
        return maxVal;
    return value;
}


