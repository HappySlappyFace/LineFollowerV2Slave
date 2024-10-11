#include <Arduino.h>
#include "declarations.h"
#include "motor.h"
#include "setup.h"
#include "communication.h"


// =====================
// === Setup Function ===
// =====================

void setup() {
    // Initialize Serial Communication for Debugging
    Serial.begin(115200);
    while (!Serial) { ; } // Wait for Serial to initialize

    mySerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

    // Initialize PWM Channels
    setupPWM();

    // Initialize Encoder Pins and Attach Interrupts
    initializeEncoders();

    // Set Button Pin as Input with Internal Pull-Up
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Initialize RPM Calculation Timing
    lastRPMCalcTime = millis();

    // Initialize PID Timing
    lastPIDTime = millis();
}

// =======================
// === Main Loop Function ===
// =======================

void loop() {
    unsigned long currentTime = millis();

    // Calculate RPM at Regular Intervals
    if (currentTime - lastRPMCalcTime >= RPM_CALC_INTERVAL) {
        calculateRPM();
        lastRPMCalcTime = currentTime;
    }

    // PID Control at Regular Intervals
    if (currentTime - lastPIDTime >= PID_INTERVAL) {
        calculatePID();
        applyMotorControl();
        lastPIDTime = currentTime;
    }

    // Optionally, process serial commands for dynamic setpoints
    processSerialCommands();
}


