//
// Created by hsf on 10/11/24.
//
#include "declarations.h"
#include "setup.h"

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