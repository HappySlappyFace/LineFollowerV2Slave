#include <Arduino.h>
#include <IRSensors.h>

#include "declarations.h"
#include "motor.h"
#include "setup.h"
#include "CommandLibrary.h"



// =====================
// === Setup Function ===
// =====================

CommandLibrary cl(false);
IRSensors irSensors;

unsigned long lastCommandTime = 0;
unsigned long lastPIDSent = 0;

const unsigned long commandInterval = 5000;  //     500 ms delay between commands

int commandIndex = 0;
int test=1;

void setup() {

    // Initialize Serial Communication for Debugging
    Serial.begin(115200);
    while (!Serial) { ; } // Wait for Serial to initialize

    mySerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

    // Initialize PWM Channels
    setupPWM();

    // Initialize Encoder Pins and Attach Interrupts
    initializeEncoders();

    irSensors.initialize();

    Serial.println("Calibrating sensors...");
    irSensors.calibrate();
    Serial.println("Calibration complete.");

    // Set Button Pin as Input with Internal Pull-Up
    // pinMode(BUTTON_PIN, INPUT_PULLUP);

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
    irSensors.readSensorArray();
    // irSensors.printSensorValues();
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
    int error = irSensors.calculateError();
    cl.setTarget(error);


    // Serial.println(error);
    if(test){
        // if (millis() - lastPIDSent>=1000) {
        //     lastPIDSent = millis();
        //     cl.setTarget(error);
        // }
        if (millis() - lastCommandTime >= commandInterval) {
            lastCommandTime = millis();  // Reset the timer

            // Execute different commands based on commandIndex
            switch (commandIndex) {
                case 0:
                    cl.setPID(0.7, 2.5, 0.1);  // Set PID values (P=1.0, I=0.5, D=0.1)
                    // cl.setPID(3, 3.0, 0.05);  // Set PID values (P=1.0, I=0.5, D=0.1)
                    cl.setSpeed(200);  // Set speed to 100
                    // cl.setRPM1(100);
                    // cl.setRPM2(-100);
                break;
                case 1:
                    cl.setPID(1.4, 1.4, 0);
                    cl.stopMotors();         // Stop the motors
                    test=0;
                break;
                case 2:
                    cl.setPID(2.4, 3, 0.01);  // Set PID values (P=1.0, I=0.5, D=0.1)
                    cl.setRPM1(250);
                    cl.setRPM2(220);
                // cl.setTarget(50);  // Set target (steering) to 50

                break;
                case 3:
                    cl.setPID(2.4, 3, 0.01);  // Set PID values (P=1.0, I=0.5, D=0.1)
                    cl.setRPM1(100);
                    cl.setRPM2(150);
                    // cl.setPID(1.0, 0.5, 0.1);  // Set PID values (P=1.0, I=0.5, D=0.1)
                break;
                case 4:
                    cl.setPID(1.4, 1.4, 0);
                    cl.stopMotors();         // Stop the motors
                    break;
                case 5:
                    cl.setPID(0.7, 2.5, 0.1);  // Set PID values (P=1.0, I=0.5, D=0.1)
                    // cl.setPID(3, 3.0, 0.05);  // Set PID values (P=1.0, I=0.5, D=0.1)
                    // cl.setSpeed(600);  // Set speed to 100
                    cl.setRPM1(100);
                    cl.setRPM2(-100);
                break;
                default:
                    cl.setPID(1.4, 1.4, 0);
                    cl.stopMotors();         // Stop the motors
                    test=0;

            }
            // Increment command index, loop back if it exceeds the number of commands
            commandIndex = (commandIndex + 1);  // We have 4 commands, so we loop after the last command
        }
    }

    // Optionally, process serial commands for dynamic setpoints
    // processSerialCommands();
}


