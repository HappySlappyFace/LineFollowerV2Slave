//
// Created by hsf on 10/11/24.
//

#include "motor.h"
#include "declarations.h"

// ===============================
// === Process Serial Commands ===
// ===============================

// Define a buffer to accumulate incoming serial data

void processSerialCommands() {
    // Check if data is available in the custom serial buffer
    while (mySerial.available()) {
        // Read the incoming byte
        char incomingByte = mySerial.read();

        // If the byte is a newline character, process the command
        if (incomingByte == '\n') {
            // Convert the accumulated command into a String
            String command = serialBuffer;
            serialBuffer = ""; // Clear the buffer for the next command
            command.trim(); // Remove any trailing whitespace
            if(debugCommunication){
                Serial.println(command); // Debug: print the full command received
            }

            // Process the command
            if (command.startsWith("SETRPM1:")) {
                // Extract the RPM value for Motor 1
                String rpmStr = command.substring(8); // After "SETRPM1:"
                double rpm = rpmStr.toDouble();

                // Update the setpoint for Motor 1
                setpointRPM1 = rpm;

                // Provide feedback
                if(debugCommunication){
                    Serial.print("Setpoint RPM1 set to: ");
                    Serial.println(setpointRPM1);
                }
            }
            else if (command.startsWith("SETRPM2:")) {
                // Extract the RPM value for Motor 2
                String rpmStr = command.substring(8); // After "SETRPM2:"
                double rpm = rpmStr.toDouble();

                // Update the setpoint for Motor 2
                setpointRPM2 = rpm;

                // Provide feedback
                if(debugCommunication){
                    Serial.print("Setpoint RPM2 set to: ");
                    Serial.println(setpointRPM2);
                }
            }
            else if (command.startsWith("SETRPM:")) {
                // Extract RPM values for both motors
                String rpmValues = command.substring(7); // After "SETRPM:"
                int commaIndex = rpmValues.indexOf(',');

                if (commaIndex > 0) {
                    String rpm1Str = rpmValues.substring(0, commaIndex);
                    String rpm2Str = rpmValues.substring(commaIndex + 1);

                    double rpm1 = rpm1Str.toDouble();
                    double rpm2 = rpm2Str.toDouble();

                    // Update setpoints
                    setpointRPM1 = rpm1;
                    setpointRPM2 = rpm2;

                    // Provide feedback
                    if(debugCommunication){
                        Serial.print("Setpoint RPM1 set to: ");
                        Serial.print(setpointRPM1);
                        Serial.print(", Setpoint RPM2 set to: ");
                        Serial.println(setpointRPM2);
                    }
                }
                else {
                    // Invalid command format
//                    mySerial.println("Invalid SETRPM command format. Use SETRPM:RPM1,RPM2");
                    if(debugCommunication){
                        Serial.println("Invalid SETRPM command format.");
                    }
                }
            }
            else if (command.startsWith("SETPID:")) {
                // Extract PID values (kp, ki, kd)
                String pidValues = command.substring(7); // After "SETPID:"
                int comma1 = pidValues.indexOf(',');
                int comma2 = pidValues.indexOf(',', comma1 + 1);

                if (comma1 > 0 && comma2 > comma1) {
                    kp = pidValues.substring(0, comma1).toDouble();
                    ki = pidValues.substring(comma1 + 1, comma2).toDouble();
                    kd = pidValues.substring(comma2 + 1).toDouble();

                    // Provide feedback
                    if(debugCommunication){
                        Serial.print("Set PID values - kp: ");
                        Serial.print(kp);
                        Serial.print(", ki: ");
                        Serial.print(ki);
                        Serial.print(", kd: ");
                        Serial.println(kd);
                    }
                } else {
                    // Invalid PID format
                    Serial.println("Invalid SETPID command format. Use SETPID:kP,kI,kD");
                }
            }
            else if (command.startsWith("SETSPD:")) {
                // Extract speed value for both motors
                String speedStr = command.substring(7); // After "SETSPD:"
                double speed = speedStr.toDouble();

                // Update setpoints
                setpointRPM1 = speed;
                setpointRPM2 = speed;

                // Provide feedback
                if(debugCommunication){

                    Serial.print("Set speed to: ");
                    Serial.println(speed);
                }
            }
            else if (command.startsWith("STOP")) {
                // Stop motors
                setpointRPM1 = 0;
                setpointRPM2 = 0;
                applyMotorControl();  // Stop both motors

                // Provide feedback
                if(debugCommunication){

                    Serial.println("Motors stopped.");
                }
            }
            else if (command.startsWith("SETTGT:")) {
                // Extract target value (this will be used as the error for PID)
                String targetStr = command.substring(7); // After "SETTGT:"
                double targetValue = targetStr.toDouble();

                // Handle the target value as per your logic (for now just print)
                if(debugCommunication){

                    Serial.print("Set target (error) value to: ");
                    Serial.println(targetValue);
                }

                // This value can be assigned to a global variable you use in PID
                // targetError = targetValue;
            }
            else {
                // Unknown command
                if(debugCommunication){

                    Serial.println("Unknown command. Available commands:");
                    Serial.println("SETRPM1:<value> - Set RPM for Motor 1");
                    Serial.println("SETRPM2:<value> - Set RPM for Motor 2");
                    Serial.println("SETRPM:<value1>,<value2> - Set RPM for both motors");
                    Serial.println("SETPID:<kp>,<ki>,<kd> - Set PID values");
                    Serial.println("SETSPD:<value> - Set speed for both motors");
                    Serial.println("STOP - Stop the motors");
                }
            }
        }
        else {
            // Accumulate the incoming bytes into a buffer
            serialBuffer += incomingByte;
        }
    }
}