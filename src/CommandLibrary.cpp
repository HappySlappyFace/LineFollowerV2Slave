#include "CommandLibrary.h"

// External variables, assuming they are declared elsewhere
extern double setpointRPM1;
extern double setpointRPM2;
extern double kp, ki, kd;
extern double targetError;

// Constructor with optional debug parameter
CommandLibrary::CommandLibrary(bool debug) : debugMode(debug) {}

// Method to set RPM for Motor 1
void CommandLibrary::setRPM1(double rpm) {
    setpointRPM1 = rpm;
    debugPrint("Set RPM1 to: " + String(rpm));
}

// Method to set RPM for Motor 2
void CommandLibrary::setRPM2(double rpm) {
    setpointRPM2 = rpm;
    debugPrint("Set RPM2 to: " + String(rpm));
}

// Method to set RPM for both motors
void CommandLibrary::setRPM(double rpm1, double rpm2) {
    setpointRPM1 = rpm1;
    setpointRPM2 = rpm2;
    debugPrint("Set RPM1 to: " + String(rpm1) + ", RPM2 to: " + String(rpm2));
}

// Method to set PID values
void CommandLibrary::setPID(double kpValue, double kiValue, double kdValue) {
    kp = kpValue;
    ki = kiValue;
    kd = kdValue;
    debugPrint("Set PID values - kp: " + String(kp) + ", ki: " + String(ki) + ", kd: " + String(kd));
}

// Method to set speed for both motors
void CommandLibrary::setSpeed(double speed) {
    setpointRPM1 = speed;
    setpointRPM2 = speed;
    debugPrint("Set speed for both motors to: " + String(speed));
}

// Method to stop both motors
void CommandLibrary::stopMotors() {
    setpointRPM1 = 0;
    setpointRPM2 = 0;
    targetError = 0;  // Reset target error as well
    debugPrint("Motors stopped.");
}

// Method to set a target error value for PID control
void CommandLibrary::setTargetError(double targetValue) {
    targetError = targetValue;
    debugPrint("Set target (error) value to: " + String(targetValue));
}

void CommandLibrary::setTarget(double error) {
    targetError = error;

}

// Private helper method for debug printing
void CommandLibrary::debugPrint(const String& message) {
    if (debugMode) {
        Serial.println(message);
    }
}
