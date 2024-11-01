#ifndef COMMAND_LIBRARY_H
#define COMMAND_LIBRARY_H

#include <Arduino.h>

class CommandLibrary {
public:
    CommandLibrary(bool debug);  // Constructor with debug option

    // Methods to control motor speeds and PID values
    void setRPM1(double rpm);
    void setRPM2(double rpm);
    void setRPM(double rpm1, double rpm2);
    void setPID(double kp, double ki, double kd);
    void setSpeed(double speed);
    void stopMotors();
    void setTargetError(double targetValue);

    void setTarget(double error);

private:
    bool debugMode;  // To control debug output
    void debugPrint(const String& message);  // Helper method for debug printing
};

#endif  // COMMAND_LIBRARY_H
