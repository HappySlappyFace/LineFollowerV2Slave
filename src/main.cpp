#include <Arduino.h>


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

// ============================
// === Function Prototypes ===
// ============================

void IRAM_ATTR encoder1ISR();
void IRAM_ATTR encoder2ISR();
void setupPWM();
void initializeEncoders();
void calculateRPM();
void calculatePID();
void applyMotorControl();
double constrainValue(double value, double minVal, double maxVal);
void processSerialCommands();

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

    // Button Handling for PID Tuning (Optional)
    int reading = digitalRead(BUTTON_PIN);

    if (reading != lastButtonState) {
        lastDebounceTime = currentTime;
    }

    processSerialCommands();
//    if ((currentTime - lastDebounceTime) > DEBOUNCE_DELAY) {
//        if (reading != buttonState) {
//            buttonState = reading;
//            if (buttonState == LOW) { // Button Pressed
//                // Example: Increment setpoint RPM
//                setpointRPM1 += 10.0;
//                setpointRPM2 += 10.0;
//                Serial.print("Setpoint RPM1: ");
//                Serial.print(setpointRPM1);
//                Serial.print(", Setpoint RPM2: ");
//                Serial.println(setpointRPM2);
//            }
//        }
//    }

    lastButtonState = reading;

    // Optionally, process serial commands for dynamic setpoints
    processSerialCommands();
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
    // RPM = (deltaTicks / ticksPerRevolution) * (60 / deltaTime)
    // Here, deltaTime = RPM_CALC_INTERVAL in seconds
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
// === PID Calculation Function ===
// ============================

void calculatePID() {
    // Calculate Error (Setpoint RPM - Current RPM)
    error1 = setpointRPM1 - currentRPM1;
    error2 = setpointRPM2 - currentRPM2;

    // Implement Deadband
    if (abs(error1) < deadband) {
        error1 = 0.0;
    }

    if (abs(error2) < deadband) {
        error2 = 0.0;
    }

    // Reset Integral and Derivative Terms if Error is Zero
    if (error1 == 0.0) {
        integral1 = 0.0;
        derivative1 = 0.0;
    } else {
        // Calculate Integral with Anti-Windup
        integral1 += error1 * (PID_INTERVAL / 1000.0);

        // Constrain Integral Term
        if (integral1 > integralMax) integral1 = integralMax;
        if (integral1 < integralMin) integral1 = integralMin;
    }

    if (error2 == 0.0) {
        integral2 = 0.0;
        derivative2 = 0.0;
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
    output1 = constrainValue(output1, -255.0, 255.0);
    output2 = constrainValue(output2, -255.0, 255.0);

    // Save Current Error for Next Derivative Calculation
    previousError1 = error1;
    previousError2 = error2;

    // Debugging Information
    if(debugPID){

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

// ===============================
// === Process Serial Commands ===
// ===============================

// Define a buffer to accumulate incoming serial data
String serialBuffer = "";

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
