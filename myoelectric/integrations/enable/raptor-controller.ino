/**
 * WIA Controller for e-NABLE Raptor Reloaded
 *
 * EMG control adaptation for the Raptor Reloaded,
 * originally a body-powered prosthetic hand from e-NABLE.
 *
 * Raptor Reloaded Specifications:
 * - 5 fingers, cable-driven (single cable)
 * - Originally wrist-powered, adapted for servo/motor
 * - Designed for children and adults
 * - Simple, robust design
 *
 * Motorization Options:
 * 1. Single servo at wrist (simple)
 * 2. Linear actuator on cable (stronger)
 * 3. DC motor with lead screw (most powerful)
 *
 * Hardware:
 * - Arduino Nano or ESP32
 * - 1x Actuator (servo/linear/DC motor)
 * - WIA EMG Module (BLE)
 *
 * License: MIT (Hardware: CC BY-NC per e-NABLE)
 */

#include <Servo.h>

// Motorization type - uncomment your setup
#define ACTUATOR_SERVO
// #define ACTUATOR_LINEAR
// #define ACTUATOR_DC_MOTOR

// Pin definitions
#ifdef ACTUATOR_SERVO
const int GRIP_SERVO_PIN = 9;
Servo gripServo;
#endif

#ifdef ACTUATOR_LINEAR
const int LINEAR_PIN = 9;
Servo linearActuator;
#endif

#ifdef ACTUATOR_DC_MOTOR
const int MOTOR_PWM = 5;
const int MOTOR_DIR = 6;
const int MOTOR_ENCODER_A = 2;
const int MOTOR_ENCODER_B = 3;
volatile long encoderCount = 0;
const long ENCODER_MAX = 1000;
#endif

// Common pins
const int LIMIT_OPEN = 7;
const int LIMIT_CLOSE = 8;
const int LED_STATUS = 13;

// Gestures (simplified for single-DOF)
enum Gesture {
    GESTURE_REST = 0,
    GESTURE_HAND_OPEN = 1,
    GESTURE_HAND_CLOSE = 2
};

// Grip state
int currentGrip = 0;    // 0-100%
int targetGrip = 0;
float emgLevel = 0.0;

// Control parameters
const float GRIP_SPEED = 0.1;
const int UPDATE_MS = 20;
bool proportionalEnabled = true;

// Calibration
int servoOpen = 0;      // Servo angle when open
int servoClose = 180;   // Servo angle when closed

void setup() {
    Serial.begin(115200);
    Serial.println("WIA Raptor Reloaded Controller v1.0");

    pinMode(LED_STATUS, OUTPUT);
    pinMode(LIMIT_OPEN, INPUT_PULLUP);
    pinMode(LIMIT_CLOSE, INPUT_PULLUP);

    #ifdef ACTUATOR_SERVO
    gripServo.attach(GRIP_SERVO_PIN);
    gripServo.write(servoOpen);
    Serial.println("Mode: Servo");
    #endif

    #ifdef ACTUATOR_LINEAR
    linearActuator.attach(LINEAR_PIN);
    linearActuator.writeMicroseconds(1000);  // Retracted
    Serial.println("Mode: Linear Actuator");
    #endif

    #ifdef ACTUATOR_DC_MOTOR
    pinMode(MOTOR_PWM, OUTPUT);
    pinMode(MOTOR_DIR, OUTPUT);
    pinMode(MOTOR_ENCODER_A, INPUT_PULLUP);
    pinMode(MOTOR_ENCODER_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(MOTOR_ENCODER_A), encoderISR, RISING);
    homeMotor();
    Serial.println("Mode: DC Motor");
    #endif

    blinkLED(3);
    Serial.println("Ready. Commands: 0/1/2 (open/close), p:0.0-1.0");
}

void loop() {
    handleSerial();
    updateGrip();
    updateLED();
    delay(UPDATE_MS);
}

#ifdef ACTUATOR_DC_MOTOR
void encoderISR() {
    if (digitalRead(MOTOR_ENCODER_B)) {
        encoderCount++;
    } else {
        encoderCount--;
    }
}

void homeMotor() {
    Serial.println("Homing...");
    digitalWrite(MOTOR_DIR, LOW);
    analogWrite(MOTOR_PWM, 150);

    unsigned long start = millis();
    while (digitalRead(LIMIT_OPEN) && (millis() - start < 5000)) {
        delay(10);
    }
    analogWrite(MOTOR_PWM, 0);
    encoderCount = 0;
    Serial.println("Homed.");
}
#endif

void handleSerial() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        // Proportional command
        if (input.startsWith("p:")) {
            float level = input.substring(2).toFloat();
            setGrip(level);
        }
        // EMG from BLE module
        else if (input.startsWith("emg:")) {
            emgLevel = input.substring(4).toFloat();
            if (proportionalEnabled) {
                setGrip(emgLevel);
            }
        }
        // Gesture command
        else if (input == "0" || input == "open") {
            targetGrip = 0;
            Serial.println("Opening");
        }
        else if (input == "1") {
            targetGrip = 0;
            Serial.println("Opening (rest)");
        }
        else if (input == "2" || input == "close") {
            targetGrip = 100;
            Serial.println("Closing");
        }
        // Calibration
        else if (input.startsWith("cal:")) {
            int comma = input.indexOf(',');
            if (comma > 4) {
                servoOpen = input.substring(4, comma).toInt();
                servoClose = input.substring(comma + 1).toInt();
                Serial.print("Calibrated: open=");
                Serial.print(servoOpen);
                Serial.print(", close=");
                Serial.println(servoClose);
            }
        }
        // Toggle proportional mode
        else if (input == "prop") {
            proportionalEnabled = !proportionalEnabled;
            Serial.print("Proportional: ");
            Serial.println(proportionalEnabled ? "ON" : "OFF");
        }
        // Status
        else if (input == "status") {
            printStatus();
        }
    }
}

void setGrip(float level) {
    level = constrain(level, 0.0, 1.0);
    targetGrip = (int)(level * 100);
}

void updateGrip() {
    // Check limits
    if (!digitalRead(LIMIT_OPEN) && targetGrip < currentGrip) {
        currentGrip = 0;
        targetGrip = 0;
    }
    if (!digitalRead(LIMIT_CLOSE) && targetGrip > currentGrip) {
        currentGrip = 100;
        targetGrip = 100;
    }

    // Smooth interpolation
    if (currentGrip != targetGrip) {
        int diff = targetGrip - currentGrip;
        int step = (int)(diff * GRIP_SPEED);
        if (abs(step) < 1) step = (diff > 0) ? 1 : -1;
        currentGrip = constrain(currentGrip + step, 0, 100);
    }

    // Apply to actuator
    #ifdef ACTUATOR_SERVO
    int angle = map(currentGrip, 0, 100, servoOpen, servoClose);
    gripServo.write(angle);
    #endif

    #ifdef ACTUATOR_LINEAR
    int pulse = map(currentGrip, 0, 100, 1000, 2000);
    linearActuator.writeMicroseconds(pulse);
    #endif

    #ifdef ACTUATOR_DC_MOTOR
    long targetPos = map(targetGrip, 0, 100, 0, ENCODER_MAX);
    if (abs(encoderCount - targetPos) > 10) {
        if (encoderCount < targetPos) {
            digitalWrite(MOTOR_DIR, HIGH);
            analogWrite(MOTOR_PWM, 200);
        } else {
            digitalWrite(MOTOR_DIR, LOW);
            analogWrite(MOTOR_PWM, 200);
        }
    } else {
        analogWrite(MOTOR_PWM, 0);
    }
    currentGrip = map(encoderCount, 0, ENCODER_MAX, 0, 100);
    #endif
}

void updateLED() {
    // LED indicates grip level
    if (currentGrip > 50) {
        digitalWrite(LED_STATUS, HIGH);
    } else if (currentGrip > 0) {
        // Blink for partial grip
        digitalWrite(LED_STATUS, (millis() / 200) % 2);
    } else {
        digitalWrite(LED_STATUS, LOW);
    }
}

void blinkLED(int times) {
    for (int i = 0; i < times; i++) {
        digitalWrite(LED_STATUS, HIGH);
        delay(100);
        digitalWrite(LED_STATUS, LOW);
        delay(100);
    }
}

void printStatus() {
    Serial.println("=== Raptor Status ===");
    Serial.print("Grip: ");
    Serial.print(currentGrip);
    Serial.println("%");
    Serial.print("Target: ");
    Serial.print(targetGrip);
    Serial.println("%");
    Serial.print("EMG: ");
    Serial.println(emgLevel);
    Serial.print("Proportional: ");
    Serial.println(proportionalEnabled ? "ON" : "OFF");
    Serial.print("Limit Open: ");
    Serial.println(!digitalRead(LIMIT_OPEN) ? "TRIGGERED" : "OK");
    Serial.print("Limit Close: ");
    Serial.println(!digitalRead(LIMIT_CLOSE) ? "TRIGGERED" : "OK");

    #ifdef ACTUATOR_DC_MOTOR
    Serial.print("Encoder: ");
    Serial.println(encoderCount);
    #endif
}
