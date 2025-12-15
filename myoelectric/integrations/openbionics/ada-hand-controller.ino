/**
 * WIA Controller for OpenBionics Ada Hand
 *
 * Optimized controller for the Ada Hand, a cable-driven
 * 3D printed prosthetic hand designed by OpenBionics.
 *
 * Ada Hand Specifications:
 * - 5 fingers, cable-driven
 * - Single actuator (body-powered or motor)
 * - 1 DOF wrist rotation (optional servo)
 * - Designed for transradial amputees
 *
 * Hardware:
 * - Arduino Nano 33 BLE or ESP32
 * - 1x Main grip motor (DC motor with encoder or servo)
 * - 1x Wrist rotation servo (optional)
 * - WIA EMG Module
 *
 * License: MIT (Hardware: CC BY-SA per OpenBionics)
 */

#include <Servo.h>
#ifdef ARDUINO_ARCH_ESP32
#include <ESP32Servo.h>
#endif

// Ada Hand specific pins
const int GRIP_MOTOR_PWM = 5;
const int GRIP_MOTOR_DIR = 6;
const int GRIP_ENCODER_A = 2;
const int GRIP_ENCODER_B = 3;
const int WRIST_SERVO_PIN = 9;

// Limit switch for grip protection
const int GRIP_LIMIT_OPEN = 7;
const int GRIP_LIMIT_CLOSE = 8;

// Motor driver
Servo wristServo;

// Gesture enumeration
enum Gesture {
    GESTURE_REST = 0,
    GESTURE_HAND_OPEN = 1,
    GESTURE_HAND_CLOSE = 2,
    GESTURE_WRIST_FLEXION = 3,
    GESTURE_WRIST_EXTENSION = 4
};

// Grip motor state
volatile long encoderCount = 0;
const long ENCODER_OPEN = 0;
const long ENCODER_CLOSE = 500;  // Adjust based on cable travel

// Wrist position
int wristPosition = 90;  // Neutral
const int WRIST_MIN = 0;
const int WRIST_MAX = 180;

// Control parameters
const int MOTOR_SPEED = 200;     // PWM value 0-255
const int GRIP_TOLERANCE = 10;   // Encoder counts
float emgLevel = 0.0;
bool proportionalMode = true;

// Safety
const unsigned long GRIP_TIMEOUT_MS = 3000;
const int MAX_MOTOR_CURRENT = 500;  // mA (requires current sensing)

void setup() {
    Serial.begin(115200);
    Serial.println("WIA Ada Hand Controller v1.0");

    // Motor pins
    pinMode(GRIP_MOTOR_PWM, OUTPUT);
    pinMode(GRIP_MOTOR_DIR, OUTPUT);

    // Encoder pins
    pinMode(GRIP_ENCODER_A, INPUT_PULLUP);
    pinMode(GRIP_ENCODER_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(GRIP_ENCODER_A), encoderISR, RISING);

    // Limit switches
    pinMode(GRIP_LIMIT_OPEN, INPUT_PULLUP);
    pinMode(GRIP_LIMIT_CLOSE, INPUT_PULLUP);

    // Wrist servo
    wristServo.attach(WRIST_SERVO_PIN);
    wristServo.write(wristPosition);

    // Home the grip
    homeGrip();

    Serial.println("Ready. Commands: 0-4 (gestures), p:0.0-1.0 (proportional)");
}

void loop() {
    handleSerialInput();
    delay(10);
}

void encoderISR() {
    if (digitalRead(GRIP_ENCODER_B)) {
        encoderCount++;
    } else {
        encoderCount--;
    }
}

void handleSerialInput() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.startsWith("p:")) {
            float level = input.substring(2).toFloat();
            setProportionalGrip(level);
        }
        else if (input.startsWith("emg:")) {
            emgLevel = input.substring(4).toFloat();
            if (proportionalMode) {
                setProportionalGrip(emgLevel);
            }
        }
        else if (input.length() == 1 && isDigit(input[0])) {
            executeGesture((Gesture)input.toInt());
        }
        else if (input == "home") {
            homeGrip();
        }
        else if (input == "status") {
            printStatus();
        }
    }
}

void executeGesture(Gesture gesture) {
    Serial.print("Executing gesture: ");
    Serial.println(gesture);

    switch (gesture) {
        case GESTURE_REST:
        case GESTURE_HAND_OPEN:
            openGrip();
            break;

        case GESTURE_HAND_CLOSE:
            closeGrip();
            break;

        case GESTURE_WRIST_FLEXION:
            // Ada Hand: wrist rotation for flexion gesture
            rotateWrist(-30);
            break;

        case GESTURE_WRIST_EXTENSION:
            rotateWrist(30);
            break;
    }
}

void setProportionalGrip(float level) {
    level = constrain(level, 0.0, 1.0);
    long targetPosition = (long)(level * ENCODER_CLOSE);

    Serial.print("Proportional grip: ");
    Serial.print(level * 100);
    Serial.println("%");

    moveGripToPosition(targetPosition);
}

void openGrip() {
    moveGripToPosition(ENCODER_OPEN);
}

void closeGrip() {
    moveGripToPosition(ENCODER_CLOSE);
}

void moveGripToPosition(long targetPosition) {
    unsigned long startTime = millis();

    while (abs(encoderCount - targetPosition) > GRIP_TOLERANCE) {
        // Timeout protection
        if (millis() - startTime > GRIP_TIMEOUT_MS) {
            stopMotor();
            Serial.println("Grip timeout!");
            return;
        }

        // Limit switch protection
        if (!digitalRead(GRIP_LIMIT_OPEN) && targetPosition < encoderCount) {
            encoderCount = ENCODER_OPEN;
            stopMotor();
            return;
        }
        if (!digitalRead(GRIP_LIMIT_CLOSE) && targetPosition > encoderCount) {
            encoderCount = ENCODER_CLOSE;
            stopMotor();
            return;
        }

        // Move motor
        if (encoderCount < targetPosition) {
            // Close (direction depends on wiring)
            digitalWrite(GRIP_MOTOR_DIR, HIGH);
            analogWrite(GRIP_MOTOR_PWM, MOTOR_SPEED);
        } else {
            // Open
            digitalWrite(GRIP_MOTOR_DIR, LOW);
            analogWrite(GRIP_MOTOR_PWM, MOTOR_SPEED);
        }

        delay(10);
    }

    stopMotor();
}

void stopMotor() {
    analogWrite(GRIP_MOTOR_PWM, 0);
}

void homeGrip() {
    Serial.println("Homing grip...");

    // Move to open limit
    digitalWrite(GRIP_MOTOR_DIR, LOW);
    analogWrite(GRIP_MOTOR_PWM, MOTOR_SPEED / 2);

    unsigned long startTime = millis();
    while (digitalRead(GRIP_LIMIT_OPEN)) {
        if (millis() - startTime > GRIP_TIMEOUT_MS) {
            stopMotor();
            Serial.println("Homing failed!");
            return;
        }
        delay(10);
    }

    stopMotor();
    encoderCount = ENCODER_OPEN;
    Serial.println("Homing complete.");
}

void rotateWrist(int degrees) {
    wristPosition = constrain(wristPosition + degrees, WRIST_MIN, WRIST_MAX);
    wristServo.write(wristPosition);

    Serial.print("Wrist: ");
    Serial.println(wristPosition);
}

void printStatus() {
    Serial.println("=== Ada Hand Status ===");
    Serial.print("Encoder: ");
    Serial.println(encoderCount);
    Serial.print("Wrist: ");
    Serial.println(wristPosition);
    Serial.print("EMG Level: ");
    Serial.println(emgLevel);
    Serial.print("Limit Open: ");
    Serial.println(!digitalRead(GRIP_LIMIT_OPEN) ? "TRIGGERED" : "OK");
    Serial.print("Limit Close: ");
    Serial.println(!digitalRead(GRIP_LIMIT_CLOSE) ? "TRIGGERED" : "OK");
}
