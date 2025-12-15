/**
 * WIA Controller for OpenBionics Brunel Hand
 *
 * Controller for the Brunel Hand, an individually-actuated
 * 5-finger robotic hand designed by OpenBionics.
 *
 * Brunel Hand Specifications:
 * - 5 fingers, individually actuated
 * - 5x Micro linear actuators
 * - 1 DOF wrist rotation
 * - Flexible fingertips for grip conformity
 *
 * Hardware:
 * - Arduino Mega or ESP32
 * - 5x Actuonix L12-30-50-6-P linear actuators
 * - 1x Wrist servo
 * - WIA EMG Module (BLE)
 *
 * License: MIT (Hardware: CC BY-SA per OpenBionics)
 */

#include <Servo.h>

// Brunel Hand actuator pins (PWM capable)
const int THUMB_PIN = 2;
const int INDEX_PIN = 3;
const int MIDDLE_PIN = 4;
const int RING_PIN = 5;
const int PINKY_PIN = 6;
const int WRIST_PIN = 7;

// Actuonix linear actuators (use Servo library)
Servo thumbActuator;
Servo indexActuator;
Servo middleActuator;
Servo ringActuator;
Servo pinkyActuator;
Servo wristServo;

// Gesture definitions
enum Gesture {
    GESTURE_REST = 0,
    GESTURE_HAND_OPEN = 1,
    GESTURE_HAND_CLOSE = 2,
    GESTURE_WRIST_FLEXION = 3,
    GESTURE_WRIST_EXTENSION = 4,
    GESTURE_PINCH = 5,
    GESTURE_TRIPOD = 6,
    GESTURE_POINT = 7,
    GESTURE_KEY_GRIP = 8,
    GESTURE_HOOK = 9
};

// Finger positions (0 = open, 100 = closed)
struct FingerState {
    int thumb;
    int index;
    int middle;
    int ring;
    int pinky;
};

FingerState currentState = {0, 0, 0, 0, 0};
FingerState targetState = {0, 0, 0, 0, 0};

// Wrist state
int wristPosition = 90;

// Control parameters
const float MOVEMENT_SPEED = 0.15;  // Smooth interpolation factor
const int UPDATE_INTERVAL_MS = 20;
float emgLevel = 0.0;
bool proportionalEnabled = true;

// Grip presets for Brunel Hand
struct GripPreset {
    FingerState fingers;
    const char* name;
};

GripPreset GRIP_PRESETS[] = {
    {{0, 0, 0, 0, 0}, "Open"},
    {{100, 100, 100, 100, 100}, "Power Grip"},
    {{80, 80, 0, 0, 0}, "Pinch"},
    {{70, 70, 70, 0, 0}, "Tripod"},
    {{100, 0, 100, 100, 100}, "Point"},
    {{90, 70, 40, 40, 40}, "Key Grip"},
    {{0, 100, 100, 100, 100}, "Hook"},
};

void setup() {
    Serial.begin(115200);
    Serial.println("WIA Brunel Hand Controller v1.0");

    // Attach actuators
    thumbActuator.attach(THUMB_PIN);
    indexActuator.attach(INDEX_PIN);
    middleActuator.attach(MIDDLE_PIN);
    ringActuator.attach(RING_PIN);
    pinkyActuator.attach(PINKY_PIN);
    wristServo.attach(WRIST_PIN);

    // Initialize to open position
    applyGrip(GRIP_PRESETS[0].fingers);
    wristServo.write(wristPosition);

    Serial.println("Ready. Commands: 0-9 (gestures), p:0.0-1.0 (proportional)");
    Serial.println("Grips: open, power, pinch, tripod, point, key, hook");
}

void loop() {
    handleSerialInput();
    updateActuators();
    delay(UPDATE_INTERVAL_MS);
}

void handleSerialInput() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        input.toLowerCase();

        // Proportional control
        if (input.startsWith("p:")) {
            float level = input.substring(2).toFloat();
            setProportionalGrip(level);
        }
        // EMG level from BLE
        else if (input.startsWith("emg:")) {
            emgLevel = input.substring(4).toFloat();
            if (proportionalEnabled) {
                setProportionalGrip(emgLevel);
            }
        }
        // Gesture number
        else if (input.length() == 1 && isDigit(input[0])) {
            executeGesture((Gesture)input.toInt());
        }
        // Named grips
        else if (input == "open") { applyGrip(GRIP_PRESETS[0].fingers); }
        else if (input == "power") { applyGrip(GRIP_PRESETS[1].fingers); }
        else if (input == "pinch") { applyGrip(GRIP_PRESETS[2].fingers); }
        else if (input == "tripod") { applyGrip(GRIP_PRESETS[3].fingers); }
        else if (input == "point") { applyGrip(GRIP_PRESETS[4].fingers); }
        else if (input == "key") { applyGrip(GRIP_PRESETS[5].fingers); }
        else if (input == "hook") { applyGrip(GRIP_PRESETS[6].fingers); }
        // Individual finger control: "f:1,80" (finger 1 to 80%)
        else if (input.startsWith("f:")) {
            int comma = input.indexOf(',');
            if (comma > 2) {
                int finger = input.substring(2, comma).toInt();
                int pos = input.substring(comma + 1).toInt();
                setIndividualFinger(finger, pos);
            }
        }
        // Status
        else if (input == "status") {
            printStatus();
        }
    }
}

void executeGesture(Gesture gesture) {
    Serial.print("Gesture: ");
    Serial.println(gesture);

    switch (gesture) {
        case GESTURE_REST:
        case GESTURE_HAND_OPEN:
            applyGrip(GRIP_PRESETS[0].fingers);
            break;

        case GESTURE_HAND_CLOSE:
            applyGrip(GRIP_PRESETS[1].fingers);
            break;

        case GESTURE_PINCH:
            applyGrip(GRIP_PRESETS[2].fingers);
            break;

        case GESTURE_TRIPOD:
            applyGrip(GRIP_PRESETS[3].fingers);
            break;

        case GESTURE_POINT:
            applyGrip(GRIP_PRESETS[4].fingers);
            break;

        case GESTURE_KEY_GRIP:
            applyGrip(GRIP_PRESETS[5].fingers);
            break;

        case GESTURE_HOOK:
            applyGrip(GRIP_PRESETS[6].fingers);
            break;

        case GESTURE_WRIST_FLEXION:
            rotateWrist(-20);
            break;

        case GESTURE_WRIST_EXTENSION:
            rotateWrist(20);
            break;
    }
}

void applyGrip(FingerState grip) {
    targetState = grip;
}

void setIndividualFinger(int finger, int position) {
    position = constrain(position, 0, 100);

    switch (finger) {
        case 0: targetState.thumb = position; break;
        case 1: targetState.index = position; break;
        case 2: targetState.middle = position; break;
        case 3: targetState.ring = position; break;
        case 4: targetState.pinky = position; break;
    }

    Serial.print("Finger ");
    Serial.print(finger);
    Serial.print(": ");
    Serial.println(position);
}

void setProportionalGrip(float level) {
    level = constrain(level, 0.0, 1.0);
    int pos = (int)(level * 100);

    targetState.thumb = pos;
    targetState.index = pos;
    targetState.middle = pos;
    targetState.ring = pos;
    targetState.pinky = pos;

    Serial.print("Proportional: ");
    Serial.print(level * 100);
    Serial.println("%");
}

void rotateWrist(int degrees) {
    wristPosition = constrain(wristPosition + degrees, 0, 180);
    wristServo.write(wristPosition);

    Serial.print("Wrist: ");
    Serial.println(wristPosition);
}

void updateActuators() {
    // Smooth interpolation
    currentState.thumb = smoothStep(currentState.thumb, targetState.thumb);
    currentState.index = smoothStep(currentState.index, targetState.index);
    currentState.middle = smoothStep(currentState.middle, targetState.middle);
    currentState.ring = smoothStep(currentState.ring, targetState.ring);
    currentState.pinky = smoothStep(currentState.pinky, targetState.pinky);

    // Apply to actuators (Actuonix uses 1000-2000us pulse width)
    thumbActuator.writeMicroseconds(positionToPulse(currentState.thumb));
    indexActuator.writeMicroseconds(positionToPulse(currentState.index));
    middleActuator.writeMicroseconds(positionToPulse(currentState.middle));
    ringActuator.writeMicroseconds(positionToPulse(currentState.ring));
    pinkyActuator.writeMicroseconds(positionToPulse(currentState.pinky));
}

int smoothStep(int current, int target) {
    if (current == target) return current;

    float diff = target - current;
    int step = (int)(diff * MOVEMENT_SPEED);

    if (abs(step) < 1) {
        step = (diff > 0) ? 1 : -1;
    }

    return constrain(current + step, 0, 100);
}

int positionToPulse(int position) {
    // Actuonix L12: 1000us = fully retracted, 2000us = fully extended
    // Map 0-100% to 1000-2000us (invert if needed for close = extended)
    return map(position, 0, 100, 1000, 2000);
}

void printStatus() {
    Serial.println("=== Brunel Hand Status ===");
    Serial.print("Thumb:  "); Serial.println(currentState.thumb);
    Serial.print("Index:  "); Serial.println(currentState.index);
    Serial.print("Middle: "); Serial.println(currentState.middle);
    Serial.print("Ring:   "); Serial.println(currentState.ring);
    Serial.print("Pinky:  "); Serial.println(currentState.pinky);
    Serial.print("Wrist:  "); Serial.println(wristPosition);
    Serial.print("EMG:    "); Serial.println(emgLevel);
}
