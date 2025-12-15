/**
 * WIA Generic Servo Controller
 *
 * Universal servo controller for 3D printed prosthetic hands.
 * Compatible with any 5-finger servo-based prosthetic design.
 *
 * Hardware:
 * - Arduino Nano or ESP32
 * - 5x SG90/MG90S micro servos
 * - WIA EMG Module (BLE connection)
 *
 * License: MIT
 */

#include <Servo.h>

// Pin definitions
const int THUMB_PIN = 3;
const int INDEX_PIN = 5;
const int MIDDLE_PIN = 6;
const int RING_PIN = 9;
const int PINKY_PIN = 10;

// Servo objects
Servo thumbServo;
Servo indexServo;
Servo middleServo;
Servo ringServo;
Servo pinkyServo;

// Gesture enumeration matching WIA standard
enum Gesture {
    GESTURE_REST = 0,
    GESTURE_HAND_OPEN = 1,
    GESTURE_HAND_CLOSE = 2,
    GESTURE_WRIST_FLEXION = 3,
    GESTURE_WRIST_EXTENSION = 4,
    GESTURE_PINCH = 5,
    GESTURE_TRIPOD = 6,
    GESTURE_POINT = 7
};

// Servo calibration (adjust per prosthetic design)
struct ServoCalibration {
    int openAngle;   // Finger extended
    int closeAngle;  // Finger fully closed
};

ServoCalibration thumbCal = {0, 180};
ServoCalibration indexCal = {0, 180};
ServoCalibration middleCal = {0, 180};
ServoCalibration ringCal = {0, 180};
ServoCalibration pinkyCal = {0, 180};

// Safety parameters
const int MAX_GRIP_FORCE = 30;     // Newtons (approximate)
const float SERVO_SPEED = 0.5;     // 0-1, controls transition speed
const int POSITION_UPDATE_MS = 20; // Position update interval

// Current finger positions (0-100%)
int currentPositions[5] = {0, 0, 0, 0, 0};
int targetPositions[5] = {0, 0, 0, 0, 0};

// EMG proportional control
float emgLevel = 0.0;
bool proportionalEnabled = true;

void setup() {
    Serial.begin(115200);
    Serial.println("WIA Generic Servo Controller v1.0");

    // Attach servos
    thumbServo.attach(THUMB_PIN);
    indexServo.attach(INDEX_PIN);
    middleServo.attach(MIDDLE_PIN);
    ringServo.attach(RING_PIN);
    pinkyServo.attach(PINKY_PIN);

    // Initialize to open position
    executeGesture(GESTURE_HAND_OPEN);

    Serial.println("Ready. Send gesture commands (0-7) or 'p:0.0-1.0' for proportional control.");
}

void loop() {
    // Handle serial commands
    handleSerialInput();

    // Smooth position updates
    updateServoPositions();

    delay(POSITION_UPDATE_MS);
}

void handleSerialInput() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        // Proportional control: "p:0.75"
        if (input.startsWith("p:")) {
            float level = input.substring(2).toFloat();
            setProportionalLevel(level);
        }
        // Gesture command: single digit 0-7
        else if (input.length() == 1 && isDigit(input[0])) {
            int gesture = input.toInt();
            executeGesture((Gesture)gesture);
        }
        // EMG level (from BLE): "emg:0.65"
        else if (input.startsWith("emg:")) {
            emgLevel = input.substring(4).toFloat();
            if (proportionalEnabled) {
                applyProportionalControl();
            }
        }
    }
}

void executeGesture(Gesture gesture) {
    switch (gesture) {
        case GESTURE_REST:
        case GESTURE_HAND_OPEN:
            setAllFingers(0);
            break;

        case GESTURE_HAND_CLOSE:
            setAllFingers(100);
            break;

        case GESTURE_PINCH:
            // Thumb and index pinch
            setFinger(0, 70);  // Thumb
            setFinger(1, 70);  // Index
            setFinger(2, 0);   // Middle
            setFinger(3, 0);   // Ring
            setFinger(4, 0);   // Pinky
            break;

        case GESTURE_TRIPOD:
            // Thumb, index, middle grip
            setFinger(0, 60);
            setFinger(1, 60);
            setFinger(2, 60);
            setFinger(3, 0);
            setFinger(4, 0);
            break;

        case GESTURE_POINT:
            // Point with index finger
            setFinger(0, 100);
            setFinger(1, 0);
            setFinger(2, 100);
            setFinger(3, 100);
            setFinger(4, 100);
            break;

        case GESTURE_WRIST_FLEXION:
        case GESTURE_WRIST_EXTENSION:
            // No servo action for wrist gestures on basic 5-finger design
            break;
    }

    Serial.print("Gesture: ");
    Serial.println(gesture);
}

void setAllFingers(int position) {
    for (int i = 0; i < 5; i++) {
        targetPositions[i] = constrain(position, 0, 100);
    }
}

void setFinger(int fingerIndex, int position) {
    if (fingerIndex >= 0 && fingerIndex < 5) {
        targetPositions[fingerIndex] = constrain(position, 0, 100);
    }
}

void setProportionalLevel(float level) {
    level = constrain(level, 0.0, 1.0);
    int position = (int)(level * 100);
    setAllFingers(position);

    Serial.print("Proportional: ");
    Serial.println(level);
}

void applyProportionalControl() {
    // Map EMG level to grip strength
    int position = (int)(emgLevel * 100);
    setAllFingers(position);
}

void updateServoPositions() {
    // Smooth interpolation between current and target positions
    for (int i = 0; i < 5; i++) {
        if (currentPositions[i] != targetPositions[i]) {
            int diff = targetPositions[i] - currentPositions[i];
            int step = (int)(diff * SERVO_SPEED);

            if (abs(step) < 1) {
                step = (diff > 0) ? 1 : -1;
            }

            currentPositions[i] += step;
            currentPositions[i] = constrain(currentPositions[i], 0, 100);
        }
    }

    // Apply to servos
    thumbServo.write(positionToAngle(currentPositions[0], thumbCal));
    indexServo.write(positionToAngle(currentPositions[1], indexCal));
    middleServo.write(positionToAngle(currentPositions[2], middleCal));
    ringServo.write(positionToAngle(currentPositions[3], ringCal));
    pinkyServo.write(positionToAngle(currentPositions[4], pinkyCal));
}

int positionToAngle(int position, ServoCalibration cal) {
    return map(position, 0, 100, cal.openAngle, cal.closeAngle);
}

// Calibration routine
void calibrateServos() {
    Serial.println("Calibration mode. Commands:");
    Serial.println("  o - Set current as OPEN position");
    Serial.println("  c - Set current as CLOSE position");
    Serial.println("  +/- - Adjust angle");
    Serial.println("  1-5 - Select finger");
    Serial.println("  x - Exit calibration");

    int selectedFinger = 0;
    int testAngle = 90;

    while (true) {
        if (Serial.available() > 0) {
            char cmd = Serial.read();

            switch (cmd) {
                case 'o':
                    Serial.println("Open position set");
                    break;
                case 'c':
                    Serial.println("Close position set");
                    break;
                case '+':
                    testAngle = min(testAngle + 5, 180);
                    break;
                case '-':
                    testAngle = max(testAngle - 5, 0);
                    break;
                case '1': case '2': case '3': case '4': case '5':
                    selectedFinger = cmd - '1';
                    break;
                case 'x':
                    Serial.println("Exiting calibration");
                    return;
            }

            // Apply test angle to selected finger
            Servo* servos[] = {&thumbServo, &indexServo, &middleServo, &ringServo, &pinkyServo};
            servos[selectedFinger]->write(testAngle);

            Serial.print("Finger ");
            Serial.print(selectedFinger + 1);
            Serial.print(": ");
            Serial.println(testAngle);
        }
        delay(50);
    }
}
