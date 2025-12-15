/**
 * WIA Multi-Gesture Example
 *
 * Multiple gesture recognition using WIA EMG module.
 * Receives classified gestures via BLE/Serial and controls
 * a multi-finger prosthetic hand.
 *
 * Hardware:
 * - ESP32 or Arduino with HC-05/BLE module
 * - 5x Servo motors (one per finger)
 * - WIA EMG Module (provides gesture classification)
 *
 * Supported Gestures:
 * - Rest (0): All fingers open
 * - Hand Open (1): All fingers extended
 * - Hand Close (2): Power grip
 * - Wrist Flexion (3): (mapped to custom action)
 * - Wrist Extension (4): (mapped to custom action)
 * - Pinch (5): Thumb + Index
 * - Tripod (6): Thumb + Index + Middle
 * - Point (7): Index extended, others closed
 */

#include <Servo.h>

// Pin definitions
const int SERVO_PINS[] = {3, 5, 6, 9, 10};  // Thumb, Index, Middle, Ring, Pinky
const int LED_PIN = 13;

// Servos
Servo servos[5];
const char* FINGER_NAMES[] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};

// Gesture definitions
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

// Finger positions for each gesture [Thumb, Index, Middle, Ring, Pinky]
// 0 = open, 100 = closed
const int GESTURE_POSITIONS[][5] = {
    {0, 0, 0, 0, 0},       // REST
    {0, 0, 0, 0, 0},       // HAND_OPEN
    {100, 100, 100, 100, 100},  // HAND_CLOSE
    {50, 50, 50, 50, 50},  // WRIST_FLEXION (half grip)
    {25, 25, 25, 25, 25},  // WRIST_EXTENSION (light grip)
    {70, 70, 0, 0, 0},     // PINCH
    {60, 60, 60, 0, 0},    // TRIPOD
    {100, 0, 100, 100, 100} // POINT
};

const char* GESTURE_NAMES[] = {
    "Rest", "Hand Open", "Hand Close", "Wrist Flexion",
    "Wrist Extension", "Pinch", "Tripod", "Point"
};

// Current state
Gesture currentGesture = GESTURE_REST;
int targetPositions[5] = {0, 0, 0, 0, 0};
int currentPositions[5] = {0, 0, 0, 0, 0};

// Control parameters
const float MOVEMENT_SPEED = 0.15;  // 0.0-1.0
const int UPDATE_INTERVAL = 20;     // ms

// Statistics
unsigned long gestureCount = 0;
unsigned long lastGestureTime = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("WIA Multi-Gesture Example");
    Serial.println("Waiting for gesture commands (0-7)...\n");

    // Attach servos
    for (int i = 0; i < 5; i++) {
        servos[i].attach(SERVO_PINS[i]);
        servos[i].write(0);
    }

    pinMode(LED_PIN, OUTPUT);

    printHelp();
}

void loop() {
    handleInput();
    updateServos();
    delay(UPDATE_INTERVAL);
}

void handleInput() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        // Gesture command from WIA module: "g:2,0.95"
        if (input.startsWith("g:")) {
            int comma = input.indexOf(',');
            if (comma > 2) {
                int gesture = input.substring(2, comma).toInt();
                float confidence = input.substring(comma + 1).toFloat();
                processGesture((Gesture)gesture, confidence);
            }
        }
        // Simple gesture number
        else if (input.length() == 1 && isDigit(input[0])) {
            int gesture = input.toInt();
            if (gesture >= 0 && gesture <= 7) {
                processGesture((Gesture)gesture, 1.0);
            }
        }
        // Named commands
        else if (input.equalsIgnoreCase("open")) {
            processGesture(GESTURE_HAND_OPEN, 1.0);
        }
        else if (input.equalsIgnoreCase("close")) {
            processGesture(GESTURE_HAND_CLOSE, 1.0);
        }
        else if (input.equalsIgnoreCase("pinch")) {
            processGesture(GESTURE_PINCH, 1.0);
        }
        else if (input.equalsIgnoreCase("tripod")) {
            processGesture(GESTURE_TRIPOD, 1.0);
        }
        else if (input.equalsIgnoreCase("point")) {
            processGesture(GESTURE_POINT, 1.0);
        }
        // Individual finger: "f:1,50" (index finger to 50%)
        else if (input.startsWith("f:")) {
            int comma = input.indexOf(',');
            if (comma > 2) {
                int finger = input.substring(2, comma).toInt();
                int pos = input.substring(comma + 1).toInt();
                setFinger(finger, pos);
            }
        }
        // Status
        else if (input.equalsIgnoreCase("status")) {
            printStatus();
        }
        // Help
        else if (input.equalsIgnoreCase("help")) {
            printHelp();
        }
    }
}

void processGesture(Gesture gesture, float confidence) {
    // Confidence threshold
    if (confidence < 0.6) {
        return;
    }

    // Ignore duplicate gestures in quick succession
    if (gesture == currentGesture && millis() - lastGestureTime < 200) {
        return;
    }

    currentGesture = gesture;
    lastGestureTime = millis();
    gestureCount++;

    // Update target positions
    for (int i = 0; i < 5; i++) {
        targetPositions[i] = GESTURE_POSITIONS[gesture][i];
    }

    // LED feedback
    digitalWrite(LED_PIN, gesture != GESTURE_REST);

    Serial.print("Gesture: ");
    Serial.print(GESTURE_NAMES[gesture]);
    Serial.print(" (confidence: ");
    Serial.print(confidence * 100, 1);
    Serial.println("%)");
}

void setFinger(int finger, int position) {
    if (finger >= 0 && finger < 5) {
        targetPositions[finger] = constrain(position, 0, 100);
        Serial.print(FINGER_NAMES[finger]);
        Serial.print(": ");
        Serial.println(position);
    }
}

void updateServos() {
    for (int i = 0; i < 5; i++) {
        if (currentPositions[i] != targetPositions[i]) {
            int diff = targetPositions[i] - currentPositions[i];
            int step = (int)(diff * MOVEMENT_SPEED);

            if (abs(step) < 1) {
                step = (diff > 0) ? 1 : -1;
            }

            currentPositions[i] = constrain(currentPositions[i] + step, 0, 100);
        }

        // Map 0-100 to servo angle
        int angle = map(currentPositions[i], 0, 100, 0, 180);
        servos[i].write(angle);
    }
}

void printStatus() {
    Serial.println("\n=== STATUS ===");
    Serial.print("Current Gesture: ");
    Serial.println(GESTURE_NAMES[currentGesture]);
    Serial.print("Gestures Executed: ");
    Serial.println(gestureCount);
    Serial.println("\nFinger Positions:");
    for (int i = 0; i < 5; i++) {
        Serial.print("  ");
        Serial.print(FINGER_NAMES[i]);
        Serial.print(": ");
        Serial.print(currentPositions[i]);
        Serial.print("% -> ");
        Serial.print(targetPositions[i]);
        Serial.println("%");
    }
    Serial.println("==============\n");
}

void printHelp() {
    Serial.println("\n=== COMMANDS ===");
    Serial.println("0-7       : Execute gesture by number");
    Serial.println("open      : Open hand");
    Serial.println("close     : Close hand");
    Serial.println("pinch     : Pinch grip");
    Serial.println("tripod    : Tripod grip");
    Serial.println("point     : Point gesture");
    Serial.println("f:N,P     : Set finger N (0-4) to position P (0-100)");
    Serial.println("status    : Show current state");
    Serial.println("help      : Show this help");
    Serial.println("\nGesture protocol: g:GESTURE,CONFIDENCE");
    Serial.println("Example: g:2,0.95 (Hand Close, 95% confidence)");
    Serial.println("================\n");
}
