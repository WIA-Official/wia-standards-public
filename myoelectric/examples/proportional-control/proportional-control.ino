/**
 * WIA Proportional Control Example
 *
 * EMG-proportional grip control - grip strength follows muscle intensity.
 * Provides natural, intuitive control for prosthetic hands.
 *
 * Hardware:
 * - Arduino Uno/Nano/ESP32
 * - 1x Servo motor (or multiple for multi-finger)
 * - EMG signal input (analog)
 *
 * Features:
 * - Smooth proportional grip response
 * - Adjustable sensitivity and dead zone
 * - Visual feedback via LED brightness
 */

#include <Servo.h>

// Pin definitions
const int EMG_PIN = A0;
const int SERVO_PIN = 9;
const int LED_PIN = 11;  // PWM pin for brightness

// Servo
Servo gripServo;

// Calibration values (adjust during setup)
int emgMin = 100;      // EMG value at rest
int emgMax = 700;      // EMG value at max contraction

// Control parameters
const float SMOOTHING = 0.1;      // Lower = smoother (0.01-0.3)
const int DEAD_ZONE = 30;         // Ignore small EMG changes
const int SERVO_MIN = 0;          // Open position
const int SERVO_MAX = 180;        // Closed position

// State
float smoothedEMG = 0;
int currentServoPos = SERVO_MIN;

// Calibration mode
bool calibrating = false;
unsigned long calibStartTime = 0;
const unsigned long CALIB_DURATION = 5000;

void setup() {
    Serial.begin(115200);
    Serial.println("WIA Proportional Control Example");
    Serial.println("Commands: 'c' = calibrate, 's' = show settings");

    gripServo.attach(SERVO_PIN);
    gripServo.write(SERVO_MIN);

    pinMode(LED_PIN, OUTPUT);

    Serial.println("Ready. Vary muscle intensity to control grip.");
}

void loop() {
    handleSerial();

    if (calibrating) {
        runCalibration();
    } else {
        runProportionalControl();
    }

    delay(10);
}

void handleSerial() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();

        switch (cmd) {
            case 'c':
            case 'C':
                startCalibration();
                break;

            case 's':
            case 'S':
                showSettings();
                break;

            case '+':
                emgMax += 50;
                Serial.print("emgMax: ");
                Serial.println(emgMax);
                break;

            case '-':
                emgMax -= 50;
                Serial.print("emgMax: ");
                Serial.println(emgMax);
                break;
        }
    }
}

void startCalibration() {
    Serial.println("\n=== CALIBRATION ===");
    Serial.println("Step 1: RELAX your muscles completely");
    Serial.println("Recording baseline in 3 seconds...");

    delay(3000);

    // Record rest values
    long sum = 0;
    for (int i = 0; i < 100; i++) {
        sum += analogRead(EMG_PIN);
        delay(10);
    }
    emgMin = sum / 100;

    Serial.print("Baseline recorded: ");
    Serial.println(emgMin);

    Serial.println("\nStep 2: CONTRACT your muscles MAXIMUM");
    Serial.println("Recording max in 3 seconds...");

    delay(3000);

    // Record max values
    int maxVal = 0;
    for (int i = 0; i < 100; i++) {
        int val = analogRead(EMG_PIN);
        if (val > maxVal) maxVal = val;
        delay(10);
    }
    emgMax = maxVal;

    Serial.print("Maximum recorded: ");
    Serial.println(emgMax);

    Serial.println("\n=== CALIBRATION COMPLETE ===");
    showSettings();

    calibrating = false;
}

void runCalibration() {
    // Dynamic calibration updates min/max during use
    int emg = analogRead(EMG_PIN);

    if (emg < emgMin) {
        emgMin = emg;
    }
    if (emg > emgMax) {
        emgMax = emg;
    }
}

void runProportionalControl() {
    // Read EMG
    int rawEMG = analogRead(EMG_PIN);

    // Apply exponential smoothing
    smoothedEMG = SMOOTHING * rawEMG + (1 - SMOOTHING) * smoothedEMG;

    // Map to 0-100% grip
    int emgRange = emgMax - emgMin;
    if (emgRange < 50) emgRange = 50;  // Prevent division issues

    int gripPercent = constrain(
        map((int)smoothedEMG, emgMin + DEAD_ZONE, emgMax, 0, 100),
        0, 100
    );

    // Apply dead zone
    if ((int)smoothedEMG < emgMin + DEAD_ZONE) {
        gripPercent = 0;
    }

    // Smooth servo movement
    int targetPos = map(gripPercent, 0, 100, SERVO_MIN, SERVO_MAX);
    int posDiff = targetPos - currentServoPos;

    if (abs(posDiff) > 1) {
        currentServoPos += posDiff * 0.2;  // Gradual movement
    }

    gripServo.write(currentServoPos);

    // LED shows grip strength
    analogWrite(LED_PIN, map(gripPercent, 0, 100, 0, 255));

    // Debug output (throttled)
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {
        Serial.print("EMG: ");
        Serial.print((int)smoothedEMG);
        Serial.print(" | Grip: ");
        Serial.print(gripPercent);
        Serial.print("% | Servo: ");
        Serial.println(currentServoPos);
        lastPrint = millis();
    }
}

void showSettings() {
    Serial.println("\n=== CURRENT SETTINGS ===");
    Serial.print("EMG Min (rest): ");
    Serial.println(emgMin);
    Serial.print("EMG Max (contract): ");
    Serial.println(emgMax);
    Serial.print("Dead Zone: ");
    Serial.println(DEAD_ZONE);
    Serial.print("Smoothing: ");
    Serial.println(SMOOTHING);
    Serial.println("========================\n");
}

/*
 * Tips for Best Proportional Control:
 *
 * 1. Calibrate in a quiet environment
 * 2. Use consistent electrode placement
 * 3. Adjust SMOOTHING:
 *    - Higher (0.2-0.3): More responsive, may be jittery
 *    - Lower (0.05-0.1): Smoother, may feel laggy
 * 4. Increase DEAD_ZONE if grip activates at rest
 * 5. Practice "graduated" muscle contractions
 */
