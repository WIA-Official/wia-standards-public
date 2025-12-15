/**
 * WIA Basic Grip Example
 *
 * Simplest possible EMG-controlled prosthetic grip.
 * Demonstrates on/off control using EMG threshold detection.
 *
 * Hardware:
 * - Arduino Uno/Nano
 * - 1x Servo motor
 * - EMG signal input (analog)
 *
 * Usage:
 * - Connect EMG amplifier output to A0
 * - Connect servo to pin 9
 * - Adjust EMG_THRESHOLD based on your signal levels
 */

#include <Servo.h>

// Pin definitions
const int EMG_PIN = A0;
const int SERVO_PIN = 9;
const int LED_PIN = 13;

// Servo
Servo gripServo;

// Configuration
const int EMG_THRESHOLD = 400;    // Adjust based on your EMG levels (0-1023)
const int SERVO_OPEN = 0;         // Servo angle when hand is open
const int SERVO_CLOSE = 180;      // Servo angle when hand is closed
const int DEBOUNCE_MS = 100;      // Prevents rapid switching

// State
bool isGripping = false;
unsigned long lastChangeTime = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("WIA Basic Grip Example");

    // Initialize servo
    gripServo.attach(SERVO_PIN);
    gripServo.write(SERVO_OPEN);

    // LED indicator
    pinMode(LED_PIN, OUTPUT);

    Serial.println("Ready. Flex muscle to grip.");
    Serial.print("Threshold: ");
    Serial.println(EMG_THRESHOLD);
}

void loop() {
    // Read EMG signal
    int emgValue = analogRead(EMG_PIN);

    // Simple moving average (3 samples)
    static int samples[3] = {0, 0, 0};
    static int sampleIndex = 0;
    samples[sampleIndex] = emgValue;
    sampleIndex = (sampleIndex + 1) % 3;
    int avgValue = (samples[0] + samples[1] + samples[2]) / 3;

    // Threshold detection with debounce
    unsigned long now = millis();
    if (now - lastChangeTime >= DEBOUNCE_MS) {
        if (avgValue > EMG_THRESHOLD && !isGripping) {
            // Muscle activated - close grip
            isGripping = true;
            gripServo.write(SERVO_CLOSE);
            digitalWrite(LED_PIN, HIGH);
            lastChangeTime = now;
            Serial.println("GRIP");
        }
        else if (avgValue <= EMG_THRESHOLD && isGripping) {
            // Muscle relaxed - open grip
            isGripping = false;
            gripServo.write(SERVO_OPEN);
            digitalWrite(LED_PIN, LOW);
            lastChangeTime = now;
            Serial.println("RELEASE");
        }
    }

    // Print EMG value for calibration
    static unsigned long lastPrint = 0;
    if (now - lastPrint >= 100) {
        Serial.print("EMG: ");
        Serial.print(avgValue);
        Serial.print(" | ");
        Serial.println(isGripping ? "GRIPPING" : "OPEN");
        lastPrint = now;
    }

    delay(10);
}

/*
 * Calibration Instructions:
 *
 * 1. Upload this sketch and open Serial Monitor
 * 2. Relax your muscles and note the baseline EMG value
 * 3. Flex your muscles and note the activated EMG value
 * 4. Set EMG_THRESHOLD to a value between baseline and activated
 *    (usually about 30-50% of the difference)
 * 5. Test and adjust until grip is responsive but stable
 *
 * Example:
 *   Baseline: 200
 *   Activated: 600
 *   Threshold: 200 + (600-200)*0.4 = 360
 */
