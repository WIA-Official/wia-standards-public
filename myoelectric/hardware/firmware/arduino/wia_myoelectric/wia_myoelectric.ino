/**
 * WIA Myoelectric 2-Channel EMG Firmware
 * For Arduino Nano 33 BLE
 *
 * @version 1.0.0
 * @author WIA Standards Committee
 *
 * This firmware implements the WIA EMG hardware interface for
 * low-cost myoelectric prosthetic control systems using
 * Arduino Nano 33 BLE board.
 */

#include <ArduinoBLE.h>

// ============================================================================
// Configuration
// ============================================================================
#define VERSION "1.0.0"

// Pin definitions (Arduino Nano 33 BLE)
#define PIN_EMG_CH1     A0      // Analog input for EMG channel 1
#define PIN_EMG_CH2     A1      // Analog input for EMG channel 2
#define PIN_BATTERY     A2      // Battery voltage monitor
#define PIN_LED_BUILTIN LED_BUILTIN
#define PIN_LED_PWR     LED_PWR
#define PIN_BUTTON      2       // Digital input for button

// Sampling configuration
#define SAMPLE_RATE     1000    // Hz
#define SAMPLE_PERIOD   (1000000 / SAMPLE_RATE)  // Microseconds
#define BUFFER_SIZE     20      // Samples per BLE packet
#define ADC_RESOLUTION  12      // bits

// ============================================================================
// BLE Service and Characteristics
// ============================================================================
BLEService emgService("12345678-1234-5678-1234-56789abcdef0");

// EMG Data characteristic (notify)
BLECharacteristic emgDataChar("12345678-1234-5678-1234-56789abcdef1",
                               BLERead | BLENotify, sizeof(uint16_t) * 2 * BUFFER_SIZE + 8);

// Command characteristic (write)
BLECharacteristic commandChar("12345678-1234-5678-1234-56789abcdef2",
                               BLEWrite, 8);

// Status characteristic (read/notify)
BLECharacteristic statusChar("12345678-1234-5678-1234-56789abcdef3",
                              BLERead | BLENotify, 8);

// ============================================================================
// Data Structures
// ============================================================================
struct EMGPacket {
    uint32_t timestamp;
    uint16_t samples[2][BUFFER_SIZE];
    uint8_t sampleCount;
};

struct StatusPacket {
    uint8_t batteryLevel;
    uint8_t connectionQuality;
    uint8_t flags;
    uint16_t sampleRate;
};

// ============================================================================
// Global Variables
// ============================================================================
EMGPacket currentPacket;
StatusPacket statusPacket;

volatile bool streamingEnabled = false;
volatile uint8_t sampleIndex = 0;
volatile bool packetReady = false;

unsigned long lastSampleTime = 0;
unsigned long lastStatusTime = 0;
bool bleConnected = false;

// ============================================================================
// Command Codes
// ============================================================================
#define CMD_START_STREAM    0x01
#define CMD_STOP_STREAM     0x02
#define CMD_SET_SAMPLE_RATE 0x03
#define CMD_GET_BATTERY     0x04
#define CMD_GET_INFO        0x05

// ============================================================================
// Setup
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=================================");
    Serial.println("WIA Myoelectric EMG v" VERSION);
    Serial.println("Arduino Nano 33 BLE Version");
    Serial.println("=================================\n");

    // Configure pins
    pinMode(PIN_LED_BUILTIN, OUTPUT);
    pinMode(PIN_LED_PWR, OUTPUT);
    pinMode(PIN_BUTTON, INPUT_PULLUP);

    digitalWrite(PIN_LED_PWR, HIGH);
    digitalWrite(PIN_LED_BUILTIN, LOW);

    // Configure ADC
    analogReadResolution(ADC_RESOLUTION);

    // Initialize BLE
    if (!BLE.begin()) {
        Serial.println("BLE initialization failed!");
        while (1) {
            digitalWrite(PIN_LED_BUILTIN, !digitalRead(PIN_LED_BUILTIN));
            delay(100);
        }
    }

    // Set BLE device name and service
    BLE.setLocalName("WIA-EMG-2CH");
    BLE.setAdvertisedService(emgService);

    // Add characteristics to service
    emgService.addCharacteristic(emgDataChar);
    emgService.addCharacteristic(commandChar);
    emgService.addCharacteristic(statusChar);

    // Add service
    BLE.addService(emgService);

    // Set initial values
    uint8_t zeros[8] = {0};
    emgDataChar.writeValue(zeros, sizeof(zeros));
    statusChar.writeValue(zeros, sizeof(zeros));

    // Set event handlers
    BLE.setEventHandler(BLEConnected, onBLEConnected);
    BLE.setEventHandler(BLEDisconnected, onBLEDisconnected);
    commandChar.setEventHandler(BLEWritten, onCommandReceived);

    // Start advertising
    BLE.advertise();

    Serial.println("BLE advertising started");
    Serial.printf("Sample rate: %d Hz\n", SAMPLE_RATE);
    Serial.println("Ready!\n");
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
    // Poll BLE events
    BLE.poll();

    // Sample EMG at regular intervals
    unsigned long currentTime = micros();
    if (streamingEnabled && (currentTime - lastSampleTime >= SAMPLE_PERIOD)) {
        lastSampleTime = currentTime;
        sampleEMG();
    }

    // Send packet when ready
    if (packetReady && bleConnected) {
        sendEMGPacket();
        packetReady = false;
    }

    // Serial output for debugging (when not connected)
    if (packetReady && !bleConnected) {
        Serial.printf("CH1: %d, CH2: %d\n",
                      currentPacket.samples[0][0],
                      currentPacket.samples[1][0]);
        packetReady = false;
    }

    // Update status periodically
    if (millis() - lastStatusTime > 5000) {
        updateStatus();
        lastStatusTime = millis();
    }

    // Button handling
    static bool lastButtonState = HIGH;
    bool buttonState = digitalRead(PIN_BUTTON);
    if (buttonState == LOW && lastButtonState == HIGH) {
        streamingEnabled = !streamingEnabled;
        sampleIndex = 0;
        Serial.printf("Streaming: %s\n", streamingEnabled ? "ON" : "OFF");
    }
    lastButtonState = buttonState;

    // LED blinking when not connected
    if (!bleConnected) {
        static unsigned long lastBlink = 0;
        if (millis() - lastBlink > 500) {
            digitalWrite(PIN_LED_BUILTIN, !digitalRead(PIN_LED_BUILTIN));
            lastBlink = millis();
        }
    }
}

// ============================================================================
// EMG Sampling
// ============================================================================
void sampleEMG() {
    // Read both channels
    currentPacket.samples[0][sampleIndex] = analogRead(PIN_EMG_CH1);
    currentPacket.samples[1][sampleIndex] = analogRead(PIN_EMG_CH2);

    sampleIndex++;

    if (sampleIndex >= BUFFER_SIZE) {
        currentPacket.timestamp = millis();
        currentPacket.sampleCount = BUFFER_SIZE;
        packetReady = true;
        sampleIndex = 0;
    }
}

// ============================================================================
// BLE Data Transmission
// ============================================================================
void sendEMGPacket() {
    // Pack data for transmission
    uint8_t buffer[sizeof(uint32_t) + sizeof(uint16_t) * 2 * BUFFER_SIZE + 1];
    uint8_t* ptr = buffer;

    // Timestamp (4 bytes)
    memcpy(ptr, &currentPacket.timestamp, sizeof(uint32_t));
    ptr += sizeof(uint32_t);

    // Interleaved samples
    for (int i = 0; i < BUFFER_SIZE; i++) {
        memcpy(ptr, &currentPacket.samples[0][i], sizeof(uint16_t));
        ptr += sizeof(uint16_t);
        memcpy(ptr, &currentPacket.samples[1][i], sizeof(uint16_t));
        ptr += sizeof(uint16_t);
    }

    // Sample count
    *ptr = currentPacket.sampleCount;

    emgDataChar.writeValue(buffer, sizeof(buffer));
}

// ============================================================================
// Status Update
// ============================================================================
void updateStatus() {
    // Read battery level
    uint16_t rawBattery = analogRead(PIN_BATTERY);
    float voltage = (rawBattery / 4095.0f) * 3.3f * 2.0f; // Assuming 2:1 divider
    uint8_t percent = constrain((uint8_t)((voltage - 3.0f) / 1.2f * 100.0f), 0, 100);

    statusPacket.batteryLevel = percent;
    statusPacket.connectionQuality = 100; // TODO: implement signal quality
    statusPacket.flags = streamingEnabled ? 0x01 : 0x00;
    statusPacket.sampleRate = SAMPLE_RATE;

    if (bleConnected) {
        statusChar.writeValue((uint8_t*)&statusPacket, sizeof(StatusPacket));
    }

    Serial.printf("Battery: %d%%\n", percent);
}

// ============================================================================
// BLE Event Handlers
// ============================================================================
void onBLEConnected(BLEDevice central) {
    bleConnected = true;
    digitalWrite(PIN_LED_BUILTIN, HIGH);
    Serial.print("Connected to: ");
    Serial.println(central.address());
}

void onBLEDisconnected(BLEDevice central) {
    bleConnected = false;
    streamingEnabled = false;
    digitalWrite(PIN_LED_BUILTIN, LOW);
    Serial.print("Disconnected from: ");
    Serial.println(central.address());
    BLE.advertise();
}

void onCommandReceived(BLEDevice central, BLECharacteristic characteristic) {
    uint8_t buffer[8];
    int len = characteristic.readValue(buffer, sizeof(buffer));

    if (len > 0) {
        uint8_t cmd = buffer[0];
        handleCommand(cmd, buffer + 1, len - 1);
    }
}

// ============================================================================
// Command Handler
// ============================================================================
void handleCommand(uint8_t cmd, uint8_t* data, size_t len) {
    switch (cmd) {
        case CMD_START_STREAM:
            streamingEnabled = true;
            sampleIndex = 0;
            Serial.println("Streaming started");
            break;

        case CMD_STOP_STREAM:
            streamingEnabled = false;
            Serial.println("Streaming stopped");
            break;

        case CMD_SET_SAMPLE_RATE:
            if (len >= 2) {
                uint16_t rate = (data[0] << 8) | data[1];
                // Note: On Arduino, sample rate is fixed by SAMPLE_RATE constant
                Serial.printf("Sample rate request: %d Hz (fixed at %d)\n", rate, SAMPLE_RATE);
            }
            break;

        case CMD_GET_BATTERY:
            updateStatus();
            break;

        case CMD_GET_INFO:
            Serial.println("Device info requested");
            break;

        default:
            Serial.printf("Unknown command: 0x%02X\n", cmd);
    }
}
