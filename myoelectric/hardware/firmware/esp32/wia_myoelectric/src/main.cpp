/**
 * @file main.cpp
 * @brief WIA Myoelectric 2-Channel EMG Firmware for ESP32-S3
 * @version 1.0.0
 *
 * This firmware implements the WIA EMG hardware interface for
 * low-cost myoelectric prosthetic control systems.
 *
 * Features:
 * - 2-channel EMG acquisition at 1000Hz
 * - BLE 5.0 data streaming
 * - USB serial output
 * - Battery monitoring
 * - Configurable gain and filters
 */

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "wia_emg.h"
#include "ble_service.h"
#include "adc_sampler.h"

// ============================================================================
// Pin Definitions
// ============================================================================
#define PIN_EMG_CH1         1   // ADC1_CH0 - EMG Channel 1
#define PIN_EMG_CH2         2   // ADC1_CH1 - EMG Channel 2
#define PIN_BATTERY         3   // ADC1_CH2 - Battery voltage monitor
#define PIN_LED_POWER       4   // Red LED - Power indicator
#define PIN_LED_BLE         5   // Blue LED - BLE status
#define PIN_BUTTON          6   // Mode button

// ============================================================================
// Configuration
// ============================================================================
#ifndef WIA_SAMPLE_RATE
#define WIA_SAMPLE_RATE     1000    // Hz
#endif

#ifndef WIA_CHANNEL_COUNT
#define WIA_CHANNEL_COUNT   2
#endif

#define SAMPLE_BUFFER_SIZE  20      // Samples per BLE packet
#define BATTERY_READ_INTERVAL 10000 // ms

// ============================================================================
// Global Objects
// ============================================================================
WiaEMG emg;
BLEService bleService;
ADCSampler adcSampler(PIN_EMG_CH1, PIN_EMG_CH2, WIA_SAMPLE_RATE);

// Sample buffer for BLE transmission
struct EMGPacket {
    uint32_t timestamp;
    uint16_t samples[WIA_CHANNEL_COUNT][SAMPLE_BUFFER_SIZE];
    uint8_t sampleCount;
} __attribute__((packed));

EMGPacket currentPacket;
volatile bool packetReady = false;
volatile uint8_t sampleIndex = 0;

// State variables
bool bleConnected = false;
bool streamingEnabled = false;
unsigned long lastBatteryRead = 0;
uint8_t batteryLevel = 100;

// ============================================================================
// Timer ISR for ADC Sampling
// ============================================================================
hw_timer_t *sampleTimer = NULL;

void IRAM_ATTR onSampleTimer() {
    if (!streamingEnabled) return;

    // Read both channels
    uint16_t ch1 = analogRead(PIN_EMG_CH1);
    uint16_t ch2 = analogRead(PIN_EMG_CH2);

    currentPacket.samples[0][sampleIndex] = ch1;
    currentPacket.samples[1][sampleIndex] = ch2;

    sampleIndex++;

    if (sampleIndex >= SAMPLE_BUFFER_SIZE) {
        currentPacket.timestamp = millis();
        currentPacket.sampleCount = SAMPLE_BUFFER_SIZE;
        packetReady = true;
        sampleIndex = 0;
    }
}

// ============================================================================
// BLE Callbacks
// ============================================================================
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) {
        bleConnected = true;
        digitalWrite(PIN_LED_BLE, HIGH);
        Serial.println("BLE Client connected");
    }

    void onDisconnect(NimBLEServer* pServer) {
        bleConnected = false;
        streamingEnabled = false;
        digitalWrite(PIN_LED_BLE, LOW);
        Serial.println("BLE Client disconnected");
        NimBLEDevice::startAdvertising();
    }
};

class CommandCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            uint8_t cmd = value[0];
            handleCommand(cmd, (uint8_t*)value.data() + 1, value.length() - 1);
        }
    }
};

void handleCommand(uint8_t cmd, uint8_t* data, size_t len) {
    switch (cmd) {
        case 0x01: // Start streaming
            streamingEnabled = true;
            sampleIndex = 0;
            Serial.println("Streaming started");
            break;

        case 0x02: // Stop streaming
            streamingEnabled = false;
            Serial.println("Streaming stopped");
            break;

        case 0x03: // Set sample rate
            if (len >= 2) {
                uint16_t rate = (data[0] << 8) | data[1];
                adcSampler.setSampleRate(rate);
                Serial.printf("Sample rate set to %d Hz\n", rate);
            }
            break;

        case 0x04: // Get battery level
            // Response sent via notification
            break;

        case 0x05: // Get device info
            Serial.println("Device info requested");
            break;

        default:
            Serial.printf("Unknown command: 0x%02X\n", cmd);
    }
}

// ============================================================================
// BLE Service Setup
// ============================================================================
NimBLEServer* pServer = nullptr;
NimBLECharacteristic* pDataCharacteristic = nullptr;
NimBLECharacteristic* pCommandCharacteristic = nullptr;
NimBLECharacteristic* pStatusCharacteristic = nullptr;

// WIA EMG Service UUID
#define SERVICE_UUID        "12345678-1234-5678-1234-56789abcdef0"
#define CHAR_DATA_UUID      "12345678-1234-5678-1234-56789abcdef1"
#define CHAR_COMMAND_UUID   "12345678-1234-5678-1234-56789abcdef2"
#define CHAR_STATUS_UUID    "12345678-1234-5678-1234-56789abcdef3"

void setupBLE() {
    NimBLEDevice::init("WIA-EMG-2CH");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService* pService = pServer->createService(SERVICE_UUID);

    // Data characteristic (notify)
    pDataCharacteristic = pService->createCharacteristic(
        CHAR_DATA_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );

    // Command characteristic (write)
    pCommandCharacteristic = pService->createCharacteristic(
        CHAR_COMMAND_UUID,
        NIMBLE_PROPERTY::WRITE
    );
    pCommandCharacteristic->setCallbacks(new CommandCallbacks());

    // Status characteristic (read/notify)
    pStatusCharacteristic = pService->createCharacteristic(
        CHAR_STATUS_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );

    pService->start();

    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->start();

    Serial.println("BLE advertising started");
}

// ============================================================================
// Battery Monitoring
// ============================================================================
uint8_t readBatteryLevel() {
    // Read battery voltage through voltage divider
    // Assuming 2:1 divider, so ADC reads half of battery voltage
    uint16_t raw = analogRead(PIN_BATTERY);

    // Convert to voltage (3.3V reference, 12-bit ADC)
    float voltage = (raw / 4095.0f) * 3.3f * 2.0f;

    // LiPo: 4.2V = 100%, 3.0V = 0%
    float percent = (voltage - 3.0f) / (4.2f - 3.0f) * 100.0f;
    return constrain((uint8_t)percent, 0, 100);
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=================================");
    Serial.println("WIA Myoelectric EMG v1.0.0");
    Serial.println("2-Channel EMG Acquisition System");
    Serial.println("=================================\n");

    // Configure pins
    pinMode(PIN_LED_POWER, OUTPUT);
    pinMode(PIN_LED_BLE, OUTPUT);
    pinMode(PIN_BUTTON, INPUT_PULLUP);

    digitalWrite(PIN_LED_POWER, HIGH);
    digitalWrite(PIN_LED_BLE, LOW);

    // Configure ADC
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db); // 0-3.3V range

    // Setup BLE
    setupBLE();

    // Setup sample timer (1000Hz = 1ms period)
    sampleTimer = timerBegin(0, 80, true); // 80MHz / 80 = 1MHz
    timerAttachInterrupt(sampleTimer, &onSampleTimer, true);
    timerAlarmWrite(sampleTimer, 1000, true); // 1000us = 1ms
    timerAlarmEnable(sampleTimer);

    Serial.printf("Sample rate: %d Hz\n", WIA_SAMPLE_RATE);
    Serial.printf("Channels: %d\n", WIA_CHANNEL_COUNT);
    Serial.println("Ready!\n");
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
    // Send EMG data when packet is ready
    if (packetReady && bleConnected) {
        pDataCharacteristic->setValue((uint8_t*)&currentPacket, sizeof(EMGPacket));
        pDataCharacteristic->notify();
        packetReady = false;
    }

    // Also output to serial for debugging
    if (packetReady && !bleConnected) {
        Serial.printf("CH1: %d, CH2: %d\n",
            currentPacket.samples[0][0],
            currentPacket.samples[1][0]);
        packetReady = false;
    }

    // Battery monitoring
    if (millis() - lastBatteryRead > BATTERY_READ_INTERVAL) {
        batteryLevel = readBatteryLevel();
        lastBatteryRead = millis();

        if (bleConnected) {
            pStatusCharacteristic->setValue(&batteryLevel, 1);
            pStatusCharacteristic->notify();
        }

        Serial.printf("Battery: %d%%\n", batteryLevel);
    }

    // Button handling
    static bool lastButtonState = HIGH;
    bool buttonState = digitalRead(PIN_BUTTON);

    if (buttonState == LOW && lastButtonState == HIGH) {
        // Button pressed - toggle streaming
        streamingEnabled = !streamingEnabled;
        Serial.printf("Streaming: %s\n", streamingEnabled ? "ON" : "OFF");
    }
    lastButtonState = buttonState;

    // LED blinking when not connected
    if (!bleConnected) {
        static unsigned long lastBlink = 0;
        if (millis() - lastBlink > 500) {
            digitalWrite(PIN_LED_BLE, !digitalRead(PIN_LED_BLE));
            lastBlink = millis();
        }
    }

    delay(1);
}
