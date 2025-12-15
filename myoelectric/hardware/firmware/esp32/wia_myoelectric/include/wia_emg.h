/**
 * @file wia_emg.h
 * @brief WIA EMG Core Definitions
 * @version 1.0.0
 */

#ifndef WIA_EMG_H
#define WIA_EMG_H

#include <Arduino.h>

// ============================================================================
// Version Information
// ============================================================================
#define WIA_EMG_VERSION_MAJOR   1
#define WIA_EMG_VERSION_MINOR   0
#define WIA_EMG_VERSION_PATCH   0

#ifndef WIA_EMG_VERSION
#define WIA_EMG_VERSION         "1.0.0"
#endif

// ============================================================================
// Hardware Configuration
// ============================================================================
#define WIA_ADC_RESOLUTION      12      // bits
#define WIA_ADC_MAX_VALUE       4095    // 2^12 - 1
#define WIA_VREF                3.3f    // Volts
#define WIA_ELECTRODE_OFFSET    1.65f   // Volts (mid-rail)

// Default settings
#define WIA_DEFAULT_SAMPLE_RATE 1000    // Hz
#define WIA_DEFAULT_GAIN        500     // V/V
#define WIA_MIN_SAMPLE_RATE     100     // Hz
#define WIA_MAX_SAMPLE_RATE     2000    // Hz

// ============================================================================
// Data Structures
// ============================================================================

/**
 * @brief EMG sample data for a single time point
 */
struct EMGSample {
    uint32_t timestamp_us;      // Microseconds since boot
    uint16_t channel[2];        // Raw ADC values (0-4095)
};

/**
 * @brief Device status information
 */
struct DeviceStatus {
    uint8_t batteryLevel;       // 0-100%
    uint8_t connectionQuality;  // 0-100%
    bool isStreaming;
    bool isCalibrated;
    uint16_t sampleRate;
    uint16_t gain;
};

/**
 * @brief Calibration data
 */
struct CalibrationData {
    uint16_t ch1_offset;        // ADC offset for channel 1
    uint16_t ch2_offset;        // ADC offset for channel 2
    float ch1_scale;            // Scale factor for channel 1
    float ch2_scale;            // Scale factor for channel 2
    uint32_t timestamp;         // Calibration timestamp
    bool valid;
};

// ============================================================================
// BLE Protocol
// ============================================================================

// Command codes
enum WIACommand : uint8_t {
    CMD_START_STREAM    = 0x01,
    CMD_STOP_STREAM     = 0x02,
    CMD_SET_SAMPLE_RATE = 0x03,
    CMD_GET_BATTERY     = 0x04,
    CMD_GET_INFO        = 0x05,
    CMD_CALIBRATE       = 0x06,
    CMD_SET_GAIN        = 0x07,
    CMD_RESET           = 0xFF
};

// Status codes
enum WIAStatus : uint8_t {
    STATUS_OK           = 0x00,
    STATUS_ERROR        = 0x01,
    STATUS_BUSY         = 0x02,
    STATUS_NOT_READY    = 0x03,
    STATUS_INVALID_CMD  = 0x04
};

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Convert raw ADC value to voltage
 * @param raw Raw ADC value (0-4095)
 * @return Voltage in volts
 */
inline float adcToVoltage(uint16_t raw) {
    return (raw / (float)WIA_ADC_MAX_VALUE) * WIA_VREF;
}

/**
 * @brief Convert raw ADC value to millivolts at electrode
 * @param raw Raw ADC value (0-4095)
 * @param gain Amplifier gain
 * @return Voltage in millivolts
 */
inline float adcToElectrodeMillivolts(uint16_t raw, uint16_t gain) {
    float voltage = adcToVoltage(raw) - WIA_ELECTRODE_OFFSET;
    return (voltage / gain) * 1000.0f;
}

// ============================================================================
// WiaEMG Class
// ============================================================================

class WiaEMG {
public:
    WiaEMG() :
        _sampleRate(WIA_DEFAULT_SAMPLE_RATE),
        _gain(WIA_DEFAULT_GAIN),
        _streaming(false),
        _calibrated(false)
    {
        _calibration.valid = false;
    }

    void begin() {
        // Initialize
    }

    void setSampleRate(uint16_t rate) {
        _sampleRate = constrain(rate, WIA_MIN_SAMPLE_RATE, WIA_MAX_SAMPLE_RATE);
    }

    uint16_t getSampleRate() const { return _sampleRate; }

    void setGain(uint16_t gain) {
        _gain = gain;
    }

    uint16_t getGain() const { return _gain; }

    void startStreaming() { _streaming = true; }
    void stopStreaming() { _streaming = false; }
    bool isStreaming() const { return _streaming; }

    void calibrate(uint16_t ch1_offset, uint16_t ch2_offset) {
        _calibration.ch1_offset = ch1_offset;
        _calibration.ch2_offset = ch2_offset;
        _calibration.ch1_scale = 1.0f;
        _calibration.ch2_scale = 1.0f;
        _calibration.timestamp = millis();
        _calibration.valid = true;
        _calibrated = true;
    }

    bool isCalibrated() const { return _calibrated; }

    DeviceStatus getStatus() const {
        DeviceStatus status;
        status.sampleRate = _sampleRate;
        status.gain = _gain;
        status.isStreaming = _streaming;
        status.isCalibrated = _calibrated;
        return status;
    }

private:
    uint16_t _sampleRate;
    uint16_t _gain;
    bool _streaming;
    bool _calibrated;
    CalibrationData _calibration;
};

#endif // WIA_EMG_H
