/**
 * @file adc_sampler.h
 * @brief ADC Sampling Module for WIA EMG
 * @version 1.0.0
 */

#ifndef ADC_SAMPLER_H
#define ADC_SAMPLER_H

#include <Arduino.h>

class ADCSampler {
public:
    ADCSampler(uint8_t ch1Pin, uint8_t ch2Pin, uint16_t sampleRate) :
        _ch1Pin(ch1Pin),
        _ch2Pin(ch2Pin),
        _sampleRate(sampleRate),
        _enabled(false)
    {}

    void begin() {
        analogReadResolution(12);
        analogSetAttenuation(ADC_11db);
    }

    void setSampleRate(uint16_t rate) {
        _sampleRate = constrain(rate, 100, 2000);
    }

    uint16_t getSampleRate() const { return _sampleRate; }

    void enable() { _enabled = true; }
    void disable() { _enabled = false; }
    bool isEnabled() const { return _enabled; }

    uint16_t readChannel1() {
        return analogRead(_ch1Pin);
    }

    uint16_t readChannel2() {
        return analogRead(_ch2Pin);
    }

    void readBoth(uint16_t& ch1, uint16_t& ch2) {
        ch1 = analogRead(_ch1Pin);
        ch2 = analogRead(_ch2Pin);
    }

private:
    uint8_t _ch1Pin;
    uint8_t _ch2Pin;
    uint16_t _sampleRate;
    bool _enabled;
};

#endif // ADC_SAMPLER_H
