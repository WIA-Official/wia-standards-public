# WIA-MED-003 Phase 1: Signal Format Standard

**Version:** 1.0.0
**Status:** Complete
**Last Updated:** 2025-01-15

## Overview

Phase 1 defines the standardized data format for all vital sign measurements in the WIA-MED-003 ecosystem.

## Base Signal Structure

All vital sign data must include these common fields:

```json
{
  "standard": "WIA-MED-003",
  "version": "1.0.0",
  "signalType": "ECG|SPO2|BLOOD_PRESSURE|HRV|TEMPERATURE|RESPIRATORY_RATE",
  "deviceId": "string",
  "timestamp": "ISO 8601 string",
  "data": {}
}
```

## Signal Types

### ECG Signal
- Sampling rate: 250-1000 Hz
- Unit: mV (millivolts)
- Leads: 1, 3, 5, or 12-lead configurations

### SpO2 Signal
- Range: 0-100%
- Pulse rate: 30-250 bpm
- PPG waveform: Red and infrared channels

### Blood Pressure
- Unit: mmHg
- Components: Systolic, Diastolic, Mean Arterial Pressure
- Methods: Auscultatory, Oscillometric, Continuous

### HRV (Heart Rate Variability)
- Time domain: SDNN, RMSSD, pNN50
- Frequency domain: VLF, LF, HF, LF/HF ratio
- Non-linear: Poincaré plot metrics

### Temperature
- Unit: Celsius or Fahrenheit
- Sites: Oral, rectal, axillary, tympanic, forehead
- Range: 30-45°C

### Respiratory Rate
- Unit: breaths/minute
- Range: 8-30 breaths/min
- Pattern: Regular, irregular, labored

## Data Compression

Supported compression methods:
1. Delta encoding
2. Run-length encoding
3. Gzip
4. Downsampling with anti-aliasing

---

**Copyright 2025 WIA / SmileStory Inc.**
**License:** MIT
**弘益人間 · Benefit All Humanity**
