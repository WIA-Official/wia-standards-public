# WIA-MED-021: Sleep Monitoring Standards
## Version 1.0.0

### Overview

WIA-MED-021 defines international standards for sleep monitoring systems, ensuring accuracy, safety, and interoperability.

### Scope

This standard applies to:
- Polysomnography (PSG) systems
- Actigraphy devices
- Consumer wearables with sleep tracking
- Smart mattresses and non-contact sensors
- Home sleep test (HST) devices
- Sleep apnea detection systems
- Circadian rhythm monitoring tools

### Key Requirements

#### 1. Sleep-Wake Classification
- **Accuracy**: ≥85% (vs. PSG gold standard)
- **Total Sleep Time**: Mean error ≤30 minutes
- **Sleep Efficiency**: Correlation coefficient ≥0.70

#### 2. Sleep Stage Classification (Optional)
- **4-Stage Accuracy**: ≥70% (Wake, Light, Deep, REM)
- **Cohen's Kappa**: ≥0.60
- **AASM Compliance**: Follow AASM manual scoring rules

#### 3. PSG Validation
- **Minimum Sample Size**: 100 participants
- **Diverse Population**: Age, sex, ethnicity, BMI
- **Sleep Disorder Inclusion**: Insomnia, sleep apnea patients
- **Gold Standard**: Concurrent PSG measurement

#### 4. Sensor Specifications

**Accelerometer (Actigraphy)**:
- Sampling rate: ≥32 Hz
- Resolution: 3-axis
- Range: ±8g minimum

**PPG (Photoplethysmography)**:
- Sampling rate: ≥25 Hz (SpO2), ≥100 Hz (heart rate)
- Wavelengths: Green (heart rate), Red+IR (SpO2)
- Accuracy: Heart rate ±5 bpm, SpO2 ±3%

**EEG (Clinical PSG)**:
- Channels: ≥3 (F4-M1, C4-M1, O2-M1)
- Sampling rate: ≥200 Hz (recommended ≥500 Hz)
- Resolution: ≥12 bit
- Impedance: <5 kΩ

#### 5. Data Formats
- **Primary**: EDF+ (European Data Format Plus)
- **Export**: CSV, JSON for app integrations
- **Epoch Length**: 30 seconds (AASM standard)

#### 6. Privacy & Security
- **GDPR Compliance**: Data protection, user consent
- **HIPAA Compliance**: For clinical devices
- **Encryption**: AES-256 for stored data
- **Anonymization**: PII removal for research data

#### 7. Certification Levels

**Class I (Clinical)**:
- PSG accuracy ≥90%
- FDA/MFDS approval required
- Clinical diagnosis support

**Class II (Wellness)**:
- PSG accuracy ≥85%
- Sleep-wake classification
- Personal tracking

**Class III (Research)**:
- PSG accuracy ≥80%
- Research purposes only

### References
- AASM Manual for the Scoring of Sleep and Associated Events (v2.6, 2020)
- ISO 80601-2-61 (Pulse oximeters)
- IEC 60601-1 (Medical electrical equipment safety)
- FDA Guidance for Clinical Decision Support Software (2022)

### Contact
- Website: https://wiastandards.com
- Email: certification@wiastandards.com
- GitHub: https://github.com/WIA-Official/wia-standards

---

© 2025 World Certification Industry Association (WIA)
弘益人間 · Benefit All Humanity
