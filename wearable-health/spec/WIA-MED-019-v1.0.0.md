# WIA MED-019: Wearable Health Device Standard v1.0.0

**Document Status:** Published
**Publication Date:** 2025-01-15
**Revision:** 1.0.0
**License:** MIT

---

## Abstract

This document specifies requirements for wearable health devices to ensure accuracy, interoperability, security, and user safety. It covers smartwatches, fitness trackers, continuous glucose monitors, ECG wearables, sleep trackers, and other body-worn health monitoring devices.

**Keywords:** wearable, health, fitness, smartwatch, CGM, ECG, PPG, sleep tracking, medical device

---

## 1. Introduction

### 1.1 Purpose

The WIA MED-019 standard aims to:

1. **Ensure Data Accuracy:** Define minimum accuracy requirements for health sensors
2. **Enable Interoperability:** Facilitate data exchange between devices and platforms
3. **Protect Privacy:** Establish security and privacy best practices
4. **Support Regulation:** Align with FDA, CE, and other medical device regulations
5. **Promote Innovation:** Provide clear guidelines for manufacturers

### 1.2 Scope

**In Scope:**
- Wearable devices with health/fitness sensors
- Consumer and medical-grade wearables
- Software applications for health data
- Data exchange protocols

**Out of Scope:**
- Implantable medical devices (covered by WIA MED-XXX)
- Stationary medical equipment
- Telemedicine platforms (covered by WIA MED-006)

### 1.3 Normative References

- ISO 13485:2016 - Medical Devices Quality Management Systems
- IEC 62304:2006 - Medical Device Software Lifecycle Processes
- ISO 14971:2019 - Application of Risk Management to Medical Devices
- HL7 FHIR R4 - Fast Healthcare Interoperability Resources
- IEEE 11073 - Health Informatics - Device Interoperability

---

## 2. Device Categories

### 2.1 Smartwatches

**Definition:** Wrist-worn devices with computing capabilities, health sensors, and smartphone connectivity.

**Minimum Requirements:**
- PPG heart rate sensor
- Accelerometer (3-axis)
- Bluetooth connectivity
- Rechargeable battery (≥18 hours)

**Optional Features:**
- ECG sensor (requires FDA/CE clearance)
- SpO2 sensor
- GPS
- Cellular connectivity
- Blood pressure (requires validation)

**Examples:** Apple Watch, Samsung Galaxy Watch, Garmin Fenix, Fitbit Sense

### 2.2 Fitness Trackers

**Definition:** Simplified wearables focused on activity tracking and basic health metrics.

**Minimum Requirements:**
- Accelerometer
- Step counting accuracy ≥95%
- Battery life ≥5 days

**Optional:**
- PPG heart rate
- SpO2
- Sleep tracking

**Examples:** Fitbit Charge, Mi Band, Whoop, Oura Ring

### 2.3 Continuous Glucose Monitors (CGM)

**Definition:** Subcutaneous sensors for continuous glucose measurement.

**Requirements:**
- MARD ≤10%
- Sensor life: 7-14 days
- Real-time alerts
- FDA/CE medical device clearance REQUIRED

**Examples:** Dexcom G7, Freestyle Libre 3, Medtronic Guardian 4

### 2.4 ECG Wearables

**Definition:** Devices capable of recording electrocardiogram signals.

**Requirements:**
- Single-lead ECG (minimum)
- Sampling rate ≥250 Hz
- AFib detection sensitivity ≥95%
- FDA/CE medical device clearance REQUIRED

**Examples:** Apple Watch, AliveCor KardiaMobile, Withings ScanWatch

### 2.5 Sleep Trackers

**Definition:** Devices or sensors for monitoring sleep patterns.

**Requirements:**
- Sleep/wake detection ≥85% accuracy
- Sleep stage classification (optional)
- Respiratory rate monitoring (optional)

**Examples:** Oura Ring, Whoop 4.0, Withings Sleep Analyzer

---

## 3. Sensor Accuracy Requirements

### 3.1 Heart Rate (PPG)

| Condition | Grade A | Grade B | Grade C |
|-----------|---------|---------|---------|
| **Resting** | ±1-2 bpm | ±3-5 bpm | ±5-10 bpm |
| **Light Activity** | ±3-5 bpm | ±5-8 bpm | ±10-15 bpm |
| **Vigorous Exercise** | ±5-8 bpm | ±8-12 bpm | ±15-20 bpm |

**Test Method:** Compare against chest strap ECG (reference standard)

### 3.2 ECG

| Parameter | Requirement |
|-----------|-------------|
| **Sampling Rate** | ≥250 Hz |
| **Resolution** | ≥10 bits |
| **AFib Sensitivity** | ≥95% |
| **AFib Specificity** | ≥98% |
| **PPV (Positive Predictive Value)** | ≥80% |

**Test Method:** Clinical validation against 12-lead ECG

### 3.3 SpO2 (Oxygen Saturation)

| Range | Accuracy |
|-------|----------|
| **95-100%** | ±2% |
| **90-94%** | ±3% |
| **70-89%** | ±4% |

**Test Method:** Pulse oximeter reference standard

### 3.4 Activity Metrics

| Metric | Accuracy | Test Method |
|--------|----------|-------------|
| **Steps** | ±3-5% | 400m track, manual count |
| **Distance (GPS)** | ±5% | Known route |
| **Floors Climbed** | ±1 floor | Controlled staircase |
| **Calories** | ±15% | Metabolic chamber |

### 3.5 Sleep Detection

| Metric | Minimum Accuracy |
|--------|------------------|
| **Sleep vs Wake** | 85% |
| **Sleep Stages (3-stage)** | 75% |
| **Sleep Stages (4-stage)** | 70% |
| **Total Sleep Time** | ±30 minutes |

**Reference:** Polysomnography (PSG)

### 3.6 Continuous Glucose Monitoring

| Parameter | Requirement |
|-----------|-------------|
| **MARD (Mean Absolute Relative Difference)** | ≤10% |
| **Clarke Error Grid Zone A+B** | ≥99% |
| **Sensor Life** | 7-14 days |
| **Warmup Time** | ≤60 minutes |
| **Lag Time** | ≤15 minutes |

---

## 4. Data Interoperability

### 4.1 Platform Support

**Mandatory Integrations:**
- Apple HealthKit (iOS devices)
- Google Fit (Android devices)
- FHIR R4 export capability

**Recommended:**
- Samsung Health
- Garmin Connect
- Fitbit API
- Strava API

### 4.2 Data Formats

**Health Data:**
- FHIR Observation resources (JSON/XML)
- ISO/IEEE 11073 Personal Health Device (PHD) standards

**Activity Data:**
- GPX (GPS Exchange Format)
- TCX (Training Center XML)
- FIT (Flexible and Interoperable Data Transfer)

**Sleep Data:**
- Hypnogram format
- Sleep stage timestamps

### 4.3 API Requirements

**RESTful API:**
- OAuth 2.0 authentication
- HTTPS only (TLS 1.3+)
- Rate limiting
- Pagination support
- Webhook notifications

**Data Access:**
- Read: User authorization required
- Write: Device authentication + user authorization
- Delete: User-initiated only

---

## 5. Security & Privacy

### 5.1 Data Encryption

**At Rest:**
- AES-256 encryption
- Secure key storage (hardware-backed preferred)
- Encrypted backups

**In Transit:**
- TLS 1.3 or higher
- Certificate pinning (recommended)
- Perfect Forward Secrecy

### 5.2 Authentication

**Device:**
- Unique device identifier
- Secure pairing (Bluetooth with PIN/passkey)
- Certificate-based authentication

**User:**
- Multi-factor authentication (optional)
- Biometric authentication (recommended)
- Session timeout

### 5.3 Privacy

**GDPR Compliance:**
- ✅ Data minimization
- ✅ Purpose limitation
- ✅ Right to access
- ✅ Right to erasure
- ✅ Data portability

**HIPAA Compliance (if applicable):**
- ✅ Business Associate Agreement
- ✅ Audit logs
- ✅ Access controls
- ✅ Breach notification

### 5.4 Data Ownership

- **User owns all health data**
- User can export data in standard formats
- User can delete all data
- Third-party access requires explicit consent

---

## 6. Medical Device Compliance

### 6.1 Regulatory Classification

**Wellness Devices (No FDA clearance required):**
- Step counters
- General heart rate monitoring
- General sleep tracking
- Activity tracking

**Medical Devices (FDA/CE clearance required):**
- ECG with AFib detection
- CGM for diabetes management
- Sleep apnea detection
- Blood pressure measurement
- Clinical decision support

### 6.2 FDA Clearance Pathways

| Pathway | Risk Level | Timeline | Cost |
|---------|------------|----------|------|
| **510(k)** | Class II | 3-12 months | $150K-300K |
| **De Novo** | Class II (novel) | 6-18 months | $500K-2M |
| **PMA** | Class III | 1-3 years | $1M-5M |

### 6.3 Quality Management

**ISO 13485 Requirements:**
- Document control
- Design controls
- Risk management
- Corrective/Preventive Action (CAPA)
- Post-market surveillance

### 6.4 Software Validation

**IEC 62304 Safety Classes:**
- Class A: No injury or health damage
- Class B: Non-serious injury
- Class C: Death or serious injury

**Requirements:**
- Software development plan
- Requirements specification
- Architecture design
- Unit/integration/system testing
- Traceability matrix

### 6.5 Cybersecurity

**FDA Guidance:**
- Threat modeling
- Vulnerability assessment
- Secure boot
- Software Bill of Materials (SBOM)
- Update mechanism security

---

## 7. User Experience

### 7.1 Wearability

**Comfort:**
- Hypoallergenic materials
- Adjustable fit
- Weight <100g (wrist wearables)

**Durability:**
- Water resistance: IP67 minimum
- Drop test: 1.2m
- Temperature range: -10°C to 50°C

### 7.2 Battery Life

| Device Type | Minimum | Target |
|-------------|---------|--------|
| **Smartwatch** | 18 hours | 2-7 days |
| **Fitness Tracker** | 5 days | 7-14 days |
| **CGM** | Sensor life | N/A |
| **ECG Device** | 100 readings | N/A |

### 7.3 User Interface

**Display:**
- Readability in sunlight
- Nighttime mode
- Accessibility (large text, voice)

**Notifications:**
- Customizable alerts
- Do Not Disturb mode
- Emergency override

### 7.4 Setup & Pairing

- First-time setup <5 minutes
- Auto-reconnection
- Multi-device support

---

## 8. Environmental Impact

### 8.1 Materials

**Preferred:**
- Recycled plastics
- Conflict-free minerals
- Bio-based materials

**Restricted:**
- RoHS compliance (lead, mercury, cadmium, etc.)
- REACH compliance (EU)

### 8.2 Packaging

- Minimal plastic packaging
- Recycled/recyclable materials
- Clear recycling instructions

### 8.3 End of Life

- Take-back/recycling program
- Battery recycling
- Data sanitization before disposal

---

## 9. Testing & Certification

### 9.1 Conformance Testing

**Sensor Accuracy:**
- Clinical study with ≥50 participants
- Range of ages, skin tones, activity levels
- Statistical validation (mean, SD, MARD)

**Interoperability:**
- HealthKit integration test
- Google Fit integration test
- FHIR export validation

**Security:**
- Penetration testing
- Vulnerability scanning
- Code review

### 9.2 WIA Certification Process

1. **Application:** Submit device details
2. **Documentation Review:** Specs, test reports
3. **Lab Testing:** Independent verification
4. **Audit:** QMS review (if medical device)
5. **Certification:** Issue WIA MED-019 certificate
6. **Surveillance:** Annual re-certification

### 9.3 Certification Levels

| Level | Requirements |
|-------|--------------|
| **Bronze** | Basic accuracy (Grade C) + Basic security |
| **Silver** | Good accuracy (Grade B) + FHIR support |
| **Gold** | High accuracy (Grade A) + Medical device clearance |

---

## 10. Future Roadmap

### Version 1.1 (Q4 2025)
- Blood pressure validation protocols
- Non-invasive glucose monitoring guidelines
- AI/ML algorithm transparency requirements

### Version 2.0 (2026)
- Advanced biosensors (hydration, lactate, cortisol)
- Implantable wearable integration
- Real-time health scoring standards

---

## Appendix A: Glossary

- **AFib:** Atrial Fibrillation
- **CGM:** Continuous Glucose Monitoring
- **ECG:** Electrocardiogram
- **FHIR:** Fast Healthcare Interoperability Resources
- **HRV:** Heart Rate Variability
- **MARD:** Mean Absolute Relative Difference
- **PPG:** Photoplethysmography
- **PSG:** Polysomnography (Sleep Study)
- **SpO2:** Oxygen Saturation

## Appendix B: References

1. FDA - General Wellness: Policy for Low Risk Devices (2016)
2. Apple HealthKit Developer Documentation
3. Google Fit REST API v1
4. HL7 FHIR R4 Specification
5. ISO 13485:2016 - Medical Devices QMS
6. IEC 62304:2006 - Medical Device Software
7. IEC 60601-1 - Medical Electrical Equipment Safety

---

**Document Control**

| Version | Date | Changes | Author |
|---------|------|---------|--------|
| 1.0.0 | 2025-01-15 | Initial release | WIA Standards Committee |

---

© 2025 WIA - World Certification Industry Association
弘益人間 · Benefit All Humanity

**License:** MIT License - Free to use, modify, and distribute with attribution.

**Contact:** standards@wiastandards.com
**Website:** https://wiastandards.com/med-019
**Certification:** https://cert.wiastandards.com
