# WIA MED-005: Medical IoT Standard

**Version:** 1.0.0
**Status:** Published
**Date:** 2025-12-26
**Organization:** WIA - World Certification Industry Association
**Philosophy:** 弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## Abstract

WIA MED-005 defines the standard for Internet of Medical Things (IoMT), providing comprehensive specifications for connected medical devices, sensor networks, data aggregation, edge computing, security, and interoperability.

## 1. Scope

This standard applies to:
- Connected medical devices and wearables
- Medical sensor networks
- Healthcare data aggregation systems
- Edge computing infrastructure in medical environments
- Remote patient monitoring systems
- Medical IoT security protocols
- Interoperability with HL7 FHIR, DICOM, and other healthcare standards

## 2. Normative References

- HL7 FHIR R4
- HL7 v2.x
- DICOM PS3
- IEEE 11073 (Point-of-care medical device communication)
- ISO/IEEE 11073-20601 (Personal Health Devices)
- HIPAA Security Rule
- GDPR
- FDA Medical Device Cybersecurity Guidance
- NIST Cybersecurity Framework

## 3. Terms and Definitions

### 3.1 IoMT (Internet of Medical Things)
The ecosystem of connected medical devices, applications, and health systems that communicate via networks to enable healthcare data collection, analysis, and transmission.

### 3.2 Connected Medical Device
A medical device with network connectivity capabilities for data transmission and remote control.

### 3.3 Remote Patient Monitoring (RPM)
Healthcare delivery method using technology to monitor patient health data outside of traditional clinical settings.

### 3.4 Edge Computing
Distributed computing paradigm bringing computation and data storage closer to data sources.

## 4. Device Classification

### 4.1 Wearable Devices
- Smartwatches
- Fitness bands
- Smart patches
- Smart clothing

### 4.2 Diagnostic Devices
- Continuous glucose monitors (CGM)
- Blood pressure monitors
- ECG monitors
- Pulse oximeters

### 4.3 Therapeutic Devices
- Insulin pumps
- Smart inhalers
- Pacemakers
- Neurostimulators

## 5. Communication Protocols

### 5.1 Supported Protocols
- **Bluetooth Low Energy (BLE)** - For wearables and personal devices
- **WiFi (802.11)** - For high-bandwidth hospital equipment
- **5G/LTE-M/NB-IoT** - For cellular connectivity
- **MQTT** - For publish/subscribe messaging
- **CoAP** - For constrained devices
- **HTTPS/REST** - For web services

### 5.2 Data Formats
All data MUST be transmitted in standardized JSON format conforming to WIA MED-005 schema.

## 6. Data Security

### 6.1 Encryption Requirements
- **Transport:** TLS 1.3 minimum
- **Storage:** AES-256-GCM
- **Keys:** RSA-4096 or ECDSA-P256

### 6.2 Authentication
- Mutual TLS (mTLS) for device authentication
- OAuth 2.0 + JWT for user authentication
- Hardware-based authentication (TPM 2.0) recommended

### 6.3 Privacy
- HIPAA compliance mandatory for US deployments
- GDPR compliance mandatory for EU deployments
- Data anonymization and pseudonymization support

## 7. Interoperability

### 7.1 HL7 FHIR Integration
All IoMT data MUST be mappable to FHIR Resources:
- Observation
- Device
- Patient
- DiagnosticReport

### 7.2 Data Transformation
WIA MED-005 devices MUST support transformation to:
- HL7 FHIR R4
- HL7 v2.x messages
- DICOM (for imaging devices)

## 8. Quality Requirements

### 8.1 Reliability
- 99.5% uptime minimum
- Mean Time Between Failures (MTBF): 10,000 hours
- Automatic failover and recovery

### 8.2 Data Accuracy
- Sensor accuracy: ±5% or better
- Regular calibration required
- Quality metrics reported

## 9. Certification

### 9.1 Requirements
To receive WIA MED-005 certification, devices MUST:
1. Pass security audit
2. Demonstrate interoperability
3. Meet accuracy requirements
4. Comply with regulatory standards (FDA/CE)
5. Complete integration testing

### 9.2 Maintenance
- Annual recertification required
- Vulnerability patching within 30 days
- Firmware update capability (OTA)

## 10. Conformance

Implementation conformance is measured across:
- Protocol compliance
- Data format adherence
- Security requirements
- Interoperability capabilities
- Performance benchmarks

---

## Appendix A: JSON Schema Examples

See separate schema files in `/schemas/` directory.

## Appendix B: Integration Examples

See implementation guides in the ebook.

## Appendix C: Security Checklist

See security audit template.

---

**License:** MIT
**Copyright:** © 2025 WIA - World Certification Industry Association
**Contact:** standards@wiastandards.com

弘益人間 · Benefit All Humanity
