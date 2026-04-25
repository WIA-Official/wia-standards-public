# WIA-SEMI-015: Smart Sensor Standard - Overview

## Document Information

**Standard:** WIA-SEMI-015  
**Version:** 1.0  
**Date:** January 2025  
**Status:** Published  
**Organization:** World Certification Industry Association (WIA)

## Abstract

This specification defines the WIA-SEMI-015 Smart Sensor Standard, a comprehensive framework for intelligent sensors that integrate embedded AI, edge processing, and ultra-low-power operation. The standard enables autonomous, energy-efficient sensing systems capable of local data processing without constant cloud connectivity.

## Scope

This standard applies to:
- MEMS and semiconductor sensors with embedded intelligence
- Edge AI-enabled sensing devices
- Battery-powered and energy-harvesting sensors
- IoT-connected intelligent sensor systems
- Automotive, industrial, healthcare, and consumer sensor applications

## Architecture Overview

### System Components

**1. Sensor Elements**
- Physical transducers (accelerometer, gyroscope, magnetometer, environmental, optical, audio)
- Analog front-end (AFE) for signal conditioning
- Multi-sensor arrays for fusion applications

**2. Compute Platform**
- MCU: ARM Cortex-M series or RISC-V (minimum Cortex-M4 equivalent)
- Memory: ≥128 KB SRAM, ≥512 KB Flash
- Optional ML accelerator (NPU, DSP, or dedicated AI engine)

**3. Power Management**
- Ultra-low-power modes (deep sleep < 50 µA)
- Dynamic voltage and frequency scaling (DVFS)
- Energy harvesting interface (optional)

**4. Communication**
- Wireless: BLE 5.x, LoRaWAN, NB-IoT, Wi-Fi, or Zigbee/Thread
- Secure protocols: TLS 1.3, DTLS 1.2, or equivalent

**5. Security**
- Secure boot with cryptographic signature verification
- Hardware root of trust (TrustZone or equivalent)
- Encrypted communication channels
- OTA firmware update capability with rollback protection

## Key Requirements

### R1: Hardware Requirements

**R1.1** The sensor system SHALL include at least one digital sensor with interrupt capability.

**R1.2** The MCU SHALL provide:
- Minimum ARM Cortex-M4F or RISC-V RV32IMFC equivalent
- ≥128 KB SRAM
- ≥512 KB Flash
- Hardware RNG for cryptographic operations

**R1.3** Power modes SHALL include:
- Active mode: < 100 mW
- Deep sleep: < 50 µA
- Shutdown: < 5 µA (if supported)

### R2: Software Requirements

**R2.1** The system SHALL support on-device machine learning inference with:
- Model size: ≤ 500 KB
- Inference latency: < 100 ms for typical models
- Support for INT8 quantization

**R2.2** An RTOS or bare-metal scheduler SHALL manage tasks with priority-based preemption.

**R2.3** Error handling SHALL include:
- Watchdog timer enabled
- Graceful degradation on sensor failures
- Low-battery detection and safe shutdown

### R3: Communication Requirements

**R3.1** The system SHALL support at least one wireless protocol:
- BLE 5.0 or higher, OR
- LoRaWAN 1.0.3 or higher, OR
- NB-IoT Cat-M1/NB1, OR
- Wi-Fi 802.11b/g/n

**R3.2** Communication SHALL be encrypted using:
- TLS 1.3 or DTLS 1.2 (minimum), OR
- AES-128-GCM or stronger for application-layer encryption

**R3.3** OTA firmware updates SHALL be supported with:
- Cryptographic signature verification
- Atomic updates (dual-bank or equivalent)
- Rollback capability on failure

### R4: Power and Battery Life

**R4.1** For battery-powered devices, the system SHALL achieve:
- Minimum 1-year battery life on standard battery (e.g., CR2032, AA), OR
- Clear documentation of expected battery life under specified usage patterns

**R4.2** Power consumption SHALL be measured and documented for:
- Deep sleep mode
- Active sensing mode
- ML inference mode
- Wireless transmission mode

### R5: Security Requirements

**R5.1** Secure boot SHALL verify firmware integrity before execution using:
- RSA-2048, ECDSA P-256, or stronger signature algorithm

**R5.2** Cryptographic keys SHALL be stored in:
- Hardware secure element, OR
- OTP (One-Time Programmable) fuses, OR
- TrustZone secure world (for ARM Cortex-M33+)

**R5.3** Sensitive data (keys, personal information) SHALL NOT be stored in plaintext.

### R6: Testing and Validation

**R6.1** The system SHALL undergo:
- Unit testing with ≥60% code coverage
- Integration testing of all major interfaces
- Environmental testing (-20°C to +60°C operating range)
- EMI/EMC compliance testing

**R6.2** Field trials SHALL include:
- ≥100 devices deployed
- ≥1 month duration
- Real-world operating conditions

## Compliance Levels

### Level 1: Basic Compliance
- Hardware requirements (R1)
- Basic software (R2.1, R2.2)
- One wireless protocol (R3.1)
- Documented power consumption (R4.2)

### Level 2: Standard Compliance
- All Level 1 requirements
- Encrypted communication (R3.2)
- OTA updates (R3.3)
- Battery life targets (R4.1)
- Basic security (R5.1)

### Level 3: Advanced Compliance
- All Level 2 requirements
- Secure key storage (R5.2)
- Comprehensive testing (R6)
- Field trial validation (R6.2)

## Normative References

- IEEE 1451: Smart Transducer Interface Standards
- IEC 61508: Functional Safety
- ISO/IEC 27001: Information Security Management
- ETSI EN 303 645: Cyber Security for Consumer IoT
- NIST Cybersecurity Framework
- Bluetooth SIG BLE Specification 5.0+
- LoRa Alliance LoRaWAN Specification 1.0.3+

## Terminology

**Edge AI**: Machine learning inference performed on the sensor device rather than in the cloud.

**TinyML**: Machine learning models optimized for execution on microcontrollers with severe resource constraints.

**Sensor Fusion**: Combining data from multiple sensors to achieve superior performance.

**OTA (Over-The-Air)**: Wireless firmware update mechanism.

**Duty Cycling**: Operating in bursts of activity followed by low-power sleep periods.

**MEMS**: Micro-Electro-Mechanical Systems - miniaturized sensors fabricated using semiconductor processes.

---

## Conformance Statement

Products claiming conformance to WIA-SEMI-015 SHALL:

1. Meet all applicable requirements in Section "Key Requirements"
2. Declare the compliance level achieved (Level 1, 2, or 3)
3. Provide test reports documenting compliance
4. Include conformance statement in product documentation

---

© 2025 SmileStory Inc. / WIA  
弘益人間 (Hongik Ingan) · Benefit All Humanity
