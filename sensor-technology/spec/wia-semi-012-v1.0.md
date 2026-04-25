# WIA-SEMI-012: Sensor Technology Standard v1.0

**Status:** Published  
**Date:** 2025-01-15  
**Authors:** WIA Technical Committee  
**License:** CC BY-SA 4.0

---

## 1. Introduction

### 1.1 Scope

This standard defines specifications, interfaces, and best practices for sensor technologies including:
- MEMS sensors (accelerometers, gyroscopes, magnetometers, pressure sensors)
- CMOS image sensors (CIS)
- Environmental sensors (temperature, humidity, gas, air quality)
- Optical sensors (ambient light, proximity, UV)
- Multi-sensor integration and fusion

### 1.2 Purpose

To establish:
- Common terminology and metrics
- Standard communication interfaces
- Calibration and testing procedures
- Performance benchmarking methodologies
- Integration guidelines

### 1.3 Normative References

- IEEE 1451: Smart Transducer Interface Standards
- IEC 61508: Functional Safety
- ISO 26262: Automotive Functional Safety
- I2C Specification v6.0
- SPI Specification
- MIPI CSI-2 v4.0

---

## 2. Terminology and Definitions

**Sensor:** A device that converts physical or chemical stimulus into an electrical signal.

**Resolution:** The smallest detectable change in input that produces a change in output.

**Sensitivity:** The ratio of output change to input change (e.g., mV/°C, LSB/g).

**Noise Floor:** The minimum detectable signal above the sensor's intrinsic noise.

**Dynamic Range:** The ratio between the maximum and minimum measurable values, expressed in dB.

**Bandwidth:** The frequency range over which the sensor can accurately respond.

**Cross-Axis Sensitivity:** Unwanted sensitivity to stimuli on orthogonal axes.

**MEMS:** Micro-Electro-Mechanical Systems - miniaturized sensors with mechanical and electrical components.

**IMU:** Inertial Measurement Unit - combination of accelerometers and gyroscopes.

**BSI:** Back-Side Illumination - image sensor technology where light enters from the back.

---

## 3. MEMS Sensor Specifications

### 3.1 Accelerometer Requirements

**Performance Metrics:**
- Full-scale range: ±2g, ±4g, ±8g, ±16g (selectable)
- Resolution: Minimum 12-bit, recommended 16-bit
- Noise density: <150 μg/√Hz
- Zero-g offset: ±40 mg initial, ±10 mg after calibration
- Temperature coefficient: <0.5 mg/°C
- Bandwidth: 10 Hz to 1 kHz minimum

**Calibration:**
- Six-position calibration required
- Temperature compensation across -40°C to +85°C
- Scale factor accuracy: ±2% before, ±0.5% after calibration

**Interfaces:**
- I2C (100/400 kHz) or SPI (up to 10 MHz)
- Interrupt pins for data-ready, threshold detection
- FIFO buffer: Minimum 32 samples recommended

### 3.2 Gyroscope Requirements

**Performance Metrics:**
- Full-scale range: ±250, ±500, ±1000, ±2000 dps
- Resolution: Minimum 16-bit
- Noise density: <0.01 dps/√Hz (consumer), <0.005 dps/√Hz (premium)
- Zero-rate level: ±10 dps initial
- Bias instability: <5 dps (consumer), <1 dps (automotive)
- Temperature coefficient: <0.03 dps/°C

**Calibration:**
- Static bias calibration at multiple temperatures
- G-sensitivity compensation (optional for high-performance)

### 3.3 Magnetometer Requirements

**Performance Metrics:**
- Full-scale range: ±50 to ±1600 μT
- Resolution: 16-bit minimum
- Noise: <0.3 μT RMS
- Sensitivity: 0.15 μT/LSB typical

**Calibration:**
- Hard-iron and soft-iron calibration mandatory
- 3D ellipsoid fitting algorithm

---

## 4. Image Sensor Specifications

### 4.1 CMOS Image Sensor Requirements

**Pixel Architecture:**
- 4T (Four-Transistor) pixel minimum for quality applications
- Dual Conversion Gain (DCG) recommended for HDR

**Performance Metrics:**
- Resolution: 2MP to 200MP
- Pixel size: 0.6 μm to 2.0 μm
- Quantum Efficiency (QE): >70% @ 550nm (BSI sensors)
- Dynamic range: >70 dB (standard), >100 dB (HDR)
- Frame rate: 30 fps minimum, 60 fps recommended

**Interfaces:**
- MIPI CSI-2 for mobile/automotive
- Parallel interface for industrial (optional)
- I2C/SPI for sensor configuration

---

## 5. Environmental Sensor Specifications

### 5.1 Temperature Sensors

**Accuracy Requirements:**
- Consumer: ±1°C
- Industrial: ±0.5°C
- Precision: ±0.1°C

**Range:**
- Operating: -40°C to +125°C minimum
- Extended: -55°C to +150°C (automotive)

### 5.2 Humidity Sensors

**Performance:**
- Range: 0-100% RH
- Accuracy: ±3% RH (consumer), ±1.8% RH (industrial)
- Response time: <8 seconds (τ63)

### 5.3 Pressure Sensors

**Barometric Pressure:**
- Range: 300-1250 hPa
- Accuracy: ±1 hPa (consumer), ±0.5 hPa (precision)
- Altitude resolution: <1 meter

---

## 6. Communication Interfaces

### 6.1 I2C Protocol

**Addressing:**
- 7-bit or 10-bit addressing
- Default addresses documented in datasheet
- Support for multiple devices on same bus

**Speed:**
- Standard mode: 100 kHz
- Fast mode: 400 kHz
- Fast mode plus: 1 MHz (optional)

**Register Map:**
- WHO_AM_I register: Device identification
- Control registers: Configuration, power modes
- Data registers: Sensor outputs
- Status registers: Data ready, error flags

### 6.2 SPI Protocol

**Configuration:**
- Mode 0 or Mode 3
- Clock speed: Up to 10 MHz
- MSB first transmission

---

## 7. Calibration Standards

### 7.1 Factory Calibration

All sensors SHALL be calibrated at factory with:
- Multi-point calibration (minimum 3 points)
- Temperature characterization (-20°C, +25°C, +60°C minimum)
- Calibration coefficients stored in non-volatile memory

### 7.2 Field Calibration

Sensors SHOULD support field calibration via:
- Software commands
- Documented calibration procedures
- Validation against known references

---

## 8. Testing and Validation

### 8.1 Environmental Testing

**Temperature:**
- Operating range verification
- Thermal shock: -40°C to +85°C, 15-minute dwell

**Vibration:**
- Random vibration: 10 G RMS, 20-2000 Hz

**Shock:**
- Half-sine pulse: 1500 G, 0.5 ms

### 8.2 Performance Testing

**MEMS:**
- Sensitivity verification
- Noise measurement
- Cross-axis sensitivity

**Image Sensors:**
- Dark current
- QE measurement
- Dynamic range testing

---

## 9. Power Management

**Power Modes:**
- Active: Full operation
- Low-power: Reduced sampling rate
- Sleep: Minimal current, wake-up capability
- Shutdown: Complete power-off

**Specifications:**
- Active current: Documented in datasheet
- Sleep current: <10 μA recommended
- Wake-up time: <10 ms recommended

---

## 10. Compliance and Certification

Conformance to this standard demonstrated by:
- Test reports following Section 8
- Datasheet completeness
- Self-declaration or third-party testing

---

© 2025 SmileStory Inc. / WIA  
弘益人間 (Hongik Ingan) · Benefit All Humanity
