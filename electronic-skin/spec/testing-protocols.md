# WIA-SEMI-016 Testing Protocols

## Comprehensive Test Procedures for Electronic Skin Devices

**Version**: 1.0
**Date**: 2025-01-01
**Organization**: WIA / SmileStory Inc.

---

## 1. Overview

This document provides detailed testing protocols for verifying compliance with WIA-SEMI-016 requirements. All tests should be performed in controlled laboratory conditions unless otherwise specified.

---

## 2. Mechanical Property Testing

### 2.1 Tensile Testing Protocol
**Objective**: Determine stretchability and Young's modulus
**Equipment**: Universal testing machine (Instron or equivalent)
**Sample**: 50mm × 10mm × thickness
**Procedure**:
1. Condition samples at 23±2°C, 50±5% RH for 24 hours
2. Mount sample with 30mm gauge length
3. Stretch at 10 mm/min until failure
4. Record force and displacement continuously
5. Calculate stress = Force/Area, strain = ΔL/L₀
6. Determine Young's modulus from linear region slope
7. Report: Modulus (MPa), elongation at break (%), ultimate strength (MPa)

### 2.2 Cyclic Durability Testing
**Objective**: Validate ≥10,000 cycle requirement
**Equipment**: Cyclic stretching machine
**Procedure**:
1. Mount device in test fixture
2. Program: 0-30% strain, 1 Hz, 10,000 cycles
3. Measure sensor output at cycles: 0, 100, 1000, 5000, 10000
4. Record any failures or degradation
5. Acceptance: ≥90% initial performance after 10,000 cycles

---

## 3. Sensing Performance Testing

### 3.1 Pressure Sensitivity Calibration
**Equipment**: Precision weights or pressure applicator
**Procedure**:
1. Zero baseline with no load
2. Apply pressures: 0.1, 0.5, 1, 2, 5, 10, 20, 50, 100 kPa
3. Record sensor output at each pressure
4. Repeat 3 times, calculate average
5. Fit linear or polynomial curve
6. Calculate sensitivity (kPa⁻¹), linearity (R²), hysteresis
7. Acceptance: Sensitivity ≥0.1 kPa⁻¹, R² ≥0.95, hysteresis <10%

### 3.2 Response Time Measurement
**Equipment**: Fast pressure applicator, oscilloscope or data acquisition at ≥1 kHz
**Procedure**:
1. Apply step pressure change (0 → 10 kPa)
2. Record output at 1000 Hz sampling
3. Measure time from 10% to 90% of final value
4. Repeat 10 times
5. Report average ± standard deviation
6. Acceptance: <20 ms for Class A, <50 ms for Class B/C

### 3.3 Spatial Resolution Testing
**Procedure**:
1. Apply two point contacts at varying separations
2. Start at 10 mm, decrease in 1 mm steps
3. Record minimum distance where two distinct signals appear
4. Test at multiple locations
5. Report average two-point discrimination threshold
6. Acceptance: ≤5 mm for Class A, ≤10 mm for Class B

---

## 4. Biocompatibility Testing

### 4.1 Cytotoxicity (ISO 10993-5)
**Method**: Extract test per ISO protocol
**Procedure**:
1. Extract material in cell culture medium at 37°C for 24 hours
2. Expose L929 cells to extract for 24 hours
3. Assess cell viability (MTT assay)
4. Calculate viability relative to negative control
5. Acceptance: ≥70% viability (Grade 0-1)

### 4.2 Skin Irritation (ISO 10993-10)
**Method**: Primary skin irritation test
**Test animals**: New Zealand white rabbits (if animal testing used) or human patch test
**Procedure**: Follow ISO 10993-10 exactly
**Acceptance**: Primary Irritation Index <2.0

---

## 5. Environmental Testing

### 5.1 Temperature Cycling
**Equipment**: Environmental chamber
**Procedure**:
1. Condition: -20°C for 30 min → +60°C for 30 min
2. Repeat for 10 cycles
3. Test functionality at each temperature extreme
4. Return to 23°C, allow 1 hour stabilization
5. Test all performance parameters
6. Acceptance: <10% change in all parameters

### 5.2 Humidity Resistance
**Equipment**: Humidity chamber
**Procedure**:
1. Expose to 85% RH, 40°C for 24 hours
2. Remove and test immediately (within 1 hour)
3. Compare to baseline measurements
4. Acceptance: Functional, <10% performance change

---

## 6. Electrical Safety Testing (Class A Devices)

### 6.1 Leakage Current (IEC 60601-1)
**Equipment**: Electrical safety analyzer
**Procedure**: Per IEC 60601-1 Section 8
**Limits**: 
- Patient auxiliary current: <100 μA
- Earth leakage: <500 μA

### 6.2 Dielectric Strength
**Test voltage**: 1500 V AC for 60 seconds
**Acceptance**: No breakdown

---

## 7. Data Communication Testing

### 7.1 BLE Performance
**Tests**:
- Connection establishment time: <3 seconds
- Data throughput: ≥250 kbps
- Range: ≥10 m line-of-sight
- Latency: <20 ms for real-time applications
- Packet loss: <1% at 10 m

**Equipment**: BLE protocol analyzer, Faraday cage

---

## 8. Accelerated Life Testing

### 8.1 Thermal Aging
**Procedure**:
1. Expose to 60°C for 1000 hours
2. Test functionality every 250 hours
3. Use Arrhenius equation to extrapolate to expected lifetime at use temperature
4. Target: 2 year lifetime at 25°C use temperature

---

## 9. Test Report Requirements

All test reports shall include:
- Test date, personnel, equipment used (with calibration dates)
- Sample identification and traceability
- Environmental conditions
- Complete procedure followed
- Raw data and calculated results
- Pass/fail determination
- Deviations from protocol (if any)
- Signature of qualified person

---

## 10. Quality Control Sampling Plans

**Incoming materials**: AQL 1.5 per ISO 2859-1
**In-process**: 100% inspection of critical parameters
**Final product**: AQL 1.0 for Class A, AQL 2.5 for Class B/C

---

**Document Control**:
- Next review date: 2026-01-01
- Revision frequency: Annual or as needed

© 2025 SmileStory Inc. / WIA
