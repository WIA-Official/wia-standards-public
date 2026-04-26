# WIA-SEMI-016 Electronic Skin Standard
## Version 1.0 - Technical Specification

**Status**: Published
**Date**: 2025-01-01
**Organization**: World Certification Industry Association (WIA) / SmileStory Inc.
**Category**: Semiconductor Equipment and Materials International (SEMI) Standards

---

## 1. Scope

This standard establishes requirements and test methods for electronic skin (e-skin) devices intended for use in prosthetics, healthcare monitoring, robotics, and human-machine interfaces. It covers mechanical properties, sensing performance, electrical characteristics, biocompatibility, and data communication protocols.

### 1.1 Applicability

This standard applies to:
- Flexible and stretchable sensor systems
- Wearable health monitoring devices
- Prosthetic sensory feedback systems
- Robotic tactile sensing systems
- Human-machine interface devices utilizing skin-like sensing

### 1.2 Exclusions

This standard does not cover:
- Rigid pressure sensors
- Non-biocompatible industrial sensors
- Implantable electrodes (covered by separate standards)
- Display technologies (unless integrated with e-skin sensing)

---

## 2. Normative References

The following standards contain provisions which, through reference in this text, constitute provisions of this standard:

- ISO 10993 (all parts): Biological evaluation of medical devices
- IEC 60601-1: Medical electrical equipment safety
- ASTM F2027: Standard Guide for Characterization and Testing of Raw or Starting Biomaterials
- IEEE 1451: Smart transducer interface standards
- ISO 13485: Medical devices quality management systems
- FDA 21 CFR Part 11: Electronic records and signatures
- GDPR: General Data Protection Regulation (for data-collecting devices)

---

## 3. Terms and Definitions

### 3.1 Electronic Skin (E-Skin)
Flexible, stretchable sensor system that mimics tactile sensing properties of biological skin.

### 3.2 Stretchability
Maximum strain (elongation) that a material or device can withstand without mechanical failure or significant performance degradation, expressed as percentage of original length.

### 3.3 Sensitivity
Change in sensor output per unit change in measured stimulus. For pressure sensors: (ΔSignal/Signal₀)/ΔPressure, units of kPa⁻¹.

### 3.4 Biocompatibility
Ability of a material to perform with appropriate host response in a specific application, as determined by ISO 10993 testing.

### 3.5 Response Time
Time required for sensor output to reach 90% of final value after stimulus application.

### 3.6 Hysteresis
Difference in sensor output during loading and unloading cycles, expressed as percentage of full-scale output.

---

## 4. Classification

### 4.1 By Application Domain
- **Class A**: Medical devices (prosthetics, patient monitoring)
- **Class B**: Consumer electronics (wearables, smartphones)
- **Class C**: Industrial/Robotics (collaborative robots, automation)
- **Class D**: Research and development

### 4.2 By Contact Duration
- **Type I**: Limited contact (<24 hours)
- **Type II**: Prolonged contact (24 hours to 30 days)
- **Type III**: Permanent contact (>30 days)

### 4.3 By Sensing Modality
- **SM-P**: Pressure/force sensing
- **SM-T**: Temperature sensing
- **SM-S**: Strain/deformation sensing
- **SM-M**: Multi-modal sensing (2+ modalities)

---

## 5. Requirements

### 5.1 Mechanical Properties

#### 5.1.1 Stretchability
- **Minimum requirement**: ≥30% strain without failure
- **Test method**: Uniaxial tensile test per ASTM D412
- **Acceptance criteria**: No cracking, delamination, or >10% resistance increase at 30% strain
- **Advanced performance**: ≥100% strain for body-conforming applications

#### 5.1.2 Thickness
- **Range**: 10 μm to 500 μm
- **Uniformity**: ±10% across device area
- **Test method**: Micrometer or profilometry

#### 5.1.3 Young's Modulus
- **Target range**: 0.5-2 MPa (to match skin compliance)
- **Test method**: Tensile testing per ASTM D638
- **Measurement**: Slope of stress-strain curve in linear elastic region

#### 5.1.4 Durability
- **Minimum requirement**: 10,000 stretch cycles at 30% strain
- **Performance retention**: ≥90% of initial sensitivity after cycling
- **Test method**: Cyclic loading at 1 Hz frequency
- **Failure criteria**: Mechanical failure, >20% resistance change, or >5% sensitivity loss

### 5.2 Sensing Performance

#### 5.2.1 Pressure Sensing

**Pressure Range**:
- Minimum detectable: ≤0.1 kPa (light touch)
- Maximum reliable: ≥10 kPa (firm grip)
- Extended range (optional): Up to 100 kPa

**Sensitivity**:
- Minimum: ≥0.1 kPa⁻¹ over working range
- Target: ≥0.5 kPa⁻¹ for high-performance applications

**Spatial Resolution**:
- Medical/prosthetic (Class A): ≤5 mm two-point discrimination
- Consumer (Class B): ≤10 mm
- Industrial (Class C): Application-dependent

**Response Time**:
- Class A (medical): <20 ms
- Class B/C: <50 ms
- Real-time applications: <10 ms

**Hysteresis**:
- Maximum: ≤10% of full-scale output
- Measurement: Difference between loading and unloading at 50% of full scale

#### 5.2.2 Temperature Sensing

**Temperature Range**:
- Operating: -20°C to +60°C
- Storage: -40°C to +85°C

**Accuracy**:
- Medical applications: ±0.2°C
- General purpose: ±0.5°C

**Response Time**:
- In air: <30 seconds to 90% of final value
- In contact: <10 seconds

#### 5.2.3 Strain Sensing

**Strain Range**: 0-30% minimum (0-100% for advanced)
**Gauge Factor**: 1-100 depending on mechanism
**Linearity**: R² ≥ 0.95 over working range

### 5.3 Electrical Characteristics

#### 5.3.1 Sheet Resistance (for conductive layers)
- Target: <100 Ω/sq
- Uniformity: <20% variation across device
- Stability: <10% change over 1000 hours at operating conditions

#### 5.3.2 Signal-to-Noise Ratio
- Minimum: 40 dB for all sensing modalities
- Target: ≥50 dB for medical applications
- Measurement bandwidth: 0.1-100 Hz

#### 5.3.3 Power Consumption
- Active mode: <100 mW per device for typical operation
- Sleep mode: <1 mW
- Target: <10 mW for battery-powered wearables

#### 5.3.4 Operating Voltage
- Low voltage: 1.8-3.3 V (preferred for wearables)
- Standard: Up to 5 V
- Maximum: 12 V (with appropriate safety measures)

### 5.4 Biocompatibility (Class A Devices)

All materials in patient contact must comply with ISO 10993:

#### 5.4.1 Cytotoxicity (ISO 10993-5)
- Test: Extract test or direct contact
- Requirement: ≥70% cell viability (Grade 0-1)
- Cell line: L929 or human dermal fibroblasts

#### 5.4.2 Sensitization (ISO 10993-10)
- Test: Guinea pig maximization test or human repeat insult patch test
- Requirement: No sensitization reactions

#### 5.4.3 Irritation (ISO 10993-10)
- Test: Primary skin irritation
- Requirement: Primary Irritation Index <2.0

#### 5.4.4 Systemic Toxicity (ISO 10993-11)
- Required for Type II and III devices
- Test: Acute, subacute, or chronic per contact duration
- Requirement: No evidence of systemic toxicity

### 5.5 Environmental Resistance

#### 5.5.1 Temperature Cycling
- Test: -20°C to +60°C, 10 cycles
- Dwell time: 30 minutes at each extreme
- Requirement: <10% performance change

#### 5.5.2 Humidity Resistance
- Test: 85% RH at 40°C for 24 hours
- Requirement: Functional after exposure, <10% performance change

#### 5.5.3 Chemical Resistance
- Test: Exposure to common substances (water, ethanol, oils, perspiration simulant)
- Duration: 24 hours
- Requirement: No delamination, <15% performance change

### 5.6 Data Communication

#### 5.6.1 Wireless Protocols (if applicable)
Supported protocols shall include at least one of:
- Bluetooth Low Energy 5.0 or later
- Zigbee 3.0 or later
- NFC
- Wi-Fi 6 (for high-bandwidth applications)

#### 5.6.2 Data Rate
- Minimum: 250 kbps
- Target: ≥1 Mbps for multi-sensor arrays

#### 5.6.3 Latency
- Real-time applications: <20 ms
- General monitoring: <100 ms

#### 5.6.4 Range
- BLE: ≥10 m line-of-sight
- Other protocols: Per specification

#### 5.6.5 Data Security
- Encryption: AES-128 minimum for medical data
- Authentication: Device and user authentication required for Class A
- Privacy: Compliance with HIPAA (US), GDPR (EU), or equivalent

---

## 6. Test Methods

### 6.1 Mechanical Testing

#### 6.1.1 Stretchability Test
1. Prepare sample: 50 mm × 10 mm strip
2. Mount in tensile tester with 30 mm gauge length
3. Stretch at 10 mm/min until failure or 100% strain
4. Record strain at failure and stress-strain curve
5. For sensors: Measure electrical properties during stretching

#### 6.1.2 Durability Cycling Test
1. Mount sample in cyclic tester
2. Apply 0-30% strain at 1 Hz
3. Run for 10,000 cycles
4. Measure sensor performance every 1,000 cycles
5. Record failure mode if occurs

### 6.2 Sensing Performance Testing

#### 6.2.1 Pressure Sensitivity
1. Calibrated weight method:
   - Place sensor on rigid surface
   - Apply known weights from 0.1 g to 1 kg
   - Record sensor output at each weight
   - Calculate sensitivity from slope

2. Pressure applicator method:
   - Use pneumatic or mechanical pressure applicator
   - Apply pressures from 0.1 to 100 kPa
   - Record sensor output
   - Calculate sensitivity, linearity, hysteresis

#### 6.2.2 Response Time
1. Apply step pressure change
2. Record sensor output at ≥1 kHz sampling rate
3. Measure time to 90% of final value
4. Repeat 10 times, report average

### 6.3 Biocompatibility Testing
Follow ISO 10993 protocols exactly as specified.
Testing must be performed by accredited laboratory.

### 6.4 Environmental Testing
Per IEC 60068 series environmental testing standards.

---

## 7. Marking and Documentation

### 7.1 Device Marking
Each device or packaging shall be marked with:
- Model number and serial number
- Manufacturer name and address
- "WIA-SEMI-016 Compliant" (if applicable)
- Classification (Class, Type, SM)
- Manufacturing date
- Sterilization method (if applicable)

### 7.2 Documentation
Manufacturers shall provide:
- User manual with operating instructions
- Technical datasheet with all specifications
- Safety warnings and contraindications
- Calibration procedures
- Compliance documentation (test reports, certificates)

---

## 8. Quality Management

Manufacturers of Class A (medical) devices shall implement ISO 13485 quality management system.
All classes should implement appropriate quality controls including:
- Incoming material inspection
- In-process quality checks
- Final product testing
- Traceability systems
- Complaint handling and adverse event reporting

---

## 9. Compliance and Certification

### 9.1 Self-Declaration
Manufacturers may self-declare compliance for Class B, C, D devices by:
- Performing all required tests
- Maintaining test records for 5 years
- Including "WIA-SEMI-016 Compliant" in documentation

### 9.2 Third-Party Certification
Class A devices require third-party testing and certification:
- Testing by accredited laboratory
- Review by certification body
- Certificate valid for 3 years
- Annual surveillance audits

---

## 10. Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-01-01 | Initial publication |

---

## Appendix A: Recommended Materials

### A.1 Substrates
- PDMS (Polydimethylsiloxane): Sylgard 184, Dow Corning
- Ecoflex: 00-30, Smooth-On
- TPU: Medical-grade thermoplastic polyurethane
- Hydrogels: PVA, alginate (for specific applications)

### A.2 Conductors
- Silver nanowires: Dispersions from ACS Material, Nanowires
- Carbon nanotubes: Single-wall or multi-wall from suppliers
- PEDOT:PSS: Clevios series from Heraeus
- Graphene: Reduced graphene oxide or CVD graphene

### A.3 Encapsulation
- Medical-grade silicone
- Parylene-C
- Polyurethane coatings

---

## Appendix B: Sample Datasheets

[Templates for documenting device specifications in standardized format]

---

## Appendix C: Bibliography

[Key research papers and standards documents]

---

**For questions or comments**: standards@wia-official.org

**Copyright © 2025 SmileStory Inc. / WIA. All rights reserved.**

弘益人間 (홍익인간) · Benefit All Humanity
