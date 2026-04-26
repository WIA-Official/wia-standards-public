# WIA-SEMI-010 v1.0: Testing and Certification

## Document Information

- **Standard**: WIA-SEMI-010
- **Version**: 1.0
- **Date**: 2025-01-15
- **Category**: Quality Assurance

## 1. Scope

This specification defines testing requirements, procedures, and certification criteria for MicroLED displays and components.

## 2. Component-Level Testing

### 2.1 Wafer-Level LED Testing

**Photoluminescence (PL) Testing**:
- Coverage: 100% of chips
- Parameters: Peak wavelength, intensity, FWHM
- Mapping resolution: <1mm

**Electroluminescence (EL) Testing**:
- Coverage: Sampling (minimum 10% of wafer)
- Parameters: Forward voltage, light output, wavelength
- Test current: As specified for chip size

**Visual Inspection**:
- Magnification: >50× for chips <30μm
- Defect detection limit: >5μm features
- Coverage: 100%

### 2.2 Post-Transfer Testing

**Automated Optical Inspection (AOI)**:
- Resolution: <2μm/pixel
- Coverage: 100% of panel
- Detection: Missing chips, misplaced chips, damaged chips

**Electrical Test**:
- I-V characteristics: All pixels or statistical sample
- Short/open detection: 100% coverage
- Leakage current: Sampling per AQL standards

### 2.3 Repair Verification

- Re-test after repair: 100% of repaired pixels
- Repair success criteria: Same as original specification
- Documentation: Repair location map

## 3. Display-Level Testing

### 3.1 Optical Performance

**Brightness**:
- Measurement: 9-point or 13-point grid
- Instrument: Calibrated luminance meter
- Uniformity: <5% variation from mean

**Color Accuracy**:
- White point: D65 ±0.003 u'v' (CIELUV)
- Color gamut: Verify coverage of target color space
- Color uniformity: ΔE <2 (CIEDE2000)

**Contrast Ratio**:
- On/Off contrast: Measure in dark room
- ANSI contrast: Checkerboard pattern
- Minimum: >1,000:1 (>100,000:1 typical for MicroLED)

**Response Time**:
- Gray-to-gray transition: <5ms typical
- Black-to-white: <2ms typical
- Measurement: Photodetector + oscilloscope

**Viewing Angle**:
- Brightness at ±80°: >50% of normal
- Color shift: ΔE <3 at ±60°

### 3.2 Electrical Performance

**Power Consumption**:
- Full white: As specified
- Typical content: 30-50% of full white
- Standby: <1W

**Interface Compliance**:
- Verify compatibility with HDMI, DisplayPort, MIPI, etc.
- Signal integrity: Eye diagram analysis
- Timing: Verify refresh rate stability

### 3.3 Defect Inspection

**Dead Pixel Count**:
- Method: Display full white, red, green, blue patterns
- Classification: Per ISO 9241-305
- Acceptable level: Class I or II as specified

**Mura Detection**:
- Method: Display uniform gray patterns (5%, 25%, 50%, 75%)
- Limit: No visible mura at normal viewing distance (3× diagonal)

**Uniformity Analysis**:
- Brightness mura: <3% JND (Just Noticeable Difference)
- Color mura: ΔE <1.5 within 100mm × 100mm area

## 4. Reliability Testing

### 4.1 Lifetime Testing

**Accelerated Aging**:
- Test condition: 85°C, 85% RH, constant current
- Duration: 1,000 hours minimum
- Measurement intervals: 0h, 250h, 500h, 1,000h
- Criteria: <10% brightness loss at 1,000 hours

**Thermal Cycling**:
- Temperature range: -20°C to +60°C (operating) or -40°C to +85°C (storage)
- Ramp rate: 5°C/minute
- Dwell time: 30 minutes at each extreme
- Cycles: 500 minimum
- Criteria: <5% performance change, no physical damage

**Humidity Testing**:
- Condition: 85°C/85% RH (operating) or 65°C/95% RH (non-operating)
- Duration: 500 hours minimum
- Criteria: No corrosion, <10% performance change

### 4.2 Environmental Testing

**Vibration**:
- Frequency: 10-500 Hz
- Amplitude: 1.5mm or 1G depending on frequency
- Duration: 2 hours per axis (3 axes)
- Standard: IEC 60068-2-6

**Shock**:
- Peak acceleration: 50G (operating), 100G (non-operating)
- Duration: 11ms half-sine
- Number of shocks: 3 per axis, both directions (18 total)
- Standard: IEC 60068-2-27

**Drop Test**:
- Height: As specified for product category
- Surface: Concrete or equivalent
- Orientation: Multiple orientations
- Criteria: Functional after drop

### 4.3 Electrical Stress

**ESD Testing**:
- HBM (Human Body Model): ±2kV minimum
- CDM (Charged Device Model): ±500V minimum
- Standard: JESD22-A114, JESD22-C101

**Electromigration**:
- Test current: 1.5× rated current
- Temperature: 125°C
- Duration: 1,000 hours
- Criteria: <10% resistance increase

## 5. Safety and EMC Testing

### 5.1 Safety

- Electrical safety: IEC 60950-1 or IEC 62368-1
- Photobiological safety: IEC 62471
- Flammability: UL 94 V-0 or better for plastics

### 5.2 Electromagnetic Compatibility (EMC)

**Emissions**:
- Radiated emissions: FCC Part 15 Class B, CISPR 32
- Conducted emissions: CISPR 32

**Immunity**:
- ESD: IEC 61000-4-2, Level 3 minimum
- Radiated immunity: IEC 61000-4-3
- EFT/Burst: IEC 61000-4-4

## 6. Certification Process

### 6.1 Design Qualification

- Prototype build and characterization
- Full environmental and reliability testing
- Design review and approval

### 6.2 Production Qualification

- First article inspection
- Process capability study (Cpk >1.33)
- Ongoing reliability monitoring

### 6.3 Compliance Certification

- CE marking (Europe)
- FCC certification (USA)
- CCC certification (China)
- Local certifications as required

## 7. Sampling Plans

### 7.1 Incoming Inspection

- LED chips: MIL-STD-105E, AQL 0.65%
- TFT backplanes: MIL-STD-105E, AQL 1.0%
- Components: Per component criticality

### 7.2 In-Process Testing

- AOI: 100% coverage
- Electrical test: 100% or sampling based on process stability
- SPC: Continuous monitoring of key parameters

### 7.3 Final Inspection

- Functional test: 100%
- Optical test: 100%
- Packaging inspection: Per AQL plan

## 8. Documentation

### 8.1 Test Reports

Each test report shall include:
- Test conditions and parameters
- Equipment used (with calibration dates)
- Raw data or summary statistics
- Pass/fail criteria and results
- Operator and date

### 8.2 Certification Documents

- Certificate of Conformance
- Test data package
- Material declarations (RoHS, REACH, Conflict Minerals)
- Traceability records

## 9. Continuous Improvement

### 9.1 Failure Analysis

- FMEA (Failure Mode and Effects Analysis)
- Root cause analysis for returns
- Corrective and preventive actions (CAPA)

### 9.2 Process Monitoring

- SPC charts for key parameters
- Yield tracking and Pareto analysis
- Regular audits and reviews

## 10. Reference Standards

- IEC 60747: Discrete Semiconductor Devices
- IEC 62341: Organic Light Emitting Diode (OLED) Displays
- IEC 61747-1-3: Measurement of LED Parameters
- ISO 9241-305: Optical Performance and Pixel Defects
- JESD Standards: Reliability Testing

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
