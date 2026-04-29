# WIA-SEMI-017: 3D Touch Standard Specification v1.0

## Force Touch, Haptic Feedback & Pressure-Sensitive Interfaces

**Status:** Published  
**Version:** 1.0  
**Date:** 2025-01-15  
**Authors:** WIA Technical Committee  
**License:** CC BY-SA 4.0  

---

## 1. Executive Summary

This specification defines comprehensive standards for 3D Touch technology design, implementation, testing, and deployment. It covers force sensing architectures, pressure level calibration, haptic feedback systems, gesture recognition protocols, and UI framework integration guidelines.

### 1.1 Scope

The WIA-SEMI-017 standard applies to:
- Force-sensitive touch displays (smartphones, tablets, laptops)
- Pressure-sensitive trackpads and input devices
- Automotive touchscreen interfaces with haptic feedback
- Gaming controllers with force sensing
- Industrial human-machine interfaces (HMI)
- Medical devices requiring precise force control

### 1.2 Normative References

- WIA-SEMI-012: Sensor Technology Standard
- IEC 61000: Electromagnetic Compatibility
- ISO 26262: Automotive Functional Safety
- IEC 60601-1: Medical Electrical Equipment Safety
- WCAG 2.1: Web Content Accessibility Guidelines

### 1.3 Terms and Definitions

**3D Touch:** Touch interface technology that detects X-Y position (traditional touch) plus Z-axis force/pressure.

**Force Touch:** Apple's implementation of pressure-sensitive touch technology.

**Haptic Touch:** Software-based simulation of force sensing using long-press gestures.

**LRA (Linear Resonant Actuator):** Electromagnetic haptic actuator operating at mechanical resonance.

**Peek Gesture:** Light force press gesture (150-300gf) for content preview.

**Pop Gesture:** Deep force press gesture (300-500gf) for full content activation.

**Force Curve:** Relationship between applied force and sensor output (capacitance, resistance, voltage).

**Hysteresis:** Difference in sensor response when force is increasing vs. decreasing.

---

## 2. Force Sensing Architecture

### 2.1 Capacitive Force Sensing

#### 2.1.1 Layer Stack Requirements

**Minimum layer stack (bottom to top):**
1. Display panel (LCD/OLED)
2. Force sensor electrode array
3. Deformable dielectric layer (50-150 μm)
4. Cover glass (0.5-0.7 mm)
5. Capacitive touch sensor
6. Oleophobic coating

**Stack thickness increase:** ≤0.6 mm compared to non-force-sensing display

#### 2.1.2 Electrode Array Specifications

**Grid pattern:**
- Minimum: 4×6 electrodes (24 sensing points)
- Recommended: 6×10 electrodes (60 sensing points)
- High-density: 8×12 electrodes (96 sensing points)

**Electrode spacing:**
- Minimum pitch: 5 mm
- Recommended pitch: 8 mm
- Maximum pitch: 12 mm

**Electrode material:**
- ITO (Indium Tin Oxide): ≥85% transparency
- Metal mesh (Cu/Ag): ≤50 μm line width
- Graphene/CNT: Experimental

#### 2.1.3 Dielectric Layer Specifications

**Material properties:**
- Elastic modulus: 0.1-10 MPa
- Dielectric constant (εᵣ): 2-4
- Thickness uniformity: ±5 μm
- Temperature stability: -0.3% per °C maximum

**Compression characteristics:**
- Linear range: 0-15% compression
- Recovery time: <100 ms to 95% baseline
- Hysteresis: <10% of applied force

### 2.2 Strain Gauge Force Sensing

#### 2.2.1 Sensor Placement

**4-corner configuration:**
- One strain gauge at each corner of display
- Mounting: Rigid frame beneath display
- Force range: 0-10 N per sensor

**6-sensor configuration:**
- 4 corners + 2 center locations
- Improved spatial resolution
- Force range: 0-8 N per sensor

#### 2.2.2 Strain Gauge Specifications

**Type:** Metal foil or semiconductor strain gauge  
**Gauge factor (GF):** 2.0-2.2 (metal foil), 50-200 (semiconductor)  
**Resistance:** 120Ω, 350Ω, or 1000Ω  
**Strain range:** 0-2000 μ (micro-strain)  
**Linearity:** ±0.5% of full scale  
**Temperature coefficient:** <±0.01% per °C  

#### 2.2.3 Signal Conditioning

**Bridge configuration:** Wheatstone bridge (full bridge preferred)  
**Amplification:** Instrumentation amplifier, gain 100-500×  
**ADC resolution:** 16-24 bits  
**Sampling rate:** 100-1000 Hz  
**Low-pass filter:** 10-50 Hz cutoff  

### 2.3 Piezoelectric Force Sensing

#### 2.3.1 Actuator Types

**Bending actuators (unimorph/bimorph):**
- PZT layer: 0.1-0.5 mm thickness
- Substrate: Metal or flexible polymer
- Displacement: 0.1-2 mm @ 100V
- Force: 0.1-5 N

**Stack actuators:**
- Multiple PZT layers
- High force, low displacement
- Force: 10-1000 N
- Displacement: 10-100 μm

#### 2.3.2 Material Specifications

**PZT ceramics:**
- Piezoelectric charge constant (d₃₃): 200-600 pC/N
- Curie temperature: >250°C
- Dielectric constant: 1000-3000

**Lead-free alternatives (RoHS compliance):**
- BaTiO₃: d₃₃ = 190 pC/N
- KNN (Potassium Sodium Niobate): d₃₃ = 80-160 pC/N

#### 2.3.3 Driver Requirements

**Boost converter:**
- Output voltage: 100-200V DC
- Current capability: 50-200 mA
- Efficiency: >85%

**Drive signal:**
- Waveform: Square wave or sine wave
- Frequency: DC to 10 kHz
- Amplitude modulation: 0-100%

---

## 3. Force Level Specifications

### 3.1 Pressure Level Resolution

**Minimum resolution:** 256 discrete levels (8-bit)  
**Recommended resolution:** 1024 discrete levels (10-bit)  
**High resolution:** 4096 discrete levels (12-bit)  

**Force range:**
- Minimum detectable force: ≤1 gf (10 mN)
- Maximum force: 400-500 gf (4-5 N)
- Dynamic range: ≥60 dB

### 3.2 Force Detection Accuracy

**Absolute accuracy:** ±2-5% of full scale  
**Repeatability:** ±1-2% over 1000 cycles  
**Linearity error:** ±3% maximum deviation from best-fit line  
**Hysteresis error:** ≤10% of applied force  

### 3.3 Force Threshold Definitions

**Touch detection threshold:**
- Range: 10-30 gf
- Purpose: Distinguish intentional touch from hover/accidental contact

**Peek gesture threshold:**
- Default: 200 gf ±20%
- Adjustable range: 150-300 gf
- Hysteresis: 10-20 gf

**Pop gesture threshold:**
- Default: 400 gf ±20%
- Adjustable range: 300-500 gf
- Hysteresis: 20-40 gf

**Maximum force limit:**
- Saturation: 500-600 gf
- Mechanical limit: >800 gf (no damage to sensor)

---

## 4. Calibration Requirements

### 4.1 Factory Calibration

#### 4.1.1 Multi-Point Calibration

**Minimum calibration points:** 5 force levels
- 0 gf (baseline, no force)
- 50 gf (light touch reference)
- 150 gf (normal tap reference)
- 250 gf (peek threshold reference)
- 400 gf (pop threshold reference)

**Recommended calibration points:** 7-10 force levels for improved accuracy

#### 4.1.2 Spatial Calibration

**Test positions:**
- Minimum: Center + 4 corners (5 points)
- Recommended: Center + 4 corners + 4 edge midpoints (9 points)
- High-precision: 4×6 grid (24 points)

**Position tolerance:** ±2 mm from nominal position

#### 4.1.3 Temperature Calibration

**Calibration temperatures:**
- Cold: 0°C ±2°C
- Room: 25°C ±1°C
- Warm: 40°C ±2°C
- Hot (automotive): 60°C ±2°C (optional for automotive applications)

**Temperature compensation:**
- Maximum drift: -0.3% per °C
- Compensation accuracy: ±0.1% per °C

### 4.2 Calibration Data Storage

**Non-volatile storage:** EEPROM, Flash, or secure element  
**Data retention:** ≥10 years at 25°C  
**Write endurance:** ≥10,000 cycles  

**Calibration data structure:**
```json
{
  "version": "1.0",
  "device_id": "unique_serial_number",
  "calibration_date": "ISO 8601 timestamp",
  "temperature_reference": 25.0,
  "force_calibration": {
    "sensor_array": [
      {"force_gf": 0, "raw_value": 1234},
      {"force_gf": 50, "raw_value": 1456},
      {"force_gf": 150, "raw_value": 2345},
      {"force_gf": 250, "raw_value": 3456},
      {"force_gf": 400, "raw_value": 4567}
    ]
  },
  "temperature_coefficient": -0.002
}
```

### 4.3 Runtime Calibration

**Auto-zero calibration:**
- Trigger condition: No touch detected for >10 seconds
- Update frequency: Every 5-10 minutes when idle
- Drift threshold: ±5% of baseline before recalibration required

**User-adjustable sensitivity:**
- Light: 0.7× force thresholds
- Medium: 1.0× force thresholds (default)
- Firm: 1.4× force thresholds

---

## 5. Haptic Feedback Standards

### 5.1 LRA Specifications

**Resonant frequency:** 150-250 Hz ±5%  
**Acceleration:** 1-4 G (9.81-39.2 m/s²)  
**Response time:** <15 ms (10-90% rise time)  
**Latency:** <10 ms from trigger to perceptible vibration  

**Size classes:**
- Small: 7×7×2 mm (smartphones, wearables)
- Medium: 10×10×3 mm (tablets, larger phones)
- Large: 13×11×5 mm (laptops, trackpads)

**Lifetime:** ≥5 million activation cycles

### 5.2 Haptic Waveforms

#### 5.2.1 Click (Transient)

**Duration:** 10-15 ms total  
**Rise time:** 2-5 ms  
**Peak amplitude:** 3-5 G  
**Decay time:** 8-12 ms  
**Frequency:** LRA resonant frequency  

**Use cases:** Button press confirmation, Peek gesture entry

#### 5.2.2 Thud (Soft Impact)

**Duration:** 25-35 ms total  
**Rise time:** 5-10 ms  
**Peak amplitude:** 2-3 G  
**Decay time:** 15-20 ms  

**Use cases:** Pop gesture entry, heavy selection

#### 5.2.3 Continuous Vibration

**Duration:** 100-5000 ms  
**Envelope:** 10-20 ms fade in/out  
**Frequency:** LRA resonant ±10 Hz  
**Amplitude:** 0.5-2 G adjustable  

**Use cases:** Alerts, notifications, force feedback

### 5.3 Haptic Latency Requirements

**Total latency budget:** <20 ms from force detection to haptic onset

**Breakdown:**
- Touch/force scanning: <8 ms
- Processing (filtering, gesture detection): <5 ms
- Driver response: <7 ms

**Measurement:** High-speed camera (≥1000 fps) or accelerometer

---

## 6. Testing and Validation

### 6.1 Force Accuracy Testing

**Equipment:**
- Calibrated force applicator (±0.1% accuracy)
- Reference load cell (±0.05% accuracy)
- Automated test fixture

**Test procedure:**
1. Apply forces: 0, 50, 100, 150, 200, 250, 300, 400, 500 gf
2. Record sensor output for each force
3. Repeat 10 times at each force level
4. Calculate mean, standard deviation, linearity error

**Pass criteria:**
- Absolute accuracy: ±5% of applied force
- Repeatability: Standard deviation <2% of mean

### 6.2 Durability Testing

**Lifecycle test:**
- Activate force sensor 1 million times
- Force level: 300 gf (mid-range)
- Frequency: 1 Hz (60 cycles per minute)
- Duration: ~278 hours

**Pass criteria:**
- Calibration drift: <10% after 1M cycles
- No mechanical failure or delamination
- Haptic actuator: ≥5 million cycles without failure

### 6.3 Environmental Testing

**Temperature:**
- Operating range: 0°C to 60°C
- Storage range: -20°C to 70°C
- Temperature cycle: 1000 cycles (-20°C to 60°C)

**Humidity:**
- Operating: 10% to 90% RH non-condensing
- Storage: 5% to 95% RH

**Mechanical:**
- Drop test: 1.5 m onto hard surface (6 drops)
- Vibration: IEC 60068-2-64 (automotive: 10-2000 Hz)

---

## 7. Integration Guidelines

### 7.1 iOS Integration

**Framework:** UIKit  
**API:** UITouch.force, UIFeedbackGenerator  

**Force value normalization:**
```swift
let normalizedForce = touch.force / touch.maximumPossibleForce
// Returns 0.0 (no force) to 1.0 (maximum force)
```

**Haptic feedback:**
```swift
let generator = UIImpactFeedbackGenerator(style: .medium)
generator.prepare()
generator.impactOccurred()
```

### 7.2 Android Integration

**API:** MotionEvent.getPressure()  

**Pressure value:**
```java
float pressure = event.getPressure();
// Returns 0.0 (no pressure) to 1.0+ (normalized by device)
```

**Haptic feedback:**
```java
VibrationEffect effect = VibrationEffect.createOneShot(10, VibrationEffect.DEFAULT_AMPLITUDE);
vibrator.vibrate(effect);
```

### 7.3 Web Integration

**Force Touch Events API (Safari):**
```javascript
element.addEventListener('webkitmouseforcewillbegin', handleForceStart);
element.addEventListener('webkitmouseforcechanged', handleForceChange);
element.addEventListener('webkitmouseforcedown', handlePeek);
```

**Fallback (touch area estimation):**
```javascript
const force = (touch.radiusX * touch.radiusY) / CALIBRATION_CONSTANT;
```

---

## 8. Accessibility Requirements

### 8.1 Adjustable Force Thresholds

**Requirement:** Users must be able to adjust force thresholds or disable force sensing.

**Settings:**
- Light sensitivity (0.7× thresholds)
- Medium sensitivity (1.0× thresholds, default)
- Firm sensitivity (1.4× thresholds)
- Disable 3D Touch (use long-press alternative)

### 8.2 Alternative Interaction Methods

**Requirement:** All force-based interactions must have alternative input methods.

**Alternatives:**
- Long-press gesture (500-800 ms) for Peek
- Double-long-press for Pop
- Context menu buttons
- Accessibility shortcuts

### 8.3 Haptic Feedback Control

**Requirement:** Users must be able to:
- Disable haptic feedback
- Adjust haptic intensity (0-100%)
- Choose haptic patterns (subtle, standard, pronounced)

---

## 9. Security and Privacy

### 9.1 Force Data Privacy

**Requirement:** Force data shall not be transmitted to external servers without explicit user consent.

**Biometric considerations:**
- Force application patterns may be user-identifiable
- Treat as sensitive data (same level as touch patterns)
- Encrypt stored calibration data

### 9.2 Calibration Integrity

**Requirement:** Prevent malicious calibration tampering.

**Protection:**
- Store calibration data in secure element (if available)
- Cryptographic signature on calibration data
- Tamper detection (checksum validation)

---

## 10. Compliance and Certification

### 10.1 WIA Certification Process

**Levels:**
- **Basic Compliance:** Meets minimum specifications (Sections 2-5)
- **Full Compliance:** Passes all testing requirements (Section 6)
- **Premium Certification:** Exceeds specifications + accessibility + security

**Certification mark:** "WIA-SEMI-017 Certified"

### 10.2 Regulatory Compliance

**Electromagnetic compatibility:**
- IEC 61000-6-3 (Emissions)
- IEC 61000-6-1 (Immunity)

**Safety:**
- IEC 62368-1 (Audio/video equipment)
- UL 60950 (Information technology equipment)

**Automotive (if applicable):**
- ISO 26262 ASIL-B minimum for HMI
- AEC-Q100 component qualification

**Medical (if applicable):**
- IEC 60601-1 (Medical electrical equipment)
- FDA 510(k) clearance (US)
- CE Mark (EU)

---

## Appendix A: Reference Implementations

See GitHub repository: https://github.com/WIA-Official/wia-standards/tree/main/standards/3d-touch

**Contents:**
- TypeScript SDK for force sensing
- iOS example project
- Android example project
- Calibration tools
- Test fixtures

---

## Appendix B: Revision History

**Version 1.0 (2025-01-15):**
- Initial release
- Specifications for capacitive, strain gauge, and piezoelectric force sensing
- Haptic feedback standards
- Calibration and testing protocols

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
