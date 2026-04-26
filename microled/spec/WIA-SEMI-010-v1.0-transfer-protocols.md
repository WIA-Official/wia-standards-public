# WIA-SEMI-010 v1.0: Mass Transfer Protocols

## Document Information

- **Standard**: WIA-SEMI-010
- **Version**: 1.0
- **Date**: 2025-01-15
- **Category**: Manufacturing Processes

## 1. Scope

This specification defines requirements and best practices for mass transfer of MicroLED chips from donor wafers to receiving substrates.

## 2. Transfer Yield Requirements

### 2.1 Minimum Yield Targets

| Application | Minimum Transfer Yield |
|-------------|------------------------|
| Premium Displays | 99.99% |
| Standard Displays | 99.95% |
| Prototyping | 99.90% |

### 2.2 Placement Accuracy

- Position tolerance: ±2μm (3σ)
- Rotation tolerance: ±2°
- Height variation: ±1μm across panel

## 3. Transfer Technologies

### 3.1 Elastomer Stamp Transfer

**Process Parameters**:
- Stamp material: PDMS or equivalent
- Pickup pressure: 0.1-1.0 MPa
- Contact time: 1-10 seconds
- Release velocity: 0.1-10 mm/s
- Temperature: 20-150°C

**Requirements**:
- Stamp cleanliness: <10 particles/cm² >5μm
- Stamp lifetime: >100 cycles minimum
- Stamp dimensional stability: <±5μm over 100mm

### 3.2 Laser-Induced Forward Transfer (LIFT)

**Process Parameters**:
- Laser wavelength: 248nm, 355nm, or 1064nm
- Pulse duration: 5-50ns
- Energy: 1-100μJ per pulse
- Spot size: 10-50μm
- Repetition rate: 10-100kHz

**Requirements**:
- Chip damage: <0.01%
- Position accuracy: ±0.5μm
- Throughput: >10,000 chips/hour minimum

### 3.3 Fluidic Assembly

**Process Parameters**:
- Fluid: DI water or IPA-based
- Flow rate: 0.1-10 mL/min
- Binding site hydrophilicity: Contact angle <30°
- Assembly time: 1-60 minutes

**Requirements**:
- Chip orientation: <±5° variation
- Binding selectivity: >99%
- Fluid contamination: <10 ppb metals

### 3.4 Pick-and-Place

**Process Parameters**:
- Vacuum pressure: 50-500 mbar
- Placement force: 10-100 grams
- Speed: 3,000-50,000 chips/hour
- Vision alignment: <±1μm

**Requirements**:
- Gripper wear: <1% per 1M cycles
- Chip damage rate: <0.1%
- Placement repeatability: ±1μm (6σ)

## 4. Donor Wafer Preparation

### 4.1 Release Layer

- Compatibility with selected transfer method
- Uniformity: <±5% thickness variation
- No degradation during storage (<1% change over 6 months)

### 4.2 Chip Underside Metallization

- Planarity: <±0.5μm across wafer
- Adhesion: >10 MPa to transfer adhesive
- Compatibility with bonding method

## 5. Receiver Substrate Requirements

### 5.1 Landing Pad Specifications

- Material: Cu, Al, or ITO
- Size tolerance: ±0.5μm
- Height variation: <±0.2μm
- Surface roughness: Ra <100nm

### 5.2 Cleanliness

- Particle count: <10 particles/cm² >0.5μm
- Organic contamination: <1×10¹² atoms/cm²
- Metal contamination: <1×10¹⁰ atoms/cm²

## 6. Quality Control

### 6.1 In-Process Monitoring

- Real-time defect detection
- Statistical process control (SPC)
- Automated feedback loops

### 6.2 Post-Transfer Inspection

- AOI (Automated Optical Inspection): 100% coverage
- Electrical test: Sampling per MIL-STD-105E
- Defect classification per Section 7

## 7. Defect Classification

### 7.1 Transfer Defects

- Missing chips: <100 ppm
- Misplaced chips (>±2μm): <50 ppm
- Damaged chips: <10 ppm
- Contamination under chips: <50 ppm

### 7.2 Acceptable Defect Levels

Total defects (all categories): <300 ppm for premium displays

## 8. Environmental and Safety

- Cleanroom: Class 1000 minimum
- ESD protection: All surfaces grounded
- Temperature: 20-25°C ±2°C
- Humidity: 40-60% RH

## 9. Documentation

Each transfer batch requires:
- Process parameters log
- Defect maps
- Yield statistics
- Equipment maintenance records

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
