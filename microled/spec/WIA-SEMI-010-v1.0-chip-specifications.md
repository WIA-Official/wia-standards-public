# WIA-SEMI-010 v1.0: MicroLED Chip Specifications

## Document Information

- **Standard**: WIA-SEMI-010
- **Version**: 1.0
- **Date**: 2025-01-15
- **Status**: Published
- **Category**: Semiconductor Display Standards

## 1. Scope

This specification defines the requirements for MicroLED chips used in display applications, including chip dimensions, optical properties, electrical characteristics, and quality metrics.

## 2. Chip Dimensions

### 2.1 Size Categories

| Category | Chip Size (μm) | Primary Applications |
|----------|----------------|----------------------|
| Ultra-Micro | <10 | AR/VR microdisplays |
| Small | 10-30 | Smartphones, tablets |
| Medium | 30-75 | Wearables, automotive |
| Large | 75-150 | Large format, transparent displays |

### 2.2 Dimensional Tolerances

- Chip size tolerance: ±5% of nominal dimension
- Thickness variation: <±10μm
- Edge straightness: <2μm deviation over chip length
- Corner radius: <5μm

### 2.3 Mesa Structure

- Mesa height: 1.5-3.0μm (from substrate to p-contact)
- Sidewall angle: 70-90° from substrate plane
- Surface roughness (Ra): <50nm

## 3. Optical Properties

### 3.1 Wavelength Specifications

#### Blue LEDs
- Peak wavelength: 450-470nm
- Spectral width (FWHM): 15-25nm
- Wavelength tolerance: ±5nm within batch

#### Green LEDs
- Peak wavelength: 520-535nm
- Spectral width (FWHM): 25-40nm
- Wavelength tolerance: ±7nm within batch

#### Red LEDs (AlGaInP)
- Peak wavelength: 615-630nm
- Spectral width (FWHM): 15-25nm
- Wavelength tolerance: ±5nm within batch

#### Red LEDs (InGaN, emerging)
- Peak wavelength: 610-630nm
- Spectral width (FWHM): 25-40nm
- Wavelength tolerance: ±8nm within batch

### 3.2 Light Output

- Luminous intensity: Specified per application
- Uniformity across wafer: <±15% from mean
- External quantum efficiency (EQE):
  - Blue: >55%
  - Green: >35%
  - Red (AlGaInP): >15% (for chips >30μm)
  - Red (InGaN): >30% (target)

### 3.3 Emission Pattern

- Lambertian or near-Lambertian emission
- Viewing angle (50% intensity): >120°
- No significant emission directionality bias

## 4. Electrical Characteristics

### 4.1 Forward Voltage

At specified test current (typically 100μA for small chips):

- Blue LEDs: 2.5-3.5V
- Green LEDs: 2.6-3.6V
- Red LEDs (AlGaInP): 1.8-2.5V
- Red LEDs (InGaN): 2.4-3.2V

Tolerance: ±0.3V within batch

### 4.2 Reverse Leakage

At -5V reverse bias:
- Leakage current: <10nA
- No breakdown or avalanche behavior

### 4.3 Series Resistance

- Total series resistance: <50Ω for chips >20μm
- Contact resistance: <1×10⁻⁴ Ω·cm²

### 4.4 Capacitance

- Junction capacitance @ 0V: Typically 10-100pF depending on chip size
- Measured at 1MHz

## 5. Thermal Properties

### 5.1 Thermal Resistance

- Junction-to-case thermal resistance (θJC):
  - 10μm chip: <100 K/W
  - 30μm chip: <50 K/W
  - 50μm chip: <20 K/W

### 5.2 Operating Temperature

- Junction temperature range: -40°C to +125°C
- Storage temperature: -55°C to +150°C
- Peak temperature during processing: <300°C

## 6. Reliability

### 6.1 Lifetime

- L70 lifetime: >50,000 hours at rated current and Tj=85°C
- L50 lifetime: >100,000 hours at rated current and Tj=85°C

Where LXX = time to XX% of initial luminous flux

### 6.2 Environmental Testing

- Temperature cycling: -40°C to +125°C, 1000 cycles minimum
- Humidity: 85°C/85% RH, 1000 hours
- Thermal shock: -40°C to +125°C, <5 minute transfer time, 100 cycles

### 6.3 ESD Protection

- Human Body Model (HBM): >2kV
- Machine Model (MM): >200V
- Charged Device Model (CDM): >500V

## 7. Defect Classification

### 7.1 Critical Defects (Zero tolerance)

- Cracked or broken chips
- Missing electrodes
- Short circuits
- Incorrect wavelength (>±10nm from specification)

### 7.2 Major Defects (<100 ppm)

- Low light output (<80% of mean)
- High forward voltage (>+0.5V from mean)
- Cosmetic defects visible at 10× magnification

### 7.3 Minor Defects (<1000 ppm)

- Slight wavelength variation (±5-10nm from specification)
- Brightness variation (80-90% of mean)
- Minor cosmetic defects

## 8. Packaging and Transfer Compatibility

### 8.1 Substrate Requirements

- Substrate material: Sapphire, SiC, GaN, or Silicon
- Substrate thickness: 100-400μm (prior to transfer)
- Surface flatness: <10μm across wafer

### 8.2 Release Layer

- Compatible with laser lift-off (LLO) or chemical release
- LLO wavelength: 248nm or 355nm
- Release layer uniformity: <±5% thickness variation

### 8.3 Contact Metallization

- N-contact metal: Ti/Al/Ti/Au or equivalent
- P-contact metal: ITO + Ni/Au or equivalent
- Contact pad size: >50% of chip area for good heat dissipation
- Pad coplanarity: <±1μm

## 9. Testing and Characterization

### 9.1 Optical Testing

- Measurement current: As specified for application (typically 20-100μA)
- Measurement geometry: Integrating sphere or calibrated photodetector
- Spectral measurement: Resolution <2nm

### 9.2 Electrical Testing

- I-V measurement: 0-5V forward, 0-10V reverse
- Current range: 1nA to 10mA
- Measurement accuracy: ±2%

### 9.3 Visual Inspection

- Magnification: Minimum 50× for chips <30μm
- Illumination: Bright field and dark field
- Defect detection: >5μm features

## 10. Quality Assurance

### 10.1 Sampling

- Lot size: Defined by manufacturer (typically 1 wafer or 1 day production)
- Sample size: Per MIL-STD-105E or equivalent
- AQL (Acceptable Quality Level): 0.65% for critical parameters

### 10.2 Binning

- Wavelength bins: Maximum 5nm width
- Brightness bins: Maximum 20% width
- Minimum 3 bins per color to ensure display uniformity

### 10.3 Documentation

Each lot shall include:
- Wafer map with bin assignments
- Electrical and optical test data
- Yield statistics
- Traceability information

## 11. Environmental and Safety

### 11.1 Material Restrictions

- RoHS compliant
- REACH compliant
- Halogen-free preferred
- Cadmium-free quantum dots (if used)

### 11.2 Handling Precautions

- ESD precautions required (Class 1 or better)
- Cleanroom environment: Class 1000 or better
- Moisture sensitivity level: MSL 3 or better

## 12. References

- IEC 60747-5: LED Standards
- JESD625: Requirements for Handling Electrostatic Discharge Sensitive Devices
- MIL-STD-883: Test Methods for Semiconductor Devices
- SEMI Standards: Semiconductor Equipment and Materials

## Appendix A: Test Methods

### A.1 Forward Voltage Measurement

```
Equipment: Source measure unit (SMU)
Current: 100 μA ± 1%
Compliance: 5V
Settling time: 100ms
Temperature: 25°C ± 2°C
```

### A.2 Light Output Measurement

```
Equipment: Integrating sphere + spectrometer
Current: As specified (typically 100 μA)
Integration time: 100ms minimum
Calibration: NIST-traceable standard
Temperature: 25°C ± 2°C
```

### A.3 Wavelength Measurement

```
Equipment: Spectrometer
Resolution: <2nm
Wavelength range: 380-780nm
Accuracy: ±0.5nm
Calibration: Standard emission lines
```

## Appendix B: Calculation Methods

### B.1 External Quantum Efficiency

```
EQE = (Optical Power Output / Photon Energy) / (Input Current / Elementary Charge)
    = (P_optical [W] / (h × c / λ)) / (I [A] / e)

Where:
h = Planck's constant (6.626×10⁻³⁴ J·s)
c = Speed of light (3×10⁸ m/s)
λ = Peak wavelength [m]
e = Elementary charge (1.602×10⁻¹⁹ C)
```

### B.2 Luminous Efficacy

```
Luminous Efficacy [lm/W] = Luminous Flux [lm] / Electrical Power [W]
```

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity

WIA-SEMI-010 v1.0 | Page 1 of 1
