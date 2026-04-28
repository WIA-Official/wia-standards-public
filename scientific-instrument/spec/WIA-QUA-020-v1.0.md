# WIA-QUA-020: Scientific Instrument Standard
# Version 1.0.0

**Standard ID:** WIA-QUA-020
**Title:** Scientific Instrument
**Category:** QUA (미래기술/양자/물리)
**Status:** Active
**Published:** 2025-01-01
**Updated:** 2025-01-01

---

## Abstract

This specification defines a comprehensive standard for advanced scientific instruments used in research, discovery, and innovation. The WIA-QUA-020 standard covers particle accelerators, mass spectrometers, electron microscopes, X-ray crystallography systems, NMR spectrometers, gravitational wave detectors, telescopes, spectrophotometers, chromatography systems, calorimeters, data acquisition systems, and calibration standards.

**弘益人間 (Benefit All Humanity)** - This standard facilitates global collaboration, data sharing, and reproducibility in scientific research.

---

## 1. Introduction

### 1.1 Purpose

The purpose of this standard is to:
- Define common interfaces for scientific instruments
- Establish data formats for instrument output
- Specify calibration and quality assurance procedures
- Enable instrument interoperability and automation
- Facilitate remote operation and data sharing

### 1.2 Scope

This standard applies to:
- Particle physics instrumentation
- Analytical chemistry instruments
- Electron and optical microscopy
- Spectroscopic techniques
- Astronomical observatories
- Chromatographic separations
- Thermal analysis equipment
- Data acquisition and control systems

### 1.3 Definitions

- **Instrument**: A device for measuring, analyzing, or observing physical phenomena
- **Detector**: A component that converts physical signals to electrical signals
- **Resolution**: The minimum distinguishable difference between two measurements
- **Sensitivity**: The minimum detectable signal above noise
- **Calibration**: The process of establishing accuracy against known standards
- **Dynamic Range**: The ratio of maximum to minimum measurable signal

---

## 2. Instrument Categories

### 2.1 Particle Accelerators

#### 2.1.1 Synchrotron

A synchrotron accelerates charged particles in a circular path using synchronized RF fields and magnetic bending.

**Key Parameters:**
- Beam energy: 1-10 GeV
- Circumference: 100-500 m
- Magnetic field: 1-2 Tesla
- RF frequency: 100-500 MHz
- Beam current: 100-500 mA

**Applications:**
- Synchrotron radiation sources
- X-ray crystallography
- Material characterization
- Medical imaging

**Physics:**
```
E = γ·m₀·c²
γ = 1/√(1 - v²/c²)
```

Where:
- E = Total energy
- γ = Lorentz factor
- m₀ = Rest mass
- c = Speed of light
- v = Particle velocity

#### 2.1.2 Large Hadron Collider (LHC)

The LHC accelerates protons to 6.5 TeV per beam in a 27 km ring.

**Specifications:**
- Beam energy: 6.5 TeV (13 TeV collision)
- Circumference: 26,659 m
- Magnetic field: 8.3 Tesla (dipoles)
- Number of bunches: 2,808
- Bunch spacing: 25 ns
- Luminosity: 10³⁴ cm⁻²s⁻¹

**Detectors:**
- ATLAS: General-purpose detector
- CMS: Compact Muon Solenoid
- ALICE: Heavy-ion physics
- LHCb: B-meson physics

### 2.2 Mass Spectrometers

#### 2.2.1 Orbitrap Mass Spectrometer

The Orbitrap uses electrostatic trapping with image current detection.

**Principle:**
Ions orbit around a central electrode with frequency:
```
ω = √(k/m)
```

Where:
- ω = Angular frequency
- k = Force constant (electrode geometry)
- m = Ion mass

**Specifications:**
- Resolution: 240,000 (FWHM at m/z 200)
- Mass accuracy: <1 ppm
- Mass range: 50-6,000 m/z
- Scan rate: 12 Hz
- Dynamic range: >5,000

**Applications:**
- Proteomics and protein identification
- Metabolomics profiling
- Small molecule analysis
- Lipidomics
- Top-down proteomics

#### 2.2.2 Time-of-Flight Mass Spectrometer (TOF-MS)

TOF-MS measures ion arrival time after acceleration.

**Equation:**
```
t = L·√(m/z) / √(2·e·V)
```

Where:
- t = Flight time
- L = Flight path length
- m/z = Mass-to-charge ratio
- e = Elementary charge
- V = Acceleration voltage

**Specifications:**
- Resolution: 40,000
- Mass accuracy: 5 ppm
- Mass range: 50-10,000 m/z
- Acquisition rate: 100 kHz

### 2.3 Electron Microscopes

#### 2.3.1 Transmission Electron Microscope (TEM)

TEM transmits electrons through ultra-thin specimens.

**Resolution Limit:**
```
d = 0.61·λ / NA
λ = h / √(2·m·e·V)
```

Where:
- d = Resolution
- λ = Electron wavelength
- NA = Numerical aperture
- h = Planck constant
- m = Electron mass
- V = Acceleration voltage

**Specifications:**
- Acceleration voltage: 80-300 kV
- Point resolution: 0.05 nm (aberration-corrected)
- Line resolution: 0.1 nm
- Magnification: 50× to 1,000,000×
- Tilt range: ±70°

**Operating Modes:**
- Bright-field imaging
- Dark-field imaging
- Diffraction mode
- High-resolution TEM (HRTEM)
- Scanning TEM (STEM)

#### 2.3.2 Scanning Electron Microscope (SEM)

SEM scans a focused electron beam across a surface.

**Specifications:**
- Acceleration voltage: 0.1-30 kV
- Resolution: 1 nm (at 30 kV)
- Magnification: 10× to 1,000,000×
- Vacuum: 10⁻⁵ to 10⁻⁶ Torr

**Detectors:**
- Secondary electron (SE) detector
- Backscattered electron (BSE) detector
- Energy-dispersive X-ray (EDX)
- Cathodoluminescence (CL)

### 2.4 X-ray Crystallography

#### 2.4.1 Single Crystal X-ray Diffraction

**Bragg's Law:**
```
n·λ = 2·d·sin(θ)
```

Where:
- n = Order of reflection
- λ = X-ray wavelength
- d = Lattice spacing
- θ = Angle of incidence

**Specifications:**
- X-ray source: Cu Kα (1.5418 Å), Mo Kα (0.7107 Å)
- Detector: CCD, pixel array
- Resolution: 0.5-1.0 Å
- Temperature range: 100-400 K

**Applications:**
- Small molecule structure
- Protein crystallography
- Material science

### 2.5 Nuclear Magnetic Resonance (NMR) Spectrometer

#### 2.5.1 High-Field NMR

**Larmor Frequency:**
```
ω₀ = γ·B₀
```

Where:
- ω₀ = Larmor frequency
- γ = Gyromagnetic ratio
- B₀ = Magnetic field strength

**Specifications:**
- Field strength: 400-1000 MHz (¹H)
- Magnetic field: 9.4-23.5 Tesla
- Homogeneity: <0.1 ppb/cm³
- Stability: <0.1 ppb/hour

**Nuclei:**
- ¹H: 600 MHz (14.1 T)
- ¹³C: 150 MHz (14.1 T)
- ¹⁵N: 60 MHz (14.1 T)
- ³¹P: 243 MHz (14.1 T)

**Applications:**
- Protein structure determination
- Small molecule characterization
- Material science
- Metabolomics

### 2.6 Gravitational Wave Detectors

#### 2.6.1 LIGO (Laser Interferometer Gravitational-Wave Observatory)

**Principle:**
Michelson interferometer with 4 km arms detecting strain:
```
h = ΔL/L
```

Where:
- h = Strain amplitude
- ΔL = Arm length change
- L = Arm length (4 km)

**Specifications:**
- Arm length: 4 km
- Laser wavelength: 1064 nm
- Laser power: 200 W
- Sensitivity: 10⁻²³ strain
- Frequency range: 10-10,000 Hz

**Detected Events:**
- Binary black hole mergers
- Binary neutron star mergers
- Supernovae (predicted)

### 2.7 Telescopes

#### 2.7.1 Radio Telescope

**Angular Resolution:**
```
θ = 1.22·λ / D
```

Where:
- θ = Angular resolution (radians)
- λ = Wavelength
- D = Aperture diameter

**Example: Very Large Array (VLA)**
- Dishes: 27 × 25 m
- Frequencies: 1-50 GHz
- Baseline: 36 km
- Resolution: 0.04 arcsec (at 43 GHz)

#### 2.7.2 Optical Telescope

**Example: James Webb Space Telescope (JWST)**
- Primary mirror: 6.5 m (18 segments)
- Wavelength: 0.6-28.5 µm (infrared)
- Operating temperature: 50 K
- Location: L2 Lagrange point

**Instruments:**
- NIRCam: Near-infrared camera
- NIRSpec: Near-infrared spectrograph
- MIRI: Mid-infrared instrument
- FGS/NIRISS: Fine guidance sensor

### 2.8 Spectrophotometers

#### 2.8.1 UV-Visible Spectrophotometer

**Beer-Lambert Law:**
```
A = ε·c·l
```

Where:
- A = Absorbance
- ε = Molar absorptivity (L·mol⁻¹·cm⁻¹)
- c = Concentration (mol/L)
- l = Path length (cm)

**Specifications:**
- Wavelength range: 190-1100 nm
- Bandwidth: 0.5-2 nm
- Photometric range: -0.3 to 3 A
- Accuracy: ±0.002 A

#### 2.8.2 Raman Spectrometer

**Raman Shift:**
```
Δν̃ = 1/λ₀ - 1/λ₁
```

Where:
- Δν̃ = Raman shift (cm⁻¹)
- λ₀ = Laser wavelength
- λ₁ = Scattered light wavelength

**Specifications:**
- Laser: 532 nm, 785 nm
- Resolution: 1-5 cm⁻¹
- Range: 100-4000 cm⁻¹
- Sensitivity: 1 ppm (SERS)

### 2.9 Chromatography Systems

#### 2.9.1 High-Performance Liquid Chromatography (HPLC)

**Van Deemter Equation:**
```
H = A + B/u + C·u
```

Where:
- H = Height equivalent to theoretical plate (HETP)
- u = Linear velocity
- A = Eddy diffusion
- B = Longitudinal diffusion
- C = Mass transfer

**Specifications:**
- Pressure: 400 bar (6,000 psi)
- Flow rate: 0.1-5 mL/min
- Column: 2.1-4.6 mm × 50-250 mm
- Particle size: 1.7-5 µm

#### 2.9.2 Gas Chromatography-Mass Spectrometry (GC-MS)

**Specifications:**
- Temperature range: 40-450°C
- Injection volume: 0.1-2 µL
- Carrier gas: Helium, hydrogen
- MS resolution: 1000 (quadrupole)

### 2.10 Calorimeters

#### 2.10.1 Differential Scanning Calorimetry (DSC)

**Heat Flow:**
```
dH/dt = Cp·dT/dt
```

Where:
- dH/dt = Heat flow
- Cp = Heat capacity
- dT/dt = Heating rate

**Specifications:**
- Temperature range: -90 to 550°C
- Heating rate: 0.1-100°C/min
- Sensitivity: 0.04 µW
- Sample size: 0.5-20 mg

---

## 3. Data Acquisition Systems

### 3.1 Analog-to-Digital Conversion (ADC)

**Specifications:**
- Resolution: 12-24 bits
- Sampling rate: 1 kHz to 10 GHz
- Input range: ±10 V
- Accuracy: 0.01%

### 3.2 Signal Processing

**Digital Filtering:**
- Low-pass filters
- Band-pass filters
- Notch filters (50/60 Hz)
- Savitzky-Golay smoothing

**Fast Fourier Transform (FFT):**
```
X(k) = Σ x(n)·e^(-i·2π·k·n/N)
```

---

## 4. Calibration Standards

### 4.1 Mass Calibration

**Standards:**
- Caffeine (C₈H₁₀N₄O₂): 194.0804 m/z
- Reserpine (C₃₃H₄₀N₂O₉): 609.2812 m/z
- Polyethylene glycol (PEG)
- Polystyrene

### 4.2 Wavelength Calibration

**Standards:**
- Mercury lamp: 253.65, 365.01, 435.83 nm
- Neon lamp: 585.25, 640.22, 703.24 nm
- Laser lines: 532 nm, 632.8 nm, 1064 nm

### 4.3 Temperature Calibration

**Standards:**
- Indium: 156.6°C
- Tin: 231.9°C
- Lead: 327.5°C
- Zinc: 419.5°C

---

## 5. Data Formats

### 5.1 Mass Spectrometry Data

**mzML Format (XML-based):**
```xml
<spectrum id="scan=1" index="0">
  <cvParam name="ms level" value="1"/>
  <cvParam name="total ion current" value="1.234e6"/>
  <binaryDataArrayList count="2">
    <binaryDataArray encodedLength="...">
      <cvParam name="m/z array"/>
      <binary>...</binary>
    </binaryDataArray>
  </binaryDataArrayList>
</spectrum>
```

### 5.2 Microscopy Data

**OME-TIFF Format:**
```xml
<OME>
  <Image ID="Image:0">
    <Pixels DimensionOrder="XYZCT"
            SizeX="1024" SizeY="1024" SizeZ="100"
            Type="uint16"/>
  </Image>
</OME>
```

### 5.3 Crystallography Data

**CIF Format (Crystallographic Information File):**
```
_cell_length_a 10.123
_cell_length_b 12.456
_cell_length_c 15.789
_cell_angle_alpha 90.00
_cell_angle_beta 95.23
_cell_angle_gamma 90.00
```

---

## 6. Quality Assurance

### 6.1 Instrument Qualification

**IQ/OQ/PQ Protocols:**
- Installation Qualification (IQ)
- Operational Qualification (OQ)
- Performance Qualification (PQ)

### 6.2 Maintenance

**Preventive Maintenance:**
- Daily: Leak checks, calibration
- Weekly: Cleaning, alignment
- Monthly: Vacuum pump service
- Yearly: Full calibration, certification

### 6.3 Validation

**Method Validation Parameters:**
- Accuracy
- Precision (repeatability, reproducibility)
- Linearity
- Range
- Limit of detection (LOD)
- Limit of quantification (LOQ)
- Robustness

---

## 7. Remote Operation

### 7.1 Remote Control

**Protocols:**
- SSH for command-line access
- VNC/RDP for graphical interfaces
- REST API for programmatic control
- WebSocket for real-time data

### 7.2 Data Transfer

**Methods:**
- FTP/SFTP for large files
- rsync for synchronization
- Cloud storage (S3, Azure, GCP)
- GridFTP for high-performance transfer

---

## 8. Safety

### 8.1 Radiation Safety

**Protection:**
- Shielding (lead, concrete)
- Distance (inverse square law)
- Time minimization
- Monitoring (dosimeters, badges)

### 8.2 Electrical Safety

**Requirements:**
- Grounding
- Emergency shut-off
- Interlocks
- Isolation transformers

### 8.3 Cryogenic Safety

**Hazards:**
- Frostbite
- Asphyxiation (nitrogen, helium)
- Pressure buildup
- Embrittlement

---

## 9. API Specification

### 9.1 TypeScript SDK

See `api/typescript/` for full implementation.

### 9.2 REST API

**Base URL:** `https://api.wia-instrument.org/v1`

**Endpoints:**
- `GET /instruments` - List instruments
- `GET /instruments/{id}` - Get instrument details
- `POST /instruments/{id}/measure` - Start measurement
- `GET /instruments/{id}/status` - Get status
- `GET /instruments/{id}/data` - Download data

---

## 10. Compliance

### 10.1 Standards

This standard complies with:
- ISO/IEC 17025 (Testing and Calibration)
- ISO 9001 (Quality Management)
- FDA 21 CFR Part 11 (Electronic Records)
- GLP/GMP (Good Laboratory/Manufacturing Practice)

### 10.2 Certification

WIA certification available at: [cert.wiastandards.com](https://cert.wiastandards.com)

---

## 11. References

2. Dass, C. (2007). *Fundamentals of Contemporary Mass Spectrometry*. Wiley.
3. Williams, D.B. & Carter, C.B. (2009). *Transmission Electron Microscopy*. Springer.

---

## 12. Appendix

### A. Physical Constants

| Constant | Symbol | Value |
|----------|--------|-------|
| Speed of light | c | 299,792,458 m/s |
| Planck constant | h | 6.62607015×10⁻³⁴ J·s |
| Elementary charge | e | 1.602176634×10⁻¹⁹ C |
| Electron mass | mₑ | 9.1093837015×10⁻³¹ kg |
| Proton mass | mₚ | 1.67262192369×10⁻²⁷ kg |

### B. Unit Conversions

| From | To | Factor |
|------|-----|--------|
| eV | J | 1.602176634×10⁻¹⁹ |
| Å | m | 1×10⁻¹⁰ |
| amu | kg | 1.66053906660×10⁻²⁷ |
| bar | Pa | 1×10⁵ |

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*© 2025 SmileStory Inc. / WIA*
*MIT License*
