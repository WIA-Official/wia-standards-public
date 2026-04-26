# WIA-QUA-009: Photonics Specification v1.0

> **Standard ID:** WIA-QUA-009
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Quantum Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Photon Physics](#2-photon-physics)
3. [Optical Materials](#3-optical-materials)
4. [Optical Fibers and Waveguides](#4-optical-fibers-and-waveguides)
5. [Laser Technology](#5-laser-technology)
6. [Light Sources (LED/OLED)](#6-light-sources-ledoled)
7. [Photodetectors](#7-photodetectors)
8. [Silicon Photonics](#8-silicon-photonics)
9. [Quantum Photonics](#9-quantum-photonics)
10. [Nonlinear Optics](#10-nonlinear-optics)
11. [Optical Computing](#11-optical-computing)
12. [LiDAR Technology](#12-lidar-technology)
13. [Photonic Crystals](#13-photonic-crystals)
14. [Implementation Guidelines](#14-implementation-guidelines)
15. [References](#15-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the comprehensive framework for photonics technology, encompassing fundamental photon physics, optical components, quantum optics, and applications in telecommunications, sensing, and computing.

### 1.2 Scope

The standard covers:
- Photon physics and wave-particle duality
- Optical materials and their properties
- Fiber optics and integrated photonics
- Laser systems and light sources
- Photodetection and imaging
- Quantum photonics and single photon sources
- Nonlinear optical processes
- Optical computing architectures
- LiDAR and sensing systems
- Photonic crystals and metamaterials

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to advance photonics technology for the betterment of humanity through improved communication, sensing, computing, and scientific understanding.

### 1.4 Terminology

- **Photon**: Quantum of electromagnetic radiation
- **Wavelength (λ)**: Spatial period of a wave
- **Frequency (ν)**: Number of oscillations per unit time
- **Refractive Index (n)**: Ratio of speed of light in vacuum to speed in medium
- **Numerical Aperture (NA)**: Measure of light-gathering ability
- **Mode**: Allowed electromagnetic field distribution in waveguide

---

## 2. Photon Physics

### 2.1 Wave-Particle Duality

Photons exhibit both wave and particle properties:

#### Wave Properties
```
λν = c
```

Where:
- `λ` = Wavelength (meters)
- `ν` = Frequency (Hz)
- `c` = Speed of light (299,792,458 m/s)

#### Particle Properties
```
E = hν = hc/λ
p = h/λ = E/c
```

Where:
- `E` = Photon energy (joules)
- `p` = Photon momentum (kg·m/s)
- `h` = Planck's constant (6.626 × 10⁻³⁴ J·s)

### 2.2 Energy and Wavelength Relationship

For common wavelengths:

| Application | Wavelength | Photon Energy | Frequency |
|-------------|------------|---------------|-----------|
| UV Lithography | 193 nm | 6.42 eV | 1.55 × 10¹⁵ Hz |
| Violet Light | 400 nm | 3.10 eV | 7.50 × 10¹⁴ Hz |
| Green Laser | 532 nm | 2.33 eV | 5.64 × 10¹⁴ Hz |
| Red Light | 650 nm | 1.91 eV | 4.61 × 10¹⁴ Hz |
| Telecom (O-band) | 1310 nm | 0.95 eV | 2.29 × 10¹⁴ Hz |
| Telecom (C-band) | 1550 nm | 0.80 eV | 1.93 × 10¹⁴ Hz |

### 2.3 Polarization

Polarization describes the orientation of the electric field:

#### Linear Polarization
```
E(z,t) = E₀ cos(kz - ωt) x̂
```

#### Circular Polarization
```
E(z,t) = E₀/√2 [cos(kz - ωt) x̂ ± sin(kz - ωt) ŷ]
```

#### Stokes Parameters
```
S₀ = I_total
S₁ = I_horizontal - I_vertical
S₂ = I_+45° - I_-45°
S₃ = I_RCP - I_LCP
```

### 2.4 Optical Power and Intensity

```
P = N × hν
I = P/A
```

Where:
- `P` = Power (watts)
- `N` = Photon flux (photons/s)
- `I` = Intensity (W/m²)
- `A` = Beam cross-sectional area (m²)

---

## 3. Optical Materials

### 3.1 Refractive Index

The refractive index determines how light propagates in a material:

```
n = c/v
```

Where:
- `n` = Refractive index (dimensionless)
- `v` = Phase velocity in medium (m/s)

### 3.2 Sellmeier Equation

Wavelength-dependent refractive index:

```
n²(λ) = 1 + B₁λ²/(λ² - C₁) + B₂λ²/(λ² - C₂) + B₃λ²/(λ² - C₃)
```

#### Example: Fused Silica (SiO₂)
- B₁ = 0.6961663
- B₂ = 0.4079426
- B₃ = 0.8974794
- C₁ = 0.0684043 μm²
- C₂ = 0.1162414 μm²
- C₃ = 9.896161 μm²

### 3.3 Common Optical Materials

| Material | n @ 1550nm | Applications |
|----------|------------|--------------|
| Fused Silica | 1.444 | Optical fibers, lenses |
| BK7 Glass | 1.503 | General optics |
| Sapphire | 1.754 | Windows, substrates |
| Silicon | 3.48 | Silicon photonics |
| Silicon Nitride | 2.00 | Integrated waveguides |
| GaAs | 3.37 | Lasers, detectors |
| InP | 3.17 | Telecom lasers |
| LiNbO₃ | 2.21 | Modulators, nonlinear optics |

### 3.4 Dispersion

Chromatic dispersion causes pulse broadening:

```
D = -(λ/c) × d²n/dλ²
```

Typical fiber dispersion:
- **Zero dispersion wavelength**: ~1310 nm
- **C-band (1550 nm)**: +17 ps/(nm·km)

---

## 4. Optical Fibers and Waveguides

### 4.1 Optical Fiber Fundamentals

#### Numerical Aperture
```
NA = √(n₁² - n₂²) ≈ n₁√(2Δ)
```

Where:
- `n₁` = Core refractive index
- `n₂` = Cladding refractive index
- `Δ` = Relative index difference = (n₁ - n₂)/n₁

#### V-Number (Normalized Frequency)
```
V = (2πa/λ) × NA
```

Where:
- `a` = Core radius
- `λ` = Wavelength

**Single-mode condition**: V < 2.405

### 4.2 Mode Field Diameter

For single-mode fibers:

```
MFD = 2a(0.65 + 1.619V^(-3/2) + 2.879V^(-6))
```

### 4.3 Fiber Types

#### Single-Mode Fiber (SMF)
- Core diameter: 8-10 μm
- Cladding diameter: 125 μm
- Applications: Long-haul telecommunications

#### Multi-Mode Fiber (MMF)
- Core diameter: 50 or 62.5 μm
- Cladding diameter: 125 μm
- Applications: Short-distance data links

#### Specialty Fibers
- **Polarization-maintaining**: Maintains polarization state
- **Photonic crystal fiber**: Air-hole cladding
- **Hollow-core fiber**: Light in air core
- **Erbium-doped fiber**: Optical amplification

### 4.4 Fiber Attenuation

Typical attenuation in silica fibers:

| Wavelength | Attenuation | Primary Loss Mechanism |
|------------|-------------|------------------------|
| 850 nm | 2.5 dB/km | Rayleigh scattering |
| 1310 nm | 0.35 dB/km | OH absorption minimum |
| 1550 nm | 0.20 dB/km | Scattering minimum |

Total loss:
```
α_total = α_Rayleigh + α_absorption + α_bend + α_splice
```

### 4.5 Integrated Waveguides

#### Ridge Waveguide
```
n_eff = √[(βλ/2π)²]
```

Where:
- `n_eff` = Effective refractive index
- `β` = Propagation constant

#### Coupling Efficiency
```
η = |∫∫ E₁*E₂ dA|² / (∫∫|E₁|²dA × ∫∫|E₂|²dA)
```

---

## 5. Laser Technology

### 5.1 Laser Fundamentals

#### Population Inversion
For lasing to occur:
```
N₂ > N₁
```

Where N₂ and N₁ are populations of upper and lower energy states.

#### Threshold Condition
```
g × L ≥ α_total
```

Where:
- `g` = Gain coefficient
- `L` = Cavity length
- `α_total` = Total loss

### 5.2 Laser Cavity

#### Round-trip Gain
```
G_RT = R₁ × R₂ × exp(2gL - 2αL)
```

Where:
- `R₁, R₂` = Mirror reflectivities
- `g` = Gain coefficient
- `α` = Loss coefficient

#### Longitudinal Modes
```
ν_n = nc/2nL
```

Mode spacing:
```
Δν = c/2nL
```

### 5.3 Laser Types and Characteristics

#### Diode Lasers
- **Structure**: p-n junction
- **Wavelengths**: 405-1550 nm
- **Power**: μW to W range
- **Efficiency**: 30-50%
- **Applications**: Telecommunications, displays, sensors

#### Solid-State Lasers
- **Example**: Nd:YAG (1064 nm)
- **Pumping**: Optical (flash lamp or diode)
- **Power**: mW to kW
- **Applications**: Material processing, medical

#### Fiber Lasers
- **Gain medium**: Rare-earth doped fiber
- **Power**: W to kW
- **Beam quality**: Excellent (M² ~ 1.1)
- **Applications**: Material processing, marking

#### Gas Lasers
- **Example**: CO₂ (10.6 μm), He-Ne (632.8 nm)
- **Power**: mW to kW (CO₂)
- **Applications**: Cutting, welding, research

### 5.4 Laser Parameters

#### Beam Quality (M²)
```
M² = (π × w₀ × θ) / λ
```

Where:
- `w₀` = Beam waist radius
- `θ` = Divergence half-angle

**Ideal Gaussian beam**: M² = 1

#### Linewidth
```
Δν = c × Δλ / λ²
```

**Typical linewidths**:
- Gas laser: 1-100 MHz
- Diode laser (free-running): 10-100 MHz
- External cavity diode: 100 kHz
- Fiber laser: 1-10 kHz

---

## 6. Light Sources (LED/OLED)

### 6.1 LED Fundamentals

#### Radiative Recombination
```
E_photon = E_gap = hc/λ
```

For direct bandgap semiconductors:
- GaN (blue): 2.9 eV → 427 nm
- GaAs (IR): 1.43 eV → 867 nm

### 6.2 LED Efficiency

#### External Quantum Efficiency
```
η_ext = η_IQE × η_extraction
```

Where:
- `η_IQE` = Internal quantum efficiency
- `η_extraction` = Light extraction efficiency

#### Luminous Efficacy
```
LE = Φ_v / P_electrical
```

Units: lumens per watt (lm/W)

**Typical values**:
- Incandescent: 10-20 lm/W
- Fluorescent: 50-100 lm/W
- White LED: 80-150 lm/W
- High-efficiency LED: 150-200 lm/W

### 6.3 OLED Technology

**Structure**:
1. Anode (ITO)
2. Hole transport layer
3. Emissive layer
4. Electron transport layer
5. Cathode

**Advantages**:
- Thin, flexible
- Wide viewing angle
- High contrast
- Fast response

---

## 7. Photodetectors

### 7.1 Photodetection Mechanisms

#### Photoelectric Effect
```
E_photon = hν ≥ E_gap
```

#### Responsivity
```
R = I_photo / P_optical
```

Units: A/W

**Theoretical maximum**:
```
R_max = (λ × e) / (h × c) = λ(μm) / 1.24
```

### 7.2 Quantum Efficiency

```
QE = (R × h × c) / (e × λ)
```

For 100% QE:
- λ = 850 nm → R = 0.68 A/W
- λ = 1550 nm → R = 1.25 A/W

### 7.3 Detector Types

#### PIN Photodiode
- **Structure**: p-intrinsic-n
- **Bandwidth**: DC to 100 GHz
- **Responsivity**: 0.5-1.0 A/W
- **Applications**: Optical communications

#### Avalanche Photodiode (APD)
- **Gain**: 10-100
- **Bandwidth**: DC to 40 GHz
- **Noise**: Excess noise from multiplication
- **Applications**: Long-haul receivers

#### Single Photon Avalanche Diode (SPAD)
- **Operation**: Geiger mode
- **Sensitivity**: Single photon
- **Timing**: ps resolution
- **Applications**: Quantum communication, LiDAR

#### Photomultiplier Tube (PMT)
- **Gain**: 10⁶-10⁸
- **Spectral range**: UV-NIR
- **Applications**: Low-light detection

### 7.4 Noise in Photodetectors

#### Shot Noise
```
i_shot² = 2eI_photo B
```

#### Thermal Noise
```
i_thermal² = (4kTB) / R_L
```

#### Total Noise
```
NEP = √(i_total²) / R
```

Units: W/√Hz

---

## 8. Silicon Photonics

### 8.1 Silicon Platform

**Advantages**:
- CMOS compatibility
- High refractive index contrast (Si: 3.48, SiO₂: 1.45)
- Tight mode confinement
- Mature fabrication

**Wavelength range**: 1.1-4 μm (transparent)

### 8.2 Silicon Waveguides

#### Strip Waveguide
- Width: 400-500 nm
- Height: 220 nm
- Bend radius: 1-5 μm
- Loss: <1 dB/cm

#### Rib Waveguide
- Width: 1-3 μm
- Etch depth: 70-150 nm
- Bend radius: 5-50 μm
- Loss: <0.1 dB/cm

### 8.3 Silicon Photonic Components

#### Mach-Zehnder Modulator
```
T = cos²(πV/V_π)
```

Where:
- `V` = Applied voltage
- `V_π` = Voltage for π phase shift

**Typical V_π**: 2-6 V
**Bandwidth**: 50+ GHz

#### Ring Resonator
```
FSR = c / (2πRn_eff)
Q = λ₀ / Δλ
```

Where:
- `R` = Ring radius
- `FSR` = Free spectral range
- `Q` = Quality factor

#### Grating Coupler
- **Coupling efficiency**: 30-70%
- **Bandwidth**: 40-80 nm
- **Purpose**: Fiber-chip coupling

### 8.4 Heterogeneous Integration

**III-V on Silicon**:
- Lasers (InP, GaAs)
- Optical amplifiers
- High-speed modulators

**Bonding methods**:
- Direct bonding
- Adhesive bonding
- Molecular bonding

---

## 9. Quantum Photonics

### 9.1 Single Photon Sources

#### Spontaneous Parametric Down-Conversion (SPDC)
```
ω_p = ω_s + ω_i
k_p = k_s + k_i
```

**Heralded single photons**: Detection of idler heralds signal

#### Quantum Dots
- **Emission**: Single photons on demand
- **Purity**: g²(0) < 0.01
- **Indistinguishability**: >90%
- **Wavelength**: 900-1550 nm

#### NV Centers in Diamond
- **Wavelength**: 637 nm
- **Coherence time**: μs-ms
- **Temperature**: Room temperature operation
- **Applications**: Quantum sensing, computing

### 9.2 Single Photon Characterization

#### Second-order Correlation Function
```
g²(τ) = <I(t)I(t+τ)> / <I(t)>²
```

For single photons:
```
g²(0) < 0.5
```

Perfect single photon source: g²(0) = 0

#### Hong-Ou-Mandel Interference
Visibility:
```
V = (C_max - C_min) / (C_max + C_min)
```

Indistinguishability: V → 1

### 9.3 Quantum Entanglement

#### Bell States
```
|Φ⁺⟩ = (|HH⟩ + |VV⟩) / √2
|Φ⁻⟩ = (|HH⟩ - |VV⟩) / √2
|Ψ⁺⟩ = (|HV⟩ + |VH⟩) / √2
|Ψ⁻⟩ = (|HV⟩ - |VH⟩) / √2
```

#### Bell Inequality
```
S = |E(a,b) - E(a,b') + E(a',b) + E(a',b')| ≤ 2 (classical)
S_max = 2√2 (quantum)
```

---

## 10. Nonlinear Optics

### 10.1 Nonlinear Polarization

```
P = ε₀(χ⁽¹⁾E + χ⁽²⁾E² + χ⁽³⁾E³ + ...)
```

Where:
- `χ⁽¹⁾` = Linear susceptibility
- `χ⁽²⁾` = Second-order nonlinearity
- `χ⁽³⁾` = Third-order nonlinearity

### 10.2 Second Harmonic Generation (SHG)

```
ω₃ = 2ω₁
```

**Phase matching condition**:
```
Δk = k₃ - 2k₁ = 0
```

**Efficiency**:
```
η_SHG ∝ (d_eff)² × P_pump × L²
```

### 10.3 Four-Wave Mixing (FWM)

```
ω₄ = ω₁ + ω₂ - ω₃
```

**Applications**:
- Wavelength conversion
- Optical parametric amplification
- Supercontinuum generation

### 10.4 Self-Phase Modulation (SPM)

```
φ_NL = n₂ × I × L × 2π/λ
```

Where:
- `n₂` = Nonlinear refractive index
- `I` = Intensity
- `L` = Propagation length

**Effect**: Spectral broadening

### 10.5 Nonlinear Materials

| Material | χ⁽²⁾ (pm/V) | n₂ (m²/W) | Applications |
|----------|-------------|-----------|--------------|
| LiNbO₃ | 27 | 5×10⁻²⁰ | SHG, OPO |
| BBO | 2.2 | - | UV generation |
| KTP | 3.3 | - | Green lasers |
| Silica fiber | - | 2.6×10⁻²⁰ | FWM, SPM |
| Silicon | - | 4.5×10⁻¹⁸ | On-chip NL optics |

---

## 11. Optical Computing

### 11.1 Optical Logic Gates

#### All-Optical AND Gate
Uses nonlinear interferometer:
- **Switching energy**: fJ range
- **Speed**: ps switching time
- **Contrast**: 10-20 dB

#### Optical Flip-Flop
Based on coupled ring resonators:
- **Bistability**: Two stable states
- **Set/Reset**: Optical pulses

### 11.2 Optical Interconnects

**Advantages over electrical**:
- Higher bandwidth: Tbps links
- Lower latency: ~5 ps/mm (vs 50 ps/mm electrical)
- Lower power: pJ/bit (vs nJ/bit)
- No crosstalk

#### Data Rate Evolution
- 2020: 100 Gbps per λ
- 2025: 400 Gbps per λ
- 2030: 1.6 Tbps per λ (projected)

### 11.3 Neuromorphic Photonics

**Optical neurons**:
- Activation function: Nonlinear transfer
- Weight: Optical attenuation
- Summation: Optical interference

**Advantages**:
- Massive parallelism
- Low latency
- Energy efficiency

---

## 12. LiDAR Technology

### 12.1 LiDAR Equation

```
P_r = (P_t × ρ × A_r × η_sys) / (4π × R²)
```

Where:
- `P_r` = Received power
- `P_t` = Transmitted power
- `ρ` = Target reflectivity
- `A_r` = Receiver aperture area
- `R` = Range
- `η_sys` = System efficiency

### 12.2 Range Resolution

```
Δr = c × τ / 2
```

Where:
- `τ` = Pulse duration

**Typical**: 1-10 cm resolution

### 12.3 LiDAR Types

#### Time-of-Flight (ToF)
- **Range**: 100-300 m
- **Accuracy**: ±2-5 cm
- **Applications**: Automotive, surveying

#### Frequency-Modulated Continuous Wave (FMCW)
- **Range**: 200+ m
- **Velocity**: Direct measurement
- **Advantages**: Coherent detection, velocity

#### Flash LiDAR
- **Type**: Full-frame illumination
- **Frame rate**: 10-100 Hz
- **Applications**: Close-range 3D imaging

### 12.4 Automotive LiDAR Requirements

| Parameter | Requirement |
|-----------|-------------|
| Range | 200-300 m |
| Resolution | 0.1° horizontal |
| Frame rate | 10-20 Hz |
| Point rate | 1-4 Mpts/s |
| Eye safety | Class 1 laser |
| Wavelength | 850-950 nm or 1550 nm |

---

## 13. Photonic Crystals

### 13.1 Photonic Band Gap

Analogous to electronic band gap:

```
ω = ω(k)
```

**Band gap**: Range of frequencies with no propagating modes

### 13.2 1D Photonic Crystal

**Bragg grating**:
```
λ_Bragg = 2n_eff Λ
```

Where:
- `Λ` = Grating period
- `n_eff` = Effective index

### 13.3 2D/3D Photonic Crystals

#### 2D Hexagonal Lattice
- **Lattice constant**: a
- **Air-filling fraction**: Determines band gap
- **Applications**: Waveguides, cavities

#### 3D Woodpile Structure
- **Complete band gap**: All directions
- **Fabrication**: Layer-by-layer

### 13.4 Photonic Crystal Cavities

#### Quality Factor
```
Q = ω₀ / Δω = ω₀ × τ
```

**Ultra-high Q**: >10⁶

#### Mode Volume
```
V = ∫ε|E|²dV / max(ε|E|²)
```

**Small V**: Sub-wavelength confinement

**Applications**:
- Cavity QED
- Nonlinear optics
- Optomechanics

---

## 14. Implementation Guidelines

### 14.1 Design Considerations

#### Wavelength Selection
1. **Telecommunications**: 1310 nm (O-band), 1550 nm (C-band)
2. **Sensing**: 850 nm, 1064 nm
3. **Material processing**: 1070 nm (fiber), 10.6 μm (CO₂)
4. **Medical**: 532 nm, 1064 nm, 10.6 μm

#### Power Budget
```
P_received = P_transmitted - Loss_total + Gain_total
```

Include:
- Fiber/waveguide loss
- Connector loss
- Coupling loss
- Splitter loss
- Amplifier gain

### 14.2 Safety Standards

#### Laser Safety Classes (IEC 60825-1)

| Class | AEL (400-700 nm) | Description |
|-------|------------------|-------------|
| 1 | 0.39 mW | Eye-safe |
| 1M | 0.39 mW/cm² | Safe with naked eye |
| 2 | 1 mW | Visible, blink reflex |
| 3R | 5 mW | Low risk |
| 3B | 500 mW | Direct viewing hazard |
| 4 | >500 mW | Skin and fire hazard |

### 14.3 Testing and Characterization

#### Optical Power Measurement
- Calibrated power meter
- Wavelength-specific responsivity

#### Spectral Analysis
- Optical spectrum analyzer (OSA)
- Resolution: 0.01-1 nm

#### Temporal Characterization
- High-speed photodetector
- Oscilloscope or sampling scope

### 14.4 Environmental Considerations

#### Temperature Effects
```
Δλ/ΔT ≈ 0.1 nm/°C (typical laser)
```

#### Humidity
- Hygroscopic coatings
- Hermetic sealing for critical components

---

## 15. References

### 15.1 Standards

- ITU-T G.652: Single-mode optical fiber
- ITU-T G.657: Bend-insensitive fiber
- IEEE 802.3: Ethernet optical interfaces
- IEC 60825-1: Laser safety

### 15.2 Textbooks

- Saleh & Teich, "Fundamentals of Photonics"
- Yariv & Yeh, "Photonics: Optical Electronics"
- Siegman, "Lasers"
- Boyd, "Nonlinear Optics"

### 15.3 Online Resources

- OSA (Optica): Professional society
- SPIE: Photonics conferences
- arXiv.org: Quantum optics preprints
- RP Photonics Encyclopedia

---

## Appendix A: Common Wavelengths

| Wavelength | Color/Band | Common Source | Application |
|------------|------------|---------------|-------------|
| 193 nm | Deep UV | ArF excimer | Photolithography |
| 248 nm | UV | KrF excimer | Lithography |
| 266 nm | UV | Nd:YAG (4×) | Spectroscopy |
| 355 nm | UV | Nd:YAG (3×) | Material processing |
| 405 nm | Violet | GaN diode | Blu-ray, curing |
| 450 nm | Blue | GaN diode | Displays |
| 473 nm | Blue | Nd:YAG (2×) | Display |
| 532 nm | Green | Nd:YAG (2×) | Laser pointer, display |
| 633 nm | Red | He-Ne | Metrology |
| 650 nm | Red | AlGaInP diode | DVD, pointer |
| 780 nm | NIR | GaAlAs diode | CD player |
| 808 nm | NIR | AlGaAs diode | Pumping |
| 850 nm | NIR | GaAs diode | Datacom |
| 905 nm | NIR | GaAs diode | LiDAR |
| 980 nm | NIR | InGaAs diode | Fiber amplifier pump |
| 1064 nm | NIR | Nd:YAG | Material processing |
| 1310 nm | NIR | InGaAsP | Telecom (O-band) |
| 1550 nm | NIR | InGaAsP | Telecom (C-band) |
| 10.6 μm | LWIR | CO₂ | Cutting, welding |

---

## Appendix B: Unit Conversions

### Energy
- 1 eV = 1.602 × 10⁻¹⁹ J
- 1 eV ↔ 1240 nm (wavelength)

### Wavelength
- 1 nm = 10⁻⁹ m
- 1 μm = 10⁻⁶ m
- 1 Å = 10⁻¹⁰ m

### Power
- 0 dBm = 1 mW
- 10 dBm = 10 mW
- 20 dBm = 100 mW
- 30 dBm = 1 W

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
