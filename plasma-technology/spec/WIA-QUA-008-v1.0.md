# WIA-QUA-008: Plasma Technology Specification v1.0

> **Standard ID:** WIA-QUA-008
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Plasma Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Plasma Physics Fundamentals](#2-plasma-physics-fundamentals)
3. [Plasma Parameters & Diagnostics](#3-plasma-parameters--diagnostics)
4. [Plasma Generation Methods](#4-plasma-generation-methods)
5. [Nuclear Fusion Plasma](#5-nuclear-fusion-plasma)
6. [Plasma Processing](#6-plasma-processing)
7. [Plasma Medicine](#7-plasma-medicine)
8. [Plasma Propulsion](#8-plasma-propulsion)
9. [Magnetohydrodynamics (MHD)](#9-magnetohydrodynamics-mhd)
10. [Implementation Guidelines](#10-implementation-guidelines)
11. [Safety & Standards](#11-safety--standards)
12. [References](#12-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines comprehensive standards for plasma technology, covering fundamental physics, generation methods, industrial applications, medical uses, and advanced propulsion systems.

### 1.2 Scope

The standard covers:
- Plasma physics theory and parameters
- Thermal and non-thermal plasma systems
- Plasma generation techniques (RF, DC, microwave)
- Nuclear fusion plasma (tokamak, stellarator, ICF)
- Industrial plasma processing
- Medical plasma applications
- Plasma propulsion for spacecraft
- Plasma diagnostics and control

### 1.3 Philosophy

**弘익인간 (Benefit All Humanity)** - This standard aims to advance plasma technology for clean energy, advanced manufacturing, medical therapy, and space exploration to benefit all of humanity.

### 1.4 Terminology

- **Plasma**: Fourth state of matter; ionized gas with free electrons and ions
- **Debye Length (λD)**: Characteristic shielding distance in plasma
- **Plasma Frequency (ωp)**: Natural oscillation frequency of electron density
- **Ionization Degree (α)**: Fraction of atoms/molecules that are ionized
- **Quasi-Neutrality**: ne ≈ ni (electron density ≈ ion density)
- **Collective Behavior**: Particle interactions via long-range EM fields

---

## 2. Plasma Physics Fundamentals

### 2.1 Definition of Plasma

A plasma is defined when three criteria are met:

#### 2.1.1 Debye Shielding

```
λD = √(ε₀kTe / nee²)
```

Where:
- `λD` = Debye length (m)
- `ε₀` = Vacuum permittivity (8.854 × 10⁻¹² F/m)
- `k` = Boltzmann constant (1.381 × 10⁻²³ J/K)
- `Te` = Electron temperature (K)
- `ne` = Electron density (m⁻³)
- `e` = Elementary charge (1.602 × 10⁻¹⁹ C)

**Plasma criterion**: λD << L (system size)

#### 2.1.2 Plasma Parameter

Number of particles in Debye sphere:

```
ΛD = ne × (4/3)π × λD³
```

**Plasma criterion**: ΛD >> 1 (many particles in Debye sphere)

#### 2.1.3 Plasma Frequency

```
ωpe = √(nee² / ε₀me)
ωpi = √(niZi²e² / ε₀mi)
```

Where:
- `ωpe` = Electron plasma frequency (rad/s)
- `ωpi` = Ion plasma frequency (rad/s)
- `me` = Electron mass (9.109 × 10⁻³¹ kg)
- `mi` = Ion mass (kg)
- `Zi` = Ion charge number

**Plasma criterion**: ω × τ > 1 (where τ is collision time)

### 2.2 Ionization Mechanisms

#### 2.2.1 Thermal Ionization

Saha equation for ionization equilibrium:

```
ni × ne / nn = (2πmekTe/h²)^(3/2) × 2Zi/Z₀ × exp(-Ei/kTe)
```

Where:
- `nn` = Neutral density
- `h` = Planck constant (6.626 × 10⁻³⁴ J·s)
- `Ei` = Ionization energy (eV)
- `Z` = Partition function

#### 2.2.2 Electron Impact Ionization

Cross-section for electron impact:

```
σi(E) = a₀² × (Ei/E) × ln(E/Ei)  (for E > Ei)
```

Where:
- `σi` = Ionization cross-section (m²)
- `a₀` = Bohr radius (5.29 × 10⁻¹¹ m)
- `E` = Electron energy (eV)

Ionization rate:

```
Ri = ne × nn × <σi × v>
```

#### 2.2.3 Photo-Ionization

```
hν ≥ Ei  (photon energy exceeds ionization potential)
```

---

## 3. Plasma Parameters & Diagnostics

### 3.1 Langmuir Probe

#### 3.1.1 I-V Characteristic

```
I(V) = Ie,sat × exp[e(V - Vp)/kTe]  (for V < Vp)
I(V) = Ii,sat                        (for V << Vf)
```

Where:
- `Vp` = Plasma potential
- `Vf` = Floating potential
- `Ie,sat` = Electron saturation current
- `Ii,sat` = Ion saturation current

#### 3.1.2 Electron Density

From ion saturation current:

```
ne = Ii,sat / (0.61 × e × A × √(kTe/mi))
```

Where:
- `A` = Probe area (m²)

#### 3.1.3 Electron Temperature

From electron current slope:

```
Te = e / [k × d(ln Ie)/dV]
```

### 3.2 Optical Emission Spectroscopy

#### 3.2.1 Line Intensity

```
Iλ = (hc/λ) × Aul × nu
```

Where:
- `Iλ` = Spectral line intensity
- `λ` = Wavelength
- `Aul` = Einstein coefficient
- `nu` = Upper state population density

#### 3.2.2 Actinometry

Gas temperature from Doppler broadening:

```
Δλ_D = (λ/c) × √(8kTg ln2/m)
```

### 3.3 Interferometry

Electron density from phase shift:

```
Δφ = (e²λ/2πε₀mec²) × ∫ ne dl
```

For microwave interferometry at 94 GHz:
```
ne = 1.24 × 10¹⁷ × Δφ/L  (m⁻³, L in meters)
```

---

## 4. Plasma Generation Methods

### 4.1 RF (Radio Frequency) Plasma

#### 4.1.1 Capacitively Coupled Plasma (CCP)

Power density:

```
P/V = 0.5 × ε₀ × ω² × E₀² × (νm/(νm² + ω²))
```

Where:
- `P/V` = Power per volume (W/m³)
- `ω` = RF angular frequency (rad/s)
- `E₀` = Electric field amplitude (V/m)
- `νm` = Collision frequency (s⁻¹)

Sheath voltage:

```
Vs ≈ Vrf × (A_electrode / A_grounded)^n
```

Where n ≈ 2-4 depending on frequency.

#### 4.1.2 Inductively Coupled Plasma (ICP)

Plasma density:

```
ne ≈ (Pabs × νm) / (ε₀ × ω² × λD² × kTe × V)
```

Skin depth:

```
δ = c / ωpe = √(ε₀mec² / nee²)
```

Standard frequencies:
- 13.56 MHz (ISM band)
- 27.12 MHz (ISM band)
- 40.68 MHz

### 4.2 DC Glow Discharge

#### 4.2.1 Paschen's Law

Breakdown voltage:

```
Vb = (B × p × d) / ln(A × p × d) - ln[ln(1 + 1/γ)]
```

Where:
- `p` = Pressure (Pa)
- `d` = Gap distance (m)
- `A, B` = Gas constants
- `γ` = Secondary emission coefficient

For argon: A = 12 Pa⁻¹m⁻¹, B = 180 V/(Pa·m)

#### 4.2.2 Current Density

Child-Langmuir law for space-charge limited current:

```
J = (4ε₀/9) × √(2e/mi) × V^(3/2) / d²
```

### 4.3 Microwave Plasma

#### 4.3.1 Electron Cyclotron Resonance (ECR)

Resonance condition:

```
ωce = eB/me = ω
```

At 2.45 GHz: B = 875 Gauss

#### 4.3.2 Power Absorption

Resonant absorption:

```
P = (ne × e² × E₀²) / (2me × νm) × V
```

---

## 5. Nuclear Fusion Plasma

### 5.1 Fusion Reactions

#### 5.1.1 D-T Reaction (Primary)

```
²H + ³H → ⁴He (3.5 MeV) + n (14.1 MeV)
```

Cross-section peak: ~5 barn at 64 keV

#### 5.1.2 Fusion Rate

```
<σv> ≈ 1.1 × 10⁻²⁴ T² (m³/s)  for T = 10-20 keV
```

Fusion power density:

```
Pfus = nD × nT × <σv> × Efus / 4
```

### 5.2 Tokamak Confinement

#### 5.2.1 Magnetic Field Configuration

Toroidal field:

```
Bt = μ₀ × N × I / (2πR)
```

Poloidal field:

```
Bp ≈ μ₀ × Ip / (2πa)
```

Where:
- `R` = Major radius (m)
- `a` = Minor radius (m)
- `Ip` = Plasma current (A)

#### 5.2.2 Safety Factor (q)

```
q = (r × Bt) / (R × Bp)
```

Stability requirement: q > 1 (avoid kink modes)

#### 5.2.3 Energy Confinement Time

ITER-89P scaling:

```
τE = 0.048 × Ip^0.85 × R^1.2 × a^0.3 × κ^0.5 × Bt^0.2 / (P^0.5 × n^0.1 × A^0.5)
```

Where:
- `κ` = Elongation
- `A` = Aspect ratio (R/a)
- `P` = Heating power (MW)

#### 5.2.4 Lawson Criterion

```
n × τE × T > 3 × 10²¹ keV·s/m³
```

For D-T fusion at T = 15 keV:
```
n × τE > 2 × 10²⁰ s/m³
```

### 5.3 Stellarator

#### 5.3.1 Rotational Transform

```
ι = 1/q = (2πa²/R) × (Bp/Bt)
```

External rotational transform from coils (no plasma current needed).

### 5.4 Inertial Confinement Fusion (ICF)

#### 5.4.1 Compression Ratio

```
ρ_final / ρ_initial ≈ 1000-10,000
```

#### 5.4.2 Required Energy

```
E_laser ≈ 1-2 MJ (for ignition)
```

NIF (National Ignition Facility): 1.9 MJ delivered to target

---

## 6. Plasma Processing

### 6.1 Plasma Etching

#### 6.1.1 Etch Rate

```
R_etch = (K × Γi × Yi × M) / (ρ × NA)
```

Where:
- `Γi` = Ion flux (m⁻²s⁻¹)
- `Yi` = Sputter yield (atoms/ion)
- `M` = Molecular weight (kg/mol)
- `ρ` = Material density (kg/m³)
- `NA` = Avogadro's number

#### 6.1.2 Reactive Ion Etching (RIE)

Aspect ratio:
```
AR = depth / width > 10 (typical)
```

Selectivity:
```
S = R_etch(material) / R_etch(mask)
```

### 6.2 Plasma Enhanced Chemical Vapor Deposition (PECVD)

#### 6.2.1 Deposition Rate

```
R_dep = k₀ × exp(-Ea/kTg) × [precursor]
```

Where:
- `Ea` = Activation energy
- `Tg` = Gas temperature

Typical rates: 10-1000 nm/min

#### 6.2.2 Film Properties

- Density: 2.0-2.3 g/cm³ (SiO₂)
- Refractive index: 1.46 (SiO₂)
- Stress: -300 to +300 MPa

---

## 7. Plasma Medicine

### 7.1 Cold Atmospheric Plasma (CAP)

#### 7.1.1 Reactive Species

Key species for biological effects:
- OH radicals
- O₂⁻, O₃
- NO, NO₂
- H₂O₂

#### 7.1.2 Plasma Parameters

- Electron temperature: 1-10 eV (11,600-116,000 K)
- Gas temperature: 300-350 K (near room temperature)
- Electron density: 10¹⁵-10¹⁸ m⁻³

#### 7.1.3 Medical Applications

**Wound Healing**:
- Treatment time: 30-180 seconds
- Distance: 5-15 mm
- Power: 1-10 W

**Sterilization**:
- Reduction: >6 log (99.9999%)
- Time: 30-300 seconds
- Effective against: bacteria, fungi, spores, viruses

**Cancer Therapy**:
- Apoptosis induction
- Selective cell death
- DNA damage in cancer cells

### 7.2 Safety Limits

- UV radiation: < 1 mW/cm²
- Ozone: < 0.1 ppm
- Nitrogen oxides: < 25 ppm
- Gas temperature: < 40°C

---

## 8. Plasma Propulsion

### 8.1 Ion Thruster

#### 8.1.1 Thrust

```
F = ṁ × ve = ṁ × √(2 × V_acc × e / m_ion)
```

Where:
- `ṁ` = Mass flow rate (kg/s)
- `ve` = Exhaust velocity (m/s)
- `V_acc` = Acceleration voltage (V)

#### 8.1.2 Specific Impulse

```
Isp = ve / g₀ = √(2 × V_acc × e / (m_ion × g₀²))
```

For xenon at 1500 V:
```
Isp ≈ 3,100 seconds
```

#### 8.1.3 Efficiency

```
η = (T² / 2ṁP) = thrust power / total power
```

Typical: η = 60-80%

### 8.2 Hall Effect Thruster

#### 8.2.1 Performance

- Power: 1-50 kW
- Thrust: 50-3000 mN
- Isp: 1500-3000 s
- Efficiency: 45-70%

#### 8.2.2 E×B Drift

```
v_drift = (E × B) / B²
```

Azimuthal drift creates Hall current.

### 8.3 VASIMR (Variable Specific Impulse)

#### 8.3.1 Operating Modes

Low Isp mode:
- Isp: 3,000-5,000 s
- High thrust

High Isp mode:
- Isp: 10,000-30,000 s
- Lower thrust

#### 8.3.2 Power Requirements

```
P = 0.5 × ṁ × ve² / η
```

For 200 kW: ~6 N thrust at Isp = 5,000 s

---

## 9. Magnetohydrodynamics (MHD)

### 9.1 MHD Equations

#### 9.1.1 Continuity

```
∂ρ/∂t + ∇·(ρv) = 0
```

#### 9.1.2 Momentum

```
ρ(∂v/∂t + v·∇v) = -∇p + J×B + ρg
```

#### 9.1.3 Induction

```
∂B/∂t = ∇×(v×B) - ∇×(η∇×B)
```

Where:
- `η` = Magnetic diffusivity = 1/(μ₀σ)
- `σ` = Electrical conductivity

### 9.2 Plasma Beta

```
β = plasma pressure / magnetic pressure = 2μ₀p/B²
```

Typical values:
- Tokamak: β < 0.05
- Stellarator: β < 0.04
- Space plasma: β ≈ 1

### 9.3 Alfvén Velocity

```
vA = B / √(μ₀ρ)
```

Characteristic MHD wave speed.

---

## 10. Implementation Guidelines

### 10.1 System Design

#### 10.1.1 Vacuum System

Pressure ranges:
- Low pressure: 0.1-10 Pa (RF plasma)
- Medium pressure: 10-1000 Pa (DC plasma)
- Atmospheric: 101,325 Pa (DBD, plasma jet)

#### 10.1.2 Power Supply

RF matching network:
- L-type or π-type networks
- Automatic impedance matching
- Reflected power < 5%

#### 10.1.3 Gas Feed System

Mass flow controllers:
- Accuracy: ±1% of full scale
- Response time: < 1 second
- Typical flows: 1-1000 sccm

### 10.2 Control Systems

#### 10.2.1 Feedback Control

PID control for:
- Pressure regulation
- Power control
- Gas flow control
- Temperature management

#### 10.2.2 Safety Interlocks

- Over-pressure protection
- Water cooling flow monitoring
- RF reflected power limit
- Emergency shutdown (E-stop)

---

## 11. Safety & Standards

### 11.1 Electrical Safety

- High voltage isolation: > 10 kV
- Grounding: < 1 Ω resistance
- RF leakage: < 1 mW/cm² at 13.56 MHz
- Interlocks on all access panels

### 11.2 Chemical Safety

- Toxic gas monitoring (e.g., SiH₄, CF₄)
- Scrubber systems for exhaust
- PPE: lab coat, safety glasses, gloves
- Ventilation: > 10 air changes/hour

### 11.3 Radiation Safety

- UV exposure: < 0.1 µW/cm² (safety limit)
- X-rays (from DC arcs): shielding required
- Ozone: < 0.1 ppm (8-hour TWA)

### 11.4 Standards Compliance

- IEC 61010-1: Electrical equipment safety
- SEMI S2: Environmental, health, and safety guideline
- ISO 14644: Cleanroom standards
- IEEE C95.1: RF exposure limits

---

## 12. References

### 12.1 Textbooks

1. Lieberman, M.A. & Lichtenberg, A.J. (2005). *Principles of Plasma Discharges and Materials Processing*. Wiley.
2. Chen, F.F. (2016). *Introduction to Plasma Physics and Controlled Fusion*. Springer.
3. Fridman, A. (2008). *Plasma Chemistry*. Cambridge University Press.
4. Chabert, P. & Braithwaite, N. (2011). *Physics of Radio-Frequency Plasmas*. Cambridge University Press.

### 12.2 Standards

- ASTM F1241: Terminology for Plasma Technology
- ISO 14971: Medical devices - Risk management
- MIL-STD-1574: Electroexplosive Subsystem Safety Requirements

### 12.3 Research Institutions

- ITER Organization: www.iter.org
- Princeton Plasma Physics Laboratory: www.pppl.gov
- Max Planck Institute for Plasma Physics: www.ipp.mpg.de
- MIT Plasma Science and Fusion Center: www.psfc.mit.edu

---

**弘익人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
