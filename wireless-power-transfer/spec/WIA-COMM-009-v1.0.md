# WIA-COMM-009: Wireless Power Transfer Specification v1.0

> **Standard ID:** WIA-COMM-009
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Communication Standards Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Fundamental Principles](#2-fundamental-principles)
3. [Inductive Power Transfer (IPT)](#3-inductive-power-transfer-ipt)
4. [Capacitive Power Transfer (CPT)](#4-capacitive-power-transfer-cpt)
5. [Resonant Wireless Power](#5-resonant-wireless-power)
6. [Microwave Power Transfer](#6-microwave-power-transfer)
7. [Laser Power Beaming](#7-laser-power-beaming)
8. [EV Wireless Charging](#8-ev-wireless-charging)
9. [Space Solar Power](#9-space-solar-power)
10. [Safety and Regulations](#10-safety-and-regulations)
11. [Implementation Guidelines](#11-implementation-guidelines)
12. [References](#12-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the comprehensive framework for wireless power transfer (WPT) technologies, covering electromagnetic coupling methods from millimeters to kilometers, power levels from milliwatts to megawatts, and frequencies from kilohertz to optical.

### 1.2 Scope

The standard covers:
- Theoretical foundations of wireless power transmission
- Near-field inductive and capacitive coupling
- Mid-range resonant coupling
- Far-field microwave and laser power beaming
- Applications: consumer electronics, electric vehicles, industrial, space
- Safety standards and EMI/EMC compliance
- Efficiency optimization techniques

### 1.3 Philosophy

**弘익人間 (Benefit All Humanity)** - This standard aims to democratize wireless power technology, enabling cable-free energy infrastructure for ubiquitous charging, sustainable electrification, and access to remote power sources.

### 1.4 Terminology

- **WPT**: Wireless Power Transfer
- **IPT**: Inductive Power Transfer
- **CPT**: Capacitive Power Transfer
- **Coupling Coefficient (k)**: Measure of magnetic flux linkage (0-1)
- **Quality Factor (Q)**: Ratio of reactive to resistive power in resonant circuit
- **Rectenna**: Rectifying antenna (RF-to-DC converter)
- **SAR**: Specific Absorption Rate (W/kg)
- **FOD**: Foreign Object Detection
- **LOP**: Living Object Protection

---

## 2. Fundamental Principles

### 2.1 Electromagnetic Coupling

Wireless power transfer exploits electromagnetic field coupling between transmitter and receiver.

#### 2.1.1 Near-Field Coupling (d < λ/2π)

In the near-field region, power transfer is dominated by reactive (non-radiative) coupling:

**Inductive Coupling (Magnetic):**
```
Φ₂₁ = M × I₁
V₂ = -M × dI₁/dt
P_rx = k² × Q₁ × Q₂ × P_tx
```

Where:
- `Φ₂₁` = Magnetic flux linking receiver from transmitter
- `M` = Mutual inductance (Henry)
- `k` = Coupling coefficient
- `Q₁, Q₂` = Quality factors

**Capacitive Coupling (Electric):**
```
Q = C × V
C = ε₀ × εᵣ × A / d
P_rx ∝ ω² × C² × V²
```

Where:
- `C` = Capacitance (Farads)
- `ε₀` = Vacuum permittivity
- `εᵣ` = Relative permittivity
- `ω` = Angular frequency

#### 2.1.2 Far-Field Radiation (d > λ)

In the far-field, power transfer is via electromagnetic radiation:

**Friis Transmission Equation:**
```
P_rx / P_tx = G_tx × G_rx × (λ / 4πd)²
```

Where:
- `G_tx, G_rx` = Antenna/aperture gains (linear)
- `λ` = Wavelength
- `d` = Distance

**Path Loss:**
```
L_path [dB] = 20 × log₁₀(4πd/λ)
```

### 2.2 Efficiency vs Distance Tradeoff

Wireless power efficiency fundamentally depends on distance:

| Regime | Distance | Dominant Mechanism | Efficiency Scaling |
|--------|----------|--------------------|--------------------|
| Near-Field | d < λ/2π | Reactive coupling | ∝ (1 + d²/r²)⁻³ |
| Fresnel | λ/2π < d < 2D²/λ | Transition | ∝ 1/d |
| Far-Field | d > 2D²/λ | Radiation | ∝ 1/d² |

Where:
- `r` = Coil/plate radius
- `D` = Antenna aperture diameter

### 2.3 Power Electronics

All WPT systems require power conversion:

```
DC → AC (Inverter) → Wireless Link → AC → DC (Rectifier)
```

**Component Efficiencies:**
- DC-AC Inverter: 92-98%
- Wireless Link: 30-95% (distance dependent)
- AC-DC Rectifier: 85-95%
- Overall: Product of all stages

---

## 3. Inductive Power Transfer (IPT)

### 3.1 Physical Principles

IPT uses time-varying magnetic fields to induce voltage in receiver coil.

#### 3.1.1 Mutual Inductance

For two coaxial circular coils:

```
M = μ₀ × π × r₁² × r₂² × N₁ × N₂ / (2 × (r₁² + d²)^(3/2))
```

Where:
- `μ₀` = 4π × 10⁻⁷ H/m (permeability of free space)
- `r₁, r₂` = Coil radii
- `N₁, N₂` = Number of turns
- `d` = Separation distance

#### 3.1.2 Coupling Coefficient

```
k = M / √(L₁ × L₂)
```

Typical values:
- Excellent alignment, d < r: k = 0.6-0.9
- Moderate alignment, d ≈ r: k = 0.3-0.6
- Poor alignment, d > r: k < 0.3

#### 3.1.3 Power Transfer Efficiency

For loosely coupled coils (k² « 1):

```
η = k² × Q₁ × Q₂ / (1 + k² × Q₁ × Q₂)
```

For strongly coupled (k² × Q₁ × Q₂ » 1):

```
η → 1 (theoretical limit)
```

### 3.2 Coil Design

#### 3.2.1 Inductance Calculation

Circular coil:
```
L = μ₀ × N² × r × [ln(8r/a) - 2]
```

Where:
- `a` = Wire radius
- `r` = Coil radius
- `N` = Turns

#### 3.2.2 Quality Factor

```
Q = ω × L / R = 2πf × L / R
```

Where:
- `R` = Series resistance (wire + core losses)
- `f` = Frequency

**Optimization:**
- Use Litz wire to reduce skin effect
- Ferrite core for flux concentration (µᵣ = 1000-3000)
- Minimize coil resistance
- Optimize operating frequency

### 3.3 Standards

#### 3.3.1 Qi Standard (WPC)

- **Frequency**: 80-300 kHz (baseline), 2 MHz (extended)
- **Power**: 5-15 W (baseline), up to 100 W (extended)
- **Range**: 0-5 mm
- **Communication**: ASK modulation (2 kbps)
- **Applications**: Smartphones, wearables

#### 3.3.2 AirFuel Resonant

- **Frequency**: 6.78 MHz (ISM band)
- **Power**: 5-50 W
- **Range**: 0-50 mm
- **Communication**: Bluetooth Low Energy
- **Applications**: Laptops, tablets

#### 3.3.3 SAE J2954 (EV Charging)

- **Frequency**: 85 kHz
- **Power Classes**: WPT1 (3.7 kW), WPT2 (7.7 kW), WPT3 (11 kW), WPT4 (22 kW)
- **Air Gap**: 100-250 mm
- **Misalignment Tolerance**: ±100 mm (X), ±75 mm (Y)
- **Efficiency**: > 85% (grid to vehicle)

---

## 4. Capacitive Power Transfer (CPT)

### 4.1 Physical Principles

CPT uses electric field coupling through dielectric material.

#### 4.1.1 Capacitance

Parallel-plate capacitor:

```
C = ε₀ × εᵣ × A / d
```

Where:
- `A` = Plate area
- `d` = Separation distance
- `εᵣ` = Relative permittivity of dielectric

#### 4.1.2 Power Transfer

Capacitive reactance:
```
X_C = 1 / (2πfC)
```

Power:
```
P = V² / X_C = (2πfC) × V²
```

### 4.2 Advantages vs Inductive

| Feature | Inductive | Capacitive |
|---------|-----------|------------|
| Tolerance to metal | Poor | Good |
| Tolerance to dielectric | Good | Poor |
| EMI | Higher | Lower |
| Coil/plate cost | Higher | Lower |
| Typical efficiency | 70-90% | 60-80% |

### 4.3 Applications

- **Underwater charging**: Waterproof dielectric isolation
- **Harsh environments**: No exposed conductors
- **Rotating machinery**: Contactless slip rings
- **Medical implants**: Biocompatible dielectric

---

## 5. Resonant Wireless Power

### 5.1 Strongly Coupled Resonance

Resonant WPT operates at the natural frequency where:

```
f₀ = 1 / (2π√(LC))
```

#### 5.1.1 Coupled Mode Theory

Two resonators split into two eigenmodes with frequencies:

```
ω₊ = ω₀ × √(1 + k)
ω₋ = ω₀ × √(1 - k)
```

Power transfer is maximized when system operates at:

```
ω_opt = ω₀ × √(1 + k²)
```

#### 5.1.2 Efficiency Formula

```
η = k² × Q_L1 × Q_L2 / [(1 + k² × Q_L1 × Q_L2)² / 4]
```

Where:
- `Q_L1, Q_L2` = Loaded quality factors (including load)

### 5.2 Compensation Topologies

| Topology | TX Capacitor | RX Capacitor | Characteristics |
|----------|--------------|--------------|-----------------|
| SS (Series-Series) | Series | Series | Constant current, simple |
| SP (Series-Parallel) | Series | Parallel | Constant current TX, voltage RX |
| PS (Parallel-Series) | Parallel | Series | Constant voltage TX |
| PP (Parallel-Parallel) | Parallel | Parallel | Constant voltage |
| LCC | L + C + C | Series/Parallel | Wide ZVS range |

### 5.3 WiTricity Technology

- **Frequency**: 6.78 MHz (ISM band)
- **Range**: 1-2 meters
- **Power**: 1-11 kW
- **Efficiency**: 90-95% at optimal coupling
- **Applications**: EV charging, robotics, medical devices

---

## 6. Microwave Power Transfer

### 6.1 Physical Principles

Microwave WPT uses electromagnetic radiation in the GHz range.

#### 6.1.1 Antenna Gain

Aperture antenna:
```
G = (4π × A_eff) / λ²
```

Where:
- `A_eff` = Effective aperture area
- `λ` = Wavelength

#### 6.1.2 Power Density

At distance d from isotropic source:
```
S = P_tx / (4πd²)
```

With directional antenna:
```
S = (P_tx × G_tx) / (4πd²)
```

#### 6.1.3 Received Power

```
P_rx = S × A_eff = P_tx × G_tx × G_rx × (λ / 4πd)²
```

### 6.2 Rectenna Technology

Rectenna = Rectifying Antenna (RF-to-DC conversion)

#### 6.2.1 Components

1. **Antenna**: Dipole, patch, horn, or parabolic
2. **Matching Network**: Impedance transformation
3. **Rectifier**: Schottky diodes, GaN HEMTs
4. **DC Filter**: Smooth output voltage

#### 6.2.2 Efficiency

Rectenna efficiency depends on:
- Antenna efficiency: 80-95%
- Rectifier efficiency: 70-90%
- Overall: 60-85%

State-of-the-art:
- **2.45 GHz**: 82% @ 40 mW/cm²
- **5.8 GHz**: 85% @ 60 mW/cm²
- **24 GHz**: 70% @ 100 mW/cm²

### 6.3 Frequency Bands

| Band | Frequency | Wavelength | Applications |
|------|-----------|------------|--------------|
| S-band | 2.45 GHz (ISM) | 12.2 cm | Drones, IoT |
| C-band | 5.8 GHz (ISM) | 5.2 cm | Mid-range power |
| K-band | 24 GHz (ISM) | 1.25 cm | High-density links |
| W-band | 94 GHz | 3.2 mm | Emerging research |

### 6.4 Beam Steering

#### 6.4.1 Phased Array

Phase shift at element n:
```
Φₙ = 2π × d × sin(θ) / λ
```

Where:
- `d` = Element spacing
- `θ` = Beam steering angle

#### 6.4.2 Retrodirective Array

Automatically directs beam toward pilot signal source using phase conjugation.

### 6.5 Applications

- **Drone/UAV charging**: In-flight power delivery
- **Remote sensors**: Power IoT devices wirelessly
- **Disaster relief**: Power delivery without cables
- **Military**: Forward operating base power

---

## 7. Laser Power Beaming

### 7.1 Physical Principles

Laser power beaming uses optical or infrared light to transmit power.

#### 7.1.1 Beam Propagation

Gaussian beam divergence:
```
θ = λ / (π × w₀)
```

Spot size at distance z:
```
w(z) = w₀ × √(1 + (z × λ / π × w₀²)²)
```

Where:
- `w₀` = Beam waist radius
- `θ` = Divergence half-angle

#### 7.1.2 Power Density

```
I(r) = (2 × P) / (π × w²) × exp(-2r² / w²)
```

### 7.2 Photovoltaic Conversion

#### 7.2.1 PV Cell Efficiency

Varies with wavelength and cell type:

| PV Cell Type | Peak λ | Efficiency | Cost |
|--------------|--------|------------|------|
| Silicon (Si) | 800-900 nm | 25-30% | Low |
| GaAs | 850 nm | 35-40% | Medium |
| InGaAs | 1064-1550 nm | 30-35% | High |
| Multi-junction | 400-1800 nm | 40-50% | Very High |

#### 7.2.2 Laser-to-Electrical Efficiency

```
η_total = η_laser × η_transmission × η_PV
```

Typical values:
- Laser efficiency (wall plug): 30-60%
- Atmospheric transmission: 60-90%
- PV conversion: 25-50%
- **Overall**: 5-25%

### 7.3 Wavelength Selection

| Wavelength | Laser Type | Atm. Trans. | PV Match | Safety |
|------------|------------|-------------|----------|--------|
| 450 nm (blue) | Diode | Poor (Rayleigh) | Si good | Eye hazard |
| 808 nm (IR) | Diode | Good | GaAs/Si | Eye hazard |
| 980 nm (IR) | Diode | Good | GaAs | Eye hazard |
| 1064 nm (IR) | Nd:YAG | Excellent | InGaAs | Eye safe-ish |
| 1550 nm (IR) | Fiber | Excellent | InGaAs | Eye safe |

### 7.4 Safety Considerations

Maximum Permissible Exposure (MPE) for eye safety:

**Continuous wave laser (t > 10 s):**
```
MPE = 10^(C_A) / (7 × λ)  [W/m²]
```

Where:
- `λ` in nm
- `C_A` = correction factors

For 1064 nm CW laser:
- MPE ≈ 2 W/m² (eye)
- MPE ≈ 1000 W/m² (skin)

### 7.5 Applications

- **Space power relay**: Satellite-to-satellite, orbit-to-ground
- **Lunar base power**: Pole to polar night regions
- **Disaster relief**: Helicopter-to-ground power
- **Underwater**: Penetrates water better than RF
- **Hazardous areas**: No sparks or EMI

---

## 8. EV Wireless Charging

### 8.1 Stationary Charging

#### 8.1.1 SAE J2954 Power Classes

| Class | Power (kW) | Use Case | Charge Time (60 kWh) |
|-------|------------|----------|----------------------|
| WPT1 | 3.7 | Overnight | ~16 hours |
| WPT2 | 7.7 | Home/workplace | ~8 hours |
| WPT3 | 11 | Public charging | ~5.5 hours |
| WPT4 | 22 | Fast charging | ~2.7 hours |

#### 8.1.2 Coil Design

**Ground Assembly (GA):**
- Diameter: 400-600 mm
- Ferrite backing for flux shaping
- Shielding for EMI reduction

**Vehicle Assembly (VA):**
- Diameter: 300-500 mm
- Lightweight (< 10 kg)
- 100-250 mm ground clearance

#### 8.1.3 Alignment

Maximum misalignment tolerance:
- Lateral (X): ±100 mm
- Longitudinal (Y): ±75 mm
- Vertical (Z): 100-250 mm

### 8.2 Dynamic Charging (In-Road)

#### 8.2.1 System Architecture

```
Grid → Inverter → Segmented Road Coils → Vehicle Pickup → Battery
```

**Segment Switching:**
- Each road segment: 1-5 meters
- Dynamic activation as vehicle approaches
- Multiple vehicles per lane supported

#### 8.2.2 Power Delivery

For vehicle traveling at speed v:

```
Duty Cycle = L_segment / L_spacing
Power_avg = Power_rated × Duty_cycle × η
```

Example (100 km/h, 3m segments, 20 kW @ 85% efficiency):
- Time per segment: 0.108 s
- Energy per segment: 2.16 kJ
- Power averaged: 17 kW

#### 8.2.3 Economic Analysis

**Infrastructure Cost:**
- Coil installation: $1000-2000 per meter
- Power electronics: $100,000-200,000 per km
- Total: ~$1-2 million per km per lane

**Break-even:** High-traffic routes (buses, trucks, taxis)

### 8.3 Safety and Standards

- **FOD (Foreign Object Detection)**: Q-factor, IR thermal, capacitive
- **LOP (Living Object Protection)**: Power reduction if animal detected
- **EMC**: EN 55011 Class B, FCC Part 15
- **Ground Fault Protection**: GFCI
- **Interoperability**: ISO 19363, SAE J2954

---

## 9. Space Solar Power

### 9.1 Concept

Collect solar energy in space (24/7 sunlight, no atmosphere) and beam to Earth.

#### 9.1.1 Orbital Configurations

| Orbit | Altitude | Period | Sunlight | Coverage |
|-------|----------|--------|----------|----------|
| LEO | 400-1000 km | 90-100 min | 60-70% | Local |
| MEO | 10,000-20,000 km | 6 hours | 90-95% | Regional |
| GEO | 35,786 km | 24 hours | 99% | Hemispherical |

**GEO preferred for continuous power.**

#### 9.1.2 Solar Collection

Solar constant in space: 1361 W/m²

For 1 km² solar array @ 30% efficiency:
```
P_solar = 1361 × 10⁶ × 0.30 = 408 MW
```

### 9.2 Wireless Transmission Options

#### 9.2.1 Microwave (2.45 GHz)

**Transmit Antenna:**
- Diameter: 1 km
- Phased array: millions of elements
- Gain: ~60 dBi

**Receive Rectenna:**
- Diameter: 3-10 km (ground)
- Efficiency: 80-85%
- DC power output: 200-300 MW

**Link Efficiency:**
```
η_link = (λ / 4πd)² × π × D_rx² / (4 × (λd / D_tx)²)
       ≈ D_tx² × D_rx² / (4 × λ² × d²)
```

For D_tx = 1 km, D_rx = 5 km, d = 36,000 km, λ = 0.122 m:
```
η_link ≈ 40-60%
```

#### 9.2.2 Laser (1064 nm)

**Advantages:**
- Smaller apertures (1-10 m vs 1 km)
- No ionospheric effects
- Higher power density

**Disadvantages:**
- Atmospheric absorption and scattering
- Cloud blockage
- Beam wander due to turbulence
- Lower PV efficiency

### 9.3 Challenges

1. **Cost**: $100-1000 billion for multi-GW system
2. **Launch Mass**: 10,000-100,000 tons to orbit
3. **Assembly**: Robotic construction in space
4. **Beam Safety**: Power density < 10 W/m² at ground
5. **Regulation**: International coordination, spectrum allocation
6. **Public Acceptance**: Safety perception

### 9.4 Near-Term Applications

- **Lunar Base Power**: Pole-to-pole beaming during lunar night
- **Space-to-Space**: Power relay between satellites
- **Orbital Debris Removal**: Laser ablation propulsion
- **Mars Power**: Orbital solar to surface base

---

## 10. Safety and Regulations

### 10.1 Electromagnetic Field Exposure

#### 10.1.1 SAR (Specific Absorption Rate)

SAR measures power absorbed by human tissue:

```
SAR = σ × |E|² / ρ  [W/kg]
```

Where:
- `σ` = Tissue conductivity (S/m)
- `E` = Electric field (V/m)
- `ρ` = Tissue density (kg/m³)

**Regulatory Limits:**
- **FCC (USA)**: 1.6 W/kg (1g average)
- **ICNIRP (EU)**: 2.0 W/kg (10g average)

#### 10.1.2 Power Density Limits

**General Public (ICNIRP):**

| Frequency | Power Density Limit |
|-----------|---------------------|
| 1-10 MHz | 0.92 W/m² |
| 10-400 MHz | 2 W/m² |
| 400-2000 MHz | f/200 W/m² |
| 2-300 GHz | 10 W/m² |

**Occupational Exposure:** 5× higher

### 10.2 EMI/EMC Compliance

#### 10.2.1 Conducted Emissions

**FCC Part 15 Class B:**
- 150 kHz - 500 kHz: 48-66 dBµV (quasi-peak)
- 500 kHz - 30 MHz: 48-60 dBµV

**EN 55011 Class B:**
- Similar to FCC, with CISPR 11 Group 1

#### 10.2.2 Radiated Emissions

**10 m distance:**
- 30-230 MHz: 30-37 dBµV/m
- 230-1000 MHz: 37 dBµV/m

### 10.3 Foreign Object Detection (FOD)

#### 10.3.1 Methods

| Method | Principle | Detectable Objects | Response Time |
|--------|-----------|--------------------| --------------|
| Q-factor | Coil resistance change | Metallic | < 100 ms |
| Thermal (IR) | Temperature rise | All conductive | 1-5 s |
| Capacitive | Capacitance change | All | < 50 ms |
| Radar/Lidar | Reflection | All | < 10 ms |
| Visual (camera) | Image processing | All | 100-500 ms |

**Qi Standard FOD:**
- Power loss method: Measure ΔP = P_tx - P_rx
- If ΔP > threshold (e.g., 1 W), reduce/stop power

### 10.4 Living Object Protection (LOP)

Detection methods:
- **Motion sensing**: Camera, PIR sensor
- **Thermal signature**: Temperature, heat pattern
- **Biometric**: Heartbeat, breathing (IR imaging)
- **Pilot tone reflection**: Reflected signal from tissue

**Action:** Reduce power to < 1 W if living object detected

### 10.5 Interoperability Standards

| Standard | Organization | Technology | Status |
|----------|--------------|------------|--------|
| Qi | WPC | Inductive (80-300 kHz) | Published |
| AirFuel Resonant | AirFuel Alliance | Resonant (6.78 MHz) | Published |
| SAE J2954 | SAE International | EV IPT (85 kHz) | Published 2020 |
| ISO 19363 | ISO | EV wireless | Published 2020 |
| IEC 61980 | IEC | EV inductive | Published 2015 |

---

## 11. Implementation Guidelines

### 11.1 System Design Process

1. **Requirements Analysis**
   - Power level (W, kW, MW)
   - Distance (mm, m, km)
   - Frequency constraints (ISM bands, regulations)
   - Efficiency target
   - Cost budget

2. **Technology Selection**

   | Power | Distance | Technology |
   |-------|----------|------------|
   | < 100 W | < 10 mm | Inductive (Qi) |
   | 100 W - 50 kW | 10 mm - 2 m | Resonant |
   | > 1 kW | > 2 m | Microwave/Laser |

3. **Coil/Antenna Design**
   - FEMM, ANSYS Maxwell for magnetic simulation
   - CST Microwave Studio, HFSS for RF
   - Zemax, FRED for optical

4. **Power Electronics**
   - Inverter topology: Full-bridge, half-bridge
   - Control: PLL, frequency tracking
   - Communication: ASK, FSK, Bluetooth

5. **Safety Integration**
   - FOD, LOP sensors
   - Thermal management
   - EMI shielding

6. **Testing & Certification**
   - Efficiency measurement (Yokogawa WT5000)
   - EMC testing (CISPR 11, FCC Part 15)
   - SAR measurement (IEEE 1528, IEC 62209)

### 11.2 Efficiency Optimization

#### 11.2.1 Coil Optimization

- **Frequency selection:** Higher f → smaller coils, but higher AC losses
  - Qi: 80-300 kHz (compromise)
  - Resonant: 6.78 MHz (ISM, higher power density)
  - EV: 85 kHz (penetration, efficiency)

- **Wire selection:**
  - Litz wire for f > 20 kHz (reduce skin effect)
  - Copper tube for high current (cooling)

- **Ferrite core:**
  - Mn-Zn ferrite for < 1 MHz
  - Ni-Zn ferrite for > 1 MHz
  - Trade-off: permeability vs losses

#### 11.2.2 Impedance Matching

Maximize power transfer:
```
Z_load = Z_source*
```

Use matching network (L, C) to transform impedances.

#### 11.2.3 Rectifier Optimization

- **Schottky diodes:** Low forward voltage (0.3-0.5 V)
- **Synchronous rectification:** MOSFET switches (< 0.1 V drop)
- **GaN HEMTs:** High frequency (< 10 ns switching)

### 11.3 Thermal Management

Power dissipation:
```
P_loss = P_tx × (1 - η)
```

For 11 kW EV charger @ 90% efficiency:
```
P_loss = 11,000 × 0.10 = 1,100 W
```

Cooling options:
- **Natural convection:** < 50 W
- **Forced air:** 50-500 W
- **Liquid cooling:** > 500 W

### 11.4 Cost Analysis

| Component | Cost (per unit) | Scaling |
|-----------|-----------------|---------|
| TX coil (EV, 11 kW) | $200-500 | Volume production |
| RX coil (EV) | $300-600 | Integration |
| Power electronics | $500-1500 | Semiconductor cost |
| Control/comm | $100-300 | MCU, sensors |
| Installation (stationary) | $500-1000 | Labor |
| **Total (EV Level 2)** | **$1,600-3,900** | Target: < $2000 |

---

## 12. References

### 12.1 Standards

1. **Wireless Power Consortium**, "Qi Specification v1.3," 2021
2. **AirFuel Alliance**, "Resonant Wireless Power Transfer Specification," 2020
3. **SAE International**, "SAE J2954: Wireless Power Transfer for Light-Duty Plug-In/Electric Vehicles," 2020
4. **ISO**, "ISO 19363: Electrically propelled road vehicles - Magnetic field wireless power transfer - Safety and interoperability requirements," 2020
5. **IEC**, "IEC 61980: Electric vehicle wireless power transfer systems," 2015

### 12.2 Safety & EMC

6. **FCC**, "Part 15: Radio Frequency Devices," CFR Title 47
7. **ICNIRP**, "Guidelines for Limiting Exposure to Electromagnetic Fields (100 kHz to 300 GHz)," 2020
8. **IEEE**, "IEEE C95.1: Standard for Safety Levels with Respect to Human Exposure to Electric, Magnetic, and Electromagnetic Fields," 2019
9. **CISPR**, "CISPR 11: Industrial, scientific and medical equipment - Radio-frequency disturbance characteristics," 2015

### 12.3 Technical Literature

10. **Kurs, A. et al.**, "Wireless Power Transfer via Strongly Coupled Magnetic Resonances," *Science*, 317(5834), 2007
11. **Brown, W. C.**, "The History of Power Transmission by Radio Waves," *IEEE Trans. Microwave Theory Tech.*, 1984
12. **Shinohara, N.**, "Beam Control Technologies with a High-Efficiency Phased Array for Microwave Power Transmission," *IEEE Trans. Industrial Electronics*, 2013
13. **Landis, G. A.**, "Laser Power Beaming for Lunar Polar Exploration," *AIAA Space 2007*
14. **Covic, G. A., Boys, J. T.**, "Modern Trends in Inductive Power Transfer for Transportation Applications," *IEEE J. Emerging Technologies*, 2013

### 12.4 WIA Integration

15. **WIA-OMNI-API**: Universal energy transfer API integration
16. **WIA-INTENT**: Intent-based power request protocols
17. **WIA-IOT**: IoT device wireless charging standards

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
