# WIA-COMM-008: Wireless Charging Specification v1.0

> **Standard ID:** WIA-COMM-008
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Communication Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Inductive Coupling](#2-inductive-coupling)
3. [Magnetic Resonance](#3-magnetic-resonance)
4. [Qi Standard (WPC)](#4-qi-standard-wpc)
5. [AirFuel Alliance](#5-airfuel-alliance)
6. [Power Transfer Efficiency](#6-power-transfer-efficiency)
7. [Coil Design](#7-coil-design)
8. [Foreign Object Detection (FOD)](#8-foreign-object-detection-fod)
9. [Alignment and Positioning](#9-alignment-and-positioning)
10. [Thermal Management](#10-thermal-management)
11. [EMF Safety](#11-emf-safety)
12. [EV Wireless Charging](#12-ev-wireless-charging)
13. [Multi-Device Charging](#13-multi-device-charging)
14. [Interoperability Testing](#14-interoperability-testing)
15. [Implementation Guidelines](#15-implementation-guidelines)
16. [References](#16-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the comprehensive framework for wireless charging technology, encompassing inductive coupling, magnetic resonance charging, power delivery protocols, efficiency optimization, safety mechanisms, and interoperability standards.

### 1.2 Scope

The standard covers:
- Qi standard (WPC) inductive charging (5W to 15W)
- AirFuel resonant charging technology
- High-power wireless charging (50W+)
- EV wireless charging (up to 350kW)
- Foreign object detection and safety mechanisms
- Coil design and electromagnetic theory
- Efficiency optimization techniques
- Thermal management and cooling
- EMF exposure limits and shielding
- Multi-device charging systems
- Interoperability and certification

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to advance wireless charging technology for the betterment of humanity through convenient, safe, and efficient power delivery that eliminates cable dependency and enables seamless charging experiences.

### 1.4 Terminology

- **Inductive Coupling**: Power transfer via electromagnetic induction between coils
- **Magnetic Resonance**: Power transfer via resonant magnetic coupling
- **Qi**: Wireless Power Consortium standard for inductive charging
- **AirFuel**: Alliance of Wireless Power and Power Matters Alliance
- **FOD**: Foreign Object Detection
- **Coupling Coefficient (k)**: Measure of magnetic coupling between coils (0-1)
- **Quality Factor (Q)**: Ratio of energy stored to energy dissipated per cycle
- **Mutual Inductance (M)**: Induced voltage per unit current change
- **BPP**: Baseline Power Profile (Qi, up to 5W)
- **EPP**: Extended Power Profile (Qi, up to 15W)

---

## 2. Inductive Coupling

### 2.1 Fundamental Principle

Inductive coupling uses Faraday's law of electromagnetic induction to transfer power wirelessly between two coils.

#### Faraday's Law
```
ε = -N × dΦ/dt
```

Where:
- `ε` = Induced electromotive force (volts)
- `N` = Number of turns in coil
- `Φ` = Magnetic flux (webers)
- `t` = Time (seconds)

#### Induced Voltage
```
V₂ = M × dI₁/dt
```

Where:
- `V₂` = Induced voltage in receiver coil (volts)
- `M` = Mutual inductance (henries)
- `I₁` = Current in transmitter coil (amperes)

### 2.2 Mutual Inductance

```
M = k × √(L₁ × L₂)
```

Where:
- `M` = Mutual inductance (henries)
- `k` = Coupling coefficient (0-1)
- `L₁` = Transmitter coil inductance (henries)
- `L₂` = Receiver coil inductance (henries)

### 2.3 Coupling Coefficient

The coupling coefficient depends on:
- **Distance**: k ∝ 1/d³ for circular coils
- **Alignment**: Maximum at perfect alignment
- **Coil geometry**: Diameter, turns, wire gauge
- **Core material**: Air, ferrite, nanocrystalline

```
k ≈ (r₁ × r₂)² / [(r₁ × r₂)² + d²]^(3/2)
```

Where:
- `r₁` = Transmitter coil radius (meters)
- `r₂` = Receiver coil radius (meters)
- `d` = Separation distance (meters)

### 2.4 Power Transfer

```
P = k² × Q₁ × Q₂ × P_in
```

Where:
- `P` = Transferred power (watts)
- `Q₁, Q₂` = Quality factors of transmitter and receiver
- `P_in` = Input power (watts)

---

## 3. Magnetic Resonance

### 3.1 Resonance Principle

Magnetic resonance charging operates at the resonant frequency where inductive reactance equals capacitive reactance, maximizing power transfer efficiency.

#### Resonant Frequency
```
f₀ = 1 / (2π × √(L × C))
```

Where:
- `f₀` = Resonant frequency (Hz)
- `L` = Inductance (henries)
- `C` = Capacitance (farads)

### 3.2 Strongly Coupled Magnetic Resonance

```
η_max = (k × Q)² / (1 + k × Q)²
```

Where:
- `η_max` = Maximum efficiency (0-1)
- `k` = Coupling coefficient
- `Q` = Geometric mean of quality factors (√(Q₁ × Q₂))

### 3.3 Advantages Over Simple Induction

- **Longer range**: Up to 50 mm vs. 10 mm
- **Spatial freedom**: Less sensitive to alignment
- **Multiple devices**: Can charge several devices simultaneously
- **Higher efficiency at distance**: Efficiency degrades more slowly

---

## 4. Qi Standard (WPC)

### 4.1 Overview

The Qi standard, developed by the Wireless Power Consortium (WPC), is the most widely adopted wireless charging standard for consumer electronics.

### 4.2 Power Profiles

#### Baseline Power Profile (BPP)
- **Power**: 0-5W
- **Frequency**: 110-205 kHz
- **Voltage**: 5V
- **Current**: Up to 1A
- **Devices**: Basic smartphones, accessories

#### Extended Power Profile (EPP)
- **Power**: 5-15W
- **Frequency**: 110-205 kHz
- **Voltage**: 9V, 12V
- **Current**: Up to 1.67A
- **Devices**: Fast-charging smartphones

#### Qi v1.3 (Medium Power)
- **Power**: Up to 30W
- **Frequency**: 87-205 kHz
- **Devices**: Laptops, tablets

### 4.3 Communication Protocol

Qi uses **backscatter modulation** for communication:

#### Receiver to Transmitter
- **Modulation**: Load modulation (2 kHz)
- **Data rate**: 2 kbps
- **Packets**: Signal strength, control, power request

#### Packet Structure
```
[Preamble] [Header] [Message] [Checksum]
   11 bits    1 byte  0-27 bytes  1 byte
```

### 4.4 Power Transfer Phases

1. **Selection**: Detect object on charging surface
2. **Ping**: Send analog ping to detect receiver
3. **Identification**: Receiver identifies itself
4. **Configuration**: Negotiate power level
5. **Power Transfer**: Deliver power with control loop
6. **Renegotiation**: Adjust power as needed

### 4.5 Operating Frequency

```
f = f_base + (n × 64) Hz
```

Where:
- `f_base` = 110 kHz (BPP) or 87 kHz (EPP)
- `n` = 0-1470 (step number)
- Valid range: 110-205 kHz (BPP), 87-205 kHz (EPP)

---

## 5. AirFuel Alliance

### 5.1 Overview

AirFuel combines resonant and inductive wireless charging technologies, operating at higher frequencies than Qi.

### 5.2 Specifications

- **Frequency**: 6.78 MHz (ISM band)
- **Power**: Up to 50W
- **Range**: 0-50 mm
- **Technology**: Magnetic resonance
- **Topology**: Class-E amplifier

### 5.3 Advantages

- **Longer range**: Up to 50 mm charging distance
- **Spatial freedom**: ~100 cm² charging zone
- **Multi-device**: Charge multiple devices simultaneously
- **Power scalability**: 1W to 50W+

### 5.4 Bluetooth Low Energy (BLE) Control

AirFuel uses BLE for:
- Device discovery
- Power negotiation
- Authentication
- Status monitoring

---

## 6. Power Transfer Efficiency

### 6.1 Efficiency Calculation

```
η = (P_out / P_in) × 100%
```

Where:
- `η` = Efficiency (percentage)
- `P_out` = Output power at receiver (watts)
- `P_in` = Input power at transmitter (watts)

### 6.2 Loss Mechanisms

#### Coil Resistance Losses
```
P_loss_coil = I² × R
```

#### Core Losses
```
P_loss_core = k_h × f × B² + k_e × f² × B²
```

Where:
- `k_h` = Hysteresis loss coefficient
- `k_e` = Eddy current loss coefficient
- `f` = Frequency (Hz)
- `B` = Magnetic flux density (tesla)

#### Rectification Losses
```
P_loss_rect = V_f × I + I² × R_on
```

Where:
- `V_f` = Forward voltage drop (volts)
- `R_on` = On-resistance (ohms)

### 6.3 Typical Efficiencies

| Power Level | Technology | Efficiency | Distance |
|-------------|------------|------------|----------|
| 5W | Qi BPP | 70-75% | 3-5 mm |
| 15W | Qi EPP | 75-80% | 3-8 mm |
| 50W | AirFuel | 65-75% | 10-50 mm |
| 11kW | SAE J2954 | 85-90% | 100-200 mm |

### 6.4 Efficiency Optimization

1. **Impedance matching**: Match coil impedance to load
2. **Resonance tuning**: Operate at resonant frequency
3. **Coil alignment**: Minimize lateral/angular misalignment
4. **High-Q coils**: Use litz wire and ferrite shielding
5. **Active rectification**: Replace diodes with synchronous FETs

---

## 7. Coil Design

### 7.1 Coil Inductance

For a flat spiral coil:
```
L ≈ (μ₀ × N² × r²) / (8r + 11w)
```

Where:
- `L` = Inductance (henries)
- `μ₀` = Permeability of free space (4π × 10⁻⁷ H/m)
- `N` = Number of turns
- `r` = Mean coil radius (meters)
- `w` = Coil width (outer radius - inner radius)

### 7.2 Quality Factor

```
Q = 2πfL / R = ωL / R
```

Where:
- `Q` = Quality factor (dimensionless)
- `f` = Frequency (Hz)
- `L` = Inductance (henries)
- `R` = AC resistance (ohms)
- `ω` = Angular frequency (rad/s)

### 7.3 Litz Wire

Litz wire reduces AC resistance by using multiple insulated strands:

```
R_AC ≈ R_DC × [1 + (d⁴ × π⁴ × f²) / (192 × ρ²)]
```

Where:
- `R_AC` = AC resistance (ohms)
- `R_DC` = DC resistance (ohms)
- `d` = Wire diameter (meters)
- `f` = Frequency (Hz)
- `ρ` = Resistivity (Ω·m)

**Litz wire advantages**:
- Lower AC resistance at high frequencies
- Higher Q factor
- Better efficiency

### 7.4 Ferrite Shielding

Ferrite material behind coils:
- **Increases inductance**: By 2-5×
- **Focuses flux**: Directs magnetic field forward
- **Reduces EMI**: Shields electronics from fields
- **Common materials**: MnZn ferrite (PC40, PC95, 3F3)

---

## 8. Foreign Object Detection (FOD)

### 8.1 Purpose

Foreign objects (coins, keys, metal) can:
- Heat up dangerously (>100°C)
- Reduce efficiency
- Damage charger or receiver
- Create safety hazards

### 8.2 Detection Methods

#### 8.2.1 Q-Factor Measurement

Metal objects reduce coil Q factor:

```
ΔQ = (Q_baseline - Q_measured) / Q_baseline
```

**Threshold**: ΔQ > 10% indicates foreign object

#### 8.2.2 Power Loss Method

Compare expected vs. actual power:

```
P_loss = P_in - P_out - P_expected_loss
```

**Threshold**: P_loss > 1W indicates foreign object

#### 8.2.3 Frequency Detuning

Metal objects shift resonant frequency:

```
Δf = f_baseline - f_measured
```

**Threshold**: Δf > 2% indicates foreign object

### 8.3 Qi FOD Implementation

1. **Baseline calibration**: Measure Q without objects
2. **Continuous monitoring**: Check Q every 1 second
3. **Power cutoff**: Stop charging if FOD detected
4. **User notification**: LED, beep, or app alert

---

## 9. Alignment and Positioning

### 9.1 Alignment Importance

Misalignment reduces:
- **Coupling coefficient**: k drops with lateral offset
- **Efficiency**: η decreases quadratically
- **Power delivery**: May not meet device requirements

### 9.2 Lateral Misalignment

```
k(Δx) ≈ k₀ × exp(-α × Δx²)
```

Where:
- `Δx` = Lateral offset (meters)
- `k₀` = Coupling at perfect alignment
- `α` = Decay constant (depends on coil geometry)

### 9.3 Alignment Feedback

#### Visual
- **LED ring**: Indicates alignment quality (red/yellow/green)
- **On-screen**: App displays alignment score

#### Auditory
- **Beep frequency**: Faster beeps = better alignment
- **Tone pitch**: Higher pitch = better alignment

#### Haptic
- **Vibration pattern**: Stronger vibration = better alignment
- **Pulse rate**: Faster pulses = better alignment

### 9.4 Guided Positioning

Advanced systems use:
- **Magnets**: Passive mechanical alignment
- **Motors**: Active coil positioning
- **Cameras**: Computer vision for alignment
- **Multiple coils**: Adaptive coil selection

---

## 10. Thermal Management

### 10.1 Heat Generation

Heat sources:
- **Coil losses**: I²R heating in windings
- **Core losses**: Hysteresis and eddy currents
- **Rectifier losses**: Diode or FET dissipation
- **Foreign objects**: Induced current heating

### 10.2 Temperature Monitoring

Use NTC thermistors:
- **Location**: Embedded in transmitter coil
- **Sampling rate**: 1-10 Hz
- **Thresholds**: Warning (70°C), cutoff (80°C)

```
T = β / ln(R/R₀) - 273.15
```

Where:
- `T` = Temperature (°C)
- `β` = NTC beta coefficient (K)
- `R` = Measured resistance (Ω)
- `R₀` = Resistance at 25°C (Ω)

### 10.3 Thermal Management Strategies

1. **Dynamic power scaling**: Reduce power if temperature rises
2. **Duty cycle control**: Pulse charging with cooling intervals
3. **Forced air cooling**: Fan for high-power chargers
4. **Heatsink**: Aluminum or copper heat spreader
5. **Thermal pads**: Interface material to chassis

### 10.4 Temperature Limits

| Component | Warning | Cutoff | Material |
|-----------|---------|--------|----------|
| Transmitter coil | 70°C | 80°C | Copper/ferrite |
| Receiver coil | 70°C | 80°C | Copper/ferrite |
| Rectifier | 100°C | 125°C | Silicon (diode/FET) |
| Battery | 40°C | 45°C | Lithium-ion |
| Foreign object | N/A | 100°C | Metal |

---

## 11. EMF Safety

### 11.1 Electromagnetic Field Exposure

Wireless charging creates time-varying magnetic fields that can induce currents in conductive tissues.

### 11.2 Safety Standards

#### ICNIRP Guidelines (2010)

For 110-205 kHz (Qi):
- **Magnetic field (B)**: 6.25 μT (RMS)
- **Electric field (E)**: 83 V/m (RMS)
- **Specific Absorption Rate (SAR)**: 2 W/kg (localized)

#### IEEE C95.1 (2019)

For 3 kHz - 10 MHz:
- **Magnetic field (B)**: 27 μT (RMS)
- **Electric field (E)**: 614 V/m (RMS)

### 11.3 Field Calculation

Magnetic field at distance d from coil:

```
B(d) ≈ (μ₀ × N × I × r²) / [2 × (r² + d²)^(3/2)]
```

Where:
- `B` = Magnetic flux density (tesla)
- `μ₀` = 4π × 10⁻⁷ H/m
- `N` = Number of turns
- `I` = Current (amperes)
- `r` = Coil radius (meters)
- `d` = Distance from coil (meters)

### 11.4 Shielding

To reduce EMF exposure:
- **Ferrite sheets**: Behind transmitter coil
- **Aluminum shield**: Around sensitive electronics
- **Distance**: Inverse-cube law (B ∝ 1/d³)
- **Power reduction**: Lower current reduces B field

---

## 12. EV Wireless Charging

### 12.1 SAE J2954 Standard

The SAE J2954 standard defines wireless power transfer for electric vehicles.

#### Power Classes

| Class | Power | Voltage | Use Case |
|-------|-------|---------|----------|
| WPT1 | 3.7 kW | 400V | PHEV, small BEV |
| WPT2 | 7.7 kW | 400V | Passenger BEV |
| WPT3 | 11 kW | 400V | Large BEV |
| WPT4 | 22 kW | 800V | Fast charging |

#### Ground Assembly (GA)
- **Coil size**: 500-800 mm diameter
- **Installation**: Embedded in pavement
- **Frequency**: 85 kHz
- **Efficiency**: 85-93%

#### Vehicle Assembly (VA)
- **Mounting**: Underside of vehicle
- **Ground clearance**: 100-250 mm
- **Alignment tolerance**: ±75 mm (WPT2), ±100 mm (WPT3)

### 12.2 Dynamic Wireless Charging

Charge vehicles while driving:
- **In-road coils**: Multiple coils embedded in lanes
- **High-speed switching**: Activate coils as vehicle passes
- **Power**: 20-100 kW per vehicle
- **Applications**: Electric buses, autonomous shuttles

### 12.3 EV Charging Efficiency

Target efficiency (GA to battery):
- **WPT2 (7.7 kW)**: >85%
- **WPT3 (11 kW)**: >88%
- **WPT4 (22 kW)**: >90%

---

## 13. Multi-Device Charging

### 13.1 Architectures

#### 13.1.1 Multi-Coil Array
- **Layout**: 3-9 coils in grid pattern
- **Control**: Activate coils under devices
- **Efficiency**: Each device gets dedicated coil

#### 13.1.2 Single Large Coil
- **Layout**: One coil covers entire surface
- **Control**: Power sharing among devices
- **Efficiency**: Lower due to shared field

#### 13.1.3 Guided Positioning
- **Layout**: 1-3 coils with mechanical guides
- **Control**: Users place devices in marked zones
- **Efficiency**: High due to alignment

### 13.2 Power Allocation

```
P_i = min(P_requested_i, P_max / N)
```

Where:
- `P_i` = Power allocated to device i (watts)
- `P_requested_i` = Power requested by device i
- `P_max` = Maximum charger output power
- `N` = Number of devices

### 13.3 Coil Multiplexing

**Time-division multiplexing**: Rapidly switch between coils
- **Switching frequency**: 10-100 Hz
- **Duty cycle**: Equal or priority-based
- **Advantage**: Simplified power electronics

---

## 14. Interoperability Testing

### 14.1 Qi Certification

WPC certification tests:
1. **Power transfer**: Verify power levels (BPP/EPP)
2. **Communication**: Packet structure and timing
3. **Efficiency**: Meet minimum efficiency targets
4. **FOD**: Detect standard metal objects
5. **EMI/EMC**: Electromagnetic compatibility
6. **Safety**: Temperature, overcurrent, overvoltage

### 14.2 AirFuel Certification

AirFuel Alliance tests:
1. **Resonant frequency**: 6.78 MHz ±5%
2. **BLE communication**: Device pairing and control
3. **Multi-device**: Simultaneous charging capability
4. **Safety**: EMF exposure, thermal limits

### 14.3 Cross-Platform Testing

Test devices on:
- Multiple charger brands
- Different power levels
- Various alignment offsets
- Temperature extremes (-10°C to 45°C)

---

## 15. Implementation Guidelines

### 15.1 Transmitter Design

1. **Select power level**: BPP (5W), EPP (15W), or higher
2. **Choose frequency**: 110 kHz (Qi) or 6.78 MHz (AirFuel)
3. **Design coil**: Calculate turns, wire gauge, ferrite
4. **Implement FOD**: Q-factor or power loss method
5. **Add temperature sensor**: NTC thermistor in coil
6. **Design power electronics**: Full-bridge inverter, controller
7. **Implement communication**: Demodulate backscatter (Qi) or BLE (AirFuel)

### 15.2 Receiver Design

1. **Select power level**: Match device requirements
2. **Design coil**: Fit within device form factor
3. **Add rectifier**: Synchronous or diode-based
4. **Implement voltage regulation**: Buck/boost converter
5. **Add protection**: Overvoltage, overcurrent, thermal
6. **Implement communication**: Load modulation (Qi) or BLE (AirFuel)

### 15.3 Safety Checklist

- [ ] FOD implemented and tested
- [ ] Temperature monitoring with cutoff
- [ ] Overcurrent protection
- [ ] Overvoltage protection
- [ ] EMF exposure within limits
- [ ] UL/CE/FCC certification obtained
- [ ] User manual with safety warnings

---

## 16. References

### Standards
- **Qi v1.3**: Wireless Power Consortium (WPC)
- **AirFuel Resonant v1.3**: AirFuel Alliance
- **SAE J2954**: Wireless Power Transfer for Light-Duty Plug-In/Electric Vehicles
- **ISO 19363**: Electrically propelled road vehicles — Magnetic field wireless power transfer
- **IEC 63028**: Wireless Power Transfer (WPT) - Air interface for electric vehicle charging

### Safety Standards
- **ICNIRP 2010**: Guidelines for Limiting Exposure to Time-Varying Electric and Magnetic Fields
- **IEEE C95.1-2019**: IEEE Standard for Safety Levels with Respect to Human Exposure to Electric, Magnetic, and Electromagnetic Fields
- **UL 2738**: Standard for Low Power Wireless Charging Equipment
- **IEC 62368-1**: Audio/video, information and communication technology equipment - Safety requirements

### Technical References
- Covic, G., & Boys, J. (2013). "Inductive Power Transfer". *IEEE Transactions on Industrial Electronics*.

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
