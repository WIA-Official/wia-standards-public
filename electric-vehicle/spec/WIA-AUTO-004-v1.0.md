# WIA-AUTO-004: Electric Vehicle Specification v1.0

> **Standard ID:** WIA-AUTO-004
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Automotive Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Electric Powertrain Architecture](#2-electric-powertrain-architecture)
3. [Battery Systems](#3-battery-systems)
4. [Electric Motor Technologies](#4-electric-motor-technologies)
5. [Power Electronics](#5-power-electronics)
6. [Regenerative Braking](#6-regenerative-braking)
7. [Thermal Management](#7-thermal-management)
8. [Range Calculation](#8-range-calculation)
9. [Energy Efficiency](#9-energy-efficiency)
10. [Charging Systems](#10-charging-systems)
11. [Data Formats](#11-data-formats)
12. [API Interface](#12-api-interface)
13. [Safety Protocols](#13-safety-protocols)
14. [References](#14-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines the comprehensive framework for electric vehicle (EV) technology, providing standardized methods for designing, analyzing, and optimizing electric powertrains. The standard covers all major components from battery systems to thermal management, enabling consistent implementation across the automotive industry.

### 1.2 Scope

The standard covers:
- Complete electric powertrain architecture and component integration
- Battery technologies (Li-ion, Solid-state, next-generation chemistries)
- Electric motor types (PMSM, Induction, SRM) with performance characteristics
- Power electronics design (inverters, converters, charging systems)
- Regenerative braking systems and energy recovery
- Thermal management for batteries, motors, and power electronics
- Range calculation methodologies and energy consumption modeling
- Charging infrastructure and protocols (AC, DC, wireless)
- Safety systems and fault protection

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to accelerate the global transition to sustainable electric transportation. By providing a unified, open framework for EV technology, we enable innovation, reduce development costs, and make clean transportation accessible to all, thereby reducing global carbon emissions and improving air quality for future generations.

### 1.4 Terminology

- **BEV**: Battery Electric Vehicle - fully electric, no combustion engine
- **PHEV**: Plug-in Hybrid Electric Vehicle - combination of electric and combustion
- **PMSM**: Permanent Magnet Synchronous Motor
- **IM**: Induction Motor (Asynchronous Motor)
- **SRM**: Switched Reluctance Motor
- **BMS**: Battery Management System
- **SoC**: State of Charge (% of battery capacity remaining)
- **SoH**: State of Health (% of original battery capacity)
- **DoD**: Depth of Discharge (% of battery capacity used)
- **IGBT**: Insulated Gate Bipolar Transistor (power switching device)
- **MOSFET**: Metal-Oxide-Semiconductor Field-Effect Transistor
- **V2G**: Vehicle-to-Grid (bidirectional energy flow)
- **V2V**: Vehicle-to-Vehicle communication
- **OBC**: On-Board Charger

---

## 2. Electric Powertrain Architecture

### 2.1 System Overview

The electric powertrain consists of the following primary components:

```
Battery Pack → BMS → DC/DC Converter → Inverter → Motor → Transmission → Wheels
                ↓
           On-Board Charger ← Charging Port
                ↓
           Accessories (HVAC, Lights, etc.)
```

### 2.2 Component Hierarchy

#### 2.2.1 Energy Storage Layer
- **Battery Pack**: Primary energy storage (typically 40-150 kWh for passenger vehicles)
- **Battery Management System (BMS)**: Monitors and controls battery pack
- **Thermal Management**: Liquid or air cooling/heating for optimal temperature

#### 2.2.2 Power Conversion Layer
- **DC/DC Converter**: Steps down high-voltage DC to 12V/48V for accessories
- **Inverter**: Converts DC to AC for motor drive
- **On-Board Charger**: Converts AC from grid to DC for battery

#### 2.2.3 Propulsion Layer
- **Electric Motor**: Converts electrical energy to mechanical energy
- **Transmission**: Single-speed or multi-speed gearbox (typically single-speed)
- **Differential**: Distributes power to wheels

#### 2.2.4 Control Layer
- **Vehicle Control Unit (VCU)**: Master controller for powertrain
- **Motor Controller**: Precision control of motor speed and torque
- **Battery Management System**: Cell balancing, SoC estimation

### 2.3 Power Flow

#### 2.3.1 Acceleration (Discharge Mode)
```
Battery (DC) → Inverter (DC→AC) → Motor (AC) → Wheels (Mechanical)
Efficiency: 85-95%
```

#### 2.3.2 Regenerative Braking (Charge Mode)
```
Wheels (Mechanical) → Motor (Generator) → Inverter (AC→DC) → Battery (DC)
Efficiency: 60-80%
```

#### 2.3.3 Charging Mode
```
Grid (AC) → On-Board Charger (AC→DC) → BMS → Battery Pack (DC)
AC Charging Efficiency: 85-95%
DC Fast Charging Efficiency: 90-95%
```

### 2.4 Voltage Architecture

| Component | Typical Voltage | Range |
|-----------|-----------------|-------|
| Battery Pack | 400V | 300-800V |
| High-Voltage Bus | 400V | 300-800V |
| DC/DC Output | 12V/48V | 12-48V |
| Motor Drive | 0-400V AC | Variable |
| AC Charging | 240V AC | 120-240V |
| DC Fast Charging | 400-800V DC | 200-920V |

---

## 3. Battery Systems

### 3.1 Lithium-Ion Battery Technologies

#### 3.1.1 Nickel Manganese Cobalt (NMC)
```
Cathode: LiNiₓMnᵧCoₖO₂ (x+y+z=1)
Common ratios: NMC 111, NMC 532, NMC 622, NMC 811
```

**Characteristics**:
- Energy density: 200-260 Wh/kg (cell level)
- Voltage: 3.6-3.7V nominal
- Cycle life: 1000-2000 cycles (80% SoH)
- Cost: Moderate ($100-150/kWh)
- Applications: Most passenger EVs

**Advantages**:
- Balanced energy and power density
- Good cycle life
- Proven technology

**Disadvantages**:
- Thermal stability concerns (especially NMC 811)
- Cobalt supply chain issues
- Moderate cost

#### 3.1.2 Lithium Iron Phosphate (LFP)
```
Cathode: LiFePO₄
```

**Characteristics**:
- Energy density: 150-180 Wh/kg (cell level)
- Voltage: 3.2V nominal
- Cycle life: 2000-4000 cycles (80% SoH)
- Cost: Low ($60-90/kWh)
- Applications: Budget EVs, commercial vehicles, stationary storage

**Advantages**:
- Excellent thermal stability and safety
- Very long cycle life
- Low cost
- No cobalt

**Disadvantages**:
- Lower energy density
- Poor cold weather performance
- Flat discharge curve (difficult SoC estimation)

#### 3.1.3 Nickel Cobalt Aluminum (NCA)
```
Cathode: LiNiCoAlO₂
Typical: 80% Ni, 15% Co, 5% Al
```

**Characteristics**:
- Energy density: 220-280 Wh/kg (cell level)
- Voltage: 3.6V nominal
- Cycle life: 500-1000 cycles (80% SoH)
- Cost: High ($120-180/kWh)
- Applications: Tesla Model S/X, premium vehicles

**Advantages**:
- Highest energy density of commercial Li-ion
- Good power density
- Proven at scale (Tesla)

**Disadvantages**:
- Lower cycle life
- Thermal stability concerns
- Higher cost

### 3.2 Next-Generation Battery Technologies

#### 3.2.1 Solid-State Batteries
```
Electrolyte: Solid ceramic or polymer (vs. liquid organic electrolyte)
Anode: Lithium metal (vs. graphite)
```

**Projected Characteristics**:
- Energy density: 400-500 Wh/kg (cell level)
- Voltage: Similar to Li-ion
- Cycle life: 5000+ cycles
- Cost: Very high initially ($200-400/kWh)
- Timeline: 2025-2030 for commercial production

**Advantages**:
- 2-3× energy density of current Li-ion
- Significantly improved safety (non-flammable)
- Faster charging potential
- Wider temperature range

**Disadvantages**:
- Currently very expensive
- Manufacturing challenges
- Interface resistance issues
- Limited commercial availability

#### 3.2.2 Lithium-Sulfur (Li-S)
```
Cathode: Sulfur
Anode: Lithium metal
```

**Projected Characteristics**:
- Energy density: 400-600 Wh/kg (theoretical 2600 Wh/kg)
- Cycle life: 200-500 cycles (current limitation)
- Cost: Potentially very low (sulfur is abundant)

**Status**: Research phase, 5-10 years from commercialization

#### 3.2.3 Lithium-Air (Li-Air)
```
Cathode: Porous carbon (oxygen from air)
Anode: Lithium metal
```

**Theoretical Characteristics**:
- Energy density: 800-1200 Wh/kg (theoretical ~3500 Wh/kg)
- Could match gasoline energy density

**Status**: Early research, 10-20 years from commercialization

### 3.3 Battery Pack Design

#### 3.3.1 Cell Formats

**Cylindrical Cells** (e.g., 18650, 21700, 4680):
- Form factor: 18mm × 65mm, 21mm × 70mm, 46mm × 80mm
- Energy: 3-25 Wh per cell
- Pros: Proven manufacturing, good thermal management, mechanical strength
- Cons: Complex pack assembly, many connections
- Users: Tesla, Panasonic

**Pouch Cells**:
- Form factor: Flat rectangular pouch, customizable
- Energy: 20-60 Wh per cell
- Pros: High energy density, flexible form factor, lightweight
- Cons: Requires external support, potential for swelling
- Users: LG, Samsung, SK

**Prismatic Cells**:
- Form factor: Hard-cased rectangular, standardized sizes
- Energy: 20-100 Wh per cell
- Pros: Easy pack integration, rigid structure
- Cons: Lower energy density, limited cooling paths
- Users: BYD, CATL, Panasonic

#### 3.3.2 Pack Configuration

**Module-based Architecture**:
```
Cells → Modules → Pack
Typical: 6-12 modules per pack, 10-30 cells per module
```

**Cell-to-Pack (CTP)**:
```
Cells → Pack (no modules)
Advantages: 10-15% higher energy density, lower cost
Pioneered by: BYD, CATL
```

**Cell-to-Body (CTB)**:
```
Cells integrated directly into vehicle structure
Advantages: Maximum space efficiency, structural battery
Example: Tesla 4680 structural pack
```

#### 3.3.3 Battery Pack Specifications

For a typical 75 kWh pack:

```
Configuration: 96 series × 4 parallel (96s4p)
Cell type: Cylindrical 21700 NMC
Cell capacity: 5.0 Ah
Cell voltage: 3.7V nominal (4.2V max, 2.5V min)
Cell energy: 18.5 Wh

Total cells: 384 cells
Pack voltage: 355V nominal (403V max, 240V min)
Pack capacity: 211 Ah
Pack energy: 75 kWh
Pack weight: ~450 kg
Energy density: 167 Wh/kg (pack level)
```

### 3.4 Battery Management System (BMS)

#### 3.4.1 Core Functions

**1. Cell Monitoring**:
- Voltage measurement (±5 mV accuracy)
- Current measurement (±0.5% accuracy)
- Temperature monitoring (multiple sensors per module)
- Isolation resistance monitoring

**2. State Estimation**:
```python
# State of Charge (SoC) estimation
SoC(t) = SoC(0) - ∫(I(t)/C_nom)dt + Error_correction

# Coulomb counting with voltage-based correction
SoC = (OCV⁻¹(V_measured) + ∫I dt) / 2

# State of Health (SoH) estimation
SoH = (C_current / C_rated) × 100%
```

**3. Cell Balancing**:

**Passive Balancing**:
```
Dissipate excess energy from higher-voltage cells via resistors
Power: 50-200 mW per cell
Balancing current: 50-200 mA
Efficiency: ~0% (energy wasted as heat)
```

**Active Balancing**:
```
Transfer energy from higher to lower voltage cells
Methods: Capacitor-based, inductor-based, transformer-based
Efficiency: 85-95%
Balancing current: 200-1000 mA
```

**4. Thermal Management**:
- Monitor cell/module temperatures
- Control cooling/heating systems
- Prevent thermal runaway propagation

**5. Safety Protection**:
- Over-voltage protection (per cell and pack)
- Under-voltage protection
- Over-current protection (charge and discharge)
- Over-temperature protection
- Short-circuit protection
- Isolation fault detection

#### 3.4.2 BMS Architecture

**Centralized BMS**:
- Single ECU for entire pack
- Pros: Lower cost, simpler communication
- Cons: Complex wiring, single point of failure

**Distributed BMS**:
- Module-level controllers + master controller
- Pros: Modular, fault-tolerant, shorter wires
- Cons: Higher cost, more complex

**Modular BMS**:
- Cell monitoring units + communication daisy chain
- Pros: Scalable, flexible
- Most common in modern EVs

### 3.5 Battery Performance Characteristics

#### 3.5.1 Energy and Power Density

```
Gravimetric Energy Density (Wh/kg):
- Cell level: 150-280 Wh/kg (current Li-ion)
- Module level: 120-220 Wh/kg (70-85% of cell)
- Pack level: 100-180 Wh/kg (60-75% of cell)

Volumetric Energy Density (Wh/L):
- Cell level: 400-700 Wh/L
- Pack level: 250-450 Wh/L

Power Density (W/kg):
- Continuous: 200-400 W/kg
- Peak (10s): 600-1200 W/kg
```

#### 3.5.2 Charge and Discharge Characteristics

**C-Rate Definition**:
```
C-rate = Current / Rated Capacity
Example: 75 kWh battery
- 1C = 75 kW (charge or discharge)
- 0.5C = 37.5 kW
- 2C = 150 kW
```

**Typical Limits**:
```
Maximum Continuous Discharge: 2-4C
Peak Discharge (10s): 4-8C
Maximum Continuous Charge: 0.5-1C
Peak Charge (DC fast): 1.5-3C (with thermal management)
```

**Voltage-SoC Relationship** (NMC):
```
100% SoC → 4.2V per cell
90% SoC → 4.1V
50% SoC → 3.7V
10% SoC → 3.3V
0% SoC → 2.5V (cutoff)
```

#### 3.5.3 Battery Degradation

**Calendar Aging**:
```
Capacity Loss Rate = k × exp(-Ea / RT) × exp(β × SoC)

Where:
- k = pre-exponential factor
- Ea = activation energy
- R = gas constant
- T = temperature (K)
- β = SoC stress factor

Typical: 2-3% capacity loss per year at optimal storage (50% SoC, 20°C)
Accelerated at: High SoC, high temperature
```

**Cycle Aging**:
```
Capacity Loss = α × (Cycles)^β

Where:
- α = degradation coefficient (depends on DoD, temperature)
- β = aging exponent (typically 0.5-0.7)

Typical: 20% capacity loss after 1000-2000 full cycles (NMC)
```

**Factors Affecting Degradation**:
1. Temperature: Accelerated aging above 30°C
2. State of Charge: Faster aging at high SoC (>80%) or low SoC (<20%)
3. Depth of Discharge: Deeper cycles → faster aging
4. Charge/Discharge Rate: High C-rates → faster aging
5. Charge End Voltage: Lower max voltage → longer life
6. Number of Cycles: Linear or power-law relationship

**Degradation Mitigation**:
- Limit usable SoC range (e.g., 10-90% buffer)
- Active thermal management (keep 20-35°C)
- Limit DC fast charging frequency
- Avoid constant full charge or full discharge
- Reduce calendar aging with storage at 50% SoC

---

## 4. Electric Motor Technologies

### 4.1 Permanent Magnet Synchronous Motor (PMSM)

#### 4.1.1 Operating Principle

```
Stator: 3-phase AC windings creating rotating magnetic field
Rotor: Permanent magnets (typically NdFeB - Neodymium-Iron-Boron)
Synchronization: Rotor speed = synchronous speed (no slip)
```

**Torque Equation**:
```
T = (3/2) × p × ψ_m × I_q

Where:
- p = pole pairs
- ψ_m = magnet flux linkage
- I_q = quadrature axis current (torque-producing component)
```

**Power Equation**:
```
P = T × ω = T × 2πN / 60

Where:
- ω = angular velocity (rad/s)
- N = rotational speed (RPM)
```

#### 4.1.2 Characteristics

**Advantages**:
- High efficiency: 95-97% (across wide operating range)
- High power density: 5-15 kW/kg
- Excellent low-speed torque (no field weakening needed below base speed)
- Compact and lightweight
- Low rotor losses (no rotor currents)

**Disadvantages**:
- Higher cost due to rare earth magnets (Nd, Dy)
- Risk of demagnetization at high temperature (>150°C)
- Limited field weakening range (typically 2-3× base speed)
- Back-EMF at high speed (safety concern if inverter fails)

**Applications**:
- Most passenger EVs (Nissan Leaf, Chevy Bolt, BMW i3, etc.)
- Premium vehicles (Mercedes EQ, Audi e-tron)
- High-efficiency applications

**Typical Specifications** (150 kW PMSM):
```
Rated Power: 150 kW (continuous)
Peak Power: 200-250 kW (10s)
Rated Torque: 310 N·m @ 4,600 RPM
Peak Torque: 400 N·m @ 0-4,000 RPM
Max Speed: 14,000 RPM
Efficiency: 96% @ rated point
Power Density: 8-12 kW/kg
```

### 4.2 Induction Motor (IM)

#### 4.2.1 Operating Principle

```
Stator: 3-phase AC windings creating rotating magnetic field
Rotor: Squirrel cage (aluminum or copper bars)
Slip: Rotor speed = synchronous speed × (1 - s), where s = slip (1-5%)
```

**Torque Equation**:
```
T = (3 × p / ω_s) × (R_r' / s) / [(R_s + R_r'/s)² + (X_s + X_r')²] × V_s²

Simplified:
T ≈ k × s × V_s² / R_r

Where:
- p = pole pairs
- s = slip
- R_r = rotor resistance
- V_s = stator voltage
```

#### 4.2.2 Characteristics

**Advantages**:
- Lower cost (no rare earth magnets)
- Robust and reliable
- Excellent field weakening (4-5× base speed)
- No back-EMF safety issue
- High-speed capability
- Wide constant power range

**Disadvantages**:
- Lower efficiency: 90-93% (due to rotor losses)
- Heavier and larger (vs. PMSM of same power)
- Requires magnetizing current (lower power factor)
- More complex control

**Applications**:
- Tesla Model S, Model X (prior to 2021)
- Commercial vehicles
- High-speed applications
- Cost-sensitive applications

**Typical Specifications** (150 kW IM):
```
Rated Power: 150 kW (continuous)
Peak Power: 250-300 kW (10s)
Rated Torque: 300 N·m @ 4,800 RPM
Peak Torque: 440 N·m @ 0-3,500 RPM
Max Speed: 18,000 RPM
Efficiency: 92% @ rated point
Power Density: 5-8 kW/kg
```

### 4.3 Switched Reluctance Motor (SRM)

#### 4.3.1 Operating Principle

```
Stator: Concentrated windings on salient poles
Rotor: Salient poles (simple iron laminations, no windings or magnets)
Operation: Sequential energization of stator phases
Torque: Produced by magnetic reluctance (rotor alignment)
```

**Torque Equation**:
```
T = (1/2) × I² × dL/dθ

Where:
- I = phase current
- L = phase inductance
- θ = rotor position
```

#### 4.3.2 Characteristics

**Advantages**:
- Very low cost (simple rotor, no magnets)
- Robust construction
- Fault-tolerant (independent phases)
- High-temperature capability
- No rare earth materials

**Disadvantages**:
- High torque ripple (noise and vibration)
- Complex control
- Lower power density
- Acoustic noise issues
- Requires rotor position sensor

**Applications**:
- Budget EVs (in development)
- Research vehicles
- Industrial drives

**Status**: Limited commercial adoption in EVs, ongoing research

### 4.4 Motor Performance Comparison

| Parameter | PMSM | IM | SRM |
|-----------|------|-----|-----|
| Efficiency | 95-97% | 90-93% | 85-90% |
| Power Density | 8-12 kW/kg | 5-8 kW/kg | 3-6 kW/kg |
| Cost | High | Medium | Low |
| Rare Earth | Yes (Nd, Dy) | No | No |
| Field Weakening | 2-3× | 4-5× | 3-4× |
| Torque Ripple | Low (<2%) | Low (<2%) | High (10-30%) |
| Noise | Low | Low | High |
| Control Complexity | Medium | High | Very High |
| Reliability | High | Very High | High |
| Maturity | Mature | Mature | Emerging |

### 4.5 Motor Control

#### 4.5.1 Field-Oriented Control (FOC)

**Principle**: Decouple torque and flux control using Park transformation

**Transformation Sequence**:
```
3-phase ABC → 2-phase αβ (Clarke) → 2-phase dq (Park)

Clarke Transform:
I_α = I_a
I_β = (2×I_b + I_a) / √3

Park Transform:
I_d = I_α×cos(θ) + I_β×sin(θ)  (flux component)
I_q = -I_α×sin(θ) + I_β×cos(θ) (torque component)
```

**Control Loop**:
```
Torque Command → I_q Reference → PI Controller → V_q
Flux Command → I_d Reference → PI Controller → V_d
→ Inverse Park → Inverse Clarke → PWM → Inverter
```

#### 4.5.2 Maximum Torque Per Ampere (MTPA)

For PMSM:
```
Optimal I_d and I_q to maximize torque for given stator current

I_d = -(ψ_m / 2×L_d) + √[(ψ_m / 2×L_d)² + I_s²]
I_q = √(I_s² - I_d²)
```

#### 4.5.3 Field Weakening

At high speed, back-EMF limits voltage. Reduce flux to extend speed range:

```
Base Speed: ω_base = V_max / ψ_m
Field Weakening: I_d < 0 (negative d-axis current)
Max Speed: ω_max = 2-5 × ω_base (depending on motor type)
```

**Constant Power Region**:
```
P = T × ω = constant
T ∝ 1/ω (torque decreases inversely with speed)
```

---

## 5. Power Electronics

### 5.1 Inverter (DC/AC Converter)

#### 5.1.1 Topology

**3-Phase 2-Level Voltage Source Inverter** (most common):
```
Configuration: 6 switches (IGBTs or MOSFETs) + 6 freewheeling diodes
Input: DC from battery (300-800V)
Output: 3-phase AC (variable voltage and frequency)
Switching Frequency: 10-20 kHz
```

**Power Module Components**:
- Power switches: IGBT (for high voltage >650V) or SiC MOSFET (next-gen)
- Freewheeling diodes: Fast recovery diodes or SiC Schottky
- Gate drivers: Isolated drivers for each switch
- DC-link capacitor: Filter and energy storage (100-500 µF)

#### 5.1.2 Space Vector Modulation (SVM)

**Principle**: Generate 3-phase AC by switching between 8 voltage vectors

**Switching States**:
```
8 vectors: V0, V1, V2, V3, V4, V5, V6, V7
- V0, V7: Zero vectors (all switches on or off)
- V1-V6: Active vectors (at 60° intervals)
```

**Duty Cycle Calculation**:
```
T_a = T_s × m × sin(π/3 - θ)
T_b = T_s × m × sin(θ)
T_0 = T_s - T_a - T_b

Where:
- T_s = switching period
- m = modulation index (0-1)
- θ = angle within sector (0-60°)
```

**Modulation Index**:
```
m = V_out / (V_dc / √3)

Maximum without overmodulation: m = 0.907
```

#### 5.1.3 Switching Losses

**Conduction Losses**:
```
P_cond = I_rms² × R_on × D

Where:
- I_rms = RMS current
- R_on = on-state resistance
- D = duty cycle
```

**Switching Losses**:
```
P_sw = f_sw × (E_on + E_off)

Where:
- f_sw = switching frequency
- E_on = turn-on energy loss
- E_off = turn-off energy loss
```

**Total Inverter Efficiency**:
```
η = P_out / (P_out + P_cond + P_sw + P_driver)
Typical: 95-98% for Si IGBT, 98-99% for SiC MOSFET
```

### 5.2 DC/DC Converter

#### 5.2.1 High-Voltage to Low-Voltage (HV→12V/48V)

**Topology**: Isolated DC/DC converter (flyback, full-bridge, LLC resonant)

**Specifications**:
```
Input: 300-800V DC (from HV battery)
Output: 12-14V DC (for accessories)
Power: 2-3 kW (continuous)
Efficiency: 90-95%
Isolation: >1000V
```

**Applications**:
- 12V battery charging
- Accessories (lights, infotainment, controls)
- Replaces alternator in conventional vehicle

#### 5.2.2 Bidirectional DC/DC Converter

**Topology**: Dual active bridge (DAB)

**Applications**:
- 48V hybrid systems
- V2L (Vehicle-to-Load) applications
- Auxiliary battery management

### 5.3 On-Board Charger (OBC)

#### 5.3.1 AC/DC Conversion

**Topology**: Power Factor Correction (PFC) + DC/DC converter

**Stages**:
```
1. AC Input Filter: EMI suppression
2. PFC Stage: Boost converter (unity power factor)
3. DC/DC Stage: Isolated converter (output regulation)
4. Output Filter: Ripple reduction
```

**Specifications**:
```
Input: 120-240V AC, 50/60 Hz (single-phase)
       208-480V AC (3-phase for high-power OBC)
Output: 300-800V DC (to battery)
Power: 3.3-22 kW (single-phase), up to 50 kW (3-phase)
Efficiency: 92-95%
Power Factor: >0.99
THD: <5%
```

**Charging Profiles**:
```
1. Constant Current (CC): I = I_max until V = V_max
2. Constant Voltage (CV): V = V_max, I decreases exponentially
3. Taper: Transition from CC to CV

Typical: CC to 80% SoC, CV from 80-100%
```

### 5.4 Next-Generation Power Semiconductors

#### 5.4.1 Silicon Carbide (SiC)

**Advantages over Si**:
- Higher breakdown voltage: 10× vs. Si
- Lower on-resistance: 50% reduction
- Higher switching frequency: 50-100 kHz vs. 10-20 kHz
- Higher temperature operation: 200°C vs. 150°C
- Lower switching losses: 75% reduction

**Impact on EV**:
- Inverter efficiency: 98-99% vs. 95-97% (Si)
- Size reduction: 40-50% smaller and lighter
- Range improvement: 5-10% from lower losses
- Cost: Currently 2-3× Si, decreasing rapidly

**Adoption**:
- Tesla Model 3/Y (SiC inverter)
- Lucid Air
- Most new EVs from 2023+

#### 5.4.2 Gallium Nitride (GaN)

**Characteristics**:
- Even lower losses than SiC
- Very high switching frequency: 100-500 kHz
- Lower cost potential than SiC
- Currently limited to lower voltages (<650V)

**EV Applications**:
- On-board chargers
- DC/DC converters
- 48V systems
- Future: HV inverters (in development)

---

## 6. Regenerative Braking

### 6.1 Operating Principle

**Concept**: Electric motor operates as generator during deceleration

**Energy Flow**:
```
Kinetic Energy → Motor (Generator) → Inverter (Rectifier) → Battery
```

**Torque Control**:
```
Regenerative Torque = min(T_requested, T_motor_limit, T_battery_limit)

Where:
- T_requested: Driver brake input
- T_motor_limit: Max generator torque at current speed
- T_battery_limit: Max charge current × motor constant
```

### 6.2 Energy Recovery

#### 6.2.1 Kinetic Energy Available

```
E_kinetic = (1/2) × m × v²

Where:
- m = vehicle mass (kg)
- v = velocity (m/s)

Example (1800 kg vehicle, 100 km/h → 0):
E_kinetic = 0.5 × 1800 × (100/3.6)²
         = 0.5 × 1800 × 27.78²
         = 694,440 J = 0.193 kWh
```

#### 6.2.2 Recoverable Energy

```
E_recovered = E_kinetic × η_motor × η_inverter × η_battery

Typical efficiencies:
- Motor (as generator): 85-90%
- Inverter (AC→DC): 95-97%
- Battery charging: 95-98%
- Overall: 77-85%

Example:
E_recovered = 0.193 × 0.85 × 0.96 × 0.96
            = 0.151 kWh (78% recovery)
```

#### 6.2.3 Limitations

**Motor Speed Limits**:
- At very low speed (<5 km/h), motor torque insufficient → friction brakes engage
- At very high speed, battery current limit → reduced regen torque

**Battery Charge Limits**:
- At high SoC (>90%), charge current limited → reduced regen
- At low temperature (<0°C), charge acceptance reduced → limited regen
- C-rate limit (typically 0.5-1C continuous)

**Power Electronics Limits**:
- Maximum inverter current
- DC-link voltage limits

### 6.3 Blended Braking

**Coordination**: Seamless blend between regenerative and friction braking

**Strategies**:

**1. Serial Braking** (most common):
```
if (Brake_Request < Regen_Capacity):
    Friction_Brake = 0
    Regen_Brake = Brake_Request
else:
    Regen_Brake = Regen_Capacity
    Friction_Brake = Brake_Request - Regen_Capacity
```

**2. Parallel Braking**:
```
Regen_Brake = k × Brake_Request
Friction_Brake = (1-k) × Brake_Request

Where k varies with speed, SoC, temperature
```

**3. One-Pedal Driving**:
```
Accelerator_Lifted → Strong_Regen (up to 0.2-0.3g decel)
Brake_Pedal → Additional friction braking if needed

Allows driving with minimal brake pedal use in city
```

### 6.4 Regenerative Braking Efficiency

**Real-World Energy Recovery**:
```
Urban Driving: 15-25% of total energy recovered
Highway Driving: 5-10% (less braking)
Mixed Driving: 10-20%
```

**Range Extension**:
```
EPA City Range Improvement: 20-30% vs. no regen
EPA Highway Range Improvement: 5-10% vs. no regen
```

**Factors Affecting Recovery**:
- Driving pattern (stop-and-go vs. steady speed)
- Battery SoC (limited at high SoC)
- Temperature (limited in cold weather)
- Motor/inverter efficiency
- Driver behavior (smooth vs. aggressive braking)

---

## 7. Thermal Management

### 7.1 Battery Thermal Management

#### 7.1.1 Temperature Requirements

**Optimal Operating Range**:
```
Ideal: 20-35°C
Acceptable: 15-45°C
Critical: >60°C (risk of damage)
           <0°C (reduced performance and capacity)
```

**Temperature Effects**:
```
Capacity:
- At -20°C: 50-70% of rated capacity
- At 0°C: 80-90% of rated capacity
- At 25°C: 100% (nominal)
- At 45°C: 100% but accelerated degradation

Resistance:
- At -20°C: 3-5× nominal
- At 0°C: 1.5-2× nominal
- At 25°C: 1× (nominal)
- At 45°C: 0.9-1× nominal

Power:
- At -20°C: 30-50% of rated power
- At 0°C: 60-80% of rated power
- At 25°C: 100% (nominal)
- At 45°C: 100% but thermal limits may apply
```

#### 7.1.2 Cooling Methods

**1. Passive Air Cooling**:
```
Method: Natural convection or forced air flow
Heat Transfer: Air → Cell surface
Thermal Resistance: 20-50 K/W

Pros:
- Simple, low cost
- No additional power consumption (natural)
- Minimal complexity

Cons:
- Poor heat transfer (air: 0.026 W/m·K)
- Uneven temperature distribution
- Insufficient for high-power applications

Applications:
- Low-power EVs (e.g., early Nissan Leaf)
- Mild climates
```

**2. Active Air Cooling**:
```
Method: Forced air circulation with fans/blowers
Heat Transfer: Air (forced) → Cell surface
Thermal Resistance: 10-30 K/W
Power Consumption: 100-500 W

Improvements over passive:
- Better heat transfer (forced convection)
- Controllable cooling rate
- Lower cost than liquid

Limitations:
- Still limited heat transfer
- Fan noise
- Not sufficient for fast charging

Applications:
- Budget EVs
- Mild performance requirements
```

**3. Liquid Cooling**:
```
Method: Coolant circulation (water-glycol, 50:50 mix)
Heat Transfer: Coolant → Cold plate → Cell
Thermal Resistance: 1-5 K/W
Power Consumption: 200-1000 W (pump + radiator fan)
Coolant: 50% water, 50% glycol (freezing point: -37°C)
Flow Rate: 5-20 L/min

Cooling Plate Designs:
a) Serpentine channels
b) Parallel channels
c) Mini-channel heat exchanger

Pros:
- Excellent heat transfer (water: 0.6 W/m·K)
- Uniform temperature distribution
- Enables fast charging
- Heating capability (with PTC or heat pump)

Cons:
- Higher cost and complexity
- Additional weight (~10-20 kg)
- Risk of coolant leakage
- Requires maintenance

Applications:
- Most modern EVs
- High-performance and long-range vehicles
- DC fast charging capable
```

**4. Refrigerant Cooling** (Direct Refrigeration):
```
Method: Refrigerant (R134a or R1234yf) phase change cooling
Heat Transfer: Refrigerant evaporation → Cell
Thermal Resistance: 0.5-2 K/W
COP (Coefficient of Performance): 2-4

Pros:
- Highest cooling performance
- Can achieve sub-ambient temperatures
- Very uniform temperature

Cons:
- Highest complexity and cost
- Shares AC system (may impact cabin comfort)
- Higher power consumption

Applications:
- High-performance EVs (some Tesla, Porsche Taycan)
- Extreme climates (very hot regions)
```

**5. Phase Change Material (PCM)**:
```
Method: PCM (wax, salt hydrate) absorbs heat via phase transition
Phase Change Temperature: 25-35°C (matched to optimal battery temp)
Latent Heat: 150-250 kJ/kg

Pros:
- Passive thermal buffering
- No power consumption
- Uniform temperature during phase change
- Compact

Cons:
- Limited heat capacity (saturates)
- Requires external cooling to reset
- Added weight
- Not standalone solution

Applications:
- Supplementary to active cooling
- Racing/high-performance (thermal buffering during sprint)
```

#### 7.1.3 Heating Methods

**Cold Weather Challenges**:
- Reduced battery capacity and power
- Reduced charging acceptance
- Increased internal resistance

**Heating Solutions**:

**1. Resistive Heating** (PTC - Positive Temperature Coefficient):
```
Method: Electric resistance heater in coolant loop
Power: 3-7 kW
Efficiency: ~100% (electrical → heat)
Energy Consumption: High (from battery)

Pros:
- Simple, reliable
- Fast heating
- Low cost

Cons:
- High energy consumption (reduces range 20-40% in cold weather)
- Inefficient

Applications:
- Most current EVs
```

**2. Heat Pump**:
```
Method: Reverse refrigeration cycle
COP: 2-4 (2-4× more efficient than resistive)
Power: 1-3 kW electrical input → 2-12 kW heat output
Operating Range: Down to -10°C to -20°C (ambient)

Pros:
- 2-4× more efficient than resistive
- Can also cool
- Reduces range loss in cold weather

Cons:
- Higher cost
- Complexity
- Reduced COP at very low ambient temperatures

Applications:
- Premium EVs (Tesla Model Y, Nissan Ariya, Hyundai Ioniq 5)
- Cold climate markets
```

**3. Coolant Pre-conditioning**:
```
Method: Heat battery while plugged in (grid power)
Timing: Before departure (scheduled or remote activated)

Pros:
- Zero impact on driving range
- Battery at optimal temp for departure
- Enables faster charging

Cons:
- Requires grid connection
- Time required (20-60 min)
```

### 7.2 Motor Thermal Management

#### 7.2.1 Heat Generation

**Losses in Motor**:
```
Total Loss = Copper Loss + Iron Loss + Mechanical Loss + Stray Loss

Copper Loss (I²R):
P_copper = 3 × I²_rms × R_phase
Dominant at low speed (high current)

Iron Loss (hysteresis + eddy current):
P_iron = k_h × f × B² + k_e × f² × B²
Dominant at high speed (high frequency)

Mechanical Loss:
P_mech = friction + windage
Increases with speed

Typical Distribution at Rated Load:
- Copper loss: 50-60%
- Iron loss: 20-30%
- Mechanical loss: 10-15%
- Stray loss: 5-10%
```

**Heat Distribution**:
```
Winding (hottest): 80-120°C (continuous)
                   150-180°C (peak, short-term)
Magnets (PMSM): 80-120°C (risk of demagnetization >150°C)
Stator core: 60-100°C
Rotor: 60-100°C
```

#### 7.2.2 Cooling Methods

**Liquid Cooling** (most common):
```
Method: Coolant jacket around stator
Coolant: Water-glycol (same as battery, often shared loop)
Flow Rate: 3-10 L/min
Thermal Resistance: 0.1-0.5 K/W
Power Removal: 5-15 kW (at rated load)

Cooling Paths:
1. Stator jacket cooling (most common)
2. Oil spray cooling on windings (high-performance)
3. Shaft cooling (water channels through hollow shaft)
```

**Oil Cooling** (next-gen):
```
Method: Direct oil spray on windings (ATF - Automatic Transmission Fluid)
Advantages:
- Direct cooling of hottest components (windings)
- Better heat transfer than water jacket
- Also lubricates bearings
- Enables higher power density

Disadvantages:
- Requires oil filtration and cooling
- More complex sealing
- Higher cost

Applications:
- High-performance EVs (Porsche Taycan)
- Race applications
```

### 7.3 Power Electronics Thermal Management

**Critical Components**:
- IGBTs/MOSFETs: Junction temperature <150°C (Si), <175°C (SiC)
- Diodes: Similar limits as switches
- Capacitors: Temperature-sensitive (lifetime halves per 10°C increase)

**Cooling Methods**:
```
Liquid Cold Plate:
- Integrated into power module baseplate
- Coolant: Water-glycol
- Thermal Interface: Thermal grease or pad (0.5-2 K/W)
- Heat Sink: Aluminum cold plate with micro-channels
- Thermal Resistance: Junction to coolant: 0.1-0.5 K/W
```

**Power Derating**:
```
If (T_junction > T_max - margin):
    P_max = P_rated × (T_max - T_ambient) / (T_junction - T_ambient)
```

### 7.4 Integrated Thermal Management

**System Integration**:
```
Components sharing cooling loops:
1. Battery + Motor + Inverter (most common)
2. Separate battery loop (if different temp requirements)
3. Cabin HVAC integration (heat pump systems)
```

**Coolant Loops**:

**Single Loop** (simple):
```
Battery → Motor → Inverter → Radiator → Pump → Battery
Pros: Simple, low cost
Cons: Compromised temperatures, single point of failure
```

**Dual Loop** (common):
```
Loop 1: Battery → Battery Chiller/Heater → Pump → Battery
Loop 2: Motor + Inverter → Radiator → Pump → Motor/Inverter
Pros: Optimized temperatures for each component
Cons: Higher complexity and cost
```

**Integrated with HVAC** (premium):
```
Heat Pump System:
- Summer: Battery/Motor heat → Rejected to ambient, Cabin cooled
- Winter: Ambient heat + Waste heat → Battery/Motor warming, Cabin heating
Advantages: Maximum efficiency, fast warm-up
Complexity: High
Examples: Tesla Model Y, BMW iX
```

---

## 8. Range Calculation

### 8.1 Fundamental Range Equation

```
Range (km) = (E_battery × DoD × η_total) / E_consumption

Where:
- E_battery = Total battery capacity (kWh)
- DoD = Usable Depth of Discharge (0.8-0.95)
- η_total = Overall drivetrain efficiency (0.85-0.95)
- E_consumption = Energy consumption per km (kWh/km)
```

**Example** (75 kWh battery, 0.18 kWh/km consumption):
```
Range = (75 × 0.9 × 0.92) / 0.18
      = 62.1 / 0.18
      = 345 km
```

### 8.2 Energy Consumption Modeling

#### 8.2.1 Driving Resistance Forces

**Total Resistance**:
```
F_total = F_aero + F_roll + F_grade + F_accel

Where:
- F_aero = Aerodynamic drag
- F_roll = Rolling resistance
- F_grade = Gravitational resistance on slope
- F_accel = Inertial resistance during acceleration
```

**1. Aerodynamic Drag**:
```
F_aero = 0.5 × ρ × C_d × A × v²

Where:
- ρ = Air density (1.225 kg/m³ at sea level, 15°C)
- C_d = Drag coefficient (0.20-0.35 for EVs)
- A = Frontal area (1.8-2.5 m² for sedans)
- v = Velocity (m/s)

Power required:
P_aero = F_aero × v = 0.5 × ρ × C_d × A × v³

Note: Power increases with cube of velocity!

Example (v = 120 km/h = 33.3 m/s, C_d = 0.24, A = 2.3 m²):
F_aero = 0.5 × 1.225 × 0.24 × 2.3 × 33.3²
       = 313 N
P_aero = 313 × 33.3 = 10.4 kW
```

**2. Rolling Resistance**:
```
F_roll = C_rr × m × g × cos(α)

Where:
- C_rr = Rolling resistance coefficient (0.006-0.012 for EVs)
- m = Vehicle mass (kg)
- g = Gravitational acceleration (9.81 m/s²)
- α = Road grade angle (degrees)

Power required:
P_roll = F_roll × v

Example (m = 1800 kg, C_rr = 0.008, flat road):
F_roll = 0.008 × 1800 × 9.81 × 1
       = 141 N
P_roll (at 120 km/h) = 141 × 33.3 = 4.7 kW
```

**3. Grade Resistance**:
```
F_grade = m × g × sin(α)

Where:
- α = Road grade angle

For small angles: sin(α) ≈ grade (%)
Example: 5% grade ≈ sin(2.86°) ≈ 0.05

Power required:
P_grade = F_grade × v

Example (m = 1800 kg, 5% grade, 50 km/h):
F_grade = 1800 × 9.81 × 0.05 = 883 N
P_grade = 883 × (50/3.6) = 12.3 kW
```

**4. Acceleration Force**:
```
F_accel = m_eff × a

Where:
- m_eff = m × (1 + k_rot)
- k_rot = Rotational inertia factor (0.05-0.1 for EVs)
- a = Acceleration (m/s²)

Power required:
P_accel = F_accel × v

Example (m = 1800 kg, a = 1 m/s², v = 50 km/h):
m_eff = 1800 × 1.08 = 1944 kg
F_accel = 1944 × 1 = 1944 N
P_accel = 1944 × (50/3.6) = 27.0 kW
```

#### 8.2.2 Total Power Requirement

**At Wheels**:
```
P_wheel = (F_aero + F_roll + F_grade + F_accel) × v

Example (120 km/h, flat, steady):
P_wheel = (313 + 141) × 33.3 = 15.1 kW
```

**At Battery** (including losses):
```
P_battery = P_wheel / (η_motor × η_inverter × η_gearbox)

Typical efficiencies:
- η_motor: 0.92-0.96
- η_inverter: 0.96-0.98
- η_gearbox: 0.96-0.98
- Combined: 0.85-0.92

Example:
P_battery = 15.1 / (0.94 × 0.97 × 0.97)
          = 15.1 / 0.885
          = 17.1 kW
```

**Energy Consumption**:
```
E_consumption (kWh/km) = P_battery (kW) / v (km/h)

Example:
E_consumption = 17.1 / 120 = 0.143 kWh/km
              = 14.3 kWh/100km
```

### 8.3 Drive Cycle Analysis

#### 8.3.1 Standard Drive Cycles

**1. EPA Combined** (55% City, 45% Highway):
```
City (FTP-75):
- Average speed: 34 km/h
- Max speed: 91 km/h
- Distance: 17.77 km
- Duration: 31.2 min
- Stops: 23

Highway (HWFET):
- Average speed: 77 km/h
- Max speed: 97 km/h
- Distance: 16.45 km
- Duration: 12.75 min
- Stops: 0

Combined:
E_combined = 0.55 × E_city + 0.45 × E_highway
```

**2. WLTP** (Worldwide Harmonized Light Vehicle Test Procedure):
```
Low: 0-15 min, max 56 km/h
Medium: 15-23 min, max 76 km/h
High: 23-28 min, max 97 km/h
Extra High: 28-30 min, max 131 km/h

More aggressive than EPA → higher consumption
```

**3. NEDC** (obsolete but reference):
```
Less realistic, lower consumption than EPA/WLTP
```

#### 8.3.2 Real-World Factors

**Temperature Effects**:
```
Cabin Heating/Cooling:
- Heating (0°C ambient): +3-7 kW → 30-50% range reduction
- Cooling (35°C ambient): +1-3 kW → 10-20% range reduction

Battery Performance:
- At -10°C: -20% capacity, -30% power → 25-35% range reduction
- At -20°C: -30% capacity, -50% power → 40-50% range reduction
```

**Driving Style**:
```
Aggressive (rapid accel/decel): +20-40% consumption
Moderate: Reference
Eco (smooth, anticipatory): -10-20% consumption
```

**Auxiliary Loads**:
```
Baseline (always on): 200-500 W (computers, lights)
HVAC: 1-7 kW (climate dependent)
Heated seats/steering: 100-300 W
Headlights: 50-150 W (LED)
Infotainment: 50-100 W
```

**Terrain**:
```
Flat: Reference
Rolling hills: +10-20% consumption (energy lost to braking, partial regen recovery)
Mountainous: +20-40% consumption (going up) or -10-20% (going down with regen)
```

**Speed**:
```
Urban (30-50 km/h): Low aero drag, frequent stop/start, moderate consumption
Highway (100-120 km/h): High aero drag, steady speed, moderate-high consumption
Autobahn (150-180 km/h): Very high aero drag (cubic!), highest consumption

Consumption vs. Speed (typical sedan):
50 km/h: 12 kWh/100km
80 km/h: 14 kWh/100km
100 km/h: 16 kWh/100km
120 km/h: 19 kWh/100km
140 km/h: 23 kWh/100km
160 km/h: 28 kWh/100km
```

### 8.4 Range Estimation Algorithm

```python
def calculate_range(battery_capacity, soc, avg_speed, ambient_temp,
                   drive_style, terrain, hvac_use):
    """
    Comprehensive range estimation
    """
    # Base energy available
    energy_available = battery_capacity * (soc / 100) * USABLE_DOD

    # Base consumption from vehicle characteristics
    base_consumption = calculate_base_consumption(avg_speed)

    # Temperature adjustment
    temp_factor = temperature_factor(ambient_temp)

    # HVAC load
    hvac_power = calculate_hvac_power(ambient_temp, hvac_use)
    hvac_consumption = hvac_power / avg_speed

    # Driving style adjustment
    style_factor = {
        'eco': 0.85,
        'normal': 1.0,
        'sport': 1.25
    }[drive_style]

    # Terrain adjustment
    terrain_factor = {
        'flat': 1.0,
        'rolling': 1.15,
        'mountainous': 1.30
    }[terrain]

    # Total consumption
    total_consumption = (base_consumption * temp_factor * style_factor *
                        terrain_factor + hvac_consumption)

    # Estimated range
    estimated_range = energy_available / total_consumption

    # Confidence interval
    uncertainty = 0.15  # ±15%
    range_min = estimated_range * (1 - uncertainty)
    range_max = estimated_range * (1 + uncertainty)

    return {
        'range': estimated_range,
        'range_min': range_min,
        'range_max': range_max,
        'consumption': total_consumption,
        'energy_available': energy_available
    }
```

---

## 9. Energy Efficiency

### 9.1 Powertrain Efficiency

**Component Efficiencies**:
```
Battery discharge efficiency: 95-98%
Inverter efficiency: 95-98%
Motor efficiency: 90-97%
Gearbox efficiency: 96-98%
Differential efficiency: 97-99%

Tank-to-Wheel Efficiency: 85-95%

vs. ICE Vehicle:
Engine efficiency: 20-30% (gasoline), 30-40% (diesel)
Transmission efficiency: 85-90%
Total: 17-27% (gasoline), 25-36% (diesel)

EV Advantage: 3-5× more efficient!
```

### 9.2 Energy Flow Sankey Diagram

```
100 kWh from Battery
├─ 2 kWh: Battery internal resistance (2%)
├─ 2 kWh: Inverter losses (2%)
├─ 6 kWh: Motor losses (6%)
├─ 2 kWh: Gearbox/differential (2%)
└─ 88 kWh: To wheels (88% efficiency)

At wheels:
├─ 50 kWh: Aerodynamic drag (56%)
├─ 20 kWh: Rolling resistance (23%)
├─ 10 kWh: Braking (11%, partially recovered)
└─ 8 kWh: Accessories (9%)

Note: Distribution varies with speed
- Low speed: Rolling resistance dominant
- High speed: Aerodynamic drag dominant
```

### 9.3 Efficiency Optimization Strategies

**1. Motor Operating Point Optimization**:
```
- Operate motor in high-efficiency region (sweet spot)
- Typical: 75-85% of rated torque, 2000-6000 RPM
- Efficiency map-based control
```

**2. Regenerative Braking Maximization**:
```
- Predictive deceleration
- One-pedal driving mode
- Route-based optimization (GPS + map data)
```

**3. Thermal Management**:
```
- Preheat/precool while charging
- Minimize HVAC power:
  * Seat heating instead of cabin (1/10 power)
  * Heat pump instead of resistive heating (1/3 power)
  * Smart ventilation (use outside air when possible)
```

**4. Aerodynamic Enhancements**:
```
- Active grille shutters (close when cooling not needed)
- Auto-lowering suspension at high speed
- Wheel covers/aero wheels
- Underbody panels
```

**5. Weight Reduction**:
```
- Aluminum/carbon fiber body
- Structural battery pack
- Lightweight wheels
```

**6. Tire Optimization**:
```
- Low rolling resistance tires (C_rr = 0.006-0.008)
- Proper inflation (under-inflation increases C_rr 10-20%)
- Narrower tires (lower drag, but less grip)
```

### 9.4 Efficiency Comparison

| Vehicle Type | Tank-to-Wheel Efficiency | Well-to-Wheel Efficiency* |
|--------------|-------------------------|--------------------------|
| EV (Grid) | 85-95% | 30-45% (depends on grid mix) |
| EV (Solar) | 85-95% | 75-85% |
| ICE Gasoline | 17-27% | 12-20% |
| ICE Diesel | 25-36% | 18-27% |
| Hybrid | 30-40% | 22-30% |
| PHEV | 40-60% | 30-45% |
| Hydrogen FCV | 40-60% | 15-25% (H2 from electrolysis) |

*Well-to-Wheel includes fuel production and delivery

---

## 10. Charging Systems

### 10.1 AC Charging (On-Board Charger)

#### 10.1.1 Level 1 (120V, 12-16A)
```
Voltage: 120V AC
Current: 12-16A
Power: 1.4-1.9 kW
Connector: J1772 (US), Type 2 (EU)
Charging Time (75 kWh): 40-50 hours (0-100%)
Use Case: Emergency, overnight (PHEV)
```

#### 10.1.2 Level 2 (240V, 16-80A)
```
Voltage: 208-240V AC (single-phase)
Current: 16-80A
Power: 3.3-19.2 kW
Connector: J1772 (US), Type 2 (EU)
Charging Time (75 kWh): 4-23 hours (0-100%)
Use Cases:
- Home charging (typical: 7.2 kW, 11-11 hours)
- Workplace charging
- Public L2 chargers
- Hotel/shopping center

Common Configurations:
- 3.3 kW: 14A @ 240V (basic)
- 7.2 kW: 30A @ 240V (most common home)
- 11 kW: 48A @ 230V (3-phase, Europe)
- 22 kW: 32A @ 400V (3-phase, EU, rare for vehicles)
```

**Charging Curve** (Level 2, 7.2 kW):
```
0-80% SoC: Constant current (7.2 kW)
80-95% SoC: Transition to constant voltage
95-100% SoC: Constant voltage (power tapers to <1 kW)

Time to 80%: ~7.5 hours (from empty, 75 kWh battery)
Time to 100%: ~11 hours
```

### 10.2 DC Fast Charging

#### 10.2.1 Charging Standards

**CCS (Combined Charging System)** - Most common globally
```
CCS1 (North America):
- Connector: J1772 + 2 DC pins
- Power: 50-350 kW
- Voltage: 200-920V DC
- Current: 200-500A
- Vehicles: Most non-Tesla EVs in US

CCS2 (Europe):
- Connector: Type 2 + 2 DC pins
- Power: 50-350 kW
- Same specs as CCS1
- Vehicles: All EVs in Europe (including Tesla Model 3/Y)
```

**CHAdeMO** - Japanese standard
```
Connector: Dedicated DC connector
Power: 50-400 kW (400 kW: CHAdeMO 3.0)
Voltage: 50-500V (up to 1000V for v3.0)
Current: 125-400A (up to 600A for v3.0)
Vehicles: Nissan Leaf, older Japanese EVs
Status: Declining market share outside Japan
```

**Tesla Supercharger**
```
North America (before 2024):
- Connector: Proprietary Tesla connector
- Power: 72-250 kW
- Voltage: 50-500V
- Current: 300-630A

V3 Supercharger:
- Power: Up to 250 kW (limited by vehicle)
- Vehicles: Model 3 Performance ~250 kW, Model S/X ~200 kW

V4 Supercharger (2023+):
- Power: Up to 350 kW
- Connector: Longer cable, CCS adapter support

NACS (North American Charging Standard):
- Tesla connector adopted as SAE J3400
- Ford, GM, Rivian, etc. adopting for 2025+ vehicles
```

**GB/T** - Chinese standard
```
Connector: GB/T DC connector
Power: 37.5-237.5 kW (up to 900 kW proposed)
Voltage: 200-750V
Current: 80-250A (up to 600A proposed)
Vehicles: All EVs sold in China
```

#### 10.2.2 Charging Power Levels

```
Level 3 / DC Fast:
- 50 kW: Early DC fast chargers, ~45 min (10-80%)
- 100 kW: Common public chargers, ~30 min (10-80%)
- 150 kW: Modern highway chargers, ~25 min (10-80%)
- 250 kW: Tesla V3, Ionity, ~18 min (10-80%)
- 350 kW: Ultra-fast chargers, ~15 min (10-80%, if vehicle supports)

Note: Actual charging time depends on:
1. Battery capacity
2. Battery chemistry and thermal management
3. Current SoC
4. Battery temperature
5. Charger power vs. vehicle acceptance rate
```

#### 10.2.3 Charging Curve

**Typical DC Fast Charging Profile**:
```
Phase 1: Preconditioning (if needed)
- If battery too cold (<15°C) or hot (>45°C)
- Active heating/cooling to optimal range (20-35°C)
- Power limited (e.g., 50 kW) during preconditioning
- Duration: 5-15 min

Phase 2: Constant Power / Constant Current (0-50% SoC)
- Maximum power delivery
- Limited by: min(Charger_Power, Battery_Power, Cable_Current)
- Example: 150 kW charger, battery accepts 175 kW → 150 kW
- Duration: 10-15 min

Phase 3: Power Ramp (50-80% SoC)
- Power gradually reduces to protect battery
- Voltage approaches maximum (e.g., 410V for 400V system)
- Transition to constant voltage
- Duration: 10-15 min

Phase 4: Constant Voltage (80-100% SoC)
- Voltage held constant at maximum
- Current (and power) decrease exponentially
- Battery balancing occurs
- Very slow above 90% SoC
- Duration: 15-30 min (80-100%)

Recommendation: Charge to 80% for fastest charging, stop at 90% for optimal time/energy
```

**Charging Power vs. SoC** (Example: 800V vehicle, 350 kW charger):
```
SoC    Power   Voltage  Current  Duration
0-10%  50 kW   320V     156A     ~6 min  (precondition)
10-40% 300 kW  400V     750A     ~8 min
40-60% 270 kW  450V     600A     ~6 min
60-75% 200 kW  500V     400A     ~6 min
75-80% 150 kW  520V     288A     ~3 min
80-90% 75 kW   540V     139A     ~8 min
90-100% 25 kW  550V     45A      ~20 min

10-80%: ~29 minutes
10-100%: ~57 minutes
```

### 10.3 Charging Infrastructure

#### 10.3.1 Communication Protocols

**ISO 15118** (Plug & Charge):
```
Features:
- Automatic authentication (no RFID card needed)
- Dynamic pricing
- Bidirectional communication (V2G ready)
- Encrypted communication
- Smart charging (load management)

Adoption: CCS and Tesla (in some regions)
```

**OCPP** (Open Charge Point Protocol):
```
Purpose: Charger ↔ Network communication
Features:
- Remote monitoring
- Firmware updates
- Load balancing
- Transaction management
- Widely adopted for network management
```

#### 10.3.2 Smart Charging

**Load Management**:
```
Dynamic load balancing across multiple charging points
Example: 100 kW total capacity, 4 vehicles:
- If 1 vehicle: 100 kW
- If 2 vehicles: 50 kW each
- If 4 vehicles: 25 kW each
```

**Time-of-Use Optimization**:
```
Charge when electricity is cheap (off-peak)
Typical: Night (9 PM - 7 AM) at 50-70% of peak price
Smart: Delay charging or modulate power based on grid signal
```

**V2G (Vehicle-to-Grid)**:
```
Bidirectional charging: Vehicle can discharge to grid
Applications:
- Grid stabilization (frequency regulation)
- Peak shaving (reduce demand during high-price periods)
- Backup power
- Renewable integration (solar/wind buffering)

Revenue potential: $500-1500/year per vehicle (varies by market)
Battery degradation: Minimal if managed properly (<2% extra degradation)
```

### 10.4 Wireless Charging (WPT)

**Technology**: Inductive coupling (magnetic resonance)

**Efficiency**: 85-93% (vs. 95-98% for wired)

**Power Levels**:
```
Level 1 (≤3.7 kW): Residential (SAE J2954)
Level 2 (3.7-11 kW): Commercial, public
Level 3 (>11 kW): Rapid wireless (in development)
```

**Alignment Tolerance**: ±10-15 cm (modern systems)

**Advantages**:
- Convenience (no plug)
- Automatic charging
- Weather-proof
- Suitable for autonomous vehicles

**Disadvantages**:
- Lower efficiency
- Higher cost
- Requires precise alignment
- Foreign object detection needed

**Status**: Limited commercial deployment (BMW, Genesis)

---

## 11. Data Formats

### 11.1 Vehicle Configuration

```json
{
  "vehicle": {
    "id": "EV-2024-001",
    "manufacturer": "Example Motors",
    "model": "E-Sedan",
    "year": 2024,
    "type": "BEV",
    "battery": {
      "capacity_kwh": 75.0,
      "usable_capacity_kwh": 72.0,
      "chemistry": "NMC811",
      "voltage_nominal": 355,
      "voltage_max": 403,
      "voltage_min": 240,
      "cell_type": "cylindrical_21700",
      "configuration": "96s4p",
      "cooling_type": "liquid"
    },
    "motor": {
      "type": "PMSM",
      "count": 1,
      "location": "rear",
      "power_rated_kw": 150,
      "power_peak_kw": 200,
      "torque_rated_nm": 310,
      "torque_peak_nm": 400,
      "max_rpm": 14000,
      "efficiency_percent": 96
    },
    "powertrain": {
      "inverter_type": "SiC",
      "inverter_efficiency_percent": 98,
      "gearbox_type": "single_speed",
      "gearbox_ratio": 9.0,
      "gearbox_efficiency_percent": 97,
      "drivetrain": "RWD"
    },
    "vehicle_specs": {
      "mass_kg": 1800,
      "drag_coefficient": 0.24,
      "frontal_area_m2": 2.3,
      "rolling_resistance": 0.008,
      "tire_radius_m": 0.34
    },
    "charging": {
      "ac_max_kw": 11,
      "dc_max_kw": 150,
      "ac_connector": "Type2",
      "dc_connector": "CCS2"
    }
  }
}
```

### 11.2 Drive Profile

```json
{
  "drive_profile": {
    "timestamp": "2024-12-26T10:00:00Z",
    "duration_seconds": 3600,
    "distance_km": 85.3,
    "average_speed_kmh": 85.3,
    "energy_consumed_kwh": 14.5,
    "efficiency_kwh_per_100km": 17.0,
    "conditions": {
      "ambient_temp_c": 22,
      "hvac_power_kw": 1.2,
      "terrain": "rolling",
      "weather": "clear"
    },
    "soc_start_percent": 85,
    "soc_end_percent": 66,
    "regenerative_energy_kwh": 2.1,
    "regenerative_efficiency_percent": 78,
    "segments": [
      {
        "type": "urban",
        "distance_km": 12.5,
        "average_speed_kmh": 42,
        "energy_kwh": 2.1
      },
      {
        "type": "highway",
        "distance_km": 68.8,
        "average_speed_kmh": 110,
        "energy_kwh": 11.2
      },
      {
        "type": "rural",
        "distance_km": 4.0,
        "average_speed_kmh": 68,
        "energy_kwh": 1.2
      }
    ]
  }
}
```

### 11.3 Charging Session

```json
{
  "charging_session": {
    "session_id": "CHG-20241226-123456",
    "start_time": "2024-12-26T14:30:00Z",
    "end_time": "2024-12-26T14:58:00Z",
    "duration_minutes": 28,
    "charger_id": "STATION-042-DC-02",
    "charger_type": "DC_FAST",
    "max_power_kw": 150,
    "standard": "CCS2",
    "energy_delivered_kwh": 52.3,
    "cost_total": 18.31,
    "cost_currency": "USD",
    "soc_start_percent": 18,
    "soc_end_percent": 82,
    "battery_temp_start_c": 28,
    "battery_temp_end_c": 35,
    "power_profile": [
      {"time": "14:30", "power_kw": 145, "soc": 18, "temp_c": 28},
      {"time": "14:35", "power_kw": 148, "soc": 28, "temp_c": 30},
      {"time": "14:40", "power_kw": 142, "soc": 38, "temp_c": 32},
      {"time": "14:45", "power_kw": 125, "soc": 52, "temp_c": 34},
      {"time": "14:50", "power_kw": 95, "soc": 68, "temp_c": 35},
      {"time": "14:55", "power_kw": 62, "soc": 78, "temp_c": 35},
      {"time": "14:58", "power_kw": 42, "soc": 82, "temp_c": 35}
    ],
    "average_power_kw": 112,
    "peak_power_kw": 148,
    "efficiency_percent": 94
  }
}
```

---

## 12. API Interface

### 12.1 Core Functions

#### 12.1.1 Range Calculation

```typescript
interface RangeCalculationRequest {
  batteryCapacity: number;      // kWh
  currentSoC: number;            // %
  averageSpeed: number;          // km/h
  ambientTemp: number;           // °C
  driveProfile: 'urban' | 'highway' | 'mixed';
  hvacEnabled: boolean;
  terrainType: 'flat' | 'rolling' | 'mountainous';
}

interface RangeCalculationResponse {
  estimatedRange: number;        // km
  rangeMin: number;              // km (worst case)
  rangeMax: number;              // km (best case)
  energyConsumption: number;     // kWh/100km
  confidence: number;            // 0-1
  factors: {
    baseConsumption: number;
    temperatureImpact: number;
    hvacImpact: number;
    terrainImpact: number;
  };
}

function calculateRange(request: RangeCalculationRequest): RangeCalculationResponse;
```

#### 12.1.2 Charging Time Estimation

```typescript
interface ChargingTimeRequest {
  batteryCapacity: number;       // kWh
  currentSoC: number;             // %
  targetSoC: number;              // %
  chargerPower: number;           // kW
  chargerType: 'AC_L1' | 'AC_L2' | 'DC_FAST';
  batteryTemp: number;            // °C
}

interface ChargingTimeResponse {
  estimatedTime: number;          // minutes
  energyToDeliver: number;        // kWh
  averagePower: number;           // kW
  chargingCurve: Array<{
    time: number;                 // minutes
    soc: number;                  // %
    power: number;                // kW
  }>;
  preConditionTime?: number;      // minutes (if battery too cold/hot)
}

function calculateChargingTime(request: ChargingTimeRequest): ChargingTimeResponse;
```

#### 12.1.3 Energy Consumption

```typescript
interface EnergyConsumptionRequest {
  distance: number;               // km
  averageSpeed: number;           // km/h
  vehicleMass: number;            // kg
  dragCoefficient: number;        // dimensionless
  frontalArea: number;            // m²
  rollingResistance: number;      // dimensionless
  grade: number;                  // % (positive = uphill)
  acceleration: number;           // m/s²
  ambientTemp: number;            // °C
  hvacPower: number;              // kW
}

interface EnergyConsumptionResponse {
  totalEnergy: number;            // kWh
  energyPerKm: number;            // kWh/km
  breakdown: {
    aerodynamic: number;          // kWh
    rolling: number;              // kWh
    grade: number;                // kWh
    acceleration: number;         // kWh
    hvac: number;                 // kWh
    accessories: number;          // kWh
    drivetrain_loss: number;      // kWh
  };
  efficiency: number;             // % (useful energy / total energy)
}

function calculateEnergyConsumption(request: EnergyConsumptionRequest): EnergyConsumptionResponse;
```

#### 12.1.4 Regenerative Braking

```typescript
interface RegenerativeBrakingRequest {
  vehicleMass: number;            // kg
  initialSpeed: number;           // km/h
  finalSpeed: number;             // km/h
  decelerationRate: number;       // m/s²
  motorEfficiency: number;        // 0-1
  inverterEfficiency: number;     // 0-1
  batteryEfficiency: number;      // 0-1
  currentSoC: number;             // %
  batteryTemp: number;            // °C
}

interface RegenerativeBrakingResponse {
  kineticEnergy: number;          // kWh
  recoverableEnergy: number;      // kWh
  actualRecovered: number;        // kWh
  recoveryEfficiency: number;     // %
  limitations: string[];          // e.g., ["battery_soc_high", "battery_temp_low"]
  rangeExtension: number;         // km
}

function calculateRegenerativeEnergy(request: RegenerativeBrakingRequest): RegenerativeBrakingResponse;
```

### 12.2 Battery Management

```typescript
interface BatteryStatus {
  soc: number;                    // %
  soh: number;                    // %
  voltage: number;                // V
  current: number;                // A
  temperature: number;            // °C
  power: number;                  // kW (positive = discharge, negative = charge)
  maxChargePower: number;         // kW
  maxDischargePower: number;      // kW
  cellVoltages: number[];         // V (per cell/module)
  cellTemperatures: number[];     // °C (per cell/module)
  cycleCount: number;
  estimatedRemainingLife: number; // years
}

interface BatteryHealthPrediction {
  currentSoH: number;             // %
  predictedSoH: Array<{
    years: number;
    soh: number;                  // %
    confidence: number;           // 0-1
  }>;
  degradationRate: number;        // % per year
  factorsInfluencingHealth: {
    temperature_stress: number;   // 0-1 (higher = more stress)
    cycle_depth_stress: number;
    fast_charge_frequency: number;
    calendar_aging: number;
  };
  recommendations: string[];
}

function getBatteryStatus(): BatteryStatus;
function predictBatteryHealth(currentStatus: BatteryStatus, usagePattern: UsagePattern): BatteryHealthPrediction;
```

### 12.3 Motor Control

```typescript
interface MotorCommand {
  torqueRequest: number;          // N·m
  speedLimit: number;             // RPM
  efficiencyMode: 'eco' | 'normal' | 'sport';
}

interface MotorStatus {
  actualTorque: number;           // N·m
  speed: number;                  // RPM
  power: number;                  // kW
  efficiency: number;             // %
  temperature: {
    stator: number;               // °C
    rotor: number;                // °C
    windings: number;             // °C
  };
  current: {
    d_axis: number;               // A
    q_axis: number;               // A
  };
  voltage: {
    d_axis: number;               // V
    q_axis: number;               // V
  };
}

function setMotorCommand(command: MotorCommand): void;
function getMotorStatus(): MotorStatus;
```

---

## 13. Safety Protocols

### 13.1 High Voltage Safety

#### 13.1.1 Isolation Monitoring

**Requirements**:
```
Minimum isolation resistance: 100 Ω/V of system voltage
Example: 400V system → 40 kΩ minimum

Continuous monitoring:
- Measure resistance between HV+ and chassis
- Measure resistance between HV- and chassis
- Alert if <100 Ω/V
- Disconnect HV system if <50 Ω/V (critical fault)
```

**Detection Method**:
```
Inject low-frequency AC signal (1-20 Hz, <1 mA)
Measure current leakage to chassis
Calculate isolation resistance from Ohm's law
Update every 100-500 ms
```

#### 13.1.2 Automatic Disconnection

**Conditions for HV Disconnect**:
1. Isolation fault (R < threshold)
2. Over-voltage (cell voltage > 4.25V for Li-ion)
3. Under-voltage (cell voltage < 2.5V)
4. Over-current (>C-rate limit)
5. Over-temperature (>60°C for most chemistries)
6. Crash detection (accelerometer >5g)
7. Service mode activation
8. Emergency stop button

**Disconnection Mechanism**:
```
Primary: HV contactors (relays) in series with HV+
Backup: Pyrofuses (explosive fuses) for crash scenarios
Response time: <10 ms for crash, <100 ms for electrical faults
```

#### 13.1.3 Service Disconnect

**Manual Service Disconnect**:
- Physical disconnect mechanism
- Removes power from HV system
- Required for service work
- Interlocked (cannot remove while HV active)
- Storage in safe location when removed

### 13.2 Battery Protection

#### 13.2.1 Cell-Level Protection

**Voltage Limits**:
```
Over-voltage:
- Warning: >4.15V (for NMC, 3.6V nominal)
- Error: >4.20V
- Critical: >4.25V (disconnect)

Under-voltage:
- Warning: <3.0V
- Error: <2.7V
- Critical: <2.5V (disconnect, risk of copper dissolution)
```

**Current Limits**:
```
Discharge:
- Continuous: 2-4C (e.g., 150-300 kW for 75 kWh)
- Peak (10s): 4-8C (e.g., 300-600 kW)
- Absolute max: Hardware limit at 10C

Charge:
- Continuous: 0.5-1C (e.g., 37.5-75 kW)
- Peak (DC fast): 1.5-3C (e.g., 112-225 kW, with thermal management)
- Absolute max: Hardware limit at 4C
```

**Temperature Limits**:
```
Discharge Operation:
- Optimal: 20-35°C
- Acceptable: -10 to 50°C
- Limited performance: -20 to -10°C, 50 to 55°C
- Prohibited: <-20°C, >55°C (disconnect)

Charge Operation:
- Optimal: 20-35°C
- Acceptable: 0 to 45°C
- Limited (slow): -10 to 0°C
- Prohibited: <-10°C, >45°C (prevent Li plating and degradation)

Storage:
- Optimal: 20-25°C at 50% SoC
- Acceptable: 0-30°C
```

#### 13.2.2 Thermal Runaway Prevention

**Detection**:
```
Temperature Rise Rate:
- Normal: <1°C/min
- Warning: >2°C/min
- Critical: >5°C/min (possible thermal runaway)

Voltage Drop:
- Sudden drop >0.5V in <1s indicates internal short

Gas Sensors:
- Detect venting gases (electrolyte vapor)
```

**Mitigation**:
```
1. Immediate HV disconnect
2. Activate aggressive cooling (max flow rate)
3. Isolate affected module (if architecture allows)
4. Alert driver and emergency services
5. Prevent propagation to adjacent cells (thermal barriers)
```

**Thermal Barriers**:
```
- Phase change materials between cells/modules
- Mica or ceramic insulation
- Air gaps
- Goal: Delay propagation >5 minutes (allow evacuation)
```

### 13.3 Crash Safety

#### 13.3.1 Crash Detection

**Sensors**:
```
Accelerometers: Detect >5g impact (any direction)
Pressure sensors: Detect cabin intrusion
Roll sensors: Detect rollover

Fusion: Combine multiple signals for robustness
Response time: <2 ms (faster than airbag deployment)
```

**Actions**:
```
1. Fire pyrofuses (disconnect HV battery) <10 ms
2. Open HV contactors <10 ms
3. Disable charging system
4. Activate emergency flashers
5. Unlock doors
6. Cut fuel pump (if PHEV)
7. Send crash notification (if connected)
```

#### 13.3.2 Battery Pack Structural Protection

**Design Requirements**:
```
Side impact: Withstand 50 km/h pole impact
Front/rear: Withstand 60 km/h offset frontal impact
Bottom: Withstand road debris, curb strikes

Methods:
- Reinforced battery enclosure (aluminum or steel)
- Crush zones around pack
- Elevated mounting (ground clearance)
- Underbody shield
```

**Post-Crash Safety**:
```
- HV components de-energized automatically
- Emergency responders can confirm via indicator (e.g., flashing LED)
- Manual disconnect easily accessible and labeled
- First responder guide with HV component locations
```

### 13.4 Electromagnetic Compatibility (EMC)

**Requirements**:
```
Emissions (conducted and radiated):
- Must meet CISPR 25 (automotive EMC standard)
- Frequency range: 150 kHz to 2.5 GHz
- Limit interference with radio, navigation, safety systems

Immunity:
- Withstand external electromagnetic fields
- From: Radio transmitters, power lines, other vehicles
- No malfunction or safety issue

Shielding:
- HV cables: Shielded and grounded
- Inverter: Metal enclosure, EMI filters
- Battery: Conductive enclosure (Faraday cage)
```

### 13.5 Functional Safety (ISO 26262)

**ASIL Levels** (Automotive Safety Integrity Level):
```
ASIL D (highest): HV safety, crash detection, braking
ASIL C: Motor control, battery management
ASIL B: Charging control, thermal management
ASIL A: Infotainment, comfort features
QM (Quality Management): Non-safety features

Requirements:
- Redundant sensors and controllers for ASIL D
- Fail-safe defaults (e.g., open HV contactors on fault)
- Continuous self-diagnostics
- Error logging and reporting
```

### 13.6 Charging Safety

**Grid Fault Detection**:
```
- Ground fault detection (GFCI)
- Over/under voltage detection
- Over-current protection
- Arc fault detection

Response: Immediately stop charging, open contactors
```

**Pilot Signal** (SAE J1772):
```
Function: Communication between vehicle and EVSE
- Charger readiness
- Maximum available current (PWM duty cycle)
- Charging permission (vehicle requests)

Voltage Levels:
- +12V: Standby (charger ready, vehicle not connected)
- +9V: Vehicle detected
- +6V: Vehicle ready, charging authorized
- +3V: Ventilation required (rare, DC fast)
- -12V: Fault condition
```

**Interlock Circuit**:
```
Purpose: Prevent removal of plug while energized
Mechanism: Solenoid locks connector in place during charging
Release: Only when HV de-energized
```

---

## 14. References

### 14.1 Standards and Regulations

**International**:
- ISO 6469: Electric Road Vehicles - Safety Specifications
- ISO 17409: Electric Vehicles - Conductive Power Transfer - Safety Requirements
- ISO 15118: Vehicle-to-Grid Communication Interface
- IEC 61851: Electric Vehicle Conductive Charging System
- IEC 62196: Plugs, Socket-Outlets, Vehicle Connectors and Vehicle Inlets - Conductive Charging

**North America**:
- SAE J1772: AC Level 1 and Level 2 Charging
- SAE J3068: AC Level 2 Charging for Fleets
- SAE J3400: NACS (North American Charging Standard)
- SAE J2954: Wireless Power Transfer
- UL 2202: Electric Vehicle Charging Equipment
- FMVSS 305: Electric Powered Vehicles - Electrolyte Spillage and Electrical Shock Protection

**Europe**:
- ECE R100: Electric Powertrains - Safety Requirements
- EN 62196: Charging Connectors
- VDA: German Association Standards

**China**:
- GB/T 18384: Electric Vehicles - Safety Requirements
- GB/T 20234: Connection Set for Conductive Charging
- GB/T 27930: Communication Protocol

### 14.2 Battery Research

1. Goodenough, J.B., Park, K.S. (2013). "The Li-Ion Rechargeable Battery: A Perspective". *Journal of the American Chemical Society*.

2. Blomgren, G.E. (2017). "The Development and Future of Lithium Ion Batteries". *Journal of The Electrochemical Society*.

3. USABC (United States Advanced Battery Consortium). "Battery Technology Roadmap".

4. 선행 연구. "Li-ion Battery Materials: Present and Future". *Materials Today*.

### 14.3 Motor and Power Electronics

1. Chan, C.C., Chau, K.T. (2001). "Modern Electric Vehicle Technology". Oxford University Press.

2. Emadi, A. (2015). "Advanced Electric Drive Vehicles". CRC Press.

3. Krishnan, R. (2010). "Permanent Magnet Synchronous and Brushless DC Motor Drives". CRC Press.

4. Mohan, N., Undeland, T.M. (2007). "Power Electronics: Converters, Applications, and Design". Wiley.

### 14.4 Vehicle Dynamics and Efficiency

1. Gillespie, T.D. (1992). "Fundamentals of Vehicle Dynamics". SAE International.

2. Guzzella, L., Sciarretta, A. (2013). "Vehicle Propulsion Systems". Springer.

3. EPA: "Electric Vehicle and Fuel Cell Vehicle Dynamometer Testing".

### 14.5 Thermal Management

1. Pesaran, A.A. (2001). "Battery Thermal Management in EVs and HEVs". *Advanced Automotive Battery Conference*.

2. 선행 연구. "A Review of Thermal Performance Improving Methods of Lithium Ion Battery". *Journal of Power Sources*.

### 14.6 WIA Standards

- WIA-INTENT: Intent-based vehicle control interfaces
- WIA-OMNI-API: Universal API for automotive systems
- WIA-SOCIAL: V2V and V2X communication protocols
- WIA-ENERGY: Smart grid and V2G integration
- WIA-CLIMATE: Carbon accounting and sustainability metrics

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUTO-004 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
