# WIA-AUTO-004 PHASE 1 — Data Format Specification

**Standard:** WIA-AUTO-004
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

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


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-1-DATA-FORMAT. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-data-format/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-1-DATA-FORMAT with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-DATA-FORMAT does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-1-DATA-FORMAT.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-1-DATA-FORMAT. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P1-DATA-FORMAT-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-1-DATA-FORMAT validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
