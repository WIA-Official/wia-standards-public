# WIA-COMM-009 — Phase 1: Data Format

> Wireless-power-transfer canonical Phase 1: WPT-system + resonator + microwave/laser + safety + space-solar envelopes.

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




---

## A.1 WPT-system descriptor

The Phase 1 envelope groups WPT systems by transfer mechanism (IPT — inductive power transfer at LF/HF coils with k = 0.05-0.5; CPT — capacitive power transfer through electric-field coupling between conductive plates; resonant — high-Q LC tanks at fixed frequency with k = 0.05-0.15 typical; microwave — directed microwave beam at 2.45 GHz / 5.8 GHz / 24 GHz / 35 GHz / 94 GHz with rectenna receivers; laser — directed near-IR / SWIR beam at 808 / 976 / 1550 nm onto photovoltaic receivers; space-solar — orbital transmitter to Earth-side rectenna farm). Records carry the canonical fields: system identifier, mechanism, operating frequency, peak transmit power, end-to-end efficiency profile, link-distance envelope, and the regulatory authorisation envelope.

## A.2 Resonator and link descriptor

Resonator descriptors carry: resonator topology (Tx side: planar coil with LCC compensation / parallel LC; Rx side: matched LCC / series-parallel hybrid), Q factor (typical 100-500 for high-Q resonant), operating-frequency stability across temperature, the impedance-matching envelope, the per-receiver decoupling envelope, and the SOTIF envelope (where the WPT operates in human-occupied space). Link descriptors carry the per-link distance, alignment envelope (lateral / angular tolerance before efficiency falls below policy), and the link-budget envelope (Tx power → free-space loss / coupling loss / Rx-side conversion → delivered DC power).

## A.3 Microwave / laser-beaming envelope

Microwave-beaming envelopes carry: transmitter aperture and gain, beam-steering envelope (mechanical / electronic phased-array / liquid-crystal); receiver rectenna count and per-rectenna conversion efficiency; beam-safety envelope (interlock chain that gates the transmitter unless the receiver acknowledges presence and the safety scanner clears the beam path); the maximum-permissible-exposure envelope per IEEE C95.1; and the regulatory authorisation envelope per the operator's licensing regime. Laser-beaming envelopes carry the laser class per IEC 60825-1 (Class 1M / Class 4 with documented hazard zone), the beam-divergence envelope, the per-receiver photovoltaic conversion efficiency, and the active-safety-scanner envelope (LiDAR / camera-based intrusion detection in the beam path).

## A.4 Safety and exposure envelope

Safety envelopes follow ICNIRP guidelines for the relevant frequency band: at the IPT/CPT frequencies (kHz range) the body-coupled current limit applies; at the resonant frequencies (MHz) the SAR (specific absorption rate) limits apply; at microwave frequencies the field-strength and SAR limits apply per IEEE C95.1-2019; at laser frequencies the MPE (maximum permissible exposure) per IEC 60825-1 applies. Active medical-implant compatibility per ISO 14117 is required for systems operating in medical environments. Beam-blocking interlocks are mandatory for microwave and laser systems with peak powers above the operator's documented threshold.

## A.5 Space-solar-power envelope

Space-solar-power records carry: transmitter constellation envelope (single satellite / cluster / phased-array with sub-aperture envelope), orbit class (GEO / sun-synchronous LEO with rectenna handover / LEO with continuous-coverage cluster), beam-frequency envelope (typically 2.45 GHz or 5.8 GHz for atmospheric-window propagation), receiver rectenna farm envelope (km-scale), end-to-end efficiency profile (DC-RF in space + free-space + atmospheric absorption + RF-DC at the rectenna), and the regulatory authorisation envelope (ITU coordination per Radio Regulations + national regulator authorisation + International Maritime Organisation / ICAO clearances for over-flight). The standard's space-solar envelope is research-track only.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/wireless-power-transfer/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-wireless-power-transfer-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/wireless-power-transfer-host:1.0.0` ships every wireless-power-transfer envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/wireless-power-transfer.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Wireless-power-transfer deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
