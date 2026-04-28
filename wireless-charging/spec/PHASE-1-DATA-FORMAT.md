# WIA-COMM-008 — Phase 1: Data Format

> Wireless-charging canonical Phase 1: charger + coil + resonance + FOD + EMF-safety envelopes.

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




---

## A.1 Charger-record envelope

The Phase 1 envelope groups wireless-charger records by power class (low-power inductive — Qi 5W / 15W per WPC; mid-power resonant — Qi 30W+ / AirFuel / proprietary; high-power EV — SAE J2954 WPT 1/2/3 at 3.7/7.7/11.1 kW; experimental — 22 kW WPT 4) with the canonical fields: charger identifier, transmitter coil topology (single-coil / multi-coil array / dynamic alignment), operating frequency band (Qi LF 87-205 kHz; AirFuel HF 6.78 MHz; J2954 79-90 kHz; experimental MHz/GHz beaming separately catalogued in WIA-COMM-009), maximum input/output power, supported communication channel (Qi in-band ASK; AirFuel BLE; J2954 LIN/CAN sideband), and the manufacturer-of-record certificate chain.

## A.2 Coil-design descriptor

A coil-design descriptor MUST list: coil-pair geometry (planar circular / square / rectangular / DD-style for J2954; multi-coil arrays for spatial-freedom Qi), conductor-of-record (Litz wire AWG / strand count for skin-effect minimisation), turn count (Tx and Rx), inductance in microhenries, AC resistance at the operating frequency, the magnetic-shield catalogue (ferrite type / geometry), the coupling coefficient envelope (typical k = 0.1-0.3 for loosely-coupled inductive; k = 0.05-0.15 for resonant), and the FOD (foreign object detection) coil envelope per Phase 1 §A.4.

## A.3 Magnetic-resonance descriptor

Resonance descriptors carry: resonator topology (LC tank with Q factor 100-300 typical for AirFuel; LCC compensation for J2954; CLC for proprietary), the per-side compensation network, the impedance-matching envelope, the operating frequency stability envelope (typically <1% drift over the operating temperature range), the multi-device parallel-charging envelope (number of receivers, coupling cross-talk profile, per-device power-allocation policy), and the spatial-freedom envelope (allowable Tx-Rx misalignment in mm before efficiency drops below the policy threshold).

## A.4 Foreign-object-detection envelope

FOD envelopes carry: FOD method (power-loss accounting per Qi v1.3 with the calibrated reference power, parametric Q-measurement on the Tx coil, NFC-tag-based metal-layer detection, RFID-detection, MEMS-thermal sensing, machine-learning-based pattern recognition), the false-positive / false-negative envelope, the response-time envelope (typical <500 ms from object placement to power cut-off), and the fail-safe envelope (uncertain detection → reduced power; clear positive → power off; clear negative → continue). FOD descriptors cross-reference the EMF-safety envelope at Phase 1 §A.5.

## A.5 EMF-safety envelope

EMF safety envelopes follow ICNIRP 2010/2020 guidelines (general public + occupational exposure limits at the operating frequency) plus IEEE C95.1-2019 where adopted. The envelope carries the per-coil-region exposure profile (1 mT B-field reference at the body surface for Qi LF; 27 V/m E-field reference at AirFuel HF), the perpendicular-distance envelope from accessible surfaces, and the user-warning envelope (medical-device-implant warnings per ISO 14117 + IEC 60601-1-2 EMC for active implantable medical devices like pacemakers, ICDs, neurostimulators, cochlear implants).


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/wireless-charging/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-wireless-charging-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/wireless-charging-host:1.0.0` ships every wireless-charging envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/wireless-charging.sh` ships sample envelope generators with no
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
ecosystem. Wireless-charging deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
