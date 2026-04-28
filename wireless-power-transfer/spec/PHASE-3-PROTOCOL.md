# WIA-COMM-009 — Phase 3: Protocol

> Wireless-power-transfer canonical Phase 3: protocols (IPT/CPT + resonant + microwave/laser + EV-WPT + space-solar).

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




---

## A.1 IPT-and-CPT operating protocol

IPT operating protocol covers the operator's selected band (LF kHz for high-power EV and industrial; HF MHz for resonant-IPT for spatial-freedom consumer), the magnetic-coupling control loop (Tx-side current envelope, Rx-side rectifier-and-DC-DC envelope), the alignment-tracking envelope, the multi-receiver-coordination envelope where applicable, and the per-cycle FOD envelope per Phase 1 §A.4 (cross-referenced from WIA-COMM-008 wireless-charging). CPT operating protocol covers the electric-field-coupling between conductive plate pairs at the operator's selected frequency, the safety-distance envelope from accessible plates, and the impedance-matching control loop.

## A.2 Resonant operating protocol

Resonant-WPT protocol covers the high-Q tank operating at the operator's selected frequency (typically 6.78 MHz for AirFuel-class ISM-band systems), the BLE / Wi-Fi / proprietary side-channel for control coordination, the per-receiver-coupling decoupling envelope, the impedance-matching control loop, and the regulatory envelope per ETSI EN 301 489 / FCC Part 15 / Part 18. The protocol's strict frequency stability is required for regulatory compliance and to avoid cross-link interference at high-density deployment sites.

## A.3 Microwave-and-laser beaming protocol

Microwave-beaming protocol covers the beam-pointing acquisition (IFF-style transponder challenge from the receiver to verify presence, then beam-pointing servoing onto the receiver's beacon), the rectenna-conversion envelope (typical 60-85% per-rectenna efficiency at the design frequency), the safety-scanner interlock chain (independent radar/LiDAR/camera intrusion detection that immediately gates the beam off), and the regulatory-authorisation envelope (ITU-R + national regulator + per-deployment site clearance). Laser-beaming protocol covers the analogous beacon-acquisition, the rectenna-equivalent photovoltaic-receiver envelope, and the laser-class-1M-or-strictly-controlled-Class-4 safety-envelope per IEC 60825-1.

## A.4 EV-WPT and dynamic-charging protocol

EV-WPT protocol covers the static-charging case (vehicle parked over a ground-pad; alignment-assist by parking-radar and visual cue; LIN/CAN sideband per SAE J2847/6) and the dynamic-charging research case (instrumented road-segments with multiple ground pads; vehicle-side controller activates the segment as it approaches and deactivates as it passes; per-pad billing and inter-vendor settlement). The dynamic case is research-track only and is not certified for commercial deployment under this standard.

## A.5 Space-solar-power research protocol

Space-solar-power research protocol covers the orbit-and-beam coordination envelope (handover between satellites in a constellation; rectenna-farm-active-element selection; on-orbit phased-array calibration), the atmospheric-loss model (frequency-dependent absorption per ITU-R P.676; rain-fade allowance per ITU-R P.838), the regulatory-coordination envelope (ITU + national + international-maritime / aeronautical clearances), and the ground-segment safety envelope (rectenna farm exclusion zone with the documented field-strength contour). The protocol is research-track only and the standard does NOT certify any commercial space-solar-power service.

## A.6 Replay and integrity defence

Standard 96-bit nonce + 300-second skew window + 600-second seen-nonce cache for the management-plane control traffic. Per-link safety-event records are signed at emission time and the signature chain is anchored in the per-tenant Merkle tree; chain breaks invalidate the safety self-test and require re-execution before further `POST /links` calls. Telemetry traffic uses mTLS with per-channel monotonic counters; replay attempts are detected and dropped at the broker.


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
