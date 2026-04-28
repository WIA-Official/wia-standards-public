# WIA-COMM-008 — Phase 4: Integration

> Wireless-charging canonical Phase 4: ecosystem integration (WPC + AirFuel + SAE + IEC + ICNIRP + ISO 14117).

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



---

## A.1 Standards cross-walk

| Concern                       | Standard                                  |
|-------------------------------|-------------------------------------------|
| Low-power inductive (Qi)      | WPC Qi v1.3 / v2.0                        |
| Resonant                      | AirFuel Alliance specifications           |
| EV wireless charging          | SAE J2954                                 |
| EV in-band sideband           | SAE J2847/6                               |
| EU-harmonised EV WPT          | IEC 61980-1 / -2 / -3                     |
| EMC — radiated                | CISPR 11                                  |
| EMC — vehicle                 | CISPR 25                                  |
| EMC immunity                  | IEC 61000-4 series                        |
| EMC — generic conformance     | ETSI EN 301 489 + EN 300 330              |
| RF compliance (US)            | FCC 47 CFR Part 15 / Part 18              |
| RF compliance (EU)            | RED Directive 2014/53/EU                  |
| EMF — public exposure         | ICNIRP 2010 + 2020                        |
| EMF — RF exposure             | IEEE C95.1-2019                           |
| Implantable medical devices   | ISO 14117 + IEC 60601-1-2                 |
| Battery-charger safety        | IEC 62368-1                               |

This cross-walk is informative; implementers obtain authoritative copies from the issuing body.

## A.2 Multi-device and infrastructure integration

Multi-device integration captures the array-charger envelope (desk / countertop / vehicle-rear-shelf chargers with multiple Tx coils and an active coil-selection envelope), the public-charging-infrastructure envelope (airports, cafes, transit; per-station authentication via NFC / QR / BLE / app), the meter-of-record envelope for billed charging, and the operator-of-record envelope for distributed-fleet-managed charging. EV WPT integration captures the road-side power-distribution envelope, the per-pad utilisation reporting, and the predictive-maintenance envelope for high-cycle pad installations.

## A.3 EV-side and OEM integration

EV-side integration captures the receiver-of-record envelope per OEM (front, mid, rear pad placement; vehicle-frame Faraday-shield envelope; on-board secondary-resonant tuning), the SAE J2954 conformance tier, the per-vehicle alignment-estimation envelope (visual feedback + ultrasonic + vehicle-side coil sensing), the in-vehicle HMI integration for charging status, and the OTA-update envelope for receiver firmware. OEMs integrating wireless charging into ADAS-equipped vehicles align the alignment-estimation pipeline with the parking-assist subsystem.

## A.4 Safety and accessibility integration

Safety integration honours the implantable-medical-device envelope (no metallic warning labels in mandatory contact zones; dedicated screen/voice user-warning per ICD/pacemaker manufacturer guidance; minimum-clearance envelope for active implantable devices), the pet-and-child-safety envelope, the vehicle-occupant safety envelope for J2954 EV WPT (no underbody activity during active charging; vehicle-presence detection prior to power-on), and the post-installation EMC re-verification cadence. Accessibility integration honours WCAG 2.2 plus the operator's alignment-aid envelope for low-vision users (audible alignment cue; haptic alignment cue on the device).

## A.5 Future directions

Active research tracks: in-room ambient charging (low-power energy delivery to multiple devices via beamformed-RF or focused-acoustic); spatial-freedom MagSafe-like charging at 15-25W; dynamic in-motion EV WPT on instrumented road segments; bidirectional WPT for V2G feedback through wireless link; cross-vendor interoperability between Qi v2.0 and AirFuel via dual-mode chargers; LED-and-EMF-shielded operator-side coil arrays for high-power applications. The standard's roadmap envelope (`POST /standards/v1/proposals`) tracks active proposals through the WIA Committee voting process per Phase 4 §Z.4.

## A.6 Reference list

- WPC Qi Wireless Power Specification v1.3 / v2.0
- AirFuel Alliance Resonant Wireless Power Specification
- SAE J2954 — Wireless Power Transfer for Light-Duty Plug-In/Electric Vehicles
- SAE J2847/6 — Communication for Wireless Power Transfer
- IEC 61980-1 / -2 / -3 — Electric Vehicle Wireless Power Transfer
- CISPR 11 — Industrial, scientific, medical (ISM) radio-frequency equipment
- CISPR 25 — Vehicles, boats and internal-combustion engines — radio disturbance
- IEC 61000 series — EMC immunity
- ETSI EN 301 489 / EN 300 330 — EMC and short-range devices
- FCC 47 CFR Part 15 / Part 18 — radio frequency device rules
- EU RED Directive 2014/53/EU — radio equipment
- ICNIRP guidelines on limiting exposure to time-varying electric and magnetic fields (2010 + 2020 updates)
- IEEE C95.1-2019 — IEEE safety levels with respect to human exposure to electric, magnetic, and electromagnetic fields
- ISO 14117 — Active implantable medical devices — Electromagnetic compatibility
- IEC 60601-1-2 — Medical electrical equipment EMC
- IEC 62368-1 — Audio/video, information and communication technology equipment safety


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
