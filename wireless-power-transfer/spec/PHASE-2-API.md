# WIA-COMM-009 — Phase 2: API Interface

> Wireless-power-transfer canonical Phase 2: API surface (systems + links + safety-test + telemetry + regulatory).

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




---

## A.1 Endpoint reference

```http
POST /wireless-power-transfer/v1/systems         # register WPT system
GET  /wireless-power-transfer/v1/systems/{id}    # fetch system record
POST /wireless-power-transfer/v1/links           # establish link
GET  /wireless-power-transfer/v1/links/{id}/state# current link state
POST /wireless-power-transfer/v1/safety/test     # run safety self-test
WS   /wireless-power-transfer/v1/telemetry/stream# live link telemetry
GET  /wireless-power-transfer/v1/regulatory/{id} # regulatory authorisation status
```

Every endpoint follows the discovery convention at `/.well-known/wia-wireless-power-transfer`. Microwave / laser / space-solar systems require additional authorisation per the regulatory envelope.

## A.2 System-and-link API

`POST /systems` accepts the Phase 1 §A.1 envelope and returns a stable `systemId`. `POST /links` opens a link between a registered Tx and a registered Rx with the per-link parameters (alignment, requested-power, link-duration). The endpoint validates the link request against the regulatory envelope: a microwave-beaming link requires a current operator authorisation; a laser-beaming link requires the safety-scanner self-test to have passed within the documented validity window.

## A.3 Safety self-test API

`POST /safety/test` triggers a system safety self-test: continuity of the interlock chain, validity of the safety-scanner calibration, transmitter power-meter calibration check, and the regulatory-token freshness check. The endpoint returns the per-test result; a failed result blocks subsequent `POST /links` requests until the issue is remediated and the self-test is re-run successfully.

## A.4 Telemetry WebSocket

The telemetry WebSocket multiplexes per-link events: instantaneous Tx and Rx power, end-to-end efficiency, alignment metric, beam-block intrusion events, thermal status of the transmitter and receiver, and the regulatory-event stream (e.g., scheduled-shutdown approaching, unscheduled-shutdown triggered). Subscribers can filter by link-class and by event-class. The broker emits alarm events on safety-trigger conditions per Phase 3 §A.6.

## A.5 Regulatory and audit API

`GET /regulatory/{id}` returns the system's regulatory authorisation envelope: per-jurisdiction licence reference, validity period, conditional-restrictions, and the post-issuance compliance-monitoring envelope. Audit-trail endpoints expose the immutable history of system registrations, safety-test outcomes, link establishments, regulatory events, and unscheduled-shutdown events. Audit integrity is anchored in a Merkle tree per-tenant.

## A.6 Rate-limit envelope

System-and-link endpoints: 1000 req/h authenticated. Safety self-test endpoints rate-limited per system to 60 req/h (defeats abuse). Telemetry WebSocket subscriptions are bounded at 50 simultaneous per credential. Microwave / laser / space-solar regulatory endpoints additionally enforce per-jurisdiction quota envelopes.


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
