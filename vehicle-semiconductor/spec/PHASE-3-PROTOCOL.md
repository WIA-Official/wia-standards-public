# WIA-AUTO-009 — Phase 3: Protocol

> Vehicle-semiconductor canonical Phase 3: protocols (ISO 26262 + SOTIF + cybersecurity + qualification + reliability).

# WIA-AUTO-009: Vehicle Semiconductor Specification v1.0

> **Standard ID:** WIA-AUTO-009
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Automotive Semiconductor Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Automotive Grade Classifications](#2-automotive-grade-classifications)
3. [MCU and SoC Architecture](#3-mcu-and-soc-architecture)
4. [Power Management ICs](#4-power-management-ics)
5. [Sensor ICs](#5-sensor-ics)
6. [AI/ML Accelerators](#6-aiml-accelerators)
7. [Functional Safety (ISO 26262)](#7-functional-safety-iso-26262)
8. [Qualification and Testing](#8-qualification-and-testing)
9. [Data Formats](#9-data-formats)
10. [API Interface](#10-api-interface)
11. [References](#11-references)

---


## 7. Functional Safety (ISO 26262)

### 7.1 ASIL Classification

ISO 26262 defines Automotive Safety Integrity Levels:

```
ASIL D: Highest risk of injury or death
- Applications: Steering, braking, airbag
- Failure rate target: <10 FIT

ASIL C: High risk
- Applications: Antilock braking, traction control
- Failure rate target: <100 FIT

ASIL B: Medium risk
- Applications: Headlights, wipers
- Failure rate target: <1000 FIT

ASIL A: Low risk
- Applications: Rear lights, horn
- Failure rate target: <10000 FIT

QM: Quality Management (non-safety)
- Applications: Infotainment, comfort features
```

### 7.2 Safety Mechanisms

#### 7.2.1 Hardware Safety Mechanisms

```
Redundancy:
- Dual CPU cores in lockstep
- Dual power supplies
- Redundant sensors

Error Detection:
- Memory ECC (single/double bit)
- CRC on data buses
- Parity checking
- Watchdog timers

Fault Handling:
- Graceful degradation
- Safe state transition
- Error logging and reporting
```

#### 7.2.2 Diagnostic Coverage

```
Diagnostic Coverage (DC):
- ASIL D: DC ≥ 99%
- ASIL C: DC ≥ 97%
- ASIL B: DC ≥ 90%
- ASIL A: DC ≥ 60%

Diagnostic Test Interval:
- Power-on self-test (POST)
- Periodic background tests
- On-demand diagnostics
```

### 7.3 Safety Metrics

#### 7.3.1 PMHF (Probabilistic Metric for Hardware Failures)

```
Target:
- ASIL D: <10 FIT
- ASIL C: <100 FIT
- ASIL B: <100 FIT

Calculation includes:
- Single-point faults
- Residual faults
- Dual-point faults
```

#### 7.3.2 LFM (Latent Fault Metric)

```
Target:
- ASIL D: <1%
- ASIL C: <10%

Detected by:
- Periodic self-tests
- External monitoring
```

### 7.4 Safety Development Process

```
1. Hazard Analysis and Risk Assessment (HARA)
2. Safety Goals definition
3. Functional Safety Concept
4. Technical Safety Concept
5. Hardware/Software Safety Requirements
6. Design and Implementation
7. Verification and Validation
8. Functional Safety Assessment
9. Production Release
10. Operation and Maintenance
```

---



## 8. Qualification and Testing

### 8.1 Environmental Testing

#### 8.1.1 Temperature Testing

```
Temperature Cycling (TC):
- Condition A: -55°C to +150°C
- Condition B: -40°C to +125°C
- Dwell time: 10-15 minutes
- Cycles: 1000
- Transfer time: <1 minute
```

#### 8.1.2 High Temperature Operating Life (HTOL)

```
Duration: 1000 hours minimum
Temperature: Tj max (125°C or 150°C)
Voltage: VDD max
Sample Size: 231 units (77 per lot)
Acceptance: 0 failures
```

#### 8.1.3 Temperature Humidity Bias (THB)

```
Temperature: 85°C
Humidity: 85% RH
Voltage: VDD max
Duration: 1000 hours
Purpose: Detect moisture-related failures
```

### 8.2 Electrical Testing

#### 8.2.1 Electrostatic Discharge (ESD)

```
Human Body Model (HBM):
- Minimum: ±2kV
- Typical: ±4kV to ±8kV
- Critical pins: ±8kV

Charged Device Model (CDM):
- Minimum: ±500V
- Typical: ±1000V to ±2000V

Machine Model (MM):
- Minimum: ±200V
```

#### 8.2.2 Latchup Testing

```
Current: ±100mA minimum
Temperature: 125°C
Overvoltage: VDD + 2V
Purpose: Ensure no SCR triggering
```

#### 8.2.3 Electrical Overstress (EOS)

```
Overvoltage on supply pins
Reverse polarity protection
Load dump (ISO 7637-2): +100V pulse
Cold crank: down to 4.5V
```

### 8.3 Mechanical Testing

#### 8.3.1 Vibration Testing

```
Frequency Range: 10Hz to 2000Hz
Acceleration: 20g to 50g peak
Duration: 12 hours per axis
Standard: IEC 60068-2-64
```

#### 8.3.2 Mechanical Shock

```
Half-sine pulse: 1500g, 0.5ms
Multiple orientations
Standard: IEC 60068-2-27
```

### 8.4 Reliability Metrics

#### 8.4.1 FIT Rate Calculation

```
FIT = (failures / device-hours) × 10⁹

Acceleration Factor (AF):
AF = exp[Ea/k × (1/T₁ - 1/T₂)]

Where:
- Ea: Activation energy (0.7 eV typical)
- k: Boltzmann constant (8.617×10⁻⁵ eV/K)
- T₁: Use temperature (K)
- T₂: Test temperature (K)
```

#### 8.4.2 MTBF Calculation

```
MTBF = 10⁹ / FIT

Example:
FIT = 10 @ 55°C
MTBF = 10⁹ / 10 = 100 million hours
MTBF = 11,415 years
```

---




---

## A.1 ISO 26262 functional-safety protocol

The functional-safety protocol enforces ISO 26262-1 (vocabulary), -3 (concept phase), -5 (hardware development), -6 (software development), -8 (supporting processes), -9 (ASIL-oriented analyses), -10 (guideline), and -11 (semiconductors). For each device the protocol captures the ASIL claim, the SEooC (Safety Element out of Context) envelope, the assumptions of use, the FMEA / FTA / DFA evidence, the random-hardware-failure metrics (SPFM >= 90% for ASIL-C, >= 99% for ASIL-D; LFM >= 60-80% per the AISL level; PMHF <= 10^-7 / hour for ASIL-C, <= 10^-8 / hour for ASIL-D), and the systematic-capability claim.

## A.2 SOTIF protocol

SOTIF (Safety of the Intended Functionality) protocol per ISO 21448 covers the unknown-unsafe envelope for AI accelerators and perception sensors: the trigger-condition catalogue (e.g., camera blooming under low sun angle; radar ghost echoes from highway barriers; LiDAR specular reflection from wet roads), the operational-design-domain envelope, the residual-risk acceptance, and the post-deployment monitoring plan. SOTIF integrates with the platform's functional-safety envelope per ISO 26262 — a device's SOTIF claim is conditional on the safety mechanisms remaining operational.

## A.3 Cybersecurity protocol

The cybersecurity protocol follows ISO/SAE 21434 plus UN ECE WP.29 R155 / R156. Each device carries the threat-model envelope, the cybersecurity-claim envelope (HSM-anchored secure boot; signed firmware; debug-port lockdown; physical attack resistance class per JIL Common Criteria scheme), the post-shipment vulnerability-management envelope, and the OTA-update protocol envelope per ISO 24089. Device-level CVE management feeds the OEM's vehicle-cybersecurity management system.

## A.4 Qualification-and-testing protocol

AEC-Q100 qualification testing follows the test-method matrix Group A (environment) / Group B (accelerated environment) / Group C (package integrity) / Group D (electrical verification) / Group E (defect screening) / Group F (cavity package integrity, where applicable) / Group G (die fabrication reliability). Each group carries the documented sample size, the duration, the bias condition, the inspection criteria, and the zero-defects acceptance per the AEC document. Re-qualification triggers (process change, design change, qualification-failure remediation) are documented in the manufacturer's change-control envelope.

## A.5 Reliability and mission-profile protocol

Mission-profile protocol covers the operator's per-vehicle drive-cycle envelope (urban / highway / mountain; daily / weekly / annual; expected lifetime miles), the thermal envelope (ambient + self-heating + cabin heat-soak; cyclic thermal expansion stress), the vibration envelope per ISO 16750-3 / IEC 60068-2-64, the EMC envelope per ISO 11452 / CISPR 25, and the wear-out mechanism budget (electromigration, NBTI / PBTI, TDDB, hot-carrier injection, stress-migration). Mission-profile derating sets the de-rated SOA (safe operating area) for fielded use.

## A.6 Replay and integrity defence

Standard 96-bit nonce + 300-second skew window + 600-second seen-nonce cache for the qualification-result registration control plane. Test-result records are signed at registration time and the signature chain is anchored into a Merkle tree per-manufacturer so revisions to the qualification history can be detected during post-recall investigations. Test-house telemetry uses mTLS with per-machine monotonic counters; replay attempts are detected and dropped at the broker.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/vehicle-semiconductor/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-vehicle-semiconductor-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/vehicle-semiconductor-host:1.0.0` ships every vehicle-semiconductor envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/vehicle-semiconductor.sh` ships sample envelope generators with no
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
ecosystem. Vehicle-semiconductor deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
