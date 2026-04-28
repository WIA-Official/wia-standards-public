# WIA-AUTO-009 — Phase 1: Data Format

> Vehicle-semiconductor canonical Phase 1: AEC-grade + MCU/SoC + PMIC + sensor-IC + AI-accelerator envelopes.

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


## 2. Automotive Grade Classifications

### 2.1 AEC-Q100: Failure Mechanism Based Stress Test Qualification

AEC-Q100 is the automotive qualification standard for integrated circuits.

#### 2.1.1 Temperature Grades

| Grade | Temperature Range | Typical Application |
|-------|------------------|---------------------|
| Grade 0 | -40°C to +150°C | Engine compartment, exhaust systems |
| Grade 1 | -40°C to +125°C | Under hood, near engine |
| Grade 2 | -40°C to +105°C | Passenger cabin, dashboard |
| Grade 3 | -40°C to +85°C | Mild environment, trunk |

#### 2.1.2 Required Tests

1. **Preconditioning** (if applicable)
   - Moisture Sensitivity Level (MSL)
   - Baking and storage conditions

2. **Accelerated Environmental Stress Tests**
   - Temperature Cycling (TC): -55°C to +150°C
   - High Temperature Operating Life (HTOL): 1000 hours @ Tj max
   - High Temperature Storage Life (HTSL): 1000 hours
   - Temperature Humidity Bias (THB): 85°C/85%RH

3. **Accelerated Lifetime Simulation**
   - Power Temperature Cycling (PTC)
   - Highly Accelerated Stress Test (HAST)
   - Autoclave or Unbiased HAST (uHAST)

4. **Package Assembly Integrity**
   - Wire Bond Shear
   - Die Shear
   - Solderability
   - Physical Dimensions

5. **Electrical Verification**
   - Electrical Distribution (ESD)
   - Latchup
   - Electrostatic Discharge (ESD): ±2kV HBM minimum

### 2.2 AEC-Q101: Discrete Semiconductor Devices

Qualification for power transistors, diodes, and other discrete components.

**Key Requirements**:
- High-voltage capability (up to 1200V for EVs)
- Low on-resistance for efficiency
- Fast switching for reduced losses
- Avalanche and surge current ratings

### 2.3 AEC-Q200: Passive Components

Standards for resistors, capacitors, and inductors.

**Requirements**:
- High temperature stability
- Low ESR/ESL for power applications
- Vibration and mechanical stress resistance

### 2.4 Reliability Requirements

```
Target FIT Rate: <10 FIT @ 55°C ambient
Qualification Lifetime: 15-20 years
Operating Hours: >200,000 hours
Failure Rate: <0.001% per year
```

---



## 9. Data Formats

### 9.1 Component Specification

```json
{
  "partNumber": "WIA-MCU-ASILD-001",
  "type": "MCU",
  "manufacturer": "WIA Semiconductor",
  "description": "Automotive MCU with ASIL-D safety",
  "specifications": {
    "core": "ARM Cortex-M7",
    "frequency": 400,
    "flash": 4096,
    "ram": 512,
    "package": "LQFP176",
    "temperatureGrade": 1,
    "temperatureRange": {
      "min": -40,
      "max": 125,
      "unit": "celsius"
    },
    "voltage": {
      "nominal": 5.0,
      "min": 4.5,
      "max": 5.5,
      "unit": "V"
    },
    "asilLevel": "ASIL-D",
    "aecQualification": "AEC-Q100 Grade 1"
  },
  "interfaces": [
    "CAN-FD",
    "LIN",
    "FlexRay",
    "SPI",
    "I2C",
    "UART"
  ],
  "safetyFeatures": [
    "Lockstep CPU cores",
    "Memory ECC",
    "Watchdog timer",
    "Voltage monitoring",
    "Temperature monitoring"
  ],
  "reliability": {
    "fitRate": 8,
    "mtbf": 125000000,
    "expectedLifetime": 20,
    "unit": "years"
  }
}
```

### 9.2 Test Results

```json
{
  "componentId": "WIA-MCU-ASILD-001",
  "testDate": "2025-12-26",
  "lotNumber": "LOT2025001",
  "testResults": {
    "temperatureCycling": {
      "status": "PASS",
      "cycles": 1000,
      "failures": 0
    },
    "htol": {
      "status": "PASS",
      "duration": 1000,
      "temperature": 150,
      "failures": 0
    },
    "thb": {
      "status": "PASS",
      "duration": 1000,
      "failures": 0
    },
    "esd": {
      "status": "PASS",
      "hbm": 8000,
      "cdm": 2000
    },
    "latchup": {
      "status": "PASS",
      "temperature": 125
    },
    "vibration": {
      "status": "PASS",
      "duration": 12,
      "acceleration": 30
    }
  },
  "certification": {
    "aecQ100": true,
    "iso26262": "ASIL-D",
    "certificationDate": "2025-12-26",
    "expiryDate": "2030-12-26"
  }
}
```

---




---

## A.1 Automotive-grade-classification envelope

The Phase 1 envelope groups semiconductor records by AEC qualification grade per AEC-Q100 (ICs), AEC-Q101 (discretes), AEC-Q102 (optoelectronics), AEC-Q103 (MEMS sensors), AEC-Q104 (multi-chip modules), and AEC-Q200 (passive components). Grade levels are: Grade 0 (-40 to +150 C — under-hood, exhaust); Grade 1 (-40 to +125 C — most powertrain and chassis); Grade 2 (-40 to +105 C — passenger compartment); Grade 3 (-40 to +85 C — most interior); Grade 4 (0 to +70 C — entertainment-only). Records carry the operating-junction-temperature envelope, the mission-profile envelope (drive-cycle hours, key-on/off cycles, thermal-cycling profile), and the qualification-test summary referencing JEDEC and AEC test specs.

## A.2 MCU-and-SoC descriptor

MCU and SoC descriptors carry: process node (typical automotive: 65 nm / 40 nm / 28 nm / 16 nm / 7 nm / 5 nm with bring-up timelines), core architecture (Arm Cortex-R / Cortex-A / Cortex-M lockstep; RISC-V baseline), per-core ASIL classification per ISO 26262 (lockstep ASIL-D, single-core ASIL-B by claim of independence, freedom-from-interference partitioning), on-chip memory (TCM, L1/L2/L3 cache, ECC-protected SRAM, on-chip flash with sector-by-sector ECC), security envelope (HSM hardware security module per AUTOSAR and EVITA Full / Medium / Light, TRNG, secure-boot ROM with anchored measurement registers), and the SOTIF envelope.

## A.3 Power-management-IC descriptor

PMIC descriptors carry: input-voltage envelope (system 12V / 24V / 48V / 800V HV-bus), output-rail count and per-rail load-step envelope, regulator topology (LDO, switching buck / boost / SEPIC, multi-phase for high-current cores), efficiency profile across load, the AEC-Q100 grade, and the safety-monitoring features (over-voltage / under-voltage / over-current / over-temperature with the documented response-time envelope). HV-side ICs (gate drivers, isolated DC-DC for traction inverters, BMS analog front-ends) carry the isolation-rating envelope per IEC 60664-1 and the partial-discharge envelope.

## A.4 Sensor-IC descriptor

Sensor descriptors carry: sensor class (CMOS image sensor with HDR and LFM-flicker mitigation; mmWave radar 24 GHz / 77 GHz / 79 GHz front-end and DSP; LiDAR transceiver — VCSEL / EEL emitter, SPAD / APD / PIN photoreceiver; ultrasonic transducer driver; inertial — MEMS gyroscope / accelerometer; pressure — MEMS / piezoresistive; magnetic — Hall / TMR / GMR), the per-channel resolution and noise envelope, the calibration envelope, the AEC-Q100 / Q103 grade, and the functional-safety contribution envelope (ASIL decomposition where the sensor participates in an ASIL-rated function).

## A.5 AI-accelerator descriptor

AI-accelerator descriptors carry: TOPS / TFLOPS rating at INT8 / FP16 / BF16; per-watt efficiency; on-chip SRAM size and bandwidth; ISO 26262 evidence package (lockstep, ECC-protected matrix-multiply units, fault-injection campaign results); SOTIF envelope for the inference workload; the supported model envelope (ONNX, TensorRT, TVM, vendor-specific tooling); and the partitioning envelope (which network operators run on the accelerator vs. on host CPU/DSP). Accelerators intended for ADAS L2+/L3 functions carry the freedom-from-interference partitioning evidence per ISO 26262-6.


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
