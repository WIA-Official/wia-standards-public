# WIA-AUTO-009 — Phase 2: API Interface

> Vehicle-semiconductor canonical Phase 2: API surface (devices + qualification + safety-passport + telemetry).

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


## 10. API Interface

### 10.1 Component Classification

```typescript
interface ComponentClassification {
  partNumber: string;
  type: SemiconductorType;
  temperatureGrade: 0 | 1 | 2 | 3;
  voltageClass: string;
  asilLevel: 'QM' | 'ASIL-A' | 'ASIL-B' | 'ASIL-C' | 'ASIL-D';
  application: string;
}

interface ClassificationResult {
  isAutomotiveGrade: boolean;
  aecQualification: string;
  recommendedApplications: string[];
  safetyRating: string;
  reliabilityMetrics: ReliabilityMetrics;
}
```

### 10.2 AEC-Q100 Validation

```typescript
interface AECQ100Validation {
  componentId: string;
  testResults: {
    thermalCycling: 'PASS' | 'FAIL' | 'PENDING';
    htol: 'PASS' | 'FAIL' | 'PENDING';
    htsl: 'PASS' | 'FAIL' | 'PENDING';
    thb: 'PASS' | 'FAIL' | 'PENDING';
    eMSL: 'PASS' | 'FAIL' | 'PENDING';
    electricalDisturbance: 'PASS' | 'FAIL' | 'PENDING';
    esd: 'PASS' | 'FAIL' | 'PENDING';
    latchup: 'PASS' | 'FAIL' | 'PENDING';
  };
}

interface ValidationResult {
  isCompliant: boolean;
  certificationLevel: string;
  failedTests: string[];
  recommendations: string[];
  expiryDate: Date;
}
```

### 10.3 Reliability Calculation

```typescript
interface ReliabilityParams {
  fitRate: number;           // Failures in time @ reference temp
  operatingTemp: number;      // Celsius
  referenceTemp: number;      // Celsius (typically 55°C)
  activationEnergy: number;   // eV (typically 0.7)
}

interface ReliabilityMetrics {
  fitRate: number;            // Adjusted for operating temp
  mtbf: number;               // Hours
  failureRate: number;        // Per year
  expectedLifetime: number;   // Years
  confidence: number;         // Percentage
}
```

### 10.4 Safety Assessment

```typescript
interface SafetyAssessment {
  asilRequired: 'QM' | 'ASIL-A' | 'ASIL-B' | 'ASIL-C' | 'ASIL-D';
  application: string;
  componentSpecs: ComponentSpecification;
}

interface SafetyResult {
  isCompliant: boolean;
  asilAchieved: string;
  safetyMechanisms: string[];
  diagnosticCoverage: number;  // Percentage
  pmhf: number;                // FIT
  lfm: number;                 // Percentage
  recommendations: string[];
}
```

---




---

## A.1 Endpoint reference

```http
POST /vehicle-semiconductor/v1/devices              # register device record
GET  /vehicle-semiconductor/v1/devices/{id}         # fetch device record
GET  /vehicle-semiconductor/v1/devices/{id}/qual    # AEC qualification summary
POST /vehicle-semiconductor/v1/qual-results         # contribute qualification test
GET  /vehicle-semiconductor/v1/passports/{partNo}   # device safety passport
WS   /vehicle-semiconductor/v1/test/stream          # qualification-test telemetry
```

Every endpoint follows the discovery convention at `/.well-known/vehicle-semiconductor`.

## A.2 Device-record API

`POST /devices` accepts the Phase 1 §A.1-§A.5 envelope and returns a stable `deviceId`. Subsequent qualification reports reference the `deviceId`. Read endpoints expose the per-die mission-profile envelope and the post-qualification reliability summary so OEM and Tier-1 customers can complete their item-level safety case without re-querying the manufacturer's portal. Device-record updates are versioned with the prior version preserved so post-recall investigations always resolve to the as-shipped record.

## A.3 Qualification-result API

`POST /qual-results` accepts a qualification-test envelope per the AEC test catalogue: HTOL (high-temperature operating life), HTGB (high-temperature gate bias), TC (temperature cycle), HAST (highly-accelerated stress test), TH (temperature humidity bias), AC (autoclave), ESD-HBM and ESD-CDM, latch-up, EMC per CISPR 25 + ISO 11452. Each report carries the test-house identifier, the per-test sample size and zero-defects acceptance per AEC-Q100, the failure-analysis envelope where applicable, and the test-house certificate per ISO/IEC 17025.

## A.4 Safety-passport API

`GET /passports/{partNo}` returns the device safety passport: ISO 26262 ASIL claim with the supporting Safety Element out of Context (SEooC) envelope, the FMEA / FTA / DFA evidence references, the random-hardware-failure metrics (SPFM, LFM, PMHF), the systematic-capability envelope, and the supported safety mechanisms (CRC, ECC, lockstep, software self-test). Devices intended as ADAS perception accelerators additionally carry the SOTIF claim envelope per ISO 21448.

## A.5 Telemetry WebSocket

The test-stream WebSocket multiplexes live qualification-test telemetry (oven temperature, bias supply, leakage current, parametric drift) for HTOL, HAST, TC, and HTGB test runs. Subscribers can filter by test-house and by device-class. The broker emits test-progress events and test-completion events with the digested result payload. Subscriber endpoints MUST validate the test-house certificate per ISO/IEC 17025 before accepting the data as authoritative.

## A.6 Rate-limit envelope

1000 req/h unauthenticated, 5000 req/h authenticated, 10000 req/h premium tier. WebSocket subscriptions are bounded at 50 simultaneous per credential. Bulk-export endpoints use cursor-based pagination via `?after=cursor&limit=N` (max 200) and emit `Retry-After` headers under load.

## A.7 Webhook delivery for safety-passport changes

Operators registered as OEM ADAS engineering, OEM safety, or Tier-1 platform integrators can subscribe to webhook deliveries on safety-passport changes (recall notice, errata-sheet update, safety-mechanism deprecation, ASIL re-claim). Webhook payloads carry the same envelope shape as the API GET response, signed by the WIA tenant key, and retried with exponential backoff up to 24 h before the broker logs a permanent-failure event for operator review.


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
