# WIA-AUTO-021 — Phase 2: API Interface

> Vehicle-lightweight-material canonical Phase 2: API surface (materials + processes + joints + test-results + passport).

# WIA-AUTO-021: Vehicle Lightweight Material Specification v1.0

> **Standard ID:** WIA-AUTO-021
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Automotive Materials Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Material Types](#2-material-types)
3. [Mechanical Properties](#3-mechanical-properties)
4. [Manufacturing Processes](#4-manufacturing-processes)
5. [Joining Technologies](#5-joining-technologies)
6. [Crash Safety Performance](#6-crash-safety-performance)
7. [Corrosion Resistance](#7-corrosion-resistance)
8. [Recyclability](#8-recyclability)
9. [Data Formats](#9-data-formats)
10. [API Interface](#10-api-interface)
11. [Testing Standards](#11-testing-standards)
12. [References](#12-references)

---


## 10. API Interface

### 10.1 Material Database API

#### 10.1.1 Get Material Properties

```typescript
interface MaterialRequest {
  materialId: string;
  properties?: string[]; // e.g., ['mechanical', 'thermal']
  temperature?: number; // °C (for temperature-dependent properties)
}

interface MaterialResponse {
  material: {
    id: string;
    name: string;
    category: string;
    properties: {
      mechanical?: MechanicalProperties;
      thermal?: ThermalProperties;
      electrical?: ElectricalProperties;
    };
  };
  metadata: {
    source: string;
    testStandards: string[];
    lastUpdated: string;
  };
}
```

#### 10.1.2 Compare Materials

```typescript
interface ComparisonRequest {
  materialIds: string[];
  metrics: string[]; // e.g., ['specificStrength', 'cost', 'recyclability']
  weightingFactors?: Record<string, number>; // For scoring
}

interface ComparisonResponse {
  materials: MaterialSummary[];
  comparison: {
    metric: string;
    values: number[];
    normalized: number[]; // 0-1 scale
  }[];
  recommendation: {
    materialId: string;
    score: number;
    reasoning: string;
  };
}
```

### 10.2 Weight Reduction Calculator API

```typescript
interface WeightReductionRequest {
  component: string;
  originalMaterial: string;
  newMaterial: string;
  geometry: {
    type: 'sheet' | 'tube' | 'extrusion' | 'casting' | 'custom';
    dimensions: Record<string, number>;
  };
  quantity?: number; // For total vehicle calculation
}

interface WeightReductionResponse {
  weightSaved: number; // kg per component
  totalWeightSaved: number; // kg for all components
  percentReduction: number;
  fuelSavings: {
    percent: number;
    liters100km: number;
    co2Reduction: number; // g/km
  };
  costImplication: {
    materialCost: number;
    toolingCost: number;
    totalCost: number;
    paybackPeriod: number; // years
  };
}
```

### 10.3 Material Selection API

```typescript
interface SelectionCriteria {
  application: string;
  loadCases: LoadCase[];
  constraints: {
    maxWeight?: number;
    maxCost?: number;
    minStrength?: number;
    minStiffness?: number;
    corrosionResistance?: 'low' | 'medium' | 'high';
    formability?: 'low' | 'medium' | 'high';
  };
  preferences?: {
    recyclability?: number; // 0-1
    sustainability?: number; // 0-1
    availability?: number; // 0-1
  };
}

interface MaterialRecommendation {
  ranking: {
    materialId: string;
    score: number;
    pros: string[];
    cons: string[];
  }[];
  analysis: {
    weightVsStrength: ChartData;
    costVsPerformance: ChartData;
    ashbyChart: ChartData;
  };
}
```

### 10.4 Crash Simulation API

```typescript
interface CrashSimulationRequest {
  material: string;
  geometry: {
    type: 'tube' | 'beam' | 'panel';
    dimensions: Record<string, number>;
    wallThickness: number;
  };
  impactConditions: {
    velocity: number; // m/s
    mass: number; // kg
    angle: number; // degrees
  };
  constraints: 'fixed' | 'simply-supported' | 'free';
}

interface CrashSimulationResponse {
  energyAbsorbed: number; // J
  peakForce: number; // N
  meanForce: number; // N
  crushDistance: number; // mm
  specificEnergyAbsorption: number; // kJ/kg
  deformationMode: string;
  timeline: {
    time: number[]; // ms
    force: number[]; // N
    displacement: number[]; // mm
  };
}
```

---




---

## A.1 Endpoint reference

```http
POST /vehicle-lightweight-material/v1/materials      # register material record
GET  /vehicle-lightweight-material/v1/materials/{id} # fetch material record
POST /vehicle-lightweight-material/v1/processes      # register process record
POST /vehicle-lightweight-material/v1/joints         # register joint record
POST /vehicle-lightweight-material/v1/test-results   # contribute test measurement
GET  /vehicle-lightweight-material/v1/passports/{vin}# vehicle material passport
WS   /vehicle-lightweight-material/v1/coupon/stream  # live coupon-test telemetry
```

Every endpoint follows the discovery convention at `/.well-known/wia-vehicle-lightweight-material`.

## A.2 Material-record API

`POST /materials` accepts the Phase 1 §A.1 envelope and returns a stable `materialId`. Subsequent characterisation reports reference the `materialId`. Read endpoints expose the Bayesian posterior over yield, ultimate, and fatigue-limit so downstream FE simulators can propagate measurement uncertainty into structural predictions. Material-record updates are versioned with the prior version preserved so post-build investigations always resolve to the as-tested record.

## A.3 Process and joint API

`POST /processes` registers a manufacturing-process record per Phase 1 §A.3; `POST /joints` registers a joining-technology record per Phase 1 §A.4 and cross-references the joint's parent material records. Test-result endpoints accept the binary measurement payload (stress-strain curve, S-N curve, fracture-toughness load-displacement curve) plus the analyst signature; calibration-record references are mandatory and validated against the operator's ISO/IEC 17025 envelope.

## A.4 Vehicle-material-passport API

`GET /passports/{vin}` returns the vehicle material passport: per-component material breakdown, per-component recyclate content, per-component end-of-life dismantling instructions, and the post-warranty repairability envelope. The passport cross-references the EU Digital Product Passport schema for end-of-life vehicles and the EU Battery Passport for the propulsion battery. Access to per-VIN passport data requires owner consent or the dismantler's authorised credential.

## A.5 Coupon-test WebSocket

The coupon-stream WebSocket multiplexes live test-machine telemetry (force, displacement, strain, temperature, acoustic emission) for tensile, fatigue, and fracture-toughness tests. Subscribers can filter by test-type and by test-machine identifier; the broker emits test-completion events with the digested result payload. Subscriber endpoints MUST validate the test-machine certificate against the operator's ISO/IEC 17025 envelope before accepting the data as authoritative.

## A.6 Rate-limit envelope

1000 req/h unauthenticated, 5000 req/h authenticated, 10000 req/h premium tier. WebSocket subscriptions are bounded at 50 simultaneous per credential. Bulk-export endpoints use cursor-based pagination via `?after=cursor&limit=N` (max 200) and emit `Retry-After` headers under load.

## A.7 Webhook delivery for test completion

Operators registered as material-test labs, OEM materials engineering, or partnered Tier-1 suppliers can subscribe to webhook deliveries on test-completion events. Webhook payloads carry the same envelope shape as the API GET response, signed by the WIA tenant key, and retried with exponential backoff up to 24 h before the broker logs a permanent-failure event for operator review.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/vehicle-lightweight-material/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-vehicle-lightweight-material-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/vehicle-lightweight-material-host:1.0.0` ships every vehicle-lightweight-material envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/vehicle-lightweight-material.sh` ships sample envelope generators with no
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
ecosystem. Vehicle-lightweight-material deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
