# WIA-AUTO-012 — Phase 2: API Interface

> Traffic-management canonical Phase 2: API surface (detectors + signal-plans + incidents + V2X broker).

# WIA-AUTO-012: Traffic Management Specification v1.0

> **Standard ID:** WIA-AUTO-012
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Automotive & Mobility Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Traffic Flow Theory](#2-traffic-flow-theory)
3. [Signal Timing Optimization](#3-signal-timing-optimization)
4. [Congestion Detection and Management](#4-congestion-detection-and-management)
5. [Incident Detection and Response](#5-incident-detection-and-response)
6. [Traffic Prediction Models](#6-traffic-prediction-models)
7. [Data Formats](#7-data-formats)
8. [API Interface](#8-api-interface)
9. [Integration Protocols](#9-integration-protocols)
10. [References](#10-references)

---


## 8. API Interface

### 8.1 Calculate Traffic Flow

```typescript
interface FlowRequest {
  density: number;      // veh/km
  speed: number;        // km/h
  laneCount?: number;   // default: 1
}

interface FlowResponse {
  flow: number;         // veh/h
  capacity: number;     // veh/h
  levelOfService: 'A' | 'B' | 'C' | 'D' | 'E' | 'F';
  regime: 'free-flow' | 'capacity' | 'congested';
}
```

### 8.2 Optimize Signal Timing

```typescript
interface SignalOptimizationRequest {
  phases: Array<{
    name: string;
    volume: number;           // veh/h
    saturationFlow: number;   // veh/h
  }>;
  lostTime: number;          // seconds per phase
  targetDelay?: number;      // seconds (optional)
}

interface SignalOptimizationResponse {
  cycleTime: number;         // seconds
  greenTimes: number[];      // seconds per phase
  delays: number[];          // seconds per phase
  levelOfService: string;
  volumeToCapacity: number;
}
```

### 8.3 Detect Congestion

```typescript
interface CongestionDetectionRequest {
  speed: number;             // km/h
  density: number;           // veh/km
  freeFlowSpeed: number;     // km/h
  jamDensity?: number;       // veh/km (optional)
}

interface CongestionDetectionResponse {
  isCongested: boolean;
  severity: 'none' | 'mild' | 'moderate' | 'severe';
  travelTimeIndex: number;
  recommendedActions: string[];
}
```

### 8.4 Predict Traffic

```typescript
interface TrafficPredictionRequest {
  locationId: string;
  horizon: number;           // minutes ahead
  historicalData?: number[]; // recent flow data
  externalFactors?: {
    weather: string;
    event: string;
    dayOfWeek: string;
  };
}

interface TrafficPredictionResponse {
  predictions: Array<{
    timestamp: Date;
    flow: number;
    speed: number;
    confidence: number;       // 0-1
  }>;
  model: string;
  accuracy: {
    mape: number;
    rmse: number;
  };
}
```

---




---

## A.1 Endpoint reference

```http
GET    /traffic-management/v1/detectors                # list detectors
GET    /traffic-management/v1/detectors/{id}/state     # latest detector state
POST   /traffic-management/v1/signal-plans             # publish signal plan
GET    /traffic-management/v1/signal-plans/{id}        # fetch signal plan
GET    /traffic-management/v1/incidents                # active incident list
POST   /traffic-management/v1/incidents                # report incident
WS     /traffic-management/v1/state/stream             # live state stream
WS     /traffic-management/v1/v2x/stream               # V2X message multiplex
```

Every endpoint follows the discovery convention at `/.well-known/wia-traffic-management`.

## A.2 Detector and state API

`GET /detectors` enumerates detectors with pagination via `?after=cursor&limit=N` (max 200) and filtering by network segment, detector type, and operational status. `GET /detectors/{id}/state` returns the current Phase 1 §A.2 flow-state record. The endpoint exposes the Bayesian posterior over volume and occupancy so downstream simulators can propagate measurement uncertainty into demand forecasts.

## A.3 Signal-plan API

`POST /signal-plans` accepts the Phase 1 §A.3 envelope and returns a stable `signalPlanId`. Plan activation is gated by the operating agency's release process: a plan can be in states `draft`, `under-review`, `approved`, `active`, `superseded`, with state transitions emitted as audit events. The `GET /signal-plans/{id}/timing` endpoint returns the per-phase timing trace at the requested rolling window for offline analysis.

## A.4 Incident API

`POST /incidents` registers an incident; the request is gated by an operator credential at WIA-OMNI-API trust tier 2 or higher. Incidents emit push events on the `/state/stream` WebSocket so downstream variable-message-sign controllers, navigation apps, and emergency-services dashboards can react within the safety-loop's hard time budget. Incident closure follows the symmetric protocol with closure-time finalisation and an after-action report attached to the incident record.

## A.5 V2X message broker

The V2X message broker exposes a multiplexed WebSocket carrying the J2735 message catalogue (BSM, CAM, DENM, MAP, SPaT, TIM). Subscribers can filter by message type, geographic bounding box, and originating-station-class. The broker enforces the IEEE 1609.2 certificate validation chain and rejects messages with revoked or expired certificates per the ETSI TS 102 941 management protocol.

## A.6 Rate-limit envelope

1000 req/h unauthenticated, 5000 req/h authenticated, 10000 req/h premium tier. WebSocket subscriptions count separately and are bounded at 100 simultaneous subscriptions per credential for /state/stream and at 50 for /v2x/stream. Bulk-export endpoints use cursor-based pagination via `?after=cursor&limit=N` (max 1000) and emit `Retry-After` headers under load.

## A.7 Webhook delivery for incident lifecycle

Operators registered as emergency-services dispatchers, traffic-operations centres, or partnered navigation-app providers can subscribe to webhook deliveries on incident-lifecycle events (created, updated, closed). Webhook payloads carry the same envelope shape as the API GET response, signed by the WIA tenant key, and are retried with exponential backoff up to 24 h before the broker logs a permanent-failure event for operator review. Subscriber endpoints MUST validate the webhook signature against the tenant key chain before processing; failure to validate is a conformance defect and is reported to the WIA monitoring sink.

## A.8 Bulk-export and historical-replay API

`GET /detectors/{id}/state/history` returns paginated historical state records for a detector across a configurable time window (default 24 h; max 90 d for premium tier). The response carries the per-record envelope plus a per-window summary (mean volume, mean speed, occupancy distribution, peak time-of-day). `GET /signal-plans/{id}/history` returns the activation history of a signal plan including transitions between drafts, approved, active, superseded, with the operator-of-record signature on each transition. Historical-replay endpoints respect the privacy envelope: per-vehicle V2X messages are NOT exposed through replay; only aggregate flow metrics are.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/traffic-management/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-traffic-management-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/traffic-management-host:1.0.0` ships every traffic-management envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/traffic-management.sh` ships sample envelope generators with no
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
ecosystem. Traffic-management deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
