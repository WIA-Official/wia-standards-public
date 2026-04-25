# WIA-CITY-017 (traffic-simulation) — Phase 1: Data Format Specification

> **Version:** 1.0.0
> **Status:** Official
> **Phase:** 1 of 4 (Data Format)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This document specifies the canonical data formats for WIA-CITY-017 traffic-simulation deployments. The formats describe **road network**, **traffic-control elements**, **vehicle traces**, **demand matrices**, **simulation scenarios**, and **simulation results** in a way that interoperates with the ISO 14817 ITS data-dictionary family, the ISO 14825 Geographic Data Files (GDF) family, the ISO/TS 19091 cooperative-intersection messages, and the ISO 19082:2025 roadside-module / signal-controller data frames.

### 1.1 Goals

- Provide a JSON / CBOR canonical encoding lossless against ISO 14817-1 ITS data concepts.
- Carry road-network geometry compatible with ISO 14825 GDF and ISO 17572 location-referencing identifiers.
- Express signalised-intersection state and control sequences compatible with ISO/TS 19091 *MAP* and *SPaT* messages and with ISO 19082 roadside-module data frames.
- Allow simulation outputs to be replayed against real-world traffic-management deployments without lossy conversion.

### 1.2 Non-goals

- Defining new physical-layer messaging. V2X PHY/MAC defers to relevant national radio regulations and to the ISO 21217 communications-access reference architecture.
- Defining new road-safety policy. Safety policy defers to ISO 39001 *Road traffic safety management systems* (informative reference).

---

## 2. Top-level Object Model

```
SimulationProject ──┬── Network
                    ├── Demand
                    ├── Scenario[]
                    ├── Run[]
                    └── Result[]
```

Each entity has a stable URI of the form `wia-traffic://<project-id>/<entity-type>/<entity-id>`.

### 2.1 SimulationProject

```json
{
  "type": "SimulationProject",
  "version": "1.0.0",
  "projectId": "<UUID v4 per RFC 9562>",
  "owner": {"name": "string"},
  "iso14817ConceptRegistry": "string (URI of CIDCR mirror or local registry)",
  "iso17572LocationReferencing": "DLR|PLR|AGORA-C",
  "createdAt": "<RFC 3339 date-time>",
  "updatedAt": "<RFC 3339 date-time>"
}
```

`iso17572LocationReferencing` selects the dynamic, pre-coded, or AGORA-C location-referencing method defined by the ISO 17572 series.

### 2.2 Network

A *Network* is an ISO 14825 GDF-compatible road network plus the additional simulation-only attributes required for microscopic and mesoscopic simulation.

```json
{
  "type": "Network",
  "networkId": "string",
  "boundary": {"iso19115Geometry": "<GeoJSON omitted>"},
  "nodes": [
    {
      "nodeId": "string",
      "geometry": {"iso6709": "+37.5326-126.9905/", "altitudeMeters": "number"},
      "type": "intersection|merge|diverge|terminator",
      "iso14825GdfFeatureType": "uint16"
    }
  ],
  "links": [
    {
      "linkId": "string",
      "fromNodeId": "string",
      "toNodeId": "string",
      "geometry": "<GeoJSON LineString>",
      "iso14825GdfFeatureType": "uint16",
      "lanes": [
        {
          "laneId": "string",
          "ordinalFromCenter": "int",
          "directionality": "FORWARD|REVERSE|REVERSIBLE",
          "permittedClasses": ["car","truck","bus","emergency","bike","pedestrian"],
          "speedLimitKph": "number",
          "iso14817ConceptRefs": ["string"]
        }
      ]
    }
  ]
}
```

### 2.3 Signalised intersections

Each intersection that participates in cooperative ITS exchanges is represented by an *Intersection* descriptor. The descriptor MUST be losslessly convertible to and from the ISO/TS 19091 *MAP* (intersection geometry) and *SPaT* (signal phase and timing) messages.

```json
{
  "type": "Intersection",
  "intersectionId": "string",
  "nodeId": "string",
  "iso19091MapMessage": "<base64 ISO/TS 19091 MAP message bytes>",
  "iso19091SpatStream": "<URI of SPaT message stream>",
  "iso19082RoadsideModule": {
    "controllerId": "string",
    "dataFrames": ["string (per ISO 19082:2025 §6 dataframe IDs)"]
  },
  "phases": [
    {
      "phaseId": "string",
      "movements": ["<MovementId>"],
      "minGreenSeconds": "number",
      "maxGreenSeconds": "number",
      "yellowSeconds": "number",
      "redClearanceSeconds": "number"
    }
  ]
}
```

### 2.4 Demand

Demand is expressed as either an O/D (origin–destination) matrix or as an activity-based plan-set. Both forms reference the *Network* via stable identifiers.

```json
{
  "type": "Demand",
  "demandId": "string",
  "kind": "OD-MATRIX|ACTIVITY-BASED",
  "horizon": {"from": "<RFC 3339 date-time>", "to": "<RFC 3339 date-time>"},
  "timeStepSeconds": "uint",
  "matrix": [
    {"originNodeId": "string", "destNodeId": "string", "trips": "number", "vehicleClass": "string"}
  ]
}
```

### 2.5 Scenario

A *Scenario* freezes a Network + Demand + control-policy combination for repeated runs.

```json
{
  "type": "Scenario",
  "scenarioId": "string",
  "networkId": "string",
  "demandId": "string",
  "controlPolicy": "FIXED-TIME|ACTUATED|ADAPTIVE|V2X-COOPERATIVE|LEARNED",
  "weather": "CLEAR|RAIN|SNOW|FOG",
  "incidents": [{"incidentId":"string","linkId":"string","startSeconds":"number","durationSeconds":"number"}],
  "iso14817ConceptRefs": ["string"]
}
```

### 2.6 Run

A *Run* is a single execution of a Scenario.

```json
{
  "type": "Run",
  "runId": "string",
  "scenarioId": "string",
  "rngSeed": "uint64",
  "startedAt": "<RFC 3339 date-time>",
  "completedAt": "<RFC 3339 date-time | null>",
  "engineVersion": "string",
  "engineCapabilities": ["MICROSCOPIC","MESOSCOPIC","MACROSCOPIC","HYBRID"]
}
```

### 2.7 Result

```json
{
  "type": "Result",
  "resultId": "string",
  "runId": "string",
  "metrics": {
    "totalVehicleHours": "number",
    "averageDelaySeconds": "number",
    "averageSpeedKph": "number",
    "co2GramsTotal": "number",
    "intersectionLevelOfService": [
      {"intersectionId":"string","los":"A|B|C|D|E|F","controlDelaySeconds":"number"}
    ]
  },
  "trace": "<URI of vehicle-trace bundle>",
  "evidence": "<URI of evidence bundle (Phase 4 §7)>"
}
```

`intersectionLevelOfService` follows the qualitative A–F scale used in transportation-engineering practice; the numeric `controlDelaySeconds` is the authoritative quantitative metric.

---

## 3. Canonical Encodings

WIA-CITY-017 mandates two interchangeable encodings:

1. **JSON** — RFC 8259, UTF-8, NFC-normalised strings.
2. **CBOR** — RFC 8949, with the *Concise Data Definition Language* (RFC 8610) schema published alongside.

Both encodings MUST be byte-equivalent under round-trip via the canonical CBOR deterministic encoding rules (RFC 8949 §4.2). Trace data uses CBOR with optional COSE_Sign1 / COSE_Encrypt0 protection (RFC 9052 / 9053).

### 3.1 Time

Timestamps use RFC 3339 *date-time*. Simulation steps use a `tStartSeconds` offset relative to `Demand.horizon.from`. Real-world replay against a deployed system uses RFC 5905 NTPv4 (informative reference; the controller's clock authority is the binding source).

### 3.2 Identifiers

UUID v4 (RFC 9562) is the default. ISO 14825 GDF feature identifiers are preserved as `iso14825FeatureId` strings on every node and link to enable round-trip with cartographic exports.

### 3.3 Vehicle trace bundles

A vehicle-trace bundle is a CBOR sequence (RFC 8742) of records:

```json
{
  "vehicleId": "string",
  "vehicleClass": "car|truck|bus|emergency|bike|pedestrian",
  "states": [
    {"tSeconds":"number","linkId":"string","laneId":"string","sMeters":"number","speedKph":"number","accelMpss":"number"}
  ]
}
```

Bundles are typically gzip-compressed and may be streamed via HTTP/2 (RFC 9113) or HTTP/3 (RFC 9114).

---

## 4. Versioning and Compatibility

Phase 1 v1.x payloads are backwards-compatible within the major version. Edition references for normative work:

- ISO 14817-1:2015; ISO 14817-2:2015; ISO 14817-3:2017
- ISO 14825:2011 — *Geographic Data Files (GDF) 5.0.*
- ISO 17572 (all parts) — *Location referencing.*
- ISO 21217:2020 — *CALM communications access for land mobiles architecture.*
- ISO/TS 19091:2017/2019 — *Cooperative ITS V2I/I2V — signalised intersections.*
- ISO 19082:2025 — *Roadside modules / signal controllers data frames.*
- ISO 19115-1:2014; ISO 19107:2003 — *Geographic information.*
- ISO 39001:2012 — *Road traffic safety management* (informative).

---

## 5. Privacy

Vehicle traces, when collected from real deployments, contain personal data. Phase-1 reserves the following privacy controls:

- `Run.privacyPolicy`: a structured policy referencing the operator's ISO/IEC 27701 PIM record.
- `vehicleId`: when carrying real-world identifiers, MUST be pseudonymised with a keyed hash (HMAC-SHA-256, FIPS 198-1) where the key is rotated per privacy policy.
- Trace records MAY be aggregated to link-level summaries when the privacy policy disallows micro-traces.

---

## 6. References

1. ISO 14817-1:2015; ISO 14817-2:2015; ISO 14817-3:2017 — *ITS data dictionaries.*
2. ISO 14825:2011 — *GDF 5.0.*
3. ISO 17572 (all parts) — *Location referencing.*
4. ISO 19082:2025 — *Roadside-module / signal-controller data frames.*
5. ISO 19107:2003 — *Geographic information — Spatial schema.*
6. ISO 19115-1:2014 — *Geographic information — Metadata.*
7. ISO 21217:2020 — *CALM architecture.*
8. ISO/TS 19091:2017/2019 — *Cooperative ITS V2I/I2V signalised intersections.*
9. ISO/IEC 27001:2022; ISO/IEC 27701:2019 — *ISMS / PIM.*
10. ISO 39001:2012 — *Road-traffic safety management* (informative).
11. RFC 3339; RFC 5905; RFC 8259; RFC 8610; RFC 8742; RFC 8949; RFC 9052; RFC 9053; RFC 9113; RFC 9114; RFC 9562.
12. FIPS 180-4; FIPS 198-1.

---

## 7. Vehicle Class Taxonomy

The default vehicle-class taxonomy is informed by ISO 14817-1 ITS data concepts and is reproduced here for normative clarity. Engines MAY extend the taxonomy locally but MUST preserve the canonical class names when interoperating with peers.

| Class | Description | Typical length range (m) |
|-------|-------------|--------------------------|
| `car` | Passenger car | 3.5 – 5.5 |
| `motorcycle` | Two-wheeled powered vehicle | 1.6 – 2.4 |
| `taxi` | Hire car | 4.0 – 5.5 |
| `bus` | Transit bus, coach | 9.0 – 13.0 |
| `articulated-bus` | Articulated transit vehicle | 17.0 – 19.0 |
| `truck-light` | Light commercial vehicle | 4.5 – 7.0 |
| `truck-heavy` | Rigid heavy goods vehicle | 7.0 – 12.0 |
| `truck-articulated` | Articulated heavy goods vehicle | 14.0 – 18.5 |
| `emergency-fire` | Fire-service vehicle | 7.0 – 12.0 |
| `emergency-medical` | Emergency-medical vehicle | 5.0 – 7.5 |
| `emergency-police` | Police vehicle | 4.0 – 5.5 |
| `bike` | Bicycle, e-bike, kick-scooter | 1.5 – 2.0 |
| `pedestrian` | Pedestrian | 0.4 – 0.7 |
| `microtransit` | On-demand transit vehicle | 4.0 – 7.0 |
| `autonomous-shuttle` | Autonomous low-speed shuttle | 4.0 – 6.0 |

The Phase-1 *Lane.permittedClasses* field references these tokens. Locally extended classes MUST be prefixed `x-<vendor>-<token>` to avoid collision.

## 8. Detector Taxonomy

Real-world deployments source vehicle detection from a heterogeneous mix of sensors. WIA-CITY-017 supports the following canonical detector types, each carrying the relevant ISO 14817 data concept reference where one exists.

| Detector | Modality | Notes |
|----------|----------|-------|
| `inductive-loop` | Magnetic flux change | Underground loop, wide deployment |
| `magnetometer-puck` | Magnetic field anomaly | Battery-powered surface puck |
| `microwave-radar` | RF backscatter | Roadside-mounted radar |
| `lidar` | Time-of-flight laser | Roadside or overhead |
| `video-detection` | Image processing | Roadside camera, AI-classified |
| `pneumatic-tube` | Pressure differential | Temporary count surveys |
| `floating-car-data` | Connected vehicles | Aggregated from V2N |
| `mobile-network-probe` | Cellular signalling | Aggregated from operator probes |

Detector outputs are mapped to the Phase-1 *Sensor* descriptor with the appropriate `modality`. Calibration metadata follows the JCGM 100:2008 GUM uncertainty conventions.

## 9. Geographic Coordinate Conventions

All coordinates use ISO 6709:2008 representation. Engines MAY internally use a projected coordinate system (e.g. UTM); export-side coordinates MUST be reprojected to ISO 6709-compatible WGS-84 latitude/longitude unless the partner project explicitly negotiates a projection.

Altitude is expressed in metres above the WGS-84 ellipsoid; deployments operating with a local geoid MUST convert to WGS-84 ellipsoid heights at export time and MUST record the conversion in Phase-1 *SimulationProject* notes.
