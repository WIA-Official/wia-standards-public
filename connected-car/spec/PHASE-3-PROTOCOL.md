# WIA-AUTO-008 PHASE 3 — Protocol Specification

**Standard:** WIA-AUTO-008
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

consents: {
    level: ConsentLevel;
    granted: boolean;
    timestamp: Date;
    expiresAt?: Date;
  }[];
  lastUpdated: Date;
}
```

#### 6.3.2 Consent User Interface

**Requirements**:
- Clear, plain language descriptions
- Granular control per data category
- Easy opt-in/opt-out mechanism
- Accessible from vehicle and mobile app
- Audit trail of consent changes

---

## 7. Cloud Platform Integration

### 7.1 Cloud Architecture Patterns

#### 7.1.1 Microservices Architecture

```
Service Catalog:
├── Ingestion Service
│   └── MQTT/HTTP endpoint for telemetry
├── Authentication Service
│   └── Vehicle and user authentication
├── Telemetry Service
│   └── Process and store vehicle data
├── Diagnostics Service
│   └── Analyze health and predict issues
├── OTA Service
│   └── Manage update packages and delivery
├── Notification Service
│   └── Push alerts to users
├── Analytics Service
│   └── Generate insights and reports
└── API Gateway
    └── Expose REST/GraphQL APIs
```

#### 7.1.2 Event-Driven Architecture

```
Event Flow:
1. Vehicle sends event → Event Bus (Kafka/Kinesis)
2. Event consumed by multiple services
3. Services process independently
4. Services emit new events
5. Aggregate results for user applications

Event Types:
  - Telemetry.DataPoint
  - Diagnostics.DTCDetected
  - OTA.UpdateAvailable
  - Alert.CriticalWarning
  - User.CommandReceived
```

### 7.2 Supported Cloud Platforms

#### 7.2.1 AWS IoT Core

**Components**:
- **IoT Device Gateway**: MQTT, WebSocket, HTTPS
- **Device Shadow**: Virtual device state
- **Rules Engine**: Route and transform messages
- **Fleet Indexing**: Search and aggregate fleet data
- **Jobs**: Manage OTA updates

**Integration**:
```typescript
const config = {
  endpoint: 'xxxxx.iot.us-east-1.amazonaws.com',
  clientId: `vehicle-${vin}`,
  protocol: 'mqtts',
  port: 8883,
  certificate: '/path/to/device-cert.pem',
  privateKey: '/path/to/private-key.pem',
  caCert: '/path/to/root-ca.pem'
};
```

#### 7.2.2 Azure IoT Hub

**Components**:
- **Device-to-Cloud**: Telemetry ingestion
- **Cloud-to-Device**: Commands and notifications
- **Device Twins**: Device state management
- **Direct Methods**: Synchronous device control
- **Automatic Device Management**: OTA updates

**Integration**:
```typescript
const config = {
  connectionString: 'HostName=xxxxx.azure-devices.net;DeviceId=vehicle-${vin};SharedAccessKey=xxxxx',
  protocol: 'amqp', // or 'mqtt', 'https'
};
```

#### 7.2.3 Google Cloud IoT Core

**Note**: Google Cloud IoT Core was retired August 16, 2023.
**Alternative**: Use Google Cloud Pub/Sub + Compute

**Components**:
- **Pub/Sub**: Message broker
- **Dataflow**: Stream processing
- **BigQuery**: Data warehousing
- **Cloud Functions**: Event processing

### 7.3 Communication Protocols

#### 7.3.1 MQTT (Message Queuing Telemetry Transport)

**Specifications**:
- **Version**: MQTT 3.1.1 or 5.0
- **QoS Levels**:
  - QoS 0: At most once
  - QoS 1: At least once (recommended for telemetry)
  - QoS 2: Exactly once (for critical commands)
- **Topic Structure**:
  ```
  vehicles/{vin}/telemetry/location
  vehicles/{vin}/telemetry/diagnostics
  vehicles/{vin}/commands/lock
  vehicles/{vin}/events/alert
  ```

**Advantages**:
- Lightweight protocol
- Low bandwidth usage
- Bi-directional communication
- Built-in QoS levels

#### 7.3.2 HTTPS/REST

**Specifications**:
- **Version**: HTTP/2 or HTTP/3
- **Security**: TLS 1.3
- **Methods**: GET, POST, PUT, DELETE
- **Authentication**: OAuth 2.0 or JWT

**Endpoint Examples**:
```
POST   /v1/vehicles/{vin}/telemetry
GET    /v1/vehicles/{vin}/status
POST   /v1/vehicles/{vin}/diagnostics/run
GET    /v1/vehicles/{vin}/ota/updates
POST   /v1/vehicles/{vin}/ota/install
```

**Advantages**:
- Widely supported
- Human-readable
- Stateless
- Cacheable

#### 7.3.3 WebSocket

**Specifications**:
- **Protocol**: RFC 6455
- **Security**: WSS (WebSocket Secure)
- **Use Cases**: Real-time streaming, bidirectional communication

**Connection**:
```javascript
const ws = new WebSocket('wss://api.example.com/vehicles/VIN123/stream');
ws.onmessage = (event) => {
  const telemetry = JSON.parse(event.data);
  console.log('Real-time data:', telemetry);
};
```

#### 7.3.4 CoAP (Constrained Application Protocol)

**Specifications**:
- **Transport**: UDP
- **Security**: DTLS
- **Use Cases**: Resource-constrained devices

**Advantages**:
- Low overhead
- Efficient for IoT
- Similar to HTTP semantics

---

## 8. Data Formats

### 8.1 Telemetry Data Format

#### 8.1.1 JSON Schema

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "VehicleTelemetry",
  "type": "object",
  "required": ["vehicleId", "timestamp", "location", "status"],
  "properties": {
    "vehicleId": {
      "type": "string",
      "pattern": "^[A-HJ-NPR-Z0-9]{17}$",
      "description": "Vehicle Identification Number (VIN)"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time",
      "description": "ISO 8601 timestamp"
    },
    "location": {
      "type": "object",
      "required": ["latitude", "longitude"],
      "properties": {
        "latitude": { "type": "number", "minimum": -90, "maximum": 90 },
        "longitude": { "type": "number", "minimum": -180, "maximum": 180 },
        "altitude": { "type": "number" },
        "heading": { "type": "number", "minimum": 0, "maximum": 360 },
        "speed": { "type": "number", "minimum": 0 }
      }
    },
    "status": {
      "type": "object",
      "properties": {
        "ignition": { "type": "boolean" },
        "odometer": { "type": "number" },
        "fuelLevel": { "type": "number", "minimum": 0, "maximum": 100 },
        "batteryVoltage": { "type": "number" },
        "engineRpm": { "type": "number", "minimum": 0 }
      }
    },
    "diagnostics": {
      "type": "object",
      "properties": {
        "dtcs": {
          "type": "array",
          "items": {
            "type": "object",
            "properties": {
              "code": { "type": "string" },
              "description": { "type": "string" },
              "severity": {
                "type": "string",
                "enum": ["critical", "high", "medium", "low"]
              }
            }
          }
        }
      }
    }
  }
}
```

#### 8.1.2 Example Telemetry Message

```json
{
  "vehicleId": "1HGBH41JXMN109186",
  "timestamp": "2025-12-26T10:30:00Z",
  "location": {
    "latitude": 37.7749,
    "longitude": -122.4194,
    "altitude": 15.5,
    "heading": 270,
    "speed": 65.5
  },
  "status": {
    "ignition": true,
    "odometer": 45230.5,
    "fuelLevel": 62.3,
    "batteryVoltage": 12.6,
    "engineRpm": 2150,
    "engineTemp": 92,
    "tirePressure": {
      "frontLeft": 35,
      "frontRight": 35,
      "rearLeft": 34,
      "rearRight": 34
    }
  },
  "diagnostics": {
    "dtcs": [],
    "warnings": []
  },
  "metadata": {
    "firmwareVersion": "2.5.0",
    "tcuSerial": "TCU-20241015-001",
    "dataVersion": "1.0"
  }
}
```

### 8.2 Protocol Buffers (Protobuf)

**Advantages**:
- Compact binary format
- 3-10x smaller than JSON
- Faster serialization/deserialization
- Schema evolution support

```protobuf
syntax = "proto3";

message VehicleTelemetry {
  string vehicle_id = 1;
  int64 timestamp = 2;

  message Location {
    double latitude = 1;
    double longitude = 2;
    double altitude = 3;
    double heading = 4;
    double speed = 5;
  }
  Location location = 3;

  message Status {
    bool ignition = 1;
    double odometer = 2;
    double fuel_level = 3;
    double battery_voltage = 4;
    int32 engine_rpm = 5;
  }
  Status status = 4;

  message Diagnostics {
    repeated string dtc_codes = 1;
  }
  Diagnostics diagnostics = 5;
}
```

---

## 9. API Interface

### 9.1 REST API Specification

#### 9.1.1 Authentication

**OAuth 2.0 Flow**:
```
1. Client → Authorization Server: Request access token
2. Authorization Server → Client: Access token + refresh token
3. Client → API: Request with Bearer token
4. API → Client: Response
```

**Headers**:
```http
Authorization: Bearer eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json
X-API-Version: 1.0
```

#### 9.1.2 Endpoints

**Vehicle Telemetry**

```http
POST /v1/vehicles/{vin}/telemetry
Content-Type: application/json

{
  "timestamp": "2025-12-26T10:30:00Z",
  "location": { "latitude": 37.7749, "longitude": -122.4194 },
  "status": { "speed": 65.5, "fuelLevel": 62.3 }
}

Response: 201 Created
{
  "id": "tel-123456",
  "status": "accepted",
  "timestamp": "2025-12-26T10:30:01Z"
}
```

**Vehicle Status**

```http
GET /v1/vehicles/{vin}/status

Response: 200 OK
{
  "vehicleId": "1HGBH41JXMN109186",


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-3-PROTOCOL.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-3-PROTOCOL. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P3-PROTOCOL-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-3-PROTOCOL validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
