# WIA-IND-028 PHASE 4 — Integration Specification

**Standard:** WIA-IND-028
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

| SimaaS | 99.5% | < 5 s | Business hours |
| AIaaS | 99.9% | < 200 ms | 24/7 |
| AaaS | 99.95% | < 50 ms | 24/7 |
| TaaS | 99.5% | < 100 ms | Business hours |

### 14.5 Multi-tenancy

#### 14.5.1 Data Isolation
- Separate database per tenant
- Row-level security
- Encryption at rest and in transit

#### 14.5.2 Customization
- Tenant-specific configurations
- Custom dashboards
- Branding (white-label option)

#### 14.5.3 Resource Limits
- CPU/memory quotas
- Storage limits
- API rate limiting

---

## 15. Data Models

### 15.1 Factory Model

```json
{
  "factoryId": "FAC-001",
  "name": "Assembly Plant A",
  "location": {
    "address": "123 Industry Blvd, Detroit, MI 48201",
    "gps": {
      "latitude": 42.3314,
      "longitude": -83.0458
    }
  },
  "size": {
    "area": 15000,
    "unit": "sqm"
  },
  "layout": {
    "buildingGeometry": "base64-encoded-3d-model",
    "productionLines": [...]
  },
  "infrastructure": {
    "power": {
      "mainCapacity": 5000,
      "unit": "kW",
      "voltage": 480,
      "phases": 3
    },
    "compressedAir": {
      "pressure": 90,
      "unit": "psi",
      "capacity": 1000,
      "cfm": true
    }
  },
  "metadata": {
    "created": "2025-01-15T10:00:00Z",
    "updated": "2025-12-27T14:30:00Z"
  }
}
```

### 15.2 Digital Twin Model

```json
{
  "twinId": "TWIN-001",
  "factoryId": "FAC-001",
  "name": "Assembly Plant A Digital Twin",
  "fidelityLevel": 3,
  "synchronization": {
    "enabled": true,
    "method": "real-time",
    "frequency": 1000,
    "latency": 85
  },
  "components": [
    {
      "componentId": "ROBOT-A1",
      "type": "robot-arm",
      "physicalId": "UR10e-12345",
      "model": "assets/robots/ur10e.gltf",
      "position": {"x": 10.5, "y": 5.2, "z": 0.0},
      "orientation": {"rx": 0, "ry": 0, "rz": 90},
      "sensors": [
        {
          "sensorId": "TEMP-01",
          "type": "temperature",
          "value": 45.2,
          "unit": "celsius",
          "timestamp": "2025-12-27T14:30:15Z"
        }
      ]
    }
  ],
  "predictiveModels": [
    {
      "modelId": "PM-001",
      "type": "predictive-maintenance",
      "algorithm": "random-forest",
      "accuracy": 0.93,
      "lastTrained": "2025-12-20T08:00:00Z"
    }
  ]
}
```

### 15.3 Simulation Model

```json
{
  "simulationId": "SIM-001",
  "name": "Peak Demand Scenario",
  "type": "discrete-event",
  "duration": 86400,
  "parameters": {
    "productMix": [
      {"productId": "PROD-A", "demandRate": 100},
      {"productId": "PROD-B", "demandRate": 75}
    ],
    "shifts": 3,
    "overtime": 4
  },
  "results": {
    "throughput": 1850,
    "utilization": 87.5,
    "bottlenecks": ["WS-03"],
    "energyCost": 4250.50
  }
}
```

---

## 16. API Specifications

### 16.1 RESTful API

Base URL: `https://api.digitalfactory.example.com/v1`

#### 16.1.1 Authentication
```http
Authorization: Bearer <JWT_TOKEN>
```

#### 16.1.2 Endpoints

**Create Digital Twin**
```http
POST /factories/{factoryId}/twins
Content-Type: application/json

{
  "name": "Assembly Plant A",
  "fidelityLevel": 3,
  "enableRealTimeSync": true
}

Response: 201 Created
{
  "twinId": "TWIN-001",
  "status": "created"
}
```

**Get Twin Status**
```http
GET /twins/{twinId}/status

Response: 200 OK
{
  "twinId": "TWIN-001",
  "syncStatus": "active",
  "lastUpdate": "2025-12-27T14:30:45Z",
  "latency": 87
}
```

**Run Simulation**
```http
POST /simulations
Content-Type: application/json

{
  "twinId": "TWIN-001",
  "scenario": "peak-demand",
  "duration": 86400
}

Response: 202 Accepted
{
  "simulationId": "SIM-001",
  "status": "running",
  "estimatedCompletion": "2025-12-27T15:00:00Z"
}
```

### 16.2 WebSocket API

Real-time data streaming.

**Connect**
```javascript
ws://api.digitalfactory.example.com/v1/ws?token=JWT_TOKEN
```

**Subscribe to Twin Updates**
```json
{
  "action": "subscribe",
  "twinId": "TWIN-001",
  "dataPoints": ["sensors", "production", "energy"]
}
```

**Receive Updates**
```json
{
  "twinId": "TWIN-001",
  "timestamp": "2025-12-27T14:30:50Z",
  "data": {
    "sensors": {
      "TEMP-01": 45.3,
      "VIBR-01": 2.1
    },
    "production": {
      "output": 1247
    }
  }
}
```

### 16.3 GraphQL API

Flexible querying.

**Endpoint**: `https://api.digitalfactory.example.com/v1/graphql`

**Query Example**
```graphql
query {
  factory(id: "FAC-001") {
    name
    twins {
      twinId
      status
      components {
        componentId
        type
        sensors {
          sensorId
          value
          unit
        }
      }
    }
  }
}
```

---

## 17. Security

### 17.1 Authentication

- **OAuth 2.0**: Authorization framework
- **OpenID Connect**: Identity layer
- **Multi-factor Authentication (MFA)**: Required for admin access
- **SSO**: SAML 2.0, LDAP, Active Directory integration

### 17.2 Authorization

- **Role-Based Access Control (RBAC)**: Permissions by role
- **Attribute-Based Access Control (ABAC)**: Context-aware permissions
- **Least Privilege**: Minimum necessary permissions

### 17.3 Data Protection

- **Encryption at Rest**: AES-256
- **Encryption in Transit**: TLS 1.3
- **Key Management**: HSM or cloud KMS
- **Data Masking**: PII protection in logs

### 17.4 Network Security

- **VPN**: Encrypted connection to factory
- **Firewall**: Industrial firewall (IEC 62443)
- **Network Segmentation**: Separate OT and IT networks
- **DDoS Protection**: Rate limiting, IP filtering

### 17.5 Compliance

- **GDPR**: Data protection and privacy (EU)
- **CCPA**: Consumer privacy (California)
- **SOC 2**: Security and availability controls
- **ISO 27001**: Information security management
- **IEC 62443**: Industrial cybersecurity
- **NIST Cybersecurity Framework**

---

## 18. Interoperability

### 18.1 Standards Support

- **OPC UA**: Industrial communication
- **MQTT**: IoT messaging
- **MTConnect**: Manufacturing data exchange
- **AutomationML**: Engineering data exchange
- **B2MML**: Business to manufacturing markup language

### 18.2 Data Formats

- **JSON**: API data exchange
- **Protobuf**: Efficient serialization
- **glTF**: 3D model exchange
- **STEP**: CAD data exchange
- **CSV**: Tabular data

### 18.3 Integration Patterns

- **REST API**: Request/response
- **WebSocket**: Real-time streaming
- **Message Queue**: Asynchronous (RabbitMQ, Kafka)
- **Event-Driven**: Pub/sub (MQTT, Azure Event Hub)

---

## 19. Performance Requirements

### 19.1 Latency

| Operation | Maximum Latency |
|-----------|-----------------|
| Real-time sensor sync | < 100 ms |
| Dashboard update | < 500 ms |
| Simulation start | < 5 s |
| API request | < 200 ms |
| 3D rendering | < 16 ms (60 FPS) |

### 19.2 Throughput

| Metric | Minimum |
|--------|---------|
| Sensor data points/sec | 10,000 |
| API requests/sec | 1,000 |
| Concurrent users | 500 |
| WebSocket connections | 10,000 |

### 19.3 Scalability

- **Horizontal Scaling**: Add servers to handle load
- **Auto-scaling**: Based on CPU, memory, request rate
- **Load Balancing**: Distribute traffic across servers
- **CDN**: Cache static assets globally

---

## 20. Conformance

### 20.1 Conformance Levels

**Level 1: Basic**
- Digital twin with static 3D model
- Manual data updates
- Basic dashboards

**Level 2: Intermediate**
- Real-time sensor synchronization
- Production simulation
- Energy monitoring

**Level 3: Advanced**
- AI-powered optimization
- AR/VR training
- Predictive analytics

**Level 4: Expert**
- Fully autonomous digital twin
- Factory-as-a-Service
- Multi-site integration

### 20.2 Certification

WIA offers certification for:
- **Products**: Digital twin platforms, simulation tools
- **Services**: Implementation consultants, integrators
- **Personnel**: Digital factory engineers, data scientists

**Certification Process:**
1. Application submission
2. Technical review
3. Conformance testing
4. Audit (for services)
5. Certification issuance
6. Annual renewal

### 20.3 Testing

Conformance testing includes:
- API compliance tests
- Performance benchmarks
- Security audits
- Interoperability tests

---

## Appendix A: Use Case Examples

### A.1 Automotive Assembly

A major automotive manufacturer implements WIA-IND-028 for its assembly plant:

- **Digital Twin**: Real-time model of 4 assembly lines with 500+ robots
- **Virtual Commissioning**: New model changeover tested virtually, reducing physical commissioning from 6 weeks to 2 weeks
- **Energy Management**: AI optimization reduces energy costs by $1.2M annually
- **Safety Monitoring**: AI vision detects PPE violations, 45% reduction in incidents
- **Results**: OEE increased from 78% to 92%, annual savings of $8M

### A.2 Electronics Manufacturing

An electronics manufacturer uses digital factory for PCB assembly:

- **Production Simulation**: Optimized line balancing, 30% throughput increase
- **Layout Optimization**: Reduced material handling distance by 40%
- **AR Maintenance**: Technicians use AR glasses for guided repair, 50% reduction in downtime
- **Results**: 25% productivity increase, 60% faster new product introduction

---

## Appendix B: Glossary

- **Digital Twin**: Virtual representation of physical factory
- **OEE**: Overall Equipment Effectiveness
- **VR**: Virtual Reality
- **AR**: Augmented Reality
- **DES**: Discrete Event Simulation
- **ABS**: Agent-Based Simulation
- **FaaS**: Factory-as-a-Service
- **KPI**: Key Performance Indicator
- **IoT**: Internet of Things
- **PLC**: Programmable Logic Controller
- **SCADA**: Supervisory Control and Data Acquisition
- **MES**: Manufacturing Execution System
- **ERP**: Enterprise Resource Planning

---

## Appendix C: References

1. ISO 23247 series - Digital twin framework for manufacturing
2. Grieves, M. (2014). Digital Twin: Manufacturing Excellence through Virtual Factory Replication
4. Industry 4.0 Reference Architecture Model (RAMI 4.0)
5. NIST Cybersecurity Framework
6. IEC 62443 - Industrial Cybersecurity

---

**Document Version:** 1.0.0
**Release Date:** 2025-12-27
**Next Review:** 2026-12-27

**Contact:**
WIA Industry Research Group
Email: industry@wiastandards.org
Web: https://wiastandards.com

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*© 2025 SmileStory Inc. / WIA*
*World Certification Industry Association*
*MIT License*


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

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
evidence for PHASE-4-INTEGRATION. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-4-integration/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-4-INTEGRATION with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-4-INTEGRATION does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-4-INTEGRATION.
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
for PHASE-4-INTEGRATION. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P4-INTEGRATION-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-4-INTEGRATION validation when the
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
