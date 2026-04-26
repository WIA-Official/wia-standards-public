# WIA-AUTO-008 PHASE 4 — Integration Specification

**Standard:** WIA-AUTO-008
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

"lastSeen": "2025-12-26T10:30:00Z",
  "location": { "latitude": 37.7749, "longitude": -122.4194 },
  "status": {
    "ignition": true,
    "locked": false,
    "fuelLevel": 62.3,
    "batteryVoltage": 12.6,
    "odometer": 45230.5
  },
  "health": {
    "overallScore": 92,
    "issues": []
  }
}
```

**Remote Diagnostics**

```http
POST /v1/vehicles/{vin}/diagnostics/run
Content-Type: application/json

{
  "level": "comprehensive",
  "systems": ["engine", "battery", "transmission"]
}

Response: 202 Accepted
{
  "jobId": "diag-789012",
  "status": "running",
  "estimatedCompletionTime": "2025-12-26T10:35:00Z"
}

GET /v1/vehicles/{vin}/diagnostics/jobs/{jobId}

Response: 200 OK
{
  "jobId": "diag-789012",
  "status": "completed",
  "results": {
    "healthScore": 92,
    "issues": [],
    "recommendations": ["Schedule oil change in 500 km"]
  }
}
```

**OTA Updates**

```http
GET /v1/vehicles/{vin}/ota/updates

Response: 200 OK
{
  "updates": [
    {
      "packageId": "PKG-2025-001",
      "version": "2.5.0",
      "component": "infotainment",
      "size": 524288000,
      "releaseDate": "2025-01-15T00:00:00Z",
      "criticality": "recommended",
      "releaseNotes": "Bug fixes and performance improvements"
    }
  ]
}

POST /v1/vehicles/{vin}/ota/install
Content-Type: application/json

{
  "packageId": "PKG-2025-001",
  "scheduleAt": "2025-12-27T02:00:00Z"
}

Response: 202 Accepted
{
  "jobId": "ota-345678",
  "status": "scheduled",
  "scheduledTime": "2025-12-27T02:00:00Z"
}
```

### 9.2 GraphQL API

**Schema**:
```graphql
type Query {
  vehicle(vin: String!): Vehicle
  vehicles(filter: VehicleFilter): [Vehicle!]!
}

type Mutation {
  updateTelemetry(vin: String!, data: TelemetryInput!): TelemetryResponse
  runDiagnostics(vin: String!, level: DiagnosticLevel!): DiagnosticJob
  installOTAUpdate(vin: String!, packageId: String!): OTAJob
}

type Subscription {
  telemetryStream(vin: String!): Telemetry
  diagnosticUpdates(vin: String!): DiagnosticUpdate
}

type Vehicle {
  vin: String!
  model: String!
  year: Int!
  status: VehicleStatus!
  location: Location!
  health: VehicleHealth!
  telemetry(since: DateTime): [Telemetry!]!
}
```

**Example Query**:
```graphql
query GetVehicleStatus($vin: String!) {
  vehicle(vin: $vin) {
    vin
    status {
      ignition
      locked
      fuelLevel
      batteryVoltage
    }
    location {
      latitude
      longitude
      speed
    }
    health {
      overallScore
      issues {
        code
        severity
        description
      }
    }
  }
}
```

---

## 10. Security Protocols

### 10.1 Authentication and Authorization

#### 10.1.1 Vehicle Authentication

**Certificate-Based (PKI)**:
```
1. Manufacturing: Provision unique certificate per vehicle
2. Storage: Store in HSM/TPM
3. Connection: Mutual TLS (mTLS) authentication
4. Validation: Cloud verifies certificate chain
5. Renewal: Automatic before expiration
```

**Token-Based (OAuth 2.0)**:
```
1. Vehicle → Auth Server: Client credentials grant
2. Auth Server → Vehicle: JWT access token
3. Vehicle → API: Bearer token in header
4. Token Refresh: Before expiration
```

#### 10.1.2 User Authentication

**Multi-Factor Authentication (MFA)**:
```
Factor 1: Password or PIN
Factor 2: SMS/TOTP code or biometric
Factor 3 (optional): Physical key or trusted device
```

**Single Sign-On (SSO)**:
- SAML 2.0
- OpenID Connect (OIDC)
- OAuth 2.0

### 10.2 Encryption

#### 10.2.1 Transport Layer Security

**TLS 1.3 Configuration**:
```
Cipher Suites (Recommended):
  - TLS_AES_256_GCM_SHA384
  - TLS_CHACHA20_POLY1305_SHA256
  - TLS_AES_128_GCM_SHA256

Perfect Forward Secrecy: Required
Session Resumption: Allowed (with restrictions)
Renegotiation: Disabled
Compression: Disabled (CRIME vulnerability)
```

#### 10.2.2 Data Encryption

**At Rest**:
- **Algorithm**: AES-256-GCM
- **Key Management**: AWS KMS, Azure Key Vault, or HSM
- **Key Rotation**: Annual or on compromise

**In Transit**:
- **Protocol**: TLS 1.3
- **Mutual TLS**: For vehicle-cloud communication
- **Certificate Pinning**: Prevent MITM attacks

### 10.3 Intrusion Detection and Prevention

#### 10.3.1 Vehicle-Side IDS

**Monitored Events**:
- Unauthorized access attempts
- Abnormal CAN bus traffic
- Unexpected firmware modifications
- Anomalous sensor readings
- Repeated authentication failures

**Response Actions**:
- Log event
- Alert cloud platform
- Rate limit connections
- Temporary lockdown
- Notify user

#### 10.3.2 Cloud-Side IDS

**Monitored Events**:
- Unusual API request patterns
- Geographic anomalies
- Impossible travel scenarios
- Brute force attempts
- Data exfiltration patterns

**Response Actions**:
- Block IP address
- Require re-authentication
- Escalate to security team
- Notify vehicle owner

### 10.4 Compliance Standards

#### 10.4.1 ISO 27001 (Information Security)

- Risk assessment and management
- Security policies and procedures
- Access control
- Incident management
- Business continuity

#### 10.4.2 UNECE WP.29 (Cybersecurity)

**Requirements**:
- Risk assessment (TARA - Threat Analysis and Risk Assessment)
- Security by design
- Secure software updates
- Cybersecurity monitoring
- Incident response

#### 10.4.3 SAE J3061 (Cybersecurity Guidebook)

**Lifecycle Phases**:
1. Concept
2. Product Development
3. Production
4. Operations and Maintenance
5. Decommissioning

---

## 11. References

### 11.1 Standards and Specifications

- **ISO 26262**: Functional Safety for Road Vehicles
- **ISO/SAE 21434**: Cybersecurity Engineering
- **UNECE WP.29**: Cybersecurity and Software Update Regulations
- **SAE J3061**: Cybersecurity Guidebook for Cyber-Physical Systems
- **SAE J1979**: E/E Diagnostic Test Modes (OBD-II)
- **ISO 14229**: Unified Diagnostic Services (UDS)
- **ISO 15765**: Diagnostic Communication over CAN
- **MQTT 3.1.1 / 5.0**: MQTT Protocol Specifications
- **IEEE 802.11**: WiFi Standards
- **3GPP TS 23.285**: V2X Services

### 11.2 Connectivity Standards

- **4G LTE**: 3GPP Release 8-15
- **5G NR**: 3GPP Release 15+
- **C-V2X**: 3GPP Release 14-16
- **DSRC**: IEEE 802.11p, SAE J2735
- **Bluetooth**: Bluetooth Core Specification 5.0+

### 11.3 Security Standards

- **TLS 1.3**: RFC 8446
- **OAuth 2.0**: RFC 6749
- **JWT**: RFC 7519
- **X.509**: Public Key Infrastructure
- **NIST Cybersecurity Framework**: CSF v1.1

### 11.4 Privacy Regulations

- **GDPR**: EU General Data Protection Regulation
- **CCPA**: California Consumer Privacy Act
- **LGPD**: Brazilian General Data Protection Law
- **PIPEDA**: Canadian Personal Information Protection

### 11.5 WIA Standards

- **WIA-INTENT**: Intent-based interfaces
- **WIA-OMNI-API**: Universal API gateway
- **WIA-SOCIAL**: Social coordination protocols
- **WIA-QUANTUM**: Quantum-safe cryptography

---

## Appendix A: Example Implementations

### A.1 Telemetry Collection

```typescript
import { ConnectedCarSDK } from '@wia/auto-008';

const sdk = new ConnectedCarSDK({
  vehicleId: '1HGBH41JXMN109186',
  apiKey: process.env.API_KEY
});

// Start continuous telemetry collection
sdk.startTelemetryCollection({
  interval: 5000, // 5 seconds
  includeLocation: true,
  includeDiagnostics: true,
  onData: async (data) => {
    console.log('Telemetry:', data);

    // Check for critical warnings
    if (data.diagnostics?.dtcs?.some(dtc => dtc.severity === 'critical')) {
      await sdk.sendAlert({
        type: 'critical_dtc',
        message: 'Critical diagnostic issue detected',
        dtcs: data.diagnostics.dtcs
      });
    }
  },
  onError: (error) => {
    console.error('Telemetry error:', error);
  }
});
```

### A.2 OTA Update Installation

```bash
#!/bin/bash
# OTA Update Script

VIN="1HGBH41JXMN109186"

# Check for updates
echo "Checking for OTA updates..."
UPDATES=$(wia-auto-008 ota-check --vin $VIN --format json)

if [ $(echo $UPDATES | jq '.updates | length') -gt 0 ]; then
  PACKAGE_ID=$(echo $UPDATES | jq -r '.updates[0].packageId')

  echo "Update available: $PACKAGE_ID"
  echo "Downloading and installing..."

  wia-auto-008 ota-install \
    --vin $VIN \
    --package $PACKAGE_ID \
    --auto-rollback

  echo "Update completed successfully"
else
  echo "No updates available"
fi
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUTO-008 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
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
