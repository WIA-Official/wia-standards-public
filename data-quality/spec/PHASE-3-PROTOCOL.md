# WIA-DATA-005: Data Quality - Phase 3 Protocol Specification

**Version:** 1.0
**Status:** Draft
**Last Updated:** 2025-12-26

---

## Overview

This document defines the communication protocols for the WIA-DATA-005 Data Quality standard, including real-time streaming, event-driven architecture, and inter-service communication patterns.

## Protocol Stack

### Layer 1: Transport
- HTTP/2 for RESTful APIs
- WebSocket for real-time streams
- gRPC for high-performance RPC
- MQTT for IoT data quality monitoring

### Layer 2: Serialization
- JSON for human-readable data
- Protocol Buffers for binary efficiency
- Avro for schema evolution
- MessagePack for compact representation

### Layer 3: Application
- Data Quality Streaming Protocol (DQSP)
- Quality Event Protocol (QEP)
- Validation Request Protocol (VRP)

## Data Quality Streaming Protocol (DQSP)

### Connection Establishment

WebSocket handshake:

```http
GET /wia/dq/stream HTTP/1.1
Host: api.example.com
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==
Sec-WebSocket-Protocol: dqsp-v1
Sec-WebSocket-Version: 13
```

Response:

```http
HTTP/1.1 101 Switching Protocols
Upgrade: websocket
Connection: Upgrade
Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=
Sec-WebSocket-Protocol: dqsp-v1
```

### Message Format

```json
{
  "messageType": "quality.metric|validation.result|issue.alert",
  "messageId": "uuid",
  "timestamp": "ISO8601",
  "payload": {}
}
```

### Subscription

Client subscribes to quality metrics stream:

```json
{
  "action": "subscribe",
  "channels": [
    "dataset.customers.metrics",
    "dataset.orders.validations"
  ],
  "filters": {
    "dimensions": ["accuracy", "completeness"],
    "severity": ["high", "critical"]
  }
}
```

Server acknowledgment:

```json
{
  "messageType": "subscription.confirmed",
  "subscriptionId": "uuid",
  "channels": ["dataset.customers.metrics"]
}
```

### Streaming Metrics

```json
{
  "messageType": "quality.metric",
  "messageId": "uuid",
  "timestamp": "2025-12-26T10:00:00Z",
  "payload": {
    "datasetId": "customers",
    "dimension": "completeness",
    "value": 95.5,
    "delta": -0.5,
    "trend": "declining"
  }
}
```

### Heartbeat

Client and server exchange heartbeats every 30 seconds:

```json
{
  "messageType": "heartbeat",
  "timestamp": "ISO8601"
}
```

## Quality Event Protocol (QEP)

### Event Categories

1. **Validation Events**
   - validation.started
   - validation.completed
   - validation.failed
   - validation.error

2. **Issue Events**
   - issue.created
   - issue.assigned
   - issue.resolved
   - issue.escalated

3. **Threshold Events**
   - threshold.warning
   - threshold.critical
   - threshold.recovered

4. **System Events**
   - profile.generated
   - cleansing.completed
   - rule.created

### Event Message Structure

```json
{
  "eventId": "uuid",
  "eventType": "validation.failed",
  "timestamp": "ISO8601",
  "source": {
    "service": "data-quality-validator",
    "version": "1.0.0",
    "instance": "validator-01"
  },
  "data": {
    "validationId": "uuid",
    "ruleId": "uuid",
    "datasetId": "customers",
    "failedRecords": 150,
    "passRate": 85.0
  },
  "metadata": {
    "correlationId": "uuid",
    "causationId": "uuid"
  }
}
```

### Event Delivery Guarantees

- **At-least-once delivery**: Events may be delivered multiple times
- **Ordering**: Events within same partition maintain order
- **Retention**: Events retained for 7 days by default

## Validation Request Protocol (VRP)

### Request-Response Pattern

Request:

```json
{
  "requestId": "uuid",
  "requestType": "validate",
  "timestamp": "ISO8601",
  "payload": {
    "datasetId": "customers",
    "ruleIds": ["uuid-1", "uuid-2"],
    "options": {
      "async": true,
      "sampleSize": 1000
    }
  }
}
```

Response:

```json
{
  "requestId": "uuid",
  "responseType": "validation.accepted",
  "timestamp": "ISO8601",
  "payload": {
    "validationId": "uuid",
    "status": "queued",
    "estimatedCompletion": "ISO8601"
  }
}
```

### Batch Validation Protocol

Submit multiple validation requests:

```json
{
  "batchId": "uuid",
  "requests": [
    {
      "datasetId": "customers",
      "ruleIds": ["uuid-1"]
    },
    {
      "datasetId": "orders",
      "ruleIds": ["uuid-2"]
    }
  ]
}
```

## gRPC Service Definitions

### Quality Service

```protobuf
syntax = "proto3";

package wia.dataquality.v1;

service QualityService {
  rpc ProfileDataset(ProfileRequest) returns (ProfileResponse);
  rpc ValidateDataset(ValidationRequest) returns (ValidationResponse);
  rpc StreamMetrics(StreamRequest) returns (stream Metric);
  rpc SubmitIssue(IssueRequest) returns (IssueResponse);
}

message ProfileRequest {
  string dataset_id = 1;
  int32 sample_size = 2;
  repeated string columns = 3;
}

message ProfileResponse {
  string profile_id = 1;
  DataQualityProfile profile = 2;
}

message ValidationRequest {
  string dataset_id = 1;
  repeated string rule_ids = 2;
  ValidationOptions options = 3;
}

message ValidationResponse {
  string validation_id = 1;
  string status = 2;
  repeated ValidationResult results = 3;
}
```

## Message Queue Integration

### Apache Kafka Topics

```
wia.dataquality.metrics           # Quality metrics stream
wia.dataquality.validations       # Validation results
wia.dataquality.issues            # Quality issues
wia.dataquality.events            # General events
```

### Message Format

Kafka message with Avro schema:

```json
{
  "key": "customers",
  "value": {
    "metricName": "completeness",
    "value": 95.5,
    "timestamp": 1640534400000
  },
  "headers": {
    "schema-version": "1.0",
    "content-type": "application/avro"
  }
}
```

## MQTT for IoT

### Topic Structure

```
wia/dq/{device-id}/metrics/{metric-name}
wia/dq/{device-id}/status
wia/dq/{device-id}/alerts
```

### QoS Levels

- QoS 0: At most once (metrics)
- QoS 1: At least once (validations)
- QoS 2: Exactly once (critical alerts)

## Security

### TLS/SSL
- Minimum TLS 1.2
- Strong cipher suites only
- Certificate-based authentication for services

### Message Encryption
- Payload encryption for sensitive data
- End-to-end encryption for compliance requirements

### Authentication
- OAuth 2.0 for API access
- JWT tokens for service-to-service
- mTLS for critical services

## Error Handling

### Protocol-Level Errors

```json
{
  "errorCode": "PROTOCOL_ERROR",
  "errorMessage": "Invalid message format",
  "details": {
    "field": "timestamp",
    "violation": "must be ISO8601 format"
  }
}
```

### Retry Strategy

- Exponential backoff: 1s, 2s, 4s, 8s, 16s
- Maximum retries: 5
- Circuit breaker after 3 consecutive failures

## Performance Specifications

### Latency Targets
- API response: < 100ms (p95)
- WebSocket message delivery: < 50ms (p95)
- gRPC call: < 20ms (p95)

### Throughput
- REST API: 10,000 requests/sec
- WebSocket: 100,000 messages/sec
- Kafka: 1,000,000 messages/sec

### Connection Limits
- WebSocket: 10,000 concurrent connections per instance
- HTTP/2: 1,000 concurrent streams per connection

---

**Copyright © 2025 SmileStory Inc. / WIA**
**弘益人間 (홍익인간) · Benefit All Humanity**


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
