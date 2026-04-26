# WIA-EDU-008 Digital Textbook Standard v1.2

## Phase 3: Protocol & Synchronization

**Status:** ✅ Complete
**Version:** 1.2.0
**Date:** 2025-03-15
**Philosophy:** 弘益人間 (Benefit All Humanity)
**Builds on:** v1.0 (Phase 1), v1.1 (Phase 2)

---

## 1. Overview

Phase 3 defines synchronization protocols for multi-device access, offline functionality, and conflict resolution. This ensures students can seamlessly access their textbooks across all devices, even without internet connectivity.

## 2. Synchronization Architecture

### 2.1 Sync Protocol Overview

The standard uses a hybrid approach:
- **Real-time:** WebSocket for immediate updates when online
- **Batch:** REST API for periodic sync and offline queue processing
- **Conflict Resolution:** Operational Transformation (OT) for annotations

### 2.2 Synchronization Scope

What gets synchronized:
1. Reading position (last page/location)
2. Annotations (highlights, notes, bookmarks)
3. User preferences (font size, theme, etc.)
4. Assessment responses
5. Analytics data

## 3. WebSocket Protocol

### 3.1 Connection Establishment

```javascript
const ws = new WebSocket('wss://sync.provider.com/v1/sync');

ws.onopen = function() {
  // Authenticate
  ws.send(JSON.stringify({
    type: 'auth',
    token: 'Bearer eyJhbGc...'
  }));
};
```

### 3.2 Sync Message Format

```json
{
  "type": "sync.annotation.create",
  "messageId": "msg-123456",
  "timestamp": "2025-12-25T10:30:15.234Z",
  "deviceId": "device-abc123",
  "vectorClock": {
    "device-abc123": 15,
    "device-def456": 12
  },
  "data": {
    "annotationId": "anno-789",
    "textbookId": "978-3-16-148410-0",
    "type": "highlight",
    "selector": {
      "start": 1234,
      "end": 1456
    },
    "color": "#FFFF00"
  }
}
```

### 3.3 Server Acknowledgment

```json
{
  "type": "sync.ack",
  "messageId": "msg-123456",
  "status": "success",
  "serverTime": "2025-12-25T10:30:15.567Z"
}
```

## 4. Conflict Resolution

### 4.1 Vector Clocks

Each device maintains a vector clock to track causal ordering:

```json
{
  "vectorClock": {
    "device-abc123": 15,  // This device's counter
    "device-def456": 12,  // Last seen counter from other device
    "device-ghi789": 8    // Last seen counter from third device
  }
}
```

### 4.2 Operational Transformation for Annotations

When two devices create overlapping highlights:

```javascript
// Device A creates highlight
{
  "type": "highlight",
  "range": { "start": 100, "end": 150 },
  "color": "yellow",
  "vectorClock": { "A": 5, "B": 3 }
}

// Device B creates highlight (concurrently)
{
  "type": "highlight",
  "range": { "start": 120, "end": 180 },
  "color": "green",
  "vectorClock": { "A": 4, "B": 4 }
}

// Server resolves: Keep both highlights
// Result: Two separate, overlapping highlights
```

### 4.3 Conflict Resolution Strategies

| Data Type | Strategy | Rationale |
|-----------|----------|-----------|
| Reading Position | Last-write-wins | Most recent position is most relevant |
| Highlights | Merge (keep all) | User intent is to mark multiple passages |
| Notes | Merge (keep all) | All notes have value |
| Bookmarks | Union | All bookmarks should be preserved |
| Quiz Responses | First-write-wins | Prevent duplicate submissions |
| Preferences | Last-write-wins | Most recent preference is desired |

## 5. Offline Support

### 5.1 Local Storage Requirements

Implement local-first architecture using:
- **IndexedDB** (Web): For content and user data
- **SQLite** (Native): For mobile and desktop apps
- **Sync Queue**: Persist operations when offline

### 5.2 Offline Queue Structure

```json
{
  "queueId": "queue-user123",
  "operations": [
    {
      "id": "op-001",
      "type": "annotation.create",
      "timestamp": "2025-12-25T08:15:00Z",
      "attempts": 0,
      "maxAttempts": 5,
      "data": {...},
      "dependencies": []
    },
    {
      "id": "op-002",
      "type": "progress.update",
      "timestamp": "2025-12-25T08:30:00Z",
      "attempts": 0,
      "data": {...},
      "dependencies": ["op-001"]
    }
  ]
}
```

### 5.3 Sync Process When Connectivity Returns

1. Check server connectivity
2. Authenticate/refresh tokens
3. Fetch server-side changes since last sync
4. Apply operational transformation if conflicts exist
5. Push queued local operations
6. Verify all operations acknowledged
7. Update local state with server confirmations

## 6. Content Updates and Versioning

### 6.1 Version Detection

```http
GET /textbooks/978-3-16-148410-0/version HTTP/1.1

Response:
{
  "currentVersion": "1.2.0",
  "localVersion": "1.1.0",
  "updateAvailable": true,
  "updateType": "minor",
  "releaseDate": "2025-12-20",
  "changelog": "Updated Chapter 5 with new research findings",
  "deltaSize": 2400000,
  "fullSize": 127834289
}
```

### 6.2 Delta Sync for Updates

```http
GET /textbooks/978-3-16-148410-0/delta?from=1.1.0&to=1.2.0 HTTP/1.1

Response: Binary diff (bsdiff format)
```

### 6.3 Annotation Mapping Across Versions

```json
{
  "annotationId": "anno-123",
  "originalVersion": "1.1.0",
  "newVersion": "1.2.0",
  "mapping": {
    "originalSelector": {
      "type": "TextPositionSelector",
      "start": 1234,
      "end": 1456
    },
    "mappedSelector": {
      "type": "TextPositionSelector",
      "start": 1289,
      "end": 1511
    },
    "confidence": 0.98,
    "method": "fuzzy-text-match"
  }
}
```

## 7. Bandwidth Optimization

### 7.1 Delta Sync Algorithms

Use binary diff algorithms:
- **bsdiff/bspatch**: For content files
- **JSON Patch (RFC 6902)**: For metadata and user data

### 7.2 Compression

All sync traffic MUST support:
- Gzip compression (minimum)
- Brotli compression (preferred)

Request header:
```
Accept-Encoding: br, gzip
```

### 7.3 Deduplication

Server MUST detect and eliminate redundant sync operations:
```json
{
  "operationId": "op-123",
  "hash": "sha256:abc123...",
  "deduplicated": true,
  "originalOperationId": "op-120"
}
```

## 8. Security

### 8.1 Transport Security

- **TLS 1.3** minimum for all connections
- Certificate pinning for mobile apps
- Perfect Forward Secrecy (PFS)

### 8.2 Local Data Encryption

All local data MUST be encrypted:
- **AES-256-GCM** for storage encryption
- Keys derived from user credentials via PBKDF2
- Per-device keys for multi-device scenarios

### 8.3 End-to-End Encryption (Optional)

For sensitive annotations:
```json
{
  "annotationId": "anno-456",
  "encrypted": true,
  "algorithm": "AES-256-GCM",
  "ciphertext": "base64-encoded-data",
  "iv": "initialization-vector",
  "keyId": "user-key-1"
}
```

## 9. Performance Requirements

### 9.1 Sync Latency

- Real-time sync: < 500ms (WebSocket)
- Batch sync: < 2s for typical operations
- Offline queue processing: < 5s for 100 operations

### 9.2 Bandwidth Usage

Typical sync scenarios:

| Operation | Full Sync | Delta Sync | Savings |
|-----------|-----------|------------|---------|
| 20 annotations | 125 KB | 8 KB | 94% |
| Reading progress | 125 KB | 0.5 KB | 99.6% |
| Content update (minor) | 127 MB | 2.3 MB | 98.2% |
| Preferences | 125 KB | 0.3 KB | 99.8% |

## 10. Testing and Validation

### 10.1 Sync Testing Scenarios

Test cases MUST include:
1. Concurrent edits from 2+ devices
2. Offline operation for 24+ hours
3. Network interruption during sync
4. Large annotation sets (1000+ items)
5. Content version updates
6. Conflict resolution accuracy

### 10.2 Performance Testing

- Sync 1000 annotations in < 10 seconds
- Handle 100 concurrent users per server
- Maintain < 100ms WebSocket ping latency

---

**Philosophy:** 弘益人間 · Benefit All Humanity

© 2025 WIA - World Certification Industry Association


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
