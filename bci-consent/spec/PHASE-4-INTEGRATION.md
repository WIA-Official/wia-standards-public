# WIA BCI Consent Protocol
## Phase 4: System Integration Specification

**Version:** 1.0.0  
**Status:** Release  
**Last Updated:** 2025-12-25

---

## 1. Introduction

This specification defines how to integrate the consent protocol with BCI devices, healthcare systems, and regulatory frameworks.

## 2. BCI Device Integration

### 2.1 Pre-Collection Consent Check

```pseudo
Before collecting neural data:
  1. Retrieve current consent record
  2. Verify consent is active (not expired/revoked)
  3. Check required permissions granted
  4. Log verification
  5. If valid, proceed with collection
  6. If invalid, block operation and notify user
```

### 2.2 Embedded Consent Verification

```javascript
async function collectNeuralData() {
    const consentValid = await api.verifyConsent(
        user.consentId,
        ['dataCollection', 'realTimeProcessing']
    );
    
    if (!consentValid) {
        throw new ConsentError('Valid consent required');
    }
    
    return await device.readElectrodes();
}
```

## 3. Healthcare System Integration

### 3.1 EHR Integration

- **HL7 FHIR Consent Resource** mapping
- **Integration points**: Epic, Cerner, Allscripts
- **Data synchronization**: Real-time consent status updates
- **Clinical decision support**: Alerts for expired consent

### 3.2 FHIR Consent Resource Mapping

```json
{
  "resourceType": "Consent",
  "id": "consent-bci-001",
  "status": "active",
  "patient": {"reference": "Patient/123"},
  "dateTime": "2025-12-25T10:00:00Z",
  "provision": {
    "type": "permit",
    "period": {
      "start": "2025-12-25",
      "end": "2026-12-25"
    }
  }
}
```

## 4. Research Platform Integration

### 4.1 IRB Integration

- Automated consent document submission
- Version control linked to IRB approvals
- Participant enrollment tracking
- Adverse event reporting
- Protocol deviation documentation

### 4.2 Clinical Trial Management

- Participant screening and enrollment
- Consent version management
- Site management and monitoring
- Data collection workflow
- Regulatory submission preparation

## 5. Regulatory Compliance

### 5.1 Compliance Monitoring

- Real-time compliance dashboards
- Automated violation detection
- Regulatory change tracking
- Audit preparation
- Reporting and analytics

### 5.2 Audit Requirements

- Monthly internal audit
- Quarterly compliance review
- Annual external audit
- Post-incident review
- Continuous automated checking

## 6. Cloud Platform Integration

### 6.1 AWS

```python
# Lambda function for consent verification
def lambda_handler(event, context):
    consent_id = event['consentId']
    permissions = event['permissions']
    
    consent = get_consent_from_dynamodb(consent_id)
    valid = verify_permissions(consent, permissions)
    
    return {
        'statusCode': 200,
        'body': json.dumps({'valid': valid})
    }
```

### 6.2 Azure

```csharp
[FunctionName("VerifyConsent")]
public async Task<IActionResult> Run(
    [HttpTrigger] HttpRequest req,
    [CosmosDB] CosmosClient client)
{
    var consentId = req.Query["consentId"];
    var valid = await VerifyConsent(client, consentId);
    return new OkObjectResult(new { valid });
}
```

### 6.3 Google Cloud

```javascript
exports.consentWebhook = functions.https.onRequest(async (req, res) => {
    const event = req.body;
    await firestore.collection('consent_events').add(event);
    res.status(200).send('OK');
});
```

## 7. Mobile Integration

### 7.1 iOS

```swift
import WIABCIConsent

let client = ConsentClient(apiKey: "key")
let valid = try await client.verifyConsent(
    id: consent.id,
    permissions: [.dataCollection]
)
```

### 7.2 Android

```kotlin
val client = ConsentClient(apiKey)
val valid = client.verifyConsent(
    consentId,
    listOf("dataCollection")
)
```

## 8. Security Requirements

### 8.1 Encryption

- **In Transit**: TLS 1.3+
- **At Rest**: AES-256
- **Backups**: Encrypted storage
- **Keys**: Hardware Security Modules (HSM)

### 8.2 Access Control

- Role-Based Access Control (RBAC)
- Least privilege principle
- Multi-Factor Authentication (MFA)
- Audit logging for all access

## 9. Performance Requirements

| Operation | Target Latency | P95 Latency |
|-----------|---------------|-------------|
| Cached consent check | < 1ms | 0.8ms |
| Database consent check | < 10ms | 8ms |
| API verification | < 100ms | 85ms |
| Cross-region verification | < 200ms | 180ms |

## 10. Disaster Recovery

### 10.1 Backup Strategies

- Real-time replication to secondary region
- Hourly snapshots for point-in-time recovery
- Daily archives to immutable storage
- Offline backups for catastrophic events

### 10.2 Recovery Objectives

| Scenario | RTO | RPO |
|----------|-----|-----|
| Server failure | < 5 min | 0 |
| Data center outage | < 15 min | 0 |
| Regional disaster | < 1 hour | < 1 hour |
| Catastrophic event | < 24 hours | < 24 hours |

---

**弘益人間 (Hongik Ingan)** - *Benefit All Humanity*

© 2025 SmileStory Inc. / WIA

---

## Annex A — Conformance Tier Matrix

WIA conformance for bci-consent is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/bci-consent/api/` — TypeScript SDK skeleton
- `wia-standards/standards/bci-consent/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/bci-consent/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---


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
