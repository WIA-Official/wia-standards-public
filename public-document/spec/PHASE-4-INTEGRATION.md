# WIA-SOC-013 PHASE 4: INTEGRATION SPECIFICATION

**Public Document Standard - Cross-Border and System Integration**

Version: 1.0
Date: 2025-01-15
Status: Final

---

## 1. Cross-Border Recognition

### 1.1 Mutual Recognition Framework

Countries establish bilateral/multilateral agreements:

```json
{
  "agreement": "US-EU-Mutual-Recognition-2025",
  "parties": ["US", "EU-27"],
  "documentTypes": ["passport", "birthCertificate", "educationalDiploma"],
  "effectiveDate": "2025-07-01",
  "trustAnchors": {
    "US": "did:gov:us:federal-pki-root",
    "EU": "did:gov:eu:eidas-root"
  }
}
```

### 1.2 International Standards Compliance

- **eIDAS** (EU): Electronic identification and trust services
- **ICAO 9303**: Machine-readable travel documents
- **W3C VC**: Verifiable credentials data model
- **ISO 15489**: Records management

## 2. Multilingual Support

### 2.1 Language Codes

Use ISO 639-1 (2-letter) or ISO 639-2 (3-letter):
- English: `en`
- Korean: `ko`
- Chinese: `zh`
- Arabic: `ar`
- Spanish: `es`

### 2.2 Text Normalization

- **Unicode**: NFC (Canonical Composition) normalization
- **Transliteration**: UN GEOGN romanization standards
- **Right-to-Left**: Support for Arabic, Hebrew scripts

## 3. Legacy System Integration

### 3.1 Document Management Systems

Integration with:
- SharePoint (Microsoft 365)
- Documentum (OpenText)
- Alfresco
- IBM FileNet

### 3.2 API Adapters

```typescript
import { LegacyAdapter } from 'wia-soc-013';

const adapter = new LegacyAdapter({
  system: 'sharepoint-2016',
  apiUrl: 'https://legacy.gov/api',
  credentials: {...}
});

await adapter.syncDocuments({
  filter: { type: 'birthCertificate', year: 2025 },
  direction: 'bidirectional'
});
```

## 4. Mobile Integration

### 4.1 Mobile SDKs

- **iOS**: Swift package via CocoaPods/SPM
- **Android**: Kotlin library via Maven
- **React Native**: npm package
- **Flutter**: Dart package

### 4.2 Offline Support

- Local document caching (encrypted)
- Queue operations for sync when online
- Conflict resolution strategies

## 5. Blockchain Integration

### 5.1 Smart Contract ABI

```javascript
const abi = [
  "function anchor(bytes32 hash, string memory docId) public",
  "function verify(bytes32 hash) public view returns (DocumentAnchor memory)",
  "event DocumentAnchored(bytes32 indexed hash, address issuer, uint256 timestamp)"
];
```

### 5.2 Event Listening

```javascript
contract.on("DocumentAnchored", (hash, issuer, timestamp) => {
  console.log(`Document ${hash} anchored by ${issuer} at ${timestamp}`);
});
```

## 6. Identity Integration

### 6.1 Decentralized Identifiers (DIDs)

Government DIDs follow pattern:
```
did:gov:{country}:{agency}:{identifier}
```

Example:
```
did:gov:us:state-department:passport-authority
```

### 6.2 Verifiable Credentials

Documents issued as W3C Verifiable Credentials:

```json
{
  "@context": ["https://www.w3.org/2018/credentials/v1"],
  "type": ["VerifiableCredential", "BirthCertificate"],
  "issuer": "did:gov:us:california:registry",
  "issuanceDate": "2025-01-15T10:30:00Z",
  "credentialSubject": {
    "id": "did:example:citizen123",
    "birthDate": "1990-05-20",
    "birthPlace": "San Francisco, CA, USA"
  },
  "proof": {
    "type": "Ed25519Signature2020",
    "created": "2025-01-15T10:30:00Z",
    "proofPurpose": "assertionMethod",
    "verificationMethod": "did:gov:us:california:registry#key-1",
    "proofValue": "z..."
  }
}
```

## 7. Compliance and Reporting

### 7.1 GDPR Compliance

- Data portability: Export documents in JSON/XML
- Right to erasure: Delete digitized copies (blockchain anchors remain)
- Consent management: Explicit opt-in for document sharing
- Data minimization: Collect only necessary information

### 7.2 Audit Reports

Generate compliance reports:
- Monthly access logs
- Quarterly security audits
- Annual archival verification
- Ad-hoc incident reports

## 8. Disaster Recovery

### 8.1 Backup Strategy

- **Primary**: On-premises storage
- **Secondary**: Cloud replication (multi-region)
- **Tertiary**: Offline tape backup

### 8.2 Recovery Objectives

| Priority | RTO | RPO |
|----------|-----|-----|
| Critical | 1 hour | 0 (sync replication) |
| High | 4 hours | 1 hour |
| Medium | 24 hours | 24 hours |
| Low | 7 days | 7 days |

## 9. Monitoring and Observability

### 9.1 Metrics

- API latency (p50, p95, p99)
- Error rates
- Document processing throughput
- Storage utilization
- Blockchain confirmation times

### 9.2 Alerting

- PagerDuty for critical alerts
- Email for warnings
- Slack for informational notices

## 10. Future Enhancements

- AI-powered document classification
- Quantum-resistant cryptography
- Enhanced zero-knowledge proofs
- IoT device integration (biometric scanners)
- Satellite-based document delivery for remote areas

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity

---

## Annex A — Conformance Tier Matrix

WIA conformance for public-document is evaluated across three tiers:

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

- `wia-standards/standards/public-document/api/` — TypeScript SDK skeleton
- `wia-standards/standards/public-document/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/public-document/simulator/` — interactive browser-based simulator for the PHASE protocol

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



---

## Annex F — Operations and lifecycle notes

This informative annex captures operational guidance that has emerged from reference implementations and is expected to migrate into the normative body in a future minor revision.

### F.1 Deprecation policy

When a member of a canonical schema is deprecated, the schema MUST continue to accept and emit the member for at least 12 months from the publication of the deprecation notice. During the deprecation window, the implementation SHOULD emit a `Deprecation` HTTP header per the IETF deprecation-header draft for any response that contains the deprecated member, and SHOULD provide a `Sunset` header indicating the planned removal date per IETF RFC 8594.

### F.2 Backwards-compatible extensions

Vendors who extend a canonical schema with their own members MUST namespace those members with a reverse-DNS prefix, MUST treat the extension as opt-in, and MUST NOT shadow any normative member name reserved for future minor revisions of the standard.

### F.3 Operational telemetry

Every conformant deployment SHOULD expose a small set of operational metrics aligned with the OpenTelemetry semantic conventions. The recommended metric names are `wia.<slug>.requests.duration`, `wia.<slug>.requests.errors`, and `wia.<slug>.records.in_flight`.

### F.4 Logging

Logs MUST NOT contain unredacted authentication tokens, raw evidence pointers that the deployment does not own, or any plaintext personal data. Implementations SHOULD adopt structured JSON logging and SHOULD include the W3C Trace Context `trace_id` in every log line so that logs can be joined to the distributed-tracing graph.


---

## Annex G — Conformance attestation template

This informative annex offers a recommended template that conformant deployments may use when publishing their conformance attestation. The template is JSON Schema 2020-12 and is published under the WIA-Official catalogue.

```json
{
  "$id": "https://wiastandards.com/templates/conformance-attestation.json",
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "type": "object",
  "required": ["standard", "tier", "issued_at", "valid_until", "attesting_party", "evidence"],
  "properties": {
    "standard": { "type": "string", "format": "uri" },
    "tier": { "enum": ["tier-1", "tier-2", "tier-3"] },
    "issued_at": { "type": "string", "format": "date-time" },
    "valid_until": { "type": "string", "format": "date-time" },
    "attesting_party": { "type": "string", "format": "uri" },
    "evidence": { "type": "array", "minItems": 1, "items": { "type": "string", "format": "uri" } },
    "remarks": { "type": "string" }
  }
}
```

The template intentionally omits any sector-specific fields. Sector profiles SHOULD extend the template via JSON Schema composition (`allOf`) rather than redefinition.
