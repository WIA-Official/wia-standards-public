# WIA-SOC-013 PHASE 3: PROTOCOL SPECIFICATION

**Public Document Standard - Security Protocols**

Version: 1.0
Date: 2025-01-15
Status: Final

---

## 1. Digital Signature Protocol

### 1.1 Supported Algorithms

- **ECDSA**: secp256r1 (P-256), secp384r1 (P-384)
- **RSA**: 2048, 3072, 4096 bits
- **EdDSA**: Ed25519, Ed448
- **Post-Quantum**: CRYSTALS-Dilithium (future-ready)

### 1.2 Signature Process

1. Calculate SHA-256 hash of document
2. Sign hash with private key
3. Embed signature in PDF using PAdES-LTV
4. Include certificate chain
5. Add RFC 3161 timestamp
6. Anchor hash to blockchain

## 2. PKI Infrastructure

### 2.1 Certificate Hierarchy

```
Root CA (20-year validity)
  ├── Intermediate CA (10-year)
  │   ├── Issuing CA (5-year)
  │   │   └── End-Entity Cert (2-year)
```

### 2.2 Certificate Validation

1. Verify signature chain to trusted root
2. Check certificate expiration
3. Query OCSP for revocation status
4. Validate certificate policies
5. Check key usage extensions

## 3. Encryption

### 3.1 At Rest

- Algorithm: AES-256-GCM
- Key Management: AWS KMS / Azure Key Vault / HSM
- Key Rotation: Annual

### 3.2 In Transit

- Protocol: TLS 1.3
- Cipher Suites: TLS_AES_256_GCM_SHA384
- Certificate Pinning: Recommended for mobile apps

## 4. Blockchain Anchoring

### 4.1 Supported Networks

- Ethereum Mainnet
- Polygon (Layer 2)
- Solana
- Hyperledger Fabric (permissioned)

### 4.2 Anchoring Process

```solidity
contract DocumentRegistry {
    mapping(bytes32 => DocumentAnchor) public anchors;
    
    struct DocumentAnchor {
        bytes32 documentHash;
        address issuer;
        uint256 timestamp;
        string documentId;
    }
    
    function anchor(bytes32 hash, string memory docId) public {
        anchors[hash] = DocumentAnchor(hash, msg.sender, block.timestamp, docId);
    }
}
```

## 5. Zero-Knowledge Proofs

### 5.1 Selective Disclosure

Prove attributes without revealing full document:

- Prove "age > 18" without revealing date of birth
- Prove "citizen of country X" without revealing full ID
- Prove "valid license" without revealing license number

### 5.2 zk-SNARK Implementation

```
Circuit:
  public input: attribute_commitment
  private input: full_document, salt
  
  verify:
    hash(full_document, salt) == attribute_commitment
    attribute(full_document) satisfies condition
```

## 6. Revocation

### 6.1 CRL (Certificate Revocation List)

- Published every 24 hours
- Signed by issuing CA
- Available via HTTP and LDAP

### 6.2 OCSP (Online Certificate Status Protocol)

- Real-time status queries
- Response signed by OCSP responder
- Must-staple for critical documents

## 7. Audit Trail

Every operation logged with:
- Timestamp (RFC 3339)
- Actor (DID or username)
- Action (create, read, sign, verify, revoke)
- Document ID
- IP address
- Result (success/failure)

---

© 2025 SmileStory Inc. / WIA

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
