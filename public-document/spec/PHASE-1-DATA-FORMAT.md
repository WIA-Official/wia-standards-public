# WIA-SOC-013 PHASE 1: DATA FORMAT SPECIFICATION

**Public Document Standard - Data Formats**

Version: 1.0
Date: 2025-01-15
Status: Final

---

## 1. Overview

This specification defines data formats for public document digitization, including image formats, OCR output, metadata schemas, and archival formats compliant with international standards.

## 2. Document Structure

### 2.1 JSON Document Schema

```json
{
  "documentId": "doc:wia:soc013:{type}:{country}:{year}:{uniqueId}",
  "version": "1.0",
  "type": "birthCertificate | passport | nationalID | driverLicense | diploma | deed",
  "issuer": "did:gov:{country}:{agency}",
  "issuedAt": "ISO 8601 timestamp",
  "expiresAt": "ISO 8601 timestamp (optional)",
  "subject": {
    "name": "string",
    "nameLocal": "string (native script)",
    "nameRomanized": "string",
    "dateOfBirth": "YYYY-MM-DD",
    "placeOfBirth": "string",
    "nationality": "ISO 3166-1 alpha-3"
  },
  "images": [{
    "pageNumber": "integer",
    "format": "image/jpeg | image/png | image/tiff",
    "resolution": "integer (DPI)",
    "url": "string",
    "sha256": "string"
  }],
  "metadata": {},
  "signatures": [],
  "blockchainAnchors": []
}
```

### 2.2 XML Document Schema

```xml
<document xmlns="https://wiastandards.org/soc-013/v1"
          documentId="doc:wia:soc013:birth:US:2025:abc123"
          type="birthCertificate"
          version="1.0">
  <issuer>did:gov:us:california:registry</issuer>
  <issuedAt>2025-01-15T10:30:00Z</issuedAt>
  <subject>
    <name>John Alexander Doe</name>
    <dateOfBirth>1990-05-20</dateOfBirth>
    <placeOfBirth>San Francisco, CA, USA</placeOfBirth>
  </subject>
  <images>
    <image pageNumber="1" format="image/jpeg" resolution="600" 
           url="..." sha256="..."/>
  </images>
</document>
```

## 3. Image Formats

### 3.1 Master Archive Format: TIFF

- **Compression**: LZW (lossless) or Uncompressed
- **Color Depth**: 8-bit grayscale or 24-bit RGB
- **Resolution**: 600-1200 DPI minimum
- **Photometric**: Min-is-White for grayscale
- **Byte Order**: Little Endian (Intel)

### 3.2 Distribution Format: PDF/A

- **Version**: PDF/A-2b or PDF/A-3b (ISO 19005-2/3)
- **Conformance**: Level B minimum, Level A for accessibility
- **Compression**: JPEG2000 for images (quality 85)
- **Fonts**: All fonts embedded
- **Color Space**: sRGB

### 3.3 Derivative Formats

- **JPEG**: 150-300 DPI, quality 85, for web delivery
- **PNG**: 300 DPI, lossless, for transparency support
- **WebP**: 150-300 DPI, for modern web browsers

## 4. OCR Output Format

### 4.1 hOCR (HTML-based OCR)

```xml
<span class="ocr_line" title="bbox 120 80 520 140; baseline 0.015 -12; x_size 48; x_descenders 12; x_ascenders 36">
  <span class="ocrx_word" title="bbox 120 80 280 140; x_wconf 98">United</span>
  <span class="ocrx_word" title="bbox 290 80 420 140; x_wconf 99">States</span>
  <span class="ocrx_word" title="bbox 430 80 520 140; x_wconf 97">Passport</span>
</span>
```

### 4.2 ALTO XML (Analyzed Layout and Text Object)

```xml
<TextBlock ID="TB1" HPOS="120" VPOS="80" WIDTH="400" HEIGHT="60">
  <TextLine ID="TL1" HPOS="120" VPOS="80" WIDTH="400" HEIGHT="60">
    <String CONTENT="United States Passport" WC="0.98"/>
  </TextLine>
</TextBlock>
```

## 5. Metadata Schemas

### 5.1 Dublin Core

Required elements: `dc:title`, `dc:creator`, `dc:date`, `dc:identifier`, `dc:format`, `dc:language`

Optional elements: `dc:subject`, `dc:description`, `dc:publisher`, `dc:rights`, `dc:coverage`

### 5.2 PREMIS

```json
{
  "premis": {
    "objectIdentifier": "doc:wia:soc013:abc123",
    "preservationLevel": "full",
    "significantProperties": ["authenticity", "integrity"],
    "environment": {
      "software": "WIA-SOC-013 Scanner v1.0",
      "hardware": "Fujitsu fi-7600 Scanner"
    },
    "events": [],
    "agents": [],
    "rights": {}
  }
}
```

## 6. Cryptographic Formats

### 6.1 Digital Signature

```json
{
  "signature": {
    "algorithm": "ECDSA-SHA256 | RSA-SHA256 | EdDSA-Ed25519",
    "value": "base64-encoded signature",
    "publicKey": "PEM-encoded public key",
    "certificateChain": ["base64-cert-1", "base64-cert-2", "base64-root"],
    "timestamp": "RFC 3161 timestamp token"
  }
}
```

### 6.2 Blockchain Anchor

```json
{
  "blockchainAnchor": {
    "network": "ethereum-mainnet | polygon | solana",
    "transactionHash": "0x...",
    "blockNumber": 12345678,
    "timestamp": "2025-01-15T10:30:00Z",
    "documentHash": "sha256 hash",
    "smartContract": "0x..."
  }
}
```

## 7. Quality Metrics

| Metric | Standard | Measurement |
|--------|----------|-------------|
| Image Resolution | ≥ 300 DPI | EXIF metadata |
| OCR Confidence | ≥ 95% | Per-word confidence scores |
| Metadata Completeness | 100% required fields | Field validation |
| Color Accuracy | ΔE < 3 | CIE Lab color difference |

## 8. Validation Rules

1. Document ID must follow pattern: `doc:wia:soc013:{type}:{country}:{year}:{uid}`
2. Timestamps must be ISO 8601 format in UTC
3. Language codes must be ISO 639-1 or ISO 639-2
4. Country codes must be ISO 3166-1 alpha-2 or alpha-3
5. SHA-256 hashes must be lowercase hexadecimal

## 9. Extensibility

Custom metadata may be added under `extensions` namespace:

```json
{
  "extensions": {
    "custom:securityLevel": "confidential",
    "custom:retentionPeriod": "99 years"
  }
}
```

## 10. Compliance

This specification complies with:
- ISO 15489 (Records Management)
- ISO 19005 (PDF/A)
- ISO 15836 (Dublin Core)
- W3C Verifiable Credentials
- ICAO 9303 (Machine Readable Travel Documents)

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


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

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
