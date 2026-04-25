# WIA-SOC-015: Phase 1 - Ballot Data Format Specification

**Version:** 1.0  
**Status:** FINAL  
**Last Updated:** 2025-01-15  
**Standards Body:** World Certification Industry Association (WIA)

---

## 1. Overview

Phase 1 of WIA-SOC-015 establishes standardized data formats for ballots, candidates, contests, and election results. This specification enables interoperability across different voting systems by defining common JSON schemas, validation rules, and encoding standards.

### 1.1 Purpose

- Enable data exchange between different voting system vendors
- Provide consistent ballot representation across jurisdictions
- Support multilingual and accessible ballot formats
- Establish foundation for Phases 2-4 functionality

### 1.2 Scope

This specification covers:
- Ballot data structures
- Candidate and contest definitions
- Election metadata
- Result data formats
- Multilingual support
- Accessibility annotations
- Security metadata

---

## 2. Core Data Structures

### 2.1 Ballot Definition

A ballot is represented as a JSON object conforming to the following schema:

```json
{
  "$schema": "https://wia-standards.org/schemas/voting/ballot-v1.0.json",
  "ballotId": "string (required, unique identifier)",
  "version": "string (required, WIA-SOC-015 version)",
  "electionId": "string (required, election identifier)",
  "electionName": "object (required, i18n)",
  "jurisdiction": "object (required)",
  "metadata": "object (required)",
  "contests": "array (required)",
  "accessibility": "object (optional)",
  "security": "object (required)"
}
```

### 2.2 Contest Definition

Each contest on a ballot is defined as:

```json
{
  "contestId": "string (required)",
  "contestType": "enum: candidate|referendum|recall",
  "contestName": "object (required, i18n)",
  "votingMethod": "enum: single-choice|multiple-choice|ranked-choice",
  "maxSelections": "number (required)",
  "minSelections": "number (optional, default: 0)",
  "writeInAllowed": "boolean (optional, default: false)",
  "candidates": "array (required for candidate type)",
  "ballotStyle": "object (optional)"
}
```

### 2.3 Candidate Definition

Candidates are represented as:

```json
{
  "candidateId": "string (required, unique)",
  "name": "string (required)",
  "party": "string (optional)",
  "photoUrl": "string (optional, URL)",
  "biography": "object (optional, i18n)",
  "contactInfo": "object (optional)"
}
```

---

## 3. Field Specifications

### 3.1 Required Fields

| Field | Type | Description | Constraints |
|-------|------|-------------|-------------|
| ballotId | string | Unique ballot identifier | Max 255 chars, alphanumeric+dash |
| version | string | WIA standard version | Format: WIA-SOC-015-v{major}.{minor} |
| electionId | string | Election identifier | Max 255 chars, unique within jurisdiction |
| electionName | object | Multilingual election name | Must include 'en' key minimum |
| contests | array | List of contests | Min 1 contest required |

### 3.2 Multilingual Support (i18n)

All user-facing text must be provided as internationalization objects:

```json
{
  "en": "Presidential Election 2025",
  "ko": "2025년 대통령 선거",
  "es": "Elección Presidencial 2025",
  "fr": "Élection présidentielle 2025"
}
```

**Requirements:**
- Must include 'en' (English) as baseline
- Should include languages spoken by >5% of jurisdiction
- Platinum certification requires all 99 WIA-supported languages
- Use ISO 639-1 two-letter language codes

---

## 4. Validation Rules

### 4.1 Schema Validation

All ballot data must pass JSON Schema validation before acceptance.

**Schema URL:** `https://wia-standards.org/schemas/voting/ballot-v1.0.json`

### 4.2 Business Logic Validation

| Rule ID | Rule Description | Error Code |
|---------|------------------|------------|
| VAL-001 | Election dates must be in future at creation | FUTURE_DATE_REQUIRED |
| VAL-002 | maxSelections >= minSelections | INVALID_SELECTION_RANGE |
| VAL-003 | No duplicate candidateId within contest | DUPLICATE_CANDIDATE |
| VAL-004 | All i18n objects must include 'en' key | MISSING_ENGLISH |
| VAL-005 | Digital signatures must be present and valid | INVALID_SIGNATURE |
| VAL-006 | Contest array cannot be empty | NO_CONTESTS |
| VAL-007 | Jurisdiction country code must be ISO 3166-1 | INVALID_COUNTRY_CODE |

---

## 5. Accessibility Annotations

### 5.1 Screen Reader Support

```json
{
  "accessibility": {
    "screenReader": {
      "ballotDescription": {
        "en": "Presidential Election Ballot containing 3 contests",
        "ko": "3개의 경선을 포함한 대통령 선거 투표용지"
      },
      "navigationHints": {
        "en": "Use arrow keys to navigate between contests"
      }
    },
    "audioFormat": {
      "available": true,
      "audioUrl": "https://cdn.example.com/ballot-audio-en.mp3"
    },
    "tactileFormat": {
      "available": true,
      "brailleUrl": "https://cdn.example.com/ballot-braille.brf"
    }
  }
}
```

### 5.2 Visual Accommodations

- High contrast color schemes
- Scalable text (up to 36pt)
- Symbol/icon support for candidates
- Color-blind friendly palettes

---

## 6. Security Metadata

### 6.1 Digital Signatures

Every ballot must include cryptographic signature:

```json
{
  "security": {
    "digitalSignature": {
      "algorithm": "ECDSA-P256",
      "publicKey": "base64-encoded-key",
      "signature": "base64-encoded-signature",
      "timestamp": "2025-01-15T10:00:00Z",
      "signer": "Election Authority CA"
    }
  }
}
```

### 6.2 Hash Chains

Ballots participate in hash chain for integrity:

```json
{
  "hashChain": {
    "algorithm": "SHA3-256",
    "ballotHash": "hex-encoded-hash",
    "previousHash": "hex-encoded-previous-hash",
    "blockchainTxId": "blockchain-transaction-id"
  }
}
```

---

## 7. Result Data Format

### 7.1 Result Structure

```json
{
  "resultId": "string (required)",
  "version": "WIA-SOC-015-v1.0",
  "electionId": "string (required)",
  "contestId": "string (required)",
  "reportingUnit": {
    "type": "enum: precinct|district|state|national",
    "id": "string",
    "name": "string"
  },
  "timestamp": "ISO 8601 datetime",
  "status": "enum: preliminary|final|certified",
  "totalBallots": "number",
  "results": [
    {
      "candidateId": "string",
      "votes": "number",
      "percentage": "number (0-100)"
    }
  ],
  "auditData": {
    "ballotsAudited": "number",
    "discrepancies": "number",
    "auditHash": "string",
    "blockchainProof": "string"
  }
}
```

---

## 8. Extension Mechanisms

### 8.1 Custom Fields

Jurisdictions may add custom fields under "extensions" object:

```json
{
  "ballotId": "BALLOT-2025-001",
  "version": "WIA-SOC-015-v1.0",
  "extensions": {
    "jurisdiction-specific": {
      "customField1": "value",
      "customField2": 123
    }
  }
}
```

**Requirements:**
- Extensions must not duplicate standard fields
- Extensions must not contradict standard requirements
- Core functionality must work without extension data

---

## 9. Implementation Guidelines

### 9.1 Data Storage

- Use encryption at rest (AES-256 minimum)
- Implement immutable storage (append-only)
- Maintain multiple redundant copies
- Log all data access with audit trail

### 9.2 Data Migration

When converting legacy data:
1. Analyze existing data structures
2. Create mapping rules to WIA format
3. Implement validation to ensure data quality
4. Test migration with sample data
5. Maintain legacy data alongside WIA format during transition

### 9.3 Performance Optimization

- Use compression for ballot transmission (gzip/brotli)
- Implement caching for frequently accessed ballots
- Distribute static content via CDN
- Index key fields for efficient querying

---

## 10. Conformance Requirements

### 10.1 Bronze Certification

To achieve Bronze certification, systems must:
- Accept and validate WIA-SOC-015 ballot JSON
- Support English language minimum
- Implement required field validation
- Generate compliant result data

### 10.2 Silver/Gold/Platinum Additional Requirements

Higher certification levels add:
- Multi-language support (Silver: 10+, Gold: 50+, Platinum: 99)
- Full accessibility annotations
- Advanced security metadata
- Extension mechanism support

---

## 11. Examples

### 11.1 Complete Ballot Example

See Appendix A for full ballot example including all required and optional fields.

### 11.2 Result Example

See Appendix B for complete result data example.

---

## 12. References

- [JSON Schema Specification](https://json-schema.org/)
- [ISO 639-1 Language Codes](https://www.iso.org/iso-639-language-codes.html)
- [ISO 3166-1 Country Codes](https://www.iso.org/iso-3166-country-codes.html)
- [ISO 8601 Date/Time Format](https://www.iso.org/iso-8601-date-and-time-format.html)

---

**© 2025 World Certification Industry Association (WIA)**  
**弘益人間 (Hongik Ingan) - Benefit All Humanity**  
**License:** MIT License

## P.1 Data Format Cross-References

This Phase defines the canonical data types referenced by the API surface (Phase 2),
the wire protocol (Phase 3), and integration scenarios (Phase 4). Implementations
MUST round-trip every canonical type through serialization and deserialization
without loss of precision or semantics.

### P.1.1 Canonical Encoding Rules

1. UTF-8 is the required character encoding for textual fields.
2. Numeric fields use IEEE 754 binary64 unless explicitly marked as fixed-point.
3. Timestamps use RFC 3339 with timezone offset; durations use ISO 8601.
4. UUIDs follow RFC 4122 v4 unless deterministic IDs are required.
5. Binary payloads are encoded as Base64 (RFC 4648 §4) in JSON contexts and as
   raw octet strings in Protocol Buffers / CBOR contexts.

### P.1.2 Schema Evolution

Schema changes follow these compatibility classes:

| Class | Allowed Changes | Wire-Compat |
|-------|-----------------|-------------|
| Patch | Doc fixes, examples, validator tightening within existing range | Forward & backward |
| Minor | New optional fields, new enum values with default fallback        | Forward |
| Major | Field rename, type change, removal, semantics change              | None |

### P.1.3 Validation Order

Validators MUST apply checks in this order: (1) syntactic well-formedness,
(2) schema conformance, (3) cross-field invariants, (4) external referential
integrity, (5) policy / authorization. A failure short-circuits subsequent
checks; the response message identifies the first failing rule by ID.


---

## Appendix · Common WIA Standard Provisions

> The following provisions apply to every WIA standard and are kept in sync
> across the WIA-Standards corpus. Standard-specific deviations, where they
> exist, are listed in the standard's normative body.

### A. Conformance & Compliance

#### A.1 Conformance Levels

WIA-Standards defines four conformance levels:

| Level | Required | Description |
|-------|----------|-------------|
| L1 — Format | Phase 1 | Implementation produces and consumes the canonical data format losslessly |
| L2 — Interface | Phase 1 + 2 | Implementation exposes the API surface with required behaviour |
| L3 — Protocol | Phase 1 + 2 + 3 | Implementation interoperates over at least one normative transport binding |
| L4 — Integration | All Phases | Implementation passes the conformance test suite end-to-end in a production-shaped deployment |

Conformance claims MUST cite the level and the version of the standard
against which the claim is made (e.g. "L3 conformant against v1.0").

#### A.2 Compliance Verification

The conformance test suite is published alongside this standard at
`/cli/conformance/` and `/api/conformance/`. Implementations claiming
L2 or higher MUST publish their test report. Independent re-tests are
encouraged; the WIA Working Group accepts third-party verification reports
under the policy in §E.

### B. Security Considerations

#### B.1 Threat Model

Implementers SHOULD apply STRIDE analysis covering: spoofing of identity,
tampering with messages or stored state, repudiation of operations,
information disclosure, denial of service, and elevation of privilege.

| Threat | Default Control | Where Strengthened |
|--------|-----------------|--------------------|
| Spoofing | Mutual TLS or signed tokens | Phase 3 §P.3 |
| Tampering | TLS in transit, AEAD at rest | Phase 1 §P.1 |
| Repudiation | Append-only audit log with notarization | Phase 4 §P.4 |
| Disclosure | Field-level encryption for PII / secrets | Phase 1 §P.1 |
| DoS | Rate limit per principal & global circuit breaker | Phase 2 §P.2 |
| EoP | Least-privilege RBAC + scoped tokens | Phase 2 §P.2 |

#### B.2 Cryptographic Suites

Mandatory: TLS 1.3 with AEAD ciphers (AES-128-GCM, AES-256-GCM,
CHACHA20-POLY1305). Forbidden: TLS 1.0, TLS 1.1, RC4, MD5, SHA-1 for
signatures, RSA below 2048 bits, ECDSA on curves smaller than P-256.

Post-quantum migration: implementations SHOULD adopt hybrid key
exchange combining a classical primitive with ML-KEM (FIPS 203) once a
profile is published; signature migration to ML-DSA (FIPS 204) is
expected within the L4 conformance window of v2.0.

#### B.3 Audit Requirements

L3 and L4 implementations MUST log: (a) every authentication decision,
(b) every authorization decision, (c) every state-changing operation,
(d) every export of data outside its sovereignty boundary. Logs are
write-once for at least 1 year and 90 days indexed for incident search.

### C. Versioning & Lifecycle

Versions follow Semantic Versioning 2.0.0 (MAJOR.MINOR.PATCH).

| Phase | Duration | Conformance Status |
|-------|---------:|-------------------|
| Draft | until ratification | Non-binding |
| Active | indefinite | Binding for new deployments |
| Maintenance | 24 months from successor's Active date | Binding for existing deployments |
| Retired | indefinite | Non-binding; conformance claims rescinded |

Deprecation MUST be announced at least one minor version before a feature
is removed in a major version.

### D. Internationalization & Accessibility

Implementations SHOULD support locale negotiation via the
`Accept-Language` header (RFC 4647). Date, time, number, and currency
formatting follow CLDR. User-facing surfaces MUST satisfy WCAG 2.1 AA at
minimum and SHOULD progress towards WCAG 2.2 AA. Right-to-left scripts
(Arabic, Hebrew, Persian, Urdu) and East-Asian wide characters MUST be
laid out correctly without line-breaking heuristics that split graphemes.

### E. Governance & IP Policy

This standard is maintained by the WIA Working Group under the WIA
governance charter. Editorial changes are merged via pull request. Normative
changes require working-group consensus and a 30-day public review.
Contributions are accepted under the Apache License 2.0 with explicit
patent grant. Members participate under the WIA Patent Policy,
which requires royalty-free licensing of any essential claim necessary
to implement a normative requirement.

### F. Normative References

The following references are normative; implementations MUST satisfy
the cited clauses:

- ISO/IEC 27001:2022 — Information security management systems
- ISO/IEC 27017:2015 — Cloud-services security controls
- ISO/IEC 27701:2019 — Privacy information management
- ISO/IEC 19790:2012 — Security requirements for cryptographic modules
- ISO 8601-1:2019 — Date and time representation
- IETF RFC 8446 — TLS 1.3
- IETF RFC 7519 — JSON Web Token
- IETF RFC 6749 — OAuth 2.0
- IETF RFC 9110 — HTTP Semantics
- IETF RFC 9112 — HTTP/1.1 message syntax
- IETF RFC 9113 — HTTP/2
- IETF RFC 9114 — HTTP/3
- IETF RFC 9000 — QUIC transport
- IETF RFC 4122 — UUID URN namespace
- IETF RFC 3339 — Date and time on the Internet
- IETF RFC 6838 — Media-type specifications and registration
- W3C TraceContext — Distributed tracing context
- W3C WCAG 2.1 — Accessibility guidelines
- FIPS PUB 197 — AES
- FIPS PUB 180-4 — SHA-2 family
- FIPS PUB 203 — ML-KEM (post-quantum KEM)
- FIPS PUB 204 — ML-DSA (post-quantum signature)

### G. Glossary

| Term | Definition |
|------|------------|
| Conformance | The state of satisfying every normative requirement at a given level |
| Implementation | A software, hardware, or composite artefact that claims conformance |
| Principal | The authenticated entity bound to a security context |
| Subject | The resource or person to which an operation applies |
| Sovereignty Boundary | The legal / regulatory perimeter outside of which data export is restricted |

---

*This Appendix is authored by the WIA Standards Working Group and is kept
in lockstep across Phases 1–4 of voting-system so that conformance claims at any
Phase remain unambiguous.*

