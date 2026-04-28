# WIA-UNI-002: Phase 1 - Data Format Specification

**Version:** 1.0
**Status:** Complete
**Last Updated:** 2025-01-15

---

## 1. Overview

This document specifies the data format for the WIA-UNI-002 Unified ID System. It defines the JSON schema, field mappings, validation rules, and serialization formats required for interoperable identity records across the unified Korean territory.

## 2. Core Data Schema

### 2.1 Unified ID Record

```json
{
  "$schema": "https://wiastandards.com/schemas/uni-002/v1.0/unified-id.json",
  "version": "1.0",
  "standard": "WIA-UNI-002",
  "unifiedId": "UNI-KR-900515-1234",
  "personalInfo": {
    "familyName": "Kim",
    "givenName": "MinJun",
    "fullName": "Kim MinJun",
    "birthDate": "1990-05-15",
    "gender": "M"
  },
  "origin": {
    "region": "south",
    "previousId": "900515-1234567",
    "registrationDate": "2025-01-15T09:00:00Z"
  },
  "biometric": {
    "fingerprintHash": "sha256:a1b2c3d4e5f6...",
    "facialHash": "sha256:g7h8i9j0k1l2...",
    "irisHash": null
  },
  "issuedAt": "2025-01-15T09:30:00Z",
  "expiresAt": "2035-01-15T00:00:00Z",
  "issuer": "did:wia:unified-korea-authority",
  "status": "active",
  "privacyLevel": "protected"
}
```

## 3. Field Definitions

### 3.1 Required Fields

| Field | Type | Format | Description |
|-------|------|--------|-------------|
| `version` | string | `x.y` | Schema version |
| `standard` | string | `WIA-UNI-002` | Standard identifier |
| `unifiedId` | string | `UNI-KR-YYMMDD-NNNN` | Unique unified ID |
| `personalInfo` | object | - | Personal information |
| `personalInfo.familyName` | string | UTF-8 | Family name (surname) |
| `personalInfo.givenName` | string | UTF-8 | Given name |
| `personalInfo.birthDate` | string | ISO 8601 date | Birth date |
| `biometric` | object | - | Biometric hashes |
| `issuedAt` | string | ISO 8601 datetime | Issuance timestamp |
| `issuer` | string | DID | Issuing authority DID |
| `status` | string | enum | Credential status |

### 3.2 Optional Fields

| Field | Type | Format | Description |
|-------|------|--------|-------------|
| `personalInfo.gender` | string | M/F/X | Gender |
| `origin.region` | string | enum | Origin region (encrypted) |
| `origin.previousId` | string | - | Legacy ID (encrypted) |
| `biometric.irisHash` | string | hex | Iris biometric hash |
| `expiresAt` | string | ISO 8601 | Expiration date |

## 4. Unified ID Format

### 4.1 Structure

```
UNI-KR-YYMMDD-NNNN
```

- **UNI**: Universal prefix
- **KR**: Country code (Korea)
- **YYMMDD**: Birth date (last 2 digits of year, month, day)
- **NNNN**: Random 4-digit serial (NOT sequential)

### 4.2 Generation Algorithm

```python
def generate_unified_id(birth_date):
    import random
    yy = str(birth_date.year)[-2:]
    mm = f"{birth_date.month:02d}"
    dd = f"{birth_date.day:02d}"
    serial = f"{random.randint(1000, 9999)}"
    return f"UNI-KR-{yy}{mm}{dd}-{serial}"
```

## 5. Biometric Hashing

### 5.1 Fingerprint Processing

1. **Capture**: High-resolution scan (500+ DPI)
2. **Feature Extraction**: Extract minutiae points
3. **Template Generation**: ISO/IEC 19794-2 format
4. **Hashing**: SHA-256 hash of template

```python
biometric.fingerprintHash = "sha256:" + sha256(fingerprint_template).hexdigest()
```

### 5.2 Facial Recognition

1. **Capture**: 2D color photo + optional 3D scan
2. **Embedding**: Deep learning model (128D vector)
3. **Hashing**: Keyed hash with secret key

## 6. Validation Rules

### 6.1 Required Field Validation

- All required fields must be present
- `unifiedId` must match regex: `^UNI-KR-\d{6}-\d{4}$`
- `birthDate` must be valid ISO 8601 date in the past
- At least one biometric hash must be present

### 6.2 Data Type Validation

- Strings must be UTF-8 encoded
- Dates must be ISO 8601 format
- Enums must match allowed values
- DIDs must conform to W3C DID specification

## 7. Encryption for Sensitive Fields

### 7.1 Encrypted Personal Info

```json
{
  "personalInfo": {
    "encrypted": true,
    "algorithm": "AES-256-GCM",
    "ciphertext": "base64-encoded-ciphertext",
    "iv": "base64-encoded-iv",
    "keyId": "did:wia:unified-korea-authority#key-1"
  }
}
```

### 7.2 Key Management

- Encryption keys stored in Hardware Security Module (HSM)
- Key rotation every 90 days
- Decryption keys accessible only to authorized entities

## 8. Interoperability Formats

### 8.1 Supported Serializations

| Format | Use Case | Features |
|--------|----------|----------|
| JSON | APIs, web apps | Human-readable |
| JSON-LD | Verifiable Credentials | Linked data |
| Protocol Buffers | High-performance | Compact, fast |
| CBOR | IoT, offline | Binary, minimal overhead |
| QR Code | Physical verification | Camera-readable |

## 9. Versioning

### 9.1 Semantic Versioning

- **Major version**: Breaking changes (1.0 → 2.0)
- **Minor version**: Backward-compatible additions (1.0 → 1.1)
- **Patch version**: Bug fixes (1.0.0 → 1.0.1)

### 9.2 Compatibility

- Implementations MUST support current version
- Implementations SHOULD support one previous major version

## 10. Test Vectors

### 10.1 Valid Record Example

See Section 2.1 for a complete valid example.

### 10.2 Invalid Record Examples

**Missing required field:**
```json
{
  "version": "1.0",
  "standard": "WIA-UNI-002"
  // ERROR: Missing unifiedId, personalInfo, etc.
}
```

**Invalid ID format:**
```json
{
  "unifiedId": "INVALID-ID"
  // ERROR: Does not match UNI-KR-YYMMDD-NNNN pattern
}
```

---

## Appendix A: JSON Schema

Full JSON Schema available at:
https://wiastandards.com/schemas/uni-002/v1.0/unified-id.json

## Appendix B: Migration Tools

Legacy data migration tools available at:
https://github.com/WIA-Official/uni-002-migration-tools

---

**弘益人間 (Benefit All Humanity)**

*WIA - World Certification Industry Association*
*© 2025 MIT License*

---

## Z.1 Audit transport and observability hooks (Phase 1)

Every Phase 1 envelope SHOULD emit a structured log line at the
host's audit transport: timestamp per RFC 3339, host identifier,
tenant identifier, envelope class, envelope identifier, operation
outcome, and a W3C Trace Context `traceparent` propagated end-to-end
so a single operation can be reconstructed across hosts. Phase 2
surfaces this trace identifier as the `X-WIA-Trace-Id` response
header. Phase 3 protocol exchanges propagate the trace identifier
inside the exchange envelope so that a federation crossing remains
correlatable end-to-end. Phase 4 integrators consume the audit
stream into the operator's SIEM (Splunk, Elastic, Sumo Logic,
Wazuh, Microsoft Sentinel) per OpenTelemetry semantic conventions,
with `wia.standard.slug` = `unified-id-system` and `wia.standard.phase` =
`1` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 1)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 Sec 5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations. The composition also lets the operator's SIEM
correlate per-tenant audit records across multiple standards
without per-standard schema-mapping work.

## Z.3 Capabilities discovery and SemVer (Phase 1)

Hosts SHOULD publish a capabilities document at
`/.well-known/wia-unified-id-system-capabilities` enumerating per-endpoint
optionality. Clients MUST treat unsupported capabilities as absent
rather than as an error condition; a client that needs a capability
the host does not advertise MUST surface a clear configuration
error rather than silently degrade. Hosts moving from one minor
version to the next MUST publish the change in the host's release
notes with the per-capability migration window per IETF RFC 8594
(Sunset header) + RFC 9745 (Deprecation header) + RFC 9651
(Structured Field Values) so machine consumers can plan migration
without waiting for human-channel notification.

## Z.4 Privacy envelope per per-jurisdiction law (Phase 1)

Phase 1 envelopes that carry personal data MUST honour the
operator's per-jurisdiction privacy law (EU GDPR per Regulation
2016/679; UK GDPR per UK Data Protection Act 2018; California
CPRA per Cal. Civ. Code Sec 1798.100; Brazil LGPD per Lei 13.709/2018;
Canada PIPEDA per S.C. 2000 c.5; Korea PIPA per 개인정보 보호법;
Japan APPI per 個人情報の保護に関する法律; Australia Privacy Act
1988 per Cth) including data-minimisation, purpose-limitation,
storage-limitation, integrity + confidentiality, and accountability
principles. Subject-rights endpoints (access, rectification,
erasure, portability, restriction, objection) compose with
WIA-OMNI-API Sec 5 subject-rights surface and need not be
re-implemented per-standard.

## Z.5 DR / continuity envelope per ISO 22301 (Phase 1)

Hosts running this Phase MUST publish a continuity-of-operations
envelope per ISO 22301:2019 + ISO/IEC 27031 + NIST SP 800-34 Rev 1
covering: per-host RTO (Recovery Time Objective) per the operator's
business-impact analysis; per-host RPO (Recovery Point Objective)
tied to the host's audit-stream replication policy; per-host
backup envelope (per-region cross-replicated immutable backup with
the per-tier retention envelope per the per-jurisdiction record-
retention policy); per-host failover-rehearsal envelope (typically
quarterly per the operator's BC/DR program); per-host vendor-exit
envelope so the operator can migrate the host to an alternate
implementation without losing audit-trail continuity.

## Z.6 Supply-chain envelope per SLSA (Phase 1)

Every host implementation MUST publish a software-bill-of-materials
(SBOM) per SPDX 2.3 / 3.0 (per ISO/IEC 5962 + Linux Foundation SPDX)
or CycloneDX 1.6 (per OWASP Foundation). The SBOM enumerates every
direct + transitive dependency with the per-component name +
version + licence + supplier + per-component hash + per-component
PURL (Package URL per package-url spec) + per-component CPE
(Common Platform Enumeration per NIST). Supply-chain attestation
follows in-toto per CNCF in-toto + SLSA (Supply-chain Levels for
Software Artifacts) per OpenSSF SLSA Framework — typically targeting
SLSA Level 3 for hosted production deployments.

弘益人間 — Benefit All Humanity.

---

## Z.1 Audit transport and observability hooks (Phase 1 (variant 1))

Every Phase 1 envelope SHOULD emit a structured log line at the
host's audit transport: timestamp per RFC 3339, host identifier,
tenant identifier, envelope class, envelope identifier, operation
outcome, and a W3C Trace Context `traceparent` propagated end-to-end
so a single operation can be reconstructed across hosts. Phase 2
surfaces this trace identifier as the `X-WIA-Trace-Id` response
header. Phase 3 protocol exchanges propagate the trace identifier
inside the exchange envelope so that a federation crossing remains
correlatable end-to-end. Phase 4 integrators consume the audit
stream into the operator's SIEM (Splunk, Elastic, Sumo Logic,
Wazuh, Microsoft Sentinel) per OpenTelemetry semantic conventions,
with `wia.standard.slug` = `unified-id-system` and `wia.standard.phase` =
`1` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 1 (variant 1))

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 Sec 5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations. The composition also lets the operator's SIEM
correlate per-tenant audit records across multiple standards
without per-standard schema-mapping work.

## Z.3 Capabilities discovery and SemVer (Phase 1 (variant 1))

Hosts SHOULD publish a capabilities document at
`/.well-known/wia-unified-id-system-capabilities` enumerating per-endpoint
optionality. Clients MUST treat unsupported capabilities as absent
rather than as an error condition; a client that needs a capability
the host does not advertise MUST surface a clear configuration
error rather than silently degrade. Hosts moving from one minor
version to the next MUST publish the change in the host's release
notes with the per-capability migration window per IETF RFC 8594
(Sunset header) + RFC 9745 (Deprecation header) + RFC 9651
(Structured Field Values) so machine consumers can plan migration
without waiting for human-channel notification.

## Z.4 Privacy envelope per per-jurisdiction law (Phase 1 (variant 1))

Phase 1 envelopes that carry personal data MUST honour the
operator's per-jurisdiction privacy law (EU GDPR per Regulation
2016/679; UK GDPR per UK Data Protection Act 2018; California
CPRA per Cal. Civ. Code Sec 1798.100; Brazil LGPD per Lei 13.709/2018;
Canada PIPEDA per S.C. 2000 c.5; Korea PIPA per 개인정보 보호법;
Japan APPI per 個人情報の保護に関する法律; Australia Privacy Act
1988 per Cth) including data-minimisation, purpose-limitation,
storage-limitation, integrity + confidentiality, and accountability
principles. Subject-rights endpoints (access, rectification,
erasure, portability, restriction, objection) compose with
WIA-OMNI-API Sec 5 subject-rights surface and need not be
re-implemented per-standard.

## Z.5 DR / continuity envelope per ISO 22301 (Phase 1 (variant 1))

Hosts running this Phase MUST publish a continuity-of-operations
envelope per ISO 22301:2019 + ISO/IEC 27031 + NIST SP 800-34 Rev 1
covering: per-host RTO (Recovery Time Objective) per the operator's
business-impact analysis; per-host RPO (Recovery Point Objective)
tied to the host's audit-stream replication policy; per-host
backup envelope (per-region cross-replicated immutable backup with
the per-tier retention envelope per the per-jurisdiction record-
retention policy); per-host failover-rehearsal envelope (typically
quarterly per the operator's BC/DR program); per-host vendor-exit
envelope so the operator can migrate the host to an alternate
implementation without losing audit-trail continuity.

## Z.6 Supply-chain envelope per SLSA (Phase 1 (variant 1))

Every host implementation MUST publish a software-bill-of-materials
(SBOM) per SPDX 2.3 / 3.0 (per ISO/IEC 5962 + Linux Foundation SPDX)
or CycloneDX 1.6 (per OWASP Foundation). The SBOM enumerates every
direct + transitive dependency with the per-component name +
version + licence + supplier + per-component hash + per-component
PURL (Package URL per package-url spec) + per-component CPE
(Common Platform Enumeration per NIST). Supply-chain attestation
follows in-toto per CNCF in-toto + SLSA (Supply-chain Levels for
Software Artifacts) per OpenSSF SLSA Framework — typically targeting
SLSA Level 3 for hosted production deployments.

弘益人間 — Benefit All Humanity.
