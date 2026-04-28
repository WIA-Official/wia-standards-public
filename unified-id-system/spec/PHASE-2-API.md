# WIA-UNI-002: Phase 2 - API Interface Specification

**Version:** 1.0
**Status:** Complete
**Last Updated:** 2025-01-15

---

## 1. Overview

This document specifies the REST API for WIA-UNI-002 Unified ID System operations including credential issuance, verification, revocation, and zero-knowledge proof generation.

## 2. Base URL

```
Production: https://api.unified-id.kr/v1/
Staging: https://api-staging.unified-id.kr/v1/
```

## 3. Authentication

### 3.1 OAuth 2.0 with OpenID Connect

All API requests require Bearer token authentication:

```http
Authorization: Bearer {access_token}
```

### 3.2 Token Endpoint

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials&
client_id={client_id}&
client_secret={client_secret}&
scope=verify:credentials
```

## 4. Core Endpoints

### 4.1 Issue Credential

**Endpoint:** `POST /v1/credentials`

**Authorization:** Government Authority only

**Request:**
```json
{
  "personalInfo": {
    "familyName": "Kim",
    "givenName": "MinJun",
    "birthDate": "1990-05-15"
  },
  "biometric": {
    "fingerprintHash": "sha256:...",
    "facialHash": "sha256:..."
  }
}
```

**Response (201 Created):**
```json
{
  "unifiedId": "UNI-KR-900515-1234",
  "credential": {
    "@context": ["https://www.w3.org/2018/credentials/v1"],
    "type": ["VerifiableCredential", "UnifiedIDCredential"],
    "issuer": "did:wia:unified-korea-authority",
    "credentialSubject": {
      "id": "did:wia:UNI-KR-900515-1234"
    },
    "proof": {...}
  }
}
```

### 4.2 Verify Credential

**Endpoint:** `POST /v1/verify`

**Authorization:** Service Provider

**Request:**
```json
{
  "credential": {
    "@context": ["https://www.w3.org/2018/credentials/v1"],
    ...
  }
}
```

**Response (200 OK):**
```json
{
  "verified": true,
  "checks": {
    "signature": "valid",
    "expiration": "valid",
    "revocation": "not_revoked"
  }
}
```

### 4.3 Generate Zero-Knowledge Proof

**Endpoint:** `POST /v1/proofs/zk`

**Authorization:** Citizen

**Request:**
```json
{
  "unifiedId": "UNI-KR-900515-1234",
  "proofType": "age-over-18",
  "parameters": {
    "currentDate": "2025-01-15"
  }
}
```

**Response (200 OK):**
```json
{
  "proof": {
    "type": "zk-SNARK",
    "proof": "0x...",
    "publicInputs": {
      "currentDate": "2025-01-15",
      "ageThreshold": 18
    }
  }
}
```

### 4.4 Revoke Credential

**Endpoint:** `POST /v1/revoke`

**Authorization:** Authority or Citizen

**Request:**
```json
{
  "unifiedId": "UNI-KR-900515-1234",
  "reason": "lost",
  "effectiveDate": "2025-01-15T10:00:00Z"
}
```

**Response (200 OK):**
```json
{
  "revoked": true,
  "revocationId": "rev-abc123"
}
```

## 5. Rate Limiting

### 5.1 Rate Limits by Client Type

| Client Type | Requests/Hour | Burst Limit |
|-------------|---------------|-------------|
| Government Authority | 100,000 | 1,000 |
| Certified Service Provider | 10,000 | 100 |
| Standard Service Provider | 1,000 | 50 |
| Individual Citizen | 100 | 10 |

### 5.2 Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 999
X-RateLimit-Reset: 1642252800
```

## 6. Error Handling

### 6.1 Standard Error Response

```json
{
  "error": {
    "code": "invalid_request",
    "message": "Missing required field: personalInfo.birthDate",
    "timestamp": "2025-01-15T10:30:00Z",
    "requestId": "req-xyz789"
  }
}
```

### 6.2 HTTP Status Codes

| Code | Description | Example |
|------|-------------|---------|
| 200 | Success | Verification successful |
| 201 | Created | Credential issued |
| 400 | Bad Request | Invalid JSON |
| 401 | Unauthorized | Missing/invalid token |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Credential not found |
| 429 | Rate Limit Exceeded | Too many requests |
| 500 | Internal Server Error | System error |

## 7. Webhooks

### 7.1 Credential Revocation Notifications

Registered service providers receive webhook notifications when credentials are revoked:

```http
POST {webhook_url}
Content-Type: application/json
X-WIA-Signature: sha256-hmac-signature

{
  "event": "credential.revoked",
  "unifiedId": "UNI-KR-900515-1234",
  "timestamp": "2025-01-15T10:00:00Z"
}
```

---

**弘益人間 (Benefit All Humanity)**

*WIA - World Certification Industry Association*
*© 2025 MIT License*

---

## Z.1 Audit transport and observability hooks (Phase 2)

Every Phase 2 envelope SHOULD emit a structured log line at the
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
`2` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 2)

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

## Z.3 Capabilities discovery and SemVer (Phase 2)

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

## Z.4 Privacy envelope per per-jurisdiction law (Phase 2)

Phase 2 envelopes that carry personal data MUST honour the
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

## Z.5 DR / continuity envelope per ISO 22301 (Phase 2)

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

## Z.6 Supply-chain envelope per SLSA (Phase 2)

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

## Z.1 Audit transport and observability hooks (Phase 2 (variant 1))

Every Phase 2 envelope SHOULD emit a structured log line at the
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
`2` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 2 (variant 1))

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

## Z.3 Capabilities discovery and SemVer (Phase 2 (variant 1))

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

## Z.4 Privacy envelope per per-jurisdiction law (Phase 2 (variant 1))

Phase 2 envelopes that carry personal data MUST honour the
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

## Z.5 DR / continuity envelope per ISO 22301 (Phase 2 (variant 1))

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

## Z.6 Supply-chain envelope per SLSA (Phase 2 (variant 1))

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
