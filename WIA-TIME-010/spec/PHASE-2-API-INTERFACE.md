# WIA-TIME-001: Phase 2 - API Interface Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document specifies the API interfaces for time travel physics systems in the WIA-TIME-001 standard. All implementations MUST provide these endpoints for temporal calculations, navigation, and safety verification.

## 2. REST API Endpoints

### 2.1 Base URL
```
https://api.wia-time.org/v1/
```

### 2.2 Temporal Calculation Endpoints

#### Calculate Displacement
```http
POST /calculate/displacement
Content-Type: application/json
Authorization: Bearer {token}

{
  "origin": {
    "time": "2025-01-01T00:00:00Z",
    "coordinates": { "x": 0, "y": 0, "z": 0 },
    "referenceFrame": "earth"
  },
  "destination": {
    "time": "2020-01-01T00:00:00Z",
    "coordinates": { "x": 0, "y": 0, "z": 0 },
    "referenceFrame": "earth"
  },
  "travelerMass": 75
}

Response 200:
{
  "displacementId": "uuid",
  "deltaTime": -157766400,
  "energyRequired": 6.75e18,
  "causalityRisk": "low",
  "feasibility": "possible"
}
```

#### Calculate Energy Requirements
```http
POST /calculate/energy
Content-Type: application/json

{
  "displacementId": "uuid",
  "method": "wormhole|alcubierre|tipler"
}

Response 200:
{
  "totalEnergy": 6.75e18,
  "exoticMatter": -1e15,
  "powerDuration": 3.5,
  "sources": [
    { "type": "antimatter", "amount": 75 }
  ]
}
```

### 2.3 Causality Endpoints

#### Check Paradox Risk
```http
POST /causality/check
Content-Type: application/json

{
  "displacementId": "uuid",
  "plannedActions": [
    { "type": "observation", "target": "historical_event" }
  ]
}

Response 200:
{
  "paradoxRisk": "low",
  "probability": 0.001,
  "warnings": [],
  "approved": true
}
```

#### Verify Timeline Integrity
```http
GET /causality/timeline/{timelineId}/integrity
Authorization: Bearer {token}

Response 200:
{
  "timelineId": "TL-PRIME-A1-001",
  "integrity": 0.9999,
  "anomalies": 0,
  "lastVerified": "2025-01-01T00:00:00Z"
}
```

### 2.4 Navigation Endpoints

#### Plot Worldline
```http
POST /navigation/worldline
Content-Type: application/json

{
  "origin": "SpacetimeCoordinate",
  "destination": "SpacetimeCoordinate",
  "constraints": {
    "maxAcceleration": 10,
    "avoidMassive": true
  }
}

Response 200:
{
  "worldlineId": "uuid",
  "segments": [...],
  "totalProperTime": 3600,
  "waypoints": [...]
}
```

## 3. TypeScript SDK

### 3.1 Installation
```bash
npm install @wia/time-travel-sdk
```

### 3.2 Basic Usage
```typescript
import { TemporalClient, Displacement, CausalityChecker } from '@wia/time-travel-sdk';

const client = new TemporalClient({
  apiKey: process.env.WIA_TIME_API_KEY,
  version: 'v1'
});

// Calculate displacement
const displacement = await client.calculate.displacement({
  origin: { time: new Date('2025-01-01'), frame: 'earth' },
  destination: { time: new Date('2020-01-01'), frame: 'earth' },
  travelerMass: 75
});

// Check causality
const causalityCheck = await client.causality.check({
  displacementId: displacement.id,
  plannedActions: [{ type: 'observation' }]
});

if (causalityCheck.approved) {
  console.log('Time travel approved');
}
```

### 3.3 Type Definitions
```typescript
interface SpacetimeCoordinate {
  time: Date | number;
  x: number;
  y: number;
  z: number;
  frame: 'earth' | 'solar' | 'galactic';
}

interface Displacement {
  id: string;
  origin: SpacetimeCoordinate;
  destination: SpacetimeCoordinate;
  deltaTime: number;
  energyRequired: number;
  causalityRisk: 'none' | 'low' | 'medium' | 'high';
}

interface CausalityResult {
  paradoxRisk: string;
  probability: number;
  approved: boolean;
  warnings: string[];
}
```

## 4. Authentication

### 4.1 API Keys
```http
Authorization: Bearer {api_key}
```

### 4.2 OAuth 2.0
```http
Authorization: Bearer {access_token}
```

### 4.3 Temporal Signatures
```http
X-Temporal-Signature: HMAC-SHA256({timestamp}:{request_hash})
X-Temporal-Timestamp: 1704067200
```

## 5. Rate Limits

| Tier | Requests/Hour | Calculations/Day |
|------|---------------|------------------|
| Free | 100 | 10 |
| Basic | 1,000 | 100 |
| Professional | 10,000 | 1,000 |
| Enterprise | Unlimited | Unlimited |

## 6. Error Codes

| Code | Description |
|------|-------------|
| 400 | Invalid parameters |
| 401 | Unauthorized |
| 403 | Causality violation detected |
| 404 | Resource not found |
| 429 | Rate limit exceeded |
| 500 | Temporal calculation error |
| 503 | Timeline unstable |

---

**弘益人間 (Benefit All Humanity)**

*© 2025 WIA - World Certification Industry Association*
*MIT License*

## P.2 API Surface Cross-References

The API surface defined in this Phase consumes and emits the data formats from
Phase 1 and is transported by the protocol layer in Phase 3. Operators deploy
the surface using the integration patterns in Phase 4.

### P.2.1 Resource Naming

Resource paths follow REST conventions with snake_case segments. Identifier
segments use the canonical UUID encoding from Phase 1.

```
/v1/{collection}                        # collection
/v1/{collection}/{id}                    # member
/v1/{collection}/{id}/{sub_collection}   # nested collection
/v1/{collection}/{id}:{action}           # custom action (POST)
```

### P.2.2 Pagination

List endpoints support cursor-based pagination:

| Param | Default | Max | Description |
|-------|---------|-----|-------------|
| `page_size` | 50 | 500 | Items per page |
| `page_token` | empty | — | Opaque continuation token |

Servers MUST return `next_page_token` when the result set is truncated and an
empty string when the final page has been delivered.

### P.2.3 Idempotency

State-changing operations accept the `Idempotency-Key` header (RFC-style).
Servers MUST cache the response keyed by `(principal, key)` for at least 24 h
and replay the same response on retry.

### P.2.4 Field Masks

Partial-update operations use field masks (Google AIP-161 style) to avoid
clobbering unspecified fields. Masks are dot-paths into the canonical schema
with `*` wildcards.


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
in lockstep across Phases 1–4 of WIA-TIME-010: Paradox Prevention Specification v1.0 so that conformance claims at any
Phase remain unambiguous.*

