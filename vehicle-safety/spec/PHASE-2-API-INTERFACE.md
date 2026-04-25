# WIA-AUTO-022 PHASE 2: API Interface

> **Version:** 1.0.0  
> **Status:** Active  
> **Last Updated:** 2025-12-27  
> **Focus:** RESTful APIs, TypeScript interfaces, integration patterns

---

## Overview

Phase 2 establishes RESTful API specifications for vehicle safety system integration. Building upon Phase 1's data formats, these APIs enable real-time safety assessments, collision warnings, airbag deployment decisions, and certification verification.

**弘益人間 (Benefit All Humanity)** - Open APIs democratize safety technology, enabling third-party innovation that saves lives.

---

## Core API Endpoints

### 1. Crash Assessment API

**Endpoint:** `POST /api/v1/crash-assessment`

**Purpose:** Calculate injury risk and safety ratings from crash parameters

**Request:**
```typescript
interface CrashAssessmentRequest {
  vehicleMass: number;           // kg
  impactVelocity: number;        // m/s
  impactAngle: number;           // degrees (0=frontal, 90=side)
  crumpleZoneLength: number;     // meters
  occupants: OccupantInfo[];
}
```

**Response:**
```typescript
interface CrashAssessmentResponse {
  impactForce: number;           // Newtons
  deceleration: number;          // g's
  impactDuration: number;        // seconds
  energyAbsorbed: number;        // joules
  occupantInjury: {
    [position: string]: {
      hic15: number;
      chestDeflection: number;   // mm
      femurLoad: number;         // kN
      tibiaIndex: number;
      injuryRisk: 'low' | 'moderate' | 'high' | 'severe';
    }
  };
  safetyRating: 1 | 2 | 3 | 4 | 5;
}
```

**Latency SLA:** < 50ms p99

---

### 2. Airbag Deployment Decision API

**Endpoint:** `POST /api/v1/airbag-deployment`

**Purpose:** Determine optimal airbag deployment strategy in real-time

**Request:**
```typescript
interface AirbagDeploymentRequest {
  crashSeverity: number;         // 0-10 scale
  crashType: 'frontal' | 'side' | 'rear' | 'rollover';
  impactVelocity: number;        // m/s
  occupantPresence: boolean[];   // [driver, passenger, rear_L, rear_R]
  occupantClassification: string[]; // ['adult', 'child', 'infant', 'empty']
  beltStatus: boolean[];
  crashPulse: number[];          // acceleration time series (g)
}
```

**Response:**
```typescript
interface AirbagDeploymentResponse {
  shouldDeploy: {
    driverFrontal: {deploy: boolean, stage: 0 | 1 | 2, timing: number},
    passengerFrontal: {deploy: boolean, stage: 0 | 1 | 2, timing: number},
    driverSide: {deploy: boolean, timing: number},
    curtainLeft: {deploy: boolean, timing: number}
  };
  pretensioners: {
    driver: {activate: boolean, timing: number, force: number},
    passenger: {activate: boolean, timing: number, force: number}
  };
  reasoning: string[];
}
```

**Latency SLA:** < 10ms p99 (safety-critical)

---

### 3. Collision Warning API

**Endpoint:** `POST /api/v1/collision-warning`

**Purpose:** Assess collision threats and recommend interventions

**Request:**
```typescript
interface CollisionWarningRequest {
  ownVehicle: {
    velocity: number;            // m/s
    position: {x: number, y: number};
    heading: number;             // degrees
  };
  obstacles: Array<{
    type: 'vehicle' | 'pedestrian' | 'cyclist' | 'object';
    position: {x: number, y: number};
    velocity: {vx: number, vy: number};
    confidence: number;          // 0-1
  }>;
  roadCondition: 'dry' | 'wet' | 'snow' | 'ice';
}
```

**Response:**
```typescript
interface CollisionWarningResponse {
  threats: Array<{
    obstacleId: number;
    ttc: number;                 // time to collision (seconds)
    severity: 'low' | 'medium' | 'high' | 'critical';
    collisionProbability: number; // 0-1
    recommendedAction: 'none' | 'warning' | 'brake' | 'emergency_brake' | 'evade';
  }>;
  warningLevel: 0 | 1 | 2 | 3;
  interventionRequired: boolean;
}
```

**Latency SLA:** < 20ms p99

---

### 4. Safety Status Query API

**Endpoint:** `GET /api/v1/safety-status/{vin}`

**Purpose:** Retrieve current safety system status for a vehicle

**Response:**
```typescript
interface SafetyStatusResponse {
  timestamp: string;
  vehicle_id: string;
  systems: {
    abs: {status: string, last_test: string, fault_codes: string[]},
    esc: {status: string, calibration_date: string},
    airbags: {[position: string]: {status: string, deployment_count: number}},
    aeb: {status: string, sensitivity: string, range: number},
    ldw: {status: string, last_calibration: string}
  };
}
```

---

### 5. NCAP Rating Query API

**Endpoint:** `GET /api/v1/ncap-rating/{vin}`

**Purpose:** Retrieve vehicle safety ratings and certification

**Response:**
```typescript
interface NCAPRatingResponse {
  program: string;
  year: number;
  vehicle: {make: string, model: string, variant: string};
  overall_rating: 1 | 2 | 3 | 4 | 5;
  scores: {
    adult_occupant: {percentage: number, points: number, max_points: number},
    child_occupant: {percentage: number, points: number, max_points: number},
    vru: {percentage: number, points: number, max_points: number},
    safety_assist: {percentage: number, points: number, max_points: number}
  };
  verifiable_credential: object; // W3C VC format
}
```

---

## Authentication & Authorization

### OAuth 2.0 Client Credentials Flow

**Token Endpoint:** `POST /oauth/token`

**Request:**
```
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials&
client_id=YOUR_CLIENT_ID&
client_secret=YOUR_CLIENT_SECRET&
scope=safety:read safety:write
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "safety:read safety:write"
}
```

### Scopes

- `safety:read` - Read-only access to safety data and ratings
- `safety:write` - Submit crash assessments and safety evaluations
- `crash:assess` - Access to real-time crash assessment APIs
- `certification:verify` - Verify safety certifications

### Rate Limiting

- Safety-critical APIs (airbag, collision): 1000 req/sec per client
- Informational APIs (status, rating): 100 req/sec per client
- Rate limit headers: `X-RateLimit-Limit`, `X-RateLimit-Remaining`, `X-RateLimit-Reset`

---

## Implementation Example

```typescript
import axios from 'axios';

class WIASafetyClient {
  private baseURL = 'https://api.wia-standards.org';
  private accessToken: string;

  async authenticate(clientId: string, clientSecret: string) {
    const response = await axios.post(`${this.baseURL}/oauth/token`, {
      grant_type: 'client_credentials',
      client_id: clientId,
      client_secret: clientSecret,
      scope: 'crash:assess'
    });
    this.accessToken = response.data.access_token;
  }

  async assessCrash(request: CrashAssessmentRequest): Promise<CrashAssessmentResponse> {
    const response = await axios.post(
      `${this.baseURL}/api/v1/crash-assessment`,
      request,
      {
        headers: {
          'Authorization': `Bearer ${this.accessToken}`,
          'Content-Type': 'application/json'
        }
      }
    );
    return response.data;
  }
}

// Usage
const client = new WIASafetyClient();
await client.authenticate(process.env.CLIENT_ID, process.env.CLIENT_SECRET);

const result = await client.assessCrash({
  vehicleMass: 1500,
  impactVelocity: 15.6,
  impactAngle: 0,
  crumpleZoneLength: 0.8,
  occupants: [{position: 'driver', mass: 75, age: 35, beltStatus: true}]
});

console.log(`Safety Rating: ${result.safetyRating} stars`);
console.log(`Driver HIC-15: ${result.occupantInjury.driver.hic15}`);
```

---

## Best Practices

1. **Error Handling**: Implement exponential backoff for retries
2. **Caching**: Cache GET responses with appropriate TTL
3. **Timeout**: Set reasonable timeouts (5s for most APIs, 1s for critical)
4. **Logging**: Log all API calls for audit and debugging
5. **Monitoring**: Track latency, error rates, and rate limit consumption

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA | MIT License

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
in lockstep across Phases 1–4 of vehicle-safety so that conformance claims at any
Phase remain unambiguous.*

