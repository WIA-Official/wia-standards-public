# WIA-AUTO-025: PHASE 2 - API Specification

**Version:** 1.0
**Status:** Complete
**Last Updated:** 2025-12-27

---

## Overview

This document defines PHASE 2 of the WIA-AUTO-025 Mobility-as-a-Service (MaaS) standard, focusing on RESTful endpoints, authentication, rate limiting. This specification ensures interoperability across different MaaS platforms and service providers while maintaining flexibility for innovation.

### Purpose

The primary purpose of this phase is to establish standardized approaches for RESTful endpoints, authentication, rate limiting within MaaS ecosystems. By adhering to this specification, implementers can build systems that seamlessly integrate with the broader mobility services landscape.

### Scope

This phase covers:
- Technical requirements for RESTful endpoints, authentication, rate limiting
- Data structures and schemas
- API contracts and interfaces  
- Security and privacy considerations
- Performance and scalability guidelines
- Implementation best practices
- Testing and validation procedures

---

## Core Principles

### 1. Interoperability

All implementations MUST support the standard interfaces defined in this specification to ensure seamless communication between different MaaS platforms and transport providers. This includes:

- Standardized data formats (JSON Schema v7+)
- Common API endpoints following RESTful principles
- Consistent error handling and status codes
- Version negotiation mechanisms

### 2. Security

Security is paramount in MaaS systems handling sensitive user data and financial transactions. Required security measures include:

- TLS 1.3 for all network communications
- OAuth 2.0 with PKCE for authentication
- JWT tokens for API authorization
- Data encryption at rest (AES-256)
- Field-level encryption for PII
- Regular security audits and penetration testing

### 3. Privacy

Privacy by design principles MUST be incorporated throughout implementation:

- Data minimization - collect only necessary information
- Purpose limitation - use data only for stated purposes
- Transparent data policies and user consent
- Right to erasure and data portability
- Anonymization of analytics data
- GDPR, CCPA, and local privacy regulation compliance

### 4. Scalability

Systems MUST be designed to scale from pilot deployments (thousands of users) to metropolitan-scale operations (millions of users):

- Horizontal scaling through stateless microservices
- Database sharding and read replicas
- Caching strategies (multi-tier: browser, CDN, application, database)
- Async processing for long-running operations
- Geographic distribution for global deployments

---

## Technical Specifications

### Data Models

All data exchanged between MaaS components MUST conform to the following schemas:

#### Journey Request Schema

```json
{
  "type": "object",
  "required": ["origin", "destination", "departure_time"],
  "properties": {
    "journey_id": {
      "type": "string",
      "pattern": "^j_[a-zA-Z0-9]{16}$"
    },
    "user_id": {
      "type": "string",
      "pattern": "^u_[a-zA-Z0-9]{16}$"
    },
    "origin": {
      "type": "object",
      "required": ["coordinates"],
      "properties": {
        "name": {"type": "string"},
        "coordinates": {
          "type": "object",
          "required": ["lat", "lon"],
          "properties": {
            "lat": {"type": "number", "minimum": -90, "maximum": 90},
            "lon": {"type": "number", "minimum": -180, "maximum": 180}
          }
        },
        "type": {"type": "string", "enum": ["address", "poi", "transit_stop"]}
      }
    },
    "destination": {
      "type": "object",
      "required": ["coordinates"],
      "properties": {
        "name": {"type": "string"},
        "coordinates": {
          "type": "object",
          "required": ["lat", "lon"],
          "properties": {
            "lat": {"type": "number", "minimum": -90, "maximum": 90},
            "lon": {"type": "number", "minimum": -180, "maximum": 180}
          }
        },
        "type": {"type": "string", "enum": ["address", "poi", "transit_stop"]}
      }
    },
    "departure_time": {
      "type": "string",
      "format": "date-time"
    },
    "preferences": {
      "type": "object",
      "properties": {
        "optimize_for": {"type": "string", "enum": ["time", "cost", "eco", "comfort"]},
        "allowed_modes": {
          "type": "array",
          "items": {"type": "string", "enum": ["walk", "bike", "bus", "subway", "taxi", "car_share", "scooter"]}
        },
        "max_walking_distance_km": {"type": "number", "minimum": 0, "maximum": 10},
        "max_transfers": {"type": "integer", "minimum": 0, "maximum": 5},
        "accessibility_required": {"type": "boolean"}
      }
    }
  }
}
```

#### Journey Response Schema

```json
{
  "type": "object",
  "required": ["journey_id", "routes"],
  "properties": {
    "journey_id": {"type": "string"},
    "routes": {
      "type": "array",
      "minItems": 1,
      "items": {
        "type": "object",
        "required": ["route_id", "segments", "total_duration_minutes", "total_cost"],
        "properties": {
          "route_id": {"type": "string"},
          "segments": {
            "type": "array",
            "items": {
              "type": "object",
              "required": ["mode", "from", "to", "duration_minutes"],
              "properties": {
                "mode": {"type": "string"},
                "provider": {"type": "string"},
                "line": {"type": "string"},
                "from": {"type": "string"},
                "to": {"type": "string"},
                "departure_time": {"type": "string", "format": "date-time"},
                "arrival_time": {"type": "string", "format": "date-time"},
                "duration_minutes": {"type": "number"},
                "distance_km": {"type": "number"},
                "cost": {"type": "number"},
                "currency": {"type": "string", "pattern": "^[A-Z]{3}$"}
              }
            }
          },
          "total_duration_minutes": {"type": "number"},
          "total_cost": {"type": "number"},
          "total_distance_km": {"type": "number"},
          "carbon_saved_kg": {"type": "number"},
          "comfort_score": {"type": "number", "minimum": 0, "maximum": 100},
          "accessibility_compatible": {"type": "boolean"}
        }
      }
    },
    "metadata": {
      "type": "object",
      "properties": {
        "request_time": {"type": "string", "format": "date-time"},
        "computation_time_ms": {"type": "number"},
        "api_version": {"type": "string"}
      }
    }
  }
}
```

### API Endpoints

All MaaS platforms MUST implement the following RESTful API endpoints:

#### 1. Journey Planning

```
POST /api/v1/journeys/plan
Content-Type: application/json
Authorization: Bearer {jwt_token}

Request: Journey Request Schema (see above)
Response: Journey Response Schema (see above)

Rate Limit: 100 requests per minute per user
Timeout: 5 seconds maximum
```

#### 2. Booking Management

```
POST /api/v1/bookings
PUT /api/v1/bookings/{booking_id}
GET /api/v1/bookings/{booking_id}
DELETE /api/v1/bookings/{booking_id}

Authorization: Bearer {jwt_token}
Rate Limit: 50 requests per minute per user
```

#### 3. Real-Time Updates

```
GET /api/v1/real-time/vehicles/{mode}/{provider}
GET /api/v1/real-time/disruptions

Authorization: Bearer {jwt_token}
Cache-Control: max-age=30
Rate Limit: 200 requests per minute per user
```

### Error Handling

All API responses MUST use standard HTTP status codes and include detailed error information:

```json
{
  "error": {
    "code": "INVALID_REQUEST",
    "message": "Origin coordinates are required",
    "details": {
      "field": "origin.coordinates",
      "constraint": "required"
    },
    "request_id": "req_abc123xyz",
    "timestamp": "2025-12-27T10:30:00Z"
  }
}
```

Standard error codes:
- `INVALID_REQUEST` - Malformed request data
- `AUTHENTICATION_FAILED` - Invalid or expired credentials
- `AUTHORIZATION_DENIED` - Insufficient permissions
- `RESOURCE_NOT_FOUND` - Requested resource doesn't exist
- `RATE_LIMIT_EXCEEDED` - Too many requests
- `INTERNAL_ERROR` - Server-side error
- `SERVICE_UNAVAILABLE` - Temporary outage or maintenance

---

## Implementation Guidelines

### Performance Requirements

Implementations MUST meet the following performance targets:

| Metric | Target | Critical Threshold |
|--------|--------|-------------------|
| API Response Time (p95) | < 200ms | > 1000ms |
| Journey Planning Time | < 2s | > 5s |
| System Availability | > 99.9% | < 99.0% |
| Error Rate | < 0.1% | > 1.0% |
| Concurrent Users | 10,000+ | N/A |

### Testing Requirements

All implementations MUST include:

1. **Unit Tests**: Minimum 80% code coverage
2. **Integration Tests**: All API endpoints with success and error cases
3. **Load Tests**: Sustained load of 1000 requests/second
4. **Security Tests**: OWASP Top 10 vulnerability scanning
5. **Accessibility Tests**: WCAG 2.1 AA compliance

### Documentation Requirements

Complete documentation MUST be provided including:

- API reference with OpenAPI 3.0 specification
- Data schema documentation with examples
- Integration guides for providers
- SDK documentation for supported languages
- Troubleshooting and FAQ sections

---

## Security Considerations

### Authentication

OAuth 2.0 with PKCE (Proof Key for Code Exchange) MUST be used for user authentication:

```
# Authorization request
GET /oauth/authorize
  ?response_type=code
  &client_id={client_id}
  &redirect_uri={redirect_uri}
  &scope=journeys.read+journeys.write+bookings.manage
  &state={random_state}
  &code_challenge={code_challenge}
  &code_challenge_method=S256

# Token request
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code
&code={authorization_code}
&redirect_uri={redirect_uri}
&client_id={client_id}
&code_verifier={code_verifier}
```

### Authorization

JWT tokens MUST include appropriate scopes:

```json
{
  "sub": "u_abc123",
  "aud": "maas-platform",
  "iat": 1703750400,
  "exp": 1703754000,
  "scopes": ["journeys.read", "journeys.write", "bookings.manage"]
}
```

### Data Protection

All sensitive data MUST be encrypted:

- In transit: TLS 1.3
- At rest: AES-256
- Field-level: For PII (name, email, phone, payment info)

---

## Compliance and Certification

Organizations implementing this specification can pursue WIA-AUTO-025 PHASE 2 certification through:

1. **Self-Assessment**: Complete the compliance checklist
2. **Technical Review**: Submit implementation for code review
3. **Testing**: Pass the standard test suite
4. **Documentation Review**: Provide complete API and integration docs
5. **Security Audit**: Third-party security assessment

Certified implementations receive:
- WIA-AUTO-025 PHASE 2 certification badge
- Listing in the official MaaS provider directory
- Access to reference implementations and support resources

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-12-27 | Initial release of PHASE 2 specification |

---

## References

- [WIA Standards Homepage](https://wiastandards.com)
- [WIA-AUTO-025 GitHub Repository](https://github.com/WIA-Official/wia-standards/tree/main/standards/maas)
- [OpenAPI 3.0 Specification](https://spec.openapis.org/oas/v3.0.0)
- [JSON Schema Documentation](https://json-schema.org/)
- [OAuth 2.0 RFC 6749](https://tools.ietf.org/html/rfc6749)
- [JWT RFC 7519](https://tools.ietf.org/html/rfc7519)

---

## Contact

For questions, clarifications, or feedback on this specification:

- **Email**: standards@wiastandards.com
- **GitHub Issues**: [wia-standards/issues](https://github.com/WIA-Official/wia-standards/issues)
- **Discussion Forum**: [WIA Community](https://community.wiastandards.com)

---

**弘益人間 · Benefit All Humanity**

© 2025 WIA - World Certification Industry Association  
Licensed under MIT License

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-AUTO-025 (Mobility-as-a-Service) is evaluated across three tiers, applied to trip planning · multimodal routing · ticketing · service-level monitoring:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | None (annual self-review recommended) |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST clearly disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references the following published standards. Implementers SHOULD review the listed standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 19086-1:2016 — Cloud SLA framework (referenced for journey-planning SLAs)
- ISO 39001:2012 — Road traffic safety management systems
- IETF RFC 7946 — GeoJSON (location payload encoding)
- IETF RFC 8040 — RESTCONF (configuration plane reference)
- OASIS OData v4 — query semantics for booking and inventory endpoints

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/maas/api/` — TypeScript SDK skeleton
- `wia-standards/standards/maas/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/maas/simulator/` — interactive browser-based simulator for the PHASE protocol

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
