# WIA ROB-010 Delivery Robot Standard - Phase 2: API Interface

> **Version**: 1.0.0  
> **Status**: Stable  
> **Last Updated**: 2025-12-26

---

## 1. Overview

Phase 2 defines the RESTful API interface for controlling delivery robots, managing packages, planning routes, and coordinating fleets. This specification ensures consistent, secure, and scalable communication between client applications and robot systems.

### 1.1 API Design Principles

- **RESTful**: Standard HTTP methods and status codes  
- **Stateless**: Each request contains all necessary information  
- **Versioned**: API version in URL path  
- **Secure**: OAuth 2.0 authentication and HTTPS encryption  
- **Documented**: OpenAPI 3.0 specification provided  

---

## 2. Authentication

### 2.1 OAuth 2.0

All API requests require authentication using OAuth 2.0:

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=client_credentials&
client_id=YOUR_CLIENT_ID&
client_secret=YOUR_CLIENT_SECRET&
scope=robot:read robot:control package:write
```

Response:
```json
{
    "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
    "token_type": "Bearer",
    "expires_in": 3600,
    "scope": "robot:read robot:control package:write"
}
```

### 2.2 Scopes

- `robot:read` - Read robot state and telemetry  
- `robot:control` - Send control commands to robots  
- `package:read` - View package information  
- `package:write` - Create and manage packages  
- `fleet:admin` - Fleet management and configuration  

---

## 3. Robot Control API

### 3.1 Get Robot State

```http
GET /api/v1/robots/{robot_id}
Authorization: Bearer {access_token}
```

Response (200 OK):
```json
{
    "robot_id": "550e8400-e29b-41d4-a716-446655440000",
    "timestamp": "2025-12-26T10:30:15Z",
    "version": "1.0.0",
    "position": {
        "latitude": 37.5665,
        "longitude": 126.9780,
        "accuracy": 2.5,
        "source": "gps"
    },
    "heading": 45.0,
    "speed": 2.5,
    "battery_level": 87.0,
    "mode": "autonomous",
    "cargo": {
        "occupied_slots": 3,
        "total_slots": 5
    }
}
```

### 3.2 Start Autonomous Navigation

```http
POST /api/v1/robots/{robot_id}/commands/start
Authorization: Bearer {access_token}
Content-Type: application/json

{
    "destination": {
        "latitude": 37.5700,
        "longitude": 126.9850
    },
    "route_preference": "fastest",
    "max_speed_ms": 2.5
}
```

Response (202 Accepted):
```json
{
    "command_id": "cmd-2025-12-26-001",
    "status": "queued",
    "estimated_arrival": "2025-12-26T10:45:00Z"
}
```

### 3.3 Emergency Stop

```http
POST /api/v1/robots/{robot_id}/commands/emergency_stop
Authorization: Bearer {access_token}
```

Response (200 OK):
```json
{
    "command_id": "cmd-2025-12-26-002",
    "status": "executed",
    "stopped_at": "2025-12-26T10:31:00Z"
}
```

---

## 4. Package Management API

### 4.1 Create Package

```http
POST /api/v1/packages
Authorization: Bearer {access_token}
Content-Type: application/json

{
    "recipient": {
        "name": "John Doe",
        "phone": "+82-10-1234-5678",
        "address": "123 Gangnam-daero, Seoul"
    },
    "pickup_location": {
        "latitude": 37.5665,
        "longitude": 126.9780
    },
    "dropoff_location": {
        "latitude": 37.5700,
        "longitude": 126.9850
    },
    "package_details": {
        "weight_kg": 2.5,
        "category": "food",
        "temperature_controlled": true
    },
    "access_control": {
        "access_code": "1234"
    },
    "priority": "express"
}
```

Response (201 Created):
```json
{
    "package_id": "pkg-2025-12-26-0001",
    "tracking_number": "WIA1234567890",
    "status": "pending",
    "estimated_delivery": "2025-12-26T11:00:00Z",
    "created_at": "2025-12-26T10:30:00Z"
}
```

### 4.2 Get Package Status

```http
GET /api/v1/packages/{package_id}
Authorization: Bearer {access_token}
```

Response (200 OK):
```json
{
    "package_id": "pkg-2025-12-26-0001",
    "status": "in_transit",
    "assigned_robot": "550e8400-e29b-41d4-a716-446655440000",
    "current_location": {
        "latitude": 37.5680,
        "longitude": 126.9810
    },
    "estimated_delivery": "2025-12-26T10:55:00Z",
    "tracking_events": [
        {
            "timestamp": "2025-12-26T10:30:00Z",
            "event": "created"
        },
        {
            "timestamp": "2025-12-26T10:35:00Z",
            "event": "assigned_to_robot"
        },
        {
            "timestamp": "2025-12-26T10:40:00Z",
            "event": "picked_up"
        }
    ]
}
```

---

## 5. Route Planning API

### 5.1 Calculate Route

```http
POST /api/v1/routes/calculate
Authorization: Bearer {access_token}
Content-Type: application/json

{
    "start": {
        "latitude": 37.5665,
        "longitude": 126.9780
    },
    "destination": {
        "latitude": 37.5700,
        "longitude": 126.9850
    },
    "route_preference": "fastest",
    "constraints": {
        "sidewalk_only": true,
        "avoid_roads": true
    }
}
```

Response (200 OK):
```json
{
    "route_id": "route-2025-12-26-001",
    "total_distance_meters": 2300,
    "estimated_duration_seconds": 720,
    "battery_usage_percent": 8,
    "waypoints": [
        {
            "position": {"latitude": 37.5665, "longitude": 126.9780},
            "waypoint_type": "start"
        },
        {
            "position": {"latitude": 37.5680, "longitude": 126.9810},
            "waypoint_type": "intermediate"
        },
        {
            "position": {"latitude": 37.5700, "longitude": 126.9850},
            "waypoint_type": "destination"
        }
    ]
}
```

---

## 6. Fleet Management API

### 6.1 List Robots

```http
GET /api/v1/fleet/robots?status=active&zone=gangnam
Authorization: Bearer {access_token}
```

Response (200 OK):
```json
{
    "total_count": 24,
    "page": 1,
    "per_page": 20,
    "robots": [
        {
            "robot_id": "550e8400-e29b-41d4-a716-446655440000",
            "status": "active",
            "battery_level": 87,
            "packages": 3,
            "location": {"latitude": 37.5665, "longitude": 126.9780}
        }
    ]
}
```

### 6.2 Fleet Statistics

```http
GET /api/v1/fleet/statistics
Authorization: Bearer {access_token}
```

Response (200 OK):
```json
{
    "timestamp": "2025-12-26T10:30:00Z",
    "total_robots": 30,
    "active": 24,
    "idle": 4,
    "charging": 2,
    "deliveries_today": 128,
    "success_rate": 98.7,
    "average_delivery_time_minutes": 14
}
```

---

## 7. Error Handling

### 7.1 Error Response Format

```json
{
    "error": {
        "code": "ROBOT_NOT_FOUND",
        "message": "Robot with ID 550e8400-e29b-41d4-a716-446655440000 not found",
        "details": {
            "robot_id": "550e8400-e29b-41d4-a716-446655440000"
        },
        "timestamp": "2025-12-26T10:30:00Z",
        "request_id": "req-2025-12-26-001"
    }
}
```

### 7.2 HTTP Status Codes

- `200 OK` - Request successful  
- `201 Created` - Resource created successfully  
- `202 Accepted` - Request accepted, processing asynchronously  
- `400 Bad Request` - Invalid request parameters  
- `401 Unauthorized` - Authentication required  
- `403 Forbidden` - Insufficient permissions  
- `404 Not Found` - Resource not found  
- `429 Too Many Requests` - Rate limit exceeded  
- `500 Internal Server Error` - Server error  

---

## 8. Rate Limiting

API requests are rate-limited to ensure fair usage:

- **Default**: 1000 requests/hour per API key  
- **Headers**: `X-RateLimit-Limit`, `X-RateLimit-Remaining`, `X-RateLimit-Reset`  

When exceeded:
```json
{
    "error": {
        "code": "RATE_LIMIT_EXCEEDED",
        "message": "API rate limit exceeded. Retry after 2025-12-26T11:00:00Z",
        "retry_after": "2025-12-26T11:00:00Z"
    }
}
```

---

**Copyright 2025 WIA / SmileStory Inc.**  
**License**: MIT


## Annex E — Implementation Notes for PHASE-2-API-INTERFACE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API-INTERFACE.

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

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-2-API-INTERFACE. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-2-api-interface/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-2-API-INTERFACE with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-2-API-INTERFACE does not require bespoke
auditor tooling.
