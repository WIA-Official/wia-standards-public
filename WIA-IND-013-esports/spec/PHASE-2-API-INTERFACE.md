# WIA-IND-013: E-Sports Platform Standard
## Phase 2: API Interface Specification
### 弘益人間 - Broadly Benefiting Humanity

**Version:** 1.0.0
**Status:** Final
**Last Updated:** 2025-01-15

---

## Table of Contents

1. [Introduction](#introduction)
2. [API Architecture](#api-architecture)
3. [Authentication & Authorization](#authentication--authorization)
4. [Core Endpoints](#core-endpoints)
5. [Rate Limiting](#rate-limiting)
6. [Error Handling](#error-handling)
7. [Versioning Strategy](#versioning-strategy)

---

## Introduction

Phase 2 defines RESTful API specifications for accessing and manipulating e-sports data. These APIs enable third-party applications, analytics platforms, and mobile apps to integrate with any WIA-IND-013 compliant system.

### Design Principles

- **RESTful:** Follow REST architectural constraints
- **Stateless:** No server-side session state
- **Cacheable:** HTTP caching for performance
- **Uniform Interface:** Consistent resource naming
- **Philosophy:** 弘益人間 through open, documented APIs

### Base URL Structure

```
https://api.{platform}.com/wia-ind-013/v1/{resource}
```

Example:
```
https://api.esports-platform.com/wia-ind-013/v1/matches
```

---

## API Architecture

### HTTP Methods

| Method | Purpose | Idempotent | Safe |
|--------|---------|-----------|------|
| GET | Retrieve resources | Yes | Yes |
| POST | Create resources | No | No |
| PUT | Update/replace resources | Yes | No |
| PATCH | Partial update | No | No |
| DELETE | Remove resources | Yes | No |

### Content Types

- **Request:** `application/json` or `application/x-www-form-urlencoded`
- **Response:** `application/json`
- **Charset:** UTF-8

### Standard Headers

```http
Content-Type: application/json
Accept: application/json
Authorization: Bearer {access_token}
X-WIA-Version: 1.0.0
X-Request-ID: {uuid}
```

---

## Authentication & Authorization

### OAuth 2.0 Implementation

WIA-IND-013 APIs use OAuth 2.0 for authentication:

#### Authorization Code Flow

```http
GET /oauth/authorize?
    response_type=code&
    client_id={client_id}&
    redirect_uri={redirect_uri}&
    scope=matches:read players:read&
    state={random_string}
```

#### Token Exchange

```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code&
code={authorization_code}&
redirect_uri={redirect_uri}&
client_id={client_id}&
client_secret={client_secret}
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "def50200abc...",
  "scope": "matches:read players:read"
}
```

### Scopes

| Scope | Description |
|-------|-------------|
| `matches:read` | Read match data |
| `matches:write` | Create/update matches |
| `players:read` | Read player profiles |
| `players:write` | Update player data |
| `tournaments:read` | Read tournament data |
| `tournaments:admin` | Manage tournaments |
| `stats:read` | Read statistics |
| `admin` | Full system access |

### API Keys (for server-to-server)

```http
GET /api/v1/matches
X-API-Key: wia_sk_live_abc123def456
```

---

## Core Endpoints

### Matches API

#### List Matches

```http
GET /v1/matches?game=league-of-legends&limit=20&offset=0
```

**Query Parameters:**
- `game` (optional): Filter by game title
- `tournament_id` (optional): Filter by tournament
- `team_id` (optional): Filter by team
- `player_id` (optional): Filter by player
- `date_from` (optional): ISO 8601 date
- `date_to` (optional): ISO 8601 date
- `status` (optional): scheduled, live, completed
- `limit` (default: 20, max: 100)
- `offset` (default: 0)

**Response:**
```json
{
  "data": [
    {
      "matchId": "match-2025-001",
      "game": "League of Legends",
      "timestamp": {
        "scheduled": "2025-01-15T18:00:00Z",
        "started": "2025-01-15T18:12:00Z"
      },
      "teams": [
        {"teamId": "team-t1", "name": "T1"},
        {"teamId": "team-geng", "name": "Gen.G"}
      ],
      "status": "live"
    }
  ],
  "pagination": {
    "total": 250,
    "limit": 20,
    "offset": 0,
    "hasMore": true
  }
}
```

#### Get Match Details

```http
GET /v1/matches/{matchId}
```

**Response:** Returns complete match object per Phase 1 schema

#### Create Match

```http
POST /v1/matches
Content-Type: application/json
Authorization: Bearer {token}

{
  "tournamentId": "tournament-wc-2025",
  "game": "League of Legends",
  "scheduledTime": "2025-11-15T18:00:00Z",
  "teams": [
    {"teamId": "team-t1"},
    {"teamId": "team-geng"}
  ]
}
```

**Response:** 201 Created with match object

#### Update Match Result

```http
PATCH /v1/matches/{matchId}
Content-Type: application/json

{
  "status": "completed",
  "result": {
    "winner": "team-t1",
    "score": "3-1"
  },
  "statistics": { ... }
}
```

---

### Players API

#### Get Player Profile

```http
GET /v1/players/{playerId}
```

**Response:**
```json
{
  "playerId": "player-faker",
  "username": "Faker",
  "region": "KR",
  "currentTeam": {
    "teamId": "team-t1",
    "name": "T1",
    "role": "mid"
  },
  "statistics": { ... },
  "rankings": { ... }
}
```

#### Search Players

```http
GET /v1/players/search?
    q=faker&
    game=league-of-legends&
    region=KR
```

#### Update Player Stats

```http
POST /v1/players/{playerId}/statistics
Content-Type: application/json

{
  "matchId": "match-001",
  "performance": {
    "kills": 8,
    "deaths": 2,
    "assists": 12
  }
}
```

---

### Tournaments API

#### List Tournaments

```http
GET /v1/tournaments?
    game=league-of-legends&
    status=upcoming&
    region=global
```

**Response:**
```json
{
  "data": [
    {
      "tournamentId": "tournament-wc-2025",
      "name": "World Championship 2025",
      "game": "League of Legends",
      "startDate": "2025-11-01",
      "endDate": "2025-11-19",
      "prizePool": 2500000,
      "participants": 16,
      "status": "registration_open"
    }
  ]
}
```

#### Get Tournament Bracket

```http
GET /v1/tournaments/{tournamentId}/bracket
```

#### Register Team

```http
POST /v1/tournaments/{tournamentId}/register
Content-Type: application/json

{
  "teamId": "team-t1",
  "roster": [
    {"playerId": "player-faker", "role": "mid"},
    {"playerId": "player-keria", "role": "support"}
  ]
}
```

---

### Statistics API

#### Get Player Statistics

```http
GET /v1/statistics/players/{playerId}?
    game=league-of-legends&
    period=season-2024-summer
```

#### Get Team Statistics

```http
GET /v1/statistics/teams/{teamId}?
    game=league-of-legends
```

#### Get Leaderboard

```http
GET /v1/leaderboards/{game}?
    metric=elo&
    region=KR&
    limit=100
```

**Response:**
```json
{
  "game": "League of Legends",
  "metric": "elo",
  "region": "KR",
  "data": [
    {
      "rank": 1,
      "playerId": "player-faker",
      "username": "Faker",
      "value": 2847,
      "change": "+12"
    }
  ]
}
```

---

## Rate Limiting

### Rate Limit Headers

```http
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 987
X-RateLimit-Reset: 1642694400
```

### Limits by Tier

| Tier | Requests/Hour | Burst |
|------|---------------|-------|
| Free | 1,000 | 50 |
| Developer | 10,000 | 200 |
| Professional | 100,000 | 1,000 |
| Enterprise | Custom | Custom |

### Rate Limit Exceeded

```http
HTTP/1.1 429 Too Many Requests
Content-Type: application/json
Retry-After: 3600

{
  "error": {
    "code": "rate_limit_exceeded",
    "message": "Rate limit exceeded. Try again in 3600 seconds.",
    "limit": 1000,
    "remaining": 0,
    "resetAt": "2025-01-15T19:00:00Z"
  }
}
```

---

## Error Handling

### Standard Error Response

```json
{
  "error": {
    "code": "resource_not_found",
    "message": "Match not found",
    "details": {
      "matchId": "match-invalid-id"
    },
    "requestId": "req_abc123",
    "timestamp": "2025-01-15T12:00:00Z"
  }
}
```

### HTTP Status Codes

| Code | Meaning | Usage |
|------|---------|-------|
| 200 | OK | Successful GET/PATCH |
| 201 | Created | Successful POST |
| 204 | No Content | Successful DELETE |
| 400 | Bad Request | Invalid parameters |
| 401 | Unauthorized | Missing/invalid auth |
| 403 | Forbidden | Insufficient permissions |
| 404 | Not Found | Resource doesn't exist |
| 409 | Conflict | Resource conflict |
| 422 | Unprocessable | Validation failed |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server error |
| 503 | Service Unavailable | Maintenance/overload |

### Common Error Codes

- `invalid_request`: Malformed request
- `unauthorized`: Authentication failed
- `forbidden`: Insufficient permissions
- `not_found`: Resource not found
- `validation_error`: Input validation failed
- `conflict`: Resource conflict
- `rate_limit_exceeded`: Too many requests
- `server_error`: Internal server error

---

## Versioning Strategy

### URL Versioning

```
https://api.platform.com/wia-ind-013/v1/matches
https://api.platform.com/wia-ind-013/v2/matches
```

### Header Versioning (alternative)

```http
GET /matches
Accept: application/vnd.wia.v1+json
```

### Deprecation Policy

1. New version announced 6 months before release
2. Old version supported for 12 months after new release
3. Deprecation warnings in response headers

```http
Deprecation: true
Sunset: Sat, 31 Dec 2025 23:59:59 GMT
Link: <https://api.platform.com/wia-ind-013/v2/matches>; rel="successor-version"
```

---

## Philosophy: 弘益人間

These APIs embody "Broadly Benefiting Humanity" through:

- **Open Documentation:** Freely available specifications
- **Consistent Standards:** Predictable, uniform interfaces
- **Developer-Friendly:** Clear errors, comprehensive docs
- **Fair Access:** Rate limits that enable innovation
- **Data Portability:** Easy export and migration

---

## Implementation Checklist

- [ ] OAuth 2.0 authentication
- [ ] API key management
- [ ] Rate limiting per tier
- [ ] Standard error responses
- [ ] Pagination for list endpoints
- [ ] Field filtering (?fields=id,name)
- [ ] Sorting (?sort=name,-created)
- [ ] CORS headers for web clients
- [ ] Request ID tracking
- [ ] Comprehensive logging

---

## References

- [OAuth 2.0 Specification](https://oauth.net/2/)
- [REST API Design Best Practices](https://restfulapi.net/)
- [HTTP Status Codes](https://httpstatuses.com/)
- [WIA-IND-013 Phase 3: Protocol](./PHASE-3-PROTOCOL.md)

---

**© 2025 SmileStory Inc. / WIA**
**弘益人間 (홍익인간) · Benefit All Humanity**

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-IND-013-esports is evaluated across three tiers:

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

- `wia-standards/standards/WIA-IND-013-esports/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-IND-013-esports/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-IND-013-esports/simulator/` — interactive browser-based simulator for the PHASE protocol

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
