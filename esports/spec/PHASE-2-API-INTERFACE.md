# WIA-EDU-022: API Reference

> **弘益人間** (Benefit All Humanity)

## Base URL

```
Production: https://api.wia-esports.edu/v1
Sandbox: https://sandbox-api.wia-esports.edu/v1
```

## Authentication

All API requests require authentication using Bearer tokens:

```http
Authorization: Bearer <your_access_token>
```

### Obtaining Access Token

```http
POST /auth/token
Content-Type: application/json

{
  "grant_type": "client_credentials",
  "client_id": "your_client_id",
  "client_secret": "your_client_secret",
  "scope": "programs:read programs:write teams:read teams:write"
}
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "scope": "programs:read programs:write teams:read teams:write"
}
```

## Programs API

### List Programs

```http
GET /programs
```

**Query Parameters:**
- `institutionId` (optional): Filter by institution
- `status` (optional): Filter by status (active, planning, suspended, archived)
- `page` (optional): Page number (default: 1)
- `limit` (optional): Results per page (default: 20, max: 100)

**Response:**
```json
{
  "data": [
    {
      "id": "prog_123456",
      "standard": "WIA-EDU-022",
      "institution": {
        "id": "inst_789",
        "name": "Lincoln High School",
        "type": "high_school"
      },
      "program": {
        "name": "Varsity Esports Program",
        "type": "varsity",
        "status": "active",
        "games": ["League of Legends", "Rocket League"]
      }
    }
  ],
  "pagination": {
    "page": 1,
    "limit": 20,
    "total": 150,
    "totalPages": 8
  }
}
```

### Get Program

```http
GET /programs/{programId}
```

**Response:** Full program object (see Technical Specification)

### Create Program

```http
POST /programs
Content-Type: application/json

{
  "institution": {
    "id": "inst_789",
    "name": "Lincoln High School",
    "type": "high_school",
    "location": {
      "country": "US",
      "state": "CA",
      "city": "San Francisco"
    }
  },
  "program": {
    "name": "Varsity Esports Program",
    "type": "varsity",
    "status": "planning",
    "games": ["Rocket League"],
    "gradeLevels": {"min": 9, "max": 12},
    "learningObjectives": [
      "Develop teamwork and communication skills",
      "Build strategic thinking abilities"
    ]
  }
}
```

**Response:** 201 Created with program object

### Update Program

```http
PATCH /programs/{programId}
Content-Type: application/json

{
  "program": {
    "status": "active",
    "games": ["Rocket League", "Valorant"]
  }
}
```

**Response:** 200 OK with updated program object

### Delete Program

```http
DELETE /programs/{programId}
```

**Response:** 204 No Content

## Teams API

### List Teams

```http
GET /teams?programId={programId}
```

**Query Parameters:**
- `programId` (required): Program ID
- `game` (optional): Filter by game
- `tier` (optional): Filter by tier (varsity, jv, novice)
- `season` (optional): Filter by season year

**Response:**
```json
{
  "data": [
    {
      "id": "team_abc123",
      "programId": "prog_123456",
      "name": "Dragons Varsity",
      "game": "Rocket League",
      "tier": "varsity",
      "roster": {
        "starters": ["player_001", "player_002", "player_003"],
        "substitutes": ["player_004"]
      },
      "record": {
        "wins": 12,
        "losses": 5,
        "ties": 1
      }
    }
  ]
}
```

### Get Team

```http
GET /teams/{teamId}
```

### Create Team

```http
POST /teams
Content-Type: application/json

{
  "programId": "prog_123456",
  "name": "Dragons Varsity",
  "game": "Rocket League",
  "tier": "varsity",
  "season": {
    "year": 2025,
    "league": "PlayVS",
    "division": "California - Division 1"
  }
}
```

**Response:** 201 Created with team object

### Update Team Roster

```http
PATCH /teams/{teamId}/roster
Content-Type: application/json

{
  "starters": ["player_001", "player_002", "player_003"],
  "substitutes": ["player_004", "player_005"]
}
```

**Response:** 200 OK with updated team object

## Players API

### List Players

```http
GET /players?programId={programId}
```

**Query Parameters:**
- `programId` (required): Program ID
- `teamId` (optional): Filter by team
- `grade` (optional): Filter by grade level
- `status` (optional): Filter by status (active, inactive, graduated)

### Get Player

```http
GET /players/{playerId}
```

### Create Player

```http
POST /players
Content-Type: application/json

{
  "personal": {
    "studentId": "STU123456",
    "gamerTag": "ProGamer2025",
    "grade": 10,
    "enrollmentYear": 2023
  },
  "consent": {
    "parentalConsent": true,
    "dataSharing": true,
    "mediaRelease": false,
    "consentDate": "2025-01-15T00:00:00Z"
  }
}
```

**Response:** 201 Created with player object

### Update Player

```http
PATCH /players/{playerId}
Content-Type: application/json

{
  "performance": {
    "individualStats": {
      "gamesPlayed": 15,
      "winRate": 68.5,
      "skillRating": 1850
    }
  }
}
```

### Assign Player to Team

```http
POST /players/{playerId}/teams
Content-Type: application/json

{
  "teamId": "team_abc123",
  "role": "starter",
  "position": "midfielder",
  "joinedDate": "2025-01-20T00:00:00Z"
}
```

**Response:** 200 OK with updated player object

## Matches API

### List Matches

```http
GET /matches?teamId={teamId}
```

**Query Parameters:**
- `teamId` (required): Team ID
- `type` (optional): Filter by match type
- `startDate` (optional): Filter matches after date
- `endDate` (optional): Filter matches before date

### Get Match

```http
GET /matches/{matchId}
```

### Create Match

```http
POST /matches
Content-Type: application/json

{
  "type": "league",
  "teamId": "team_abc123",
  "opponent": {
    "name": "Riverside Warriors",
    "institution": "Riverside High School"
  },
  "schedule": {
    "date": "2025-02-15T18:00:00Z",
    "location": "online",
    "venue": "PlayVS Platform"
  },
  "roster": {
    "starters": ["player_001", "player_002", "player_003"]
  }
}
```

**Response:** 201 Created with match object

### Update Match Result

```http
PATCH /matches/{matchId}/result
Content-Type: application/json

{
  "score": {
    "team": 3,
    "opponent": 1
  },
  "outcome": "win",
  "duration": 45,
  "analysis": {
    "vod": "https://youtube.com/watch?v=...",
    "coachNotes": "Great teamwork and communication"
  }
}
```

**Response:** 200 OK with updated match object

## Career Pathways API

### Get Player Career Profile

```http
GET /players/{playerId}/career
```

### Update Career Interests

```http
PATCH /players/{playerId}/career/interests
Content-Type: application/json

{
  "interests": ["content_creator", "analyst", "coach"]
}
```

### Add Experience

```http
POST /players/{playerId}/career/experiences
Content-Type: application/json

{
  "type": "internship",
  "title": "Content Creator Intern",
  "organization": "Cloud9 Esports",
  "dateRange": {
    "start": "2025-06-01T00:00:00Z",
    "end": "2025-08-15T00:00:00Z"
  },
  "description": "Created social media content for competitive team",
  "skills": ["video editing", "social media marketing", "content strategy"]
}
```

### Add Achievement

```http
POST /players/{playerId}/career/achievements
Content-Type: application/json

{
  "type": "scholarship",
  "title": "UC Irvine Esports Scholarship",
  "issuer": "University of California, Irvine",
  "date": "2025-03-01T00:00:00Z",
  "amount": 15000
}
```

## Webhooks

Subscribe to real-time events:

```http
POST /webhooks
Content-Type: application/json

{
  "url": "https://your-app.com/webhooks/esports",
  "events": ["match.completed", "player.joined", "team.updated"],
  "secret": "your_webhook_secret"
}
```

**Event Payload:**
```json
{
  "event": "match.completed",
  "timestamp": "2025-02-15T20:30:00Z",
  "data": {
    "matchId": "match_xyz789",
    "teamId": "team_abc123",
    "result": "win",
    "score": {"team": 3, "opponent": 1}
  },
  "signature": "sha256=..."
}
```

## Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| UNAUTHORIZED | 401 | Invalid or expired authentication token |
| FORBIDDEN | 403 | Insufficient permissions for requested resource |
| NOT_FOUND | 404 | Resource does not exist |
| VALIDATION_ERROR | 422 | Request data validation failed |
| RATE_LIMIT_EXCEEDED | 429 | Too many requests, retry after specified time |
| INTERNAL_ERROR | 500 | Server error, contact support with requestId |

## SDK Examples

### TypeScript/JavaScript

```typescript
import { createClient } from '@wia/esports-sdk';

const client = createClient({
  apiKey: 'your_api_key',
  environment: 'production'
});

// List programs
const programs = await client.programs.list({
  status: 'active',
  limit: 50
});

// Create team
const team = await client.teams.create({
  programId: 'prog_123456',
  name: 'Dragons Varsity',
  game: 'Rocket League',
  tier: 'varsity'
});

// Update match result
await client.matches.updateResult('match_xyz789', {
  score: { team: 3, opponent: 1 },
  outcome: 'win'
});
```

### Python

```python
from wia_esports import Client

client = Client(api_key='your_api_key')

# List players
players = client.players.list(program_id='prog_123456', status='active')

# Add player experience
client.players.career.add_experience(
    player_id='player_001',
    experience={
        'type': 'workshop',
        'title': 'Esports Broadcasting Workshop',
        'organization': 'NASEF',
        'date_range': {'start': '2025-04-10', 'end': '2025-04-12'},
        'skills': ['commentary', 'obs studio', 'production']
    }
)
```

---

© 2025 WIA - World Certification Industry Association
Licensed under MIT License

**弘益人間** · Benefit All Humanity


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

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-2-API-INTERFACE.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-2-API-INTERFACE. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P2-API-INTERFACE-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.
