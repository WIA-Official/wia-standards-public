# WIA-IND-013: E-Sports Platform Standard
## Phase 1: Data Format Specification
### 弘益人間 - Broadly Benefiting Humanity

**Version:** 1.0.0
**Status:** Final
**Last Updated:** 2025-01-15

---

## Table of Contents

1. [Introduction](#introduction)
2. [Match Data Schema](#match-data-schema)
3. [Player Profile Schema](#player-profile-schema)
4. [Tournament Bracket Schema](#tournament-bracket-schema)
5. [Team Structure Schema](#team-structure-schema)
6. [Performance Metrics Schema](#performance-metrics-schema)
7. [Implementation Guidelines](#implementation-guidelines)

---

## Introduction

Phase 1 of WIA-IND-013 defines standardized JSON schemas for all e-sports platform data. These schemas ensure interoperability across platforms, games, and services while maintaining flexibility for game-specific extensions.

### Design Principles

- **Simplicity:** Clear, understandable structures
- **Extensibility:** Support for game-specific fields
- **Validation:** JSON Schema validation for all documents
- **Versioning:** Backward-compatible schema evolution
- **Philosophy:** Embodying 弘益人間 through open, accessible formats

### Base Types

All schemas use these common field types:

```json
{
  "id": "string (UUID v4)",
  "timestamp": "string (ISO 8601)",
  "version": "string (semver)",
  "metadata": {
    "created": "ISO 8601 timestamp",
    "modified": "ISO 8601 timestamp",
    "author": "string (user ID)"
  }
}
```

---

## Match Data Schema

### Core Match Object

```json
{
  "$schema": "https://wia.org/schemas/match/v1.0.0",
  "matchId": "match-2025-wc-finals-g1",
  "tournamentId": "tournament-wc-2025",
  "game": {
    "title": "League of Legends",
    "version": "14.5.1",
    "mode": "5v5-summoners-rift"
  },
  "timestamp": {
    "scheduled": "2025-11-15T18:00:00Z",
    "started": "2025-11-15T18:12:34Z",
    "ended": "2025-11-15T18:47:21Z"
  },
  "teams": [
    {
      "teamId": "team-t1",
      "name": "T1",
      "side": "blue",
      "result": "victory",
      "score": 1
    },
    {
      "teamId": "team-geng",
      "name": "Gen.G",
      "side": "red",
      "result": "defeat",
      "score": 0
    }
  ],
  "players": [
    {
      "playerId": "player-faker",
      "teamId": "team-t1",
      "role": "mid",
      "champion": "Azir",
      "performance": {
        "kills": 8,
        "deaths": 2,
        "assists": 12,
        "cs": 342,
        "gold": 18500,
        "damage": 45230
      }
    }
  ],
  "events": [
    {
      "eventId": "event-001",
      "type": "first_blood",
      "timestamp": "2025-11-15T18:15:23Z",
      "playerId": "player-faker",
      "targetId": "player-chovy",
      "position": {"x": 1240, "y": 3450}
    }
  ],
  "objectives": [
    {
      "type": "dragon",
      "subtype": "cloud",
      "teamId": "team-t1",
      "timestamp": "2025-11-15T18:20:15Z"
    }
  ],
  "statistics": {
    "duration": 2087,
    "totalKills": 24,
    "totalGold": 67800,
    "towersDestroyed": 9,
    "dragonsKilled": 4,
    "baronsKilled": 1
  },
  "verification": {
    "hash": "sha256:a1b2c3d4...",
    "signature": "rsa:e5f6g7h8...",
    "verified": true
  }
}
```

### Match Result Types

- `victory`: Team won the match
- `defeat`: Team lost the match
- `draw`: Match ended in a tie
- `forfeit`: Team forfeited
- `disqualified`: Team was disqualified

### Event Types

Standard event types across all games:

- `first_blood`: First kill of the match
- `kill`: Player elimination
- `objective_taken`: Strategic objective captured
- `structure_destroyed`: Building/tower destroyed
- `item_purchased`: Player bought an item
- `ability_upgrade`: Skill/ability leveled up

---

## Player Profile Schema

### Core Player Profile

```json
{
  "$schema": "https://wia.org/schemas/player/v1.0.0",
  "playerId": "player-faker",
  "username": "Faker",
  "realName": {
    "given": "Sang-hyeok",
    "family": "Lee",
    "display": "Lee Sang-hyeok"
  },
  "region": "KR",
  "nationality": "KR",
  "birthdate": "1996-05-07",
  "currentTeam": {
    "teamId": "team-t1",
    "name": "T1",
    "joinedDate": "2013-02-21",
    "role": "mid"
  },
  "games": [
    {
      "title": "League of Legends",
      "primaryRole": "mid",
      "secondaryRoles": ["jungle"],
      "statistics": {
        "gamesPlayed": 1247,
        "wins": 789,
        "losses": 458,
        "winRate": 63.3,
        "avgKDA": 4.52
      },
      "rankings": {
        "elo": 2847,
        "mmr": 3201,
        "tier": "challenger",
        "division": 1,
        "lp": 1284,
        "globalRank": 12,
        "regionalRank": 3
      }
    }
  ],
  "achievements": [
    {
      "achievementId": "achievement-worlds-2023",
      "title": "World Champion 2023",
      "category": "tournament",
      "rarity": "legendary",
      "earnedDate": "2023-11-19",
      "verifiable": true,
      "credentialUrl": "https://wia.org/vc/achievement-worlds-2023"
    }
  ],
  "careerStats": {
    "tournamentWins": 47,
    "mvpAwards": 18,
    "totalPrizeMoney": 1250000,
    "yearsActive": 12
  },
  "socialMedia": {
    "twitter": "@Faker",
    "twitch": "faker",
    "youtube": "faker"
  },
  "privacy": {
    "profileVisibility": "public",
    "statsVisibility": "public",
    "contactVisibility": "team_only"
  }
}
```

### Privacy Levels

- `public`: Visible to everyone
- `friends`: Visible to confirmed friends
- `team_only`: Visible to current team members
- `private`: Hidden from all

---

## Tournament Bracket Schema

### Single Elimination Bracket

```json
{
  "$schema": "https://wia.org/schemas/tournament/v1.0.0",
  "tournamentId": "tournament-wc-2025",
  "name": "World Championship 2025",
  "game": "League of Legends",
  "organizerId": "org-riot-games",
  "format": {
    "type": "single_elimination",
    "bestOf": 5,
    "seeding": "standard",
    "thirdPlaceMatch": true
  },
  "schedule": {
    "startDate": "2025-11-01",
    "endDate": "2025-11-19",
    "timezone": "UTC"
  },
  "participants": {
    "totalSlots": 16,
    "registered": 16,
    "qualified": 16
  },
  "bracket": {
    "rounds": [
      {
        "roundId": "quarterfinals",
        "roundNumber": 1,
        "name": "Quarterfinals",
        "matches": [
          {
            "matchId": "match-qf1",
            "position": 1,
            "scheduledTime": "2025-11-15T14:00:00Z",
            "participants": [
              {"seed": 1, "teamId": "team-t1", "name": "T1"},
              {"seed": 16, "teamId": "team-lng", "name": "LNG"}
            ],
            "result": {
              "winner": "team-t1",
              "score": "3-0",
              "completed": true
            },
            "advancesTo": "match-sf1"
          }
        ]
      }
    ]
  },
  "rules": {
    "rosterLock": "2025-10-31T23:59:59Z",
    "substitutionLimit": 2,
    "pauseRules": "standard",
    "rematchConditions": ["technical_issue", "critical_bug"]
  },
  "prizePool": {
    "total": 2500000,
    "currency": "USD",
    "distribution": [
      {"place": 1, "amount": 500000},
      {"place": 2, "amount": 300000},
      {"place": 3, "amount": 200000}
    ]
  }
}
```

### Bracket Types

- `single_elimination`: Lose once and you're out
- `double_elimination`: Two-bracket system (upper/lower)
- `round_robin`: Everyone plays everyone
- `swiss`: Modified round-robin
- `group_stage`: Multiple groups with advancement

### Seeding Methods

- `standard`: 1v16, 2v15, etc.
- `random`: Random pairings
- `manual`: Custom seeding by organizers
- `performance_based`: Based on prior tournament results

---

## Team Structure Schema

### Team Organization

```json
{
  "$schema": "https://wia.org/schemas/team/v1.0.0",
  "teamId": "team-t1",
  "name": "T1",
  "abbreviation": "T1",
  "organizationId": "org-t1",
  "region": "KR",
  "founded": "2012-12-01",
  "games": [
    {
      "game": "League of Legends",
      "division": "main",
      "roster": {
        "active": [
          {
            "playerId": "player-faker",
            "role": "mid",
            "jerseyNumber": 3,
            "joinedDate": "2013-02-21",
            "contractStatus": "signed",
            "contractExpiry": "2025-12-31"
          }
        ],
        "substitutes": [
          {
            "playerId": "player-poby",
            "role": "mid",
            "joinedDate": "2024-01-01"
          }
        ],
        "coach": [
          {
            "staffId": "coach-bengi",
            "role": "head_coach",
            "joinedDate": "2023-01-01"
          }
        ]
      },
      "statistics": {
        "matchesPlayed": 89,
        "wins": 67,
        "losses": 22,
        "winRate": 75.3,
        "tournamentsWon": 5
      }
    }
  ],
  "sponsors": [
    {
      "sponsorId": "sponsor-nike",
      "name": "Nike",
      "category": "apparel",
      "since": "2020-01-01",
      "tier": "title"
    }
  ],
  "achievements": [
    {
      "year": 2023,
      "tournament": "World Championship",
      "placement": 1,
      "roster": ["player-faker", "player-keria", "player-zeus"]
    }
  ],
  "socialMedia": {
    "twitter": "@T1",
    "youtube": "T1",
    "website": "https://t1.gg"
  }
}
```

---

## Performance Metrics Schema

### Statistical Data

```json
{
  "$schema": "https://wia.org/schemas/metrics/v1.0.0",
  "playerId": "player-faker",
  "game": "League of Legends",
  "period": {
    "type": "season",
    "year": 2024,
    "split": "summer"
  },
  "aggregated": {
    "gamesPlayed": 45,
    "wins": 32,
    "losses": 13,
    "winRate": 71.1,
    "kda": {
      "kills": 234,
      "deaths": 56,
      "assists": 312,
      "ratio": 9.75,
      "avgPerGame": {
        "kills": 5.2,
        "deaths": 1.2,
        "assists": 6.9
      }
    },
    "economyStats": {
      "avgGold": 12450,
      "avgCS": 285,
      "goldShare": 24.5,
      "csPerMin": 8.9
    },
    "championPool": [
      {
        "champion": "Azir",
        "games": 12,
        "wins": 10,
        "winRate": 83.3,
        "avgKDA": 11.2
      }
    ]
  },
  "trends": {
    "winRateTrend": "increasing",
    "performanceConsistency": 0.87,
    "peakPerformanceGames": 8
  },
  "rankings": {
    "roleRank": 1,
    "regionalRank": 3,
    "globalRank": 12,
    "percentile": 99.8
  },
  "comparisons": {
    "vsRegionalAverage": {
      "kda": "+245%",
      "winRate": "+18.5%",
      "goldShare": "+3.2%"
    }
  }
}
```

---

## Implementation Guidelines

### JSON Schema Validation

All documents MUST validate against their respective JSON schemas. Implementations SHOULD:

1. Validate all incoming data against schemas
2. Reject invalid documents with clear error messages
3. Log validation failures for debugging
4. Support schema version negotiation

### Extensibility

Games may add custom fields under the `extensions` object:

```json
{
  "matchId": "match-001",
  "game": "Valorant",
  "extensions": {
    "valorant": {
      "agentComposition": ["Jett", "Sage", "Cypher", "Sova", "Brimstone"],
      "mapVotes": ["Haven", "Split"],
      "attackersStartSide": "team-blue"
    }
  }
}
```

### Data Integrity

1. **Cryptographic Hashing:** All match results include SHA-256 hashes
2. **Digital Signatures:** Tournament-verified matches use RSA signatures
3. **Timestamps:** All events use ISO 8601 format in UTC
4. **IDs:** Use UUID v4 for uniqueness

### Philosophy: 弘益人間

These data formats embody "Broadly Benefiting Humanity" by:

- Being freely available without licensing fees
- Supporting multiple platforms and games equally
- Prioritizing player data ownership
- Enabling transparent, verifiable competition
- Ensuring accessibility through standard formats

---

## References

- [JSON Schema Specification](https://json-schema.org/)
- [ISO 8601 Datetime Format](https://www.iso.org/iso-8601-date-and-time-format.html)
- [UUID v4 Specification](https://tools.ietf.org/html/rfc4122)
- [WIA-IND-013 Phase 2: API Interface](./PHASE-2-API-INTERFACE.md)

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
