# WIA-EDU-022: Technical Specification

> **弘益人間** (Benefit All Humanity)

## Data Models

### Program Model

```json
{
  "id": "string (UUID)",
  "standard": "WIA-EDU-022",
  "version": "1.0.0",
  "institution": {
    "id": "string",
    "name": "string",
    "type": "middle_school | high_school | college",
    "location": {
      "country": "string (ISO 3166-1)",
      "state": "string",
      "city": "string"
    }
  },
  "program": {
    "name": "string",
    "type": "club | class | varsity | hybrid",
    "status": "planning | active | suspended | archived",
    "startDate": "string (ISO 8601)",
    "games": ["string"],
    "gradeLevels": {
      "min": "number (6-16)",
      "max": "number (6-16)"
    },
    "learningObjectives": ["string"],
    "codeOfConduct": "string (URL to document)"
  },
  "staff": [
    {
      "id": "string",
      "role": "director | head_coach | assistant_coach | advisor",
      "name": "string",
      "contact": "string (email)"
    }
  ],
  "metadata": {
    "created": "string (ISO 8601)",
    "updated": "string (ISO 8601)",
    "certificationLevel": "bronze | silver | gold | platinum | null"
  }
}
```

### Team Model

```json
{
  "id": "string (UUID)",
  "programId": "string (UUID, ref to Program)",
  "name": "string",
  "game": "string",
  "tier": "varsity | jv | novice | practice",
  "roster": {
    "starters": ["string (player IDs)"],
    "substitutes": ["string (player IDs)"],
    "coaches": ["string (staff IDs)"]
  },
  "season": {
    "year": "number",
    "league": "string",
    "division": "string"
  },
  "record": {
    "wins": "number",
    "losses": "number",
    "ties": "number",
    "tournamentPlacements": [
      {
        "tournament": "string",
        "placement": "number",
        "date": "string (ISO 8601)"
      }
    ]
  },
  "practice": {
    "weeklyHours": "number",
    "schedule": [
      {
        "day": "monday | tuesday | ...",
        "startTime": "string (HH:MM)",
        "duration": "number (minutes)"
      }
    ]
  },
  "metadata": {
    "created": "string (ISO 8601)",
    "updated": "string (ISO 8601)"
  }
}
```

### Player Model

```json
{
  "id": "string (UUID)",
  "standard": "WIA-EDU-022",
  "personal": {
    "studentId": "string (institutional ID)",
    "gamerTag": "string",
    "grade": "number (6-16)",
    "enrollmentYear": "number"
  },
  "teamAssignments": [
    {
      "teamId": "string (UUID)",
      "role": "starter | substitute | captain | practice",
      "position": "string (game-specific role)",
      "joinedDate": "string (ISO 8601)",
      "status": "active | inactive | graduated"
    }
  ],
  "eligibility": {
    "academicStanding": "boolean",
    "conductStanding": "boolean",
    "attendanceRequirement": "boolean",
    "lastVerified": "string (ISO 8601)"
  },
  "performance": {
    "individualStats": {
      "gamesPlayed": "number",
      "winRate": "number (0-100)",
      "skillRating": "number",
      "improvementRate": "number"
    },
    "teamContributions": {
      "communicationRating": "number (1-10)",
      "teamworkRating": "number (1-10)",
      "leadershipRating": "number (1-10)"
    }
  },
  "wellness": {
    "screenTimeWeekly": "number (hours)",
    "physicalActivityWeekly": "number (hours)",
    "sleepAverageHours": "number",
    "lastWellnessCheck": "string (ISO 8601)"
  },
  "consent": {
    "parentalConsent": "boolean",
    "dataSharing": "boolean",
    "mediaRelease": "boolean",
    "consentDate": "string (ISO 8601)"
  },
  "metadata": {
    "created": "string (ISO 8601)",
    "updated": "string (ISO 8601)"
  }
}
```

### Match Model

```json
{
  "id": "string (UUID)",
  "type": "scrimmage | league | tournament | championship",
  "teamId": "string (UUID)",
  "opponent": {
    "teamId": "string (UUID) | null",
    "name": "string",
    "institution": "string"
  },
  "schedule": {
    "date": "string (ISO 8601)",
    "location": "home | away | neutral | online",
    "venue": "string"
  },
  "roster": {
    "starters": ["string (player IDs)"],
    "substitutes": ["string (player IDs)"]
  },
  "result": {
    "score": {
      "team": "number",
      "opponent": "number"
    },
    "outcome": "win | loss | tie | cancelled",
    "duration": "number (minutes)",
    "forfeit": "boolean"
  },
  "analysis": {
    "vod": "string (URL)",
    "stats": "object (game-specific stats)",
    "coachNotes": "string",
    "playerReflections": ["string"]
  },
  "metadata": {
    "created": "string (ISO 8601)",
    "updated": "string (ISO 8601)"
  }
}
```

### Career Pathway Model

```json
{
  "id": "string (UUID)",
  "playerId": "string (UUID)",
  "interests": ["player | coach | content_creator | analyst | ..."],
  "experiences": [
    {
      "type": "competition | internship | workshop | mentorship",
      "title": "string",
      "organization": "string",
      "dateRange": {
        "start": "string (ISO 8601)",
        "end": "string (ISO 8601) | null"
      },
      "description": "string",
      "skills": ["string"]
    }
  ],
  "achievements": [
    {
      "type": "award | certification | scholarship | recognition",
      "title": "string",
      "issuer": "string",
      "date": "string (ISO 8601)",
      "verificationUrl": "string (URL) | null"
    }
  ],
  "goals": [
    {
      "category": "short_term | medium_term | long_term",
      "description": "string",
      "targetDate": "string (ISO 8601) | null",
      "status": "pending | in_progress | achieved | revised",
      "milestones": ["string"]
    }
  ],
  "portfolio": {
    "streamingChannels": ["string (URLs)"],
    "contentSamples": ["string (URLs)"],
    "resume": "string (URL)",
    "personalWebsite": "string (URL) | null"
  },
  "metadata": {
    "created": "string (ISO 8601)",
    "updated": "string (ISO 8601)"
  }
}
```

## Data Privacy & Security

### Personal Information Protection

1. **PII Handling**
   - Student names, addresses, contact information must be encrypted at rest
   - Transmission over HTTPS/TLS 1.3 only
   - Access restricted to authorized personnel only
   - Audit logging for all PII access

2. **Data Minimization**
   - Collect only data necessary for program operation
   - Anonymous identifiers used where possible
   - Retention policies: delete data after graduation + 2 years

3. **Parental Consent**
   - Required for students under 18
   - Granular consent for data sharing, media release, research
   - Revocable at any time with 30-day grace period

4. **Compliance**
   - FERPA (Family Educational Rights and Privacy Act)
   - COPPA (Children's Online Privacy Protection Act)
   - GDPR (General Data Protection Regulation) where applicable
   - State-specific privacy laws (CCPA, etc.)

### Access Control

```json
{
  "roles": [
    {
      "role": "student_player",
      "permissions": ["read:own_profile", "update:own_preferences"]
    },
    {
      "role": "parent_guardian",
      "permissions": ["read:own_student", "update:consent"]
    },
    {
      "role": "coach",
      "permissions": ["read:team_players", "update:team_roster", "create:matches"]
    },
    {
      "role": "program_director",
      "permissions": ["read:all_program_data", "update:program", "delete:archived_data"]
    },
    {
      "role": "system_admin",
      "permissions": ["*"]
    }
  ]
}
```

### Data Retention

| Data Type | Retention Period | After Retention |
|-----------|------------------|-----------------|
| Student records | Graduation + 2 years | Deleted or anonymized |
| Match statistics | Indefinite (anonymized) | Archived |
| Practice logs | Current year + 1 year | Deleted |
| Consent forms | Until revoked + 7 years | Securely archived |
| Health/wellness data | Current year only | Deleted |

## Integration Standards

### Authentication

- OAuth 2.0 for third-party integrations
- SAML 2.0 for institutional SSO
- API keys for server-to-server communication
- JWT tokens for session management

### API Versioning

- Semantic versioning (MAJOR.MINOR.PATCH)
- API version in URL: `/api/v1/programs`
- Backward compatibility maintained for MINOR versions
- Deprecation notices 6 months before MAJOR changes

### Rate Limiting

- 1000 requests per hour per API key
- 100 requests per minute for burst traffic
- 429 Too Many Requests response with Retry-After header
- Higher limits available for certified partners

### Error Responses

```json
{
  "error": {
    "code": "string (ERROR_CODE)",
    "message": "string (human-readable)",
    "details": "object (additional context)",
    "timestamp": "string (ISO 8601)",
    "requestId": "string (UUID for support)"
  }
}
```

---

© 2025 WIA - World Certification Industry Association
Licensed under MIT License

**弘益人間** · Benefit All Humanity


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

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
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-3-PROTOCOL.
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
for PHASE-3-PROTOCOL. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P3-PROTOCOL-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.
