# WIA-IND-018 PHASE 4 — Integration Specification

**Standard:** WIA-IND-018
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

## 10. Analytics & Reporting

### 10.1 Real-Time Dashboard

#### 10.1.1 Key Metrics

**Attendance Metrics**:
```json
{
  "attendance": {
    "current": {
      "total": "number",
      "physical": "number",
      "virtual": "number",
      "bySession": {}
    },
    "cumulative": {
      "registered": "number",
      "checkedIn": "number",
      "noShows": "number",
      "rate": "percentage"
    },
    "trends": {
      "peakAttendance": "number",
      "peakTime": "timestamp",
      "averageSession": "number"
    }
  }
}
```

**Engagement Metrics**:
```json
{
  "engagement": {
    "chat": {
      "messages": "number",
      "participants": "number",
      "messagesPerUser": "average"
    },
    "qna": {
      "questions": "number",
      "answered": "number",
      "answerRate": "percentage"
    },
    "polls": {
      "conducted": "number",
      "responses": "number",
      "participationRate": "percentage"
    },
    "networking": {
      "connections": "number",
      "meetings": "number",
      "cardsExchanged": "number"
    }
  }
}
```

**Revenue Metrics**:
```json
{
  "revenue": {
    "tickets": {
      "sold": "number",
      "revenue": "number",
      "byType": {}
    },
    "sponsorships": {
      "secured": "number",
      "revenue": "number",
      "byTier": {}
    },
    "total": "number",
    "projection": "number",
    "vsTarget": "percentage"
  }
}
```

### 10.2 Post-Event Reports

#### 10.2.1 Executive Summary

Automatically generated report includes:

1. **Overview**
   - Event dates and duration
   - Total attendance (physical + virtual)
   - Number of sessions and speakers
   - Geographic distribution

2. **Financial Performance**
   - Total revenue by source
   - Total expenses by category
   - Net profit/loss
   - ROI percentage

3. **Attendee Insights**
   - Demographics breakdown
   - Industry representation
   - Job function distribution
   - Satisfaction scores

4. **Engagement Analysis**
   - Most attended sessions
   - Highest rated speakers
   - Networking statistics
   - Content interaction

5. **Sponsor ROI**
   - Brand exposure metrics
   - Lead generation results
   - Engagement statistics
   - Satisfaction ratings

6. **Recommendations**
   - What worked well
   - Areas for improvement
   - Suggestions for next event

#### 10.2.2 Custom Reports

**Available Reports**:
- Attendance by session
- Speaker ratings
- Sponsor analytics
- Revenue breakdown
- Marketing attribution
- Geographic analysis
- Feedback themes
- Content engagement
- Networking activity

**Export Formats**:
- PDF (formatted report)
- Excel (raw data + charts)
- CSV (raw data only)
- JSON (API integration)

---

## 11. Feedback & Surveys

### 11.1 Survey Types

#### 11.1.1 Registration Survey

Collected during registration:
- How did you hear about the event?
- What are your goals for attending?
- Which topics interest you most?
- Any special requirements?

#### 11.1.2 Session Feedback

Collected after each session:
```json
{
  "sessionSurvey": {
    "timing": "immediately-after|end-of-day",
    "questions": [
      {
        "type": "rating",
        "question": "Rate this session overall",
        "scale": 5,
        "required": true
      },
      {
        "type": "rating",
        "question": "Speaker knowledge and expertise",
        "scale": 5
      },
      {
        "type": "rating",
        "question": "Content relevance",
        "scale": 5
      },
      {
        "type": "rating",
        "question": "Presentation quality",
        "scale": 5
      },
      {
        "type": "text",
        "question": "What was most valuable?",
        "maxLength": 500
      },
      {
        "type": "text",
        "question": "What could be improved?",
        "maxLength": 500
      }
    ]
  }
}
```

#### 11.1.3 Post-Event Survey

Comprehensive feedback collected 1-2 days after event:

**Net Promoter Score (NPS)**:
- "How likely are you to recommend this event?" (0-10)
- Follow-up: "What's the primary reason for your score?"

**Overall Satisfaction**:
- Event organization (1-5)
- Venue/platform quality (1-5)
- Content quality (1-5)
- Networking opportunities (1-5)
- Value for money (1-5)

**Specific Feedback**:
- Best aspects of the event
- Areas for improvement
- Topics for next time
- Would you attend again?
- Additional comments

### 11.2 Sentiment Analysis

#### 11.2.1 Text Analysis

Automatically analyze open-text responses:

**Sentiment Classification**:
- Positive: 😊 (score > 0.6)
- Neutral: 😐 (score 0.4-0.6)
- Negative: 😞 (score < 0.4)

**Theme Extraction**:
```json
{
  "themes": [
    {
      "theme": "Great speakers",
      "mentions": 145,
      "sentiment": 0.85,
      "examples": ["Keynote was inspiring", "Loved the speaker lineup"]
    },
    {
      "theme": "Networking opportunities",
      "mentions": 98,
      "sentiment": 0.78
    },
    {
      "theme": "Food quality",
      "mentions": 67,
      "sentiment": 0.45
    }
  ]
}
```

**Action Items**:
- Identify top 3 strengths to maintain
- Identify top 3 areas for improvement
- Categorize feedback by urgency
- Generate response templates

---

## 12. Security & Privacy

### 12.1 Data Protection

#### 12.1.1 GDPR Compliance

**Personal Data Collection**:
- Clear consent requests
- Purpose specification
- Minimal data collection
- Right to access
- Right to deletion
- Data portability

**Privacy Controls**:
```json
{
  "privacy": {
    "consent": {
      "marketing": "opt-in|opt-out",
      "dataSharing": "opt-in",
      "photos": "opt-in",
      "recording": "opt-in"
    },
    "visibility": {
      "profile": "public|connections|private",
      "attendance": "visible|hidden",
      "networking": "enabled|disabled"
    },
    "retention": {
      "personal": "365 days",
      "anonymous": "indefinite",
      "onRequest": "delete within 30 days"
    }
  }
}
```

### 12.2 Access Control

#### 12.2.1 Role-Based Permissions

**Roles**:
- **Super Admin**: Full system access
- **Event Manager**: Event configuration and management
- **Content Manager**: Session and speaker management
- **Marketing**: Registration and communication
- **Finance**: Payment and revenue tracking
- **Support**: Attendee assistance
- **Speaker**: Own session management
- **Sponsor**: Sponsor portal access
- **Attendee**: Public features only

**Permission Matrix**:
```
Feature                  | Admin | Manager | Content | Marketing | Speaker | Attendee
-------------------------|-------|---------|---------|-----------|---------|----------
Create Event             |   ✓   |    ✓    |         |           |         |
Edit Event Settings      |   ✓   |    ✓    |         |           |         |
Manage Sessions          |   ✓   |    ✓    |    ✓    |           |    *    |
Manage Speakers          |   ✓   |    ✓    |    ✓    |           |         |
View Registrations       |   ✓   |    ✓    |         |     ✓     |         |
Send Communications      |   ✓   |    ✓    |         |     ✓     |         |
View Analytics           |   ✓   |    ✓    |    ✓    |     ✓     |    *    |
Export Data              |   ✓   |    ✓    |         |     ✓     |         |
Manage Payments          |   ✓   |    ✓    |         |           |         |

* = Own data only
```

---

## 13. Integration Requirements

### 13.1 API Specifications

#### 13.1.1 RESTful API

**Base URL**: `https://api.events.wia.org/v1`

**Authentication**:
```http
Authorization: Bearer {api_key}
Content-Type: application/json
```

**Core Endpoints**:

```
Events
  GET    /events                    - List all events
  POST   /events                    - Create new event
  GET    /events/{id}               - Get event details
  PUT    /events/{id}               - Update event
  DELETE /events/{id}               - Delete event

Registrations
  GET    /events/{id}/registrations - List registrations
  POST   /events/{id}/register      - Register attendee
  GET    /registrations/{id}        - Get registration
  PUT    /registrations/{id}        - Update registration
  DELETE /registrations/{id}        - Cancel registration

Sessions
  GET    /events/{id}/sessions      - List sessions
  POST   /events/{id}/sessions      - Create session
  GET    /sessions/{id}             - Get session
  PUT    /sessions/{id}             - Update session
  DELETE /sessions/{id}             - Delete session

Speakers
  GET    /events/{id}/speakers      - List speakers
  POST   /events/{id}/speakers      - Add speaker
  GET    /speakers/{id}             - Get speaker
  PUT    /speakers/{id}             - Update speaker

Analytics
  GET    /events/{id}/analytics     - Get event analytics
  GET    /sessions/{id}/analytics   - Get session analytics
  GET    /speakers/{id}/analytics   - Get speaker analytics
```

**Rate Limiting**:
- 1000 requests per hour per API key
- 10000 requests per day per API key
- 429 status code when exceeded

#### 13.1.2 Webhooks

Subscribe to real-time events:

```json
{
  "webhook": {
    "url": "https://your-server.com/webhook",
    "events": [
      "registration.created",
      "registration.updated",
      "registration.cancelled",
      "checkin.completed",
      "session.started",
      "session.completed"
    ],
    "secret": "webhook_signing_key"
  }
}
```

**Webhook Payload**:
```json
{
  "event": "registration.created",
  "timestamp": "2025-12-27T10:30:00Z",
  "data": {
    "eventId": "evt_123",
    "registrationId": "reg_456",
    "attendee": {}
  }
}
```

---

## 14. Implementation Guidelines

### 14.1 Technical Stack Recommendations

**Backend**:
- Language: Node.js, Python, or Go
- Framework: Express, FastAPI, or Gin
- Database: PostgreSQL (relational) + Redis (caching)
- Queue: RabbitMQ or AWS SQS
- Storage: S3-compatible object storage

**Frontend**:
- Framework: React, Vue, or Svelte
- State Management: Redux or Zustand
- UI Components: Material-UI or Tailwind
- Mobile: React Native or Flutter

**Infrastructure**:
- Hosting: AWS, GCP, or Azure
- CDN: CloudFront or Cloudflare
- Email: SendGrid or AWS SES
- SMS: Twilio or AWS SNS
- Video: Zoom, Agora, or custom WebRTC

### 14.2 Scalability Requirements

**Performance Targets**:
- API response time: < 200ms (p95)
- Page load time: < 2 seconds
- Registration process: < 60 seconds
- Check-in process: < 10 seconds
- Real-time updates: < 500ms latency

**Capacity Planning**:
```json
{
  "capacity": {
    "small": {
      "attendees": "< 500",
      "sessions": "< 50",
      "concurrent": "< 100",
      "infrastructure": "single-region"
    },
    "medium": {
      "attendees": "500-5000",
      "sessions": "50-200",
      "concurrent": "100-1000",
      "infrastructure": "multi-az"
    },
    "large": {
      "attendees": "5000-50000",
      "sessions": "200-1000",
      "concurrent": "1000-10000",
      "infrastructure": "multi-region"
    },
    "enterprise": {
      "attendees": "> 50000",
      "sessions": "> 1000",
      "concurrent": "> 10000",
      "infrastructure": "global-cdn"
    }
  }
}
```

---

## 15. Compliance & Standards

### 15.1 Accessibility (WCAG 2.1 Level AA)

**Requirements**:
- Keyboard navigation support
- Screen reader compatibility
- Color contrast ratios (4.5:1 minimum)
- Text resizing (up to 200%)
- Alternative text for images
- Captions for video content
- Transcript for audio content

### 15.2 Industry Standards

**Compliance**:
- PCI DSS for payment processing
- GDPR for EU data protection
- CCPA for California residents
- SOC 2 Type II certification
- ISO 27001 information security

---

## 16. References

### 16.1 Related Standards

- ISO 20121: Event Sustainability Management
- ISO 9001: Quality Management
- APEX (Accepted Practices Exchange): Event industry standards
- MPI (Meeting Professionals International): Best practices

### 16.2 Technical References

- WebRTC 1.0: Real-time communication
- OAuth 2.0: Authentication
- OpenID Connect: Identity layer
- JSON API: API specification
- Webhooks: Event-driven architecture

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*This specification provides a comprehensive framework for modern event management, supporting the full lifecycle from planning through post-event analysis.*

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

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
evidence for PHASE-4-INTEGRATION. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-4-integration/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-4-INTEGRATION with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-4-INTEGRATION does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-4-INTEGRATION.
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
for PHASE-4-INTEGRATION. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P4-INTEGRATION-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-4-INTEGRATION validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
