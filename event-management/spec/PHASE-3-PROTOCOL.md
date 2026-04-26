# WIA-IND-018 PHASE 3 — Protocol Specification

**Standard:** WIA-IND-018
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 3 of 4)

---

- 10 VIP tickets
  - Lead retrieval system
  - Email mention
  - Social media promotion (10 posts)

**Gold Tier** ($25,000-$49,999)
- Benefits:
  - Standard booth (10x10 ft)
  - Quarter-page program ad
  - Logo on website
  - 5 VIP tickets
  - Lead retrieval system
  - Social media promotion (5 posts)

**Silver Tier** ($10,000-$24,999)
- Benefits:
  - Tabletop display
  - Logo on website
  - 3 general tickets
  - Social media mention

**Bronze Tier** ($5,000-$9,999)
- Benefits:
  - Logo on website
  - 2 general tickets
  - Recognition in program

#### 6.1.2 Sponsor Data Model

```json
{
  "sponsor": {
    "id": "string (UUID)",
    "eventId": "string",
    "company": {
      "name": "string",
      "logo": {
        "url": "string",
        "formats": ["png", "svg", "eps"]
      },
      "description": "string",
      "website": "string",
      "industry": "string",
      "size": "startup|small|medium|enterprise"
    },
    "tier": "diamond|platinum|gold|silver|bronze|custom",
    "investment": {
      "amount": "number",
      "currency": "string",
      "paymentTerms": "upfront|installments|post-event",
      "paid": "boolean",
      "paidDate": "date",
      "invoiceNumber": "string"
    },
    "benefits": [
      {
        "type": "booth|speaking-slot|tickets|branding|leads|email|social",
        "description": "string",
        "quantity": "number",
        "delivered": "boolean"
      }
    ],
    "booth": {
      "number": "string",
      "location": "string",
      "size": "10x10|20x20|30x30|custom",
      "electrical": "boolean",
      "internet": "boolean",
      "furniture": ["table", "chairs", "display"],
      "staff": [
        {
          "name": "string",
          "email": "string",
          "tickets": "number"
        }
      ]
    },
    "leads": {
      "systemProvided": "boolean",
      "captured": "number",
      "exported": "boolean",
      "exportDate": "date"
    },
    "marketing": {
      "emailsSent": "number",
      "emailOpens": "number",
      "emailClicks": "number",
      "socialPosts": "number",
      "socialReach": "number",
      "socialEngagement": "number",
      "websiteImpressions": "number"
    },
    "contacts": [
      {
        "name": "string",
        "role": "primary|billing|marketing|logistics",
        "email": "string",
        "phone": "string"
      }
    ],
    "contract": {
      "signed": "boolean",
      "signedDate": "date",
      "documentUrl": "string"
    },
    "satisfaction": {
      "rating": "number (1-5)",
      "feedback": "string",
      "wouldSponsorAgain": "boolean",
      "roi": "positive|neutral|negative"
    }
  }
}
```

### 6.2 Sponsor Portal

Sponsors receive access to dedicated portal:

**Features**:
- Dashboard with ROI metrics
- Lead management interface
- Booth staff registration
- Marketing asset uploads
- Email blast scheduling
- Analytics and reports
- Invoice and payment tracking

**Analytics Provided**:
```json
{
  "analytics": {
    "visibility": {
      "logoImpressions": "number",
      "websiteClicks": "number",
      "boothVisits": "number",
      "videoPlatform Views": "number"
    },
    "engagement": {
      "leadsCollected": "number",
      "conversationDuration": "average minutes",
      "materialsDownloaded": "number",
      "demoRequests": "number"
    },
    "value": {
      "estimatedReach": "number",
      "costPerLead": "number",
      "projectedConversions": "number",
      "estimatedRevenue": "number"
    }
  }
}
```

---

## 7. Virtual & Hybrid Events

### 7.1 Platform Requirements

#### 7.1.1 Core Virtual Features

**Minimum Requirements**:
- HD video streaming (1080p)
- Real-time chat
- Q&A functionality
- Screen sharing
- Recording capability
- Mobile apps (iOS/Android)
- Browser support (no plugins)
- Accessibility features

**Advanced Features**:
- AI-powered transcription
- Real-time translation
- Virtual backgrounds
- Breakout rooms
- Polls and surveys
- Hand raising
- Emoji reactions
- Private messaging
- Networking rooms
- Gamification

#### 7.1.2 Integration Points

Virtual platform must integrate with:

```json
{
  "integrations": {
    "authentication": {
      "sso": "SAML 2.0|OAuth 2.0",
      "providers": ["Google", "Microsoft", "custom"]
    },
    "registration": {
      "api": "REST|GraphQL",
      "webhooks": ["registration", "check-in", "attendance"]
    },
    "crm": {
      "providers": ["Salesforce", "HubSpot", "Marketo"],
      "dataSync": "real-time|batch"
    },
    "analytics": {
      "providers": ["Google Analytics", "Mixpanel", "custom"],
      "events": ["join", "leave", "chat", "poll", "network"]
    },
    "streaming": {
      "ingest": "RTMP|WebRTC",
      "output": ["YouTube", "Facebook", "Vimeo", "custom"]
    }
  }
}
```

### 7.2 Hybrid Experience Design

#### 7.2.1 Parity Requirements

Virtual and physical attendees must have equivalent:

**Content Access**:
- Same session availability
- Equal audio/video quality
- Simultaneous delivery
- Shared presentation materials

**Engagement Opportunities**:
- Combined Q&A queue
- Unified chat
- Equal polling participation
- Cross-venue networking

**Recognition**:
- Visible presence (screens showing virtual attendees)
- Named participation
- Certificate parity
- Equal value perception

#### 7.2.2 Hybrid Production Setup

**Hardware Requirements**:
```json
{
  "production": {
    "cameras": {
      "stage": {
        "quantity": 3,
        "type": "PTZ 4K",
        "positions": ["wide", "presenter", "audience"]
      },
      "confidence": {
        "quantity": 1,
        "type": "monitor",
        "purpose": "speaker-view-of-slides"
      }
    },
    "audio": {
      "speakers": "lapel-wireless × 3",
      "audience": "boundary-mics × 2",
      "mixer": "digital-16-channel",
      "output": ["house-speakers", "stream-feed"]
    },
    "video": {
      "switcher": "ATEM Mini Extreme",
      "screens": {
        "stage": "LED 20ft",
        "confidence": "monitor 32in",
        "virtual-attendees": "display 55in"
      }
    },
    "network": {
      "primary": "dedicated-1gbps",
      "backup": "bonded-cellular",
      "encoder": "hardware-h264"
    }
  }
}
```

**Software Stack**:
- Production: vMix or OBS Studio
- Streaming Platform: Custom or Hopin/Zoom
- Encoder: Hardware or software H.264
- CDN: Multi-region for global reach
- Monitoring: Stream health and quality metrics

---

## 8. Live Streaming

### 8.1 Streaming Architecture

#### 8.1.1 Multi-Platform Distribution

```
[Source] → [Encoder] → [Origin Server] → [CDN] → [Platforms]
                                                    ├─ YouTube
                                                    ├─ Facebook
                                                    ├─ LinkedIn
                                                    ├─ Twitter
                                                    └─ Custom Player
```

**Technical Specifications**:
```json
{
  "encoding": {
    "video": {
      "codec": "H.264",
      "profile": "High",
      "level": "4.2",
      "bitrate": "4-8 Mbps",
      "resolution": "1920x1080",
      "framerate": "30 fps",
      "keyframe": "2 seconds"
    },
    "audio": {
      "codec": "AAC",
      "bitrate": "192 kbps",
      "sampleRate": "48 kHz",
      "channels": "stereo"
    }
  },
  "streaming": {
    "protocol": "RTMP|SRT|WebRTC",
    "latency": {
      "ultra-low": "< 1 second (WebRTC)",
      "low": "< 3 seconds (RTMP)",
      "standard": "< 10 seconds (HLS)"
    },
    "adaptive": {
      "enabled": true,
      "qualities": ["1080p", "720p", "480p", "360p"]
    }
  }
}
```

### 8.2 Interactive Features

#### 8.2.1 Real-Time Engagement

**Chat System**:
```json
{
  "chat": {
    "features": {
      "publicChat": true,
      "privateMessages": true,
      "moderation": "pre|post|ai-assisted",
      "emojis": true,
      "reactions": true,
      "links": "allowed|preview|blocked",
      "fileSharing": true
    },
    "roles": {
      "moderator": ["delete", "ban", "slow-mode"],
      "speaker": ["pin", "highlight"],
      "attendee": ["post", "react", "reply"]
    },
    "settings": {
      "slowMode": "seconds between messages",
      "languageFiltering": true,
      "spam Detection": "ai-powered",
      "maxLength": 500
    }
  }
}
```

**Q&A System**:
```json
{
  "qna": {
    "submission": {
      "authenticated": "required|optional",
      "moderation": "pre-approval|post-filter",
      "anonymous": "allowed|disallowed"
    },
    "voting": {
      "enabled": true,
      "type": "upvote-only|up-down",
      "sortBy": "votes|time|manual"
    },
    "answering": {
      "textResponse": true,
      "liveVerbal": true,
      "pinAnswers": true,
      "markResolved": true
    },
    "export": {
      "formats": ["csv", "json", "pdf"],
      "timing": "during|after"
    }
  }
}
```

**Polls System**:
```json
{
  "polls": {
    "types": ["single-choice", "multiple-choice", "text", "rating", "ranking"],
    "timing": {
      "scheduled": true,
      "manual": true,
      "duration": "seconds"
    },
    "results": {
      "showLive": "boolean",
      "showAfter": "boolean",
      "anonymous": "boolean",
      "export": true
    },
    "integration": {
      "displayOnStream": true,
      "sharable": true,
      "downloadable": true
    }
  }
}
```

---

## 9. Networking Features

### 9.1 AI-Powered Matchmaking

#### 9.1.1 Matching Algorithm

**Profile Factors**:
```json
{
  "profile": {
    "professional": {
      "industry": "weight: 0.20",
      "jobFunction": "weight: 0.15",
      "seniority": "weight: 0.10",
      "company": "weight: 0.05"
    },
    "interests": {
      "topics": "weight: 0.25",
      "sessions": "weight: 0.15",
      "goals": "weight: 0.10"
    }
  }
}
```

**Similarity Score Calculation**:
```
MatchScore = Σ(weight_i × similarity_i)

Where similarity_i is calculated as:
- Exact match: 1.0
- Category match: 0.7
- Related: 0.4
- No match: 0.0
```

**Connection Suggestions**:
- Top 10 matches displayed
- Mutual interest highlighted
- Conversation starters provided
- Video chat invitations enabled

#### 9.1.2 Networking Modes

**Speed Networking**:
- 5-minute video sessions
- Auto-rotation every 5 minutes
- 6-10 connections per hour
- Follow-up connection option

**Topic Tables**:
- Virtual rooms by topic
- Drop-in/drop-out flexibility
- 4-8 participants per table
- Scheduled and ad-hoc

**One-on-One Meetings**:
- Schedule via app
- 15/30/60 minute blocks
- Video or chat
- Calendar integration

**Virtual Lounges**:
- Always-on video spaces
- Topic-based or general
- Screen sharing capability
- Casual networking

### 9.2 Business Card Exchange

#### 9.2.1 Digital Card Format

```json
{
  "digitalCard": {
    "id": "string (UUID)",
    "owner": {
      "name": "string",
      "photo": "url",
      "title": "string",
      "company": "string"
    },
    "contact": {
      "email": "string",
      "phone": "string",
      "website": "string"
    },
    "social": {
      "linkedin": "string",
      "twitter": "string"
    },
    "exchange": {
      "method": "qr-code|nfc|bump|link",
      "permission": "public|connections|private"
    },
    "notes": {
      "addedBy": "recipient",
      "text": "string",
      "tags": ["string"]
    }
  }
}
```

#### 9.2.2 Exchange Methods

**QR Code**:
- Displayed in app
- Scannable by any attendee
- Instant contact save
- Works offline

**NFC Tap**:
- Phone-to-phone transfer
- Badge-to-phone transfer
- Encrypted transmission
- Requires compatible devices

**Digital Bump**:
- Shake phones simultaneously
- Bluetooth proximity detection
- Mutual confirmation
- Fun interaction

**Share Link**:
- Unique personal URL
- Shareable via any channel
- View/save contact
- Track shares (optional)

---


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

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-3-PROTOCOL validation when the
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
