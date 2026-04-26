# WIA-IND-018 PHASE 1 — Data Format Specification

**Standard:** WIA-IND-018
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 1 of 4)

---

# WIA-IND-018: Event Management Specification v1.0

> **Standard ID:** WIA-IND-018
> **Version:** 1.0.0
> **Published:** 2025-12-27
> **Status:** Active
> **Authors:** WIA Industry Standards Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Event Planning & Lifecycle](#2-event-planning--lifecycle)
3. [Venue Management](#3-venue-management)
4. [Attendee Registration System](#4-attendee-registration-system)
5. [Speaker & Performer Management](#5-speaker--performer-management)
6. [Sponsor Integration](#6-sponsor-integration)
7. [Virtual & Hybrid Events](#7-virtual--hybrid-events)
8. [Live Streaming](#8-live-streaming)
9. [Networking Features](#9-networking-features)
10. [Analytics & Reporting](#10-analytics--reporting)
11. [Feedback & Surveys](#11-feedback--surveys)
12. [Security & Privacy](#12-security--privacy)
13. [Integration Requirements](#13-integration-requirements)
14. [Implementation Guidelines](#14-implementation-guidelines)
15. [Compliance & Standards](#15-compliance--standards)
16. [References](#16-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines a comprehensive framework for managing events of all types and scales, from small workshops to large international conferences. The standard provides unified interfaces for event planning, execution, and analysis.

### 1.2 Scope

The standard covers:
- Complete event lifecycle management
- Registration and ticketing systems
- Venue coordination (physical and virtual)
- Speaker and sponsor management
- Live streaming and broadcasting
- Attendee networking and engagement
- Real-time analytics and reporting
- Post-event feedback collection

### 1.3 Philosophy

**弘益人間 (Benefit All Humanity)** - This standard aims to democratize professional event management, making it accessible to organizations of all sizes while fostering meaningful human connections and knowledge sharing.

### 1.4 Terminology

- **Event**: A planned occasion with defined objectives, audience, and timeline
- **Attendee**: A person registered to participate in an event
- **Session**: A scheduled component of an event (talk, workshop, panel, etc.)
- **Track**: A themed series of sessions within an event
- **Venue**: Physical or virtual location where an event occurs
- **Sponsor**: Organization providing financial or in-kind support
- **Organizer**: Individual or team responsible for event planning and execution
- **Hybrid Event**: Event combining physical and virtual attendance options

---

## 2. Event Planning & Lifecycle

### 2.1 Event Lifecycle Phases

The event lifecycle consists of six primary phases:

#### 2.1.1 Conception Phase

**Duration**: 6-12 months before event
**Key Activities**:
- Define event objectives and KPIs
- Identify target audience and size
- Establish budget and funding sources
- Select dates and preliminary venue
- Form organizing committee

**Required Data**:
```json
{
  "eventConcept": {
    "title": "string",
    "objectives": ["string"],
    "targetAudience": {
      "demographics": {},
      "size": "number",
      "profile": "string"
    },
    "budget": {
      "total": "number",
      "sources": ["ticket-sales", "sponsors", "grants"],
      "allocation": {}
    },
    "dates": {
      "preferred": ["date"],
      "blackoutDates": ["date"]
    }
  }
}
```

#### 2.1.2 Planning Phase

**Duration**: 3-6 months before event
**Key Activities**:
- Finalize venue contracts
- Develop event agenda and schedule
- Recruit speakers and performers
- Design marketing materials
- Set up registration system
- Secure sponsorships

**Planning Checklist**:
- [ ] Venue confirmed and contracted
- [ ] Event schedule created
- [ ] Registration system live
- [ ] Marketing campaign launched
- [ ] Speaker confirmations received
- [ ] Sponsor packages defined
- [ ] Volunteer team recruited
- [ ] Technology requirements identified

#### 2.1.3 Promotion Phase

**Duration**: 1-3 months before event
**Key Activities**:
- Launch registration
- Execute marketing campaigns
- Send speaker reminders
- Confirm vendor contracts
- Produce event materials
- Set up event app/website

**Promotion Metrics**:
```json
{
  "registrations": {
    "daily": "number",
    "cumulative": "number",
    "conversionRate": "percentage"
  },
  "marketing": {
    "emailOpens": "number",
    "socialReach": "number",
    "websiteVisits": "number"
  }
}
```

#### 2.1.4 Execution Phase

**Duration**: Event days
**Key Activities**:
- Attendee check-in
- Session coordination
- Real-time problem solving
- Sponsor activation
- Live streaming
- Social media engagement

**Real-time Monitoring**:
- Attendance tracking per session
- Technical issue resolution
- Attendee engagement metrics
- Social media sentiment
- Vendor coordination

#### 2.1.5 Analysis Phase

**Duration**: 1-2 weeks after event
**Key Activities**:
- Collect feedback surveys
- Analyze attendance data
- Calculate ROI
- Review sponsor satisfaction
- Assess speaker performance
- Document lessons learned

**Success Metrics**:
```json
{
  "attendance": {
    "registered": "number",
    "attended": "number",
    "attendanceRate": "percentage"
  },
  "satisfaction": {
    "nps": "number",
    "averageRating": "number",
    "wouldReturn": "percentage"
  },
  "financial": {
    "revenue": "number",
    "expenses": "number",
    "roi": "percentage"
  }
}
```

#### 2.1.6 Follow-up Phase

**Duration**: 1-3 months after event
**Key Activities**:
- Share recordings and materials
- Send thank you communications
- Nurture attendee community
- Plan next event
- Report to stakeholders

### 2.2 Event Types

The standard supports multiple event formats:

#### 2.2.1 Conference

**Characteristics**:
- Multi-day duration (1-5 days)
- Multiple parallel tracks
- 100-10,000+ attendees
- Keynotes, sessions, workshops
- Exhibition area
- Networking events

**Required Features**:
- Multi-track scheduling
- Session capacity management
- Badge printing
- Mobile app
- Live streaming
- Sponsor booths

#### 2.2.2 Workshop

**Characteristics**:
- Single or half-day duration
- Hands-on learning
- 10-100 attendees
- Single track
- Interactive exercises

**Required Features**:
- Materials distribution
- Breakout rooms
- Capacity limits
- Prerequisites checking

#### 2.2.3 Webinar

**Characteristics**:
- 30-120 minute duration
- Online only
- 50-10,000+ attendees
- Single presenter or panel
- Q&A session

**Required Features**:
- Live streaming platform
- Chat and Q&A
- Recording
- Polls and surveys
- Attendee analytics

#### 2.2.4 Hybrid Event

**Characteristics**:
- Simultaneous physical and virtual
- Unified experience design
- Different ticket tiers
- Technology integration

**Required Features**:
- Dual registration system
- Live streaming infrastructure
- Virtual networking spaces
- Unified chat/Q&A
- Separate capacity tracking

#### 2.2.5 Exhibition/Trade Show

**Characteristics**:
- 1-3 day duration
- Focus on vendor booths
- 1,000-50,000+ attendees
- Product demonstrations
- Lead generation

**Required Features**:
- Booth management system
- Lead capture tools
- Floor plan management
- Exhibitor portal
- Analytics dashboard

### 2.3 Event Status Workflow

Events progress through defined statuses:

```
DRAFT → PLANNING → PUBLISHED → REGISTRATION_OPEN →
REGISTRATION_CLOSED → IN_PROGRESS → COMPLETED → ARCHIVED
```

**Status Definitions**:

- **DRAFT**: Initial creation, not visible to public
- **PLANNING**: Active planning, team collaboration
- **PUBLISHED**: Public announcement, pre-registration
- **REGISTRATION_OPEN**: Accepting registrations
- **REGISTRATION_CLOSED**: No new registrations accepted
- **IN_PROGRESS**: Event is currently happening
- **COMPLETED**: Event finished, analysis phase
- **ARCHIVED**: Historical record, no active updates

---

## 3. Venue Management

### 3.1 Physical Venue Requirements

#### 3.1.1 Venue Data Model

```json
{
  "venue": {
    "id": "string",
    "name": "string",
    "type": "conference-center|hotel|university|outdoor|custom",
    "address": {
      "street": "string",
      "city": "string",
      "state": "string",
      "country": "string",
      "postalCode": "string",
      "coordinates": {
        "latitude": "number",
        "longitude": "number"
      }
    },
    "capacity": {
      "total": "number",
      "seated": "number",
      "standing": "number",
      "byRoom": {}
    },
    "amenities": {
      "wifi": {
        "available": "boolean",
        "bandwidth": "string",
        "guestNetwork": "boolean"
      },
      "av": {
        "projectors": "number",
        "screens": "number",
        "soundSystem": "boolean",
        "recording": "boolean"
      },
      "catering": {
        "onSite": "boolean",
        "vendors": ["string"],
        "dietary": ["vegetarian", "vegan", "gluten-free", "halal", "kosher"]
      },
      "parking": {
        "spaces": "number",
        "cost": "number",
        "validation": "boolean"
      },
      "accessibility": {
        "wheelchairAccessible": "boolean",
        "elevators": "boolean",
        "signLanguage": "boolean",
        "hearingAssistance": "boolean"
      }
    },
    "rooms": [
      {
        "id": "string",
        "name": "string",
        "capacity": "number",
        "setup": "theater|classroom|banquet|boardroom|u-shape",
        "dimensions": {
          "length": "number",
          "width": "number",
          "height": "number"
        },
        "equipment": ["projector", "screen", "microphones", "whiteboard"]
      }
    ],
    "contact": {
      "name": "string",
      "email": "string",
      "phone": "string"
    },
    "policies": {
      "cancellation": "string",
      "insurance": "boolean",
      "security": "string"
    }
  }
}
```

#### 3.1.2 Space Allocation Algorithm

The system must optimize room assignments based on:

1. **Session Capacity**: Match expected attendance to room size
2. **Equipment Needs**: Ensure required AV equipment available
3. **Schedule Conflicts**: Prevent double-booking
4. **Proximity**: Group related sessions near each other
5. **Flow**: Minimize attendee movement between sessions

**Optimization Formula**:
```
Score = (0.4 × CapacityMatch) + (0.3 × EquipmentFit) +
        (0.2 × ProximityScore) + (0.1 × FlowEfficiency)
```

Where:
- CapacityMatch = 1 - |ActualCapacity - RequiredCapacity| / RequiredCapacity
- EquipmentFit = AvailableEquipment ∩ RequiredEquipment / RequiredEquipment
- ProximityScore = Calculated based on related session distances
- FlowEfficiency = Measured by expected attendee walking time

### 3.2 Virtual Venue Requirements

#### 3.2.1 Platform Specifications

Virtual venues must support:

**Core Features**:
- High-definition video streaming (1080p minimum)
- Real-time chat and Q&A
- Screen sharing and presentation
- Recording and playback
- Breakout rooms
- Virtual backgrounds
- Closed captioning

**Capacity Tiers**:
- Small: Up to 100 concurrent users
- Medium: 100-1,000 concurrent users
- Large: 1,000-10,000 concurrent users
- Enterprise: 10,000+ concurrent users

**Technical Requirements**:
```json
{
  "streaming": {
    "protocol": "RTMP|WebRTC|HLS",
    "bitrate": "2-8 Mbps",
    "resolution": "1920x1080",
    "framerate": "30 fps",
    "latency": "< 3 seconds"
  },
  "audio": {
    "codec": "Opus|AAC",
    "sampleRate": "48 kHz",
    "channels": "stereo",
    "bitrate": "128-192 kbps"
  },
  "platform": {
    "browsers": ["Chrome", "Firefox", "Safari", "Edge"],
    "mobile": ["iOS", "Android"],
    "bandwidth": "5 Mbps minimum"
  }
}
```

#### 3.2.2 Virtual Lobby Design

The virtual lobby serves as the main navigation hub:

**Components**:
- **Event Schedule**: Interactive agenda with session links
- **Networking Lounge**: Video chat matching system
- **Exhibition Hall**: Virtual sponsor booths
- **Resource Center**: Downloadable materials
- **Help Desk**: Live support chat
- **Social Feed**: Real-time activity stream

**User Interface Requirements**:
- Single-page application architecture
- Responsive design (desktop, tablet, mobile)
- Keyboard navigation support
- Screen reader compatibility
- Multi-language support

### 3.3 Hybrid Venue Coordination

#### 3.3.1 Integration Points

Hybrid events require seamless integration:

**Physical to Virtual**:
- Camera feeds from main stage
- Audience microphones for Q&A
- Shared presentation screens
- In-room participant visibility

**Virtual to Physical**:
- Large screen showing virtual attendees
- Remote speaker integration
- Virtual Q&A display
- Online poll results

**Unified Experience**:
- Same content delivery timing
- Integrated chat system
- Combined networking opportunities
- Consistent branding

#### 3.3.2 Technology Stack

```json
{
  "hardware": {
    "cameras": {
      "type": "PTZ|fixed",
      "quantity": "3-5",
      "resolution": "4K",
      "features": ["auto-tracking", "wide-angle"]
    },
    "audio": {
      "microphones": ["lapel", "handheld", "audience"],
      "mixer": "digital",
      "speakers": "line-array"
    },
    "network": {
      "primary": "dedicated fiber",
      "backup": "4G/5G bonding",
      "bandwidth": "100+ Mbps"
    }
  },
  "software": {
    "streaming": "OBS|vMix|Wirecast",
    "platform": "Zoom|Hopin|custom",
    "production": "ATEM|TriCaster",
    "management": "event-platform-api"
  }
}
```

---


## Annex E — Implementation Notes for PHASE-1-DATA-FORMAT

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1-DATA-FORMAT.

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
evidence for PHASE-1-DATA-FORMAT. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1-data-format/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-1-DATA-FORMAT with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1-DATA-FORMAT does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-1-DATA-FORMAT.
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
for PHASE-1-DATA-FORMAT. The profile mechanism is non-normative and exists so that
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
  advertises `WIA-P1-DATA-FORMAT-Edge-only/2` is asserting conformance with
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
  is published manually. Sufficient for PHASE-1-DATA-FORMAT validation when the
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
