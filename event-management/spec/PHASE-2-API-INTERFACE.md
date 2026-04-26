# WIA-IND-018 PHASE 2 — API Interface Specification

**Standard:** WIA-IND-018
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 2 of 4)

---

## 4. Attendee Registration System

### 4.1 Registration Flow

#### 4.1.1 Multi-Step Registration

**Step 1: Discovery**
- Event landing page
- Schedule preview
- Speaker information
- Pricing tiers

**Step 2: Account Creation**
- Email/social login
- Profile information
- Communication preferences

**Step 3: Ticket Selection**
- Ticket type choice
- Add-ons (workshops, meals, etc.)
- Discount code application

**Step 4: Information Collection**
- Required fields (name, email, company)
- Optional fields (job title, interests, dietary)
- Emergency contact
- Special accommodations

**Step 5: Payment**
- Payment method selection
- Billing information
- Terms acceptance
- Invoice option

**Step 6: Confirmation**
- Order summary
- Calendar file download
- Email confirmation
- Social sharing

#### 4.1.2 Registration Data Model

```json
{
  "registration": {
    "id": "string (UUID)",
    "eventId": "string",
    "status": "pending|confirmed|cancelled|refunded|waitlist",
    "attendee": {
      "id": "string",
      "firstName": "string",
      "lastName": "string",
      "email": "string (validated)",
      "phone": "string",
      "company": "string",
      "jobTitle": "string",
      "profilePhoto": "url",
      "bio": "string (max 500 chars)",
      "social": {
        "twitter": "string",
        "linkedin": "string",
        "website": "string"
      }
    },
    "ticket": {
      "type": "general|vip|student|speaker|sponsor|media",
      "price": "number",
      "currency": "string (ISO 4217)",
      "addOns": [
        {
          "id": "string",
          "name": "string",
          "price": "number"
        }
      ],
      "discountCode": "string",
      "discountAmount": "number",
      "totalPaid": "number"
    },
    "preferences": {
      "dietary": ["vegetarian", "vegan", "gluten-free", "nut-allergy", "other"],
      "accessibility": {
        "wheelchairAccess": "boolean",
        "signLanguage": "boolean",
        "other": "string"
      },
      "interests": ["string"],
      "sessions": ["sessionId"],
      "networking": {
        "enabled": "boolean",
        "visibility": "public|connections|private"
      }
    },
    "metadata": {
      "registeredAt": "timestamp",
      "source": "direct|social|referral|organic",
      "referrer": "string",
      "utm": {
        "source": "string",
        "medium": "string",
        "campaign": "string"
      },
      "ipAddress": "string",
      "userAgent": "string"
    },
    "checkIn": {
      "checkedIn": "boolean",
      "checkInTime": "timestamp",
      "checkInMethod": "qr-code|manual|nfc",
      "badgePrinted": "boolean"
    }
  }
}
```

### 4.2 Ticket Types & Pricing

#### 4.2.1 Standard Ticket Categories

**General Admission**:
- Access to all main sessions
- Standard meals and refreshments
- Event materials
- Certificate of attendance

**VIP/Premium**:
- All General benefits
- Reserved seating
- VIP lounge access
- Premium meals
- Exclusive networking events
- Speaker meet-and-greet
- Gift bag

**Student/Academic**:
- Discounted rate (50-70% off)
- Verification required
- All General benefits
- Student networking sessions

**Early Bird**:
- Time-limited discount (20-30% off)
- Limited quantity
- Encourages early commitment

**Group Rates**:
- 3+ attendees: 10% discount
- 5+ attendees: 15% discount
- 10+ attendees: 20% discount

**Virtual-Only**:
- Live stream access
- Recording access (30-90 days)
- Virtual networking
- Digital materials

#### 4.2.2 Dynamic Pricing Strategy

```javascript
function calculateTicketPrice(basePrice, factors) {
  let price = basePrice;

  // Early bird discount
  if (daysUntilEvent > 60) {
    price *= 0.7; // 30% off
  } else if (daysUntilEvent > 30) {
    price *= 0.85; // 15% off
  }

  // Scarcity pricing
  const capacityUsed = registrations / totalCapacity;
  if (capacityUsed > 0.8) {
    price *= 1.2; // 20% increase
  } else if (capacityUsed > 0.6) {
    price *= 1.1; // 10% increase
  }

  // Demand-based adjustment
  const registrationRate = recentRegistrations / daysSinceOpen;
  if (registrationRate > averageRate * 1.5) {
    price *= 1.15; // High demand
  }

  return Math.round(price);
}
```

### 4.3 Waitlist Management

When event reaches capacity:

1. **Automatic Waitlist**
   - Collect registration information
   - Set expectations (estimated wait time)
   - Require deposit or full payment

2. **Notification System**
   - Email when spots available
   - 24-hour response window
   - Automatic next-in-line promotion

3. **Capacity Monitoring**
   - Track cancellations in real-time
   - Release spots immediately
   - Priority based on waitlist join time

4. **Overflow Options**
   - Offer virtual attendance
   - Suggest related events
   - VIP upgrade opportunity

### 4.4 Group Registration

#### 4.4.1 Team Registration Flow

1. **Team Lead Registration**
   - Create team account
   - Specify number of attendees
   - Receive team discount

2. **Team Member Invitations**
   - Email invites with unique codes
   - Individual profile completion
   - Centralized billing

3. **Team Management**
   - Add/remove members
   - View team roster
   - Assign sessions
   - Group seating requests

4. **Billing Options**
   - Single invoice for all
   - Individual invoices
   - Split payment
   - Purchase order support

---

## 5. Speaker & Performer Management

### 5.1 Speaker Lifecycle

#### 5.1.1 Speaker Data Model

```json
{
  "speaker": {
    "id": "string (UUID)",
    "type": "keynote|presenter|panelist|workshop-leader|mc",
    "personal": {
      "firstName": "string",
      "lastName": "string",
      "title": "string",
      "company": "string",
      "email": "string",
      "phone": "string",
      "photo": {
        "url": "string",
        "highRes": "boolean"
      }
    },
    "professional": {
      "bio": {
        "short": "string (max 150 chars)",
        "long": "string (max 500 chars)"
      },
      "expertise": ["string"],
      "previousSpeaking": [
        {
          "event": "string",
          "date": "date",
          "topic": "string"
        }
      ],
      "publications": ["string"],
      "awards": ["string"]
    },
    "social": {
      "twitter": "string",
      "linkedin": "string",
      "website": "string",
      "instagram": "string"
    },
    "sessions": [
      {
        "sessionId": "string",
        "role": "primary|co-presenter|panelist",
        "preparedPresentation": "boolean"
      }
    ],
    "requirements": {
      "av": {
        "laptop": "own|provided",
        "connectors": ["HDMI", "USB-C"],
        "microphone": "lapel|handheld|headset",
        "clicker": "needed|not-needed",
        "internet": "required|preferred|not-needed"
      },
      "room": {
        "greenRoom": "boolean",
        "privateArea": "boolean",
        "secureStorage": "boolean"
      },
      "travel": {
        "flight": {
          "required": "boolean",
          "class": "economy|business|first",
          "paid": "boolean"
        },
        "hotel": {
          "required": "boolean",
          "nights": "number",
          "paid": "boolean"
        },
        "ground": {
          "pickup": "boolean",
          "rental": "boolean"
        }
      },
      "dietary": ["string"],
      "accessibility": ["string"]
    },
    "compensation": {
      "type": "none|honorarium|fee|expenses-only",
      "amount": "number",
      "currency": "string",
      "paid": "boolean",
      "paidDate": "date"
    },
    "contract": {
      "signed": "boolean",
      "signedDate": "date",
      "documentUrl": "string",
      "terms": {
        "recordingPermission": "boolean",
        "photoPermission": "boolean",
        "materialSharing": "boolean",
        "exclusivity": "boolean"
      }
    },
    "communications": [
      {
        "date": "timestamp",
        "type": "email|call|meeting",
        "subject": "string",
        "notes": "string"
      }
    ],
    "status": "invited|confirmed|declined|tentative|cancelled"
  }
}
```

#### 5.1.2 Speaker Recruitment Process

**Step 1: Call for Proposals (CFP)**
```json
{
  "cfp": {
    "opensAt": "timestamp",
    "closesAt": "timestamp",
    "requirements": {
      "sessionTypes": ["talk", "workshop", "panel"],
      "duration": [30, 45, 60],
      "topics": ["string"],
      "level": ["beginner", "intermediate", "advanced"]
    },
    "submissionForm": {
      "speakerInfo": "required",
      "sessionTitle": "required (max 100 chars)",
      "abstract": "required (max 300 chars)",
      "description": "required (max 1000 chars)",
      "learningObjectives": "required (3-5 items)",
      "targetAudience": "required",
      "previousExperience": "optional",
      "coSpeakers": "optional"
    }
  }
}
```

**Step 2: Review Process**
- Anonymous review by committee
- Scoring rubric (relevance, quality, diversity)
- Selection based on overall program balance

**Step 3: Notification**
- Acceptance emails with contract
- Decline emails with encouragement
- Waitlist for borderline submissions

**Step 4: Onboarding**
- Send speaker kit
- Schedule prep calls
- Collect presentation materials
- Arrange travel and accommodations

### 5.2 Session Management

#### 5.2.1 Session Data Model

```json
{
  "session": {
    "id": "string (UUID)",
    "eventId": "string",
    "type": "keynote|talk|panel|workshop|demo|networking|break",
    "title": "string",
    "description": "string",
    "learningObjectives": ["string"],
    "level": "beginner|intermediate|advanced|all",
    "track": "string",
    "tags": ["string"],
    "speakers": [
      {
        "speakerId": "string",
        "role": "primary|co-presenter|moderator|panelist"
      }
    ],
    "schedule": {
      "date": "date",
      "startTime": "timestamp",
      "endTime": "timestamp",
      "duration": "number (minutes)",
      "timezone": "string (IANA)"
    },
    "venue": {
      "room": "string",
      "capacity": "number",
      "setup": "theater|classroom|roundtable",
      "virtual": {
        "enabled": "boolean",
        "url": "string",
        "platform": "string"
      }
    },
    "resources": {
      "slides": "url",
      "handouts": "url",
      "recording": "url",
      "additionalLinks": ["url"]
    },
    "attendance": {
      "registered": "number",
      "capacity": "number",
      "waitlist": "number",
      "attended": "number",
      "completionRate": "percentage"
    },
    "engagement": {
      "questions": "number",
      "pollResponses": "number",
      "chatMessages": "number",
      "rating": "number (1-5)"
    },
    "requirements": {
      "registration": "required|optional|walk-in",
      "prerequisites": ["string"],
      "materials": ["string"],
      "cost": "number (if add-on)"
    }
  }
}
```

#### 5.2.2 Schedule Optimization

**Constraints**:
1. No speaker double-booking
2. Adequate transition time between sessions (10-15 min)
3. Popular sessions in larger rooms
4. Related sessions in proximity
5. Keynotes in main hall
6. Breaks evenly distributed

**Optimization Algorithm**:
```
minimize:
  TotalAttendeeWalkingDistance +
  UnusedCapacity +
  ScheduleConflicts

subject to:
  - One session per speaker per timeslot
  - Session capacity ≤ Room capacity
  - Break duration ≥ 15 minutes
  - Lunch duration ≥ 45 minutes
```

### 5.3 Speaker Communication

#### 5.3.1 Automated Email Sequences

**Pre-Event (6 weeks before)**:
- Confirmation of session details
- Speaker kit with guidelines
- Request for AV requirements
- Travel booking information

**Pre-Event (2 weeks before)**:
- Reminder of session time and location
- Request final presentation
- Tech check scheduling
- Green room information

**Pre-Event (1 week before)**:
- Final logistics confirmation
- On-site contact information
- Parking and arrival instructions
- Event app access

**Day Before**:
- Session reminder
- Room assignment
- Tech support contact
- Emergency contacts

**Post-Event**:
- Thank you message
- Feedback survey
- Session recording link
- Certificate of participation
- Future event invitation

---

## 6. Sponsor Integration

### 6.1 Sponsorship Tiers

#### 6.1.1 Tier Structure

**Diamond Tier** ($100,000+)
- Benefits:
  - Title sponsor designation
  - Prime booth location (30x30 ft)
  - Keynote speaking slot
  - Full-page program ad
  - Logo on all materials
  - 20 VIP tickets
  - Exclusive reception
  - Lead retrieval system
  - Dedicated email to attendees
  - Social media promotion (20 posts)

**Platinum Tier** ($50,000-$99,999)
- Benefits:
  - Premier booth location (20x20 ft)
  - Speaking slot
  - Half-page program ad
  - Logo on materials


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

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-2-API-INTERFACE validation when the
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
