# WIA-IND-016 PHASE 4 — Integration Specification

**Standard:** WIA-IND-016
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable
**Source:** synthesized from the original `PHASE-1-DATA-FORMAT.md` (quarter 4 of 4)

---

## 12. Guest Feedback & Reputation

### 12.1 Feedback Collection

#### 12.1.1 Survey Distribution

**Post-Stay Survey Timing**:
```
Day 0 (check-out): Thank you email
Day 1: Feedback survey email
Day 3: Reminder (if not completed)
Day 7: Final reminder
```

**Survey Questions**:
```json
{
  "survey": {
    "title": "How was your stay?",
    "questions": [
      {
        "id": 1,
        "type": "rating",
        "question": "Overall satisfaction",
        "scale": "1-5 stars"
      },
      {
        "id": 2,
        "type": "rating",
        "question": "Room cleanliness",
        "scale": "1-5 stars"
      },
      {
        "id": 3,
        "type": "rating",
        "question": "Staff friendliness",
        "scale": "1-5 stars"
      },
      {
        "id": 4,
        "type": "rating",
        "question": "Value for money",
        "scale": "1-5 stars"
      },
      {
        "id": 5,
        "type": "text",
        "question": "What did you enjoy most?"
      },
      {
        "id": 6,
        "type": "text",
        "question": "How can we improve?"
      },
      {
        "id": 7,
        "type": "boolean",
        "question": "Would you recommend us?"
      }
    ]
  }
}
```

#### 12.1.2 Sentiment Analysis

**Automated Analysis**:
```javascript
function analyzeFeedback(feedback) {
  const text = feedback.comments;

  // Sentiment analysis
  const sentiment = analyzeTextSentiment(text);

  // Extract keywords
  const keywords = extractKeywords(text);

  // Identify issues
  const issues = detectIssues(text, keywords);

  // Calculate Net Promoter Score (NPS)
  const nps = calculateNPS(feedback.wouldRecommend);

  return {
    overallSentiment: sentiment, // positive/neutral/negative
    sentimentScore: sentiment.score, // -1 to +1
    keywords: keywords,
    detectedIssues: issues,
    nps: nps,
    requiresResponse: sentiment.score < 0.5 || issues.length > 0
  };
}
```

### 12.2 Review Management

#### 12.2.1 Review Response

**Response Guidelines**:
```
Positive Reviews (4-5 stars):
- Thank the guest
- Mention specific details they liked
- Invite them back
- Response time: within 24 hours

Negative Reviews (1-3 stars):
- Apologize for the experience
- Address specific concerns
- Explain corrective actions
- Offer to make it right
- Response time: within 12 hours
```

**Response Template**:
```
Dear [Guest Name],

Thank you for your recent stay and for taking the time to share your feedback.

[For Positive]:
We're delighted to hear you enjoyed [specific aspect]. Our team works hard to [related effort], and it's wonderful to know it made a difference in your experience.

[For Negative]:
We sincerely apologize that your stay didn't meet expectations, particularly regarding [specific issue]. We've addressed this with our team and have implemented [corrective action] to prevent this from happening again.

We hope to have the opportunity to welcome you back and provide the experience you deserve.

Warm regards,
[Manager Name]
[Title]
[Hotel Name]
```

### 12.3 Reputation Metrics

#### 12.3.1 Key Metrics

**Reputation Score**:
```
Overall Score = (Σ(Rating_i × Weight_i)) / Σ(Weight_i)

Where:
Rating_i = Rating from source i
Weight_i = Weight of source i

Weights:
- Google Reviews: 30%
- Booking.com: 25%
- TripAdvisor: 20%
- Expedia: 15%
- Direct surveys: 10%
```

**Net Promoter Score (NPS)**:
```
NPS = % Promoters (9-10) - % Detractors (0-6)

Scale:
>70: World class
50-70: Excellent
30-50: Good
0-30: Needs improvement
<0: Critical
```

---

## 13. Security & Compliance

### 13.1 Payment Security

#### 13.1.1 PCI DSS Compliance

**Requirements**:
1. **Network Security**:
   - Firewall configuration
   - No default passwords
   - Encrypted transmission

2. **Cardholder Data Protection**:
   - Encrypt stored data
   - Mask PAN (show only last 4 digits)
   - Secure key management

3. **Access Control**:
   - Unique user IDs
   - Role-based access
   - Multi-factor authentication

4. **Monitoring**:
   - Log all access to cardholder data
   - Regular security testing
   - Incident response plan

**Card Data Handling**:
```javascript
function processPayment(cardData, amount) {
  // Tokenize card data immediately
  const token = tokenizeCard(cardData);

  // Never store full PAN
  const maskedPAN = maskCardNumber(cardData.pan);

  // Process payment via secure gateway
  const result = paymentGateway.charge({
    token: token,
    amount: amount,
    currency: 'USD'
  });

  // Store only token and masked number
  return {
    success: result.success,
    transactionId: result.transactionId,
    maskedPAN: maskedPAN,
    // Original card data is never stored
  };
}
```

### 13.2 Data Privacy

#### 13.2.1 GDPR Compliance

**Guest Data Rights**:
1. Right to access
2. Right to rectification
3. Right to erasure (right to be forgotten)
4. Right to data portability
5. Right to object
6. Right to restrict processing

**Data Retention**:
```
Guest profiles: 7 years (or last stay + 3 years)
Financial records: 7 years (tax requirement)
Marketing consent: Until withdrawn
Security logs: 90 days minimum
```

**Data Export**:
```javascript
async function exportGuestData(guestId) {
  const data = {
    personalInfo: await getGuestProfile(guestId),
    stayHistory: await getStayHistory(guestId),
    preferences: await getPreferences(guestId),
    communications: await getCommunications(guestId),
    marketingConsent: await getMarketingConsent(guestId)
  };

  // Format as JSON
  const jsonData = JSON.stringify(data, null, 2);

  // Encrypt for transmission
  const encrypted = encryptData(jsonData);

  return {
    data: encrypted,
    format: 'JSON',
    encryptionType: 'AES-256'
  };
}
```

### 13.3 System Security

#### 13.3.1 Authentication

**Multi-Factor Authentication**:
```
Staff Login:
1. Username + Password
2. SMS/Email OTP or Authenticator app
3. Optional: Biometric (fingerprint/face)

Session timeout: 30 minutes of inactivity
Password requirements:
- Minimum 12 characters
- Uppercase + lowercase + numbers + symbols
- No common words
- Change every 90 days
```

#### 13.3.2 Audit Logging

**Logged Events**:
```json
{
  "eventId": "audit-20250115-001234",
  "timestamp": "2025-01-15T10:30:45Z",
  "userId": "user-123",
  "userName": "john.smith",
  "action": "view-guest-profile",
  "resource": "guest-12345",
  "ipAddress": "192.168.1.100",
  "deviceId": "workstation-05",
  "result": "success",
  "details": {
    "accessReason": "guest check-in",
    "dataAccessed": ["name", "contact", "preferences"]
  }
}
```

---

## 14. API Specifications

### 14.1 RESTful API Design

#### 14.1.1 Authentication

**OAuth 2.0 Flow**:
```
POST /api/v1/auth/token
Content-Type: application/json

{
  "grant_type": "client_credentials",
  "client_id": "your-client-id",
  "client_secret": "your-client-secret"
}

Response:
{
  "access_token": "eyJhbGciOiJSUzI1NiIs...",
  "token_type": "Bearer",
  "expires_in": 3600
}
```

**Using Token**:
```
GET /api/v1/reservations/ABC123
Authorization: Bearer eyJhbGciOiJSUzI1NiIs...
```

#### 14.1.2 Endpoint Structure

**Reservations API**:

```
GET    /api/v1/reservations           - List reservations
POST   /api/v1/reservations           - Create reservation
GET    /api/v1/reservations/{id}      - Get reservation
PUT    /api/v1/reservations/{id}      - Update reservation
DELETE /api/v1/reservations/{id}      - Cancel reservation

GET    /api/v1/availability           - Check availability
POST   /api/v1/reservations/{id}/checkin   - Check in
POST   /api/v1/reservations/{id}/checkout  - Check out
```

**Rooms API**:

```
GET    /api/v1/rooms                  - List rooms
GET    /api/v1/rooms/{number}         - Get room details
PUT    /api/v1/rooms/{number}/status  - Update room status
GET    /api/v1/rooms/availability     - Check room availability
```

**Guests API**:

```
GET    /api/v1/guests                 - List guests
POST   /api/v1/guests                 - Create guest profile
GET    /api/v1/guests/{id}            - Get guest profile
PUT    /api/v1/guests/{id}            - Update guest profile
GET    /api/v1/guests/{id}/history    - Get stay history
```

#### 14.1.3 Response Format

**Success Response**:
```json
{
  "success": true,
  "data": {
    "confirmationNumber": "ABC123",
    "status": "confirmed",
    ...
  },
  "meta": {
    "timestamp": "2025-01-15T10:30:00Z",
    "requestId": "req-123456"
  }
}
```

**Error Response**:
```json
{
  "success": false,
  "error": {
    "code": "ROOM_NOT_AVAILABLE",
    "message": "Requested room type not available for selected dates",
    "details": {
      "roomType": "deluxe-king",
      "checkIn": "2025-01-15",
      "checkOut": "2025-01-18"
    }
  },
  "meta": {
    "timestamp": "2025-01-15T10:30:00Z",
    "requestId": "req-123456"
  }
}
```

---

## 15. Data Models

### 15.1 Core Entities

#### 15.1.1 Entity Relationship Diagram

```
┌──────────────┐
│   Property   │
└───────┬──────┘
        │
        │ 1:N
        ▼
┌──────────────┐
│  Room Type   │
└───────┬──────┘
        │
        │ 1:N
        ▼
┌──────────────┐       1:N      ┌──────────────┐
│     Room     │◄────────────────│ Reservation  │
└──────────────┘                 └───────┬──────┘
                                         │
                                         │ N:1
                                         ▼
                                 ┌──────────────┐
                                 │    Guest     │
                                 └──────────────┘
```

### 15.2 Database Schema

#### 15.2.1 Reservations Table

```sql
CREATE TABLE reservations (
  id UUID PRIMARY KEY,
  confirmation_number VARCHAR(10) UNIQUE NOT NULL,
  property_id UUID NOT NULL,
  guest_id UUID NOT NULL,
  room_type_id UUID NOT NULL,
  room_number VARCHAR(10),
  check_in DATE NOT NULL,
  check_out DATE NOT NULL,
  nights INTEGER NOT NULL,
  adults INTEGER NOT NULL,
  children INTEGER DEFAULT 0,
  rate_code VARCHAR(10) NOT NULL,
  room_rate DECIMAL(10,2) NOT NULL,
  total_amount DECIMAL(10,2) NOT NULL,
  taxes_and_fees DECIMAL(10,2) NOT NULL,
  status VARCHAR(20) NOT NULL,
  special_requests TEXT,
  created_at TIMESTAMP NOT NULL,
  updated_at TIMESTAMP NOT NULL,
  FOREIGN KEY (property_id) REFERENCES properties(id),
  FOREIGN KEY (guest_id) REFERENCES guests(id),
  FOREIGN KEY (room_type_id) REFERENCES room_types(id)
);

CREATE INDEX idx_reservations_confirmation ON reservations(confirmation_number);
CREATE INDEX idx_reservations_guest ON reservations(guest_id);
CREATE INDEX idx_reservations_dates ON reservations(check_in, check_out);
CREATE INDEX idx_reservations_status ON reservations(status);
```

---

## 16. Implementation Guidelines

### 16.1 Integration Checklist

- [ ] PMS core functionality implemented
- [ ] Reservation management operational
- [ ] Room status tracking enabled
- [ ] Payment processing integrated (PCI compliant)
- [ ] Channel manager connected
- [ ] Mobile key system deployed
- [ ] Smart room controls configured
- [ ] Revenue management active
- [ ] Guest feedback system live
- [ ] API endpoints documented
- [ ] Security measures in place
- [ ] Staff training completed

### 16.2 Best Practices

1. **Data Synchronization**:
   - Real-time sync for critical data (reservations, room status)
   - Batch sync for analytics (nightly)
   - Conflict resolution strategy

2. **Scalability**:
   - Design for 10x current load
   - Use caching for frequent queries
   - Implement rate limiting

3. **Monitoring**:
   - System uptime (target: 99.9%)
   - API response times (<500ms)
   - Error rates (<0.1%)
   - User satisfaction scores

4. **Backup and Recovery**:
   - Daily automated backups
   - Point-in-time recovery
   - Disaster recovery plan
   - Regular backup testing

---

**弘益人間 (홍익인간) · Benefit All Humanity**

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
