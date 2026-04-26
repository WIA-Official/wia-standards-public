# WIA-EDU-003: Adaptive Learning Standard
## PHASE 1: Foundation

**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2025-12-25

---

## 弘益人間 (Benefit All Humanity)

The WIA-EDU-003 standard is built on the principle that adaptive learning should be accessible, effective, and beneficial for all learners, regardless of their background or circumstances.

---

## Phase Overview

Phase 1 establishes the foundational infrastructure and core capabilities required for adaptive learning systems. This phase focuses on creating the basic building blocks that will support more advanced features in subsequent phases.

### Duration
Typically 3-4 months for initial implementation

### Goals
1. Establish technical infrastructure
2. Create initial learner model schema
3. Import or create basic content library
4. Implement core APIs
5. Build basic user interface

---

## 1. System Requirements

### 1.1 Technical Infrastructure

**Required Components:**
- Application server (Node.js, Python, or Java)
- Relational database (PostgreSQL, MySQL)
- Document database (MongoDB, CouchDB) - optional but recommended
- Cache layer (Redis, Memcached)
- Web server (Nginx, Apache)
- SSL/TLS certificates for HTTPS

**Minimum Hardware (for pilot deployment):**
- 4 CPU cores
- 16 GB RAM
- 500 GB storage (SSD recommended)
- 100 Mbps network connection

**Scalability Targets:**
- Support 50-500 concurrent users
- API response time < 500ms
- 99% uptime

### 1.2 Development Environment

**Required Tools:**
- Version control (Git)
- CI/CD pipeline (GitHub Actions, GitLab CI, Jenkins)
- Testing framework (Jest, PyTest, JUnit)
- API documentation tool (Swagger, OpenAPI)
- Monitoring and logging (basic setup)

---

## 2. Learner Model

### 2.1 Core Profile Schema

Every learner must have a profile containing:

```json
{
  "id": "uuid-string",
  "demographics": {
    "age": number,
    "grade": number,
    "language": "string (ISO 639-1)",
    "timezone": "string (IANA timezone)"
  },
  "preferences": {
    "learningStyles": ["visual", "auditory", "kinesthetic", "reading"],
    "contentTypes": ["text", "video", "audio", "interactive"],
    "difficulty": "beginner|intermediate|advanced",
    "pace": "self-paced|instructor-led|hybrid"
  },
  "knowledgeState": {
    "concept-id": number (0-1 representing mastery level)
  },
  "created": "ISO 8601 timestamp",
  "lastActive": "ISO 8601 timestamp"
}
```

### 2.2 Initial Assessment

**Purpose:** Establish baseline knowledge and preferences

**Components:**
1. **Onboarding Questionnaire**
   - Learning goals
   - Prior experience
   - Preferred learning methods
   - Time availability

2. **Diagnostic Assessment**
   - 10-20 questions covering key concepts
   - Adaptive difficulty adjustment
   - 15-30 minute duration
   - Results used to initialize knowledge state

3. **Learning Style Inventory (Optional)**
   - VARK assessment or similar
   - Used to inform content recommendations
   - Learner can skip if preferred

### 2.3 Privacy Requirements

- [ ] Obtain consent before collecting personal data
- [ ] Encrypt PII in database (AES-256)
- [ ] Implement data minimization principles
- [ ] Provide data access and export capabilities
- [ ] Support data deletion requests
- [ ] Comply with FERPA/GDPR/COPPA as applicable

---

## 3. Content Management

### 3.1 Minimum Content Library

**Phase 1 Requirements:**
- At least 50 learning objects across target subject area
- Minimum 3 different content types (e.g., text, video, quiz)
- Clear learning objectives for each object
- Prerequisite relationships defined

### 3.2 Learning Object Metadata

**Required Fields:**
```json
{
  "id": "uuid",
  "title": "string",
  "description": "string",
  "type": "text|video|audio|interactive|assessment",
  "format": "html|mp4|mp3|h5p|scorm",
  "learningObjectives": ["string"],
  "prerequisites": ["object-id"],
  "difficulty": number (0-100),
  "estimatedDuration": number (minutes),
  "language": "string (ISO 639-1)",
  "created": "ISO 8601 timestamp",
  "lastModified": "ISO 8601 timestamp"
}
```

### 3.3 Content Storage

**Options:**
1. Local file storage with database references
2. Cloud storage (S3, Google Cloud Storage, Azure Blob)
3. CDN for media files

**Requirements:**
- Versioning support
- Backup and recovery
- Access control
- Content delivery optimization

---

## 4. Core APIs

### 4.1 Authentication API

```
POST /api/v1/auth/login
POST /api/v1/auth/logout
POST /api/v1/auth/refresh
GET  /api/v1/auth/verify
```

**Security:**
- JWT tokens with expiration
- Secure password hashing (bcrypt, Argon2)
- Optional MFA support
- Rate limiting on login attempts

### 4.2 Learner Profile API

```
GET    /api/v1/learners/{id}
POST   /api/v1/learners
PUT    /api/v1/learners/{id}
DELETE /api/v1/learners/{id}
GET    /api/v1/learners/{id}/preferences
PUT    /api/v1/learners/{id}/preferences
```

### 4.3 Content API

```
GET /api/v1/content
GET /api/v1/content/{id}
GET /api/v1/content/search?q={query}&type={type}&difficulty={level}
```

### 4.4 Recommendation API (Basic)

```
GET /api/v1/recommendations/{learnerId}
```

**Phase 1 Implementation:**
- Rule-based recommendation engine
- Consider learner preferences
- Check prerequisites
- Match difficulty level
- Return 1-5 recommended learning objects

---

## 5. User Interface

### 5.1 Required Views

1. **Login/Registration**
   - Email/password authentication
   - Optional SSO integration
   - Password reset functionality

2. **Learner Dashboard**
   - Current progress overview
   - Recommended next activities
   - Recent activity
   - Learning goals

3. **Content Viewer**
   - Support for text, video, interactive content
   - Navigation controls
   - Progress tracking
   - Embedded assessments

4. **Assessment Interface**
   - Multiple choice questions
   - Text input
   - Immediate feedback
   - Progress indication

5. **Profile Settings**
   - Update preferences
   - View privacy settings
   - Manage account

### 5.2 Accessibility Requirements

- [ ] WCAG 2.1 Level A minimum (AA recommended)
- [ ] Keyboard navigation
- [ ] Screen reader compatibility
- [ ] Color contrast ratios (4.5:1 for normal text)
- [ ] Responsive design (mobile, tablet, desktop)

---

## 6. Basic Adaptation Logic

### 6.1 Rule-Based Recommendations

**Algorithm:**
1. Identify concepts below mastery threshold (< 0.7)
2. Filter by prerequisites (all prerequisites must be mastered)
3. Match learner's preferred content types
4. Adjust difficulty based on recent performance
5. Select highest-priority unmastered concept
6. Return best-matching content

**Example Rules:**
```
IF mastery < 0.5 THEN recommend easier content
IF mastery >= 0.7 AND < 0.9 THEN recommend practice content
IF mastery >= 0.9 THEN recommend next concept
IF 3 consecutive incorrect THEN provide hint or easier content
IF 3 consecutive correct THEN increase difficulty
```

### 6.2 Progress Tracking

**Metrics:**
- Completion status (started, in-progress, completed)
- Time spent on each learning object
- Assessment scores
- Number of attempts
- Concept mastery levels (updated after each assessment)

---

## 7. Data Collection

### 7.1 Learning Events

**Minimum Events to Track:**
- Login/logout
- Content access
- Content completion
- Assessment start
- Assessment submission
- Assessment results
- Time on task

**Event Format (xAPI compatible):**
```json
{
  "actor": {
    "id": "learner-id",
    "name": "learner-name"
  },
  "verb": "completed|accessed|answered",
  "object": {
    "id": "content-id",
    "type": "content|assessment"
  },
  "result": {
    "score": number,
    "success": boolean,
    "duration": "ISO 8601 duration"
  },
  "timestamp": "ISO 8601 timestamp"
}
```

### 7.2 Analytics (Basic)

**Dashboards:**
1. **Learner Dashboard**
   - Personal progress
   - Mastery levels by concept
   - Time spent
   - Completion rate

2. **Educator Dashboard (if applicable)**
   - Class roster
   - Average progress
   - Struggling students alert
   - Completion statistics

---

## 8. Testing and Quality Assurance

### 8.1 Testing Requirements

- [ ] Unit tests for core functions (>70% code coverage)
- [ ] Integration tests for API endpoints
- [ ] UI/UX testing with target users
- [ ] Load testing (simulate target concurrent users)
- [ ] Security testing (vulnerability scan)
- [ ] Accessibility testing (WCAG compliance)

### 8.2 Pilot Deployment

**Recommended Approach:**
1. Start with 20-50 pilot users
2. Run for 2-4 weeks
3. Collect feedback
4. Measure key metrics:
   - User engagement (sessions per week, time per session)
   - Learning effectiveness (mastery gains)
   - System performance (response times, errors)
   - User satisfaction (survey, NPS)

---

## 9. Documentation

### 9.1 Required Documentation

- [ ] API documentation (OpenAPI/Swagger)
- [ ] User guide (learner-facing)
- [ ] Administrator guide
- [ ] Privacy policy
- [ ] Terms of service
- [ ] Accessibility statement
- [ ] Installation and deployment guide

### 9.2 Code Documentation

- [ ] README with setup instructions
- [ ] Architecture overview
- [ ] Database schema documentation
- [ ] API endpoint descriptions
- [ ] Configuration guide

---

## 10. Phase 1 Completion Checklist

### Infrastructure
- [ ] Application server deployed
- [ ] Database configured and accessible
- [ ] HTTPS enabled
- [ ] Backup system in place
- [ ] Monitoring and logging operational

### Core Functionality
- [ ] User authentication working
- [ ] Learner profile creation and management
- [ ] Content management system operational
- [ ] Basic recommendation engine functional
- [ ] Progress tracking implemented
- [ ] Assessment delivery working

### Compliance and Quality
- [ ] Privacy policy published
- [ ] Data protection measures in place
- [ ] Accessibility requirements met (WCAG 2.1 A minimum)
- [ ] Testing completed with >70% code coverage
- [ ] Documentation completed

### Pilot Deployment
- [ ] Pilot users onboarded
- [ ] Feedback collection mechanism in place
- [ ] Analytics dashboard operational
- [ ] Support process established

---

## 11. Success Metrics

**Phase 1 is considered successful when:**
- ✅ System is stable with <1% error rate
- ✅ 80%+ of pilot users complete onboarding
- ✅ Average session duration > 15 minutes
- ✅ Positive user feedback (NPS > 0)
- ✅ Measurable learning progress in pilot users
- ✅ All technical requirements met

---

## 12. Next Steps

Upon successful completion of Phase 1, proceed to:

**[PHASE 2: Intelligence](./PHASE2-implementation.md)**
- Advanced recommendation algorithms
- Machine learning integration
- Knowledge tracing
- Adaptive assessments
- Enhanced analytics

---

## Resources

### Reference Implementations
- GitHub: https://github.com/WIA-Official/adaptive-learning-reference
- Documentation: https://docs.wia-official.org/edu-003
- Community: https://community.wia-official.org

### Standards and Specifications
- xAPI: https://xapi.com/
- WCAG 2.1: https://www.w3.org/WAI/WCAG21/quickref/
- FERPA: https://www2.ed.gov/policy/gen/guid/fpco/ferpa/
- GDPR: https://gdpr.eu/

---

**弘益人間 (Benefit All Humanity)**

*© 2025 SmileStory Inc. / WIA*
*Licensed under Creative Commons Attribution 4.0 International*


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
