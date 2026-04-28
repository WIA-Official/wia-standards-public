# WIA-ai-content-moderation PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-ai-content-moderation
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-ai-content-moderation. The standard covers persistent
record shapes for AI-assisted content moderation deployed
on online platforms — moderation policy versions, content
items submitted for review, classifier outputs, human-
reviewer decisions, appeals, transparency disclosures (per
EU Digital Services Act 2022/2065 / DSA Article 15
transparency reports), and statutory escalation channels
(NCMEC CyberTipline for child sexual abuse material per
US 18 U.S.C. § 2258A and equivalent national reporting
authorities). The format is consumed by platform
operators, content provenance authentication services
(C2PA), trusted-flagger civil-society organisations, and
the regulators that supervise the operator's moderation
under the operating jurisdiction's content-governance
regime.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID)
- ISO/IEC 22989:2022 (AI concepts and terminology)
- ISO/IEC TR 24028:2020 (overview of trustworthiness in AI)
- ISO/IEC 27001:2022 (information security management)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- C2PA Content Credentials specification (canonical
  framework for content-provenance manifests; the operator's
  moderation pipeline consumes C2PA manifests where present)
- NIST AI Risk Management Framework (AI RMF 1.0) and
  NIST AI 600-1 GenAI Profile (cited for content-
  moderation alignment with AI risk management)
- EU Digital Services Act (Regulation (EU) 2022/2065;
  cited normatively for transparency-report obligations,
  notice-and-action mechanisms, trusted-flagger handling,
  and very-large-online-platform risk-assessment
  obligations)
- EU AI Act 2024 (Regulation (EU) 2024/1689; cited
  for the AI Act's intersect with moderation systems
  classified as high-risk under Annex III)
- ETSI TS 104 224 (cited where the operating environment
  consumes the standard's reference profile for trusted
  AI for online services)
- US 18 U.S.C. § 2258A (CSAM reporting obligation in the
  US; the operator's escalation path to NCMEC follows
  this statutory requirement)
- NCMEC CyberTipline reporting protocol (cited as the
  US CSAM-reporting authority)
- W3C Verifiable Credentials Data Model 2.0 (used in
  PHASE-4 for optional re-issuance)

---

## §1 Scope

This PHASE defines persistent shapes for the artefacts an
AI-content-moderation operator manages. Implementations
covered include:

- General-purpose social-media platform moderation.
- Video-sharing platform moderation.
- Marketplace and classifieds platform moderation.
- Online-gaming and chat-room moderation.
- User-generated-content community moderation (forum,
  Q&A, encyclopedia).
- Generative-AI output safety filtering (where the
  operator runs both a generative model and the
  moderation pipeline over its outputs).
- Live-streaming moderation (real-time pipelines with
  per-segment review).

CSAM detection and reporting are handled through the
NCMEC integration in PHASE-4 §6 with the statutory
discipline in PHASE-3 §8; this PHASE addresses the
record shapes, not the underlying classifier algorithms.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
programmeOperator    : string (institutional identifier of
                         the platform operator)
programmeRegistered  : string (ISO 8601 / RFC 3339)
platformClass        : enum ("social-media" |
                         "video-sharing" |
                         "marketplace-classifieds" |
                         "online-gaming-chat" |
                         "ugc-community-forum" |
                         "ugc-encyclopedia" |
                         "live-streaming" |
                         "generative-ai-output" |
                         "user-defined")
governingFrameworks  : array of enum ("EU-DSA-2022-2065" |
                         "EU-AI-Act-2024-Annex-III" |
                         "US-Section-230" |
                         "UK-Online-Safety-Act-2023" |
                         "KR-Information-and-Communications-Network-Act"
                         | "user-defined")
vlopDesignation      : boolean (per EU DSA Article 33,
                         very-large-online-platform
                         designation; true triggers
                         additional Article 34/35
                         risk-assessment obligations)
jurisdictionScope    : array of string (ISO 3166-1)
programmeStatus      : enum ("design" | "operating" |
                         "limited-rollout" | "frozen" |
                         "archived")
```

## §3 Moderation Policy Version Record

```
policyVersion:
  policyId           : string (uuidv7)
  programmeId        : string (uuidv7)
  publishedAt        : string (ISO 8601)
  effectiveFrom      : string (ISO 8601)
  effectiveUntil     : string (ISO 8601; absent until
                         superseded)
  policyArtefactRef  : string (content-addressed URI of
                         the rendered policy text in each
                         supported language; per DSA
                         Article 14 transparency in terms
                         and conditions)
  policyDigest       : string (SHA-256)
  changesummaryRef   : string (URI of the diff narrative
                         for users / regulators)
  approvedBy         : string (operator-internal approver
                         token; the operator's policy-
                         governance committee)
  supersededBy       : string (URI of the successor policy
                         version; absent for current)
```

## §4 Content Item Record

```
contentItem:
  contentId          : string (uuidv7)
  programmeId        : string (uuidv7)
  submittedAt        : string (ISO 8601)
  authorTokenRef     : string (opaque user token; clinical
                         author identity in the operator's
                         IDP)
  contentKind        : enum ("text-post" | "comment" |
                         "image" | "video" | "audio" |
                         "live-stream-segment" |
                         "marketplace-listing" |
                         "user-profile-attribute")
  contentArtefactRef : string (content-addressed URI of
                         the content; redacted of PII for
                         transparency-report aggregation)
  contentDigest      : string (SHA-256)
  c2paManifestRef    : string (URI of the C2PA Content
                         Credentials manifest, where
                         present)
  jurisdictionContext : string (ISO 3166-1; the
                         jurisdiction the content is
                         distributed under)
```

## §5 Classifier Output Record

```
classifierOutput:
  outputId           : string (uuidv7)
  contentRef         : string (content UUID)
  classifierVersion  : string (operator's classifier
                         model version)
  evaluatedAt        : string (ISO 8601)
  perPolicyScores    : array of object (per moderation-
                         policy category — e.g. hate
                         speech, harassment, sexual
                         content, violence, self-harm,
                         dangerous-organisation, CSAM-
                         indicator — with confidence
                         score and decision threshold)
  recommendedAction  : enum ("allow" | "limit-distribution"
                         | "label-context" | "remove" |
                         "remove-and-escalate-statutory" |
                         "human-review-required")
  modelCardRef       : string (URI of the classifier's
                         model card)
```

## §6 Human Reviewer Decision Record

```
reviewerDecision:
  decisionId         : string (uuidv7)
  contentRef         : string (content UUID)
  reviewerTokenRef   : string (opaque reviewer token;
                         clinical reviewer identity in the
                         operator's HR)
  decidedAt          : string (ISO 8601)
  policyVersionRef   : string (policy UUID the decision
                         is grounded in)
  decision           : enum ("allow" | "limit-distribution"
                         | "label-context" | "remove" |
                         "remove-and-account-suspend" |
                         "remove-and-statutory-escalate" |
                         "no-action-pending-additional-
                         context")
  rationaleRef       : string (URI of the redacted
                         rationale narrative)
  reviewerWellnessFlag : boolean (true when the operator's
                         wellness pipeline observed reviewer
                         indicators warranting time-off or
                         cohort rotation)
```

## §7 Appeal Record

Per DSA Article 20, users receive a statement of reasons
for moderation decisions and an internal complaint-handling
mechanism for appeals.

```
appeal:
  appealId           : string (uuidv7)
  contentRef         : string (content UUID)
  appellantTokenRef  : string (opaque user token)
  filedAt            : string (ISO 8601)
  appellantStatementRef : string (URI of the appellant's
                         statement)
  resolvedAt         : string (ISO 8601; absent until
                         resolved)
  resolutionDecision : enum ("upheld-original-decision" |
                         "reversed-content-restored" |
                         "reversed-with-modification" |
                         "escalated-to-out-of-court-dispute"
                         | "withdrawn-by-appellant")
  outOfCourtBodyRef  : string (URI of the out-of-court
                         dispute body; absent unless
                         escalated)
```

## §8 Statutory Escalation Record

```
statutoryEscalation:
  escalationId       : string (uuidv7)
  contentRef         : string (content UUID)
  escalatedAt        : string (ISO 8601)
  authorityRef       : enum ("ncmec-cybertipline" |
                         "national-csam-authority" |
                         "national-counter-terrorism-
                         authority" | "law-enforcement-
                         direct-request" |
                         "user-defined")
  escalationKind     : enum ("csam-mandatory-report" |
                         "ndc-non-consensual-intimate-
                         imagery" | "imminent-violence-
                         threat" | "credible-self-harm-
                         threat" | "terrorism-content" |
                         "court-order-takedown")
  acknowledgedAt     : string (ISO 8601; absent until
                         authority acknowledges)
  caseReference      : string (authority-issued case
                         reference)
```

## §9 Transparency Report Record

```
transparencyReport:
  reportId           : string (uuidv7)
  programmeId        : string (uuidv7)
  reportPeriodStart  : string (ISO 8601)
  reportPeriodEnd    : string (ISO 8601)
  reportRegime       : enum ("EU-DSA-Article-15" |
                         "EU-DSA-Article-42-VLOP-Risk-
                         Assessment" |
                         "operator-voluntary" |
                         "user-defined")
  reportArtefactRef  : string (URI of the rendered report)
  reportSchemaRef    : string (URI of the schema the
                         operator binds the report to —
                         the EU DSA transparency-report
                         template where applicable)
  publishedAt        : string (ISO 8601)
```

## §10 Hash Submission Record

```
hashSubmission:
  submissionId       : string (uuidv7)
  contentRef         : string (content UUID)
  submittedAt        : string (ISO 8601)
  databaseRef        : enum ("gifct-terrorism" |
                         "ncmec-photodna-csam" |
                         "stopncii-intimate" |
                         "operator-internal" |
                         "user-defined")
  hashAlgorithm      : enum ("photodna-perceptual" |
                         "pdq-perceptual" |
                         "tmk-perceptual-video" |
                         "sha-256" |
                         "user-defined")
  hashValue          : string (hex-encoded hash; the
                         hash is the perceptual fingerprint
                         that participating platforms can
                         use to match similar content
                         without exchanging the content
                         itself)
  acknowledgedAt     : string (ISO 8601; absent until
                         database acknowledges)
```

## §11 Crisis Period Record

```
crisisPeriod:
  crisisId           : string (uuidv7)
  programmeId        : string (uuidv7)
  declaredAt         : string (ISO 8601)
  expectedEndAt      : string (ISO 8601)
  crisisKind         : enum ("election-period" |
                         "mass-violence-event" |
                         "natural-disaster" |
                         "public-health-emergency" |
                         "civil-unrest" |
                         "user-defined")
  surgeReviewerCohort : integer (additional reviewer
                         hours per day)
  policyClarificationRef : string (URI of the operator's
                         crisis-period policy
                         clarification)
  postEventReviewRef : string (URI of the post-crisis
                         public review report; absent
                         until published)
```

## §12 Conformance

Implementations claiming PHASE-1 conformance emit each of
the records defined above for every operating programme
and honour the policy-version content-addressing rule per
§3.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-ai-content-moderation
- **Last Updated:** 2026-04-28
