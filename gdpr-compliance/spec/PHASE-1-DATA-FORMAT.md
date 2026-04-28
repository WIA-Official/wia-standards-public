# WIA-gdpr-compliance PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-gdpr-compliance
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-gdpr-compliance. The standard covers persistent record
shapes for organisations subject to the EU General Data
Protection Regulation (Regulation (EU) 2016/679 / GDPR)
and to the equivalent UK GDPR retained-EU-law regime —
controllers, processors, sub-processors, joint
controllers, the data-protection officer, the records of
processing activities (per GDPR Article 30), data subject
requests (Articles 15 to 22), consent records (Article 7),
international transfer mechanisms (Chapter V), data-
breach notifications (Articles 33 and 34), data-protection
impact assessments (Article 35), and supervisory-authority
correspondence (Articles 51 to 59). The format is
consumed by data-protection officers (DPO), supervisory
authorities (the EU's national Data Protection
Authorities and the UK Information Commissioner's
Office), the European Data Protection Board (EDPB), the
controller's joint controllers and processors, the
controller's external auditors, and the data subjects
themselves through the Article 15 right of access.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27018:2019 (PII in public clouds)
- ISO/IEC 27701:2019 (privacy information management
  system; the controller's PIMS aligns its records with
  the PIMS clauses where adopted)
- ISO/IEC 29100:2011 (privacy framework)
- ISO/IEC 29134:2017 (privacy impact assessment)
- IETF RFC 4122 (UUID URN)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- EU GDPR (Regulation (EU) 2016/679; cited normatively
  for every Article that this PHASE references)
- UK GDPR (the retained-EU-law version of GDPR plus the
  Data Protection Act 2018; cited where the operating
  jurisdiction is the UK)
- EU Law Enforcement Directive (Directive (EU) 2016/680;
  cited where the controller is a competent authority
  for criminal-investigation purposes)
- EU Standard Contractual Clauses (Commission Implementing
  Decision (EU) 2021/914; cited for the international
  transfer mechanism in Chapter V)
- EDPB Guidelines (cited as the European Data Protection
  Board's authoritative interpretive guidance, including
  Guidelines on consent, on transparency, on Article 6(1)
  legal bases, and on the international transfer toolkit)
- W3C Verifiable Credentials Data Model 2.0 (used in
  PHASE-4 for optional re-issuance of compliance
  attestations)

---

## §1 Scope

This PHASE defines persistent shapes for the artefacts a
controller (or, where applicable, a joint controller or a
processor) maintains under GDPR. Implementations covered
include:

- Single-establishment controllers operating from one
  EU Member State.
- Multi-establishment controllers with a lead supervisory
  authority designated under the one-stop-shop mechanism
  per Article 56.
- Controllers established outside the EU but offering
  goods or services to EU data subjects (Article 3(2)),
  who must designate a representative per Article 27.
- Joint controllers operating under an Article 26
  arrangement.
- Processors (Article 28) and sub-processors operating
  under data-processing agreements with the controller.
- UK-only controllers operating under UK GDPR with the
  ICO as supervisory authority.

GDPR Article 9 special-category data (health, biometric,
genetic, racial, religious, political-opinion, sexual-
orientation, trade-union-membership) is handled with the
record-shape extensions in §6; the additional safeguards
required by Article 9 are encoded in PHASE-3 §4.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
controllerName       : string (legal name of the
                       controller; the entity that
                       determines the purposes and means
                       of processing per GDPR Article
                       4(7))
controllerEstablishment : array of string (ISO 3166-1
                       country codes of the controller's
                       EU establishments)
leadSupervisoryAuthority : string (the lead DPA per
                       Article 56; absent for non-cross-
                       border processing)
representative       : object (Article 27 representative
                       reference; absent if the
                       controller is established in the
                       EU)
dpoReference         : string (the controller's
                       designated DPO contact reference,
                       per Article 37)
processingScope      : enum ("single-establishment" |
                       "multi-establishment-eu" |
                       "extra-territorial-art-3-2" |
                       "joint-controller" |
                       "processor-only" |
                       "uk-only-uk-gdpr" |
                       "user-defined")
governingFrameworks  : array of enum ("EU-GDPR" |
                       "UK-GDPR" |
                       "EU-Law-Enforcement-Directive" |
                       "Member-State-Derogation" |
                       "user-defined")
programmeStatus      : enum ("design" | "operating" |
                       "limited-rollout" |
                       "wind-down" | "archived")
```

## §3 Records of Processing Activities (Article 30)

Article 30 obligates controllers (and processors) to
maintain records of processing activities. The record
shape:

```
processingActivity:
  activityId         : string (uuidv7)
  programmeId        : string (uuidv7)
  controllerOrProcessor : enum ("controller" |
                       "joint-controller" | "processor")
  activityName       : string
  purpose            : string (the purpose of the
                       processing; specific, explicit,
                       legitimate per Article 5(1)(b))
  legalBasis         : enum ("art-6-1-a-consent" |
                       "art-6-1-b-contract" |
                       "art-6-1-c-legal-obligation" |
                       "art-6-1-d-vital-interest" |
                       "art-6-1-e-public-task" |
                       "art-6-1-f-legitimate-interest")
  specialCategoryBasis : enum ("art-9-2-a-explicit-
                       consent" | "art-9-2-b-employment"
                       | "art-9-2-c-vital-interest" |
                       "art-9-2-d-not-for-profit" |
                       "art-9-2-e-publicly-disclosed" |
                       "art-9-2-f-legal-claim" |
                       "art-9-2-g-substantial-public-
                       interest" | "art-9-2-h-health-
                       care" | "art-9-2-i-public-
                       health" | "art-9-2-j-archiving")
                       (absent unless special-category
                       data is processed)
  dataSubjectCategory : array of enum ("customer" |
                       "employee" | "applicant" |
                       "child-under-16" | "patient" |
                       "research-participant" |
                       "platform-user" | "user-defined")
  personalDataCategory : array of string (controller's
                       data dictionary categories
                       cross-walked to the EDPB
                       transparency-guidance taxonomy)
  recipientCategory  : array of string (joint
                       controllers, processors, third
                       parties, public authorities;
                       Article 30(1)(d))
  internationalTransfer : object (Chapter V mechanism
                       reference; absent if no transfer)
  retentionPolicy    : string (retention horizon and
                       criteria, Article 30(1)(f))
  securityMeasures   : array of string (Article 32
                       technical-and-organisational
                       measure cross-references)
  dpiaReference      : string (PHASE-1 §8 record
                       reference; absent if Article
                       35 DPIA not triggered)
```

## §4 Data Subject Request Record (Articles 15 to 22)

```
dataSubjectRequest:
  requestId          : string (uuidv7)
  programmeId        : string (uuidv7)
  receivedAt         : string (ISO 8601)
  requestKind        : enum ("art-15-access" |
                       "art-16-rectification" |
                       "art-17-erasure" |
                       "art-18-restriction" |
                       "art-19-recipient-notification" |
                       "art-20-portability" |
                       "art-21-objection" |
                       "art-22-automated-decision-
                       review")
  dataSubjectIdentity : string (the controller's
                       identity-verification record
                       reference; verified per the
                       controller's Article 12(6)
                       discipline)
  responseDueAt      : string (ISO 8601; the Article 12(3)
                       one-month deadline plus any
                       Article 12(3) two-month extension
                       declared)
  responseExtensionReason : string (the controller's
                       documented reason for an Article
                       12(3) extension; absent unless
                       extension declared)
  responseStatus     : enum ("pending" | "fulfilled" |
                       "partially-fulfilled" |
                       "refused" | "subject-withdrew")
  responseRationale  : string (URI of the response
                       narrative; refused responses cite
                       the Article 12(5) or Article 23
                       basis explicitly)
  responseDeliveredAt : string (ISO 8601; absent until
                       delivery)
```

## §5 Consent Record (Article 7)

```
consentRecord:
  consentId          : string (uuidv7)
  programmeId        : string (uuidv7)
  dataSubjectIdentity : string (identity reference)
  capturedAt         : string (ISO 8601)
  consentText        : string (URI of the consent
                       statement as presented to the
                       data subject; the EDPB consent
                       guidelines require freely-given,
                       specific, informed and
                       unambiguous statements)
  granularPurposes   : array of object (per-purpose
                       opt-in flags so that bundled
                       consent — prohibited under the
                       EDPB consent guidelines — is
                       avoided by design)
  parentalConsent    : boolean (true when the data
                       subject is under the operating
                       jurisdiction's Article 8 age
                       threshold and parental consent
                       was captured)
  withdrawalAt       : string (ISO 8601; absent until
                       withdrawn)
  withdrawalChannel  : string (URI of the withdrawal
                       channel — under Article 7(3) the
                       channel must be as easy as the
                       capture channel)
```

## §6 Special-Category Data Annotation

Special-category data (Article 9) records require an
extension annotation:

```
specialCategoryAnnotation:
  recordRef          : string (the PHASE-1 §3 activity
                       reference)
  category           : enum ("health" | "biometric" |
                       "genetic" | "racial-ethnic" |
                       "religious" | "political-opinion"
                       | "sexual-orientation" | "trade-
                       union-membership" | "criminal-
                       record")
  art9Basis          : string (Article 9(2)(a) to (j)
                       basis — duplicating PHASE-1 §3
                       for cross-validation)
  memberStateLawRef  : string (the operating Member
                       State's national derogation, per
                       Article 9(4); absent for bases
                       that do not require national law)
  additionalSafeguards : array of string (PHASE-3 §4
                       safeguard cross-references)
```

## §7 International-Transfer Record (Chapter V)

```
internationalTransfer:
  transferId         : string (uuidv7)
  programmeId        : string (uuidv7)
  destinationCountry : string (ISO 3166-1)
  transferMechanism  : enum ("art-45-adequacy" |
                       "art-46-2-c-scc-controller-to-
                       controller" | "art-46-2-c-scc-
                       controller-to-processor" |
                       "art-46-2-c-scc-processor-to-
                       processor" | "art-46-2-d-scc-
                       processor-to-controller" |
                       "art-46-2-b-bcr" |
                       "art-49-derogation" |
                       "user-defined")
  transferImpactAssessmentRef : string (URI of the
                       transfer-impact assessment
                       performed against the
                       destination country's third-
                       country surveillance regime, per
                       the EDPB Recommendations)
  supplementaryMeasures : array of string (technical
                       and organisational supplementary
                       measures applied to the transfer
                       per the EDPB Recommendations on
                       supplementary measures)
  effectiveFrom      : string (ISO 8601)
  effectiveUntil     : string (ISO 8601; absent until
                       superseded)
```

## §8 Data-Protection Impact Assessment Record (Article 35)

```
dpiaRecord:
  dpiaId             : string (uuidv7)
  activityRef        : string (PHASE-1 §3 activity
                       reference)
  triggeringFactor   : enum ("art-35-3-a-systematic-
                       evaluation" | "art-35-3-b-
                       large-scale-special-category" |
                       "art-35-3-c-public-monitoring" |
                       "supervisory-authority-list" |
                       "controller-discretionary")
  conductedAt        : string (ISO 8601)
  dpoConsultedAt     : string (ISO 8601; per Article
                       35(2))
  residualRiskAssessment : string (URI of the residual-
                       risk narrative)
  priorConsultationRef : string (Article 36 prior-
                       consultation correspondence
                       reference; present when
                       residual high risk persists
                       after mitigation)
```

## §9 Data-Breach Notification Record (Articles 33 and 34)

```
breachNotification:
  notificationId     : string (uuidv7)
  programmeId        : string (uuidv7)
  detectedAt         : string (ISO 8601)
  awareAt            : string (ISO 8601; the Article 33
                       72-hour clock starts from this
                       moment)
  saNotifiedAt       : string (ISO 8601; absent if
                       Article 33(1) "unlikely to result
                       in a risk" applies)
  saNotificationDelayReason : string (the Article 33(1)
                       reason for delay beyond 72 hours;
                       absent unless delay declared)
  dataSubjectNotifiedAt : string (ISO 8601; absent
                       unless Article 34 high-risk
                       threshold met or supervisory
                       authority required notification)
  affectedDataSubjectCount : integer (approximate count;
                       Article 33(3)(a))
  breachKind         : enum ("confidentiality-loss" |
                       "integrity-loss" |
                       "availability-loss")
  containmentNarrativeRef : string (URI of the
                       containment-and-recovery
                       narrative)
  remediationNarrativeRef : string (URI of the
                       lessons-learned narrative;
                       absent until remediation
                       complete)
```

## §10 Supervisory-Authority Correspondence Record

```
saCorrespondence:
  correspondenceId   : string (uuidv7)
  programmeId        : string (uuidv7)
  saIdentity         : string (the supervisory
                       authority's identity — lead DPA
                       or other DPA per Article 56 / 60)
  correspondenceKind : enum ("inquiry" | "investigation"
                       | "complaint-relayed" | "audit"
                       | "guidance-request" |
                       "voluntary-notification" |
                       "art-58-2-corrective-measure")
  receivedAt         : string (ISO 8601)
  responseDueAt      : string (ISO 8601)
  responseDeliveredAt : string (ISO 8601; absent until
                       delivery)
  outcomeKind        : enum ("closed-no-action" |
                       "advisory-comment" |
                       "warning-issued" |
                       "reprimand-issued" |
                       "compliance-order-issued" |
                       "ban-on-processing-issued" |
                       "fine-imposed" |
                       "open-pending")
  outcomeReferenceRef : string (URI of the supervisory
                       authority's decision document)
```

## §11 Joint-Controller and Processor Record (Articles 26 and 28)

```
controllerArrangement:
  arrangementId      : string (uuidv7)
  programmeId        : string (uuidv7)
  arrangementKind    : enum ("art-26-joint-controller"
                       | "art-28-processor-dpa" |
                       "art-28-sub-processor-dpa")
  counterpartyRef    : string (counterparty legal
                       identifier)
  contractRef        : string (URI of the executed
                       contract / DPA)
  effectiveFrom      : string (ISO 8601)
  effectiveUntil     : string (ISO 8601; absent until
                       superseded)
  responsibilityMatrixRef : string (URI of the Article
                       26(1) "essence" published to
                       data subjects; absent for
                       processor DPAs)
  auditFrequency     : enum ("annual" | "biennial" |
                       "ad-hoc-on-request" |
                       "user-defined")
```

## §12 Conformance

Implementations claiming PHASE-1 conformance emit each of
the records defined above for every operating programme,
maintain the Article 30 records of processing activities,
honour the Article 12(3) deadlines for data subject
requests, observe the Article 33 72-hour breach clock,
and preserve correspondence with the lead supervisory
authority for the operating retention horizon.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-gdpr-compliance
- **Last Updated:** 2026-04-28
