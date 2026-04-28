# WIA-content-ai PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-content-ai
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format
layer for WIA-content-ai. The standard covers
persistent record shapes for the lifecycle of an
AI-assisted content production, distribution, and
moderation programme — the content provider's
identity and platform inventory; the AI-assisted
content-generation transcript; the content-
provenance record under C2PA Content Credentials;
the synthetic-and-deepfake disclosure record under
EU AI Act Article 50; the content-moderation
classifier-and-decision record; the appeals and
out-of-court dispute settlement record under EU DSA
Articles 20 to 21; the trusted-flagger record under
EU DSA Article 22; the transparency-report record
under EU DSA Article 24; the child-safety record
under COPPA / UK AADC / KR Online Safety; the
accessibility record under WCAG 2.2 + EU EAA + ADA
Title III; the model-evaluation and red-team record;
the rights-holder copyright record; and the
supervisory and law-enforcement correspondence
record. WIA-content-ai is distinct from WIA-
generative-ai (which governs the model substrate);
this standard governs the deployed content-platform
surface that distributes AI-assisted content to
end-users at scale.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID) and IETF RFC 4122 (UUID URN)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 22989:2022 (AI concepts and terminology)
- ISO/IEC 23053:2022 (framework for AI systems
  using ML)
- ISO/IEC 42001:2023 (AI management system)
- IETF RFC 8259 (JSON), RFC 9457 (Problem Details)
- C2PA (Coalition for Content Provenance and
  Authenticity) Content Credentials specification
  v1.4 + Trust List + Manifest Store
- W3C Verifiable Credentials Data Model 2.0
- EU AI Act (Regulation (EU) 2024/1689) Articles 5
  (prohibited practices including manipulative AI),
  50 (transparency obligations for AI systems
  intended to interact with natural persons /
  generate synthetic content / emotion-recognition /
  deepfake), 51 to 55 (general-purpose AI models)
- EU Digital Services Act (Regulation (EU) 2022/2065)
  Articles 14 (transparency reporting), 16 (notice-
  and-action), 17 (statement of reasons), 20 (internal
  complaint-handling), 21 (out-of-court dispute
  settlement), 22 (trusted flaggers), 23 (suspension
  of users), 24 (transparency reporting), 25 (online
  interface design — dark-patterns), 26 (advertising
  transparency), 27 (recommender systems), 28 (online
  protection of minors), 33 to 43 (very-large-online-
  platform / very-large-online-search-engine
  obligations), 70 to 88 (enforcement)
- EU Empowering Consumers for the Green Transition
  Directive ((EU) 2024/825) for AI-assisted green-
  claims content
- US Section 230 of the Communications Decency Act
  (47 USC 230) — the safe-harbour baseline for US-
  jurisdiction platforms
- US COPPA (Children's Online Privacy Protection
  Act, 15 USC 6501-6506; 16 CFR Part 312)
- US California SB-1001 (Bot Disclosure)
- US California Online Privacy Protection Act
  (CalOPPA) and CCPA / CPRA
- UK Age Appropriate Design Code (the ICO's
  Children's Code)
- UK Online Safety Act 2023 + Ofcom enforcement
- KR PIPA (개인정보 보호법) and KR 청소년보호법
  (Youth Protection Act)
- KR 정보통신망법 + KR 방송통신심의위원회 + KR
  방송통신위원회 (KCC) deliberation regime
- W3C WCAG 2.2 + ATAG 2.0 + UAAG 2.0
- EU European Accessibility Act (Directive (EU)
  2019/882) + EN 301 549
- US ADA Title III + US Section 508 (29 USC 794d)
- ETSI TS 104 224 (AI in media — manifest format
  for AI-generated content)
- IPTC Photo Metadata 2024 (the news-industry
  metadata schema for AI-generated imagery)
- NIST AI 600-1 (Generative AI Profile of NIST AI
  RMF 1.0)
- UNESCO Recommendation on the Ethics of Artificial
  Intelligence (2021)

---

## §1 Scope

This PHASE defines persistent shapes for the artefacts
a content-platform operator (a social-media platform,
a video-sharing platform, a news publisher, an online
marketplace, a search engine, a chat / messaging
service, a creator-tool platform, an AI-assisted
content production studio, or an aggregator-of-creator
content) maintains:

- The platform-and-inventory record.
- The AI-assisted content-generation transcript.
- The content-provenance manifest (C2PA).
- The synthetic / deepfake disclosure record.
- The content-moderation decision record.
- The notice-and-action record (DSA Article 16).
- The appeals and out-of-court dispute settlement
  record (DSA Articles 20-21).
- The trusted-flagger record (DSA Article 22).
- The transparency-report record (DSA Article 24).
- The child-safety record.
- The accessibility-conformance record.
- The model-evaluation, red-team, and incident
  record.
- The rights-holder copyright correspondence record.
- The supervisory and law-enforcement correspondence
  record.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name)
operatorRole         : enum ("social-media-platform"
                       | "video-sharing-platform" |
                       "news-publisher" | "online-
                       marketplace" | "search-engine"
                       | "chat-messaging-service" |
                       "creator-tool-platform" |
                       "ai-content-studio" |
                       "aggregator" | "user-defined")
operatorJurisdiction : array of string (ISO 3166-1)
governingFrameworks  : array of enum ("EU-AI-ACT-
                       2024-1689" | "EU-AI-ACT-ART-
                       50-TRANSPARENCY" |
                       "EU-DSA-2022-2065" |
                       "EU-DSA-VLOP-VLOSE" |
                       "EU-EMPOWERING-CONSUMERS-
                       2024-825" |
                       "US-SECTION-230-47-USC-230"
                       | "US-COPPA-15-USC-6501" |
                       "US-CA-SB-1001-BOT-
                       DISCLOSURE" | "US-CCPA-CPRA"
                       | "UK-AADC" |
                       "UK-ONLINE-SAFETY-ACT-2023"
                       | "KR-PIPA" |
                       "KR-청소년보호법" |
                       "KR-정보통신망법" |
                       "KR-KCC-방송통신위원회" |
                       "C2PA-CONTENT-CREDENTIALS-
                       1-4" | "ETSI-TS-104-224" |
                       "IPTC-PHOTO-METADATA-2024" |
                       "NIST-AI-600-1" |
                       "ISO-IEC-42001" |
                       "WCAG-2-2-LEVEL-AA" |
                       "EU-EAA-2019-882" |
                       "US-ADA-TITLE-III" |
                       "user-defined")
vlopVloseDesignation : enum ("not-designated" |
                       "vlop-designated" | "vlose-
                       designated")
                       (per EU DSA Article 33 Very
                       Large Online Platform / Very
                       Large Online Search Engine
                       designation by the Commission
                       above the 45-million-monthly-
                       active-users threshold)
programmeStatus      : enum ("design" | "operating"
                       | "limited-rollout" |
                       "wind-down" | "archived")
```

## §3 Content Generation Transcript Record

```
contentTranscript:
  transcriptId       : string (uuidv7)
  programmeRef       : string
  authoringIdentity  : string (the human author or
                       AI-system identity that
                       produced the content)
  generationKind     : enum ("human-only" | "ai-
                       assisted" | "ai-fully-
                       generated" | "ai-modified-
                       human-supplied" | "user-
                       defined")
  modelRef           : string (the WIA-generative-ai
                       model reference where AI was
                       used; absent for human-only)
  generatedAt        : string (ISO 8601)
  promptList         : array of object (the prompts
                       supplied by the author)
  outputList         : array of object (the outputs
                       returned by the model)
  retainedSafetyDecisions : array of object (the
                       upstream-model safety-filter
                       decisions surfaced)
```

## §4 C2PA Content Credentials Manifest Record

```
contentCredentials:
  credentialId       : string (uuidv7)
  contentRef         : string (the content asset
                       reference — image, video,
                       audio, or text URI)
  c2paManifestRef    : string (URI of the C2PA
                       Content Credentials manifest
                       v1.4)
  claimGeneratorRef  : string (the operator's claim
                       generator identity)
  contentBindings    : array of object (the
                       cryptographic binding between
                       the manifest and the content
                       — for images / video this is
                       embedded JUMBF / XMP; for
                       text this is the side-car
                       URL)
  signingKeyRef      : string (the operator's
                       signing key registered with
                       the C2PA trust list)
  parentManifestRef  : string (the upstream
                       manifest the operator's
                       content was derived from;
                       absent for original content)
```

## §5 Synthetic and Deepfake Disclosure Record

The disclosure record encodes the EU AI Act Article
50 transparency obligations:

```
disclosureRecord:
  disclosureId       : string (uuidv7)
  contentRef         : string
  disclosureKind     : enum ("ai-generated" |
                       "ai-modified" | "deepfake" |
                       "emotion-recognition" |
                       "biometric-categorisation" |
                       "user-defined")
  art50Subparagraph  : enum ("art-50-1-interaction-
                       with-ai" | "art-50-2-
                       generative-ai-output" |
                       "art-50-3-emotion-recognition
                       -biometric-categorisation" |
                       "art-50-4-deepfake-or-
                       artificially-generated-
                       content")
  disclosureLanguage : array of string (BCP 47
                       language codes the disclosure
                       is delivered in — the
                       operator emits in the user's
                       UI locale)
  disclosurePresentation : object (the visual /
                       auditory disclosure — banner,
                       watermark, label,
                       voiceover annotation)
  effectiveFrom      : string (ISO 8601)
```

## §6 Content-Moderation Decision Record (DSA Article
       17 statement of reasons)

```
moderationDecision:
  decisionId         : string (uuidv7)
  contentRef         : string
  triggerKind        : enum ("user-report-art-16-
                       notice" | "trusted-flagger-
                       art-22" | "automated-detection"
                       | "law-enforcement-order-
                       art-9" | "voluntary-review" |
                       "user-defined")
  classifierVersion  : string (the operator's
                       classifier model version)
  classifierLabel    : string (the classifier's
                       label and confidence)
  decisionKind       : enum ("no-action" | "remove-
                       or-disable-access" |
                       "restrict-visibility" |
                       "demote-rank" | "interstitial-
                       warning" | "age-gate" |
                       "monetisation-suspended" |
                       "account-suspension" |
                       "user-defined")
  legalGroundRef     : string (the legal-basis-
                       and-policy reference the
                       operator relies on — DSA Art
                       9 / Member-State law / the
                       operator's terms-of-service
                       clause)
  statementOfReasonsRef : string (URI of the DSA
                       Article 17 statement of
                       reasons delivered to the
                       affected user)
  decidedAt          : string (ISO 8601)
  reviewerRef        : string (absent for fully
                       automated decisions)
```

## §7 Notice-and-Action Record (DSA Article 16)

```
noticeRecord:
  noticeId           : string (uuidv7)
  contentRef         : string
  noticeKind         : enum ("art-16-user-notice" |
                       "art-22-trusted-flagger" |
                       "art-9-removal-order" |
                       "user-defined")
  noticeSubmittedBy  : string (the user identity or
                       the trusted-flagger identity
                       or the issuing authority)
  noticeReceivedAt   : string (ISO 8601)
  noticeBody         : object (the structured-notice
                       payload per DSA Article 16(2))
  outcomeKind        : enum ("acted-on-content" |
                       "rejected-as-baseless" |
                       "forwarded-to-authority" |
                       "pending-review")
  resolutionAt       : string (ISO 8601; absent
                       until resolved)
```

## §8 Internal Appeal and Out-of-Court Dispute Record
       (DSA Articles 20 + 21)

```
appealRecord:
  appealId           : string (uuidv7)
  underlyingDecisionRef : string
  appellantIdentity  : string
  appealLodgedAt     : string (ISO 8601)
  internalReviewOutcomeRef : string (URI of the DSA
                       Article 20 internal-review
                       outcome)
  internalReviewDecidedAt : string (ISO 8601)
  oodsBody           : string (DSA Article 21
                       certified out-of-court
                       dispute settlement body
                       reference; absent unless
                       escalated)
  oodsOutcomeRef     : string (URI of the OODS
                       outcome; absent until
                       received)
```

## §9 Trusted-Flagger and Transparency-Report Record

```
trustedFlagger:
  flaggerId          : string (uuidv7)
  flaggerLegalEntity : string
  certifyingDscRef   : string (the Digital Services
                       Coordinator that certified the
                       trusted flagger)
  domainsCovered     : array of string (the
                       categories of illegal content
                       the flagger is certified for)
  registeredAt       : string (ISO 8601)
  suspendedAt        : string (ISO 8601; absent
                       until suspended)

transparencyReport:
  reportId           : string (uuidv7)
  reportingPeriod    : object (the H1 / H2 reporting
                       window per DSA Article 24)
  reportPublishedAt  : string (ISO 8601)
  reportRef          : string (URI of the DSA
                       Article 24 / Article 42
                       transparency report)
```

## §10 Child-Safety and Accessibility Records

```
childSafetyRecord:
  recordId           : string (uuidv7)
  applicableRegime   : enum ("us-coppa-13-and-under"
                       | "uk-aadc" |
                       "kr-청소년보호법" |
                       "eu-dsa-art-28-online-
                       protection-of-minors" |
                       "user-defined")
  ageVerificationRef : string (URI of the age-
                       verification or assurance
                       evidence; for COPPA verifiable
                       parental consent record;
                       for UK AADC the age-assurance
                       evidence)
  defaultPrivacySettings : object (the default
                       privacy and safety settings
                       applied to minor accounts)

accessibilityRecord:
  recordId           : string (uuidv7)
  conformanceLevel   : enum ("wcag-2-2-level-a" |
                       "wcag-2-2-level-aa" |
                       "wcag-2-2-level-aaa")
  applicableRegime   : array of enum ("us-ada-title-
                       iii" | "us-section-508" |
                       "eu-eaa-2019-882" |
                       "eu-en-301-549" |
                       "kr-장애인차별금지법" |
                       "user-defined")
  conformanceReportRef : string (URI of the
                       accessibility-conformance
                       report)
  remediationLogRef  : string (URI of the
                       remediation log for non-
                       conformant items)
```

## §11 Conformance

Implementations claiming PHASE-1 conformance maintain
the records defined above for the operator's content-
platform surface, exercise the C2PA Content
Credentials manifest emission for AI-generated
content, exercise the EU AI Act Article 50
transparency obligations, exercise the DSA Article
14-28 obligations where the operator is in scope,
and preserve the records under the operating
jurisdiction's recordkeeping discipline (DSA
Article 24 transparency-report retention; KR PIPA
retention; the operator's internal three-to-five-
year retention).

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-content-ai
- **Last Updated:** 2026-04-28
