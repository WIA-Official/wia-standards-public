# WIA-ai-survival-2026 PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-ai-survival-2026
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format
layer for WIA-ai-survival-2026. The standard covers
persistent record shapes for the lifecycle of an
AI-survival programme — the operator's organisational
identity and AI-deployment perimeter; the AI-system
inventory and risk-classification record under EU AI
Act Annex III; the AI-management-system (AIMS)
record under ISO/IEC 42001:2023; the workforce-
transition and human-oversight record; the existential-
risk and frontier-AI policy record (UN AI Advisory
Body Final Report 2024 + International AI Safety
Report 2025 + AI Safety Institutes); the safety-and-
security testing record (red-teaming, evaluation
benchmarks, capability elicitation); the incident
and near-miss reporting record (EU AI Act Art 73 +
US EO 14110 + KR AI 기본법); the supply-chain-AI-
integrity record; the dual-use-and-export-control
record; and the supervisory and consultation
correspondence record. Records are consumed by the
operator's board of directors, the operator's chief
AI / risk officer, the AI red-team and safety
function, the supervisory authority for the operating
jurisdiction (US AISI + EU AI Office + KR AI 안전
연구소), the international coordination bodies (UN
AI Advisory Body, GPAI), the workforce-transition
stakeholders, and external auditors.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID) and IETF RFC 4122 (UUID URN)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 42001:2023 (AI management system)
- ISO/IEC 22989:2022 (AI concepts and terminology)
- ISO/IEC 23053:2022 (framework for AI systems
  using ML)
- ISO/IEC 23894:2023 (AI risk management)
- ISO/IEC 24029-1:2021 + 24029-2:2023 (AI
  robustness — overview and via formal methods)
- ISO/IEC TR 24027:2021 (Bias in AI systems and
  AI-aided decision making)
- ISO/IEC TR 24028:2020 (Trustworthiness of AI
  systems)
- ISO/IEC TS 4213:2022 (AI system performance
  evaluation)
- ISO/IEC 38507:2022 (Governance implications of
  the use of AI by organizations)
- IETF RFC 8259 (JSON), RFC 9457 (Problem Details)
- NIST AI Risk Management Framework 1.0 + NIST AI
  600-1 GenAI Profile + NIST IR 8332 (RMF
  Implementation) + NIST IR 8259 (IoT-AI baseline)
- US Executive Order 14110 (Safe, Secure, and
  Trustworthy Development and Use of Artificial
  Intelligence) + successor executive guidance
- US OMB Memorandum M-24-10 (federal-agency AI use)
- US AI Safety Institute (AISI) at NIST + AISI
  Consortium voluntary commitments
- EU AI Act (Regulation (EU) 2024/1689) Articles 5
  (prohibited practices), 6 (high-risk
  classification), 7 (Annex III amendment), 8 to
  17 (high-risk obligations), 25 (provider
  obligations), 26 (deployer obligations), 27
  (Fundamental Rights Impact Assessment), 50
  (transparency), 51 to 55 (general-purpose AI
  models with systemic risk), 71 (high-risk system
  database), 72 (post-market monitoring), 73
  (serious-incident reporting), Annex III + Annex
  IV + Annex XI
- EU Code of Practice for General-Purpose AI Models
  (AI Act Article 56)
- UN AI Advisory Body Final Report (Governing AI
  for Humanity, September 2024) + UN GA Resolution
  on AI 2024
- International AI Safety Report 2025 (chaired by
  Yoshua Bengio under the AI Action Summit)
- OECD AI Principles (the OECD Recommendation of
  the Council on Artificial Intelligence 2019,
  updated 2024)
- UNESCO Recommendation on the Ethics of Artificial
  Intelligence (2021)
- ETSI ISG SAI (Securing Artificial Intelligence)
  GR / GS series — ETSI GR SAI 002 (Threat
  ontology), GR SAI 004 (Privacy considerations),
  GR SAI 005 (Mitigation strategy report), GR SAI
  006 (Hardware role), GS SAI 010 (Mitigation
  strategy)
- IEC 62443 series (Industrial automation and
  control systems security; cited where AI is
  embedded in OT)
- KR 인공지능 발전 및 신뢰 기반 조성 등에 관한 기본법
  (KR AI Basic Act, in force after 2025-01-21
  promulgation; the 22 January 2026 effective date
  for high-impact-AI is the reference horizon for
  KR-jurisdiction operators) + KR PIPC AI guidance
- ISO 17442 LEI for institutional identifiers

---

## §1 Scope

This PHASE defines persistent shapes for the artefacts
an AI-survival operator (an enterprise AI-deployer,
a frontier-AI provider, a critical-infrastructure
operator with embedded AI, a research consortium, a
public-sector AI deployer, a workforce-transition
support operator) maintains:

- The organisational AI-deployment perimeter record.
- The AI-system inventory and risk-classification
  record.
- The AIMS (AI management system) record.
- The workforce-transition and human-oversight
  record.
- The frontier-AI / existential-risk policy record.
- The safety-and-security testing record.
- The incident-and-near-miss record.
- The supply-chain-AI-integrity record.
- The dual-use-and-export-control record.
- The supervisory and consultation correspondence
  record.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name)
operatorRole         : enum ("enterprise-ai-deployer"
                       | "frontier-ai-provider" |
                       "critical-infrastructure-
                       operator-embedded-ai" |
                       "research-consortium" |
                       "public-sector-ai-deployer"
                       | "workforce-transition-
                       support" | "ai-safety-
                       institute" | "user-defined")
operatorJurisdiction : array of string (ISO 3166-1)
operatorLei          : string (ISO 17442)
governingFrameworks  : array of enum ("ISO-IEC-
                       42001-2023" |
                       "ISO-IEC-22989-2022" |
                       "ISO-IEC-23053-2022" |
                       "ISO-IEC-23894-2023" |
                       "ISO-IEC-24029-1-2021" |
                       "ISO-IEC-24029-2-2023" |
                       "ISO-IEC-TR-24027-2021" |
                       "ISO-IEC-TR-24028-2020" |
                       "ISO-IEC-TS-4213-2022" |
                       "ISO-IEC-38507-2022" |
                       "NIST-AI-RMF-1-0" |
                       "NIST-AI-600-1-GENAI" |
                       "NIST-IR-8332" |
                       "US-EO-14110" |
                       "US-OMB-M-24-10" |
                       "US-AISI-CONSORTIUM" |
                       "EU-AI-ACT-2024-1689" |
                       "EU-AI-ACT-COP-GPAI" |
                       "UN-AI-ADVISORY-BODY-2024" |
                       "INTL-AI-SAFETY-REPORT-2025"
                       | "OECD-AI-PRINCIPLES-2024"
                       | "UNESCO-AI-ETHICS-2021" |
                       "ETSI-ISG-SAI" |
                       "IEC-62443-AI-OT" |
                       "KR-AI-기본법-2026" |
                       "user-defined")
existentialRiskTier  : enum ("frontier-ai-systemic"
                       | "high-risk-annex-iii" |
                       "limited-transparency-art-50"
                       | "minimal-risk" | "user-
                       defined") (per EU AI Act
                       classification + International
                       AI Safety Report 2025
                       frontier-tier)
programmeStatus      : enum ("design" | "operating"
                       | "limited-rollout" |
                       "wind-down" | "archived")
```

## §3 AI-System Inventory Record

```
aiSystemRecord:
  systemId           : string (uuidv7)
  systemName         : string
  intendedPurpose    : string (the EU AI Act Article
                       3(12) intended purpose
                       declared by the provider)
  highRiskClassification : enum ("not-high-risk" |
                       "annex-iii-1-biometric" |
                       "annex-iii-2-critical-
                       infrastructure" | "annex-iii
                       -3-education" | "annex-iii-4
                       -employment" | "annex-iii-5-
                       essential-services" |
                       "annex-iii-6-law-enforcement"
                       | "annex-iii-7-migration" |
                       "annex-iii-8-justice" |
                       "annex-iii-democratic-process"
                       | "user-defined")
  gpaiClassification : enum ("not-gpai" | "gpai-not-
                       systemic" | "gpai-systemic-
                       risk" | "gpai-foss-not-
                       systemic")
  modelArtefactRef   : string (the WIA-generative-
                       ai-aligned model registry
                       reference where applicable)
  systemDeployerRef  : string (the deployer's legal
                       identity per EU AI Act Article
                       26)
  technicalDocumentationRef : string (URI of the
                       Annex IV + Annex XI technical
                       documentation set)
  fundamentalRightsImpactAssessmentRef : string (URI
                       of the Article 27 FRIA where
                       applicable)
```

## §4 AIMS Record (ISO/IEC 42001:2023)

```
aimsRecord:
  recordId           : string (uuidv7)
  scope              : string (the AIMS scope
                       declaration)
  contextOfTheOrganisation : string (URI of the
                       Clause 4 context narrative)
  leadershipCommitment : string (URI of the Clause
                       5 leadership-commitment
                       evidence)
  policiesRef        : array of string (the AI policy
                       set per Clause 5.2)
  aiObjectivesRef    : array of string (the
                       measurable AI objectives per
                       Clause 6.2)
  riskAssessmentRef  : string (URI of the AI risk
                       assessment per Clause 6.1.2 +
                       Annex B)
  aiSystemImpactAssessmentRef : string (URI of the
                       AI-system impact assessment
                       per Clause 6.1.4 + Annex C)
  managementReviewRef : string (URI of the most
                       recent Clause 9.3 management
                       review)
```

## §5 Workforce-Transition and Human-Oversight Record

```
workforceTransitionRecord:
  recordId           : string (uuidv7)
  transitionScope    : enum ("upskilling-incumbent
                       -workforce" | "reskilling-
                       displaced-workforce" |
                       "redeployment-internal" |
                       "exit-with-severance" |
                       "exit-with-job-search-
                       support" | "user-defined")
  affectedHeadcount  : integer
  programmeRef       : string (URI of the operator's
                       transition programme
                       narrative)
  partnershipsRef    : array of string (educational-
                       institution / public-employment
                       agency / sectoral-training-
                       fund partnerships)
  fundingSourceRef   : string (the operator's fund
                       allocation; statutory severance
                       compliance per the operating
                       jurisdiction)

humanOversightRecord:
  recordId           : string (uuidv7)
  systemRef          : string (PHASE-1 §3)
  oversightModel     : enum ("human-in-the-loop" |
                       "human-on-the-loop" |
                       "human-out-of-the-loop-
                       monitored" | "user-defined")
  oversightRoleRef   : string (the operator's
                       designated oversight role per
                       AI Act Article 14 + ISO/IEC
                       38507)
  competencyAttestationRef : string (URI of the
                       oversight personnel's training
                       and competency record)
  overrideCapability : object (the human override
                       latency budget + scope)
```

## §6 Frontier-AI / Existential-Risk Policy Record

For frontier-AI providers and operators of GPAI
systems with systemic risk:

```
frontierPolicyRecord:
  recordId           : string (uuidv7)
  riskTier           : enum ("frontier-ai-systemic"
                       | "near-frontier")
  responsibleScalingPolicyRef : string (URI of the
                       operator's responsible-scaling
                       / preparedness / responsible-
                       AI scaling policy)
  capabilityEvaluationsRef : array of string (URIs
                       of the cyber / CBRN / autonomy
                       / persuasion capability
                       evaluations)
  systemSafetyCommitmentsRef : string (URI of the
                       voluntary commitments — UK /
                       US AI Safety Institute
                       evaluations, Seoul AI Summit
                       commitments, AI Action Summit
                       Paris 2025 commitments)
  preDeploymentReviewRef : string (URI of the pre-
                       deployment review record per
                       AI Act Article 55)
  postDeploymentMonitoringRef : string (URI of the
                       post-deployment monitoring
                       per AI Act Article 72)
```

## §7 Safety-and-Security Testing Record

```
safetyTestRecord:
  recordId           : string (uuidv7)
  systemRef          : string
  testKind           : enum ("capability-benchmark"
                       | "robustness-test-iso-iec-
                       24029" | "fairness-test-
                       iso-iec-tr-24027" |
                       "trustworthiness-test-iso-
                       iec-tr-24028" | "adversarial
                       -red-team" | "cyber-tabletop"
                       | "cbrn-uplift-evaluation" |
                       "autonomy-evaluation" |
                       "persuasion-evaluation" |
                       "supply-chain-attack-test" |
                       "user-defined")
  conductedBy        : string (internal team or
                       external red-team firm)
  externalRedTeamRef : string (the external red-
                       team firm reference; absent
                       for internal-only)
  startedAt          : string (ISO 8601)
  completedAt        : string (ISO 8601)
  findingsRef        : string (URI of the findings
                       narrative)
  systemicRiskIndicatorsRef : array of string (URIs
                       of any systemic-risk
                       indicators raised per AI Act
                       Article 55(1)(a))
```

## §8 Incident and Near-Miss Record

```
incidentRecord:
  incidentId         : string (uuidv7)
  systemRef          : string
  reportedAt         : string (ISO 8601)
  reporterKind       : enum ("end-user" | "operator-
                       internal" | "downstream-
                       deployer" | "third-party-
                       researcher" | "regulator" |
                       "ai-safety-institute" |
                       "user-defined")
  incidentKind       : enum ("safety-failure-injury"
                       | "fundamental-rights-
                       violation" | "privacy-leak" |
                       "fabricated-output-causing-
                       harm" | "prompt-injection-
                       exploit" | "model-jailbreak"
                       | "supply-chain-compromise"
                       | "near-miss-no-impact" |
                       "user-defined")
  severityKind       : enum ("near-miss" | "minor"
                       | "moderate" | "serious" |
                       "critical")
  rootCauseRef       : string
  correctiveActions  : array of object
  euAiActArt73ReportRef : string (URI of the EU AI
                       Act Article 73 serious-
                       incident report; absent
                       unless reported)
  usAisiNotificationRef : string (URI of the US
                       AISI notification; absent
                       unless reported)
  krAiBasicActReportRef : string (URI of the KR AI
                       기본법 incident report under
                       2025/2026 reporting
                       discipline)
```

## §9 Supply-Chain-AI-Integrity Record

```
supplyChainIntegrity:
  recordId           : string (uuidv7)
  componentKind      : enum ("foundation-model" |
                       "fine-tune-adapter" |
                       "training-dataset" |
                       "evaluation-benchmark" |
                       "inference-runtime" |
                       "ml-framework" | "hardware-
                       accelerator" | "user-defined")
  componentRef       : string
  sbomRef            : string (URI of the CycloneDX
                       AI/ML SBOM declaration —
                       CycloneDX v1.6 ML profile)
  modelCardRef       : string (URI of the
                       HuggingFace Model Card or
                       equivalent)
  datasheetRef       : string (URI of the Datasheet
                       for Datasets)
  provenanceAttestationRef : string (URI of the
                       provenance attestation —
                       in-toto / Sigstore)
```

## §10 Dual-Use and Export-Control Record

```
dualUseRecord:
  recordId           : string (uuidv7)
  systemRef          : string
  exportControlClass : enum ("eu-dual-use-reg-2021-
                       821" | "us-ear-15-cfr-742" |
                       "us-itar-22-cfr-120" |
                       "wassenaar-arrangement-
                       cybertools" | "kr-대외무역법-
                       전략물자" | "n/a-not-export-
                       controlled" | "user-defined")
  technicalParameter : object (FLOPs threshold,
                       capability-class threshold,
                       per-jurisdiction control list
                       reference)
  destinationCountrySanctionsRef : string (the
                       country-level sanctions /
                       embargo screening reference)
  licenceReferences  : array of string (export
                       licence references)
```

## §11 Conformance

Implementations claiming PHASE-1 conformance maintain
the records defined above for every AI system within
the operating perimeter, exercise the AIMS discipline
under ISO/IEC 42001:2023, satisfy the EU AI Act +
NIST AI RMF + KR AI 기본법 obligations applicable to
the operator, and preserve the records under the
operating jurisdiction's recordkeeping discipline
(EU AI Act Article 18 ten-year retention for high-
risk systems; US OMB M-24-10 federal use-case
inventory annual; KR AI 기본법 retention horizon).

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-ai-survival-2026
- **Last Updated:** 2026-04-29
