# WIA-ci-octave PHASE 1 — Data Format Specification

**Standard:** WIA-ci-octave
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format
layer for WIA-ci-octave. The standard covers
the persistent record shapes that an
information-security risk-management operator
(an enterprise CISO office, a public-sector
agency information-security team, a critical-
infrastructure operator, a healthcare-IT risk
manager, a financial-services risk officer, a
managed-security-services provider, a third-
party-risk-management programme operator)
maintains when running the OCTAVE (Operationally
Critical Threat, Asset, and Vulnerability
Evaluation) Allegro / FORTE methodology
developed by the CMU Software Engineering
Institute, anchored to ISO/IEC 27005 information-
security risk management, NIST SP 800-30 risk-
assessment guidance, and ISO 31000 enterprise
risk management. Records are consumed by the
risk owner accepting or treating the residual
risk, by the audit committee approving the
risk-treatment plan, by the certification body
auditing the operator's ISO/IEC 27001 ISMS, by
the internal-audit function running the second-
line review, and by the supervisory regulator
enforcing the operator's sectoral information-
security obligations.

References (CITATION-POLICY ALLOW only):

- CMU Software Engineering Institute (SEI)
  CERT Division — OCTAVE Allegro
  (Carnegie Mellon University SEI Technical
  Report CMU/SEI-2007-TR-012) and OCTAVE FORTE
  (CMU/SEI-2020-TR-002), the operationally-
  critical-threat-asset-and-vulnerability-
  evaluation methodologies authoritatively
  published by the SEI
- CMU SEI CERT Resilience Management Model
  (CERT-RMM) v1.2
- CMU SEI Insider Threat Programme reference
- ISO/IEC 27001:2022 (information security
  management systems — requirements)
- ISO/IEC 27002:2022 (information security
  controls)
- ISO/IEC 27005:2022 (information security
  risk management)
- ISO 31000:2018 (risk management —
  guidelines), ISO/IEC 31010:2019 (risk
  assessment techniques)
- ISO/IEC 27017:2015 (cloud-services
  information-security controls), ISO/IEC
  27018:2019 (cloud-services personal-data
  protection)
- ISO/IEC 27036 series (information security
  for supplier relationships) — ISO/IEC
  27036-1:2021, ISO/IEC 27036-2:2022, ISO/IEC
  27036-3:2023, ISO/IEC 27036-4:2016
- NIST SP 800-30 Rev. 1 (Guide for Conducting
  Risk Assessments), NIST SP 800-37 Rev. 2
  (Risk Management Framework — RMF), NIST SP
  800-53 Rev. 5 (Security and Privacy Controls
  for Information Systems and Organizations)
- NIST Cybersecurity Framework v2.0 (CSF 2.0)
- US FIPS PUB 199 (Standards for Security
  Categorization of Federal Information and
  Information Systems), FIPS PUB 200 (Minimum
  Security Requirements for Federal Information
  and Information Systems)
- US FedRAMP authorisation
- EU NIS2 Directive (Directive (EU) 2022/2555),
  EU DORA Regulation (EU) 2022/2554, EU CRA
  Regulation (EU) 2024/2847
- ENISA Threat Landscape, ENISA Risk Assessment
  Methodology
- Open FAIR (Factor Analysis of Information
  Risk) — published by The Open Group
- IEC 62443-3-2:2020 (security risk assessment
  for system design — for the OT-system
  binding)
- KR 정보통신망 이용촉진 및 정보보호 등에 관한
  법률, KR 개인정보 보호법, KR 전자금융거래법, KR
  중요정보통신기반시설 보호법 (Critical Information
  Infrastructure Protection Act)
- KR-ISMS-P (Information Security Management
  System and Personal Information Management
  System) certification scheme operated by KISA
- IETF RFC 8259 (JSON), RFC 4122 (UUID), ISO
  8601 (date-time)
- W3C Verifiable Credentials Data Model v2.0

---

## §1 Scope

This PHASE defines persistent shapes for the
artefacts exchanged when an organisation runs
an OCTAVE Allegro / FORTE risk-evaluation
cycle, anchors the cycle to the ISO/IEC 27005
risk-management process, applies the NIST SP
800-30 likelihood-and-impact discipline, and
publishes the resulting risk-treatment plan to
the audit committee and the regulator.
Implementations covered include:

- An enterprise CISO office running the
  annual OCTAVE Allegro cycle covering the
  organisation's critical-asset inventory,
  threat-and-vulnerability evaluation, and
  risk-treatment plan.
- A public-sector agency running the OCTAVE
  FORTE cycle as the agency's risk-management
  operating model under the agency's
  information-security policy and the NIST RMF
  authorisation envelope.
- A critical-infrastructure operator (an
  electric-utility operator, a water-utility
  operator, a transportation-system operator)
  running the OCTAVE FORTE cycle anchored to
  the EU NIS2 Directive's risk-management
  obligations or the equivalent national
  legislation.
- A healthcare-IT risk manager running the
  OCTAVE Allegro cycle anchored to the
  HIPAA Security Rule's risk-analysis
  requirement (45 CFR § 164.308(a)(1)) where
  applicable, or to the equivalent national
  health-information-security framework.
- A financial-services risk officer running
  the OCTAVE FORTE cycle anchored to the
  EU DORA Regulation's ICT-risk-management
  framework or the equivalent national
  financial-regulator framework (the US
  Federal Financial Institutions Examination
  Council FFIEC IT Examination Handbook).
- A managed-security-services provider
  delivering the OCTAVE Allegro evaluation as
  a contracted service to a customer
  organisation, with the per-engagement
  contract envelope and the per-engagement
  evidence-handling discipline.
- A third-party-risk-management programme
  operator running the OCTAVE Allegro
  evaluation on a supplier engagement under
  the ISO/IEC 27036 supplier-relationship
  discipline.

The OCTAVE Allegro per-asset evaluation
envelope, the OCTAVE FORTE programme-level
governance envelope, the ISO/IEC 27005
process-mapping envelope, and the NIST SP
800-30 likelihood-and-impact envelope receive
distinct encodings in this PHASE; the
additional safeguards required by each
sectoral framework are encoded in PHASE-3 §3.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name of the
                       operator — enterprise,
                       public-sector agency,
                       critical-infrastructure
                       operator, healthcare-IT
                       provider, financial-
                       services firm, MSSP,
                       third-party-risk-
                       management programme)
operatorRole         : enum ("ciso-office" |
                       "agency-information-
                       security" | "critical-
                       infrastructure-operator"
                       | "healthcare-it-risk" |
                       "financial-services-
                       risk" | "mssp" |
                       "third-party-risk" |
                       "user-defined")
governingFrameworks  : array of enum ("OCTAVE-
                       ALLEGRO" | "OCTAVE-FORTE"
                       | "CERT-RMM-1.2" |
                       "ISO-IEC-27001-2022" |
                       "ISO-IEC-27002-2022" |
                       "ISO-IEC-27005-2022" |
                       "ISO-31000-2018" |
                       "ISO-IEC-31010-2019" |
                       "ISO-IEC-27017" |
                       "ISO-IEC-27018" |
                       "ISO-IEC-27036-1" |
                       "ISO-IEC-27036-2" |
                       "NIST-SP-800-30" |
                       "NIST-SP-800-37" |
                       "NIST-SP-800-53" |
                       "NIST-CSF-2.0" |
                       "FIPS-PUB-199" |
                       "FIPS-PUB-200" |
                       "EU-NIS2-2022-2555" |
                       "EU-DORA-2022-2554" |
                       "EU-CRA-2024-2847" |
                       "Open-FAIR" |
                       "IEC-62443-3-2-2020" |
                       "KR-ISMS-P" |
                       "user-defined")
accreditationStatus  : object (the ISO/IEC
                       27001 certification
                       reference, the FedRAMP
                       authorisation reference,
                       the KR-ISMS-P certification
                       reference, the SEI CERT
                       OCTAVE-trained-evaluator
                       designation)
programmeStatus      : enum ("design" | "operating"
                       | "limited-rollout" |
                       "wind-down" | "archived")
```

## §3 Critical-Asset Record (OCTAVE Allegro Step 2)

```
assetRecord:
  assetId            : string (uuidv7)
  programmeRef       : string (PHASE-1 §2 record
                       reference)
  assetName          : string (the operator's
                       canonical name for the
                       asset — for example
                       "patient-record database"
                       or "core-banking ledger")
  assetType          : enum ("information-asset" |
                       "system-asset" | "software-
                       asset" | "hardware-asset" |
                       "people-asset" | "facility-
                       asset" | "service-asset" |
                       "user-defined")
  fipsCategorisation : object (the FIPS PUB 199
                       categorisation per
                       confidentiality, integrity,
                       and availability — each
                       at "low", "moderate", or
                       "high")
  rationale          : string (the operator's
                       documented rationale for
                       the asset's selection as
                       a critical asset per the
                       OCTAVE Allegro Step 2
                       prioritisation)
```

## §4 Risk-Area-of-Concern Record (OCTAVE Allegro Step 4)

```
areaOfConcern:
  concernId          : string (uuidv7)
  assetRef           : string (PHASE-1 §3 record
                       reference)
  threatScenario     : string (the identified
                       threat scenario per the
                       OCTAVE Allegro Step 4
                       template)
  threatActor        : enum ("insider-malicious"
                       | "insider-accidental" |
                       "outsider-criminal" |
                       "outsider-state" |
                       "outsider-hacktivist" |
                       "natural-disaster" |
                       "supply-chain" | "user-
                       defined")
  threatVector       : string (the threat-vector
                       declaration — phishing,
                       credential-stuffing,
                       supply-chain compromise,
                       physical-intrusion,
                       insider-exfiltration,
                       ransomware, denial-of-
                       service, mis-configuration,
                       prompt-injection)
  consequenceArea    : enum ("reputation-and-
                       trust" | "financial" |
                       "productivity" |
                       "safety-and-health" |
                       "fines-and-legal-penalties"
                       | "regulatory-compliance"
                       | "user-defined")
```

## §5 Risk-Score Record (NIST SP 800-30 Likelihood × Impact)

```
riskScore:
  riskScoreId        : string (uuidv7)
  concernRef         : string (PHASE-1 §4 record
                       reference)
  likelihoodLevel    : enum ("very-low" | "low"
                       | "moderate" | "high" |
                       "very-high")
  impactLevel        : enum ("very-low" | "low"
                       | "moderate" | "high" |
                       "very-high")
  overallRisk        : enum (per NIST SP 800-30
                       Appendix I Table I-2 —
                       "very-low" | "low" |
                       "moderate" | "high" |
                       "very-high")
  openFairBinding    : object (where the
                       operator additionally
                       declares an Open FAIR
                       quantitative envelope —
                       Loss Event Frequency, Loss
                       Magnitude, with the
                       operator's documented
                       distribution parameters)
```

## §6 Risk-Treatment Record (OCTAVE Allegro Step 7)

```
treatmentRecord:
  treatmentId        : string (uuidv7)
  riskScoreRef       : string (PHASE-1 §5 record
                       reference)
  treatmentDecision  : enum ("accept" | "mitigate"
                       | "transfer" | "avoid")
  acceptanceRationale : string (where decision
                       is "accept", the
                       documented residual-risk-
                       acceptance rationale)
  mitigationControls : array of object (per-
                       control — the ISO/IEC
                       27002 control reference,
                       the NIST SP 800-53
                       control reference, the
                       per-control implementation
                       owner, the per-control
                       implementation deadline)
  transferBinding    : object (where decision is
                       "transfer", the cyber-
                       insurance policy reference
                       or the contractual-transfer
                       reference)
  riskOwnerRef       : string (the per-treatment
                       risk-owner identifier —
                       the named senior accountable
                       person)
```

## §7 Per-Cycle Audit Record

```
auditRecord:
  auditRecordId      : string (uuidv7)
  programmeRef       : string (PHASE-1 §2 record
                       reference)
  cyclePeriod        : object (the per-cycle
                       coverage period — start
                       and end ISO 8601 dates)
  auditorRef         : string (the per-cycle
                       lead-auditor identifier
                       — internal-audit function,
                       external auditor, or SEI
                       CERT-trained OCTAVE
                       evaluator)
  findingsCount      : object (per-severity
                       finding count — "critical",
                       "high", "moderate", "low",
                       "informational")
  managementResponse : string (the per-cycle
                       management response per
                       ISO/IEC 27001 §9.3
                       management review)
```

## §8 Chain-of-Custody Record

```
custodyRecord:
  custodyId          : string (uuidv7)
  artefactRef        : string (the asset, area-
                       of-concern, risk-score,
                       treatment, or audit
                       identifier)
  custodyEvent       : enum ("asset-registered"
                       | "concern-identified" |
                       "risk-scored" |
                       "treatment-decided" |
                       "treatment-implemented" |
                       "cycle-completed" |
                       "audit-finding-issued" |
                       "user-defined")
  eventTimestamp     : string (ISO 8601 date-time)
  performingParty    : string (legal entity)
  hashOfArtefacts    : string (SHA-256 hex digest)
```

## §9 Manifest

Implementations publish a signed manifest carrying
`standardSlug` (constant value "ci-octave"),
`version`, `implementation`, the operator's
`accreditationStatus`, and the `profile`
declaration that selects which of the optional
records (asset, area-of-concern, risk-score,
treatment, audit) the implementation supports.
The manifest is signed using a key whose public
part is published on the operator's
`.well-known/wia/ci-octave/` discovery endpoint
declared in PHASE-2.

弘益人間 (Hongik Ingan) — Benefit All Humanity
