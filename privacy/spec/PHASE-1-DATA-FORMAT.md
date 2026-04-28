# WIA-privacy PHASE 1 — Data Format Specification

**Standard:** WIA-privacy
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for cross-
cutting privacy operations: personal-data inventory records,
processing-activity records (RoPA), data-subject identity and
consent records, data-subject-rights request records, data-
protection-impact-assessment (DPIA) records, breach-notification
records, cross-border-transfer records, and the cross-references
binding personal-data flows to controllers, processors, and
sub-processors. The shape interoperates with ISO/IEC 29100
privacy framework, ISO/IEC 27701 PIMS, ISO 31700-1:2023
Privacy by Design, GDPR records-of-processing-activities, and
the regional privacy regimes (CCPA/CPRA, K-PIPA, LGPD, PIPL,
PDPA-SG/AU/TH, NPPA-IN).

References (CITATION-POLICY ALLOW only):
- ISO/IEC 29100:2011/Amd 1:2018 — Privacy framework
- ISO/IEC 27701:2019 — Privacy Information Management System (extends 27001/27002)
- ISO/IEC 27018:2019 — Code of practice for PII protection in public-cloud processors
- ISO/IEC 29184:2020 — Online privacy notices and consent
- ISO/IEC 27551:2021 — Requirements for attribute-based unlinkable entity authentication
- ISO/IEC 27559:2022 — Privacy-enhancing data de-identification framework
- ISO 31700-1:2023 — Privacy by Design for consumer goods and services
- ISO/IEC 29134:2023 — Guidelines for privacy impact assessment
- ISO/IEC 27556:2022 — User-centric privacy preferences management
- NIST Privacy Framework v1.0
- EU GDPR (Regulation 2016/679) — Articles 5, 6, 7, 13, 14, 15-22, 25, 30, 32-36
- KR PIPA (개인정보 보호법) — 표준 개인정보 처리방침 작성지침
- US California CCPA/CPRA (CCR §999.300 et seq.)
- BR LGPD (Lei nº 13.709/2018)
- CN PIPL (个人信息保护法)
- IETF RFC 8259 (JSON), RFC 7515 (JWS), RFC 3339

---

## §1 Scope

This PHASE applies to systems acting as a privacy-control
boundary: data-controller systems (those determining purposes
and means of processing), processor systems (those processing
on behalf of a controller), and sub-processor systems (those
acting on behalf of a processor).

The standard is jurisdiction-aware: each record references
its controlling jurisdiction (one of the regional regimes
listed above). A processing activity that crosses jurisdictions
declares the legal basis and transfer mechanism per regime.

In scope: personal-data inventory, RoPA, consent and lawful
basis, data-subject-rights (DSR) requests, DPIA, breach
notification, cross-border transfer, sub-processor disclosure.
Out of scope: per-domain consent specifics for healthcare
(WIA-medical-data-privacy), financial (WIA-open-banking),
biometric specifics (WIA-identity-management), and the
implementation of de-identification algorithms (referenced
to ISO/IEC 27559 / NIST SP 800-188).

## §2 Personal-data inventory record

Every category of personal data processed carries an inventory
record:

- `inventoryId` — URN of form `urn:wia:priv:inventory:<controller>:<id>`
- `dataCategory` — closed enum from the deployment's
  data-category catalogue (e.g., `contact-info`, `government-id`,
  `payment-info`, `health-info`, `biometric`, `location`,
  `behavioural-derived`, `inferred-sensitive`)
- `sensitiveness` — closed enum: `general`, `sensitive`
  (GDPR Art. 9 / K-PIPA 민감정보 / CCPA "sensitive personal
  information"), `child` (under-13 US / under-14 KR / under-16
  EU per Member State variation)
- `dataSubjectCategories[]` — closed enum: `customer`,
  `employee`, `prospect`, `child`, `patient`, `student`,
  `vendor-contact`, `visitor`, etc.
- `sourceCategories[]` — `directly-from-subject`,
  `from-third-party`, `from-public-record`, `derived-by-inference`
- `retentionPeriod` — declared maximum retention with
  ISO 8601 duration
- `retentionTrigger` — what starts the retention clock
  (account-closure, last-purchase, regulatory-required-period,
  legal-hold-cleared)
- `disposalMethod` — closed enum: `delete`, `anonymise`
  (per ISO/IEC 27559 framework), `pseudonymise-and-archive`
- `processingPurposes[]` — references to RoPA entries

The inventory is not exhaustive personal-data per record;
it is the catalogue of data-category × purpose combinations
that the deployment processes.

## §3 Records-of-processing-activities (RoPA, GDPR Art. 30)

Each processing activity carries a RoPA entry:

- `ropaId` — URN
- `activityName` — human-readable name
- `controller` — controller URN (deployment / its parent /
  joint-controller list per GDPR Art. 26)
- `processor[]` — processor URNs (sub-processors enumerated
  separately)
- `purposes[]` — declared purposes (closed enum per the
  deployment's purpose taxonomy)
- `lawfulBases[]` — closed enum from per-regime lawful-basis
  enumerations (GDPR Art. 6: consent, contract, legal-obligation,
  vital-interests, public-task, legitimate-interests; K-PIPA
  §15: 동의, 계약 이행, 법령상 의무, 공공 업무, 정보주체 이익,
  정당한 이익; CCPA §1798.100 et seq.: business purpose
  declarations)
- `dataCategories[]` — references to inventory entries
- `dataSubjects[]` — references to subject categories
- `recipients[]` — per-recipient records (organisation URN,
  jurisdiction, transfer-mechanism reference)
- `retentionRef` — reference to retention schedule
- `crossBorderTransferRef[]` — references to transfer records
- `securityMeasuresRef` — reference to the deployment's
  security-measures catalogue (cryptography, access control,
  pseudonymisation, etc.)
- `dpiaRef` — reference to DPIA if Art. 35 triggered

RoPA mutations are signed by the controller's data-protection
authority and audit-chained. RoPA snapshots are the deployment's
canonical compliance-disclosure artefact for regulators.

## §4 Consent record

Consent (per Art. 7 GDPR / K-PIPA §22 / CCPA opt-in for
sensitive categories) carries:

- `consentId` — URN
- `dataSubjectRef` — pseudonymised subject identifier
- `controllerRef`
- `consentScope` — closed list of purposes the consent covers
- `mechanism` — closed enum: `web-form`, `mobile-prompt`,
  `paper-form-scanned`, `verbal-recorded`, `parent-on-behalf`,
  `clinician-on-behalf-emergency`
- `presentationLanguage` — BCP 47 language tag of the
  presentation at consent capture
- `noticeTextRef` — URN of the privacy notice presented at
  consent capture (per ISO/IEC 29184)
- `givenAt` — RFC 3339 with offset
- `withdrawal` — null if active, else `{withdrawnAt,
  withdrawalChannel}`
- `expirationDateTime` — RFC 3339 (jurisdictions with
  consent expiry, e.g., KR 2-year default for marketing)
- `evidenceArtifactRef` — URN of the captured evidence
  (form snapshot, audio file hash, signature image hash)

Consents are signed (JWS) and stored with their evidence
artefact. The boundary refuses consent operations that
don't carry the noticeTextRef (no consent without notice).

## §5 Data-subject-rights (DSR) request record

DSRs (Art. 15-22 GDPR / K-PIPA §35-39 / CCPA §1798.100-130)
carry:

- `dsrId` — URN
- `dataSubjectRef`
- `requestType` — closed enum: `access`, `rectification`,
  `erasure`, `restriction`, `portability`, `objection`,
  `automated-decision-objection`, `withdrawal-of-consent`,
  `do-not-sell` (CCPA), `opt-out-of-sale-or-share` (CPRA),
  `know-categories`, `know-specific-pieces`, `limit-use-of-sensitive-pi`
- `submittedAt`
- `submittedVia` — channel (`web-form`, `email`, `phone`,
  `paper`, `regulator-relayed`)
- `identityVerificationRef` — URN of identity-verification
  evidence (proportionate to the sensitivity of the request)
- `state` — `open` / `verifying` / `processing` /
  `completed-fulfilled` / `completed-refused-with-reason` /
  `extended-with-reason` / `referred-to-affiliate`
- `regulatoryDeadline` — RFC 3339 by which fulfillment is
  required (jurisdiction-specific: GDPR 30 days,
  K-PIPA 10일+30일 연장, CCPA 45 days, etc.)
- `fulfillmentRecordRef` — URN of the fulfillment evidence

DSRs are per-data-subject and audit-chained; refusal records
include the legal basis for refusal (e.g., third-party rights
override under Art. 15(4)).

## §6 Data-protection-impact-assessment (DPIA) record

DPIAs (Art. 35 GDPR / K-PIPA §33 영향평가 / per ISO/IEC 29134)
carry:

- `dpiaId` — URN
- `triggeringActivities[]` — RoPA references whose risk
  triggered the DPIA
- `riskAssessment` — narrative + structured risk catalogue
  with per-risk likelihood × severity scoring
- `mitigations[]` — per-risk mitigation measures with
  implementation evidence references
- `residualRisk` — declared residual risk classification
- `dpoOpinion` — DPO sign-off (or its regional equivalent
  CPO / 개인정보보호책임자)
- `consultations[]` — regulator consultation references
  (per Art. 36 GDPR, K-PIPA §33-2 사전 협의)
- `reviewCadence` — declared review cadence
- `effectiveFrom` / `effectiveUntil`

DPIAs are signed and audit-chained. Material RoPA changes
trigger DPIA review.

## §7 Breach-notification record

Breach records (Art. 33-34 GDPR / K-PIPA §34 / CCPA §1798.150
private right of action / state breach laws) carry:

- `breachId` — URN
- `incidentRef` — reference to the security-incident record
  (cross-domain to WIA-network-security)
- `discoveredAt` — RFC 3339
- `affectedDataCategories[]` — references to inventory entries
- `affectedSubjectCount` — estimate (with bound and
  methodology declared)
- `breachKind` — closed enum: `unauthorised-access`,
  `unauthorised-disclosure`, `accidental-loss`,
  `unauthorised-alteration`, `unavailability`
- `riskToSubjects` — per-jurisdiction harm assessment
- `regulatorNotifications[]` — per-regulator submission
  records (per regime: GDPR 72h to lead supervisory
  authority, K-PIPA KISA 72시간 + 정보주체 통지 5일,
  CCPA per state law)
- `subjectNotifications[]` — per-subject communication
  evidence (where required: GDPR Art. 34 high-risk,
  K-PIPA §34, US state breach laws)
- `containment` / `remediation` — narrative + audit-trail
  references

Breach records are append-only; updates create new records
referencing the prior. Closed breach records remain queryable
for the regulatory retention window.

## §8 Cross-border-transfer record

Cross-border transfers carry:

- `transferId` — URN
- `dataExporter` — controller / processor URN
- `dataImporter` — recipient URN with jurisdiction
- `personalDataCategories[]`
- `mechanism` — closed enum:
  - `adequacy-decision` (Art. 45) with the decision
    reference (e.g., EU-Japan, EU-Korea, EU-UK)
  - `scc-2021/914` (EU Standard Contractual Clauses
    Module 1-4)
  - `scc-uk-idta` (UK International Data Transfer Agreement)
  - `bcr-controller` / `bcr-processor` (Binding Corporate
    Rules)
  - `art-49-derogation` (specific situations)
  - `kr-cross-border-consent` / `kr-equivalent-protection`
  - `cn-pipl-art-38`
- `tia` — Transfer Impact Assessment reference (post-Schrems II)
- `effectiveFrom` / `effectiveUntil`
- `supplementaryMeasures[]` — encryption, pseudonymisation,
  access-restriction provisions

The boundary refuses cross-border processing whose declared
mechanism is invalid or expired.

## §9 Sub-processor disclosure record

Sub-processor records (per Art. 28 GDPR / K-PIPA §26):

- `subprocessorId` — URN
- `parentProcessorRef` — processor URN authorising the
  sub-processor
- `serviceDescription` — what the sub-processor provides
- `legalEntity` + `jurisdiction`
- `dataCategories[]` accessed
- `securityMeasures[]`
- `subprocessorAgreementRef` — URN of the executed DPA
- `noticeProvidedAt` — RFC 3339 of customer notice
- `objectionWindow` — `{from, until}` for customer objection
- `effectiveFrom`

Sub-processor lists are public-by-default for B2B SaaS
deployments; mutations notify subscribed customers per
the configured notice channel.

## §10 Privacy-notice record

Privacy notices (per Art. 13/14 GDPR / K-PIPA §30 / ISO/IEC
29184) carry:

- `noticeId` — URN
- `presentationLanguage` — BCP 47
- `noticeText` — full notice text (or canonical URI)
- `effectiveFrom` / `effectiveUntil`
- `referencesRopa[]` — RoPA entries the notice covers
- `signedBy` — DPO / CPO signature reference

Notice mutations are versioned; consents reference the
notice version active at consent capture.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Cross-domain references (informative)

| Reference                       | Use site                                           |
|---------------------------------|----------------------------------------------------|
| WIA-medical-data-privacy        | health-PHI consent (this PHASE underlies it)       |
| WIA-medication-adherence        | medication consent inheritance                      |
| WIA-medical-imaging             | imaging-study consent                              |
| WIA-identity-management         | identity-verification for DSRs                      |
| WIA-network-security            | security-incident → breach notification flow        |
| WIA-pq-crypto                   | encryption migration as supplementary measure       |

## Annex B — Conformance disclosure

Sections §2, §3, §4, §5, §7, §10 are mandatory. §6 (DPIA)
is mandatory for deployments processing high-risk activities
per the regime's threshold. §8 (cross-border) is mandatory
for any deployment with international processing. §9
(sub-processor) is mandatory for B2B / shared-tenant
deployments.

## Annex C — Conformance levels

| Level     | Scope                                                                |
|-----------|----------------------------------------------------------------------|
| Surface   | data formats accepted; self-attested                                |
| Verified  | annual third-party ISO/IEC 27701 audit                              |
| Anchored  | continuous evidence package + regulator-witnessed audit-chain       |

## Annex D — Worked RoPA entry (informative)

```json
{
  "ropaId": "urn:wia:priv:ropa:controller-x:r-014-2026",
  "activityName": "Subscriber email marketing",
  "controller": "urn:wia:org:controller-x",
  "processor": ["urn:wia:org:esp-vendor"],
  "purposes": ["direct-marketing-email"],
  "lawfulBases": ["consent (GDPR Art. 6(1)(a))"],
  "dataCategories": ["urn:wia:priv:inventory:controller-x:contact-info"],
  "dataSubjects": ["customer"],
  "recipients": [
    {"organisationRef": "urn:wia:org:esp-vendor", "jurisdiction": "EU", "transferMechanism": "intra-EEA"}
  ],
  "retentionRef": "urn:wia:priv:retention:controller-x:rs-marketing",
  "crossBorderTransferRef": [],
  "securityMeasuresRef": "urn:wia:priv:sec-measures:controller-x:m-2026"
}
```

## Annex E — Vocabulary cross-walk

| GDPR concept              | K-PIPA equivalent              | CCPA equivalent              |
|---------------------------|--------------------------------|------------------------------|
| Personal data             | 개인정보                        | Personal Information         |
| Sensitive data            | 민감정보                        | Sensitive Personal Information|
| Controller                | 개인정보처리자                  | Business                     |
| Processor                 | 개인정보취급자                  | Service Provider / Contractor |
| Data subject              | 정보주체                        | Consumer                     |
| Lawful basis              | 처리 근거                       | Permitted Purpose            |
| DPIA                      | 개인정보 영향평가               | Risk Assessment              |
| DPO                       | 개인정보보호책임자 (CPO)        | Privacy Officer              |
| Lead supervisory authority | KISA / 개인정보보호위원회      | California Privacy Protection Agency |

## Annex F — Versioning and deprecation

Versioning follows SemVer 2.0.0. Regional regime amendments
(e.g., GDPR-AI-Act interaction, CPRA enforcement evolution)
can trigger non-breaking minor bumps. Major bumps require a
12-month sunset for the prior major to allow integrated
deployments to migrate.

## Annex G — Subject-pseudonym lifecycle

`dataSubjectRef` values are stable, opaque pseudonyms within
the deployment. Cross-deployment portability requires
explicit subject-initiated portability (Art. 20 GDPR).
Re-identification within the deployment is governed by the
deployment's identity-vault policy and audit-chained.
