# WIA-gdpr-compliance PHASE 3 — PROTOCOL Specification

**Standard:** WIA-gdpr-compliance
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern a
controller (or processor) under GDPR: the Article 5
principles applied as machine-checkable invariants, the
Article 6 lawful-basis discipline, the Article 9
special-category data discipline, the data-subject-
rights discipline (Articles 12 to 22), the Article 24 to
30 controller-and-processor accountability discipline,
the Article 32 security-of-processing discipline, the
Article 33 to 34 breach-notification discipline, the
Article 35 DPIA discipline, the Chapter V international-
transfer discipline, the data-protection-by-design
discipline (Article 25), and the supervisory-authority
correspondence discipline (Articles 51 to 59).

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27018:2019 (PII in public clouds)
- ISO/IEC 27701:2019 (privacy information management
  system)
- ISO/IEC 29100:2011 (privacy framework)
- ISO/IEC 29134:2017 (privacy impact assessment)
- ISO 8601 (date and time)
- IETF RFC 5905 (NTPv4)
- IETF RFC 9457 (Problem Details)
- EU GDPR (Regulation (EU) 2016/679, Articles 5 to 99)
- UK GDPR (retained-EU-law version plus UK Data
  Protection Act 2018)
- EU Standard Contractual Clauses (Commission
  Implementing Decision (EU) 2021/914)
- EU Law Enforcement Directive (Directive (EU) 2016/680)
- EDPB Guidelines (the European Data Protection Board's
  authoritative interpretive guidance, including
  Guidelines on consent, on transparency, on Article
  6(1)(f) legitimate interest, on the international
  transfer toolkit, on personal-data breach
  notification, and on data subject rights)
- ICO Guidance (cited where the operating jurisdiction
  is the UK; the UK Information Commissioner's Office
  guidance is the UK supervisory authority's
  interpretive baseline)

---

## §1 Article 5 Principles as Machine Invariants

The Article 5(1) principles are encoded as machine-
checkable invariants over the records defined in
PHASE-1:

- Lawfulness, fairness, transparency (Article 5(1)(a))
  → every processing activity carries a `legalBasis`
  field (PHASE-1 §3) drawn from the Article 6(1)
  enumeration; transparency materials reference the
  per-activity purpose for each data-subject-facing
  notice.
- Purpose limitation (Article 5(1)(b)) → each activity
  declares a single purpose and reuse for compatible
  purposes is recorded as an additional activity
  rather than as a redefinition of the original.
- Data minimisation (Article 5(1)(c)) → each activity
  declares the personal-data categories processed and
  the controller's data-minimisation review cycle
  records the reduction of categories where feasible.
- Accuracy (Article 5(1)(d)) → controller's data-
  quality discipline records the rectification cadence
  for each category.
- Storage limitation (Article 5(1)(e)) → each activity
  declares a `retentionPolicy` field; expired records
  are deleted or anonymised on the controller's
  retention-cycle cadence.
- Integrity and confidentiality (Article 5(1)(f)) →
  Article 32 controls (PHASE-3 §7) protect records.
- Accountability (Article 5(2)) → controller emits the
  PHASE-4 evidence package on the operator's cadence.

## §2 Article 6 Lawful-Basis Discipline

Each processing activity cites a single Article 6(1)
basis. Where the basis is `art-6-1-f-legitimate-interest`,
the controller's legitimate-interest balancing test is
documented per the EDPB Guidelines on Article 6(1)(f)
(necessity, balancing against data-subject rights,
disclosed in transparency materials). Where the basis
is `art-6-1-c-legal-obligation`, the controller cites
the operating jurisdiction's specific legal source.

## §3 Article 9 Special-Category Data Discipline

Special-category data (Article 9(1)) is processed only
under one of the Article 9(2) bases. The controller's
Article 9 discipline:

- per-activity Article 9(2) basis annotation (PHASE-1 §6);
- per-activity additional safeguards (encryption at
  rest, access-control narrowing, audit-log enrichment,
  retention shortening) commensurate with the Article
  9(4) Member State law where applicable;
- per-activity DPO consultation record before
  commencement of processing.

## §4 Data Subject Rights Discipline (Articles 12 to 22)

Per-Article alignment:

- Article 12 transparent information → controller's
  transparency materials reference each PHASE-1 §3
  activity in plain language, in the operating
  jurisdiction's language(s);
- Article 13 / 14 information at collection → product-
  surface notices cite the activity references;
- Article 15 right of access → access export bundles
  the per-data-subject record set across all
  activities;
- Article 16 rectification → rectification flows back
  to the source of truth (the activity's primary store)
  and rectification of recipients per Article 19;
- Article 17 right to erasure ("right to be forgotten")
  → erasure flow respects the Article 17(3) exemptions
  (freedom of expression, legal obligation, public
  interest, archiving, legal claims) where applicable;
- Article 18 restriction → restricted records are
  flagged so that any further processing requires
  documented justification under Article 18(2);
- Article 19 notification of rectification or erasure
  → recipients (joint controllers, processors, third-
  party recipients in PHASE-1 §3 `recipientCategory`)
  receive the rectification or erasure notification on
  the controller's downstream-update cadence;
- Article 20 portability → portability export uses a
  structured, commonly-used and machine-readable format
  (the EDPB Guidelines on portability cite JSON, XML,
  CSV as common acceptable formats);
- Article 21 objection → objection processing for
  legitimate-interest and direct-marketing activities
  per Article 21(2) and (3);
- Article 22 automated-decision review → human-reviewer
  workflow with documented rationale (PHASE-2 §14).

The Article 12(3) deadline is a one-month default with a
two-month extension permitted when the controller
documents the reason; deadline enforcement is a machine
invariant per PHASE-2 §5.

## §5 Article 24 to 30 Controller and Processor Discipline

- Article 24 controller responsibility → programme
  registration (PHASE-1 §2);
- Article 25 data protection by design and by default
  → product-development discipline cross-referenced
  from each new processing activity's DPIA;
- Article 26 joint controllers → arrangement record
  (PHASE-1 §11) with the responsibility-matrix
  reference published to data subjects;
- Article 27 representative → programme-level field
  recording the EU representative for non-EU
  controllers offering goods or services in the EU;
- Article 28 processor → processor DPA (PHASE-1 §11),
  with sub-processor approval discipline embedded;
- Article 29 processor under controller authority →
  processor's processing only on documented controller
  instructions, recorded in the DPA;
- Article 30 records of processing activities →
  PHASE-1 §3 record set.

## §6 Article 32 Security of Processing Discipline

Article 32 obligates technical and organisational
measures appropriate to the risk. Cross-walked to
ISO/IEC 27001:2022 controls:

- pseudonymisation and encryption (Article 32(1)(a))
  → ISO/IEC 27002:2022 §8.10 / §8.24;
- ongoing confidentiality, integrity, availability,
  and resilience (Article 32(1)(b)) → ISO/IEC
  27002:2022 §5 / §8;
- ability to restore availability and access (Article
  32(1)(c)) → ISO/IEC 27002:2022 §5.30 / §8.13;
- regular testing and evaluation (Article 32(1)(d)) →
  the controller's security-testing programme.

## §7 Article 33 and 34 Breach Discipline

Article 33 obligates notification to the supervisory
authority within 72 hours of awareness; Article 34
obligates notification to the data subjects when the
breach is likely to result in a high risk. The
controller's discipline:

- per-incident `awareAt` recording so that the 72-hour
  clock is auditable;
- per-incident classification against the Article 34(1)
  high-risk threshold using the EDPB Guidelines on
  breach notification taxonomy (confidentiality,
  integrity, availability);
- per-incident supervisory-authority notification
  through the operating jurisdiction's prescribed
  notification channel (each DPA publishes a
  notification form);
- per-incident data-subject notification in clear and
  plain language where Article 34 high-risk threshold
  met;
- per-incident processor-to-controller notification
  per Article 33(2) "without undue delay".

## §8 Article 35 DPIA Discipline

DPIAs are conducted under the Article 35(1) "likely to
result in a high risk" criterion, with the Article 35(3)
mandatory triggers:

- systematic and extensive evaluation (e.g. profiling
  with significant effects);
- large-scale processing of special-category data;
- systematic monitoring of publicly accessible areas.

Each DPA publishes a list of processing operations
requiring a DPIA per Article 35(4); each DPA's list is
honoured for processing in that DPA's jurisdiction. The
controller's DPIA discipline:

- DPO consultation per Article 35(2);
- documented residual-risk assessment;
- Article 36 prior-consultation correspondence with the
  supervisory authority where residual high risk
  persists after mitigation.

## §9 Chapter V International-Transfer Discipline

Transfers of personal data to third countries follow
Chapter V:

- Article 45 adequacy → no additional safeguards
  required for transfers to adequacy-decision
  countries;
- Article 46 appropriate safeguards → SCCs
  (Commission Implementing Decision (EU) 2021/914),
  binding corporate rules, codes of conduct, or
  approved certification mechanisms;
- Article 47 binding corporate rules → for intra-group
  transfers under approved BCRs;
- Article 49 derogations → for occasional transfers
  meeting the Article 49(1) conditions; recurring
  transfers under derogations are not the appropriate
  mechanism per the EDPB Guidelines on derogations.

Each SCC-mechanism transfer is paired with a transfer-
impact assessment under the EDPB Recommendations on
supplementary measures to assess the destination
country's third-country surveillance regime.

## §10 Article 25 Data Protection by Design and by Default

Per Article 25, controllers implement appropriate
technical and organisational measures both at the time
of determination of the means for processing and at the
time of the processing itself. The controller's
discipline:

- privacy threat modelling on each new product or
  feature;
- minimum-data-by-default configuration;
- pseudonymisation by default where the processing
  purpose can be served by pseudonymous data;
- privacy-enhancing technologies (PETs) review for
  high-risk activities.

## §11 Records Retention

Programme records — every processing activity, consent
record, data subject request, breach notification,
DPIA, sa correspondence, controller arrangement —
retain per the operating jurisdiction's records-
retention rules and per the operator's documented
retention policy. Article 30 records retain for the
duration of the processing plus the operating
jurisdiction's audit horizon.

## §12 Time Synchronisation

Operator clocks synchronise per RFC 5905 (NTPv4) so that
the Article 33 72-hour clock and the Article 12(3) one-
month clock are consistent across the operator's
runtime fleet.

## §13 One-Stop-Shop Discipline (Articles 56 and 60)

For cross-border processing under Article 4(23), the
lead supervisory authority is determined per Article 56
(the controller's main establishment in the EU). The
controller's one-stop-shop discipline:

- lead-DPA designation recorded at programme-level
  (PHASE-1 §2);
- non-lead DPAs ("concerned" supervisory authorities
  per Article 4(22)) recorded at correspondence-level;
- Article 60 cooperation procedure mirrored to the
  lead DPA;
- Article 65 dispute-resolution procedure followed
  where lead and concerned DPAs disagree.

## §14 Member-State Derogation Discipline

Several GDPR Articles permit Member State derogations
(Article 6(2), Article 9(4), Article 23, Articles 85
to 91). The controller's derogation discipline records
the per-Member-State derogation references that the
operating activity relies on (e.g. the operating
Member State's national law on processing for
employment purposes under Article 88).

## §15 Children's-Data Discipline

Article 8 sets the minimum age of consent at 16 with
Member State derogation as low as 13. The controller's
children's-data discipline:

- per-jurisdiction Article 8 age-of-consent threshold
  recorded;
- parental-consent capture for data subjects below the
  threshold;
- transparency materials and consent statements drafted
  in age-appropriate language for child data subjects
  (the EDPB Guidelines on transparency endorse this
  discipline);
- Article 17(1)(f) erasure right for processing
  consented to as a child receives priority handling.

## §16 Quality Dossier and Conformance

The controller's GDPR quality dossier records the
governing frameworks, the DPO appointment, the records
of processing activities, the joint-controller and
processor inventory, the international-transfer
inventory, the breach history, the DPIA library, the
supervisory-authority correspondence history, and the
Article 32 security-measure inventory. The dossier is
reviewed at least annually by the DPO.

A programme conformant with WIA-gdpr-compliance
publishes its representative-and-DPO contact details,
its public privacy notices (Articles 13 and 14), its
breach-history disclosure where Article 34 notification
applied, and its supervisory-authority correspondence
outcomes; and answers an annual self-assessment that
maps each clause of this PHASE to the controller's
implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-gdpr-compliance
- **Last Updated:** 2026-04-28
