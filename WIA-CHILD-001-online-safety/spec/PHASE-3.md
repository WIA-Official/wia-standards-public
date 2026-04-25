# WIA-CHILD-001 PHASE 3: Integration

**弘益人間** - Benefit All Humanity

## Phase 3 Overview: Ecosystem Integration

### Objective

Phase 3 connects the WIA-CHILD-001 platform with statutory child-protection bodies, law enforcement, educational institutions, mental-health resources, and recognised hotlines. The intent is holistic protection that spans the boundaries of any single platform, with explicit limits on data-sharing, mandatory legal review, and audit-grade record-keeping.

---

## 1. Law-Enforcement and Hotline Integration

### 1.1 Mandatory Reporting

Reporting obligations differ by jurisdiction. The reference integration supports:

- **United States** — CyberTipline reporting to NCMEC per 18 U.S.C. §2258A for CSAM and child-exploitation incidents.
- **International CSAM** — INHOPE network of national hotlines.
- **United Kingdom** — Reporting to the Internet Watch Foundation (IWF) per the UK Online Safety Act 2023 statutory framework.
- **Korea** — KOSC (Korea Communications Standards Commission) reporting per 정보통신망법.
- **Other jurisdictions** — Local equivalent under the deploying jurisdiction's child-protection rule.

Each reporting integration carries the rule-specific evidence packaging, retention rules, and jurisdictional restrictions on cross-border data movement.

### 1.2 Evidence Preservation

Where a credible threat is detected, the platform preserves evidence in a form admissible under the deploying jurisdiction's rules of evidence. Evidence packages include:

- The unaltered content and surrounding context.
- Cryptographic signatures over the package using Ed25519 (RFC 8032) under the platform's evidence-key.
- A chain-of-custody record conforming to W3C PROV-O semantics.
- Time stamps following ISO 8601:2019 with nanosecond precision when sensor support permits.
- The rule of evidence the package targets and the rule's evidentiary expectations.

The chain of custody is preserved across handoffs to law enforcement using the same conventions adopted in WIA-CT (cryogenic-transport) chain-of-custody — the design idea is reused so that evidence handling across WIA standards is uniform.

### 1.3 Disclosure Boundaries

Disclosure to law enforcement is limited by the deploying jurisdiction's law and by the platform's published disclosure policy. The reference integration enforces:

- **Prior legal review** before any voluntary disclosure beyond mandated reporting.
- **Minimum-necessary disclosure** scoped to the rule-defined demand.
- **Notice to affected users** when the law permits and when notice does not jeopardise the investigation.
- **Audit log** of every disclosure with reviewer identity, legal basis, and the disclosed scope.

These boundaries are themselves auditable under the operating organisation's ISO/IEC 27701-aligned PIMS.

---

## 2. Child-Protection-Service Integration

### 2.1 Case-Management Interfaces

Integration with statutory child-protection services follows:

- **Standardised case-record exchange** using the HL7 FHIR R5 specification where the rule permits health/social-care interoperability, with extensions for child-protection-specific data elements.
- **Identifier resolution** using the deploying jurisdiction's national identifier scheme; the platform maintains a one-way mapping that preserves pseudonymity for non-disclosed cases.
- **Multi-agency coordination** through documented data-sharing agreements that themselves are produced and reviewed under the operating organisation's data-governance policy.

### 2.2 Outcome Tracking

Outcome tracking is consensual where required by the deploying jurisdiction's law and is governed by the rule's recordkeeping retention. Aggregate outcome statistics are published in the platform's transparency report; individual outcomes remain confidential.

---

## 3. Educational-Institution Integration

### 3.1 School Safety Programmes

Where the platform is integrated into educational programmes, integration covers:

- Teacher- and counsellor-facing dashboards with role-based access.
- Curriculum-aligned digital-citizenship resources.
- Optional anti-bullying intervention workflows aligned with the school's safeguarding policy.
- Record-keeping conforming to the educational-records rule (FERPA in the US, equivalent rules elsewhere).

### 3.2 Single Sign-On

Educational deployments support standards-based SSO:

- **OpenID Connect Core 1.0** with optional OIDC for IT 1.0 (educational profile).
- **SAML 2.0** (OASIS Standard) for legacy education-sector identity providers.
- **OneRoster 1.2** (IMS Global) for class-roster and grade-passback integration.

### 3.3 Student-Data Protection

Student data is protected under the rule of the deploying jurisdiction. The reference integration enforces:

- **No advertising profiling** of student accounts.
- **No selling or transferring** of student data outside the educational-services purpose.
- **Annual disclosure** of subprocessors and storage regions.
- **Right of correction and deletion** consistent with the rule-defined timelines.

---

## 4. Mental-Health-Resource Integration

### 4.1 Crisis Connection

Where the platform detects a high-risk indicator of self-harm or suicidal ideation, it surfaces the deploying jurisdiction's recognised crisis line (988 in the US; Samaritans in the UK; KSL Samaritans / 1393 in Korea; equivalent elsewhere) directly to the user, with one-tap connection where the user-experience permits.

Crisis-line surfacing follows the deploying jurisdiction's recommended language and presentation. The platform does not collect or transmit conversation content to the crisis line beyond what the user explicitly chooses to share.

### 4.2 Therapist and Support-Group Referral

The platform may surface curated referrals to therapists and support groups where the deploying organisation has vetted providers. Referrals are not commercial relationships and are reviewed annually.

### 4.3 Wellbeing Monitoring

Wellbeing monitoring is opt-in for users above the rule-defined consent age and opt-in for guardians for users below the consent age. Wellbeing telemetry is processed under the privacy-preservation regime declared in Phase 2 §1.3 and does not produce shareable health records without explicit consent.

---

## 5. Privacy and Data Governance

### 5.1 Data-Sharing Agreements

Every integration requires a data-sharing agreement that documents:

- The legal basis under the deploying jurisdiction (consent, legal obligation, vital interest, public task, legitimate interest, contractual necessity).
- The categories of personal data and child data shared.
- The retention period and deletion procedure.
- The recipient's safeguards and supervisory authority.
- The rights of data subjects under the deploying jurisdiction's law.

### 5.2 Cross-Border Transfers

Cross-border transfers conform to the deploying jurisdiction's transfer rules: GDPR Chapter V (adequacy decisions, standard contractual clauses, binding corporate rules), Korea PIPA Article 28-8 (cross-border transfer rules), and the analogous rules in other jurisdictions.

### 5.3 Audit Right

Each integrated partner provides an audit right consistent with the deploying jurisdiction's professional-services norms. Audit reports are reviewed by the operating organisation's PIMS team and become part of the integration's annual review.

---

## 6. Reference Standards Alignment

| Concern | Reference |
|---------|-----------|
| Child rights | UN Convention on the Rights of the Child (UNCRC) |
| CSAM reporting (US) | 18 U.S.C. §2258A |
| CSAM hashes | NCMEC and INHOPE registries (used under their licensing terms) |
| UK online safety | UK Online Safety Act 2023 |
| EU platform regulation | Regulation (EU) 2022/2065 (DSA) |
| Educational records (US) | 20 U.S.C. §1232g (FERPA) |
| Health-information interoperability | HL7 FHIR R5 |
| Identity (OIDC) | OpenID Connect Core 1.0 |
| Identity (SAML) | OASIS Standard, Security Assertion Markup Language v2.0 |
| Education roster | IMS Global OneRoster 1.2 |
| Time encoding | ISO 8601:2019 |
| Provenance | W3C PROV-O |
| Information security | ISO/IEC 27001:2022 |
| Privacy management | ISO/IEC 27701:2019 |
| Privacy framework | ISO/IEC 29100:2011 |
| Cloud privacy | ISO/IEC 27018:2019 |
| Crisis-line standards | Per deploying jurisdiction's recognised provider |

All references conform to the WIA Citation & Veracity Policy v1.0 §2.1 ALLOW.

---

## 7. Conformance

A Phase 3 implementation is conformant when:

1. Mandatory-reporting integrations exist for at least one deploying jurisdiction.
2. Evidence preservation produces signed, time-stamped packages with W3C PROV-O chain of custody.
3. Disclosure to law enforcement requires prior legal review and is logged in an auditable form.
4. Educational integrations honour FERPA or the analogous rule of the deploying jurisdiction.
5. Crisis-line surfacing uses the deploying jurisdiction's recognised provider with no platform-mediated content sharing.
6. Data-sharing agreements exist for every integrated partner and are reviewed annually.

---

## 8. Operational Appendix

### 8.1 Partner Onboarding

Each integrated partner is onboarded through a documented procedure that includes:

- Legal review of the data-sharing agreement under the deploying jurisdiction's law.
- Privacy review against the operating organisation's PIMS (ISO/IEC 27701:2019).
- Information-security review against the operating organisation's ISMS (ISO/IEC 27001:2022) controls.
- Operational readiness check covering authentication, transport, retry, and incident-response paths.
- Initial connectivity test in a non-production environment with synthetic data.
- Production go-live with monitored ramp-up.

Partners are assigned a relationship owner in the operating organisation who is responsible for the integration's ongoing health.

### 8.2 Incident Response (Cross-Boundary)

Incidents that cross organisational boundaries (a CSAM detection that becomes a law-enforcement matter, a missing-child case that engages multiple agencies) follow a documented cross-boundary playbook that complements the internal incident-response procedure.

The cross-boundary playbook includes:

- Named contacts at each integrated partner.
- Communication SLAs and channels (encrypted email, secured messaging, voice).
- Joint-decision authority for time-critical matters.
- Joint post-incident review with all parties.

The playbook is rehearsed at least annually with each major partner.

### 8.3 Transparency and Reporting

Aggregate integration outcomes are reported in the platform's transparency report:

- Number of mandatory reports filed by category.
- Number of evidence packages produced and delivered.
- Number of cross-boundary incidents with anonymised summaries.
- Aggregate response-time metrics against the documented SLAs.

The transparency report is published at least annually.

### 8.4 Standard Identifiers and Vocabularies

To keep the cross-organisational integration ground truth comparable across partners, the reference programme uses:

- **HL7 FHIR R5** code systems and value sets for any health-adjacent data.
- **Internal WIA child-safety vocabulary** (WIA-CHILD-VOCAB) as a SKOS concept scheme covering child-safety-specific concepts not standardised elsewhere.
- **W3C PROV-O** for activity, agent, and entity descriptions in chain-of-custody records.
- **ISO 19115-1:2014** for any geographic context.
- **W3C DID Core 1.0** for any decentralised identity assertions about agents.

These vocabularies are versioned and evolve under the editorial process of the originating organisation.

### 8.5 Integration Testing

Each integration has a documented test plan covering:

- Authentication and authorisation.
- Data-format conformance against the partner's published schema.
- Error-handling for partner-side failures.
- Idempotent retry semantics.
- End-to-end test traffic including realistic non-trivial cases.
- Boundary cases for time-zone, locale, and identifier formats.

Test results are reviewed during partner onboarding and at the partner's annual review.

### 8.6 Sunset of Integrations

Integrations may be sunset when the partner ceases to operate, when the legal basis lapses, or when the operating organisation determines that the integration is no longer necessary for the underlying child-safety purpose.

The sunset procedure includes notice to the partner, transition planning for any active cases, retention of records per the deploying jurisdiction's rule, and a public note in the next transparency report.

---

© 2025 SmileStory Inc. / WIA | 弘益人間
