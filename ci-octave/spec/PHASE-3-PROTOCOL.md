# WIA-ci-octave PHASE 3 — Protocol Specification

**Standard:** WIA-ci-octave
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern
a CI-OCTAVE risk-management operator across
the asset-to-threat-to-treatment-to-audit value
chain: the OCTAVE Allegro 8-step methodology
discipline that anchors every per-asset
evaluation; the OCTAVE FORTE programme-level
governance discipline that anchors the
operator's enterprise-risk-management cycle;
the ISO/IEC 27005 risk-management process
discipline that maps the OCTAVE methodology to
the international risk-management vocabulary;
the NIST SP 800-30 likelihood-and-impact
discipline that scores every risk; the ISO/IEC
27001 ISMS discipline that runs the operator's
information-security management system; the
ISO/IEC 27036 supplier-relationship discipline
that handles third-party risk; the EU NIS2 /
DORA / CRA discipline that gates the operator's
sectoral compliance; the chain-of-custody
anchoring discipline that prevents silent
mutation of the per-cycle artefacts; the
acceptance-and-treatment discipline that binds
risk-owner accountability; and the post-audit
remediation-and-monitoring discipline.

References (CITATION-POLICY ALLOW only):

- CMU SEI CERT OCTAVE Allegro and OCTAVE
  FORTE methodologies
- CMU SEI CERT-RMM v1.2
- ISO/IEC 27001:2022, ISO/IEC 27002:2022,
  ISO/IEC 27005:2022, ISO/IEC 27036 series
- ISO 31000:2018, ISO/IEC 31010:2019
- ISO/IEC 17021-1:2015, ISO/IEC 17065:2012
- NIST SP 800-30 Rev. 1, NIST SP 800-37 Rev. 2,
  NIST SP 800-53 Rev. 5, NIST CSF 2.0
- US FIPS PUB 199, FIPS PUB 200, FedRAMP
- EU NIS2 Directive 2022/2555, EU DORA
  Regulation 2022/2554, EU CRA Regulation
  2024/2847
- ENISA Threat Landscape, ENISA Risk Assessment
  Methodology
- Open FAIR (The Open Group)
- IEC 62443-3-2:2020 (security risk assessment
  for system design)
- IETF RFC 9110, RFC 9421, RFC 9457, RFC 6234,
  RFC 8615, RFC 6962
- W3C Trace Context
- KR 정보통신망 이용촉진 및 정보보호 등에 관한 법
  률, KR 개인정보 보호법, KR 전자금융거래법, KR
  중요정보통신기반시설 보호법
- KR-ISMS-P certification scheme operated by
  KISA

---

## §1 OCTAVE Allegro Methodology Discipline

### §1.1 Per-cycle 8-step sequence

Every per-cycle evaluation runs the OCTAVE
Allegro 8-step sequence — establish risk-
measurement criteria, develop information-
asset profile, identify information-asset
containers, identify areas of concern,
identify threat scenarios, identify risks,
analyse risks, select mitigation approach. The
operator's API enforces the per-step
sequencing; a step's submission is rejected
until the prerequisite step is closed.

### §1.2 Risk-measurement criteria pre-binding

The risk-measurement criteria are bound at the
programme level (the per-programme
consequenceArea-to-impact-table envelope) so
that the per-cycle scoring is consistent with
the programme's documented risk appetite.

### §1.3 Information-asset container declaration

Every information-asset record carries the
per-asset container declaration — the
technical container (the per-system
deployment), the physical container (the per-
facility deployment), and the people container
(the per-role deployment). The OCTAVE Allegro
methodology requires that the per-container
threat-and-vulnerability evaluation is
explicit.

## §2 OCTAVE FORTE Programme Discipline

A per-cycle evaluation under the OCTAVE FORTE
programme is bound to the FORTE 10-step
sequence — establish executive governance,
establish enterprise risk-management
programme, establish enterprise risk-
management process, allocate budget and
resources, communicate the process, identify
risks, analyse risks, select mitigation
approach, implement mitigations, monitor
performance. The operator's API records the
per-step completion status and the per-step
evidence reference.

## §3 ISO/IEC 27005 Process-Mapping Discipline

The operator declares the per-programme
mapping table from the OCTAVE Allegro / FORTE
steps to the ISO/IEC 27005 process steps —
context establishment, risk identification,
risk analysis, risk evaluation, risk
treatment, risk acceptance, risk
communication, risk-monitoring-and-review.
The mapping is published so that a downstream
ISO/IEC 27001 auditor can deterministically
trace the per-step evidence.

## §4 NIST SP 800-30 Likelihood-and-Impact Discipline

### §4.1 Per-risk scoring envelope

Every risk-score record carries the NIST SP
800-30 likelihood-and-impact envelope. The
likelihood scale is the standard's Appendix
G qualitative-likelihood table (very-low,
low, moderate, high, very-high). The impact
scale is the standard's Appendix H qualitative-
impact table.

### §4.2 Overall-risk derivation

The overall-risk level is derived from the
likelihood × impact combination per Appendix
I Table I-2. The derivation is deterministic;
an operator declaring an overall-risk level
inconsistent with the table is in non-
conformance.

### §4.3 Open FAIR quantitative envelope

An operator that additionally maintains an
Open FAIR quantitative envelope publishes the
per-risk Loss Event Frequency, Loss Magnitude,
and Annualised Loss Expectancy distributions
per the Open FAIR specification.

## §5 ISO/IEC 27001 ISMS Discipline

The operator runs an ISO/IEC 27001:2022 ISMS
covering the per-cycle risk-evaluation
process, the per-cycle treatment-implementation
process, the per-cycle audit-cycle, and the
chain-of-custody process. Internal audits run
on a frequency declared in the ISMS manual;
the nonconformity register is reviewed in the
ISO/IEC 27001 §9.3 management-review cycle.

## §6 ISO/IEC 27036 Supplier-Relationship Discipline

A third-party-risk-management programme
operator running the OCTAVE Allegro evaluation
on a supplier engagement is bound to the
ISO/IEC 27036 supplier-relationship discipline.
The operator's API records the per-supplier
engagement contract reference, the per-
supplier risk-acceptance envelope, and the
per-supplier monitoring envelope.

## §7 EU NIS2 / DORA / CRA Sectoral Discipline

### §7.1 EU NIS2 risk-management discipline

A critical-infrastructure or important-entity
operator under the EU NIS2 Directive is bound
to the directive's Article 21 risk-management
obligations — the policies on risk-analysis-
and-information-system security, the incident-
handling, the business-continuity, the supply-
chain security, the security in network-and-
information-systems acquisition. The operator's
API publishes the per-obligation evidence
register.

### §7.2 EU DORA ICT-risk-management discipline

A financial-entity operator under the EU DORA
Regulation is bound to the regulation's
Chapter II ICT-risk-management framework. The
operator runs the per-period digital-
operational-resilience testing per the
regulation's Article 24, including threat-led
penetration testing where applicable.

### §7.3 EU CRA cyber-resilience discipline

A product-with-digital-elements manufacturer
under the EU CRA Regulation is bound to the
regulation's Annex I essential-cybersecurity-
requirements. The operator's API publishes
the per-product cybersecurity-risk-assessment
envelope per the regulation's Article 13.

## §8 Chain-of-Custody Anchoring Discipline

### §8.1 Per-event transparency log

Every chain-of-custody event carried by
PHASE-1 §8 is appended to a per-operator
transparency log modelled on the IETF RFC 6962
Certificate Transparency append-only-log
structure.

### §8.2 Mutation prevention

A custody event cannot be retroactively
edited; an amendment is recorded as a new
event with `previousEventRef` pointing at the
event being amended.

## §9 Acceptance-and-Treatment Discipline

### §9.1 Risk-owner accountability

Every risk-treatment record names a senior
accountable person as the risk owner. The
risk owner countersigns the per-treatment
decision through the operator's API, and the
per-treatment audit envelope records the
countersignature.

### §9.2 Acceptance threshold gating

A risk-acceptance decision for a "high" or
"very-high" overall-risk level requires the
audit-committee chair's countersignature. The
operator's API enforces the gating; an
acceptance-decision without the required
countersignature is rejected with `403
Forbidden` at `/problems/risk-acceptance-
gating-required`.

### §9.3 Per-treatment monitoring cadence

Every "mitigate" treatment-decision carries a
per-control implementation deadline and a per-
control monitoring cadence. The operator's
API records the per-cadence monitoring report
and surfaces a dashboard of per-treatment
status.

## §10 Post-Audit Remediation Discipline

A per-cycle audit finding is bound to a per-
finding remediation envelope. The envelope
carries the corrective-action declaration,
the corrective-action owner, the corrective-
action deadline, and the per-action
implementation evidence. The operator's API
publishes the per-finding remediation status
to the audit committee on the cadence
declared in the operator's audit charter.

## §11 KR-Jurisdiction Discipline

### §11.1 KR-ISMS-P binding

A KR-jurisdiction operator certified under
the KISA-operated ISMS-P scheme binds the
per-cycle OCTAVE evaluation to the ISMS-P
certification reference. The operator's API
publishes the per-cycle evidence to the
ISMS-P certification body's intake endpoint
on the cadence declared in the certification
contract.

### §11.2 KR 중요정보통신기반시설 binding

A KR critical-information-infrastructure
operator binds the per-cycle evaluation to
the KR 중요정보통신기반시설 보호법 (Critical
Information Infrastructure Protection Act)
and to the KR 정보보호최고책임자 (CISO)
designation envelope per the act.

### §11.3 KR 전자금융거래법 binding

A KR financial-services operator binds the
per-cycle evaluation to the KR 전자금융거래법
(Electronic Financial Transactions Act) §21
electronic-finance information-security
discipline, where applicable to the operator's
sector.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## §12 Quantitative-and-Qualitative Reconciliation

An operator running both the NIST SP 800-30
qualitative scoring and the Open FAIR
quantitative scoring publishes a per-risk
reconciliation envelope. The reconciliation
maps the qualitative overall-risk level (very-
low through very-high) to the Open FAIR Loss-
Magnitude band declared in the programme's
risk-measurement criteria.

## §13 Federated-Risk-Programme Discipline

A federated risk-management programme (a
holding company's subsidiary risk programme,
a multi-jurisdiction sectoral programme, a
managed-security-services-provider's
multitenant programme) publishes the per-
member programme's per-cycle envelope to the
federation's central register. The federation
publishes the per-cycle aggregate-risk
register without per-member identifiers.

## §14 Privileged-Access Management Discipline

The operator's per-cycle evaluation includes
the per-asset privileged-access-management
declaration. A high-FIPS-PUB-199 asset MUST
declare a privileged-access-management
discipline anchored to NIST SP 800-53 AC-6
(Least Privilege), AC-2 (Account Management),
and AU-2 (Event Logging) controls.
