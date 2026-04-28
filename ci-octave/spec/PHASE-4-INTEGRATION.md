# WIA-ci-octave PHASE 4 — Integration Specification

**Standard:** WIA-ci-octave
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This document defines how a CI-OCTAVE risk-
management operator integrates with the
systems that surround the asset-to-treatment-
to-audit value chain: the CMU SEI CERT
Division operating the OCTAVE Allegro / FORTE
methodology references; the ISO/IEC 27001
certification body issuing the per-cycle
certification; the NIST RMF authorising
official issuing the per-system Authorisation
to Operate; the EU NIS2 / DORA / CRA
supervisory authority enforcing the sectoral
risk-management obligations; the sectoral CSIRT
or ISAC operating the per-sector incident-
information-sharing layer; the cyber-insurance
underwriter pricing the residual-risk envelope;
the supply-chain risk-management partner; the
internal-audit function running the second-line
review; and the audit committee approving the
per-cycle risk-treatment plan.

References (CITATION-POLICY ALLOW only):

- CMU SEI CERT OCTAVE Allegro and OCTAVE
  FORTE methodologies, CERT-RMM v1.2
- ISO/IEC 27001:2022, ISO/IEC 27002:2022,
  ISO/IEC 27005:2022, ISO/IEC 27017:2015,
  ISO/IEC 27018:2019, ISO/IEC 27036 series
- ISO 31000:2018, ISO/IEC 31010:2019, ISO/IEC
  17021-1:2015, ISO/IEC 17065:2012
- NIST SP 800-30 Rev. 1, NIST SP 800-37 Rev. 2,
  NIST SP 800-53 Rev. 5, NIST CSF 2.0
- US FIPS PUB 199, FIPS PUB 200, FedRAMP
- EU NIS2 Directive 2022/2555, EU DORA
  Regulation 2022/2554, EU CRA Regulation
  2024/2847
- ENISA Threat Landscape, ENISA Risk Assessment
  Methodology
- Open FAIR (The Open Group)
- IEC 62443-3-2:2020
- W3C Verifiable Credentials Data Model v2.0
- IETF RFC 8259, RFC 9457, RFC 8615, RFC 9421
- W3C Trace Context
- KR 정보통신망 이용촉진 및 정보보호 등에 관한
  법률, KR 개인정보 보호법, KR 전자금융거래법, KR
  중요정보통신기반시설 보호법
- KR-ISMS-P certification scheme operated by
  KISA

---

## §1 SEI CERT Methodology Integration

The operator subscribes to the CMU SEI CERT
Division's publishing endpoint for OCTAVE
Allegro and OCTAVE FORTE methodology updates.
A new SEI publication triggers an internal
review cycle in the operator's quality-
management discipline declared in PHASE-3 §5
before the new methodology revision is bound
into the operator's per-cycle template.

## §2 ISO/IEC 27001 Certification-Body Integration

A certification body accredited under ISO/IEC
17021-1:2015 issues the per-cycle ISO/IEC
27001 certification on the per-cycle audit
outcome. The certification body queries the
operator's API for the per-cycle audit
record, the per-cycle nonconformity register,
and the per-cycle management response. The
certification body's audit envelope is signed
and recorded in the operator's audit trail.

## §3 NIST RMF Authorising Official Integration

A US federal-agency operator running the NIST
RMF authorisation cycle integrates with the
authorising-official's authorisation-decision
endpoint. The authorisation-decision envelope
is published per NIST SP 800-37 Rev. 2 and
records the per-system Authorisation to
Operate, the per-system continuous-monitoring
strategy, and the per-system authorisation-
termination date.

## §4 EU NIS2 Supervisory-Authority Integration

A NIS2-essential or NIS2-important entity
publishes the per-period risk-management
report to the EU Member-State supervisory
authority. The report carries the per-
obligation evidence register, the per-incident
significant-incident register, and the per-
period business-continuity-test envelope.

## §5 EU DORA Supervisory-Authority Integration

A DORA-financial-entity publishes the per-
period digital-operational-resilience report
to the EU financial-services supervisory
authority. The report carries the per-period
ICT-risk-management framework attestation, the
per-period digital-operational-resilience
testing outcome, and the per-period third-
party ICT-risk register.

## §6 EU CRA Conformity-Assessment Integration

A CRA-product-with-digital-elements
manufacturer publishes the per-product
conformity-assessment envelope per CRA
Article 32. The envelope is verified by an
EU-CRA-designated notified body for high-risk
products and by the manufacturer for default-
risk products under the CRA's conformity-
assessment-procedure tiers.

## §7 Sectoral CSIRT / ISAC Integration

The operator's sectoral CSIRT or ISAC (the EU
CSIRTs Network, the US sector-specific ISACs
— FS-ISAC, H-ISAC, E-ISAC, IT-ISAC, the KR
ISAC operated by KISA) receives the per-
incident significant-incident envelope. The
operator's API publishes the per-incident
envelope to the CSIRT / ISAC's intake endpoint
per the CSIRT's information-sharing
agreement.

## §8 Cyber-Insurance Underwriter Integration

A cyber-insurance underwriter pricing the
operator's residual-risk envelope queries the
operator's API for the per-asset risk-score
register, the per-treatment decision register,
and the per-cycle audit envelope. The
underwriter's per-policy pricing is bound to
the operator's evidence; the per-policy
exclusion clauses are documented in the
operator's risk-transfer envelope.

## §9 Supply-Chain Risk-Management Partner Integration

An operator participating in a supply-chain
risk-management programme (the US CISA
Information and Communications Technology
Supply Chain Risk Management Task Force, the
EU NIS2 supply-chain coordination) publishes
the per-supplier risk envelope per the
operator's data-sharing agreement.

## §10 Internal-Audit Function Integration

The operator's internal-audit function (the
second-line-of-defence, per the IIA Three
Lines Model) runs the per-cycle independent
review. The function's audit envelope is
recorded in the operator's audit trail and is
referenced from the audit-committee response.

## §11 Audit-Committee Integration

The operator's audit committee approves the
per-cycle risk-treatment plan and the per-
cycle residual-risk acceptance envelope. The
committee's per-cycle minutes are stored in
the operator's audit envelope and are
available to the certification body and the
supervisory regulator on demand.

## §12 Public Retrieval and Re-Issuance

### §12.1 Public risk-summary publication

The operator publishes per-period aggregate-
risk statistics (the per-impact-level risk
count, the per-treatment-decision count, the
per-status mitigation count) on the public-
portal endpoint without per-asset
identifiers. The aggregated dataset is
consumed by sectoral analysts, journalists,
and policy researchers.

### §12.2 Verifiable-credentials re-issuance

An ISO/IEC 27001 certification or a NIST RMF
ATO is re-issuable as a W3C Verifiable
Credential signed by the certification body's
or authorising official's signing-key set so
that a downstream counterparty can verify
the certification offline.

## §13 KR-Jurisdiction Integration

### §13.1 KR-ISMS-P certification body integration

A KR-jurisdiction operator certified under
the KISA-operated ISMS-P scheme integrates
with the certification body's audit
endpoint. The body's per-cycle audit envelope
is recorded in the operator's audit trail.

### §13.2 KR 정보보호 최고책임자 designation

A KR-jurisdiction operator designates the
information-security chief officer (정보
보호 최고책임자, CISO) per the KR 정보통신망법.
The operator's API publishes the CISO
designation reference and the per-period
CISO-led risk-evaluation report.

### §13.3 KR 중요정보통신기반시설 binding

A KR critical-information-infrastructure
operator binds the per-cycle evaluation to
the KR Ministry of Science and ICT's
CIIP-supervision endpoint and to the per-
sector CIIP-coordinating-authority's
register.

### §13.4 KR 전자금융감독규정 binding

A KR financial-services operator binds the
per-cycle evaluation to the KR Financial
Services Commission's electronic-finance-
supervision regulation envelope.

## §14 References (consolidated)

The references list across PHASE-1 to PHASE-4
is the canonical citation set for the WIA-
ci-octave standard. Implementations cite the
SEI CERT Division publications, the ISO / IEC
/ NIST / EU / KR references, and the The Open
Group Open FAIR by their issuing organisation
and the publication year so that a downstream
consumer can locate the authoritative text.
Updates to a cited standard (for example, a
new SEI OCTAVE methodology revision, a new
ISO/IEC 27005 amendment, a new NIST CSF
release, a new EU NIS2 implementing-act)
trigger an internal review cycle in the
operator's quality-management discipline
declared in PHASE-3 §5 before the new
revision is bound into the operator's
enumeration set.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## §15 Public-Sector Procurement Integration

A public-sector procurement programme
selecting an OCTAVE-trained MSSP applies the
WTO Government Procurement Agreement
discipline where in scope. The operator
publishes the per-tender entry with the
procurement timetable, the technical-
specifications envelope (the SEI OCTAVE-
trained-evaluator designation reference, the
ISO/IEC 27001 certification reference, the
NIST RMF authorisation reference), and the
award criteria.

## §16 Open-Source Tool Integration

The operator's per-cycle evaluation may use
open-source tooling (the SEI's OCTAVE Allegro
worksheet templates, the ISO/IEC 27005 risk-
register templates, the OWASP Threat Dragon
threat-modelling tool, the MITRE ATT&CK
framework). The operator publishes the per-
cycle tool-version envelope so that a
downstream auditor can reproduce the per-
cycle evidence.

## §17 Continuous Improvement Programme

Each operator publishes an annual improvement
plan addressing per-cycle risk-trend,
treatment-implementation-rate, audit-finding-
remediation-rate, and per-supplier risk
register. The programme is open and the
annual report is published on the operator's
public-portal endpoint per PHASE-2 §12.

## §18 Sectoral-Specific Bindings

### §18.1 Healthcare-IT binding

A healthcare-IT operator binds the per-cycle
evaluation to the HIPAA Security Rule §164.308
risk-analysis requirement (where US scope
applies) or the equivalent national health-
information-security framework. The operator's
API publishes the per-cycle evidence to the
healthcare-IT-supervisory authority.

### §18.2 Financial-services binding

A financial-services operator binds the per-
cycle evaluation to the FFIEC IT Examination
Handbook (where US scope applies), the EU
DORA Regulation (where EU scope applies), or
the equivalent national financial-regulator
framework. The operator's API publishes the
per-cycle evidence to the relevant supervisory
authority.

### §18.3 OT and ICS binding

A critical-infrastructure operator binding
operational-technology assets binds the per-
cycle evaluation to the IEC 62443-3-2:2020
security-risk-assessment-for-system-design
discipline. The operator's API publishes the
per-zone-and-conduit risk-and-vulnerability
register.

## §19 Risk-Maturity Programme Integration

The operator's per-cycle evaluation includes
the operator's risk-maturity declaration per
the CMU SEI CERT-RMM v1.2 maturity-indicator
levels. The maturity declaration is published
to the operator's audit committee and to the
SEI CERT Division's optional benchmarking
endpoint.
