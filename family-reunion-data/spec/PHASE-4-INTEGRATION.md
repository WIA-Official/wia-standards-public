# WIA-family-reunion-data PHASE 4 — INTEGRATION Specification

**Standard:** WIA-family-reunion-data
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how a humanitarian
operator integrates with the systems that
surround restoring family links: the ICRC
Central Tracing Agency operating the global
RFL network; the UNHCR Operational Data
Portal aggregating refugee statistics; the IOM
DTM regional dashboard; the OCHA Centre for
Humanitarian Data hosting the Humanitarian
Data Exchange (HDX); the IASC cluster
coordination endpoints; the destination-state
reception authority receiving a transferred
case; the Hague-1980 / Hague-1993 Central
Authority of the destination state; the
Interpol Yellow Notice register; the
supervisory data-protection authority
overseeing GDPR Article 9 processing; and the
public-procurement authority running a
humanitarian-data interoperability programme.

References (CITATION-POLICY ALLOW only):

- 1949 Geneva Conventions and 1977 Additional
  Protocols
- 1951 Refugee Convention and 1967 Protocol
- 1989 CRC, 1993 Hague ICA, 1980 Hague ICCA,
  1963 VCCR Article 36, 1961 Statelessness
  Convention
- ICRC RFL Strategy and Code of Conduct
- ICRC Professional Standards for Protection
  Work
- UNHCR ProGres v4 schema, UNHCR Personal
  Data Policy, UNHCR Operational Data Portal
- IOM DTM Methodological Framework, IOM Data
  Protection Manual
- HXL Standard, OCHA HDX
- Sphere Handbook, IASC Operational Guidelines
- ISO 19115, ISO 6709, ISO 3166-1/-2, ISO
  5218, ISO 639, ISO 4217
- ISO 9001:2015, ISO/IEC 27001:2022, ISO/IEC
  27018:2019
- ISO/IEC 17021-1:2015, ISO/IEC 17065:2012
- IETF RFC 8259, RFC 9457, RFC 8615, RFC
  9421, RFC 6234, RFC 8032, RFC 6962
- W3C Trace Context, W3C ODRL 2.2, W3C VC
  v2.0
- Interpol Notices Manual (Yellow Notices for
  missing persons)
- IATI Standard (International Aid
  Transparency Initiative)
- EU GDPR Articles 6, 9, 12-22, 25, 32, 33-34,
  46-49, 89
- EU Council Directive 2003/86/EC
- KR 출입국관리법, KR 난민법, KR 아동복지법, KR
  개인정보보호법

---

## §1 ICRC Central Tracing Agency Integration

### §1.1 Global RFL network

A national Red Cross or Red Crescent society
participating in the ICRC RFL global network
binds the per-case file to the ICRC's central
case register through the operator's
`icrcCoordinationRef`. The ICRC's central
register coordinates cross-border tracing and
maintains the per-conflict tracing dataset.

### §1.2 ICRC online tracing service

A registered person seeking to be found by a
relative may opt-in to the ICRC online tracing
service. The opt-in is recorded as part of the
person's consent directive; the operator
publishes the opt-in entry to the ICRC tracing
service through the ICRC's federation
endpoint.

## §2 UNHCR Operational Data Portal Integration

A UNHCR-coordinated registration is published
to the UNHCR Operational Data Portal through
the per-operation aggregation pipeline. The
portal aggregates per-operation statistics
(per-country-of-origin numbers, per-vulnerability
breakdowns) without disclosing per-individual
identifiers.

## §3 IOM DTM Regional Dashboard Integration

An IOM DTM operation publishes the per-site
assessment data to the IOM regional dashboard.
The dashboard aggregates the data per the IOM
DTM Methodological Framework and presents
the per-region displacement statistics.

## §4 OCHA HDX Integration

The operator's HXL feed published under PHASE-1
§7 is ingested by the OCHA Humanitarian Data
Exchange. The HDX entry carries the per-feed
HXL Tag set so that a downstream consumer can
filter by tag.

## §5 IASC Cluster Coordination Integration

A cluster-coordinated operation binds the per-
case file to the cluster's coordination
endpoint:

- The Protection cluster ingests the per-case
  protection-monitoring summary.
- The Camp Coordination and Camp Management
  cluster ingests the per-site population-
  management summary.
- The Child Protection sub-cluster ingests the
  per-case unaccompanied-or-separated-minor
  summary.

The cluster's information-management officer
publishes the aggregated dataset on the
cluster's portal.

## §6 Destination-State Reception Authority Integration

A transferred case bound for a destination
state is published to the destination-state's
reception-authority endpoint. The reception
authority's response carries the per-case
acceptance status, the reception location
reference, and the receiving guardian
reference where the case is a minor.

## §7 Hague Central Authority Integration

### §7.1 Hague-1993 inter-country adoption

An ICA case under the 1993 Hague Convention
binds the case to both the originating and
the receiving Central Authorities. The
operator's API publishes the per-case file to
the Central Authority's intake endpoint with
the consent of both the originating and the
receiving authorities.

### §7.2 Hague-1980 child-abduction return

A Hague-1980 abduction case binds the case to
the destination-state's Central Authority. The
operator's API publishes the return application
through the Central Authority's intake endpoint.

## §8 Interpol Yellow-Notice Integration

A missing-persons case may be coordinated
through an Interpol Yellow Notice. The
operator's API binds the per-case file to the
Interpol Yellow Notice identifier where the
operator is empowered to request the issuance
through the operator's national central
bureau.

## §9 Supervisory Data-Protection Authority Integration

The supervisory data-protection authority
overseeing the operator's GDPR Article 9
processing audits the operator's records of
processing activities (Article 30) on demand.
The operator's API publishes the records to
the authority's endpoint so that the
authority's audit trail is preserved.

## §10 Public Retrieval and Re-Issuance

### §10.1 Public summary retrieval

A public consumer (a researcher running an
academic study, a journalist running a public-
interest investigation) retrieves the per-
operation aggregated summary at
`/v1/programmes/{programmeId}/public-summary`
without per-individual identifiers. The summary
is consistent with the operator's documented
public-disclosure policy under the operator's
data-protection regime.

### §10.2 Verifiable-credentials re-issuance

A positive-identification attestation may be
re-issued to the registered person and the
named relative as a W3C Verifiable Credential
signed by the operator's public-key set. The
credential carries the per-case identification
reference, the date of identification, and
the issuing authority's reference. The
credential allows the receiving person to
demonstrate the identification to a downstream
authority (a destination-state reception
authority, an inter-country adoption Central
Authority) without disclosing the underlying
case file.

## §11 KR-Jurisdiction Integration

### §11.1 KR Ministry of the Interior and Safety

A KR-jurisdiction operator binds the
registration to the KR Ministry of the
Interior and Safety's missing-persons register
where the operator is empowered to publish
to the register.

### §11.2 KR National Police Agency missing-
       persons register

A KR-jurisdiction operator coordinates with
the KR National Police Agency's missing-
persons register where the case includes a KR
domestic missing-persons declaration.

### §11.3 KR 적십자사 / KR Korean Red Cross
       integration

A KR-jurisdiction operator participating in
the ICRC RFL network through the Korean Red
Cross binds the per-case file to the Korean
Red Cross's RFL coordination endpoint.

## §12 IATI Transparency Integration

Where the operator publishes its operational
funding under the IATI Standard, the per-
operation funding envelope is bound to the
IATI publisher identifier and the per-activity
identifier. The IATI publication does not
disclose per-individual case data; it
discloses operational-level funding and
beneficiary-aggregate counts.

## §13 Audit and Conformity-Assessment Integration

### §13.1 ISO/IEC 17021-1 management-system audit

The operator's quality-management system
declared in PHASE-3 §9 is audited under
ISO/IEC 17021-1 by an accredited certification
body. The audit result is stored in the
operator's audit envelope.

### §13.2 ISO/IEC 17065 product certification

Where the operator's per-case attestation (the
per-case identification anchor signed
attestation) is issued under a product-
certification scheme, the certification body's
ISO/IEC 17065:2012 accreditation is bound to
the certification reference.

## §14 References (consolidated)

The references list across PHASE-1 to PHASE-4
is the canonical citation set for the WIA-
family-reunion-data standard. Implementations
cite the international conventions and
treaties by their formal name (the 1949 Geneva
Conventions, the 1989 CRC, the 1993 Hague ICA,
the 1980 Hague ICCA, the 1963 VCCR, the 1961
Statelessness Convention, the 1951 Refugee
Convention) and the ICRC, UNHCR, IOM, and
OCHA frameworks by their published name so
that a downstream consumer can locate the
authoritative text. Updates to a cited
framework (for example, a new edition of the
ICRC RFL Strategy or a new release of UNHCR
ProGres) trigger an internal review cycle in
the operator's quality-management discipline
declared in PHASE-3 §9 before the new
revision is bound into the operator's
enumeration set.
