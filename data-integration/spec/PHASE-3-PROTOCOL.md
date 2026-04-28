# WIA-data-integration PHASE 3 — Protocol Specification

**Standard:** WIA-data-integration
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern
a data-integration operator across the source-
to-schema-to-mapping-to-master-to-quality value
chain: the ISO/IEC 11179 metadata-registry
discipline that anchors every per-source schema
descriptor, the W3C R2RML mapping discipline
that publishes the relational-to-RDF mapping,
the ISO 8000-110 master-data exchange
discipline that gates the cross-organisation
master-data publication, the ISO/IEC 25012
data-quality discipline that scores every per-
flow measurement, the ISO/IEC 19763 metamodel-
framework discipline that anchors the
operator's metadata interoperability, the W3C
PROV-O lineage discipline that prevents silent
mutation of the per-flow transformation, the
chain-of-custody anchoring discipline, the
per-jurisdiction privacy discipline, and the
DAMA DMBoK governance discipline.

References (CITATION-POLICY ALLOW only):

- ISO/IEC 11179 series (Parts 1, 3, 5, 6),
  ISO 8000-1:2022, ISO 8000-2:2024, ISO
  8000-100:2016, ISO 8000-110:2021, ISO
  8000-115:2018
- ISO/IEC 25012:2008, ISO/IEC 25024:2015,
  ISO/IEC 19763, ISO/IEC 27001:2022
- W3C R2RML, W3C SPARQL 1.1, W3C RDF 1.1,
  W3C OWL 2, W3C JSON-LD 1.1, W3C SHACL,
  W3C SKOS, W3C DCAT v3, W3C PROV-O
- DAMA DMBoK 2nd edition
- HL7 FHIR R5 (where applicable)
- IETF RFC 9110, RFC 9421, RFC 9457, RFC
  6234, RFC 8615, RFC 6962
- W3C Trace Context
- EU GDPR (Regulation (EU) 2016/679), KR
  개인정보 보호법, KR 데이터기반행정 활성화에
  관한 법률, KR 공공데이터의 제공 및 이용 활
  성화에 관한 법률

---

## §1 ISO/IEC 11179 Metadata-Registry Discipline

### §1.1 Per-element naming discipline

Every per-source data element carries a name
conforming to the ISO/IEC 11179-5:2015 naming
conventions — the operator declares the
naming convention (the "object class word"
discipline, the "property word" discipline,
the "representation term" discipline) at the
programme level, and the operator's API
enforces the convention on each per-element
publication.

### §1.2 Per-element registration authority

Every per-element record is bound to a
registration authority per ISO/IEC 11179-6:
2023. The registration authority's
designation is published with the per-element
record so that a downstream consumer can
verify the per-element provenance.

### §1.3 Conceptual-and-value-domain separation

Every per-element record separates the
conceptual domain (the per-element semantic
meaning) from the value domain (the per-
element representation). The separation is
canonical per ISO/IEC 11179-3 and is
enforced by the operator's API.

## §2 W3C R2RML Mapping Discipline

### §2.1 Per-mapping evidence

Every per-source-and-per-ontology mapping
publishes the per-mapping R2RML document as
a Turtle, RDF/XML, or JSON-LD encoding. The
per-mapping document is parsed and validated
against the W3C R2RML grammar; an invalid
mapping is rejected.

### §2.2 Per-virtual-graph endpoint

The operator's API publishes the per-mapping
W3C SPARQL 1.1 endpoint that exposes the
mapping's virtual graph. The endpoint enforces
the per-query timeout and the per-query
result-size limit declared in the operator's
resource-governance policy.

### §2.3 Per-mapping ontology versioning

A per-mapping ontology revision (a new W3C
OWL 2 ontology release, a new SKOS concept-
scheme release) triggers an internal review
cycle in the operator's quality-management
discipline before the new revision is bound
into the mapping.

## §3 ISO 8000-110 Master-Data Exchange Discipline

### §3.1 Per-exchange conformance level

The operator declares the per-exchange
conformance level — level-1-syntactic, level-
2-semantic, or level-3-quality — per ISO
8000-110. The conformance level gates the
downstream consumer's trust envelope; a
level-1 exchange is not trusted for direct
analytics consumption without an additional
operator-declared quality envelope.

### §3.2 Per-exchange data-specification reference

Every level-2 or level-3 exchange references
a per-exchange data-specification document.
The data-specification document carries the
per-element semantic encoding and the per-
element value-domain reference. The operator's
API publishes the data-specification document
under a content-addressable URI.

### §3.3 Per-exchange quality-identifier binding

A level-3 exchange binds the per-exchange
quality-identifier per ISO 8000-115. The
quality identifier carries the per-exchange
quality measurement reference and the per-
exchange quality-attestation envelope.

## §4 ISO/IEC 25012 Data-Quality Discipline

### §4.1 Per-characteristic measurement

Every per-flow data-quality measurement is
bound to one of the ISO/IEC 25012:2008
quality characteristics. The measurement
function is per ISO/IEC 25024:2015; the
operator's API records the per-measurement
function declaration.

### §4.2 Per-baseline threshold

The operator declares the per-quality-
characteristic baseline at the programme
level; a per-flow measurement falling below
the baseline triggers a per-flow remediation
envelope.

### §4.3 Per-flow trend tracking

The operator's API publishes the per-flow
quality-trend envelope so that a per-flow
deterioration is detectable across periods.

## §5 ISO/IEC 19763 Metamodel-Framework Discipline

The operator declares the per-programme
metadata-interoperability metamodel per
ISO/IEC 19763 so that a downstream consumer
can deterministically interpret the per-
source schema and the per-mapping ontology.

## §6 W3C PROV-O Lineage Discipline

### §6.1 Per-flow lineage graph

Every per-flow transformation publishes the
per-flow PROV-O lineage graph. The graph
records the per-flow upstream-source
dependency, the per-flow transformation
envelope (the per-step transformation rule,
the per-step parameter envelope), and the
per-flow downstream-consumer dependency.

### §6.2 Per-flow signing

The lineage graph is signed by the operator's
signing-key set so that a downstream
consumer can verify the lineage envelope
without contacting the operator directly.

## §7 Chain-of-Custody Anchoring Discipline

### §7.1 Per-event transparency log

Every chain-of-custody event carried by
PHASE-1 §8 is appended to a per-operator
transparency log modelled on the IETF RFC 6962
Certificate Transparency append-only-log
structure.

### §7.2 Mutation prevention

A custody event cannot be retroactively
edited; an amendment is recorded as a new
event with `previousEventRef` pointing at the
event being amended.

## §8 Privacy Discipline

### §8.1 GDPR / KR 개인정보 보호법 binding

A per-flow envelope processing personal data
is bound to the relevant per-jurisdiction
privacy framework. A per-flow processing
under EU jurisdiction is bound to GDPR
Articles 6 (lawful basis), 9 (special-
category data where applicable), 12-22 (data
subject rights), 24-30 (controller
obligations), 32-34 (security and breach
notification). A per-flow processing under
KR jurisdiction is bound to KR 개인정보 보호법
§15, §17, §18, §28-2, and §35.

### §8.2 Per-flow purpose limitation

Every per-flow envelope carries the per-flow
purpose declaration. A per-flow processing
beyond the declared purpose is rejected with
`403 Forbidden` at `/problems/per-flow-
purpose-mismatch`.

### §8.3 Per-flow pseudonymisation

A per-flow envelope carrying personal data
applies the per-flow pseudonymisation
declared in the operator's privacy
discipline. The per-flow re-identification
table is held in the operator's identity
vault and is access-gated.

## §9 DAMA DMBoK Governance Discipline

The operator declares the per-programme
governance-and-management envelope per the
DAMA DMBoK 2nd edition functional-framework
domains (Data Governance, Data Architecture,
Data Modeling and Design, Data Storage and
Operations, Data Security, Data Integration
and Interoperability, Documents and Content,
Reference and Master Data, Data Warehousing
and Business Intelligence, Metadata, Data
Quality).

## §10 KR-Jurisdiction Discipline

### §10.1 KR 데이터기반행정 binding

A KR-jurisdiction public-sector operator is
bound to the KR 데이터기반행정 활성화에 관한
법률 (Act on Promotion of Data-Based
Administration). The operator's API publishes
the per-period administrative-data envelope
to the KR Ministry of the Interior and
Safety's data-based-administration platform.

### §10.2 KR 공공데이터법 binding

A KR-jurisdiction public-sector operator
publishing open-data is bound to the KR
공공데이터의 제공 및 이용 활성화에 관한 법률
(Act on Promotion of the Provision and Use
of Public Data). The operator's API publishes
the per-dataset envelope to the KR Open Data
Portal operated by the Ministry of the
Interior and Safety.

### §10.3 KR-MyData binding

A KR financial-services operator publishing
the per-customer MyData envelope is bound to
the KR 신용정보의 이용 및 보호에 관한 법률
(Credit Information Use and Protection Act)
and the KR Financial Services Commission's
MyData supervision regulation.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## §11 Schema-Evolution Discipline

A per-source schema revision (a per-element
addition, a per-element deprecation, a per-
element semantic redefinition) is bound to
the operator's schema-evolution discipline.
A non-breaking change (an additive per-element
addition, an optional per-element addition)
is published with a minor-version bump; a
breaking change (a per-element removal, a
per-element semantic redefinition, a per-
element value-domain restriction) is published
with a major-version bump and a per-revision
deprecation envelope declaring the per-
revision sunset window.

## §12 Per-Schema Backwards-Compatibility Discipline

The operator's API publishes the per-schema
backwards-compatibility envelope per the
operator's schema-evolution discipline. A per-
flow consumer subscribed to the per-schema
webhook receives the per-revision sunset
notice and runs the per-flow migration before
the sunset window closes.

## §13 Per-Programme Federation Discipline

A federated data-integration programme (a
multi-domain data-mesh, a multi-tenant data-
fabric, a multi-jurisdiction master-data
exchange) publishes the per-member programme
envelope to the federation's central register
per the federation's information-sharing
agreement.

## §14 Per-Flow Failure-Recovery Discipline

A per-flow failure (an upstream-source
unavailability, a per-flow transformation
exception, a downstream-target schema
mismatch) triggers the per-flow failure-
recovery envelope. The envelope carries the
per-flow retry-count, the per-flow back-off
schedule, the per-flow dead-letter destination,
and the per-flow operator-notification path.

## §15 ISO/IEC 38505 Data-Governance Discipline

The operator's per-programme governance is
bound to ISO/IEC 38505-1:2017 (Governance of
IT — Governance of Data). The operator's
per-period governance review carries the per-
period evaluate-direct-monitor (EDM) cycle,
the per-period assurance envelope, and the
per-period stakeholder-engagement record.

## §16 ISO/IEC 27040 Storage-Security Discipline

A per-source-or-per-mapping at-rest envelope
is bound to ISO/IEC 27040:2024 (information
technology — security techniques — storage
security) where applicable. The operator's
API publishes the per-storage envelope's
encryption-at-rest declaration, the per-
storage backup envelope, and the per-storage
disposal envelope.

## §17 Per-Mapping Reference-Data Discipline

A per-mapping envelope referencing a per-
domain reference-data set (ISO 4217 currency,
ISO 3166 country, ISO 8601 date-time, ISO
639-3 language, FAO commodity code, GS1
identifier) is bound to the reference-data
publisher's licensing envelope and the
reference-data publisher's update cadence.
