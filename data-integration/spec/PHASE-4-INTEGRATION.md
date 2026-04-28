# WIA-data-integration PHASE 4 — Integration Specification

**Standard:** WIA-data-integration
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This document defines how a data-integration
operator integrates with the systems that
surround the source-to-master value chain: the
ISO/IEC 11179 metadata-registry administered by
the per-jurisdiction registration authority;
the W3C R2RML / RDF / SPARQL ecosystem; the
ISO 8000 master-data exchange ecosystem; the
ISO/IEC 25012 data-quality ecosystem; the data-
catalogue platform consuming the per-programme
DCAT v3 catalogue; the analytics platform
consuming the per-source virtual-graph endpoint;
the supervisory data-protection authority
overseeing per-flow GDPR / KR 개인정보 보호법
compliance; the financial-regulatory authority
overseeing the financial-data-integration flow;
the supply-chain partner consuming the master-
data exchange; and the public-sector open-data
portal.

References (CITATION-POLICY ALLOW only):

- ISO/IEC 11179 series, ISO 8000 series,
  ISO/IEC 25012:2008, ISO/IEC 25024:2015,
  ISO/IEC 19763, ISO/IEC 27001:2022, ISO/IEC
  17021-1:2015, ISO/IEC 17065:2012
- W3C R2RML, W3C SPARQL 1.1, W3C RDF 1.1,
  W3C OWL 2, W3C JSON-LD 1.1, W3C SHACL,
  W3C SKOS, W3C DCAT v3, W3C PROV-O, W3C
  Verifiable Credentials Data Model v2.0
- HL7 FHIR R5, HL7 v2.x messaging
- DAMA DMBoK 2nd edition
- IETF RFC 8259, RFC 9457, RFC 8615, RFC 9421
- W3C Trace Context
- EU Regulation (EU) 2022/868 (Data Governance
  Act), EU Regulation (EU) 2023/2854 (Data
  Act), EU GDPR (Regulation (EU) 2016/679)
- KR 데이터기반행정 활성화에 관한 법률, KR 공공
  데이터의 제공 및 이용 활성화에 관한 법률, KR
  개인정보 보호법, KR 신용정보의 이용 및 보호에
  관한 법률
- US OMB Federal Data Strategy

---

## §1 ISO/IEC 11179 Registration-Authority Integration

The per-jurisdiction registration authority
(the ISO Central Secretariat for the global
metadata-registry, the per-country national
standards body for the national-domain
metadata-registry) operates the per-element
registration register. The operator queries
the registration authority's register on each
per-element publication to verify the per-
element provenance.

## §2 W3C R2RML / RDF / SPARQL Ecosystem Integration

The operator's per-mapping virtual-graph
endpoint integrates with the broader RDF /
SPARQL ecosystem — the W3C-listed SPARQL
endpoint registry, the per-domain ontology
network (the FOAF ontology, the Schema.org
vocabulary, the Dublin Core terms, the
domain-specific ontologies). The operator's
per-mapping ontology references are published
as resolvable URIs.

## §3 ISO 8000 Master-Data Ecosystem Integration

The operator's per-domain master-data exchange
integrates with the ISO 8000 ecosystem — the
per-domain ISO 8000-110 conformance reference,
the per-exchange ISO 8000-115 quality-
identifier, the per-exchange ISO 8000-120
provenance envelope. The operator's API
publishes the per-domain conformance evidence.

## §4 ISO/IEC 25012 Data-Quality Ecosystem Integration

The operator publishes the per-flow quality
measurement to the data-quality ecosystem —
the operator's internal data-quality dashboard,
the per-domain data-quality benchmarking
programme, the per-jurisdiction supervisory-
authority quality reporting envelope.

## §5 Data-Catalogue Platform Integration

A data-catalogue consumer (a corporate-data-
catalogue platform like Apache Atlas /
DataHub / OpenMetadata, an open-data portal
like the EU Data Portal / data.gov / data.go.kr)
ingests the operator's per-programme DCAT v3
catalogue. The catalogue ingestion is per
W3C DCAT v3 with the per-distribution
metadata.

## §6 Analytics Platform Integration

An analytics platform (a data-warehouse query
engine, a federated-query engine, a notebook-
based analytics platform) consumes the
operator's per-source virtual-graph endpoint
via W3C SPARQL 1.1. The operator's API
publishes the per-source endpoint authentication
envelope (the per-consumer credentials, the
per-consumer quota envelope).

## §7 Supervisory Data-Protection Authority Integration

The supervisory data-protection authority
overseeing the operator's per-flow processing
audits the operator's records of processing
activities on demand. Where the operator
processes personal data under GDPR, the
operator publishes the records of processing
activities per Article 30 to the authority.
Where the operator processes personal data
under KR 개인정보 보호법, the operator publishes
the per-period privacy-impact-assessment
summary per §33.

## §8 Financial-Regulatory Authority Integration

A financial-services operator (a bank, an
insurance company, an investment manager)
participating in the per-jurisdiction
financial-data-integration programme integrates
with the financial-regulatory authority. The
operator publishes the per-period regulatory-
report envelope per the per-authority reporting
schedule (the EU EBA reporting framework, the
US FFIEC Call Report, the KR Financial
Services Commission's regulatory-reporting
envelope).

## §9 Supply-Chain Partner Integration

A supply-chain partner consuming the operator's
master-data exchange (a customer consuming the
operator's product master, a supplier
consuming the operator's location master, a
logistics partner consuming the operator's
asset master) ingests the per-domain exchange
per ISO 8000-110. The per-partner ingestion
is gated by the operator's per-partner data-
sharing agreement.

## §10 EU Data Governance Act Integration

A data-intermediary operator under the EU
Regulation (EU) 2022/868 (Data Governance
Act) is bound to the act's Articles 10-14
(data-intermediation services). The operator
publishes the per-period data-intermediation
report to the EU Member-State competent
authority.

## §11 EU Data Act Integration

A connected-product operator under the EU
Regulation (EU) 2023/2854 (Data Act) is
bound to the act's per-product data-access
discipline. The operator publishes the per-
product data-access envelope to the user and
to the user's authorised third party.

## §12 Audit and Conformity-Assessment Integration

### §12.1 ISO/IEC 17021-1 management-system audit

The operator's quality-management system is
audited under ISO/IEC 17021-1 by an
accredited certification body. The audit
result is stored in the operator's audit
envelope.

### §12.2 ISO/IEC 17065 product certification

A per-domain master-data exchange whose route
to market includes a product-certification
mark (the EU Data-Act-conformity mark, the
operator's per-domain quality-certification
mark) is bound to the certification body's
ISO/IEC 17065:2012 accreditation.

## §13 Public Retrieval and Re-Issuance

### §13.1 Public catalogue summary

The operator publishes per-period catalogue
statistics on the public-portal endpoint —
total datasets, total per-distribution
encodings, total per-license envelopes —
without per-record identifiers.

### §13.2 Verifiable-credentials re-issuance

An ISO 8000-110 conformance attestation is
re-issuable as a W3C Verifiable Credential
signed by the certification body's signing-
key set so that a downstream consumer can
verify the conformance attestation without
contacting the certification body directly.

## §14 KR-Jurisdiction Integration

### §14.1 KR Open Data Portal binding

A KR-jurisdiction public-sector operator
publishing open-data binds the per-dataset
envelope to the KR Open Data Portal operated
by the Ministry of the Interior and Safety.

### §14.2 KR 데이터기반행정 binding

A KR-jurisdiction public-sector operator's
per-period administrative-data envelope binds
to the KR Ministry of the Interior and
Safety's data-based-administration platform.

### §14.3 KR-MyData binding

A KR financial-services operator's per-
customer MyData envelope binds to the KR
Financial Services Commission's MyData
supervision platform.

### §14.4 KR 데이터 산업진흥법 binding

A KR-jurisdiction data-industry operator
binds the operator's per-period data-industry
report to the KR 데이터 산업진흥과 이용촉진에
관한 기본법 (Framework Act on Data Industry
Promotion and Use Encouragement) and the KR
Ministry of Science and ICT's data-industry-
promotion platform.

## §15 References (consolidated)

The references list across PHASE-1 to PHASE-4
is the canonical citation set for the WIA-
data-integration standard. Implementations
cite the ISO / IEC / W3C / IETF / EU / KR
references by their issuing organisation and
the publication year so that a downstream
consumer can locate the authoritative text.
Updates to a cited standard (for example, an
amendment to ISO 8000-110, a new W3C SPARQL
release, a new EU Data Act delegated act)
trigger an internal review cycle in the
operator's quality-management discipline
declared in PHASE-3 §9 before the new
revision is bound into the operator's
enumeration set.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## §16 Open-Source Tool Integration

The operator's per-flow integration may use
open-source tooling (Apache Airflow for
orchestration, Apache Kafka for streaming,
dbt for transformation, OpenLineage for per-
flow lineage). The operator publishes the
per-flow tool-version envelope so that a
downstream auditor can reproduce the per-flow
evidence.

## §17 Per-Programme Continuous Improvement

Each operator publishes an annual improvement
plan addressing per-flow latency, per-flow
data-quality trend, per-flow schema-evolution
discipline, and per-domain master-data
exchange conformance. The programme is open
and the annual report is published on the
operator's public-portal endpoint per
PHASE-2 §11.

## §18 ISO/IEC 38505 Data-Governance Integration

The operator's per-programme governance
discipline is bound to ISO/IEC 38505-1:2017
(Governance of IT — Governance of Data —
Application of ISO/IEC 38500 to the governance
of data) where the operator declares the
ISO/IEC 38500-aligned governance envelope.
