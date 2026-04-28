# WIA-data-integration PHASE 1 — Data Format Specification

**Standard:** WIA-data-integration
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format
layer for WIA-data-integration. The standard
covers the persistent record shapes that an
enterprise data-integration operator (an
enterprise data-integration platform vendor, an
extract-transform-load engineering team, an
extract-load-transform engineering team, a
master-data-management programme operator, a
data-mesh domain team, a data-fabric platform
operator, an open-data publisher, a federated-
analytics operator, a healthcare-data-exchange
operator under HL7 FHIR R5) maintains when
declaring a data-source registration, recording
a per-source schema descriptor anchored to
ISO/IEC 11179 metadata-registry vocabulary,
publishing a relational-to-RDF mapping per W3C
R2RML, declaring a master-data record per ISO
8000-100 / -110 / -115, anchoring the per-flow
data-quality measurement per ISO/IEC 25012, and
tracking the per-flow chain-of-custody. Records
are consumed by the downstream data-product
team, by the central data-governance committee,
by the supervisory data-protection authority
overseeing per-flow GDPR / KR 개인정보 보호법
compliance, by the financial-regulatory authority
overseeing the financial-data-integration flow,
and by the supply-chain partner consuming the
master-data exchange.

References (CITATION-POLICY ALLOW only):

- ISO/IEC 11179-1:2023 (information technology
  — metadata registries — Part 1 framework),
  ISO/IEC 11179-3:2023 (Part 3 registry
  metamodel and basic attributes), ISO/IEC
  11179-5:2015 (Part 5 naming principles),
  ISO/IEC 11179-6:2023 (Part 6 registration)
- ISO 8000-1:2022 (data quality — overview),
  ISO 8000-2:2024 (vocabulary), ISO 8000-100:
  2016 (master data — overview), ISO 8000-110:
  2021 (master data — exchange of characteristic
  data — syntax, semantic encoding, conformance
  to data specification), ISO 8000-115:2018
  (master data — exchange of quality
  identifiers)
- ISO/IEC 25012:2008 (software product quality
  — data quality model), ISO/IEC 25024:2015
  (measurement of data quality)
- W3C R2RML (RDB to RDF Mapping Language) —
  W3C Recommendation 2012-09-27
- W3C SPARQL 1.1 Query Language, W3C SPARQL
  1.1 Update, W3C SPARQL 1.1 Federated Query
- W3C RDF 1.1 Concepts and Abstract Syntax,
  W3C RDF Schema 1.1, W3C OWL 2 Web Ontology
  Language Profiles
- W3C JSON-LD 1.1, W3C SHACL (Shapes Constraint
  Language), W3C SKOS (Simple Knowledge
  Organization System) Reference
- W3C Data Catalog Vocabulary (DCAT) v3, W3C
  Provenance Ontology (PROV-O)
- HL7 FHIR R5 (the healthcare-data-exchange
  reference, where applicable to the
  operator's healthcare-data-integration scope)
- ISO/IEC 19763 (information technology —
  metamodel framework for interoperability)
- DAMA DMBoK 2nd edition (Data Management Body
  of Knowledge) — referenced as the management
  vocabulary
- IETF RFC 7159 (JSON), RFC 8259 (JSON), RFC
  4122 (UUID), ISO 8601 (date-time)
- ISO/IEC 27001:2022 (information security
  management — used for the chain-of-custody
  record discipline in §8)
- KS X 8000-100 (Korean adoption of ISO 8000-
  100)
- KR 데이터기반행정 활성화에 관한 법률, KR 공공
  데이터의 제공 및 이용 활성화에 관한 법률, KR
  개인정보 보호법

---

## §1 Scope

This PHASE defines persistent shapes for the
artefacts exchanged when a data-integration
operator registers a data source, anchors the
per-source schema to ISO/IEC 11179, publishes
the relational-to-RDF mapping per W3C R2RML,
declares the master-data exchange per ISO
8000-110, measures the per-flow data-quality
per ISO/IEC 25012, and tracks the per-flow
chain of custody. Implementations covered
include:

- An ETL or ELT engineering team running a
  per-period batch pipeline from operational
  systems (the company's enterprise resource
  planning system, the company's customer
  relationship management system, the
  company's e-commerce platform) to the data
  warehouse.
- A master-data-management programme operator
  publishing the per-domain master-data
  exchange (the customer master, the product
  master, the supplier master, the chart-of-
  accounts master) to the downstream
  consumers.
- A data-mesh domain team operating the
  domain's data-product publishing flow per
  the per-domain data-product specification.
- A data-fabric platform operator orchestrating
  per-source-and-per-target integration flows
  with per-flow lineage tracking.
- An open-data publisher publishing the per-
  catalogue dataset descriptor per W3C DCAT
  v3 to the open-data portal.
- A federated-analytics operator publishing
  the per-query result envelope per W3C SPARQL
  1.1 Federated Query.
- A healthcare-data-exchange operator publishing
  the per-patient health-record exchange per
  HL7 FHIR R5.

The ISO/IEC 11179 metadata-registry envelope,
the W3C R2RML mapping envelope, the ISO 8000-
110 master-data exchange envelope, and the
ISO/IEC 25012 data-quality envelope receive
distinct encodings in this PHASE; the additional
safeguards required by each integration domain
are encoded in PHASE-3 §3.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name of the
                       operator — enterprise
                       data team, MDM programme,
                       data-mesh domain team,
                       data-fabric platform
                       vendor, open-data
                       publisher, healthcare-
                       data-exchange operator)
operatorRole         : enum ("etl-engineering" |
                       "elt-engineering" |
                       "mdm-programme" | "data-
                       mesh-domain" | "data-
                       fabric-platform" |
                       "open-data-publisher" |
                       "federated-analytics" |
                       "healthcare-data-exchange"
                       | "user-defined")
governingFrameworks  : array of enum ("ISO-IEC-
                       11179-1" | "ISO-IEC-11179-
                       3" | "ISO-IEC-11179-5" |
                       "ISO-IEC-11179-6" |
                       "ISO-8000-100" | "ISO-
                       8000-110" | "ISO-8000-
                       115" | "ISO-IEC-25012" |
                       "ISO-IEC-25024" | "W3C-
                       R2RML" | "W3C-SPARQL-1.1"
                       | "W3C-RDF-1.1" | "W3C-
                       OWL-2" | "W3C-JSON-LD-
                       1.1" | "W3C-SHACL" |
                       "W3C-SKOS" | "W3C-DCAT-v3"
                       | "W3C-PROV-O" | "ISO-IEC-
                       19763" | "DAMA-DMBoK-2"
                       | "HL7-FHIR-R5" | "user-
                       defined")
accreditationStatus  : object (the ISO/IEC 27001
                       certification reference,
                       the per-domain ISO 8000-
                       110 conformance reference,
                       the per-flow data-quality
                       attestation reference)
programmeStatus      : enum ("design" | "operating"
                       | "limited-rollout" |
                       "wind-down" | "archived")
```

## §3 Data-Source Record (ISO/IEC 11179-Anchored)

```
sourceRecord:
  sourceId           : string (uuidv7)
  programmeRef       : string (PHASE-1 §2 record
                       reference)
  sourceName         : string (the operator's
                       canonical name for the
                       data source)
  sourceKind         : enum ("relational-database"
                       | "noSQL-document-store"
                       | "noSQL-key-value-store"
                       | "graph-database" |
                       "data-warehouse" |
                       "data-lake" | "object-
                       store" | "message-queue"
                       | "streaming-platform" |
                       "rest-api" | "soap-api"
                       | "graphql-api" | "user-
                       defined")
  technicalContact   : object (the per-source
                       technical contact —
                       owning team, on-call
                       roster, escalation channel)
  schemaRef          : string (PHASE-1 §4 record
                       reference)
  refreshCadence     : enum ("real-time" |
                       "near-real-time" | "hourly"
                       | "daily" | "weekly" |
                       "monthly" | "quarterly" |
                       "ad-hoc" | "user-defined")
```

## §4 Schema Descriptor (ISO/IEC 11179)

```
schemaDescriptor:
  schemaDescriptorId : string (uuidv7)
  sourceRef          : string (PHASE-1 §3 record
                       reference)
  dataElements       : array of object (per-
                       element — the ISO/IEC
                       11179-3 metadata-registry
                       data-element identifier,
                       the data-element name per
                       ISO/IEC 11179-5 naming
                       conventions, the data-
                       element definition, the
                       value-domain reference,
                       the per-element registration
                       authority)
  conceptualDomain   : object (per ISO/IEC
                       11179-3, the conceptual-
                       domain reference)
  valueDomains       : array of object (per
                       ISO/IEC 11179-3, the
                       value-domain references —
                       enumerated value sets,
                       described value sets)
  registrationAuthority : object (per ISO/IEC
                       11179-6, the registration-
                       authority reference)
```

## §5 R2RML Mapping Record

```
mappingRecord:
  mappingRecordId    : string (uuidv7)
  sourceRef          : string (PHASE-1 §3 record
                       reference)
  ontologyRef        : string (the W3C OWL 2
                       ontology reference the
                       mapping targets)
  triplesMaps        : array of object (per W3C
                       R2RML, the per-table or
                       per-view triples-map
                       declaration — logicalTable,
                       subjectMap, predicateObject
                       Maps)
  mappingDocumentRef : string (the per-document
                       URI of the R2RML mapping
                       document)
```

## §6 Master-Data Exchange Record (ISO 8000-110)

```
masterDataExchange:
  exchangeRecordId   : string (uuidv7)
  programmeRef       : string (PHASE-1 §2 record
                       reference)
  domainRef          : enum ("customer-master" |
                       "product-master" |
                       "supplier-master" |
                       "chart-of-accounts-master"
                       | "location-master" |
                       "asset-master" | "user-
                       defined")
  payloadRef         : string (the per-exchange
                       payload URI)
  syntacticEncoding  : enum ("xml" | "json" |
                       "csv" | "iso-15926" |
                       "user-defined")
  semanticEncoding   : object (per ISO 8000-110,
                       the per-element semantic
                       encoding declaration —
                       the data-specification
                       reference)
  conformanceLevel   : enum ("level-1-syntactic"
                       | "level-2-semantic" |
                       "level-3-quality")
```

## §7 Data-Quality Measurement Record (ISO/IEC 25012)

```
qualityRecord:
  qualityRecordId    : string (uuidv7)
  sourceRef          : string (PHASE-1 §3 record
                       reference)
  qualityCharacteristic : enum ("accuracy" |
                       "completeness" |
                       "consistency" |
                       "credibility" |
                       "currentness" |
                       "accessibility" |
                       "compliance" |
                       "confidentiality" |
                       "efficiency" |
                       "precision" |
                       "traceability" |
                       "understandability" |
                       "availability" |
                       "portability" |
                       "recoverability")
  measurementValue   : number (per ISO/IEC
                       25024, the measured
                       quality value)
  measurementBaseline : number (per ISO/IEC
                       25024, the operator-
                       declared quality baseline)
  measurementOutcome : enum ("pass" | "fail" |
                       "warning")
```

## §8 Chain-of-Custody Record

```
custodyRecord:
  custodyId          : string (uuidv7)
  artefactRef        : string (the source, schema,
                       mapping, exchange, or
                       quality identifier)
  custodyEvent       : enum ("source-registered"
                       | "schema-published" |
                       "mapping-published" |
                       "exchange-published" |
                       "quality-measured" |
                       "exchange-consumed" |
                       "user-defined")
  eventTimestamp     : string (ISO 8601 date-time)
  performingParty    : string (legal entity)
  hashOfArtefacts    : string (SHA-256 hex digest)
```

## §9 Manifest

Implementations publish a signed manifest carrying
`standardSlug` (constant value "data-
integration"), `version`, `implementation`, the
operator's `accreditationStatus`, and the
`profile` declaration that selects which of the
optional records (R2RML mapping, master-data
exchange, data-quality measurement) the
implementation supports. The manifest is signed
using a key whose public part is published on
the operator's
`.well-known/wia/data-integration/` discovery
endpoint declared in PHASE-2.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## §10 Per-Programme Conformance Profile

Implementations declare a per-programme
conformance profile selecting the per-domain
binding scope. The profile binds:

- The per-source-kind subset (relational,
  noSQL, graph, streaming, REST, GraphQL).
- The per-domain master-data subset (customer,
  product, supplier, chart-of-accounts,
  location, asset).
- The per-quality-characteristic subset
  (accuracy, completeness, consistency,
  credibility, currentness, accessibility,
  compliance, confidentiality, efficiency,
  precision, traceability, understandability,
  availability, portability, recoverability).

A consumer querying the operator's discovery
endpoint receives the per-programme conformance
profile so that the consumer's per-flow
binding is aligned with the operator's
declared scope. Profile changes are published
through the operator's webhook channel and
follow the operator's schema-evolution
discipline.

## §11 Worked Example (Master-Data Exchange)

A retailer publishing the per-period customer-
master exchange to a downstream loyalty-
platform partner publishes the following
per-exchange envelope:

- `programmeRef` references the retailer's
  master-data-management programme.
- `domainRef` is `customer-master`.
- `payloadRef` is the content-addressable URI
  of the per-period customer-master JSON
  payload.
- `syntacticEncoding` is `json`.
- `semanticEncoding` is the per-element ISO
  8000-110 data-specification reference.
- `conformanceLevel` is `level-3-quality`,
  backed by the per-exchange ISO 8000-115
  quality-identifier record published in the
  operator's PHASE-1 §7 quality envelope.

The downstream loyalty-platform partner runs
the per-exchange ingestion against the
declared semantic encoding and applies the
per-exchange quality envelope as the partner's
trust filter.
