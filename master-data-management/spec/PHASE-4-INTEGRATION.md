# WIA-master-data-management PHASE 4 — Integration Specification

**Standard:** WIA-master-data-management
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE specifies how WIA-master-data-management
integrates with adjacent operational, analytical, and
regulatory systems: enterprise applications (ERP, CRM,
HCM, billing, KYC), data warehouses and lakehouses,
business-intelligence and analytics consumers, supply-
chain trace systems, financial-reporting frameworks
(IFRS / GAAP / regulator-specific reporting), reference-
data authorities (GS1, GLEIF, LEI ROC, ISO TC, GBIF /
ITIS / Catalogue of Life), privacy-rights pipelines,
and downstream WIA companion standards. It also
specifies the publication / consumption contracts that
operational systems honour.

References (CITATION-POLICY ALLOW only):
- ISO 8000-110 / 8000-115 / 8000-120 — master-data exchange
- ISO/IEC 11179-3 / 11179-6 — metadata-registry exchange and registration
- ISO/IEC 19763 — metamodel framework
- DAMA-DMBOK — Data Management Body of Knowledge (reference)
- HL7 FHIR R5 — Patient, Practitioner, Organization, Location, Substance, Bulk Data
- HL7 SMART App Launch 2.0
- GLEIF / LEI ROC — Global LEI System and CDF format
- GS1 GTIN, GLN, SSCC; GS1 EPCIS / CBV
- ISO 17442 — Legal Entity Identifier
- ISO 6166 (ISIN), ISO 4217 (currency), ISO 3166 (country / region)
- W3C SKOS, W3C SHACL, W3C R2RML — relational-to-RDF mapping
- US SEC 17a-4 (financial record retention), GDPR (EU 2016/679),
  K-PIPA (KR), CCPA / CPRA (US-CA), LGPD (BR), PIPL (CN)
- IETF RFC 9110 (HTTP), RFC 7515 (JWS), RFC 8259 (JSON), RFC 8785 (JCS)

---

## §1 ERP / CRM / HCM / billing integration

Master records publish to operational applications via
one of three patterns:

| Pattern         | Description                                       |
|-----------------|---------------------------------------------------|
| Registry-style  | downstream apps query MDM by reference; MDM is    |
|                 | read-aside; downstream retains operational state  |
| Transactional   | MDM is system of record; downstream uses cached  |
|                 | replicas refreshed on change-events               |
| Coexistence     | MDM holds golden values; downstream retains       |
|                 | local extensions; bidirectional reconciliation    |

Per-application binding is recorded on the source-
system reference record so an audit can verify which
domain attributes a system contributes versus consumes.

## §2 Data warehouse / lakehouse integration

| Target                | Binding                                          |
|-----------------------|--------------------------------------------------|
| Data warehouse        | dimension-table publication (slowly-changing     |
|                       | dimension type 2 with effective-dating)          |
| Data lake / lakehouse | NDJSON / Parquet snapshot per-domain;            |
|                       | golden-record changes feed CDC topic             |
| Iceberg / Delta tables| atomic snapshot + change-data-feed               |
| Streaming             | Kafka / Pulsar topic of change-events             |

Downstream consumers receive the golden record and the
change-event stream so analytics can replay any prior
state.

## §3 Reference-data authority integration

| Authority         | Reference                                          |
|-------------------|----------------------------------------------------|
| GLEIF             | LEI ROC; LEI CDF format; daily LEI file             |
| GS1 GO            | GS1 GTIN / GLN / SSCC; GS1 GDSN data pool           |
| D&B               | DUNS directory                                     |
| ANNA              | ISIN issuance                                      |
| OpenFIGI          | financial-instrument identifier                    |
| ITIS / GBIF       | taxonomic reference (where bio-domain bound)        |
| ISO TC            | ISO standards portal                                |
| W3C / IETF / IEEE | technical-standard catalogue                       |

Reference-data sets ingest with provenance (PHASE 1
§7); cross-walks expose SKOS match relations (exact /
close / broad / narrow / related).

## §4 Financial-reporting frameworks

| Framework        | Binding                                            |
|------------------|----------------------------------------------------|
| IFRS             | account / asset taxonomy alignment                  |
| US GAAP          | XBRL US-GAAP taxonomy alignment                    |
| ESEF (EU)        | XBRL ESEF taxonomy for issuer reporting             |
| Solvency II      | EIOPA reporting                                    |
| Basel III / IV   | counterparty / exposure reporting                  |
| FATCA / CRS      | tax-jurisdictional reporting                        |
| KR DART (KR)     | regulator-issuer disclosure                         |

The party / asset / account hierarchies bind to the
applicable framework's taxonomy version; reporting
exports cite the taxonomy version in force at the
reporting date.

## §5 Supply-chain trace integration

| Standard / system           | Binding                                |
|-----------------------------|----------------------------------------|
| GS1 EPCIS / CBV             | event-based supply-chain trace         |
| WIA-food-traceability       | food / agriculture lots                |
| WIA-pharmaceutical-trace    | DSCSA / EU FMD trace                   |
| WIA-vehicle-trace           | VIN / battery-passport                 |

Product / location / party master records resolve the
trace events to identified entities; the trace partner
sees the same golden record.

## §6 Privacy-rights pipeline

Master-record systems are the natural locus of subject-
rights operations:

| Right                 | Action                                       |
|-----------------------|----------------------------------------------|
| Access (GDPR Art. 15) | export of all party / personal records       |
| Rectification (Art. 16)| steward task with audit trail               |
| Erasure (Art. 17)     | tombstone — preserves audit chain hash, payload removed |
| Portability (Art. 20) | machine-readable export of party data        |
| Restriction (Art. 18) | flag prevents downstream processing           |
| Objection (Art. 21)   | restriction + steward review                 |

Subject-rights events emit audit events and propagate
to operational systems via the change-event stream.

## §7 Cross-domain WIA bindings

| Companion standard          | Binding purpose                                |
|-----------------------------|------------------------------------------------|
| WIA-data-governance         | stewardship-policy linkage                     |
| WIA-data-lineage            | cross-system lineage edges                     |
| WIA-data-quality            | continuous monitoring of quality scores        |
| WIA-data-catalog            | catalog of master domains                      |
| WIA-financial-data-exchange | party / asset / account binding                |
| WIA-healthcare-integration  | Patient / Practitioner / Organization binding   |
| WIA-food-traceability       | product / location binding                     |
| WIA-cross-border-payment    | counterparty / party binding                   |

Each binding identifies the consumed PHASE.

## §8 Long-term archival

| Domain / regulator                | Retention                          |
|-----------------------------------|------------------------------------|
| Party (KYC / AML)                 | per regulator (typ. ≥ 5 y after end of relationship) |
| Financial / asset (US SEC)        | 17a-4 — 6 years (3 readily accessible) |
| Tax records                       | per jurisdiction (typ. 7-10 years) |
| Personal data                     | per lawful basis + DPIA            |
| Reference-data versions           | indefinite (active + retired)       |

The audit-chain hash is preserved even on personal-
data erasure; the payload is replaced with a tombstone
that records the erasure rationale and the steward.

## §9 Conformance test suite

The reference test suite covers, at minimum:

- ISO 8000-110 round-trip on a party / product record
- LEI resolution on a legalEntity party record
- DUNS resolution on a legalEntity party record
- GTIN cross-walk to UNSPSC and GPC
- SHACL validation on a malformed record
- match-cluster steward action with survivorship override
- hierarchy effective-date query at a prior date
- bitemporal correction with `validFrom` distinct from
  `recordedAt`
- privacy erasure with audit-chain preservation
- FHIR R5 export of Organization / Patient (where
  clinical context applies)
- change-event stream ordering and replay

## §10 Internationalisation

Localised attributes (display names, address lines,
reference labels) carry BCP 47 language tags; the
golden record exposes the canonical fallback locale
declared in the survivorship-policy reference record.

## §11 Security and privacy posture

- Transport: TLS 1.3 with mutual TLS for sponsor /
  authority / regulator transmissions
- Authentication: SMART on FHIR (clinical contexts);
  client_credentials with key attestation (sponsor)
- At-rest: AES-256-GCM with sponsor-controlled KMS;
  per-domain key wrapping per ISO/IEC 27002 §8.24
- Audit: tamper-evident chain (PHASE 3 §8) exportable
  per ISO/IEC 27037 forensic-evidence guidance
- Privacy: special-category data follows ISO/IEC 27701
  controls; subject-rights pipeline (§6) applies
- Steward / regulator access: role-segregated;
  operations on personal data require justification
  recorded on the audit chain

## §12 Operational metrics

Sponsors report (informationally) on the WIA registry:

- domain-record counts and growth
- golden-record completeness / accuracy / uniqueness
  rolling scores
- match-cluster auto-merge vs. steward-review
  proportion
- stewardship SLA compliance per priority
- privacy-rights queue (open / resolved per period)

## §13 Recovery and continuity

- API outage — local change-event queue at source
  systems; replay on reconnect
- reference-authority outage — local mirror keyed by
  release identifier; integrity via SHA-256 manifest
- KMS outage — sealed back-up keys per sponsor's BCP
- audit-chain corruption — chain repair via prior
  snapshot + replay of the change-event stream

## Annex A — Worked end-to-end example (informative)

A multinational manufacturer consolidates three
regional ERP installations onto a transactional MDM
backbone for the party / product / location domains.
A daily LEI file from GLEIF refreshes legal-entity
identifiers; SHACL shapes flag legalName mismatches
for steward review. A merge of two duplicate "Acme
Logistics" parties consolidates 12 GLN-bearing
locations and 3,400 GTIN-bearing products. Downstream
ERP, CRM, billing, and warehouse systems consume the
golden record via the change-event stream; the data-
warehouse populates SCD type 2 dimension tables. A
GDPR Art. 17 erasure on a former employee's HCM party
record propagates to all consumers; the audit-chain
hash is preserved.

## Annex B — Conformance disclosure

Implementations declare the publication patterns
(registry / transactional / coexistence) supported,
the reference-authority bindings enabled, the FHIR R5
IG profiles exposed (where applicable), and the
financial-reporting taxonomies aligned. Disclosure is
machine-readable at `/.well-known/wia-mdm-conformance.
json`.

## Annex C — Versioning

Adding a new reference-authority binding is minor;
changing the canonical FHIR mapping is major.

## Annex D — Stewardship-policy registry

The stewardship policy in force at any point in time
is recorded as a reference record. Policy fields
include:

- match thresholds per domain
- survivorship strategy per attribute
- SLA tiers per priority
- escalation paths
- attestation requirements (steward credential)

Policy changes emit audit events and re-trigger
golden-record builds where the change affects survival.
