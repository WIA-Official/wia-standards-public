# WIA-master-data-management PHASE 1 — Data Format Specification

**Standard:** WIA-master-data-management
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for an
enterprise-grade master-data management (MDM) system
covering party, product, location, asset, financial-
account, and reference-data records, including their
golden-record constructions, survivorship rules, change-
history, stewardship workflow, hierarchies, and
cross-domain bindings. The format aligns with the ISO
8000 data-quality series, the DAMA-DMBOK reference,
the ISO/IEC 11179 metadata-registry framework, and the
GS1 identifier suites where party / product / location
binding to commerce is in scope.

References (CITATION-POLICY ALLOW only):
- ISO 8000-1 — Data quality, fundamental concepts
- ISO 8000-110 — Master data: exchange of characteristic data
- ISO 8000-115 — Master data: exchange of quality identifiers
- ISO 8000-120 — Master data: exchange of provenance
- ISO 8000-150 — Data quality management framework
- ISO/IEC 11179-1 to 11179-7 — Metadata registries (MDR)
- ISO/IEC 19763 — Information technology — Metamodel framework for interoperability
- ISO 6166 — ISIN (securities)
- ISO 4217 — currency code
- ISO 3166 — country / region code
- ISO 17442 — Legal Entity Identifier (LEI)
- LEI ROC / GLEIF — Global LEI System
- GS1 GTIN, GS1 GLN, GS1 SSCC, GS1 GS1-128, GS1 EPCIS / CBV
- DUNS — D&B legal-entity directory
- DAMA-DMBOK — Data Management Body of Knowledge (reference framework)
- HL7 FHIR R5 — Patient, Practitioner, Organization, Location, Substance
- W3C SKOS — Simple Knowledge Organization System (controlled vocabularies)
- W3C SHACL — Shapes Constraint Language
- IETF RFC 8259 (JSON), RFC 8785 (JCS), RFC 4122 (UUID), RFC 7515 (JWS), RFC 9530 (Content-Digest)

---

## §1 Scope

This PHASE applies to enterprise systems that maintain
authoritative records for parties (persons, legal
entities, households), products and services, locations
(physical, logical, jurisdictional), assets (financial,
fixed, intellectual property), accounts (financial,
relationship), and reference-data sets used across
operational systems.

In scope: party, product, location, asset, account,
reference-data, golden-record, source-record,
match-cluster, hierarchy, change-history, stewardship-
task, lineage, and quality-rule records, plus the
cross-references that bind master data to operational
domains.

Out of scope: analytical aggregates that are not the
authoritative source of master attributes (handled by
data-warehouse standards) and operational-only data
that does not survive a system retirement (handled by
operational-domain standards).

## §2 Party record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `partyRef`           | UUID (RFC 4122) — opaque internal identifier    |
| `partyKind`          | `person`, `legal-entity`, `household`,          |
|                      | `government-body`, `legal-arrangement`          |
| `legalName`          | for legal entities; with language tag (BCP 47)  |
| `displayName`        | rendering string per locale                     |
| `lei`                | ISO 17442 Legal Entity Identifier (where issued)|
| `dunsNumber`         | D&B DUNS (where bound)                          |
| `taxId`              | per-jurisdiction tax identifier                  |
| `incorporationCountry`| ISO 3166-1 alpha-3                              |
| `birthYear`          | for persons; year only                          |
| `sex`                | ISO/IEC 5218 (where lawful and necessary)       |
| `parentRef`          | parent legal entity (where relationship exists) |
| `consentRef[]`       | active consents (privacy / contractual)         |
| `goldenRecordRef`    | the surviving golden record this party links to |

## §3 Product record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `productRef`         | UUID                                            |
| `gtin`               | GS1 GTIN-14 / -13 / -12 / -8                    |
| `gpc`                | GS1 GPC brick / class / family / segment         |
| `unspsc`             | UN Standard Products and Services Code           |
| `manufacturerRef`    | party reference                                  |
| `brandRef`           | party / brand reference                          |
| `productName`        | localised label (BCP 47)                        |
| `unitOfMeasure`      | UN/CEFACT REC 20 unit                           |
| `dimensions`         | length / width / height / weight (with unit)    |
| `materialComposition`| controlled list per applicable regulator        |
| `packagingHierarchy` | each / inner-pack / case / pallet (GS1 GS1-128) |

## §4 Location record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `locationRef`        | UUID                                            |
| `gln`                | GS1 GLN (where assigned)                        |
| `kind`               | `physical-address`, `logical-site`, `route`,    |
|                      | `jurisdictional-area`                           |
| `addressLines`       | array of localised lines                        |
| `country`            | ISO 3166-1 alpha-3                              |
| `region`             | ISO 3166-2                                      |
| `postalCode`         | per-country                                     |
| `geo`                | OGC Simple Features point or polygon             |
| `parentLocationRef`  | hierarchy parent (e.g. floor → building → site) |

## §5 Asset record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `assetRef`           | UUID                                            |
| `kind`               | `financial-instrument`, `fixed-asset`,          |
|                      | `intangible`, `inventory`, `real-property`      |
| `isin`               | ISO 6166 (financial instruments)                |
| `cusip`              | CUSIP (where applicable)                        |
| `figi`               | OpenFIGI (where applicable)                     |
| `categoryCode`       | per applicable taxonomy                         |
| `acquisitionDate`    | ISO 8601                                        |
| `valuationCurrency`  | ISO 4217                                        |
| `custodianRef`       | party reference                                  |
| `lifecycleStatus`    | `acquired`, `in-service`, `retired`, `disposed` |

## §6 Account record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `accountRef`         | UUID                                            |
| `kind`               | `customer`, `supplier`, `employee`, `partner`,  |
|                      | `bank`, `general-ledger`                        |
| `partyRef`           | counterparty                                    |
| `accountNumber`      | per-domain account identifier                   |
| `currency`           | ISO 4217 (where applicable)                     |
| `legalAgreementRef`  | governing agreement                             |
| `lifecycleStatus`    | `prospect`, `active`, `dormant`, `closed`       |

## §7 Reference-data record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `referenceSetRef`    | UUID                                            |
| `code`               | controlled-vocabulary code                       |
| `term`               | localised label                                 |
| `taxonomyUri`        | URI of the issuing authority (ISO, GS1, IETF,   |
|                      | sponsor, regulator)                             |
| `version`            | release identifier                              |
| `status`             | `active`, `deprecated`, `retired`               |
| `mappings`           | cross-walk to other taxonomies (SKOS exact /    |
|                      | close / broad / narrow / related match)         |

## §8 Golden record and source-record records

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `goldenRecordRef`    | UUID                                            |
| `domain`             | party / product / location / asset / account    |
| `survivedAttributes` | per-attribute trust score + source provenance   |
| `clusterRef`         | match-cluster reference                         |
| `qualityScore`       | per ISO 8000-150 quality dimensions             |
| `effectiveFrom`      | ISO 8601                                        |
| `effectiveTo`        | ISO 8601 (open if current)                      |

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `sourceRecordRef`    | UUID                                            |
| `sourceSystemRef`    | upstream operational system identifier          |
| `sourceKey`          | upstream natural key                            |
| `payload`            | as-received payload                             |
| `ingestTime`         | ISO 8601                                        |
| `lineageRef[]`       | upstream lineage edges                          |
| `goldenRecordRef`    | the golden record this source contributes to   |

## §9 Match-cluster record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `clusterRef`         | UUID                                            |
| `matchAlgorithm`     | algorithm + version (deterministic, probabilistic|
|                      | per Fellegi–Sunter / ML)                       |
| `pairs`              | per-pair match probability + features           |
| `decisionThreshold`  | high (auto-merge) / mid (steward-review) /      |
|                      | low (no-merge)                                  |
| `stewardActions[]`   | merge / split / re-cluster / hold                |

## §10 Hierarchy record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `hierarchyRef`       | UUID                                            |
| `kind`               | `legal-entity-tree`, `cost-centre`,              |
|                      | `sales-territory`, `product-category`,           |
|                      | `general-ledger`, `regulatory-reporting`         |
| `version`            | release identifier (effective-dated)             |
| `nodes`              | per-node parent / child / depth / weight         |
| `effectiveFrom`      | ISO 8601                                        |
| `effectiveTo`        | ISO 8601 (open if current)                      |

## §11 Change-history record

Every mutation to a master record emits a change-
history entry capturing the actor, the timestamp, the
delta, the reason code, and the JWS signature over the
canonical payload (RFC 7515 / RFC 8785). The chain is
per-record and per-domain so reconstruction is
deterministic.

## §12 Stewardship-task record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `taskRef`            | UUID                                            |
| `kind`               | `dedup-review`, `quality-fix`, `survivorship-   |
|                      | exception`, `hierarchy-review`, `provenance-    |
|                      | dispute`                                        |
| `priority`           | low / medium / high / critical                   |
| `assigneeRef`        | data-steward identity                           |
| `slaDeadline`        | ISO 8601                                        |
| `status`             | open / in-progress / resolved / abandoned        |
| `resolution`         | per-resolution rationale                        |

## §13 Quality-rule record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `ruleRef`            | UUID                                            |
| `ruleExpression`     | SHACL shape, JSON-Schema, regex, SQL, or DSL    |
| `dimension`          | ISO 8000-150 dimension (completeness, accuracy, |
|                      | timeliness, consistency, uniqueness, validity)  |
| `severity`           | informational / warning / error / critical      |
| `coverage`           | per-domain selector                              |

## §14 Cross-domain references (informative)

- WIA-data-governance — for stewardship-policy linkage
- WIA-data-lineage — for cross-system lineage edges
- WIA-data-quality — for monitoring of quality scores
- WIA-data-catalog — for cataloging of master domains

## Annex A — Worked golden party (informative)

```json
{
  "goldenRecordRef": "gr-party-2026-04-12-001",
  "domain": "party",
  "survivedAttributes": {
    "legalName": {"value":"Acme Inc.","source":"sap-erp","trust":0.94},
    "lei":       {"value":"5493001RKR55V4X61F71","source":"gleif","trust":1.0},
    "incorporationCountry":{"value":"USA","source":"sec-edgar","trust":0.98}
  },
  "qualityScore": {"completeness":0.96,"accuracy":0.97,"uniqueness":1.0}
}
```

## Annex B — Conformance disclosure

Implementations declare the JSON-Schema URIs they
serve, the SHACL shape catalogue version, the
canonicalisation form (RFC 8785), and the reference-
data taxonomy versions indexed.

## Annex C — Versioning

Field additions are minor; semantic redefinition or
removal is major.
