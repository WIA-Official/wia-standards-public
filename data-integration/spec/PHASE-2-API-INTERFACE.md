# WIA-data-integration PHASE 2 — API Interface Specification

**Standard:** WIA-data-integration
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract that
a data-integration operator (ETL/ELT engineering
team, MDM programme operator, data-mesh domain
team, data-fabric platform vendor, open-data
publisher, federated-analytics operator,
healthcare-data-exchange operator) exposes for
the records defined in PHASE-1. The contract
carries the data-source registration, schema
publication, R2RML mapping upload, master-data
exchange publication, data-quality measurement
recording, and chain-of-custody anchoring
endpoints.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110, RFC 9111, RFC 9457, RFC 8288,
  RFC 6901 / 6902, RFC 8259, RFC 4122, RFC
  9421, RFC 8615
- W3C Trace Context, W3C R2RML, W3C SPARQL
  1.1, W3C RDF 1.1, W3C OWL 2, W3C JSON-LD
  1.1, W3C SHACL, W3C SKOS, W3C DCAT v3, W3C
  PROV-O
- ISO/IEC 11179 series, ISO 8000 series,
  ISO/IEC 25012:2008, ISO/IEC 25024:2015,
  ISO/IEC 19763, ISO/IEC 27001:2022, ISO/IEC
  17021-1:2015, ISO/IEC 17065:2012
- HL7 FHIR R5 RESTful API
- DAMA DMBoK 2nd edition

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published
by the operator. The OpenAPI 3.1 document at
`/v1/openapi.json` is canonical. Schema changes
follow the non-breaking conventions in PHASE-1
§2. Every endpoint carries a per-request
signature using HTTP Message Signatures (RFC
9421) anchored to the operator's accreditation
reference (the ISO/IEC 27001 certification, the
per-domain ISO 8000-110 conformance reference,
the per-flow data-quality attestation
reference); the signature key set is published
at `/.well-known/wia/data-integration/keys.json`.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-data-integration",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":          "/v1/programmes",
    "sources":             "/v1/sources",
    "schemas":             "/v1/schemas",
    "mappings":            "/v1/mappings",
    "exchanges":           "/v1/exchanges",
    "qualityRecords":      "/v1/quality-records",
    "custody":             "/v1/custody-events",
    "openapi":             "/v1/openapi.json",
    "wellKnown":           "/.well-known/wia/data-integration"
  }
}
```

## §3 Data-Source Endpoints

### §3.1 Register a data source

```
POST /v1/sources
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §3 record from
PHASE-1 (sourceName, sourceKind,
technicalContact, refreshCadence). The server
validates the `refreshCadence` against the
declared `sourceKind`: a `streaming-platform`
source MUST declare `real-time` or
`near-real-time` cadence; a declaration of
`monthly` for a streaming platform is rejected
with `422 Unprocessable Entity` at
`/problems/source-cadence-kind-mismatch`.

### §3.2 Retrieve a data source

```
GET /v1/sources/{sourceId}
Accept: application/json
```

### §3.3 Search data sources

```
GET /v1/sources?kind={kind}&cadence={cadence}
&page={cursor}&size={size}
```

## §4 Schema Endpoints

### §4.1 Publish a schema descriptor

```
POST /v1/schemas
Content-Type: application/json
Signature: <RFC 9421 signature from the
            registration authority>
```

Request body carries the §4 record from
PHASE-1. The server validates that each
`dataElement` carries a `data-element name`
conforming to the ISO/IEC 11179-5 naming
conventions and a non-empty `definition`.

### §4.2 Retrieve a schema descriptor

```
GET /v1/schemas/{schemaDescriptorId}
Accept: application/json
```

### §4.3 ISO/IEC 11179 conformance test

```
POST /v1/schemas/{schemaDescriptorId}/iso11179-test
Content-Type: application/json
Signature: <RFC 9421 signature>
```

The endpoint runs the per-schema ISO/IEC
11179-3 metadata-registry conformance test
against the declared schema descriptor and
returns the per-element conformance verdict.

## §5 R2RML Mapping Endpoints

### §5.1 Publish an R2RML mapping

```
POST /v1/mappings
Content-Type: multipart/form-data; boundary=...
Signature: <RFC 9421 signature>
```

The multipart body carries one JSON part with
the §5 record from PHASE-1 (sourceRef,
ontologyRef, triplesMaps) and one or more
file parts holding the R2RML mapping document
in Turtle, RDF/XML, or JSON-LD encoding. The
server stores the mapping document under a
content-addressable URI.

### §5.2 Retrieve an R2RML mapping

```
GET /v1/mappings/{mappingRecordId}
Accept: application/json
```

### §5.3 Run a SPARQL federated query

```
POST /v1/sparql/federated
Content-Type: application/sparql-query
Signature: <RFC 9421 signature>
```

A federated-analytics consumer runs a per-
query W3C SPARQL 1.1 Federated Query against
the operator's published R2RML-virtual-graph
endpoint. The operator's API enforces the
per-query timeout and the per-query result-
size limit declared in the operator's resource-
governance policy.

## §6 Master-Data Exchange Endpoints

### §6.1 Publish a master-data exchange

```
POST /v1/exchanges
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §6 record from
PHASE-1 (domainRef, payloadRef,
syntacticEncoding, semanticEncoding,
conformanceLevel). The server validates that
the declared `conformanceLevel` is consistent
with the per-payload evidence — a level-3-
quality declaration MUST be backed by a
per-exchange ISO 8000-115 quality-identifier
record.

### §6.2 Retrieve a master-data exchange

```
GET /v1/exchanges/{exchangeRecordId}
Accept: application/json
```

### §6.3 Search master-data exchanges

```
GET /v1/exchanges?domain={domain}
&conformanceLevel={level}
&page={cursor}&size={size}
```

## §7 Data-Quality Endpoints

### §7.1 Publish a data-quality measurement

```
POST /v1/quality-records
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §7 record from
PHASE-1. The server validates the
`qualityCharacteristic` against the ISO/IEC
25012:2008 enumeration and the
`measurementValue` against the per-quality-
characteristic ISO/IEC 25024 measurement
function.

### §7.2 Retrieve a data-quality measurement

```
GET /v1/quality-records/{qualityRecordId}
Accept: application/json
```

## §8 Custody and Error Reporting

### §8.1 Anchor a custody event

```
POST /v1/custody-events
Content-Type: application/json
Signature: <RFC 9421 signature>
```

### §8.2 Error envelope

Errors are returned using RFC 9457 Problem
Details. Validation errors carry a `pointer`
(RFC 6901). The server emits a per-request
`traceparent` header (W3C Trace Context).

## §9 Concurrency and Cache

Every retrieval endpoint emits an `ETag`
header (RFC 9110 §8.8.3). Conditional
requests are honoured.

## §10 Bulk Export for Data Catalogues

```
GET /v1/sources:bulk
Accept: application/x-ndjson
```

A data-catalogue consumer ingests the
operator's source register as a newline-
delimited JSON stream. The endpoint streams
the source records in registration-date
order; a consumer resuming the stream
provides an `If-Resume-After` header carrying
the last-received registration timestamp.

## §11 W3C DCAT v3 Catalogue Endpoint

```
GET /v1/catalogue
Accept: application/ld+json
```

The operator publishes the per-programme
data-catalogue per W3C DCAT v3 in JSON-LD
encoding. The catalogue carries the per-
dataset descriptor, the per-distribution
descriptor, the per-dataset license envelope,
and the per-dataset access-rights envelope.

## §12 Webhook Endpoint for Schema-Change Notifications

```
POST /v1/programmes/{programmeId}/webhooks
Content-Type: application/json
Signature: <RFC 9421 signature>
```

A downstream consumer (a data-mesh-domain
team consuming the per-domain master-data
exchange, an analytics-platform consuming
the per-source schema, an open-data consumer
consuming the per-catalogue dataset)
registers a webhook to receive a push
notification when a schema change is
published, when a master-data exchange is
re-published, or when a per-flow data-quality
measurement crosses the operator-declared
threshold.

## §13 Schema-Validation and Conformance

The OpenAPI 3.1 document at
`/v1/openapi.json` carries JSON Schema 2020-12
schemas for every request and response
envelope.

```
GET /v1/conformance/test-vectors
Accept: application/json
```

The operator publishes the conformance test
vectors used to qualify the API implementation
against this specification.

## §14 Per-Flow Lineage Endpoint

```
GET /v1/lineage?endpoint={endpoint}
Accept: application/ld+json
```

The operator publishes the per-flow lineage
graph per W3C PROV-O for the requested
endpoint. The graph records the per-flow
upstream-source dependency, the per-flow
transformation envelope, and the per-flow
downstream-consumer dependency.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## §15 Authorities and Roles

| Role                      | Capabilities |
|---------------------------|------|
| `data-source-owner`       | Register source, publish schema |
| `mapping-author`          | Publish R2RML mapping, run SPARQL queries |
| `master-data-steward`     | Publish master-data exchange, attest quality |
| `data-quality-engineer`   | Publish quality measurement, declare baseline |
| `data-governance-committee` | Approve programme, audit per-flow envelope |
| `external-consumer`       | Read scoped operator state under data-sharing agreement |
| `regulator`               | Read full operator state under sectoral mandate |

The operator's API enforces the per-role
authorisation policy.

## §16 Multi-Language Catalogue Surface

```
GET /v1/catalogue?lang={lang}
Accept: application/ld+json
```

The operator's data-catalogue is published
in each operator-declared language with the
per-dataset title, the per-dataset
description, and the per-license envelope
translated. The dataset identifiers, the
per-distribution encoding labels, and the
license URIs are language-neutral and carried
in the canonical form across all retrievals.

## §17 Per-Flow Replay Endpoint

```
POST /v1/sources/{sourceId}/replay
Content-Type: application/json
Signature: <RFC 9421 signature>
```

A per-source replay envelope re-runs the per-
source ingestion from a declared checkpoint
(an ISO 8601 timestamp, a per-source offset,
a per-source watermark) so that a downstream
consumer can recover from a per-flow
disruption.

## §18 HL7 FHIR R5 Healthcare-Data-Exchange Endpoint

```
GET /v1/fhir/{ResourceType}/{id}
Accept: application/fhir+json
```

A healthcare-data-integration operator
publishes the per-resource FHIR R5 envelope
per the HL7 FHIR R5 RESTful API. The per-
resource envelope is encoded as
`application/fhir+json` per the HL7 FHIR R5
specification.

## §19 Per-Programme Conformance Test Endpoint

```
GET /v1/conformance/per-programme-test
Accept: application/json
```

The operator publishes the per-programme
conformance test envelope so that a downstream
consumer can confirm the operator's per-flow
conformance with the operator's declared
profile (per PHASE-1 §10). The envelope
carries the per-domain test outcome, the per-
quality-characteristic test outcome, and the
per-mapping conformance outcome.

## §20 Per-Source Idempotency-Key Discipline

```
POST /v1/sources
Idempotency-Key: <UUID>
```

Every per-source registration carries an
`Idempotency-Key` header so that a per-source
re-registration arising from a network retry
does not produce a duplicate per-source
record. The operator's API records the per-
key envelope for 24 hours; a re-registration
with the same key returns the original per-
source record.
