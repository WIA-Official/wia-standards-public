# WIA-cultural-exchange-data PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-cultural-exchange-data
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTPS API contract
that a memory-institution operator (museum,
library, archive, heritage-site custodian,
audiovisual archive, digital-humanities centre,
or intergovernmental cultural-exchange
programme operator) exposes for the records
defined in PHASE-1. The contract carries the
catalogue publication, IIIF manifest
publication, EAD3 finding-aid publication,
linked-data graph publication, OAI-PMH harvest,
intergovernmental programme report ingestion,
and chain-of-custody anchoring endpoints. The
API is the canonical interoperability layer
between the operator's catalogue, the
international IIIF aggregator, the linked-data
discovery client, the OAI-PMH harvester, and
the OAI / Europeana federated portal.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics), RFC 9111 (HTTP
  Caching), RFC 9457 (Problem Details), RFC
  8288 (Web Linking), RFC 6901 / 6902 (JSON
  Pointer / Patch), RFC 8259 (JSON), RFC 4122
  (UUID), RFC 9421 (HTTP Message Signatures),
  RFC 8615 (well-known URIs)
- W3C Trace Context, W3C Linked Data Platform
  v1.0, W3C SKOS, W3C ODRL 2.2
- IIIF Image API 3.0, IIIF Presentation API 3.0,
  IIIF Authentication API 2.0, IIIF Search API
  2.0
- EAD3 schema and Tag Library
- ISO 15836-1:2017 / 15836-2:2019 (Dublin Core)
- ISO 21127:2014 (CIDOC CRM)
- ISO 23950:1998 (Z39.50 — cited where the
  operator runs a Z39.50 gateway alongside the
  HTTP API)
- OAI-PMH 2.0 specification
- METS / MODS / LIDO / EDM / RDA Toolkit
- ISO/IEC 27001:2022, ISO/IEC 17021-1:2015
- UNESCO 2005 Convention Operational Guidelines
  (the reporting envelope for the programme
  endpoints in §7)
- KR 박물관 및 미술관 진흥법 / KR 문화재보호법

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published
by the operator. The OpenAPI 3.1 document at
`/v1/openapi.json` is canonical for the WIA
endpoints; the IIIF Image API 3.0 and IIIF
Presentation API 3.0 endpoints are canonical
for the IIIF surface; the OAI-PMH 2.0 base URL
is canonical for the OAI surface. Schema
changes follow the non-breaking conventions in
PHASE-1 §2. Every endpoint carries a per-
request signature using HTTP Message Signatures
(RFC 9421) anchored to the operator's
institutional identifier (the ICOM / IFLA /
ICA membership reference); the signature key
set is published at
`/.well-known/wia/cultural-exchange-data/keys.
json`.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-cultural-exchange-data",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "programmes":         "/v1/programmes",
    "catalogueRecords":   "/v1/catalogue-records",
    "iiifManifests":      "/v1/iiif-manifests",
    "findingAids":        "/v1/finding-aids",
    "graphRecords":       "/v1/graph-records",
    "cepRecords":         "/v1/cep-records",
    "oaiPmh":             "/oai/",
    "iiifBase":           "/iiif/",
    "custody":            "/v1/custody-events",
    "openapi":            "/v1/openapi.json",
    "wellKnown":          "/.well-known/wia/cultural-exchange-data"
  }
}
```

## §3 Catalogue Record Endpoints

### §3.1 Publish a catalogue record

```
POST /v1/catalogue-records
Content-Type: application/json
Signature: <RFC 9421 signature from the
            operator's signing key>
```

Request body carries the §4 record from PHASE-1
(itemId, identifierBindings, cataloguingScheme,
itemType, title, creator, date,
subjectClassification, rightsExpression). The
server validates the `cataloguingScheme`
against the `itemType`: a `LIDO` record applied
to an `archival-collection` item is rejected
with `422 Unprocessable Entity` at
`/problems/cataloguing-scheme-item-type-
mismatch` (LIDO is for object records; archival
collections use EAD3 finding aids).

### §3.2 Retrieve a catalogue record

```
GET /v1/catalogue-records/{itemId}
Accept: application/json
```

Where the consumer prefers a per-scheme
serialisation, the operator supports content-
negotiation under the `Accept` header — for
example, `application/lido+xml` returns the
LIDO XML serialisation, `application/mods+xml`
returns the MODS XML serialisation, and
`application/ead+xml` returns the EAD3 finding
aid.

### §3.3 Search catalogue records

```
GET /v1/catalogue-records?type={itemType}
&schema={cataloguingScheme}
&subjectScheme={scheme}
&subjectIdentifier={id}
&creatorAuthority={authority}
&creatorIdentifier={id}
&page={cursor}&size={size}
```

The response is an RFC 8288 `Link`-paginated
collection.

## §4 IIIF Manifest Endpoints

### §4.1 Publish a IIIF manifest

```
POST /v1/iiif-manifests
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §5 record from PHASE-1.
The server validates the manifest's IIIF API
version against the operator's IIIF profile
(declared in the discovery endpoint) and
refuses a publication where the manifest's
declared version is below the operator's
minimum version.

### §4.2 Serve a IIIF Presentation manifest

```
GET /iiif/{itemId}/manifest
Accept: application/ld+json;profile="http://iiif.io/api/presentation/3/context.json"
```

The endpoint returns a IIIF Presentation API
3.0 manifest. The manifest carries the
canonical Image API 3.0 service URLs for each
image, the per-image rights statement, the
attribution metadata, and the per-canvas
range-and-structure declaration.

### §4.3 Serve a IIIF Image API request

```
GET /iiif/{itemId}/{imageId}/{region}/{size}/{rotation}/{quality}.{format}
```

The IIIF Image API 3.0 endpoint follows the
canonical URL pattern. The endpoint enforces
the operator's `imageServices.maxWidth` and
`maxHeight` parameters and rejects a request
exceeding the declared maximum with `413
Content Too Large`.

## §5 EAD3 Finding-Aid Endpoints

### §5.1 Publish an EAD3 finding aid

```
POST /v1/finding-aids
Content-Type: application/xml; charset=utf-8
Signature: <RFC 9421 signature>
```

Request body is the EAD3 XML document. The
server validates the document against the EAD3
schema and refuses a publication where the
schema validation fails.

### §5.2 Retrieve an EAD3 finding aid

```
GET /v1/finding-aids/{faId}
Accept: application/xml
```

## §6 Linked-Data Graph Endpoints

### §6.1 Publish a graph

```
POST /v1/graph-records
Content-Type: application/json
Signature: <RFC 9421 signature>
```

Request body carries the §7 record from PHASE-1
(graphSerialisation, ontologySet, triplesUri).

### §6.2 Retrieve a graph

```
GET /v1/graph-records/{graphId}
Accept: text/turtle | application/rdf+xml
       | application/ld+json
       | application/n-triples
```

### §6.3 SPARQL endpoint

```
POST /v1/sparql
Content-Type: application/sparql-query
Accept: application/sparql-results+json
```

Where the operator hosts a SPARQL endpoint, the
endpoint follows the W3C SPARQL 1.1 Protocol
specification.

## §7 Intergovernmental Programme Endpoints

### §7.1 Publish a programme report

```
POST /v1/cep-records
Content-Type: application/json
Signature: <RFC 9421 signature from the UNESCO
            Member-State focal point or the
            programme operator>
```

Request body carries the §3 record from PHASE-1
(cepId, programmeType, reportingPeriod,
subject, rightsExpression). The server
validates the `programmeType` and the
`reportingPeriod` against the UNESCO 2005
Convention quadrennial reporting cadence, the
World Heritage state-of-conservation reporting
cadence, or the Intangible Cultural Heritage
periodic-reporting cadence.

### §7.2 Retrieve a programme report

```
GET /v1/cep-records/{cepId}
Accept: application/json
```

## §8 OAI-PMH Harvest Endpoint

```
GET /oai/?verb={verb}&metadataPrefix={prefix}
   &from={iso8601}&until={iso8601}
   &set={setSpec}
   &resumptionToken={token}
```

The operator runs an OAI-PMH 2.0 harvest
endpoint. The endpoint supports the
`Identify`, `ListMetadataFormats`, `ListSets`,
`ListIdentifiers`, `ListRecords`, and
`GetRecord` verbs. The supported metadata
prefixes are `oai_dc` (Dublin Core), `mods`,
`mets`, `lido`, `edm`, and the operator's
declared user-defined prefixes.

## §9 Custody and Error Reporting

### §9.1 Anchor a custody event

```
POST /v1/custody-events
Content-Type: application/json
Signature: <RFC 9421 signature>
```

### §9.2 Error envelope

Errors are returned using RFC 9457 Problem
Details. Validation errors carry a `pointer`
field (RFC 6901). The server emits a per-
request `traceparent` header (W3C Trace
Context).

## §10 Concurrency and Cache Discipline

Every retrieval endpoint emits an `ETag` header
(RFC 9110 §8.8.3) and supports `If-Match` /
`If-None-Match` conditional requests. IIIF
manifests are cacheable per the IIIF Caching
Discipline (the manifest is served with a long
`Cache-Control` max-age and is invalidated by
republishing under a new identifier).

## §11 Bulk Export for Aggregators

```
GET /v1/catalogue-records:bulk
Accept: application/x-ndjson
```

A federated aggregator (Europeana, the
Digital Public Library of America, a national
aggregator) ingests the operator's catalogue as
a newline-delimited JSON stream. A consumer
resuming the stream provides an `If-Resume-
After` header carrying the last-received
modification timestamp.

## §12 Z39.50 Gateway (legacy, optional)

Where the operator runs a Z39.50 gateway under
ISO 23950:1998, the gateway is published at the
operator's `Z39.50` host and port declared in
the discovery endpoint. The gateway translates
between the Z39.50 query envelope and the
operator's HTTP search endpoint.

## §13 Schema-Validation and Conformance

The OpenAPI 3.1 document at `/v1/openapi.json`
carries JSON Schema 2020-12 schemas for every
WIA endpoint. The IIIF endpoints follow the
IIIF specifications' schemas. The OAI-PMH
endpoint follows the OAI-PMH 2.0 schema.

```
GET /v1/conformance/test-vectors
Accept: application/json
```

The operator publishes the conformance test
vectors used to qualify the API implementation.
Each vector references the relevant Dublin
Core / CIDOC CRM / IIIF / OAI-PMH clause.

## §14 Webhook Endpoint for Catalogue Update Notifications

```
POST /v1/programmes/{programmeId}/webhooks
Content-Type: application/json
Signature: <RFC 9421 signature>
```

A federated aggregator registers a webhook to
receive a push notification when a catalogue
record is published, updated, or withdrawn. The
webhook envelope carries the operator's
signing-key reference, the event type, and the
resource identifier.
