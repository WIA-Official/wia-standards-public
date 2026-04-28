# WIA-museum-digital-archive PHASE 2 — API Interface Specification

**Standard:** WIA-museum-digital-archive
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the resource-oriented API surface
for museum digital-archive operations: object record
ingest and retrieval, accession and provenance
management, conservation logging, exhibition / loan
workflow, surrogate (image / 3D / multimedia) serving
with IIIF, descriptive-metadata and multilingual-
label exchange, rights / licence retrieval,
preservation-event publication (PREMIS), discovery and
search, and OAIS submission / dissemination contracts.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9110 (HTTP), RFC 9112 (HTTP/1.1), RFC 9113 (HTTP/2)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- IETF RFC 8259 (JSON), RFC 8785 (JCS), RFC 4122 (UUID), RFC 9530 (Content-Digest)
- IIIF Image API 3.0; IIIF Presentation API 3.0; IIIF Auth 2.0; IIIF Search API 1.0
- W3C Activity Streams 2.0 (for change discovery)
- W3C Web Annotation Protocol
- OAI-PMH 2.0 — Open Archives Initiative Protocol for Metadata Harvesting
- ResourceSync (NISO Z39.99-2017) — change-discovery
- ISO 14721 OAIS — Submission Information Package (SIP), Archival
  Information Package (AIP), Dissemination Information Package (DIP)
- Linked Data Platform (W3C LDP); SPARQL 1.1 Protocol

---

## §1 Endpoint root

API root is implementation-controlled. All endpoints
are TLS 1.3 (RFC 8446). Public discovery endpoints
(IIIF, OAI-PMH, ResourceSync) are TLS-served read-
only. Authenticated endpoints (write, restricted
imagery) use OAuth 2 with PKCE.

## §2 Object endpoints

```
POST   /v1/objects                      register object
GET    /v1/objects/{ref}                retrieve
PATCH  /v1/objects/{ref}                amend (curatorial)
GET    /v1/objects?type=&period=        list / filter
GET    /v1/objects/{ref}/provenance     full provenance chain
GET    /v1/objects/{ref}/conservation   conservation timeline
```

## §3 Accession / provenance endpoints

```
POST   /v1/accessions                   record accession event
POST   /v1/provenance                   record provenance event
GET    /v1/accessions/{ref}             retrieve
GET    /v1/provenance/{ref}             retrieve
```

Each event is signed by the curator / collections-
manager who recorded it; the chain is append-only.

## §4 Conservation endpoints

```
POST   /v1/conservation                 log treatment event
GET    /v1/conservation/{ref}           retrieve
PATCH  /v1/conservation/{ref}           amend narrative (audited)
GET    /v1/conservation?conservator=    list / filter
```

## §5 Exhibition / loan endpoints

```
POST   /v1/exhibitions                  open exhibition
GET    /v1/exhibitions/{ref}            retrieve
POST   /v1/loans                        open loan
PATCH  /v1/loans/{ref}/status           update (in-transit / installed /
                                                de-installed / returned)
GET    /v1/loans/{ref}                  retrieve
GET    /v1/loans?borrower=&active=      list active loans
```

Loan status events emit on the audit chain so an
inspector can reconstruct the object's whereabouts.

## §6 Surrogate endpoints (IIIF integration)

```
GET    /iiif/3/{prefix}/info.json                     IIIF Image API info
GET    /iiif/3/{prefix}/{region}/{size}/{rotation}/{quality}.{format}
GET    /iiif/presentation/3/{ref}/manifest            IIIF Presentation
GET    /iiif/auth/2/probe                             IIIF Auth probe
GET    /iiif/search/1/{ref}/search?q={query}          IIIF Search

POST   /v1/surrogates                                 register surrogate metadata
POST   /v1/surrogates/{ref}/upload                    chunked upload
GET    /v1/surrogates/{ref}                           retrieve metadata
```

Restricted imagery (e.g. for sensitive cultural-
material) requires IIIF Auth 2.0 cookie / token; the
probe service signals the access tier.

## §7 Descriptive-metadata endpoints

```
GET    /v1/objects/{ref}/lido                         LIDO XML
GET    /v1/objects/{ref}/linked-art                   Linked Art JSON-LD
GET    /v1/objects/{ref}/cidoc-crm                    CIDOC-CRM RDF
GET    /v1/objects/{ref}/edm                          Europeana EDM
GET    /v1/objects/{ref}/dc                           Dublin Core
```

The implementation produces these views from a single
canonical store; the canonical store is sponsor-
controlled.

## §8 Multilingual-label endpoints

```
POST   /v1/labels                                     publish label
GET    /v1/labels/{ref}                               retrieve
GET    /v1/objects/{ref}/labels?lang=                 list per language
PATCH  /v1/labels/{ref}                               edit (curatorial)
```

## §9 Rights / licence endpoints

```
GET    /v1/objects/{ref}/rights                       retrieve rights record
PATCH  /v1/objects/{ref}/rights                       update (committee gated)
```

Rights changes require an institutional committee
approval (recorded as a stewardship task).

## §10 Preservation endpoints (PREMIS)

```
POST   /v1/preservation-events                        log preservation event
GET    /v1/preservation-events/{ref}                  retrieve
GET    /v1/objects/{ref}/preservation-events          per-object timeline
POST   /v1/preservation-events/$fixity-check          run fixity verification
```

Fixity-check verifies content digests against the
recorded values; mismatches open stewardship tasks
and emit a PREMIS validation outcome.

## §11 Discovery and harvest endpoints

```
GET    /oai/2.0?verb=Identify                         OAI-PMH Identify
GET    /oai/2.0?verb=ListMetadataFormats              OAI-PMH formats
GET    /oai/2.0?verb=ListRecords&metadataPrefix=lido  OAI-PMH harvest
GET    /resourcesync/capability-list                  ResourceSync
GET    /resourcesync/changelist                       ResourceSync change-list
GET    /sparql                                        SPARQL endpoint (read-only)
```

OAI-PMH supports `lido`, `oai_dc`, and `edm` metadata
prefixes; ResourceSync provides change discovery for
incremental harvesters.

## §12 OAIS endpoints

```
POST   /oais/sip                       submit information package
GET    /oais/sip/{ref}                 retrieve SIP status
POST   /oais/dip                       request dissemination package
GET    /oais/dip/{ref}                 retrieve DIP
```

SIP / DIP packaging follows METS 1.12; storage in the
trusted-repository back-end follows ISO 16363
guidance.

## §13 Error model (RFC 9457)

```json
{
  "type":   "urn:wia:mu:problem:rights-restricted",
  "title":  "Imagery access restricted",
  "status": 403,
  "detail": "Object o-2026-001 is sacred-secret; imagery requires committee approval",
  "instance": "/iiif/3/o-2026-001/info.json"
}
```

Common type URIs:

| Type URI suffix              | HTTP | Meaning                                       |
|------------------------------|-----:|-----------------------------------------------|
| `rights-restricted`          | 403  | imagery / metadata access restricted           |
| `accession-incomplete`       | 422  | object lacks accession record                 |
| `provenance-gap`             | 422  | provenance chain has unexplained gap          |
| `loan-conflict`              | 409  | loan period overlaps existing loan / display  |
| `conservation-pending`       | 409  | object pending conservation cannot ship        |
| `due-diligence-incomplete`   | 422  | accession lacks due-diligence record          |
| `preservation-fixity-fail`   | 422  | fixity-check failed                            |

## §14 Bulk export

```
GET  /v1/$export?_type=Object,Provenance,Conservation,Loan
GET  /v1/$status/{exportId}
GET  /v1/$result/{exportId}/{file}
```

NDJSON output. Consumers may consume the OAI-PMH or
ResourceSync surface for incremental change capture
instead.

## §15 Audit headers

| Header                  | Meaning                                       |
|-------------------------|-----------------------------------------------|
| `X-Request-Id`          | client-set, echoed                            |
| `X-Audit-Event-Id`      | server-set, links to PHASE 3 audit chain      |
| `Content-Digest`        | RFC 9530 SHA-256 of the response body         |

## Annex A — OpenAPI reference

A canonical OpenAPI 3.1 description is published at
`api/openapi-3.1.yaml`.

## Annex B — Worked IIIF Presentation 3.0 (informative)

```json
{
  "@context": "http://iiif.io/api/presentation/3/context.json",
  "id":   "https://museum.example/iiif/presentation/3/o-2026-001/manifest",
  "type": "Manifest",
  "label": {"en":["Untitled (2026)"]},
  "items": [{
    "id":"https://museum.example/iiif/presentation/3/o-2026-001/canvas/p1",
    "type":"Canvas",
    "items":[]
  }]
}
```

## Annex C — Webhook surface

Implementations expose webhooks for `object-published`,
`loan-status-changed`, `conservation-recorded`, and
`fixity-check-failed` events. Payloads sign with RFC
7515 JWS; receivers verify against
`/.well-known/wia-mu-keys.json`.

## Annex D — Conformance disclosure

Implementations declare the IIIF profiles served, the
LIDO / Linked Art / CIDOC-CRM versions served, the
OAI-PMH metadata prefixes supported, the ResourceSync
profile, the OAIS package format, and the SPARQL
endpoint conformance.

## Annex E — Async export pattern

```
POST   /v1/$export                      → 202 Accepted, Content-Location
GET    /v1/$status/{id}                 → 202 in-progress / 200 manifest
GET    /v1/$result/{id}/{file}          → 200 NDJSON or LIDO XML
DELETE /v1/$status/{id}                 → 202 cancellation
```

The 202 response carries `Retry-After` for the
polling client and `X-Progress` for human-readable
progress.

## Annex F — Federated query (SPARQL)

```
GET  /sparql?query=...
POST /sparql
```

Read-only SPARQL endpoint exposes the institution's
linked-data graph. Federated queries from external
endpoints (Europeana SPARQL, Wikidata Query Service)
participate per the federation policy. Long-running
queries enforce a per-query time-budget; queries that
exceed the budget receive a `503` with `Retry-After`.

## Annex G — Image-derivative endpoints

```
GET  /v1/objects/{ref}/derivative/iiif?{params}    derived IIIF view
GET  /v1/objects/{ref}/derivative/oembed           oEmbed JSON
GET  /v1/objects/{ref}/derivative/oai-dc           Dublin Core
```

Derivative endpoints honour the rights record's
access tier; restricted derivatives require IIIF
Auth 2.0 cookie / token.
