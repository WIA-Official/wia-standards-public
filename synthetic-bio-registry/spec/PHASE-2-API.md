# WIA-BIO-021 — Phase 2: API Interface

> Synthetic-bio-registry canonical Phase 2: API surface (search + characterisation + composition).

# WIA-BIO-021: Synthetic Biology Registry Specification v1.0

> **Standard ID:** WIA-BIO-021
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Biotechnology Research Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Part Classification System](#2-part-classification-system)
3. [Metadata Standards](#3-metadata-standards)
4. [Sequence Formats](#4-sequence-formats)
5. [SBOL Compliance](#5-sbol-compliance)
6. [Characterization Data](#6-characterization-data)
7. [Version Control](#7-version-control)
8. [Access Control and Licensing](#8-access-control-and-licensing)
9. [Registry Interoperability](#9-registry-interoperability)
10. [Implementation Guidelines](#10-implementation-guidelines)

---


## 6. Characterization Data

### 6.1 Quantitative Measurements

Characterization data MUST include measurement context:

```json
{
  "characterization": {
    "measurements": [
      {
        "type": "fluorescence",
        "value": 15000,
        "unit": "RFU",
        "stdDev": 1200,
        "replicates": 3,
        "conditions": {
          "temperature": 37,
          "timePoint": 6,
          "timeUnit": "hours",
          "opticalDensity": 0.6
        },
        "method": "Plate reader - Tecan Infinite M200",
        "performedBy": "Lab Team A",
        "date": "2024-08-15"
      }
    ]
  }
}
```

### 6.2 Growth Curves

```json
{
  "growthCurve": {
    "dataPoints": [
      {"time": 0, "od600": 0.05},
      {"time": 1, "od600": 0.08},
      {"time": 2, "od600": 0.15},
      {"time": 3, "od600": 0.32},
      {"time": 4, "od600": 0.65}
    ],
    "maxGrowthRate": 0.42,
    "unit": "1/hour",
    "strain": "E. coli DH5α",
    "medium": "LB + chloramphenicol (25 μg/mL)"
  }
}
```

### 6.3 Expression Levels

For protein-coding parts:

```json
{
  "expression": {
    "transcriptLevel": {
      "value": 500,
      "unit": "copies/cell",
      "method": "qRT-PCR"
    },
    "proteinLevel": {
      "value": 10000,
      "unit": "molecules/cell",
      "method": "Western blot quantification"
    },
    "activity": {
      "specific": 1.5,
      "unit": "μmol/min/mg",
      "substrate": "IPTG"
    }
  }
}
```

### 6.4 Reusability Metrics

Track part usage and success:

```json
{
  "reusability": {
    "timesUsed": 45,
    "successRate": 0.89,
    "averageRating": 4.3,
    "reviews": [
      {
        "user": "researcher123",
        "rating": 5,
        "comment": "Works excellently in our system",
        "date": "2024-09-01"
      }
    ],
    "citations": 12
  }
}
```

---



## 9. Registry Interoperability

### 9.1 iGEM Registry Integration

**Import from iGEM**:
```
GET http://parts.igem.org/xml/part.BBa_K123456
```

**Export to iGEM**:
- Convert to iGEM XML format
- Map metadata fields
- Preserve part ID and version

### 9.2 Addgene Integration

**Plasmid Linking**:
```json
{
  "addgene": {
    "plasmidId": 12345,
    "url": "https://www.addgene.org/12345/",
    "availability": "available",
    "price": 65.00
  }
}
```

### 9.3 SynBioHub Compatibility

Full SBOL 2.3 compliance ensures compatibility with SynBioHub instances.

**Submit to SynBioHub**:
```bash
curl -X POST https://synbiohub.org/submit \
  -H "X-authorization: <token>" \
  -F "file=@part.xml"
```

### 9.4 NCBI GenBank

**Cross-reference**:
```json
{
  "genbank": {
    "accession": "MN123456",
    "gi": "1234567890",
    "url": "https://www.ncbi.nlm.nih.gov/nuccore/MN123456"
  }
}
```

---




---

## A.1 Endpoint reference

```http
GET    /registry/v1/parts                          # search parts (paginated)
GET    /registry/v1/parts/{id}                     # fetch part record
POST   /registry/v1/parts                          # contribute new part
GET    /registry/v1/parts/{id}/sequence            # raw sequence (FASTA/GB/SBOL)
GET    /registry/v1/parts/{id}/characterisation    # measurement data
POST   /registry/v1/characterisation                # contribute measurement
GET    /registry/v1/parts/{id}/composition         # composite-part subgraph
```

Every endpoint follows the discovery convention at `/.well-known/wia-synthetic-bio-registry`.

## A.2 Search and filter

Parts are searchable by chassis, type, length, GC content, function (free-text), associated keyword, design tool, contributing institution, and licence. Cursor-based pagination via `?after=cursor&limit=N` (max 100). Response payloads carry the SBOL fragment plus the metadata block for each match.

## A.3 Characterisation contribution

`POST /characterisation` accepts a measurement envelope: chassis, medium, induction profile, instrument (plate reader, flow cytometer, qPCR, RNA-seq), raw measurement data, processing pipeline (script + version), and the derived characteristic value(s). Characterisation submissions are verified against the part's declared chassis and reviewed asynchronously.

## A.4 Composition queries

The composition endpoint walks the SBOL Component graph from the requested root, returning every sub-component plus the relationships. Cycles are broken with a `truncated` flag so consumers can re-query at the deeper level if needed.

## A.5 Webhooks

Webhook events: `part.created`, `part.updated`, `part.deprecated`, `characterisation.contributed`, `characterisation.reviewed`. HMAC-SHA256 signing per the WIA family policy. Retry policy: 3 attempts at 1s/4s/16s; failures enter the dead-letter queue.

## A.6 Rate-limit envelope

1000 req/h unauthenticated (read-only paths only), 5000 req/h authenticated, 10000 req/h premium tier. Bulk imports count their parts against an additional per-tenant quota.

## A.7 Pagination, sort, filter

Cursor-based pagination via `?after=cursor&limit=N` capped at 100. Sort via `?sort=field&order=asc|desc` with multi-field comma-list support. Filtering syntax: `?field=value` for equality, `?field[op]=value` for `gte`/`lte`/`in`/`like`. Cursors are opaque — clients MUST NOT decode them. Responses carry `Link` headers per IETF RFC 8288.

## A.8 Bulk import API

`POST /registry/v1/parts/bulk` accepts up to 1000 parts per request. Each part follows the canonical envelope (Phase 1 §A.1). Idempotency is keyed by the SHA-256 of the sorted part-identifier list; retries with the same key short-circuit to the cached response. Per-entry success/failure status is returned in the response body so partial-success is the normal case.

## A.9 GraphQL surface (advisory)

A GraphQL surface mirrors the REST endpoints for tenants that prefer a single graph. Query types cover `part`, `characterisation`, `composition`, `provenance`; mutation types cover the contribution flows; subscription types cover the webhook event set. The schema is published at `https://wiastandards.com/synthetic-bio-registry/graphql/schema.graphql`.

## A.10 Authentication surfaces

OAuth 2.0 + OpenID Connect for human-driven sessions. Mutual TLS for institution-to-institution traffic. API tokens (per-tenant, scoped, rotated every 90 days) for the SDK auth surface. The auth surface advertises its capabilities at `/.well-known/openid-configuration` and `/.well-known/oauth-authorization-server`. Token introspection follows IETF RFC 7662; revocation follows RFC 7009.

## A.11 Idempotency and retry semantics

Mutating endpoints accept an `Idempotency-Key` header (UUID v4 or a 128-bit random token). The server caches the response for 24 hours keyed by `(tenant_id, endpoint, idempotency_key)`. Replay of the same key returns the cached response without re-execution; replay with a different request body returns `409 Conflict`. Read endpoints are inherently idempotent and ignore the header.

## A.11b Versioning and deprecation

API versioning is path-based (`/v1/`, `/v2/`). New optional fields are added without bumping the major version; field renames or removals require a major bump with a 12-month deprecation window per IETF RFC 8594 and 9745. The `Deprecation` and `Sunset` response headers advertise the deprecation timeline so clients can migrate ahead of removal.

## A.12 Concrete contribution example

```http
POST /registry/v1/parts HTTP/1.1
Authorization: Bearer eyJhbGc...
Content-Type: application/json
Idempotency-Key: 4f3d2c1b-9e8a-4b7c-8d6f-1a2b3c4d5e6f

{
  "partId": "WIA_BBa_K2456001",
  "name": "RBS-PtetR-mNeonGreen",
  "type": "composite",
  "sequence": "ATGGCCAAAGTC...",
  "chassis": ["E. coli MG1655", "E. coli BL21(DE3)"],
  "license": "OpenMTA",
  "designer": "doe.j@example.org"
}
```

Response carries the registry-assigned identifier, a canonical sequence hash, and the time-stamped contribution receipt.


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/synthetic-bio-registry/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-synthetic-bio-registry-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/synthetic-bio-registry-host:1.0.0` ships every synthetic-bio-registry envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/synthetic-bio-registry.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Synthetic-bio-registry deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
