# WIA-microbiome PHASE 2 — API Interface Specification

**Standard:** WIA-microbiome
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the resource-oriented API surface
for microbiome operations: specimen registration,
library and run lifecycle, read-set ingest, taxonomic
and functional analysis dispatch, analytical-result
retrieval, biobank cataloguing, and INSDC submission
binding. The API is designed for both clinical
laboratory operations (FHIR R5 alignment) and large-
scale research data flow (sequencing-centre LIMS,
public archive submission).

References (CITATION-POLICY ALLOW only):
- IETF RFC 9110 (HTTP semantics), RFC 9112 (HTTP/1.1), RFC 9113 (HTTP/2)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- IETF RFC 8259 (JSON), RFC 8785 (JCS), RFC 4122 (UUID), RFC 9530 (Content-Digest)
- HL7 FHIR R5 — Specimen, MolecularSequence, Observation, Bulk Data
- HL7 v2.x — OUL^R22 laboratory result message
- INSDC submission — NCBI SRA submission portal, ENA Webin REST, DDBJ DRA
- ISO/IEC 17025:2017 — testing laboratory accreditation
- BIOM v2.1 (HDF5) — microbial feature-table format
- Common Workflow Language (CWL) v1.2, Workflow Description Language (WDL) 1.1, Nextflow nf-core
- ISO/IEC 18004 (QR code) — chain-of-custody label

---

## §1 Endpoint root

API root is implementation-controlled. All endpoints
are TLS 1.3 (RFC 8446). For clinical contexts the API
participates in the SMART on FHIR launch flow; for
research and biobank contexts the API uses
`client_credentials` with key attestation.

## §2 Specimen endpoints

```
POST   /v1/specimens                    register specimen
GET    /v1/specimens/{ref}              retrieve
PATCH  /v1/specimens/{ref}              amend non-identity fields
GET    /v1/specimens?subjectRef=&matrix=  list / filter
POST   /v1/specimens/{ref}/custody      append chain-of-custody event
DELETE /v1/specimens/{ref}              logical exclude (status=excluded)
```

Chain-of-custody events are append-only; events are
not editable once posted. Each event carries the actor,
timestamp, location, container temperature, and a
witness signature (RFC 7515 JWS).

## §3 Library and run endpoints

```
POST   /v1/libraries                    open library record
PATCH  /v1/libraries/{ref}              amend during prep
POST   /v1/libraries/{ref}/qc           record QC measurement
POST   /v1/runs                         open sequencing run
POST   /v1/runs/{ref}/start             record start event
POST   /v1/runs/{ref}/finish            record finish event
GET    /v1/runs/{ref}                   retrieve
GET    /v1/runs?instrument=&date=       list / filter
```

Open / Start / Finish are idempotent under an
`Idempotency-Key` header; resubmission returns the
existing event without side effects.

## §4 Read-set endpoints

```
POST   /v1/read-sets                    ingest read-set metadata
POST   /v1/read-sets/{ref}/upload       multipart upload for raw files
GET    /v1/read-sets/{ref}              retrieve
GET    /v1/read-sets/{ref}/download     server-presigned URL with expiry
DELETE /v1/read-sets/{ref}              soft delete (raw retained per policy)
```

Raw read files are stored at the implementation's
object store under content-addressed paths (file SHA-256
prefix). The API never exposes the raw bytes to a
client without an authenticated authorisation grant.

## §5 Analysis dispatch

```
POST   /v1/analyses                     dispatch a workflow run
GET    /v1/analyses/{ref}               retrieve status
GET    /v1/analyses/{ref}/log           streamed log (Server-Sent Events)
GET    /v1/analyses/{ref}/result        result manifest URI
DELETE /v1/analyses/{ref}               cancellation
```

The dispatch payload includes:

```json
{
  "workflow":     "nf-core/ampliseq:2.7",
  "containerDigest": "sha256:7f9f...",
  "inputManifest": "wia-mb://manifest/cohort-IBD-2026-Q1",
  "params":        {"primer_set": "16S-V3V4", "trunc_q": 25}
}
```

The container digest pins the exact image used for the
analysis so re-execution is byte-equivalent.

## §6 Result endpoints

```
GET    /v1/analyses/{ref}/result/feature-table     BIOM v2.1
GET    /v1/analyses/{ref}/result/taxonomy           TSV / Parquet
GET    /v1/analyses/{ref}/result/functional         TSV / Parquet
GET    /v1/analyses/{ref}/result/diversity          JSON
POST   /v1/analyses/{ref}/result/biomarker          recorded reviewer call
```

Results are content-addressed: the response
`Content-Digest: sha-256=:...` header per RFC 9530
allows clients to verify integrity end-to-end.

## §7 Biobank endpoints

```
POST   /v1/biobank/aliquots             register aliquot
GET    /v1/biobank/aliquots/{ref}       retrieve
POST   /v1/biobank/aliquots/{ref}/move  freezer / shelf / box / position
GET    /v1/biobank/aliquots?status=     list / filter
```

Aliquot moves are the only writable surface after
registration; aliquot identity and parent-specimen
reference are immutable.

## §8 INSDC submission endpoints

```
POST   /v1/insdc/submission             open submission session
POST   /v1/insdc/submission/{ref}/biosample
POST   /v1/insdc/submission/{ref}/experiment
POST   /v1/insdc/submission/{ref}/run
POST   /v1/insdc/submission/{ref}/release   (move from suppressed to public)
```

Submission events are bound to the upstream specimen,
library, run, and read-set records so the public-archive
state is reverse-resolvable.

## §9 Error model (RFC 9457)

```json
{
  "type":   "urn:wia:mb:problem:custody-broken",
  "title":  "Chain-of-custody event missing",
  "status": 409,
  "detail": "Specimen S-101 has no custody event between collection and extraction",
  "instance": "/v1/libraries"
}
```

Common type URIs:

| Type URI suffix              | HTTP | Meaning                                       |
|------------------------------|-----:|-----------------------------------------------|
| `custody-broken`             | 409  | missing chain-of-custody event                |
| `negative-control-missing`   | 422  | library has no extraction-blank linkage       |
| `positive-control-missing`   | 422  | run has no mock-community linkage              |
| `qc-fail`                    | 422  | QC metric below threshold                     |
| `unsupported-pipeline-version`| 400 | requested workflow version not available      |
| `host-removal-required`      | 422  | human specimen submitted without host filter   |

## §10 Pagination and filtering

List endpoints return cursor-paginated responses:

```json
{
  "items": [...],
  "nextCursor": "eyJsYXN0IjoiU3BlY2ltZW4tMTIzIn0"
}
```

Filtering supports MIxS-aligned terms (`matrix`,
`anatomicalSite`, `studyRef`, `runRef`, `instrument`,
`pipelineVersion`).

## §11 Bulk export (FHIR alignment)

```
GET  /v1/$export?_type=Specimen,Observation,MolecularSequence
GET  /v1/$status/{exportId}
GET  /v1/$result/{exportId}/{file}
```

Export output is NDJSON of FHIR R5 resources, one
resource per line.

## §12 Rate limits

API gateways enforce `429 Too Many Requests` with a
`Retry-After` header. INSDC submission endpoints carry a
separate quota mirroring the upstream archive's limits.

## §13 Versioning

Resource paths are version-prefixed (`/v1/...`). Major
version bumps change the prefix; minor and patch
revisions are documented in the OpenAPI changelog.

## §14 Authentication and scopes (informative)

For clinical contexts:

```
patient/Specimen.read
patient/Observation.read
user/MolecularSequence.read
system/Bulk.export
```

For research / biobank contexts: client_credentials
with `wia-mb:specimen.write`, `wia-mb:run.write`,
`wia-mb:analysis.dispatch`, `wia-mb:insdc.submit`.

## Annex A — OpenAPI reference

A canonical OpenAPI 3.1 description of the endpoints in
this PHASE is published at `api/openapi-3.1.yaml`. The
OpenAPI document is the authoritative wire-format
reference; this PHASE is the authoritative semantic
reference.

## Annex B — Audit headers

| Header                  | Meaning                                       |
|-------------------------|-----------------------------------------------|
| `X-Request-Id`          | client-set, echoed by server                  |
| `X-Audit-Event-Id`      | server-set, links to PHASE 3 §6 event         |
| `Content-Digest`        | RFC 9530 SHA-256 of the response body         |
| `X-Trace-Id`            | W3C Trace Context (`traceparent`)             |

## Annex C — Conformance disclosure

Implementations declare the OpenAPI revision they serve,
the FHIR R5 IG (US Core, KR Core, IPS) profiles they
implement, the SMART scopes required, and the workflow
runtimes they accept (CWL v1.2, WDL 1.1, Nextflow with
nf-core conventions).

## Annex D — Worked specimen registration (informative)

```http
POST /v1/specimens HTTP/1.1
Authorization: Bearer ...
WIA-MB-Schema-Version: 1.0
Idempotency-Key: 7c0d9b0f-...

{
  "subjectRef": "MB-007",
  "matrix": "stool",
  "preservationMethod": "OMNIgene-GUT",
  "collectionDate": "2026-04-12",
  "storageTemp": -80,
  "consentRef": "consent:MB-007:protocol-v1.2",
  "studyRef": "study:IBD-WIA-2026"
}
```

Response 201 returns the `specimenRef` UUID, the
audit-event identifier, and the FHIR R5 mapping.

## Annex E — Async export pattern

```
POST   /v1/$export                      → 202 with Content-Location
GET    /v1/$status/{id}                 → 202 in-progress / 200 manifest
GET    /v1/$result/{id}/{file}          → 200 NDJSON
DELETE /v1/$status/{id}                 → 202 cancellation
```

The 202 response carries `Retry-After` for the polling
client and `X-Progress` for human-readable progress.

## Annex F — Workflow dispatch parameters

| Parameter             | Type   | Default                          |
|-----------------------|--------|----------------------------------|
| `primer_set`          | enum   | per study                        |
| `trunc_q`             | int    | 25                               |
| `min_reads_per_sample`| int    | 5000                             |
| `host_reference`      | URI    | per study (GRCh38 / mm10 / null) |
| `random_seed`         | int    | study-fixed                      |
| `decontam_method`     | enum   | `prevalence` / `frequency`        |
| `taxonomy_classifier` | enum   | `naive-bayes` / `vsearch` /       |
|                       |        | `kraken2-bracken` / `metaphlan`  |

Workflows record the resolved parameter set on the
analytical-result record so re-execution under newer
defaults still yields a byte-equivalent re-run from the
recorded parameters.

## Annex G — Bulk specimen ingest

```
POST   /v1/specimens/$batch
Content-Type: application/x-ndjson

{"matrix":"stool","subjectRef":"MB-001",...}
{"matrix":"stool","subjectRef":"MB-002",...}
...
```

Batch ingest returns a per-line outcome (created /
duplicate / rejected) so partial-failure handling is
explicit. Idempotency is per-line via a content hash.

## Annex H — Reproducibility query

```
GET /v1/reproducibility/{analysisRef}
```

Returns a payload describing what is needed to re-
execute the analysis byte-for-byte:

```json
{
  "workflow":         "nf-core/ampliseq:2.7",
  "containerDigest":  "sha256:7f9f...",
  "inputManifest":    "wia-mb://manifest/cohort-IBD-2026-Q1",
  "params":           {"primer_set": "16S-V3V4", "trunc_q": 25, "random_seed": 42},
  "referenceDb":      [{"name": "SILVA", "version": "138.2"}],
  "tier":             "reproducible-strong"
}
```

The reproducibility payload is intentionally reusable:
a downstream tool can invoke the same workflow runtime
with the listed digests and parameters to obtain the
identical artefact.

## Annex I — Cross-archive resolution

```
GET /v1/specimens/{ref}/insdc-mapping
GET /v1/runs/{ref}/insdc-mapping
GET /v1/analyses/{ref}/derived-mapping
```

Resolution endpoints return the assigned INSDC
accession (BioSample / Experiment / Run / BioProject)
plus any MGnify or community-derived analysis URI bound
to the underlying record.

## Annex J — Notification and webhook surface

Implementations may expose webhook endpoints for run-
finish, qc-fail, embargo-lift, and analysis-success
events. The webhook payload signs with RFC 7515 JWS
and the receiver verifies against a published JWK set
at `/.well-known/wia-mb-keys.json`. Webhook delivery
follows at-least-once semantics; the receiver is
expected to be idempotent on `eventId`.
