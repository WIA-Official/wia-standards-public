# WIA-protein-structure PHASE 2 — API-INTERFACE Specification

**Standard:** WIA-protein-structure
**Phase:** 2 — API-INTERFACE
**Version:** 1.0
**Status:** Stable

This document defines the HTTP API contract that an accredited
protein-structure programme exposes for the records defined in
PHASE-1. Consumers include depositing laboratories, structure-
prediction service operators, archive consortia, downstream
consumers (drug-discovery, enzyme-engineering, vaccine-design),
similarity-search services, citation tools, and journals that
ingest structural records during peer review.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9110 (HTTP Semantics)
- IETF RFC 9111 (HTTP Caching)
- IETF RFC 9457 (Problem Details)
- IETF RFC 6901 (JSON Pointer)
- IETF RFC 6902 (JSON Patch)
- IETF RFC 8288 (Web Linking)
- IETF RFC 8259 (JSON)
- IETF RFC 9421 (HTTP Message Signatures)
- ISO 8601 (date and time)
- ISO/IEC 27001:2022 (information security management)
- HL7 FHIR R5 (`MolecularSequence`)
- W3C Trace Context

---

## §1 Scope and Versioning

JSON-over-HTTPS served from a domain published by the operating
programme. Versioning uses `/v1/` path segments. The OpenAPI 3.1
document at `/v1/openapi.json` is canonical. Coordinate payloads
are content-addressed CIF or BCIF artefacts; the API exposes
references to those artefacts rather than inlining them in JSON.

## §2 Root Discovery

```
GET /v1/
```

```json
{
  "standard": "WIA-protein-structure",
  "phase": "API-INTERFACE",
  "version": "1.0",
  "links": {
    "structures":         "/v1/structures",
    "coordinates":        "/v1/coordinates",
    "chains":             "/v1/chains",
    "ligands":            "/v1/ligands",
    "experimentalEvidence": "/v1/experimental-evidence",
    "predictionConfidence": "/v1/prediction-confidence",
    "validations":        "/v1/validations",
    "similarities":       "/v1/similarities",
    "crossRefs":          "/v1/cross-refs",
    "evidence":           "/v1/evidence",
    "openapi":            "/v1/openapi.json"
  }
}
```

## §3 Structures

```
POST   /v1/structures                  — register a structure
GET    /v1/structures/{sid}            — retrieve structure record
PATCH  /v1/structures/{sid}            — update mutable fields
GET    /v1/structures?provenance={p}   — list by provenance class
GET    /v1/structures?uniprot={acc}    — list structures covering
                                         a UniProt accession
```

A new structure submission MUST include either an
`experimentalMethod` or a `predictionEngine` consistent with its
provenance class; submissions whose provenance metadata is missing
return `422` with type
`urn:wia:protein-structure:provenance-incomplete`.

## §4 Coordinates

```
POST   /v1/structures/{sid}/coordinates   — register a coordinate
                                              artefact reference
GET    /v1/coordinates/{cid}              — retrieve metadata
GET    /v1/coordinates/{cid}/artefact     — fetch CIF or BCIF
                                              archive
```

The artefact endpoint is content-addressed and immutable; the
metadata endpoint may be patched only to add validation references
under §7.

## §5 Chains, Ligands, and Sequence Cross-Reference

```
POST   /v1/structures/{sid}/chains       — register a chain
GET    /v1/chains/{cid}                  — retrieve chain record
POST   /v1/structures/{sid}/ligands      — register a ligand
GET    /v1/ligands/{lid}                 — retrieve ligand record
GET    /v1/chains/{cid}/sequence-fhir    — fetch FHIR
                                              MolecularSequence
                                              representation
```

The FHIR endpoint serves the chain's sequence as a FHIR R5
`MolecularSequence` resource for clinical-genomics consumers; raw
CIF sequences flow through the coordinate artefact.

## §6 Experimental Evidence

```
POST   /v1/structures/{sid}/experimental-evidence  — register
                                                       evidence
GET    /v1/experimental-evidence/{eid}             — retrieve
                                                       evidence
GET    /v1/experimental-evidence/{eid}/raw         — fetch raw
                                                       data archive
```

Raw data archives are immutable resources delivered with content-
addressed URLs and `Cache-Control: max-age=31536000, immutable`.

## §7 Prediction Confidence and Validation

```
POST   /v1/structures/{sid}/prediction-confidence  — register
                                                       confidence
GET    /v1/prediction-confidence/{cid}             — retrieve
GET    /v1/prediction-confidence/{cid}/per-residue — fetch per-
                                                       residue
                                                       confidence
GET    /v1/prediction-confidence/{cid}/pairwise    — fetch pairwise
                                                       confidence
POST   /v1/structures/{sid}/validations            — register
                                                       validation
                                                       report
GET    /v1/validations/{vid}                       — retrieve
                                                       report
```

## §8 Similarity and Cross-References

```
POST   /v1/structures/{sid}/similarities    — register a similarity
                                               search result
GET    /v1/similarities/{sid}               — retrieve search result
POST   /v1/cross-refs                       — register a cross-
                                               reference
GET    /v1/structures/{sid}/cross-refs      — list cross-references
                                               for a structure
```

Similarity searches MUST cite the search tool, the version, and
the query parameters; submissions whose tool and version do not
resolve in the API's tool register return `422` with type
`urn:wia:protein-structure:similarity-tool-unregistered`.

## §9 Evidence Package

```
POST   /v1/structures/{sid}/evidence    — request package generation
GET    /v1/evidence/{packageId}         — retrieve a package
GET    /v1/evidence/{packageId}/manifest — manifest only
```

The evidence package contains the structure record, the coordinate
artefact (for in-place packages) or its content-address (for
reference-only packages), the chains and ligands, the experimental
evidence or the prediction confidence, the validation report, the
cross-references, and the signed manifest.

## §10 Errors

All error responses are `application/problem+json` per RFC 9457.
Defined types include:

- `urn:wia:protein-structure:provenance-incomplete`
- `urn:wia:protein-structure:cif-parse-failure`
- `urn:wia:protein-structure:similarity-tool-unregistered`
- `urn:wia:protein-structure:ccd-version-mismatch`
- `urn:wia:protein-structure:dictionary-version-mismatch`
- `urn:wia:protein-structure:evidence-mismatch`

## §11 Authentication

The API uses mutually-authenticated TLS for laboratory, prediction-
service, and archive-consortium connections. Public read-only
endpoints (released structures, the OpenAPI document) are reachable
without a client certificate.

## §12 Caching

Stable resources (released structures, signed validations, evidence
packages) are cacheable with `Cache-Control: max-age=31536000,
immutable`. Mutable resources (draft structures awaiting validation)
are cacheable for 60 seconds.

## §13 Streaming Subscriptions

Consumers subscribe via Server-Sent Events at
`/v1/structures/{sid}/events`. Topics include validation
completions, prediction-confidence updates, similarity-search
results, and cross-reference additions.

## §14 Worked Example: Predicted-of-Experimental Citation

1. Laboratory deposits an experimental structure with mmCIF
   coordinates and validation report.
2. Prediction-service operator predicts the same UniProt sequence
   and registers the predicted structure.
3. Programme registers a `predicted-of-experimental` cross-reference
   linking the two.
4. Citation tool receives a paper that cites the predicted
   structure; the tool walks the cross-reference back to the
   experimental structure for downstream readers.

## §15 PTM and Quaternary Assembly Endpoints

```
POST   /v1/structures/{sid}/ptms          — register a PTM record
GET    /v1/ptms/{pid}                     — retrieve PTM record
POST   /v1/structures/{sid}/quaternary-assemblies
                                          — register an assembly
GET    /v1/quaternary-assemblies/{qid}    — retrieve assembly
```

PTM records that cite a `provisional-ccd-` identifier flag the
structure for downstream consumers as containing pending CCD
content; the flag is cleared automatically when the CCD entry is
finalised.

## §16 Bulk and Pagination

Bulk endpoints accept arrays of structures for high-volume
prediction-service deposits:

```
POST   /v1/bulk/structures        — submit a batched prediction set
GET    /v1/bulk/{operationId}     — operation status
```

Cursor-based pagination uses the `cursor` query parameter and
`Link` headers (RFC 8288); cursors are opaque and persist for at
least 24 hours.

## §17 Audit and Observability

Every endpoint emits structured logs with `structureId`, `traceId`,
the issuing client certificate's subject, and the laboratory or
prediction-service clock skew vs the reference NTP source.

## §18 Annotation Endpoints

```
POST   /v1/structures/{sid}/annotations    — register an annotation
GET    /v1/annotations/{aid}               — retrieve annotation
GET    /v1/structures/{sid}/annotations?source={s}
                                            — list annotations by
                                              source
```

Annotation submissions cite the annotation source (SCOP, CATH,
ECOD, GO, InterPro, UniProt-feature) and the source version so
that downstream consumers can pin annotations against a specific
revision.

## §19 Privacy-Preserving Aggregation

Aggregate consumers (research-grant agencies, public-health
analysts, structure-prediction benchmark coordinators) fetch
population-level statistics through aggregation endpoints that
emit counts, means, and dispersions:

```
GET    /v1/aggregate/structures-by-method?period=...
GET    /v1/aggregate/predicted-confidence-distribution?engine=...
GET    /v1/aggregate/uniprot-coverage?period=...
```

Out-of-policy queries return `403 Forbidden` with type
`urn:wia:protein-structure:cohort-too-small`.

## §20 Provenance Endpoint

```
GET    /v1/provenance/{recordId}    — retrieve provenance entry
                                       for any PHASE-1 record
```

Provenance entries trace a structure to its parents (laboratory
deposit, prediction engine, MSA inputs, template inputs, CCD
version, mmCIF dictionary version) so that a citing reader can
walk the chain end-to-end.

## §21 Conformance

A conformant server passes the test vectors published under
`tests/phase-vectors/phase-2-api-interface/`, emits an OpenAPI 3.1
document, signs evidence packages per RFC 9421, and rejects
coordinate payloads that fail CIF parsing.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 2 — API-INTERFACE
- **Status:** Stable
- **Standard:** WIA-protein-structure
- **Last Updated:** 2026-04-27
