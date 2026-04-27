# WIA-protein-structure PHASE 4 — INTEGRATION Specification

**Standard:** WIA-protein-structure
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an accredited protein-structure programme
integrates with the systems that surround it: depositing
laboratories; structure-prediction service operators; archive
consortia (the wwPDB and its member archives, predicted-structure
archives); similarity-search and structure-mining services
(Foldseek, DALI, and successors); journals and pre-print servers
that ingest structural records during peer review; long-term
archives; sequence repositories (UniProt, NCBI); and citation
tools that resolve published structural results to their underlying
records.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- IETF RFC 8615 (well-known URIs)
- IETF RFC 8288 (Web Linking)
- IETF RFC 9421 (HTTP Message Signatures)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO 8601 (date and time)
- HL7 FHIR R5 (`MolecularSequence`)
- W3C Verifiable Credentials Data Model 2.0 (optional re-issuance
  of validation attestations)

---

## §1 Depositing Laboratory Integration

Depositing laboratories integrate via the structure submission
endpoint (PHASE-2 §3) plus the coordinate upload endpoint
(PHASE-2 §4). The laboratory's client certificate carries the
laboratory's identifier and is bound to the laboratory's ISO/IEC
17025 accreditation in the public register; the API verifies the
binding before accepting deposits.

## §2 Prediction-Service Operator Integration

Prediction-service operators integrate via the same submission
endpoints with `provenanceClass = "predicted"`. The operator's
client certificate is bound to the registered prediction engine
(PHASE-3 §5) so that the API can verify the submitter's authority
to publish under the engine's identity.

## §3 Evidence Package Format

```
evidence/
  manifest.json                  — package manifest (signed, see §4)
  structure.json                 — structure record
  coordinates/                   — coordinate metadata + content-
                                    addressed URI of CIF/BCIF
                                    artefact
  chains/                        — chain records and FHIR sequence
                                    cross-references
  ligands/                       — ligand records
  experimental-evidence/         — evidence record + raw-data URI
                                    (when experimental)
  prediction-confidence/         — confidence record + per-residue
                                    and pairwise confidence URIs
                                    (when predicted)
  validation/                    — validation report (PDF + JSON)
  similarities/                  — similarity-search results
  cross-refs/                    — cross-reference set
  audit/                         — API audit log excerpts
```

The package is content-addressable; the manifest carries the
SHA-256 of every file. The manifest is signed by the operating
programme's HTTP-message-signature key (RFC 9421) and counter-
signed by the depositing laboratory or the prediction-service
operator.

## §4 Manifest and Signatures

Verification tools recompute file digests, compare to the
manifest, and reject the package on mismatch with type
`urn:wia:protein-structure:evidence-mismatch`.

## §5 well-known URI Discovery

A conformant programme exposes a discovery document at
`/.well-known/wia-protein-structure` that links to the API root,
the public laboratory and prediction-engine accreditation
references, the published quality dossier, the dictionary and
CCD versions in force, the registered similarity tools, and the
catalogue of released structures.

## §6 Archive Consortium Integration

Archive consortia (the wwPDB and its member archives, predicted-
structure archives, model archives) consume releases on the
operating programme's release schedule. Integration is mutually
authenticated; the archive's client certificate is bound to the
archive's identifier. Releases are mirrored through standard
archive-side ingestion pipelines and the operating programme
records the archive accession in the structure record.

## §7 Similarity-Service Integration

Similarity-search services (Foldseek, DALI, TM-align, and
successors) ingest released structures into their indexes. The
operating programme publishes a feed of released structures with
their content-addresses; similarity services consume the feed and
publish search results back through the similarity endpoint
(PHASE-2 §8).

## §8 Sequence Repository Integration

Sequence repositories (UniProt, NCBI) cross-link to released
structures through the `uniprotAccession` field in the chain
record (PHASE-1 §4). The operating programme propagates the
linkage to the sequence repository through the repository's
standard cross-reference protocol so that sequence-side queries
surface the structural records.

## §9 Journal and Pre-Print Server Integration

Journals and pre-print servers that ingest structural records
during peer review consume deposits at draft state under
embargoed access. The journal's client certificate is bound to
the journal's identifier and authorised only for the embargoed
deposits cited by manuscripts under that journal's review.
Embargoes lift on the agreed publication date.

## §10 Long-Term Archive Integration

Beyond the wwPDB and equivalent archives, the operating programme
designates a long-term archive that holds operational records
(audit logs, validation pipelines, similarity search outputs)
that the structural archives do not host. Quarterly deposits
round-trip content-addresses; on programme wind-down, remaining
records transfer to the archive with content-addresses preserved.

## §11 Verifiable-Credential Re-Issuance (optional)

Programmes that wish to expose validation attestations to
consumers of W3C Verifiable Credentials MAY re-issue the
validation outcome as a Verifiable Credential under the Data
Model 2.0 specification. Re-issuance is optional; the canonical
record remains the JSON evidence-package manifest.

## §12 Worked Example: Citation Resolution

A reader encounters a peer-reviewed publication that cites a
predicted structure for a target protein. The reader's tool
resolves the citation by:

1. Parsing the citation to extract the structure ID and manifest
   digest.
2. Fetching the discovery document for the issuing programme.
3. Resolving the manifest URL and verifying the manifest signatures.
4. Recomputing the manifest digest and comparing it to the pinned
   digest; aborting on mismatch.
5. Retrieving the package; recomputing per-file digests; surfacing
   the resolved evidence (predicted coordinates, confidence
   metadata, validation report, cross-references to experimental
   structures of the same UniProt) to the reader.

A conformant tool completes this flow without further input.

## §13 Cross-Standard Linkage

Programmes that consume adjacent WIA standards (WIA-protein-
dynamics for kinetic characterisation of the same target,
clinical-genomics standards for `MolecularSequence` linkage) emit
cross-standard linkage records.

## §14 Reader Tooling

Programmes MAY publish supplementary reader hints (visual
structure renders, confidence heatmaps, similarity-result
visualisations) alongside the canonical evidence package. Reader
tools are non-normative.

## §15 Public Catalogue and Aggregator Feeds

Programmes that publish a public catalogue emit an Atom or JSON
Feed listing released structures with their evidence-package
manifest digests, the provenance class, the experimental method
or prediction engine, the date of last validation, and the
covering UniProt accession (when applicable). The feed is a
discovery mechanism, not a primary record.

## §16 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with prior-
minor clients. Major revisions go through a deprecation window of
at least one full mmCIF dictionary release cycle.

## §17 Migration from Pre-Standard Records

Programmes that operated before WIA-protein-structure reached
version 1.0 MAY migrate historical deposits by emitting synthetic
structure records with a `legacyImport` flag. Synthetic structures
are accepted by the public catalogue but are not eligible for
evidence-package generation without contemporaneous re-validation
under the in-force dictionary.

## §18 Drug-Discovery Pipeline Integration

Drug-discovery, enzyme-engineering, and vaccine-design pipelines
consume released structures alongside their similarity records and
PTM annotations. Integration is read-only; pipelines pin the
manifest digest of every structure they have used so that
downstream regulatory submissions can re-resolve the structures
without ambiguity.

## §19 Provisional CCD Maintainer Integration

The CCD maintainer (typically the wwPDB Chemical Component
Dictionary working group) consumes provisional-CCD requests
emitted from PTM records (PHASE-1 §10). The integration record
captures the maintainer's intake endpoint, the request format, and
the maintainer's adjudication SLA so that the operating programme
can communicate progress to the depositor.

## §20 Annotation-Source Integration

Annotation sources (SCOP, CATH, ECOD, GO, InterPro, UniProt
feature tables) integrate via per-source consume-publish adapters
that ingest source revisions, attach the revision identifier to
each annotation record, and re-publish through the annotation
endpoint (PHASE-2 §18). Annotation-source integrations are
typically read-only on the annotation source side; operating
programmes do not write back into the upstream catalogues.

## §21 UniProt Coverage Aggregator Integration

UniProt coverage aggregators consume the operating programme's
coverage records (PHASE-3 §20) and emit cross-source coverage
maps that re-credit the contributing programmes. The integration
record carries the aggregator's identifier and the access scope.

## §22 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with prior-
minor clients. Major revisions go through a deprecation window of
at least one full mmCIF dictionary release cycle.

## §23 Reader Tooling for Structural Reviewers

Manuscript reviewers and citation auditors benefit from
visualisation tools that render structures, similarity-search
results, and validation reports. Programmes MAY publish reader
tools (interactive structure viewers, confidence heatmap
renderers, similarity-result browsers) alongside the canonical
evidence package. Reader tools are non-normative; the canonical
record remains the JSON evidence-package manifest.

## §24 Conformance and Sunset

A programme conformant with PHASE-4 has integrated successfully
with at least one depositing laboratory, at least one prediction-
service operator, at least one archive consortium, at least one
similarity-search service, and at least one sequence repository,
and has published at least one externally citable evidence
package.

Sunsetting an integration is announced via the well-known
discovery document at least 90 calendar days before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-protein-structure
- **Last Updated:** 2026-04-27
