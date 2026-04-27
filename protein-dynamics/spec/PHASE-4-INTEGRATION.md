# WIA-protein-dynamics PHASE 4 — INTEGRATION Specification

**Standard:** WIA-protein-dynamics
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an accredited protein-dynamics programme
integrates with the systems that surround it: simulation engines and
high-performance-computing environments that produce trajectories;
spectroscopy laboratories that produce observables; biobank governance
services that mediate biospecimen consent; structural-biology
repositories that hold deposited starting structures; long-term archives
that hold externally cited evidence packages; and the regulators and
accreditation bodies that read the evidence package. It also defines
the evidence-package format that bundles a complete study for external
publication and audit.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- IETF RFC 8615 (well-known URIs)
- IETF RFC 8288 (Web Linking)
- IETF RFC 9421 (HTTP Message Signatures)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27701:2019 (privacy information management)
- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO 8601 (date and time)
- HL7 FHIR R5 (biospecimen and consent resources)
- W3C Verifiable Credentials Data Model 2.0 (optional re-issuance of
  evidence summaries)

---

## §1 Simulation Engine Integration

Computational dynamics campaigns are executed on simulation engines that
vary in input language, integrator implementation, and parallelisation
strategy. Integration with an engine is achieved through an adapter that
translates between the PHASE-1 simulation record and the engine's input
and that translates the engine's outputs back into the PHASE-1
trajectory record.

The adapter is owned by the programme operating the engine and is
exercised in the engine-vendor's reference test matrix at least once per
calendar year. Adapter source SHOULD be open-source so that downstream
consumers can reproduce simulations on alternate engines; closed-source
adapters are permitted but the resulting simulations are flagged in the
public catalogue as `closed-toolchain`.

## §2 HPC Environment Integration

Long simulation campaigns run on shared HPC environments. The integration
record carries the HPC site identifier, the queue and resource budget,
the container or environment-module manifest used to execute the
simulation, and the locked dependency manifest that PHASE-1 §7 expects.
The locked manifest is content-addressed; programmes MUST refuse to
publish a campaign whose locked manifest cannot be re-pinned at
publication time.

## §3 Evidence Package Format

The evidence package is the externally citable artefact for a study. It
is produced by the API endpoint defined in PHASE-2 and is a tarball with
the following layout:

```
evidence/
  manifest.json                     — package manifest (signed, see §4)
  study.json                        — study record (PHASE-1 §2)
  preparations/                     — preparation records and FHIR
                                     biospecimen links
  structures/                       — starting-structure records and
                                     mmCIF artefacts
  simulations/                      — simulation records, force-field
                                     manifests, and integrator
                                     parameters
  trajectories/                     — trajectory metadata and
                                     coordinate-archive references
  observables/                      — spectroscopic observable records,
                                     raw archives, and reduced data
  analyses/                         — analysis pipelines, locked
                                     environments, and outputs
  validations/                      — cross-validation records
  consent/                          — consent-chain hashes (FHIR R5)
  audit/                            — API audit log excerpts
```

The package is content-addressable; the manifest carries the SHA-256 of
each record file and the manifest itself is signed.

## §4 Manifest and Signatures

The manifest is a JSON document that lists every file in the package,
its SHA-256 digest, its size in bytes, and its content-type. The
manifest is signed by the operating programme's HTTP-message-signature
key (RFC 9421) and counter-signed by the experimental laboratory whose
record appears in the package, when one is present.

Verification tools that follow this PHASE emit Problem-Details (RFC 9457)
responses on rejection, with a `type` of
`urn:wia:protein-dynamics:evidence-mismatch`.

## §5 well-known URI Discovery

A conformant programme exposes a discovery document at
`/.well-known/wia-protein-dynamics` (RFC 8615) that links to the API
root, the public accreditation certificate, the published quality
dossier, the catalogue of published studies, and the round-robin
participation history.

## §6 Biobank Governance Integration

For studies that consume clinical biospecimens, integration with the
source biobank's governance service mediates consent verification
(PHASE-3 §2). The integration record carries the biobank's identifier,
the FHIR R5 endpoints that the biobank exposes, and the credentials
under which the protein-dynamics programme accesses those endpoints.
Credentials are mutual-TLS-based; the credential rotation cadence is
recorded in the programme's quality dossier.

A study whose biospecimen consent has been revoked at the biobank is
flagged in the public catalogue with the withdrawal notice produced by
the API per PHASE-3 §2; consumers retrieving the evidence package
receive a manifest that includes the withdrawal notice as a top-level
artefact, even when the underlying simulation and analysis records are
left in place for transparency.

## §7 Structural Repository Integration

When a study's starting structure is deposited in a community structural
repository (PDB, electron microscopy databank, model archive), the
integration record carries the repository accession and the version of
the deposited entry. The evidence package's structure record (§3)
references the deposited entry by accession and by content-address, so
that consumers can verify that the structure used in the simulation
corresponds to the entry that was deposited at the repository.

## §8 Publication and Citation

When a study is cited externally, the citing party retrieves the
evidence package once and pins the package's manifest digest in the
citation. Subsequent consumers verify the citation by re-fetching the
package and comparing content-addresses; programmes MUST keep evidence
packages addressable by their pinned manifest digests for at least
seven years from the citation event.

A citation tool that adopts this PHASE emits citations in a machine-
readable form (JSON-LD or BibTeX with WIA-specific extensions) so that
downstream consumers can resolve a paper's reference to a study back to
the study's evidence package without manual look-up.

## §9 Long-Term Archive Integration

Every programme that publishes externally cited studies designates a
long-term archive (typically a national library or a discipline-specific
archive with a stable funding commitment). Evidence packages are
deposited at the archive on a quarterly cadence and the deposit is
verified by re-pinning the content-addresses on each deposit. On
programme wind-down, remaining packages transfer to the archive with
content-addresses preserved.

## §10 Regulator and Accreditation Body Access

Regulators and accreditation bodies access the API via dedicated client
certificates issued by the certifying body. Access scopes for these
clients include the full study record set, including audit logs and
consent-chain hashes; consumer-facing scopes are narrower and are
documented in the API's OpenAPI document.

## §11 Verifiable-Credential Re-Issuance (optional)

A programme that wishes to expose its evidence summaries to consumers
of W3C Verifiable Credentials MAY re-issue a study summary as a
Verifiable Credential under the Data Model 2.0 specification. The
credential carries the manifest digest and the analysis-output
identifiers and is signed under the same programme key. Re-issuance is
optional; the canonical record remains the JSON evidence package.

## §12 Worked Example: Citation Resolution

A reader encounters a peer-reviewed paper that cites a protein-dynamics
study with the WIA citation extension. The reader's tooling resolves the
citation as follows:

1. Parse the citation to extract the study ID and the manifest digest.
2. Fetch the discovery document for the issuing programme.
3. Resolve the manifest URL via the discovery document and retrieve the
   manifest.
4. Verify the manifest signatures against the programme and laboratory
   keys advertised in the discovery document.
5. Recompute the manifest digest and compare it to the pinned digest in
   the citation; abort on mismatch.
6. Retrieve the package; recompute the per-file digests; surface the
   resolved evidence (trajectory, observable, analysis outputs, cross-
   validation report) to the reader.

A conformant tool completes this flow without further input from the
reader.

## §13 Cross-Standard Linkage

A protein-dynamics study that supports another WIA standard (a kinetic
characterisation that enters a drug-discovery dossier under a
WIA-pharma-pipeline standard, a structural ensemble that supports a
diagnostic-imaging standard) carries a cross-standard linkage record in
its evidence package. The record names the consuming standard, the
version under which the linkage is claimed, and the assertion that the
study satisfies the consuming standard's interface requirements.

Cross-standard linkages are not transitive: a consumer of the
protein-dynamics study that wishes to invoke the consuming standard's
guarantees MUST verify the consuming standard's evidence directly. The
linkage record is a navigation aid, not a substitute attestation.

## §14 Public Catalogue and Aggregator Feeds

Programmes that elect to publish a public catalogue of studies emit an
Atom or JSON Feed listing the published studies with their
evidence-package manifest digests. Feed entries carry the study's
domain (computational, NMR, single-molecule, etc.), the operating
laboratory's identifier, and the date of the most recent analysis
output. The feed is a discovery mechanism and does not carry consent
identifiers or unredacted clinical detail; it is intended for
collaborator search, not for primary record-keeping.

## §15 Reader Tooling Compatibility

The evidence-package format is consumed by reader tools that range from
human-driven viewers to automated citation crawlers. A programme MAY
publish supplementary reader hints (a sample-rendering script for
free-energy surfaces, a viewer-friendly slice of a long trajectory,
preview thumbnails of FRET histograms) alongside the canonical evidence
package; supplementary hints are non-normative and are clearly labelled
as such in the manifest so that readers do not confuse them with
authoritative records.

## §16 Conformance and Sunset

A programme conformant with PHASE-4 has integrated successfully with at
least one simulation engine, at least one experimental laboratory or
externally accredited spectroscopy facility, at least one biobank
governance service (when consuming clinical biospecimens), and at least
one long-term archive, and has published at least one externally cited
study through the catalogue.

Sunsetting an integration is announced via the well-known discovery
document at least 90 calendar days before removal; in-flight studies
that depend on the sunsetting integration are migrated before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-protein-dynamics
- **Last Updated:** 2026-04-27
