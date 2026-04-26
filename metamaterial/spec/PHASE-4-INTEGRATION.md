# WIA-metamaterial PHASE 4 — INTEGRATION Specification

**Standard:** WIA-metamaterial
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an accredited metamaterial programme integrates
with the systems that surround it: solver toolchains for simulation;
electronic design automation pipelines for hardware integration;
fabrication-vendor systems that consume design files and emit fabrication
certificates; measurement-laboratory systems that emit measurement
records; publication and citation tools that resolve externally cited
designs to their underlying evidence; and the regulators and accreditation
bodies that read the evidence package. It also defines the
evidence-package format that bundles a complete design for external
publication and audit.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details for HTTP APIs)
- IETF RFC 8615 (well-known URIs)
- IETF RFC 8288 (Web Linking)
- IETF RFC 9421 (HTTP Message Signatures)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO 8601 (date and time)
- ISO 10110 (optical drawings)
- W3C Verifiable Credentials Data Model 2.0 (the verification report MAY
  be re-issued as a Verifiable Credential for downstream consumers; this
  is optional)

---

## §1 Solver Toolchain Integration

Simulation tools that consume metamaterial designs vary in solver family,
input file format, and licensing model. Integration with a solver
toolchain is achieved through an adapter that translates between the
PHASE-1 unit-cell record and the solver's input language and that
translates the solver's output back into the PHASE-1 S-parameter,
dispersion, and field records.

The adapter is owned by the programme operating the toolchain. Adapters
SHOULD be open-source so that downstream consumers can reproduce
simulations on alternate solvers; closed-source adapters are permitted but
the resulting simulations are flagged in the public catalogue as
`closed-toolchain`, with the implication that reproducibility depends on
re-executing under the same toolchain version.

## §2 EDA Pipeline Integration

Designs intended for integration into a larger printed-circuit-board or
photonic-integrated-circuit design pass through an EDA pipeline that
consumes the unit-cell geometry and the tolerance budget. The EDA tooling
emits manufacturing artefacts (Gerber files for PCBs, GDSII files for
photonics, similar artefacts for other fabrication processes) which are
the inputs to the fabrication vendor.

The pipeline records the manufacturing-artefact content-addresses against
the design so that downstream consumers can audit the chain from unit-cell
descriptor to fabricated artefact.

## §3 Evidence Package Format

The evidence package is the externally citable artefact for a design. It is
produced by the API endpoint defined in PHASE-2 §8 and is a tarball with
the following layout:

```
evidence/
  manifest.json                     — package manifest (signed, see §4)
  design.json                       — design record (PHASE-1 §2)
  unit-cells/                       — unit-cell records and inclusion CAD
  simulations/                      — simulation records and convergence
                                     traces
  s-parameters/                     — frequency-sampled S-parameter data
  dispersion/                       — band-structure data
  retrievals/                       — effective-parameter retrieval results
  tolerances/                       — fabrication-tolerance budgets and
                                     yield estimates
  measurements/                     — measurement records, raw archives,
                                     and uncertainty budgets
  certificates/                     — fabrication certificates from vendors
  audit/                            — API audit log excerpts
```

The package is content-addressable: the manifest carries the SHA-256 of
each record file, and the manifest itself is signed.

## §4 Manifest and Signatures

The manifest is a JSON document that lists every file in the package, its
SHA-256 digest, its size in bytes, and its content-type. The manifest is
signed by the operating programme's HTTP-message-signature key (RFC 9421)
and counter-signed by the measurement laboratory whose record appears in
the package.

A consumer that receives a package verifies the signatures, recomputes
the file digests, compares them to the manifest, and rejects the package
on any mismatch. Verification tools that follow this PHASE emit
Problem-Details (RFC 9457) responses on rejection, with a `type` of
`urn:wia:metamaterial:evidence-mismatch`.

## §5 well-known URI Discovery

A conformant programme exposes a discovery document at
`/.well-known/wia-metamaterial` (RFC 8615) that links to the API root
(PHASE-2 §2), the public accreditation certificate, the published quality
dossier, the catalogue of published designs, and the round-robin
participation history.

## §6 Publication and Citation

When a metamaterial result is cited externally — in a peer-reviewed
publication, a patent, a regulatory submission, an engineering data sheet
— the citing party retrieves the evidence package once and pins the
package's manifest digest in the citation. Subsequent consumers verify
the citation by re-fetching the package and comparing content-addresses;
programmes MUST keep evidence packages addressable by their pinned
manifest digests for at least seven years from the citation event.

A citation tool that adopts this PHASE emits citations in a machine-
readable form (JSON-LD or BibTeX with WIA-specific extensions) so that
downstream consumers can resolve a paper's reference to a metamaterial
design back to the design's evidence package without manual look-up.

## §7 Fabrication-Vendor Integration

Fabrication vendors that work with a metamaterial programme integrate
through a vendor-side API that consumes the design's manufacturing
artefacts and emits a fabrication certificate (PHASE-3 §4). The
integration record carries the vendor's identifier, the agreed scope of
work, the metrology used to verify the realised geometry, and the
content-address of the standing agreement.

Programmes SHOULD publish a list of vendor integrations so that
collaborators can identify suitable vendors for a given fabrication
process and operating frequency band.

## §8 Measurement Laboratory Integration

Measurement laboratories deliver measurement records via the API endpoint
defined in PHASE-2 §7. The integration record carries the laboratory's
ISO/IEC 17025 accreditation reference and the scopes for which it is
accredited; the operating programme MUST refuse to accept a measurement
that falls outside the laboratory's accredited scopes.

## §9 Regulator and Accreditation Body Access

Regulators and accreditation bodies access the API via dedicated client
certificates issued by the certifying body. Access scopes for these
clients include the full design record set; consumer-facing scopes
(collaborator, vendor, citation tool) are narrower and are documented in
the API's OpenAPI document.

## §10 Migration from Pre-Standard Records

Programmes that operated before WIA-metamaterial reached version 1.0 MAY
migrate historical designs by emitting a synthetic design record that
carries the original design's identifying information plus a
`legacyImport` flag. Synthetic designs are accepted by the public
catalogue but are not eligible for new measurement records without a
contemporaneous re-measurement.

## §11 Public Catalogue and Aggregator Feeds

Programmes that publish a public catalogue of designs emit an Atom or
JSON Feed listing the published designs with their evidence-package
manifest digests. Feed entries carry the design's operating band, the
domain (electromagnetic, acoustic, etc.), the programme's identifier, and
the date of the most recent measurement. The feed is a discovery
mechanism and is not a primary record.

## §12 Worked Example: Citation Resolution

A reader encounters a peer-reviewed paper that cites a metamaterial design
with the WIA citation extension. The reader's tooling resolves the
citation as follows:

1. Parse the citation to extract the design ID and the manifest digest.
2. Fetch the discovery document for the issuing programme.
3. Resolve the manifest URL via the discovery document and retrieve the
   manifest.
4. Verify the manifest signatures against the programme and laboratory
   keys advertised in the discovery document.
5. Recompute the manifest digest and compare to the pinned digest in the
   citation; abort on mismatch with a Problem-Details response of type
   `urn:wia:metamaterial:evidence-mismatch`.
6. Retrieve the package, recompute the per-file digests, and surface the
   resolved evidence (S-parameters, retrieved parameters, measurement
   uncertainty) to the reader.

A conformant tool completes this flow without further input from the
reader.

## §13 Long-Term Archive Integration

Every programme that publishes externally citable designs designates a
long-term archive that will hold its evidence packages beyond programme
wind-down. The archive is operated by an institution recognised by the
certifying body (typically a national library, a national metrological
laboratory, or a community-operated archive with a stable funding
commitment). The archive integration is exercised by depositing a copy of
each newly published evidence package on a cadence agreed with the
archive (commonly quarterly), and by verifying on each deposit that the
content-addresses round-trip without corruption.

When a programme winds down, its remaining evidence packages are
transferred to the long-term archive with their content-addresses
preserved; externally pinned citations resolve to the archive without the
citing party needing to update the citation.

## §14 Verifiable-Credential Re-Issuance (optional)

A programme that wishes to expose its verification reports to consumers
of W3C Verifiable Credentials MAY re-issue the verification report as a
Verifiable Credential under the Data Model 2.0 specification. The
credential carries the same payload as the JSON verification report and
is signed under the same programme key.

Verifiable Credentials are a presentation format only; the canonical
record remains the JSON evidence-package manifest. Programmes that
re-issue MUST NOT diverge the two presentations of the same record.

## §15 Cross-Standard Linkage

A metamaterial design that supports another WIA standard (a phononic
isolator that ships under WIA-acoustic-engineering, an antenna lens that
ships under WIA-radio-front-end) carries a cross-standard linkage record
in its evidence package. The record names the consuming standard, the
version under which the linkage is claimed, and the assertion that the
metamaterial design satisfies the consuming standard's interface
requirements.

Cross-standard linkages are not transitive: a consumer of the
metamaterial design that wishes to invoke the consuming standard's
guarantees MUST verify the consuming standard's evidence directly. The
linkage record is a navigation aid, not a substitute attestation.

## §16 Conformance and Sunset

A programme conformant with PHASE-4 has integrated successfully with at
least one solver toolchain, at least one fabrication vendor, and at least
one accredited measurement laboratory, and has published at least one
externally citable design through the catalogue. The integration dossier
records the integrations and the test runs that confirmed each.

Sunsetting an integration is announced via the well-known discovery
document at least 90 calendar days before removal; in-flight designs that
depend on the sunsetting integration are migrated before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-metamaterial
- **Last Updated:** 2026-04-27
