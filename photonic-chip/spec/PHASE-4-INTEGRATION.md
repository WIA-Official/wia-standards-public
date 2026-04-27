# WIA-photonic-chip PHASE 4 — INTEGRATION Specification

**Standard:** WIA-photonic-chip
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an accredited photonic-chip programme
integrates with the systems that surround it: design-tool ecosystems
that produce schematics and layouts; foundry ordering systems that
schedule tape-outs; wafer-test and packaging facilities; system
integrators that deploy photonic modules; recall coordination
networks; long-term archives that hold externally cited evidence
packages; and the regulators and accreditation bodies that read the
evidence package. It also defines the evidence-package format that
bundles a complete photonic chip's record for external audit.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- IETF RFC 8615 (well-known URIs)
- IETF RFC 8288 (Web Linking)
- IETF RFC 9421 (HTTP Message Signatures)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO 10110 (optics — preparation of drawings)
- ISO 8601 (date and time)
- IEC 60825-1 (laser safety)
- IEC 62496-2 (optical circuit boards)

---

## §1 Design-Tool Integration

Photonic schematic and layout tools integrate with the WIA API via
plugins or adapters that submit schematic and layout artefacts on
behalf of the design house. Adapters are owned by the operating
programme; tool-vendor responsibility is limited to keeping the
adapter's public API stable.

A design-tool adapter MUST honour the PDK pinning rules of PHASE-3 §2
and MUST refuse to submit a layout whose DRC or LVS reports are not
present and clean.

## §2 Foundry Ordering Integration

Foundry ordering systems schedule tape-outs against reticle slots and
manage the fabrication queue. Integration is bidirectional: the
operating programme publishes accepted tape-outs to the foundry's
queue and the foundry publishes fabrication-run completion events
back to the programme.

The API's evidence package (§3) consumes both ends of the
integration: it includes the design house's tape-out record and the
foundry's fabrication-run record, signed by both parties.

## §3 Evidence Package Format

```
evidence/
  manifest.json                 — package manifest (signed, see §4)
  design.json                   — design record
  pdk.json                      — PDK reference
  schematic/                    — schematic file and components list
  layout/                       — GDSII or OASIS file and DRC/LVS
                                  reports
  fabrication-run.json          — foundry-emitted run record
  wafer-tests/                  — per-wafer test reports
  components/                   — per-component measurement records
  package/                      — package record and laser-safety
                                  classification
  telemetry/                    — field-telemetry summaries
  recall-history/               — any recall notices affecting this
                                  module
  audit/                        — API audit log excerpts
```

The package is content-addressable; the manifest carries the SHA-256
of every file. The manifest is signed by the operating programme's
HTTP-message-signature key (RFC 9421) and counter-signed by the
foundry whose fabrication-run record appears in the package.

## §4 Manifest and Signatures

Verification tools recompute file digests, compare to the manifest,
and reject the package on any mismatch with type
`urn:wia:photonic-chip:evidence-mismatch`.

## §5 well-known URI Discovery

A conformant programme exposes a discovery document at
`/.well-known/wia-photonic-chip` that links to the API root, the
public accreditation certificate, the published quality dossier, the
catalogue of published designs, the round-robin participation
history, and any active recall notices.

## §6 System-Integrator Integration

System integrators (transceiver vendors, LiDAR builders, photonic-
sensing system houses, photonic-compute fabrics) consume packaged
modules and integrate them into deployed systems. Integration is
mediated by the package record (PHASE-1 §9) and the evidence
package; integrators record their downstream binding (transceiver
serial number, LiDAR head serial number) so that the deployed device
can be traced back to the photonic chip and through to the
fabrication run.

## §7 Recall Coordination

When a programme issues a recall (PHASE-3 §7), the recall notice is
published through the well-known discovery document and broadcast
through the streaming subscription (PHASE-2 §13). System integrators
acknowledge the recall through a signed receipt that lists the
deployed devices that contain affected modules. The API aggregates
receipts and publishes recall reach metrics for the operating
programme and the relevant authority.

## §8 Citation and Pinning

Externally cited photonic chips are referenced by their evidence
package's manifest digest. Citing parties retrieve the package once
and pin the digest in the citation. Programmes MUST keep evidence
packages addressable by their pinned digests for at least seven
years from the citation event.

## §9 Long-Term Archive Integration

Programmes designate a long-term archive that holds evidence packages
beyond programme wind-down. Quarterly deposits round-trip
content-addresses; on wind-down, remaining packages transfer to the
archive with content-addresses preserved.

## §10 Regulator and Accreditation Body Access

Regulators and accreditation bodies access the API via dedicated
client certificates. Access scopes for these clients include the full
design record set, including DRC/LVS reports and risk files;
consumer-facing scopes (foundry-customer, system-integrator,
citation tool) are narrower.

## §11 Public Catalogue and Aggregator Feeds

Programmes that publish a public catalogue emit an Atom or JSON Feed
listing the designs they have published with their evidence-package
manifest digests, the platform, the operating band, and the
foundry. The feed does not carry tape-out commercial detail; it is
intended for engineering discovery and supply-chain analysis.

## §12 Worked Example: From Tape-Out to Field Recall

1. Design house registers a design with a pinned PDK and submits
   a tape-out.
2. Foundry accepts the tape-out, completes fabrication, and
   registers the fabrication run with the wafer lot.
3. Wafer-test laboratory registers per-die results with component
   measurements.
4. Packaging facility registers packages and sets the laser-safety
   class per IEC 60825-1.
5. System integrators deploy modules into transceivers and upload
   field telemetry.
6. Reference laboratory re-measures deployed modules; an excursion
   is detected on a sub-population of devices.
7. Programme issues a recall notice; system integrators
   acknowledge with signed receipts.
8. Recall reach is published; affected devices are remediated; the
   evidence package is amended with the recall correspondence.

A conformant tool completes the technical portion of this flow
without further input from the operating programme.

## §13 Migration from Pre-Standard Records

Programmes that operated before WIA-photonic-chip reached version
1.0 MAY migrate historical designs by emitting synthetic design
records with a `legacyImport` flag.

## §14 Cross-Standard Linkage

Photonic chips that participate in adjacent WIA standards (a
silicon-photonic transceiver under WIA-optical-transport; a photonic
front-end for WIA-quantum-key-distribution) emit cross-standard
linkage records that name the consuming standard and the version
under which the linkage is claimed.

## §15 Reader Tooling

Evidence-package consumers include foundry auditors, integrator
acceptance teams, and certifying-body inspectors. Programmes MAY
publish supplementary reader hints (compressed plain-text summaries,
visual layout previews under ISO 10110, paginated PDF exports of
component-measurement statistics) alongside the canonical evidence
package.

## §16 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with PHASE-4
clients of the prior minor version. Major revisions go through a
deprecation window of at least one full PDK release cycle.

## §17 Eye-Diagram and BER Acceptance

For modules deployed in transceiver and high-speed link applications,
the integration record carries the eye-diagram and bit-error-rate
acceptance results. The acceptance report names the standard test
pattern (PRBS31, PRBS15, KP4-FEC compliant patterns), the loss
budget under which the test was run, and the measured eye height and
width along with the bit-error-rate at the agreed observation
window. Acceptance reports are signed by the test laboratory and are
attached to the evidence package as auxiliary records.

## §18 Optical-I-O Alignment Records

Modules that include fibre or free-space optical I-O carry alignment
records: the alignment method (active alignment, passive alignment),
the achieved coupling loss per port, the alignment uncertainty, and
the post-alignment cure or fixation method. Alignment records are
referenced from the package record and are part of the evidence
package.

## §19 Quantum and Classical Module Distinction

Modules that carry quantum optical functions (single-photon sources,
heralded entangled-pair generators, quantum-key-distribution
front-ends) are distinguished from classical modules in the package
record so that downstream consumers do not consume a quantum module
in a classical role or vice versa. Quantum modules carry
quantum-specific characterisation records (single-photon purity,
heralded efficiency, second-order correlation g²(0) measurements)
that are out of scope for classical modules; these records reference
the same evidence package and are signed by quantum-specialised
reference laboratories.

## §20 Photonic-Compute Fabric Integration

Modules that participate in photonic-compute fabrics (matrix-vector
multipliers, photonic neural networks, optical interconnect for AI
accelerators) carry compute-fabric integration records. The records
name the host system (a particular AI accelerator, a particular
data-centre interconnect product line), the slot the module
occupies, and the calibration the host applies to compensate for the
module's per-channel transfer characteristic. Compute-fabric
integration is bidirectional: the host supplies calibration coefficients
back to the module's calibration record (PHASE-1 §12) so that
post-deployment drift can be tracked.

## §21 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with prior-minor
clients: foundry adapters, integrator adapters, and citation tools
continue to function across minor revisions. Major revisions go
through a deprecation window of at least one full PDK release cycle.

## §22 Conformance and Sunset

A programme conformant with PHASE-4 has integrated successfully with
at least one design tool, at least one foundry, at least one
wafer-test laboratory, at least one packaging facility, and at least
one system integrator, and has published at least one externally
citable evidence package.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-photonic-chip
- **Last Updated:** 2026-04-27
