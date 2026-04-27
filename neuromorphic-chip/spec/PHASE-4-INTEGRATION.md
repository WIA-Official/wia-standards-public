# WIA-neuromorphic-chip PHASE 4 — INTEGRATION Specification

**Standard:** WIA-neuromorphic-chip
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an accredited neuromorphic-chip programme
integrates with the systems that surround it: SDK and compiler
toolchains; vendor publication channels for hardware descriptions;
sensor front-ends that emit address-event traffic (event cameras,
spiking cochlea, tactile event sensors); host runtimes that manage
the deployed accelerator; long-term archives that preserve evidence
packages; and the regulators and certifying bodies that audit
neuromorphic deployments. It also defines the evidence-package
format for citation.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- IETF RFC 8615 (well-known URIs)
- IETF RFC 8288 (Web Linking)
- IETF RFC 9421 (HTTP Message Signatures)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO 8601 (date and time)
- W3C Verifiable Credentials Data Model 2.0 (optional re-issuance)

---

## §1 SDK and Compiler Toolchain Integration

SDKs that compile networks for deployment integrate via plugins or
adapters that submit network descriptions and consume mapping
results. Adapters are owned by the operating programme; SDK-vendor
responsibility is limited to keeping the adapter's public API stable.
Adapters MUST honour the model-support matrix declared by the target
hardware description and MUST refuse to submit a network that
declares a model not supported by the target.

## §2 Vendor Publication Channel

Vendors publish hardware descriptions through the operating programme's
API per PHASE-2 §4. The publication channel is mutually
authenticated; the vendor's release key signs each description.
Programmes mirror published descriptions to their public catalogue
under the vendor's profile.

## §3 Evidence Package Format

```
evidence/
  manifest.json                — package manifest (signed, see §4)
  network.json                 — network record
  network-description/         — network description archive
  hardware.json                — hardware description record
  mapping/                     — mapping records and routing plans
  aer-streams/                 — AER traffic (full or windowed summary)
  plasticity/                  — plasticity event logs
  characterisations/           — per-core characterisation records
  calibration/                 — calibration tables in use at deployment
  telemetry/                   — telemetry summaries
  audit/                       — API audit log excerpts
```

The package is content-addressable; the manifest carries the SHA-256
of every file. The manifest is signed by the operating programme's
HTTP-message-signature key (RFC 9421) and counter-signed by the
hardware vendor when the package is consumed for warranty or
support purposes.

## §4 Manifest and Signatures

Verification tools recompute file digests, compare to the manifest,
and reject the package on any mismatch with type
`urn:wia:neuromorphic-chip:evidence-mismatch`.

## §5 well-known URI Discovery

A conformant programme exposes a discovery document at
`/.well-known/wia-neuromorphic-chip` that links to the API root, the
public accreditation certificate, the published quality dossier, the
catalogue of published networks, and any active recall notices.

## §6 Sensor Front-End Integration

Event cameras, spiking cochlea, tactile-event sensors, and other
event-driven sensors emit AER traffic that flows directly into the
deployed accelerator. The integration record carries the sensor
identifier, the AER format the sensor emits, and the address-space
mapping from sensor output to network input populations. Sensor
front-ends MAY be operated under adjacent WIA standards; the
integration record cross-references those standards under §13.

## §7 Host-Runtime Integration

Host runtimes manage the deployed accelerator's lifecycle: power-on,
mapping load, AER stream routing, plasticity-state persistence at
shutdown, and recovery from faults. Host runtimes integrate with the
operating programme's API to fetch mappings and to publish telemetry
and plasticity events. The runtime's API client carries the system-
integrator's client certificate; the certificate is bound to the
deployed devices the integrator is authorised to manage.

## §8 Long-Term Archive Integration

Programmes designate a long-term archive that holds evidence packages
beyond programme wind-down. Quarterly deposits round-trip
content-addresses; on wind-down, remaining packages transfer to the
archive with content-addresses preserved.

## §9 Regulator and Accreditation Body Access

Regulators and accreditation bodies access the API via dedicated
client certificates issued by the certifying body. Access scopes for
these clients include the full record set; consumer-facing scopes
(SDK developer, integrator, citation tool) are narrower.

## §10 Verifiable-Credential Re-Issuance (optional)

Programmes that wish to expose their characterisation summaries to
consumers of W3C Verifiable Credentials MAY re-issue a characterisation
summary as a Verifiable Credential under the Data Model 2.0
specification. Re-issuance is optional; the canonical record remains
the JSON evidence package.

## §11 Public Catalogue and Aggregator Feeds

Programmes that publish a public catalogue of compiled networks emit
an Atom or JSON Feed listing the networks with their evidence-
package manifest digests, the target hardware family, and the
network's purpose. The feed does not carry sensor input data or
trained weight matrices; it is a discovery mechanism, not a primary
record.

## §12 Worked Example: Citation Resolution

A reader encounters a paper that cites a neuromorphic-compute result
with the WIA citation extension. The reader's tool resolves the
citation by:

1. Parsing the citation to extract the mapping ID and manifest digest.
2. Fetching the discovery document for the issuing programme.
3. Resolving the manifest URL and verifying the manifest signatures.
4. Recomputing the manifest digest and comparing it to the pinned
   digest; aborting on mismatch.
5. Retrieving the package; recomputing per-file digests; surfacing the
   resolved evidence (network description, hardware description,
   mapping, characterisation, plasticity log) to the reader.

A conformant tool completes this flow without further input.

## §13 Cross-Standard Linkage

A neuromorphic deployment that consumes input from a WIA-event-camera
sensor or that emits decisions consumed by a WIA-prosthetic-control
prosthesis emits cross-standard linkage records. Linkages are not
transitive; consumers of a downstream result verify the consuming
standard's evidence directly.

## §14 Reader Tooling

Evidence-package consumers include compiler-toolchain auditors,
deployment-team reviewers, and certifying-body inspectors. Programmes
MAY publish supplementary reader hints (visual mapping plots,
plasticity-log summaries, energy-budget reports) alongside the
canonical evidence package.

## §15 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with prior-minor
clients. Major revisions go through a deprecation window of at
least one full hardware-description release cycle.

## §16 Workload-Definition Catalogue

A workload-definition catalogue is published by the operating
programme for collaborator use. Catalogue entries cross-reference the
community workloads that the programme has adopted and the
programme-defined workloads under which it publishes its own
results. Each entry carries the workload's content-address, a
human-readable description, the date of last revision, and the
programme's reproducibility tolerance for the workload.

## §17 Energy-Reporting Convention

The integration's energy-reporting convention follows the format
documented in the energy-account record (PHASE-1 §11). Programmes
that participate in energy-comparison studies (cross-platform
benchmarks for low-power inference) submit their energy accounts to
the study coordinator under the study's named protocol. The
operating programme records each study participation in its quality
dossier so that downstream consumers can verify the lineage of
externally cited energy claims.

## §18 Open-Source Adapter Encouragement

Adapters between SDKs and the WIA API are SHOULD be open-source so
that downstream consumers can audit the path from their network to
the deployed mapping. Closed-source adapters are permitted but are
flagged in the integration record so that consumers consuming a
closed-toolchain result understand the audit constraint.

## §19 Reproducibility-Test Endpoint

Citation tools that wish to verify a published result without
re-deploying it consume a reproducibility-test endpoint that the
operating programme exposes. The endpoint accepts a network-mapping-
hardware tuple and returns a deterministic-execution receipt or a
non-determinism flag with the workload's tolerance bounds.

## §20 Cross-Standard Linkage to Sensor Front-Ends

When a deployed network consumes input from a sensor front-end
governed by an adjacent WIA standard (event-camera, spiking-cochlea,
tactile-event), the integration record carries a cross-standard
linkage that names the sensor standard, the version under which the
linkage is claimed, and the address-space mapping from sensor output
to network input. Linkages are not transitive; consumers verify each
sensor standard's evidence directly.

## §21 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with prior-minor
clients: SDK adapters, host-runtime adapters, and citation tools
continue to function across minor revisions. Major revisions go
through a deprecation window of at least one full hardware-
description release cycle.

## §22 Migration from Pre-Standard Records

Programmes that operated before WIA-neuromorphic-chip reached
version 1.0 MAY migrate historical mappings by emitting synthetic
mapping records that carry the original mapping's identifying
information plus a `legacyImport` flag. Synthetic mappings are
accepted by the public catalogue but are not eligible for evidence-
package generation without a contemporaneous re-execution against
the documented workload.

## §23 Reader Tooling for Plasticity Logs

Plasticity logs are large and require tooling to summarise for
review. Programmes MAY publish plasticity-log summary tools that
emit time-binned histograms of weight changes, projection-level
heat maps, and rule-attribution breakdowns. Summary outputs are
non-normative reader hints; the canonical record remains the
plasticity-event log itself.

## §24 Conformance and Sunset

A programme conformant with PHASE-4 has integrated successfully with
at least one SDK toolchain, at least one hardware vendor, at least
one sensor front-end, and at least one system integrator, and has
published at least one externally citable evidence package.

Sunsetting an integration is announced via the well-known discovery
document at least 90 calendar days before removal; in-flight
deployments that depend on the sunsetting integration are migrated
before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-neuromorphic-chip
- **Last Updated:** 2026-04-27
