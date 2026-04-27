# WIA-moon-base PHASE 4 — INTEGRATION Specification

**Standard:** WIA-moon-base
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an accredited moon-base programme
integrates with the systems that surround it: cislunar relay
networks; partner mission operations centres; supply-chain providers
and launch services; surface mobility-asset providers; crew-medical
operations centres; ISRU plant vendors; archives; and inter-agency
review boards. It also defines the evidence-package format that
bundles a base's record set for citation and audit.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- IETF RFC 8615 (well-known URIs)
- IETF RFC 8288 (Web Linking)
- IETF RFC 9421 (HTTP Message Signatures)
- ISO/IEC 27001:2022 (information security management)
- ISO 8601 (date and time)
- CCSDS 132.0-B / 232.0-B (Space Data Link Protocols)
- CCSDS 727.0-B (CFDP)
- COSPAR Planetary Protection Policy

---

## §1 Cislunar Relay Network Integration

The cislunar relay network (whether operated by a single agency, by
a consortium, or by a commercial provider) carries TT&C traffic
between the base and Earth. Integration is mutually authenticated;
the relay operator publishes the relay's availability schedule and
the per-pass quality metrics the base records against its operations.

## §2 Partner MOC Integration

Partner mission operations centres receive base records on the
schedule negotiated at programme inception. Some records (life-
support telemetry summaries, microgrid state changes) flow in
near-real time through the streaming subscription endpoint; others
(daily summaries, evidence packages) flow at fixed cadences.

## §3 Evidence Package Format

```
evidence/
  manifest.json                — package manifest (signed, see §4)
  base.json                    — base record
  habitat-modules/             — per-module configuration and
                                  certification references
  life-support/                — life-support telemetry windows or
                                  summaries
  power/                       — power and microgrid records
  isru/                        — per-plant operating records
  mobility/                    — mobility-asset operations
  eva/                         — EVA records
  comms/                       — link logs
  spectrum/                    — spectrum file revisions
  supply-chain/                — manifest and inventory deltas
  anomalies/                   — anomaly investigations
  audit/                       — API audit log excerpts
```

The package is content-addressable; the manifest carries the
SHA-256 of every file. The manifest is signed by the operating
programme's HTTP-message-signature key (RFC 9421) and counter-signed
by the surface-operations director when the package covers an
operational period.

## §4 Manifest and Signatures

Verification tools recompute file digests, compare to the manifest,
and reject the package on mismatch with type
`urn:wia:moon-base:evidence-mismatch`.

## §5 well-known URI Discovery

A conformant programme exposes a discovery document at
`/.well-known/wia-moon-base` that links to the API root, the
public base registration, the published quality dossier, the
supply-chain release schedule, the catalogue of released science
products, and the anomaly investigation history.

## §6 Supply-Chain Provider Integration

Cargo and crew-launch providers integrate through the supply-chain
record. The integration captures launch manifests, the upmass
allocated to the base, the agreed delivery window, and the
post-arrival inventory delta.

## §7 Surface-Mobility Vendor Integration

Surface-mobility vendors (rover providers, hopper builders,
excavator operators) integrate through the mobility-asset record.
The integration carries the vendor's identifier, the asset's
service contract, the maintenance schedule, and the per-asset
warranty.

## §8 Crew-Medical Operations Integration

Crew-medical operations centres consume EVA and life-support
records that intersect crew exposure (consumables consumption,
incident-related vitals) under the crew-medical standard's binding.
The integration is mediated by an opaque token; clinical identity
flows only inside the crew-medical standard's facade.

## §9 ISRU Plant Vendor Integration

ISRU plant vendors integrate through the isru-plant record. The
integration captures the vendor's process specification, the
expected throughput envelope, the consumables-input model, and the
maintenance and recall path.

## §10 Long-Term Archive Integration

Programmes designate a long-term archive that holds operational
records beyond programme wind-down. Quarterly deposits round-trip
content-addresses; on programme wind-down or transition to
caretaker mode, remaining records transfer to the archive with
content-addresses preserved.

## §11 Inter-Agency Review Board Integration

Inter-agency review boards (mission-critical-activity reviews,
anomaly review boards, life-support governance boards) receive
records appropriate to their scope through dedicated client
certificates.

## §12 Worked Example: Citation Resolution

A reader encounters a peer-reviewed publication that cites a
base's environmental record over a particular sol. The reader's
tool resolves the citation by:

1. Parsing the citation to extract the base ID, the time window,
   and the manifest digest.
2. Fetching the discovery document for the issuing programme.
3. Resolving the manifest URL and verifying the manifest signatures.
4. Recomputing the manifest digest and comparing it to the pinned
   digest; aborting on mismatch.
5. Retrieving the package; recomputing per-file digests; surfacing
   the resolved evidence (life-support records, power records, EVA
   records over the window) to the reader.

A conformant tool completes this flow without further input.

## §13 Cross-Standard Linkage

Bases that consume adjacent WIA standards (crew-medical, ISRU,
surface-mobility, deep-space-comms) emit cross-standard linkage
records that name the consuming standard and the version under
which the linkage is claimed. Linkages are not transitive;
consumers verify each adjacent standard's evidence directly.

## §14 Reader Tooling

Programmes MAY publish supplementary reader hints (visual base-
layout maps, life-support timeline visualisers, EVA route maps,
power-budget rollups) alongside the canonical evidence package.
Supplementary hints are non-normative.

## §15 Verifiable-Credential Re-Issuance (optional)

Programmes that wish to expose base attestations (life-support
certification, EVA authorisation) to consumers of W3C Verifiable
Credentials MAY re-issue the attestations as Verifiable Credentials
under the Data Model 2.0 specification. Re-issuance is optional;
the canonical record remains the JSON evidence-package manifest.

## §16 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with prior-
minor clients. Major revisions go through a deprecation window of
at least one full crew-rotation cycle.

## §17 Heritage-Site Coordination Integration

Heritage-site coordination integrates with the inter-agency
heritage-protection registries that the participating agencies
recognise. The integration carries the registry's identifier, the
heritage zones the registry tracks, and the change-notification
schedule. Updates to a heritage zone (geometry expansion, status
change) trigger the API to re-evaluate active traverse plans and to
emit alerts where existing plans would now breach the updated zone.

## §18 Habitability Study Coordinator Integration

Long-term habitability study coordinators consume aggregated
observations from participating bases. The integration carries the
coordinator's identifier, the data scope (atmospheric, radiation
environment, psychosocial telemetry summaries), and the consent
regime under which the data flows. The integration is one-way; the
study coordinator does not write into base records.

## §19 Public-Outreach Integration

Public-outreach systems consume daily-summary records and crew-
generated narrative content. The integration is read-only and rate-
limited; outreach systems do not consume operational records (life-
support, power, EVA) by default. Opt-in disclosures (a crew member
choosing to share narrative content publicly) are recorded against
the relevant operational record so that the public release
provenance is traceable.

## §20 Radiation-Environment Aggregator Integration

Radiation environment observations across multiple bases are
aggregated by space-weather and habitability research consortia.
The aggregator integration consumes time-binned dose-rate
observations (PHASE-1 §13) and emits cross-base dose-rate maps
that re-credit the contributing bases. Integrations carry the
aggregator's identifier and the access scope.

## §21 Pre-Standard Migration

Programmes that operated before WIA-moon-base reached version 1.0
MAY migrate historical operations by emitting synthetic base
records that carry the original operations' identifying information
plus a `legacyImport` flag. Synthetic bases are accepted by the
public catalogue but are not eligible for evidence-package
generation without contemporaneous re-validation.

## §22 Cross-Standard Linkage

Bases that consume adjacent WIA standards (crew-medical, ISRU,
deep-space-comms, surface-mobility) emit cross-standard linkage
records. Linkages are not transitive; consumers verify each
adjacent standard's evidence directly.

## §23 Citation-Indexing Adapters

Bibliographic indexing services consume citation entries that point
back to base evidence packages. Programmes MAY publish indexing
adapters that emit entries in BibTeX, JSON-LD, or the indexing
service's preferred format. Adapters are non-normative integration
extensions.

## §24 Reader Tooling for Operational Records

Operational records are dense and benefit from visualisation. A
programme MAY publish supplementary reader tools (sol-by-sol
operations charts, EVA route maps with terrain overlays, life-
support trend visualisers) alongside the canonical evidence package.
Reader tools are non-normative; the canonical record remains the
JSON evidence-package manifest.

## §25 Operational Audit-Trail Export

External auditors that conduct retrospective reviews of base
operations consume audit-trail exports through the integration
layer. The audit-trail export includes the API audit log, the
streaming-event log, the certificate-rotation history, and the
configuration-change log. Auditor client certificates are issued by
the certifying body; access is scoped to the audit window the
auditor was commissioned to review.

## §26 Verifiable-Credential Re-Issuance for Base Attestations

Programmes that wish to expose base attestations (life-support
certification, EVA authorisation, contingency-window compliance)
to consumers of W3C Verifiable Credentials MAY re-issue the
attestations as Verifiable Credentials under the Data Model 2.0
specification. Re-issuance is optional; the canonical record
remains the JSON evidence-package manifest.

## §27 Conformance and Sunset

A programme conformant with PHASE-4 has integrated successfully
with at least one cislunar relay operator, at least one supply-
chain provider, at least one surface-mobility vendor, the crew-
medical operations integration, and at least one inter-agency
review board, and has published at least one externally citable
evidence package.

Sunsetting an integration is announced via the well-known
discovery document at least 90 calendar days before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-moon-base
- **Last Updated:** 2026-04-27
