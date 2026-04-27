# WIA-nuclear-energy PHASE 4 — INTEGRATION Specification

**Standard:** WIA-nuclear-energy
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an accredited nuclear-energy programme
integrates with the systems that surround it: national nuclear
regulators; the IAEA safeguards inspectorate; energy market
operators; transmission system operators; fuel-cycle facility
operators (mining, conversion, enrichment, fabrication,
reprocessing); waste-management organisations; emergency-response
authorities; long-term archives; and certifying bodies. It also
defines the evidence-package format for citation and audit.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- IETF RFC 8615 (well-known URIs)
- IETF RFC 8288 (Web Linking)
- IETF RFC 9421 (HTTP Message Signatures)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO 19443:2018 (quality management for nuclear-supplier organisations)
- ISO 8601 (date and time)
- IAEA Safety Standards Series and Safeguards Agreements

---

## §1 National Regulator Integration

The national nuclear regulator is the primary external counterpart
of the operating programme. Integration is mutually authenticated;
the regulator's client certificate authorises submissions of
operating reports, environmental releases, safety events, and
licence-amendment applications. The integration record carries the
regulator's identifier, the submission cadence the regulator
requires, and the digital-signature requirements the regulator
applies to formal filings.

## §2 IAEA Safeguards Inspectorate Integration

For plants under safeguards, the IAEA inspectorate consumes
material balance reports, fuel-assembly inventories, and inspection-
related telemetry. Integration is mediated by the operating
programme's safeguards officer. The inspectorate's client
certificate is bound to the inspectorate's identifier and is
authorised only for the safeguards-relevant scope; out-of-scope
queries return `403 Forbidden`.

## §3 Evidence Package Format

```
evidence/
  manifest.json                  — package manifest (signed, see §4)
  plant.json                     — plant record
  authorisation/                 — current operating authorisation
                                   and amendment history
  safeguards/                    — safeguards agreement reference
                                   and submission history
  core-configurations/           — per-cycle configurations
  fuel-assemblies/               — assembly records and burnup
                                   histories
  operating-states/              — cycle-summary records
  outages/                       — per-outage records
  radiation-protection/          — RP samples and dose-ledger
                                   summaries
  spent-fuel/                    — inventory snapshots and waste
                                   categorisations
  environmental-releases/        — quarterly or annual reports
  safety-events/                 — INES classifications and
                                   investigations
  decommissioning/               — plan and milestone records
  audit/                         — API audit log excerpts
```

The package is content-addressable; the manifest carries the
SHA-256 of every file. The manifest is signed by the operating
programme's HTTP-message-signature key (RFC 9421) and counter-
signed by the regulator when the package is consumed for licensing
purposes.

## §4 Manifest and Signatures

Verification tools recompute file digests, compare to the manifest,
and reject the package on mismatch with type
`urn:wia:nuclear-energy:evidence-mismatch`.

## §5 well-known URI Discovery

A conformant programme exposes a discovery document at
`/.well-known/wia-nuclear-energy` that links to the API root, the
public licensing record, the published quality dossier, the
released environmental-release reports, and the catalogue of safety
events at INES-1 and above.

## §6 Fuel-Cycle Facility Integration

Fuel-cycle facilities (mining, conversion, enrichment, fabrication,
reprocessing) integrate via per-step custody records that link to
the assembly records (PHASE-1 §4). Each facility's contribution to
an assembly's history is signed by the facility's client
certificate. The integration carries the facility's identifier, the
relevant licences, and the per-shipment chain of custody.

## §7 Waste-Management Organisation Integration

Waste-management organisations consume waste-categorisation records
(PHASE-1 §8) and emit storage and disposal acknowledgements. The
integration is bidirectional: the operating programme publishes
waste records as they are produced, and the waste-management
organisation publishes acknowledgements that reference the original
records.

## §8 Energy Market and TSO Integration

Energy market operators and transmission system operators consume
plant operational state for dispatch and grid balancing. The
integration is read-only; market and TSO clients consume
operational-state summaries (gross MWe, net MWe, expected
availability) but do not consume safety-related telemetry.

## §9 Emergency Response Authority Integration

National and local emergency response authorities receive
notifications when safety events of INES-2 or above occur. The
integration record carries the authority's intake endpoint, the
notification format the authority requires, and the maximum
notification latency the regulator has set.

## §10 Long-Term Archive Integration

Programmes designate a long-term archive that holds operational
records beyond programme wind-down. Quarterly deposits round-trip
content-addresses; on programme wind-down or decommissioning, the
remaining records transfer to the archive with content-addresses
preserved.

## §11 Verifiable-Credential Re-Issuance (optional)

Programmes that wish to expose plant attestations (current
operating authorisation, safeguards compliance status) to consumers
of W3C Verifiable Credentials MAY re-issue the attestations as
Verifiable Credentials under the Data Model 2.0 specification. Re-
issuance is optional; the canonical record remains the JSON
evidence-package manifest.

## §12 Worked Example: Citation Resolution

A reader encounters a peer-reviewed publication that cites a
plant's annual environmental-release report. The reader's tool
resolves the citation by:

1. Parsing the citation to extract the plant ID, the reporting
   interval, and the manifest digest.
2. Fetching the discovery document for the operating programme.
3. Resolving the manifest URL and verifying the manifest signatures.
4. Recomputing the manifest digest and comparing it to the pinned
   digest; aborting on mismatch.
5. Retrieving the package; recomputing per-file digests; surfacing
   the resolved evidence (environmental release report,
   atmospheric/aquatic dispersion model, monitoring data
   references) to the reader.

A conformant tool completes this flow without further input.

## §13 Cross-Standard Linkage

Programmes that consume adjacent WIA standards (WIA-radiation-
protection for occupational dosimetry, WIA-grid-stability for
dispatch interactions) emit cross-standard linkage records.
Linkages are not transitive; consumers verify each adjacent
standard's evidence directly.

## §14 Reader Tooling

Programmes MAY publish supplementary reader hints (visual
operating-history charts, environmental-release rollups, safety-
event histograms) alongside the canonical evidence package. Reader
tools are non-normative.

## §15 Public Catalogue and Aggregator Feeds

Programmes that publish a public catalogue of plants emit an Atom
or JSON Feed listing the plants with their licensing status,
operating phase, last-released environmental report, and last
safety event at INES-1 or above. The feed does not carry
operational telemetry; it is intended for transparency and policy
analysis.

## §16 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with prior-
minor clients. Major revisions go through a deprecation window of
at least one full operating-licence renewal cycle.

## §17 Migration from Pre-Standard Records

Programmes that operated before WIA-nuclear-energy reached
version 1.0 MAY migrate historical records by emitting synthetic
plant records with a `legacyImport` flag. Synthetic plants are
accepted by the public catalogue but are not eligible for
evidence-package generation without contemporaneous re-validation
of the licensing and safeguards records.

## §18 Operating-Experience Network Integration

The operating programme integrates with operating-experience sharing
networks (the World Association of Nuclear Operators, the IAEA
Incident Reporting System, and similar bodies) through one-way push
adapters that emit anonymised event-sharing records keyed to the
underlying safety event. The receiving network's identifier and the
push schedule are recorded in the integration dossier.

## §19 Configuration-Management Adapter

A configuration-management adapter exchanges plant-state document
revisions (PHASE-1 §12) with the regulator's submission portal. The
adapter signs each submission with the operating programme's
release key, attaches the regulator's approval reference on
acceptance, and re-publishes the in-force document set through the
public catalogue.

## §20 Public-Information Integration

Public-information consumers (educators, science communicators,
journalists) consume the operating programme's plain-language
summaries: capacity-factor trends, dose-to-public estimates, and
safety-event summaries above operator-precursor classification. The
public-information integration is read-only and rate-limited; raw
operational telemetry is never exposed through this integration.

## §21 ISI Vendor Integration

In-service inspection vendors integrate with the operating
programme via per-campaign work-order records that link to the
plant's ISI plan. The vendor's accreditation reference and the per-
inspector qualifications are exposed in the integration's audit
trail so that downstream regulators can verify that the inspections
were performed by qualified personnel under accredited
laboratories.

## §22 Reader Tooling for Operational History

Operational history (capacity factor over time, outage durations,
fuel-burnup distributions, dose-to-public estimates) is dense and
benefits from visualisation. Programmes MAY publish reader tools
that render time-series charts, capacity-factor histograms, and
event-distribution maps alongside the canonical evidence package.
Reader tools are non-normative.

## §23 Conformance and Sunset

A programme conformant with PHASE-4 has integrated successfully
with the national regulator, the IAEA inspectorate (where
applicable), at least one fuel-cycle facility, the relevant
waste-management organisation, the energy market operator, and the
emergency-response authority, and has published at least one
externally citable evidence package.

Sunsetting an integration is announced via the well-known
discovery document at least 90 calendar days before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-nuclear-energy
- **Last Updated:** 2026-04-27
