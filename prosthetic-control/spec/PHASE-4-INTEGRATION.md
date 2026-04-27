# WIA-prosthetic-control PHASE 4 — INTEGRATION Specification

**Standard:** WIA-prosthetic-control
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an accredited prosthetic-control programme
integrates with the systems that surround it: clinic electronic health
records that hold the user's clinical record; manufacturer post-market
surveillance systems; clinical-trial sponsors that aggregate device-use
data; national medical-device authorities that consume adverse-event
reports; insurance carriers that price and underwrite assistive-device
benefits; rehabilitation outcome registries; and long-term archives
that preserve evidence packages beyond programme wind-down. It also
defines the evidence-package format that bundles a fitted device's
complete record for external audit.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- IETF RFC 8615 (well-known URIs)
- IETF RFC 8288 (Web Linking)
- IETF RFC 9421 (HTTP Message Signatures)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27701:2019 (privacy information management)
- ISO 13485:2016 (medical-device QMS)
- ISO 14971:2019 (risk management)
- ISO 14155:2020 (clinical investigation)
- IEC 60601-1 (medical electrical safety)
- IEC 62304 (medical device software life cycle)
- IEC 62366-1 (usability engineering)
- ISO 8601 (date and time)
- HL7 FHIR R5 (Patient, Device, DeviceUseStatement, Observation,
  ClinicalImpression)

---

## §1 Clinic EHR Integration

The clinic's electronic health record holds the user's clinical
identity. Integration with the EHR is achieved through a thin adapter
that maps WIA-native subject records to FHIR R5 resources. The adapter
is owned by the operating programme and is exercised against the
clinic's FHIR test endpoint at least once per quarter so that EHR
upgrades do not silently break the binding.

The adapter exposes the FHIR bridge documented in PHASE-2 §9 as a
read-only facade for the clinic; the clinic's writes flow through the
WIA-native endpoints, never through FHIR write APIs.

## §2 Manufacturer Post-Market Surveillance

Manufacturers operate post-market surveillance systems that aggregate
device-use, motor-command, and adverse-event data across the user
population. Integration is achieved through a one-way push from the
operating programme to the manufacturer's system, scoped to the
manufacturer's devices only. The push is authenticated with mutually-
authenticated TLS and the manufacturer's surveillance team's client
certificate.

Aggregated data is de-identified at the programme boundary; individual
subject identifiers are replaced with pseudonyms that are reversible
only at the operating programme. Manufacturers see population-level
trends and individual-device performance under the manufacturer's
warranty terms.

## §3 Evidence Package Format

The evidence package is the externally citable artefact for a fitted
device. The package is a tarball with the following layout:

```
evidence/
  manifest.json                — package manifest (signed, see §4)
  subject.json                 — subject record (PHASE-1 §2)
  device.json                  — device record
  acquisitions/                — per-acquisition records
  decoders/                    — decoder records and adaptation logs
  motor-commands/              — motor-command streams
  calibrations/                — calibration session records
  feedback/                    — feedback session records
  adverse-events/              — adverse-event reports and follow-up
  risk-file/                   — content-addressed risk-file references
  software-lifecycle/          — IEC 62304 artefact references
  audit/                       — API audit log excerpts
```

The package is content-addressable; the manifest carries the SHA-256
of every file. The manifest is signed by the operating programme's
HTTP-message-signature key (RFC 9421) and counter-signed by the
manufacturer when the package is consumed for warranty or
post-market-surveillance purposes.

## §4 Manifest and Signatures

Verification tools that follow this PHASE recompute the file digests,
compare them to the manifest, and reject the package on any mismatch
with type `urn:wia:prosthetic-control:evidence-mismatch`.

## §5 well-known URI Discovery

A conformant programme exposes a discovery document at
`/.well-known/wia-prosthetic-control` (RFC 8615) that links to the
API root, the public ISO 13485 certificate, the published risk-file
catalogue, and the post-market-surveillance reporting cadence.

## §6 Subject Identity Mediation

The user's clinical identity is held only in the clinic EHR. The
DATA-FORMAT layer carries opaque tokens (PHASE-1 §2). When an
external consumer (an insurance carrier, a research collaborator)
requests identity-bound data, the request is mediated by the EHR
through a subject-access workflow that produces a redacted export
suitable for the consumer's authorisation scope.

A subject-access request from a user returns the user's own subject
record, calibration history, and adverse-event reports in a
human-readable form alongside the machine-readable evidence package
manifest digest.

## §7 Insurance Carrier Integration

Insurance carriers that underwrite assistive-device benefits consume
selected sections of the evidence package: the calibration history
(for benefit eligibility), the adverse-event report set (for liability
analysis), and the device-use cadence (for benefit utilisation).
Programmes SHOULD publish a profile of evidence sections shareable
with carriers under a standard data-sharing agreement; carriers
SHOULD limit requests to that profile. Out-of-profile requests return
`403 Forbidden` with type
`urn:wia:prosthetic-control:evidence-profile`.

## §8 Rehabilitation Outcome Registries

Rehabilitation outcome registries aggregate user-reported outcomes
(e.g. PROMIS measures), functional benchmarks (e.g. Box-and-Block,
6-minute walk), and device satisfaction surveys. Programmes that
participate in a registry register through the registry's onboarding
process and emit registry-formatted exports on a schedule the
registry defines.

The registry export does not carry clinical identifiers; the
registry maintains its own pseudonym scheme. The operating programme
records the export schedule and the registry's identifier in its
quality dossier.

## §9 Clinical-Trial Sponsor Integration

Clinical-trial sponsors that operate ISO 14155-aligned investigations
consume per-subject acquisition and motor-command data scoped to the
trial cohort. Integration is mediated by the operating clinic's IRB-
approved data-sharing plan; the sponsor's client certificate is bound
to the trial cohort code and cannot retrieve data outside the cohort.

## §10 National Medical-Device Authority Integration

National medical-device authorities consume adverse-event reports
through the integration documented in PHASE-2 §11 and PHASE-3 §10.
The authority's intake endpoint is published in the programme's
quality dossier; the operating programme refreshes the integration
configuration whenever the authority republishes its endpoint or its
report schema.

## §11 Long-Term Archive Integration

Each programme designates a long-term archive that holds evidence
packages beyond programme wind-down. Evidence packages are deposited
on a quarterly cadence; the deposit is verified by re-pinning the
content-addresses on each deposit. On wind-down, remaining packages
transfer to the archive with content-addresses preserved.

## §12 Worked Example: Adverse-Event Reporting Flow

1. A serious adverse event occurs during home use; the clinic
   detects the event via the device's safety log on the next sync.
2. The clinic's clinical reviewer confirms the event and POSTs an
   adverse-event record (PHASE-2 §11).
3. The API forwards the report to the configured national authority
   over mutually-authenticated TLS within the authority's required
   window.
4. The authority returns a report reference, which the API appends
   to the event record.
5. The manufacturer's post-market surveillance team is notified
   through the §2 push and opens an investigation.
6. Follow-up visits, device modifications, and corrective actions
   are appended to the event record over time.

A conformant tool completes the technical portion of this flow
(steps 2-4 and 6) without further input from the clinic.

## §13 Migration from Pre-Standard Records

Programmes that operated before WIA-prosthetic-control reached
version 1.0 MAY migrate historical fittings by emitting a synthetic
subject record that carries the original fitting's identifying
information plus a `legacyImport` flag. Synthetic subjects are
accepted by the rehabilitation registry and clinic EHR profiles but
are not eligible for adverse-event reporting back-fill without a
contemporaneous clinical re-evaluation.

## §14 Public Catalogue and Aggregator Feeds

Programmes that publish a public catalogue of fitted-device counts and
adverse-event statistics emit an Atom or JSON Feed listing the
aggregate counts by device class, indication, and quarter. The feed
does not carry subject identifiers and does not list per-event detail;
it is intended for transparency and policy analysis, not clinical
review.

## §15 Cross-Standard Linkage

A prosthetic-control programme that participates in adjacent WIA
standards (a peripheral-nerve interface that ships under
WIA-neural-interface, a BCI front-end that ships under WIA-bci) emits
a cross-standard linkage record that names the consuming standard,
the version under which the linkage is claimed, and the assertion
that the prosthetic-control implementation satisfies the consuming
standard's interface requirements. Linkages are not transitive;
consumers verify each consuming standard's evidence directly.

## §16 Reader Tooling and Accessibility

Evidence-package consumers include screen-reader-driven clinical
reviewers, low-bandwidth deployments at remote clinics, and offline
auditors at certifying bodies. A programme MAY publish supplementary
reader hints (compressed plain-text summaries, accessibility-friendly
HTML renderings, paginated PDF exports) alongside the canonical
evidence package; supplementary hints are non-normative and are
clearly labelled as such in the manifest so that readers do not
confuse them with authoritative records.

## §17 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with PHASE-4
clients of the prior minor version: integration adapters, evidence-
package consumers, and FHIR-bridge consumers continue to function
without modification across minor revisions. Major-revision changes
go through a deprecation window that lasts at least one full
device-fitting cohort cycle.

## §18 Conformance and Sunset

A programme conformant with PHASE-4 has integrated successfully with
at least one clinic EHR, at least one manufacturer post-market
surveillance system, the relevant national medical-device authority,
and at least one rehabilitation outcome registry, and has published
at least one externally citable evidence package. The integration
dossier records the integrations and the test runs that confirmed
each.

Sunsetting an integration is announced via the well-known discovery
document at least 90 calendar days before removal; in-flight
fittings that depend on the sunsetting integration are migrated
before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-prosthetic-control
- **Last Updated:** 2026-04-27
