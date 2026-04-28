# WIA-interstellar-communication PHASE 4 — INTEGRATION Specification

**Standard:** WIA-interstellar-communication
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an interstellar-communication
operator integrates with the systems that surround it: partner
observatories conducting follow-up observations, the IAA
SETI Permanent Committee post-detection adjudication network,
spectrum regulators, IVOA-aligned Virtual Observatory data
catalogues, exoplanet-host catalogue providers (NASA Exoplanet
Archive, ExoplaneT.eu), precursor probe operators, deliberate-
transmission governance review boards, and long-term archives
that preserve observation campaigns across decades.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO 8601 (date and time)
- ISO 19115-1 (geographic information — metadata)
- ITU-R RA.769 / RA.1513 / RR Article 29
- IVOA ObsCore / ObsTAP / VOTable / UCD1+
- IAU Standards of Fundamental Astronomy (SOFA)
- W3C Verifiable Credentials Data Model 2.0 (optional)

---

## §1 Partner Observatory Integration

Partner observatories perform follow-up observations on
candidates that the operator has advanced to
`reproducibility-state=follow-up-pending`. Integration carries
the partner's identifier, the per-target observing capability
manifest (band, sensitivity, drift-rate tolerance), and the
follow-up observation completion endpoint.

Partner-side observation results emit per-candidate updates
through the streaming subscription so that the operator can
advance the candidate's reproducibility state on receipt of
the partner's analysis.

## §2 Post-Detection Adjudication Network Integration

Operators that adhere to the IAA SETI Permanent Committee
post-detection protocol (PHASE-3 §3) integrate with the
adjudication network. Integration carries the network's
identifier, the per-network member observatories that
participate in the operator's escalation path, and the
network's published adjudication-cycle protocol.

## §3 Spectrum Regulator Integration

The operating jurisdiction's spectrum regulator consumes the
operator's licence renewal applications, RA.1513 interference
reports, and any deliberate-transmission band-occupancy
notifications. Integration carries the regulator's identifier,
the per-licence renewal cadence, and the operator's
interference-report submission template.

## §4 Virtual Observatory Catalogue Integration

IVOA-aligned Virtual Observatory catalogues consume the
operator's observation records (PHASE-1 §4) per ObsCore /
ObsTAP semantics so that the broader astronomy community can
discover the operator's observations alongside non-SETI
observations. The integration carries the VO catalogue's
identifier, the per-observation deposit cadence, and the
catalogue's published metadata profile.

## §5 Exoplanet Host Catalogue Integration

Exoplanet-host catalogues (NASA Exoplanet Archive,
ExoplaneT.eu, the operator's local exoplanet target list)
provide the target catalogue (PHASE-1 §3) for exoplanet-
focused search programmes. Integration carries each catalogue's
identifier, the per-host cross-walk reference, and the
catalogue's release cadence so that the operator's target
list refreshes as new exoplanet hosts are confirmed or
revised.

## §6 Precursor Probe Operator Integration

Precursor probe operators (NASA Voyager Interstellar Mission,
ESA-NASA partnerships for proposed near-future small probes,
private-funded probe initiatives) emit probe-telemetry link
records (PHASE-1 §7). Integration carries the probe operator's
identifier, the per-link content-address, and the science-
product release schedule that the operator commits to.

## §7 Deliberate-Transmission Governance Board Integration

Programmes that propose deliberate transmissions (PHASE-1 §8)
integrate with their governance board. Integration carries
the board's identifier, the per-proposal review-cycle
protocol, the recorded review outcome, and the post-approval
public consultation record where the operator's jurisdiction
expects public consultation.

## §8 Evidence Package Format

```
evidence/
  manifest.json                — package manifest (signed)
  programme.json               — programme record
  targets/                     — target records cited in the
                                  evidence interval
  observations/                — observation records and
                                  archive references
  candidates/                  — candidate records and per-
                                  candidate reproduction chain
  rfi-entries/                 — RFI catalogue snapshots
  probe-telemetry-links/       — link records
  deliberate-transmissions/    — transmission records and
                                  governance reviews
  audit/                       — API audit log excerpts
```

The package is content-addressable; the manifest is signed
by the operator's HTTP-message-signature key (RFC 9421) and,
for post-detection-cleared evidence, counter-signed by the
post-detection adjudication network's chair.

## §9 Manifest and Signatures

Verification tools recompute file digests, compare to the
manifest, and reject the package on mismatch with type
`urn:wia:interstellar-communication:evidence-mismatch`.

## §10 well-known URI Discovery

A conformant operator exposes a discovery document at
`/.well-known/wia-interstellar-communication` that links to
the API root, the spectrum licence summary, the published
quality dossier, the post-detection affiliation reference,
and the catalogue of cleared candidates released to public
view.

## §11 Long-Term Archive Integration

Programmes designate a long-term archive that holds
observations and audit records beyond programme wind-down.
Many SETI-style programmes are decade-scale and produce
petabyte-scale observation archives; the long-term archive
carries the per-data-class retention policy and the content-
address preservation guarantee.

## §12 Verifiable-Credential Re-Issuance (optional)

Operators that wish to expose attestations (spectrum licence
status, post-detection affiliation, ISO/IEC 27001
certification) to consumers of W3C Verifiable Credentials
MAY re-issue the attestations as Verifiable Credentials
under the Data Model 2.0 specification. Re-issuance is
optional; the canonical record remains the JSON evidence-
package manifest.

## §13 Cross-Standard Linkage

Operators that consume adjacent WIA standards (WIA-radio-
astronomy for general radio-astronomy operations, WIA-
exoplanet-catalogue for the target list, WIA-mars-mission
or WIA-interplanetary-travel for probe-link continuity)
emit cross-standard linkage records that name the consuming
standard and the version under which the linkage is claimed.

## §14 Streaming Heartbeat

SSE subscribers receive a heartbeat every 30 seconds with
`Last-Event-ID` resume support. Subscribers that disconnect
during long observation windows resume from the last seen
event identifier without losing visibility of priority-1
events (significance-threshold candidate emissions, partner-
observatory follow-up confirmations, post-detection
escalations).

## §15 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with
prior-minor clients. Major revisions go through a
deprecation window of at least one full IVOA standard
release cycle so that VO-aligned consumers have time to
migrate.

## §16 Public Catalogue and Aggregator Feeds

Programmes publish a public catalogue of cleared candidates
and observation campaigns through an Atom or JSON Feed.
Pre-cleared candidates are NOT published on this feed; the
post-detection protocol governs candidate disclosure
(PHASE-3 §3).

## §17 Reader Tooling

Operators MAY publish supplementary reader tools (visual
candidate-detection diagnostic plots, RFI catalogue browsers,
target-catalogue maps) alongside the canonical evidence
package; the tools are non-normative.

## §18 Migration from Pre-Standard Records

Operators that operated SETI-style search programmes before
WIA-interstellar-communication reached version 1.0 MAY
migrate historical records by emitting synthetic observation
records with a `legacyImport` flag. Synthetic observations
are accepted by the public catalogue but cleared candidates
arising from synthetic observations require contemporaneous
re-validation against the operator's current pipeline before
post-detection escalation.

## §19 Cross-Observatory VLBI-Style Confirmation Integration

Cross-observatory candidate confirmation that uses VLBI-style
correlation between geographically-separated facilities
integrates through the partner-observatory channel of §1
plus the operator's correlation-service channel. The
integration carries the correlation-service identifier and
the per-correlation timing accuracy guarantee that the
service offers (sub-microsecond is typically required for
candidate-confirmation correlation).

## §20 Citizen-Science Platform Integration

Operators that engage citizen-science platforms (the SETI
@home heritage, Are We Alone In The Universe / SETILive,
Galaxy Zoo-style classification platforms) integrate through
the platform's public-API. Integration carries the platform's
identifier, the per-task data-distribution agreement, and the
crediting policy for citizen-science contributors.

## §21 Multi-Wavelength Counterpart Integration

For optical-SETI candidates and multi-wavelength campaigns,
operators integrate with sky-survey-archive providers (NASA
HEASARC, NED, SIMBAD, ESA Sky, equivalent national archives)
to retrieve known multi-wavelength counterpart information
for each target. Integration carries each archive's
identifier, the per-target query format, and the operator's
result-cache TTL so that downstream candidate-investigation
workflows can correlate the operator's candidate against
known catalogue counterparts efficiently.

## §22 Wide-Sky Survey Mission Integration

Wide-sky surveys (Gaia, LSST/Vera Rubin Observatory legacy,
SKA precursor surveys) emit transient detections that the
operator may correlate against. Integration carries the
survey's identifier, the per-survey transient-feed reference,
and the operator's coincidence-time-window for matching
survey transients to operator candidates.

## §23 Conformance and Sunset

A programme conformant with PHASE-4 has integrated
successfully with at least one partner observatory, the
post-detection adjudication network (where the operator
adheres), the operating jurisdiction's spectrum regulator,
at least one IVOA-aligned VO catalogue, and at least one
long-term archive, and has published at least one externally
citable evidence package.

Sunsetting an integration is announced via the well-known
discovery document at least 90 calendar days before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-interstellar-communication
- **Last Updated:** 2026-04-28
