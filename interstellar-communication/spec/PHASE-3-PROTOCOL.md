# WIA-interstellar-communication PHASE 3 — PROTOCOL Specification

**Standard:** WIA-interstellar-communication
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an
interstellar-communication operator: spectrum licensing under
the operating jurisdiction's radio regulations, ITU-R RA.769
radio-astronomy protection observance, post-detection
protocol adherence, deliberate-transmission ("active SETI" /
"METI") governance, signal-detection pipeline qualification,
RFI catalogue stewardship, precursor-probe link operations,
and recordkeeping.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO 19115-1 (geographic information — metadata)
- ISO/IEC 27001:2022 (information security management)
- ISO 8601 (date and time)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8446 (TLS 1.3)
- IETF RFC 9457 (Problem Details)
- ITU-R RA.769 (protection criteria for radio-astronomy
  measurements)
- ITU-R RA.1513 (loss of data due to interference)
- ITU-R RR Article 29 (radio astronomy service)
- IAU Standards of Fundamental Astronomy (SOFA)
- IAU Resolution B1 (XXVII GA, 2009) on radio frequency
  protection
- CCSDS 301.0-B (Time Code Formats)
- IVOA ObsCore / ObsTAP

---

## §1 Spectrum Licensing

An operator MAY claim conformance to WIA-interstellar-
communication only after the operating jurisdiction's radio
regulator has issued a valid licence (FCC OET in the US,
Ofcom in the UK, KCC in Korea, MIC in Japan, equivalent
authorities elsewhere) for the operator's reception bands and,
where the operator transmits, for the operator's transmission
bands. Licences are recorded against the programme; revocation
freezes the programme at its current status pending
re-licensing.

## §2 ITU-R RA.769 Observance

Radio-astronomy programmes operating in the ITU-R RA.769
protected bands (1400-1427 MHz, 1610.6-1613.8 MHz, 22.21-
22.5 GHz, others) honour the recommendation's interference
threshold and report observed interference excursions to
the operating jurisdiction's RA.1513 reporting channel.

Operators that observe outside the protected bands accept the
operating environment's natural interference floor and
reflect this in the per-pipeline RFI tagging.

## §3 Post-Detection Protocol Adherence

Operators that adhere to the IAA SETI Permanent Committee
post-detection protocol record their adherence in the
`postDetectionAffiliationRef` field (PHASE-1 §2). The
protocol prescribes:

- the operator does not announce a candidate detection
  publicly until follow-up observations have eliminated all
  reasonable terrestrial-origin and natural-source
  explanations;
- candidate-claim adjudication flows through the operator's
  post-detection adjudication network (collaborating
  observatories that perform independent re-observation);
- formal community announcement, when warranted, is
  coordinated through the IAA SETI Permanent Committee
  rather than as a unilateral operator press release.

Operators that elect not to adhere to the protocol document
their alternative governance in the quality dossier; this
standard does not impose adherence as a precondition for
conformance, but the registry consumer reads the
`postDetectionAffiliationRef` field to assess the operator's
adjudication discipline.

## §4 Reproducibility-State Lifecycle

Candidate-signal reproducibility states (PHASE-1 §5
`reproducibilityState`) advance under the following discipline:

- `unverified` is the initial state for a fresh candidate.
- `reproduced-on-source` requires at least one independent
  re-observation of the same target showing the candidate's
  signal at a consistent (or systematically-explained-shifted)
  frequency.
- `rfi-classified` requires positive identification of the
  RFI source through the operator's RFI investigation
  procedure.
- `natural-source-classified` requires positive identification
  of a natural astrophysical source (pulsar, fast radio
  burst, atmospheric emission, etc.) consistent with the
  candidate's signature.
- `follow-up-pending` is the state for candidates that
  warrant additional observations before classification.
- `post-detection-escalated` is the terminal state for
  candidates that have survived the operator's reproduction
  and source-elimination workflow and have entered the
  post-detection adjudication network.

State transitions backwards (e.g. `rfi-classified` to
`unverified`) require analyst-authorisation and are audited.

## §5 Reproduction Discipline

Reproduction observations follow these conventions:

- the same target re-observed within a window short enough
  that any putative source's frequency drift is within
  the operator's pipeline's drift-rate tolerance;
- a control off-source pointing to confirm the signal is
  source-fixed rather than facility-internal;
- re-observation by an independent facility (different
  receiver, ideally different geographic location) to
  rule out facility-localised interference.

Each reproduction observation is recorded against the
candidate's audit log so that the post-detection adjudication
network can review the full reproduction chain.

## §6 Deliberate-Transmission Governance

Any operator-proposed deliberate transmission goes through
the operator's deliberate-transmission governance review.
The review covers:

- the proposed transmission's content and rationale;
- the proposed target and the rationale for selecting that
  target (no transmission to inhabited-system candidates
  without dedicated multi-discipline review);
- the proposed transmitter's EIRP and the per-jurisdiction
  spectrum compliance;
- input from the operator's domain-expertise advisory panel
  (astronomy, exobiology, ethics, communications law, public
  policy);
- the operator's published consultation outcomes where the
  jurisdiction expects public consultation.

Approval requires a recorded review outcome of `approved`
that the API verifies before allowing the transmission to
advance to executed status (PHASE-2 §9).

## §7 Signal-Detection Pipeline Qualification

Signal-detection pipelines (TURBO_SETI, BL-SCRUNCH,
proprietary observatory pipelines, ML-based novel-pipeline
candidates) are qualified per programme. Qualification
requires:

- documented behaviour against representative synthetic-
  signal injection tests;
- documented behaviour against the programme's RFI
  catalogue;
- per-pipeline significance-metric definition and an
  agreed escalation threshold;
- conformance attestation from the operator's qualified
  pipeline reviewer.

Pipelines that lose qualification are removed from the
programme; candidates produced by removed pipelines remain
addressable for audit but are not eligible for follow-up
allocation.

## §8 RFI Catalogue Stewardship

The RFI catalogue (PHASE-1 §6) is maintained continuously by
the programme's RFI analyst. New RFI sources discovered
through candidate investigations are catalogued; cataloguing
is bidirectional with the operating jurisdiction's
radio-astronomy coordination forum (where one exists),
enabling cross-observatory awareness.

## §9 Precursor-Probe Link Operations

Precursor-probe links (PHASE-1 §7) are operated by the probe
mission's MOC. WIA-interstellar-communication consumers
ingest the per-link record so that interstellar-precursor
science products (interstellar plasma measurements,
interstellar dust impact rates, cosmic-ray spectra in the
heliosheath and beyond) are catalogued in the WIA registry
alongside the operator's search programmes.

## §10 Records Retention

Programme records — every observation, every candidate at
every reproducibility state, every RFI entry, every probe-
telemetry link, every deliberate-transmission record, and
every API audit log — retain indefinitely. Cleared
candidates remain addressable for retrospective review (a
candidate cleared today as RFI may be re-examined years
later if the operator's pipeline improves).

## §11 Time Synchronisation

Operator clocks synchronise per RFC 5905 (NTPv4) against the
operating jurisdiction's primary time reference, with sub-
microsecond accuracy required for VLBI-style cross-
observatory candidate confirmation. Probe-telemetry link
records use CCSDS 301.0-B time codes for compatibility with
the operator's deep-space mission ledger.

## §12 Cross-Observatory Coordination

Operators that share targets, candidates, or RFI entries
across observatories follow coordination conventions:

- shared-target announcements through the operator's partner-
  observatory channel so that follow-up scheduling can be
  coordinated;
- shared-candidate sharing only after the candidate reaches
  `follow-up-pending` status (sharing pre-`follow-up-pending`
  raw candidates would generate noise floods at partner
  pipelines);
- shared-RFI entries flow through the operating jurisdiction's
  radio-astronomy coordination forum where one exists.

## §13 Public Engagement

Public engagement (popular-science articles, observatory press
releases, public talks) for cleared candidates follows the
post-detection protocol's media coordination guidance.
Operators do not announce putative detections in advance of
the protocol's coordinated announcement window; pre-
coordinated leaks are recorded as protocol-non-compliance
events that the operator's quality manager investigates.

## §14 Privacy and Researcher Identity

Researchers contributing to the operator's programme are
identified through the operator's IDP. The DATA-FORMAT layer
carries operator-internal staff identifiers only as opaque
analyst tokens; researcher publications and credit
attributions flow through the operator's publication-
credit workflow rather than through this API.

## §15 Campaign-Level Discipline

Multi-observation campaigns (PHASE-1 §10) are subject to
campaign-design QA: the campaign's scientific rationale, the
target-selection methodology, the per-observation cadence,
and the campaign's expected sensitivity envelope are reviewed
by the operator's science committee before the campaign
launches. Campaigns that deviate materially from the approved
design (changes to target list, observation cadence, band
selection) re-enter the science committee's review queue.

## §16 Synthetic-Injection Audit

Synthetic-signal injection records (PHASE-1 §9) are reviewed
quarterly by the operator's pipeline-QA analyst. Recovery
rates that drift below the operator's documented thresholds
trigger pipeline re-qualification (PHASE-3 §7); false-
positive rates above the operator's threshold trigger an
RFI-catalogue refresh and a candidate-significance threshold
review.

## §17 Quality Dossier

The operator's quality dossier records the spectrum licence,
the post-detection affiliation reference, the qualified
pipeline register, the partner-observatory list, the science
committee composition, the per-pipeline RFI catalogue
maintainer, and the operator's incident history. The dossier
is reviewed at least annually by the operator's quality
manager.

## §18 Conformance and Auditing

A programme conformant with WIA-interstellar-communication
publishes its spectrum licence, its post-detection
affiliation, the catalogue of qualified pipelines, the
catalogue of cleared candidates, and the catalogue of any
deliberate transmissions, and answers an annual self-
assessment that maps each clause of this PHASE to the
operator's implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-interstellar-communication
- **Last Updated:** 2026-04-28
