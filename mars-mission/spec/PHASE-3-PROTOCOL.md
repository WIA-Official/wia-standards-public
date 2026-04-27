# WIA-mars-mission PHASE 3 — PROTOCOL Specification

**Standard:** WIA-mars-mission
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an accredited Mars-
mission programme: spectrum coordination, mission-operations centre
accreditation, navigation reproducibility, planetary-protection
governance, ground-station coordination, anomaly investigation,
science-data release policy, deep-space cybersecurity, archive
deposit, and post-mission disposition. The PROTOCOL layer binds the
data shapes of PHASE-1 and the API contract of PHASE-2 to the
international and inter-agency frameworks under which Mars missions
operate.

References (CITATION-POLICY ALLOW only):

- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 27001:2022 (information security management)
- ISO 9001:2015 (quality management systems)
- ISO 8601 (date and time)
- ITU Radio Regulations (deep-space band allocations)
- COSPAR Planetary Protection Policy
- CCSDS 132.0-B / 232.0-B (Space Data Link Protocols)
- CCSDS 301.0-B (Time Code Formats)
- CCSDS 633.0-B (Mission Operations Services)
- CCSDS 727.0-B (CFDP)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8446 (TLS 1.3)
- IETF RFC 9457 (Problem Details)

---

## §1 Spectrum Coordination

Every mission operates within ITU-R deep-space band allocations
appropriate to its phase. The mission's spectrum file records the
frequency assignments, the licensing authority, the bandwidth
allocations, and the coordination agreements with ground stations
that will support the mission. Spectrum-file revisions emit new
records; prior records remain addressable for archival continuity.

A mission whose spectrum file conflicts with a co-flying mission's
spectrum file MUST resolve the conflict before the API accepts an
observation request that would exercise the contested allocation;
the conflict-resolution mechanism is the inter-agency coordination
forum that the participating agencies recognise.

## §2 Mission-Operations Centre Accreditation

A Mission Operations Centre (MOC) MAY claim conformance to
WIA-mars-mission only after a recognised accreditation body has
issued a valid certificate against ISO 9001:2015 and an
information-security certificate against ISO/IEC 27001:2022 covering
the scopes the MOC exercises (TT&C, navigation, surface-ops planning,
science-ops planning). The accreditation register is exposed to the
API as a read-only resource.

## §3 Navigation Reproducibility

Trajectory and state products (PHASE-1 §4) are reproducible when an
independent navigation team using the same observation arc, the same
gravity-field model, the same atmospheric drag model where
applicable, and the same numerical-integrator settings produces a
state vector whose components agree within the published
uncertainty.

Products MUST carry their generation process and the version of the
process so that downstream consumers can reproduce or revise the
estimate. Products produced under a deprecated process remain
addressable but are not recommended for new operational decisions.

## §4 Planetary Protection Governance

Planetary-protection records (PHASE-1 §8) are governed by the
COSPAR Planetary Protection Policy. The policy categorises mission
targets and allowed activities; the operating mission's planetary-
protection officer maintains the bioburden inventory, approves
activities at sensitive targets, and issues the records that the API
consumes.

Activities at "special regions" (per COSPAR) require additional
approval cycles that the operating programme records as a chain of
attestations against the mission's planetary-protection record.

## §5 Ground-Station Coordination

TT&C packets traverse ground-station networks (the Deep Space
Network, ESTRACK, the Chinese Deep Space Network, commercial
deep-space providers). Ground-station coordination records carry
the booked tracking passes, the actual passes that took place, and
the per-pass quality metrics (carrier signal-to-noise ratio,
ranging precision, command-link availability). Coordination records
are signed by the providing agency.

## §6 Anomaly Investigation

Anomalies — spacecraft safe modes, instrument fault detections, lost
contact, unexpected science observations — trigger an anomaly
investigation. The investigation record carries the timeline of the
anomaly, the controlling team's response, the recovery actions, and
the lessons-learned write-up that re-enters the programme's quality
dossier.

Anomaly records that affect mission-critical activities (EDL,
sample acquisition, sample return) propagate to inter-agency review
boards that the participating agencies have established for the
mission.

## §7 Science-Data Release Policy

Science products (PHASE-1 §7) move through a release sequence
agreed at mission inception. Common sequences include immediate
release for press-relevant Level-1 products, a calibration window
during which Level-1b and Level-2 products remain restricted to
the science working group, and a public release at the end of the
calibration window. The release schedule is recorded against each
mission and is exposed as part of the well-known discovery document
(PHASE-4 §5).

## §8 Deep-Space Cybersecurity

Command links to deep-space spacecraft are authenticated and
integrity-protected. The mission's cybersecurity protocol records
the authentication scheme (typically a CCSDS-aligned authentication
profile), the key management lifecycle, and the contingency
procedures for compromise. Compromise events trigger an immediate
re-key and a parallel anomaly-investigation cycle.

## §9 Records Retention

Mission records retain indefinitely. The default archival period for
science products is "for the foreseeable lifetime of the host
discipline"; in practice, programmes deposit science products at
PDS-aligned archives that operate under multi-decade preservation
commitments. TT&C catalogues, planetary-protection records, and
anomaly investigations follow the same indefinite-retention regime.

## §10 Time Synchronisation

Ground-system clocks synchronise per RFC 5905 (NTPv4) against
national-metrological-laboratory stratum-1 services. Spacecraft-
event time is encoded per CCSDS 301.0-B and reconciled against
ground time during routine ranging passes.

## §11 Mission Wind-Down

A mission that ends operations transitions all open observation
requests to `cancelled`, deposits the final science product set at
the chosen archives, publishes a final mission report, and notifies
inter-agency partners. Spacecraft disposal — controlled deorbit,
graveyard-orbit insertion, surface decommissioning — follows the
disposal plan negotiated at mission inception.

## §12 Quality Dossier

The programme's quality dossier records the agencies it partners
with, the ground stations it uses, the planetary-protection
category it operates under, the anomaly investigations it has
conducted, and the release-policy compliance for each science
product. The dossier is reviewed at least annually by the
programme's quality manager.

## §13 Cross-Border Programme Operation

Mars missions are inherently international. The programme maintains
a primary jurisdiction of registration and operational MoUs with
partner jurisdictions. Cross-jurisdictional data transfers honour
the dual-use export controls of each participating jurisdiction;
the operating programme records the dual-use determination for each
exported record class.

## §14 Sol Activity Plan Approval Chain

Sol activity plans (PHASE-1 §10) move through a multi-stage approval
chain: science representative, mission engineer, planetary-
protection officer (where applicable), and the operations director.
Each stage records its approval against the plan; missing approvals
prevent uplink. The chain is configured at mission inception and is
exposed in the well-known discovery document for partner agency
visibility.

A plan that has been uplinked is locked: subsequent edits emit a new
plan referencing the prior plan as predecessor, and the prior plan
remains addressable as the historical record of the sol's
operations.

## §15 Sample-Return Quarantine

Sample-return missions enforce a quarantine protocol at Earth
recovery and during initial curation. The quarantine protocol is
governed by the operating jurisdiction's biocontainment authority
and is documented as a chain of records that this standard exposes
in §11 of PHASE-1. The protocol's release criteria — when curated
samples may move from the quarantine facility to general scientific
distribution — are recorded against the mission and are signed by
the responsible authority.

## §16 Crewed-Mission Coordination

For crewed missions, the operating programme coordinates with the
crew-medical, life-support, and EVA standards described in adjacent
WIA standards. Coordination is record-level only; the mars-mission
standard does not duplicate medical or life-support content.
Cross-references are recorded against the mission and exposed in
the evidence package's cross-standard linkage section (PHASE-4 §13).

## §17 Environmental Data Reuse

Environmental observations (PHASE-1 §12) are reused by partner
missions, by climate-modelling teams, and by future missions
planning surface activities. The operating mission MUST publish the
environmental data set on the mission's release schedule and MUST
flag observations that are subject to embargo (a period of
exclusive access for the originating instrument team).

Partner missions consume environmental data through dedicated
client certificates that are bound to the partner's mission record;
the API verifies the partner's authorisation against the
collaboration agreement before serving environmental queries.

## §18 Mission-Critical-Activity Reviews

Mission-critical activities (entry-descent-and-landing, primary
sample acquisition, sample return, end-of-mission disposition) are
reviewed by an inter-agency review board before execution. The
review board's findings are recorded against the mission as a
chain of attestations. Activities that proceed without a positive
review board outcome are flagged in the public catalogue and are
not externally citable as nominal operations.

## §19 Mission-Lifecycle Review Cadence

Mission programmes hold formal lifecycle reviews at major milestones
(preliminary design, critical design, launch readiness, surface-
operations transition, end-of-mission). Each review's outcome is
recorded as a content-addressed report against the mission and is
exposed under the well-known discovery document so that partner
agencies and downstream archives can resolve the review chain
without bespoke requests.

## §20 Embargo Policy for Pre-Release Products

Some science products carry an embargo window during which the
originating science team retains exclusive analytical access. The
embargo period is recorded against the science product and is
honoured by the API: requests for embargoed products from
non-originating teams return `403 Forbidden` with type
`urn:wia:mars-mission:embargo-window-active`. The embargo expires
automatically at the recorded date and the product transitions to
the agreed public-release path.

## §21 Conformance and Auditing

A programme conformant with WIA-mars-mission publishes its
accreditation certificate, its mission registration, its quality
dossier, and the catalogue of science products it has released, and
answers an annual self-assessment that maps each clause of this
PHASE to the programme's implementation. The self-assessment is
reviewed during the annual ISO 9001 / ISO 27001 surveillance audit
appropriate to the operating MOC.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-mars-mission
- **Last Updated:** 2026-04-27
