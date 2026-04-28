# WIA-interplanetary-travel PHASE 3 — PROTOCOL Specification

**Standard:** WIA-interplanetary-travel
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an
interplanetary-travel programme: launch licensing under
domestic space activities law, COSPAR planetary-protection
category assignment, deep-space tracking network coordination,
trajectory design quality assurance, conjunction-mitigation
discipline, life-support consumable governance, crew radiation
health management for crewed missions, end-of-mission
disposal, and inter-agency cooperation.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO 14620-1 (system safety)
- ISO 24113:2023 (space debris mitigation)
- ISO 27852 (orbit lifetime estimation)
- ISO 27001:2022 (information security management)
- ISO 8601 (date and time)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8446 (TLS 1.3)
- IETF RFC 9457 (Problem Details)
- CCSDS 301.0-B / 503.0-B / 504.0-B / 505.0-B / 508.0-B
- COSPAR Planetary Protection Policy
- NASA NPR 8020.12 (Planetary Protection)
- ICRP Publication 132 (radiological protection from cosmic
  radiation in aviation, applied as a baseline)
- ITU-R RR Article 22 (deep-space telecommunications
  protection)
- IADC Space Debris Mitigation Guidelines

---

## §1 Launch Licensing

A mission operator MAY claim conformance to WIA-interplanetary-
travel only after the operating jurisdiction's space-
activities authority has issued a launch licence (FAA AST in
the US, UK Space Agency in the UK, KASA in Korea, JAXA via
MEXT in Japan, equivalent authorities elsewhere) for the
specific launch event(s) the mission requires. Licence
amendments are recorded; revocation freezes the mission at
its current status pending re-licensing.

## §2 COSPAR Planetary-Protection Category Assignment

Every mission carries a COSPAR planetary-protection category
(PHASE-1 §2 `cosparCategory`) per the COSPAR Planetary
Protection Policy, with the operator's planetary-protection
officer of record adjudicating the assignment in consultation
with the operating space agency's PP authority. Category
assignments follow the COSPAR matrix:

- Category I: missions to bodies of no direct interest to
  understanding chemical evolution or origin of life (Sun,
  Mercury, undifferentiated metamorphosed asteroids).
- Category II: missions to bodies where the contamination
  threat is significant interest but not high enough to
  require restrictive protocols (Venus, comets, most
  asteroids, Jupiter, Saturn, Uranus, Neptune).
- Category III / IVa-c / V: bodies of high interest where
  restrictive protocols apply (Mars in particular, with
  IVa-c sub-categories driving cleanroom and bioburden
  expectations).

Category changes require COSPAR-aligned re-review (PHASE-2 §3).

## §3 Deep-Space Tracking Network Coordination

Missions schedule deep-space tracking through one or more
networks: NASA Deep Space Network (DSN), ESA ESTRACK, JAXA
Usuda / Uchinoura, China DSN, commercial DSN providers. The
operator's tracking schedule is recorded against the mission
and reconciled against each network's allocation calendar.
Spectrum allocations follow ITU-R RR Article 22 protections
for deep-space radio.

Tracking outages (network-side or spacecraft-side) are
recorded as audit events; downstream consumers (science teams
expecting data, partner agencies relying on shared tracking)
receive notifications through the streaming subscription.

## §4 Trajectory Design Quality Assurance

Trajectory design follows the operator's QA framework:

- per-iteration peer review by trajectory designers
  independent of the iteration's primary author;
- propagation of orbit-state errors through the operator's
  Monte-Carlo navigation analysis;
- alignment with the SPICE toolkit and the IAU body-fixed
  reference-frame definitions;
- cross-check against the operating jurisdiction's primary
  ephemeris (JPL DE441 in the US, INPOP in Europe,
  equivalent elsewhere).

Per-iteration QA outcomes are recorded against each
trajectory; iterations that fail QA cannot advance past
`design` status to `frozen` for operations.

## §5 Conjunction-Mitigation Discipline

Conjunctions (PHASE-1 §7) are assessed against the operator's
chosen Pc model and the operating jurisdiction's conjunction-
response policy. Manoeuvre decisions follow:

- continuous monitoring while Pc remains below the operator's
  notification threshold;
- manoeuvre planning when Pc exceeds the notification
  threshold;
- manoeuvre execution when Pc exceeds the operator's red
  threshold and a manoeuvre is feasible within propellant
  reserves.

Decisions to execute, decline, or defer manoeuvres are
recorded with rationale; downstream consumers (the secondary
object's operator, the operating jurisdiction's debris
office) receive notifications through the agreed inter-agency
channel.

## §6 Life-Support Consumable Governance

Crewed missions operate under the operator's life-support
consumable policy: mission-design margins (typically 25-50%
above nominal use rates depending on mission phase),
contingency-event reserves (e.g. SPE storm shelter
provisioning, emergency Earth-return propulsion), and the
ECLSS closure-ratio assumptions on which the budget rests.

Consumable depletions that breach the design margin trigger
the operator's contingency-planning workflow: re-budgeting,
supply-mission planning where feasible, and crew-health
review.

## §7 Crew Radiation Health Management

Crewed missions operate under the operating space agency's
Radiation Health Office policy. The operator records:

- per-crew-member ionizing-radiation career limits (subject
  to the agency's evolving guidance, with ICRP Publication
  132 used as a baseline reference and the agency's deep-
  space-specific adaptations applied);
- per-mission cumulative-dose limits;
- the per-vehicle SPE storm-shelter design;
- the per-EVA radiation-monitoring procedure.

Radiation Ledger updates that approach a per-mission or
career limit trigger flight-surgeon review and may require
EVA postponement, transit-vehicle shelter occupancy, or
mission re-planning.

## §8 End-of-Mission Disposal

End-of-mission disposal follows ISO 24113:2023 and IADC
guidelines:

- bodies in COSPAR Category III/IV that the spacecraft has
  not been protected to land on require disposal trajectories
  that avoid impact within the COSPAR-defined exclusion
  window (typically 50 years for Mars);
- Earth-return vehicles for Category V-restricted missions
  follow the COSPAR-aligned containment protocol upon return;
- spacecraft remaining in heliocentric orbit follow the
  operating jurisdiction's debris-mitigation rules.

Disposal decisions are recorded against the mission and
referenced in the end-of-mission report.

## §9 Inter-Agency Cooperation

Multi-agency missions operate under the agencies' bilateral
or multilateral cooperation agreements (NASA-ESA, NASA-JAXA,
NASA-CNES, ESA-Roscosmos, etc.). The cooperation agreement
governs:

- per-agency deliverables and ownership;
- per-instrument data-rights and embargo periods;
- per-component planetary-protection responsibility;
- the joint mission-operations protocol.

## §10 Records Retention

Mission records — every record defined in PHASE-1, the API
audit logs, the regulator submissions, the COSPAR PP
documentation, the inter-agency cooperation artefacts —
retain indefinitely. Crew radiation-health records retain in
the agency's medical archives per the agency's medical-
records policy, typically for the crew member's lifetime
plus a documented post-decease period.

## §11 Time Synchronisation

Operator clocks synchronise per RFC 5905 (NTPv4) against the
operating jurisdiction's primary time reference. Mission
elapsed time, spacecraft event time, and ground time are
encoded per CCSDS 301.0-B; reconciliation between spacecraft
clock and ground clock is recorded continuously through the
deep-space tracking link.

## §12 Quality Dossier

The operator's quality dossier records the launch licences,
the COSPAR PP officer of record, the deep-space tracking
network bookings, the trajectory-design QA outcomes, the
conjunction history, the life-support governance reviews,
the radiation-health policy version, and the operator's
incident history. The dossier is reviewed at least annually
by the operator's chief safety officer.

## §13 Communications-Schedule Discipline

Communications schedules (PHASE-1 §9) are subject to the
operator's tracking-pass-priority hierarchy: anomaly
recovery and time-critical events take precedence over
nominal science downlink, which takes precedence over
opportunistic engineering passes. Conflicts between the
operator's requested cadence and a tracking network's
allocation are resolved through the network's published
priority procedure, with the operator's allocation request
documented at each step.

Missed station-side passes (network outage, weather hold,
tracker re-prioritisation) and missed spacecraft-side passes
(spacecraft-side anomaly, attitude excursion) are recorded
against the schedule with rationale.

## §14 Arrival Operations

Arrival operations (PHASE-1 §10) follow the operator's
arrival-rehearsal procedure: per-event end-to-end rehearsal
in a high-fidelity simulator, per-anomaly contingency-
playbook activation drill, and the per-event final-go review
at L-7 days, L-1 day, and the immediate go/no-go window.
Rehearsal outcomes are recorded against the arrival.

For EDL events, the operator records the predicted entry
state, the predicted descent profile, the predicted touchdown
target, and the post-event reconstructed state from observed
telemetry. Arrival outcomes drive the operator's downstream
operations posture: nominal arrivals advance to commissioning;
off-nominal arrivals enter recovery; loss events enter the
anomaly-investigation board described in §15.

## §15 Cross-Mission Resource Sharing

Interplanetary missions occasionally share resources with
contemporaneous missions: relay communications through
mission-A's orbiter for mission-B's surface asset (Mars
relay between MRO/MAVEN/TGO and surface rovers/landers is
a recurring example), shared instrument calibration with a
co-located science target, joint observation campaigns
during planetary opposition windows. Resource-sharing
arrangements are recorded in the mission's quality dossier
and referenced from each beneficiary mission's records so
that downstream consumers can resolve the shared-resource
provenance.

## §16 Public Engagement and Naming Conventions

Public engagement (mission name, instrument names, target
names) follows the IAU's naming conventions for
interplanetary objects (asteroids, comets, planetary
features) and the operator's internal naming-convention
policy for spacecraft and instruments. Public engagement
artefacts (press releases, mission-imagery captions, public
educational content) cite the IAU-confirmed names where
applicable.

## §17 Anomaly Investigation Discipline

Anomaly records (PHASE-1 §11) follow a structured workflow:
detection, triage by the operator's mission-anomaly-response
team, root-cause investigation by the responsible engineering
team, corrective-action planning, implementation, and
closure verification. Anomaly records reference the mission
artefact (trajectory iteration, consumable budget, ledger,
arrival event) at which the anomaly was observed so that
historical investigators can resolve the operating context.

Critical and loss-class anomalies follow the agency-level
anomaly-investigation board procedure (PHASE-4 §21);
findings flow back into the mission's audit chain and into
adjacent missions through the operator's lessons-learned
catalogue.

## §18 Conformance and Auditing

A mission conformant with WIA-interplanetary-travel publishes
its launch licence references, its COSPAR PP category, its
trajectory-design QA outcomes, the conjunction catalogue, and
the post-end-of-mission disposition record, and answers an
annual self-assessment that maps each clause of this PHASE
to the mission's implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-interplanetary-travel
- **Last Updated:** 2026-04-28
