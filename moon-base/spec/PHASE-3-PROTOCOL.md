# WIA-moon-base PHASE 3 — PROTOCOL Specification

**Standard:** WIA-moon-base
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an accredited
moon-base programme: site-coordination among neighbouring missions,
crew safety frameworks, life-support and consumables governance,
EVA authorisation, ISRU operating regimes, power-budget governance,
surface-network spectrum coordination, supply chain and crew
rotation, anomaly investigation, planetary protection (Earth-bound
contamination), and base wind-down. The PROTOCOL layer binds the
data shapes of PHASE-1 and the API contract of PHASE-2 to the
inter-agency frameworks under which moon bases operate.

References (CITATION-POLICY ALLOW only):

- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 27001:2022 (information security management)
- ISO 9001:2015 (quality management systems)
- ISO 8601 (date and time)
- ITU Radio Regulations (lunar surface and cislunar band allocations)
- COSPAR Planetary Protection Policy (Category II / VI for Earth-
  bound contamination paths)
- CCSDS 132.0-B / 232.0-B (Space Data Link Protocols for the relay
  link)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8446 (TLS 1.3)
- IETF RFC 9457 (Problem Details)

---

## §1 Site Coordination

A moon-base programme registers its landing site, exclusion zones,
and operating envelopes with the inter-agency coordination forum
that the participating agencies recognise. Registration captures
expected dust-plume directions during landing operations, regions
of scientific interest the base will respect, and the boundaries
beyond which the base will not extend infrastructure without
further coordination.

Neighbouring missions consume the registration through dedicated
client certificates and emit acknowledgements that the API records
against the base. Conflicts (overlap with another mission's
exclusion zone, intrusion into a heritage site) are resolved through
the coordination forum before the API accepts a configuration that
would extend into the contested area.

## §2 Crew Safety Framework

A moon-base hosting crewed operations operates under a crew safety
framework that names the operating organisation, the crew-medical
authority, the safety officer of record, and the contingency plans
for emergencies (depressurisation, fire, life-support failure,
medical emergency, lost EVA crew). The framework is reviewed at
least annually and is referenced from every EVA record (PHASE-1 §8)
and every life-support telemetry alert.

## §3 Life-Support and Consumables Governance

Life-support consumables (oxygen, water, CO2 scrubber capacity,
nitrogen for atmospheric balance, food) are governed under the
operating organisation's logistics protocol. The protocol records
the consumption-rate model the operating organisation uses, the
re-supply cadence the base depends on, and the contingency
consumables window that the base maintains for unplanned
re-supply delays.

Programmes MUST refuse to advance a base from `commissioning` to
`occupied` until the contingency consumables window meets the
operating organisation's agreed minimum (typically 90 to 180 days
of full crew loading). The API enforces this via the phase-
transition endpoint with type
`urn:wia:moon-base:contingency-window-insufficient`.

## §4 EVA Authorisation

Every EVA is authorised by the surface-operations director, the
crew-medical officer, and the EVA officer prior to airlock egress.
Authorisation records are signed by the responsible officers'
client certificates and are attached to the EVA record. EVAs
executed under emergency conditions (immediate-life-threat
contingency) follow an abbreviated authorisation that the
operating organisation has pre-approved; emergency EVAs are flagged
in the record so that retrospective review captures the abbreviated
path.

## §5 ISRU Operating Regimes

ISRU plants operate in regimes appropriate to the surface
environment and the power budget. The plant's operating regime is
recorded in its quality dossier and is approved by the surface
operations director. Regime transitions (e.g. from
`hydrogen-reduction` to `molten-regolith-electrolysis`) require
re-approval and a parallel update to the consumables-input model.

## §6 Power-Budget Governance

The base's power microgrid is governed under a budget that names
generation capacity, storage capacity, allowed loads in each
microgrid state (normal, load-shedding, islanded, emergency), and
the load-shedding priority. The budget is exposed as a
content-addressed bundle in the base's quality dossier.

Programmes MUST refuse to commission a microgrid configuration
whose load-shedding priority does not preserve life-support and
EVA-recovery loads under all generation-shortfall scenarios.

## §7 Surface Spectrum Coordination

Surface communications operate under ITU Radio Regulations
allocations for lunar surface and cislunar links. The base's
spectrum file records the allocations, the licensing authority,
and the coordination agreements with neighbouring operations and
the relay-network operator.

## §8 Supply Chain and Crew Rotation

Cargo supply and crew rotation flow through the supply-chain
protocol the operating organisation maintains. The protocol records
the launch cadence, the manifest of every supply launch, the
inventory delta on every cargo arrival, and the crew-rotation
schedule. The API exposes the supply chain as a series of records
that link to the on-base consumables inventory.

## §9 Anomaly Investigation

Base-anomaly investigations follow the same record shape as
mars-mission anomaly investigations (see WIA-mars-mission PHASE-3).
Anomalies that affect life-support, EVA, or supply-chain operations
escalate through inter-agency review boards.

## §10 Earth-Bound Contamination Control

Surface samples returning to Earth follow the COSPAR Planetary
Protection Policy's Earth-bound categorisation (Category II for
Moon material under current science consensus, Category VI for
samples that derive from environments where the policy applies
otherwise). The base records the relevant approvals against each
sample export.

## §11 Records Retention

Base records — every record defined in PHASE-1, the API audit
logs, life-support telemetry archives, EVA records, and anomaly
investigations — retain indefinitely. Externally cited base records
are deposited in a long-term archive on a quarterly cadence.

## §12 Time Synchronisation

Surface clocks synchronise per RFC 5905 (NTPv4) against the relay
network's reference time service or, in disconnected operation, an
on-base GPS-like surface positioning service. Time-of-record on
all telemetry is recorded with the synchronisation source so that
ground operations can verify that the recorded time is anchored.

## §13 Cybersecurity

Surface network connections are authenticated and integrity-
protected per TLS 1.3 (RFC 8446) where the surface-network
infrastructure supports IP-style protocols, and via CCSDS-aligned
authentication profiles for the cislunar relay link. Compromise
events trigger an immediate re-key and a parallel anomaly
investigation.

## §14 Programme Wind-Down

A base that ends operations transitions to `unoccupied-caretaker`
or `decommissioned`. Caretaker bases retain monitoring telemetry
under reduced crew loading; decommissioned bases follow the
disposal plan negotiated at base inception (cap, abandon-in-place,
deconstruct).

## §15 Quality Dossier

The programme's quality dossier records the agencies it partners
with, the supply-chain providers it uses, the crew-medical
authority, the spectrum file revisions, and the anomaly
investigations conducted. The dossier is reviewed at least annually
by the programme's quality manager.

## §16 Cross-Border Programme Operation

Moon bases are inherently international. The programme maintains a
primary jurisdiction of registration and operational MoUs with
partner jurisdictions. Cross-jurisdictional data transfers honour
the dual-use export controls of each participating jurisdiction.

## §17 Heritage and Exclusion Zone Governance

Heritage sites (prior crewed-mission landing zones, scientifically
reserved areas) and active-operations exclusion zones are governed
through a registry that the operating organisation maintains and
that the inter-agency coordination forum acknowledges. Zone
boundaries are content-addressed; revisions emit new records and
prior records remain addressable as the historical regulatory
state.

A traverse plan that breaches an exclusion zone returns a Problem-
Details (RFC 9457) response of type
`urn:wia:moon-base:exclusion-breach` from the mobility task-queue
endpoint. Emergency-services breaches (response to an EVA medical
event, fire suppression, life-support recovery) follow a pre-
authorised emergency exception that the operating organisation has
agreed to with the inter-agency forum.

## §18 Crew-Health Continuity

Crew health observations are captured by adjacent crew-medical
standards. The moon-base programme records the integration point
with the crew-medical standard, the consent regime under which crew
data flows from the base to the crew-medical operations centre, and
the contingency procedures when the relay link to the crew-medical
operations centre is unavailable.

## §19 Long-Term Habitability Studies

Bases that operate over long durations participate in habitability
studies that aggregate atmospheric, radiation-environment, and
psychosocial observations across multiple operating intervals. The
study coordinator is the inter-agency forum or a body the forum
nominates. The study's protocol is content-addressed and is
referenced by the participating bases.

## §20 Cislunar-Relay Failover

The cislunar relay network is a single point of failure for many
base operations; the operating organisation maintains a failover
protocol that defines the base's degraded-operation mode when the
relay is unavailable. The failover protocol limits non-essential
loads, prioritises consumables conservation, and records all
activity locally for transmission once the relay returns.

## §21 Radiation Exposure Governance

Crew radiation exposure is governed under the operating
organisation's exposure policy. The policy specifies cumulative
limits, time-window limits, and emergency-shelter triggers in the
event of a solar particle event. Crew-dose ledger entries are
audited at every crew-rotation event and at every major
mission-critical-activity review.

## §22 Solar Particle Event Response

Solar particle events trigger a graduated response: monitoring,
non-essential-load reduction, EVA suspension, and full retreat to
deep-shelter pressurised volumes. The response is documented in the
operating organisation's safety framework and is exercised at
least annually under simulated conditions.

## §23 EVA Suit Telemetry Custody

EVA suit telemetry (suit pressure, temperature, consumables status,
biomedical observations) is captured by the EVA crew member's suit
and uploaded to the base when the crew member returns to the
airlock. The custody chain from the suit's on-board store to the
base's archive is recorded with the suit identifier, the upload
event, and the integrity check that the upload performed.

Suit-telemetry archives that fail integrity check are quarantined
pending forensic review; the API exposes the quarantine state via
the streaming endpoint so that ground teams know when telemetry
gaps reflect a data-integrity issue rather than a sensor failure.

## §24 Cryogenic Volatile Handling

Bases sited near permanently shadowed regions handle cryogenic
volatiles (water ice, methane) extracted by ISRU operations. The
operating organisation maintains a cryogenic-handling protocol that
specifies thermal-management constraints, pressure-vessel ratings,
and the contamination controls that prevent cross-mixing between
volatile feedstocks. The protocol is reviewed at least annually and
is referenced from the ISRU plant record.

## §25 Conformance and Auditing

A programme conformant with WIA-moon-base publishes its
accreditation certificate, its base registration, its quality
dossier, and the catalogue of base records it has released, and
answers an annual self-assessment that maps each clause of this
PHASE to the programme's implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-moon-base
- **Last Updated:** 2026-04-27
