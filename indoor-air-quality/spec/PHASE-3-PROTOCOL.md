# WIA-indoor-air-quality PHASE 3 — PROTOCOL Specification

**Standard:** WIA-indoor-air-quality
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an accredited
indoor-air-quality programme: sensor-package categorisation,
laboratory accreditation, ventilation-design baseline, sampling
strategy, occupant-engagement, episodic-sample chain of custody,
investigation conduct, remediation governance, post-action
verification, records retention, and public-health reporting.

References (CITATION-POLICY ALLOW only):

- ISO 16000 series (Indoor Air — sampling and test methods)
- ISO 7726:1998 (ergonomics — measurement of physical quantities)
- ISO 7730:2005 (thermal comfort)
- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 17043:2010 (proficiency testing)
- ISO/IEC 27001:2022 (information security management)
- ISO 9001:2015 (quality management systems)
- ISO 8601 (date and time)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8446 (TLS 1.3)
- IETF RFC 9457 (Problem Details)
- ASHRAE Standard 62.1 / 62.2 / 55
- WHO Guidelines for Indoor Air Quality

---

## §1 Sensor-Package Categorisation

Sensor packages are categorised in three tiers:

- **`consumer-grade`** — sensors with limited accuracy and absent
  traceability. Useful for trend awareness; not accepted as evidence
  for regulatory or ventilation-verification decisions.
- **`professional-grade-non-accredited`** — sensors with periodic
  calibration against traceable references, used by qualified
  practitioners. Accepted for trend records and as an investigation-
  triggering signal.
- **`accredited-laboratory-grade`** — sensors and methods operated
  under an ISO/IEC 17025-accredited scope. Accepted as evidence for
  any decision the standard supports.

The categorisation is recorded in the sensor-package register and is
verified at submission time per PHASE-2 §4.

## §2 Laboratory Accreditation

Episodic samples (PHASE-1 §5) are accepted only from laboratories
with current ISO/IEC 17025:2017 accreditation that covers the
methods used. The API verifies the accreditation against the
relevant national accreditation body's public register before
accepting the sample.

## §3 Ventilation-Design Baseline

Each site's ventilation design is documented against the relevant
edition of ASHRAE Standard 62.1 (commercial) or 62.2 (residential).
The baseline records the outdoor-air calculation, the assumed
occupant density, the assumed source emission factors, and the
control strategy. Re-design events emit new baselines; prior
baselines remain addressable as the historical state.

## §4 Sampling Strategy

Sites operating IAQ programmes adopt a sampling strategy
appropriate to their function and risk profile. The strategy
records the continuous-sample cadence, the episodic-sample cadence,
the suite of analytes covered, the laboratories used, and the
trigger conditions for additional sampling. Schools, childcare
facilities, and healthcare settings typically maintain a more
intensive cadence than office or warehouse environments.

## §5 Episodic-Sample Chain of Custody

Episodic samples follow a chain of custody from collection through
laboratory analysis and result publication. Each custody step
emits a record signed by the responsible party (collector, courier,
laboratory). The chain is auditable end-to-end so that downstream
consumers can verify that a published result is anchored to the
sample as collected.

## §6 Occupant Engagement

Occupant symptom feedback (PHASE-1 §7) is collected only with
explicit consent and only through the operator's CRM, which
mediates between occupants and the API. The CRM strips clinical
identifiers before submitting symptom records to the API and
records the consent reference so that subject-access requests can
be fulfilled. Aggregate trends (counts of symptom categories per
zone per period) are emitted on the operator's release schedule;
individual symptom records are not exposed externally.

## §7 Investigation Conduct

Source-identification investigations follow a hypothesis-driven
process: define the candidate sources, gather targeted evidence,
narrow the hypothesis set, identify the root cause, and propose
remediation. The investigation record (PHASE-1 §8) carries the
hypothesis log so that retrospective audits can verify that
candidate sources were considered before the conclusion was drawn.

## §8 Remediation Governance

Remediation actions are governed under change control: the
proposed action is reviewed against potential side effects (e.g.
increased outdoor-air intake during high pollen season may worsen
allergic symptoms), implemented under maintenance work orders, and
verified through post-action sampling. The verification's outcome
is published against the remediation record per PHASE-2 §8.

## §9 Inter-Laboratory Round-Robin

Programmes that publish externally cited episodic results
participate in inter-laboratory round-robin exercises at least
once every two calendar years per active method. The round-robin
protocol follows ISO/IEC 17043:2010 expectations.

## §10 Records Retention

Programme records — every record defined in PHASE-1, the API audit
logs, episodic-sample certificates, ventilation-verification
reports, investigations, and remediations — retain for a minimum
of seven calendar years from the last access of the site.
Externally cited records retain indefinitely.

## §11 Time Synchronisation

Sensor packages and laboratory clocks synchronise per RFC 5905
(NTPv4) so that sample-time correlations across zones, sensors, and
episodic samples are reliable.

## §12 Public-Health Reporting

Programmes that operate sites with public-health relevance (schools,
childcare, healthcare, mass-transit hubs) report aggregated IAQ
metrics to the relevant public-health authority on the cadence the
authority requires. The aggregated reports are produced from the
API's privacy-preserving aggregation endpoints (PHASE-2 §10) so
that individual occupant or zone identifiers are not exposed.

## §13 Programme Wind-Down

A programme that ceases operations transfers indefinite-retention
records to a recognised long-term archive and notifies known
external citers via the well-known discovery document.

## §14 Quality Dossier

The programme's quality dossier records the laboratories it works
with, the sampling strategy in force, the round-robin exercises
conducted, the public-health reporting commitments, and the
deprecation history of its IAQ findings. The dossier is reviewed
at least annually by the programme's quality manager.

## §15 Cross-Border Programme Operation

Programmes that operate across borders maintain a primary
jurisdiction of registration and operating MoUs with partner
jurisdictions. Cross-jurisdictional data transfers honour the
source-jurisdiction's data-protection law for occupant symptom
records.

## §16 Threshold Table Adoption

Programmes adopt a threshold table that is appropriate to their
operating jurisdiction and to the function of the sites they
operate. Sites with public-health relevance (schools, childcare,
healthcare) typically adopt the WHO Indoor Air Quality Guidelines
where they exist for the relevant pollutant; commercial offices
typically adopt the ASHRAE 62.1 outdoor-air calculation alongside
operator-internal thermal-comfort thresholds.

Operator-internal thresholds (basis = `operator-internal`) are
permitted but MUST be at least as protective as the
guidance-source thresholds; submissions that introduce a less-
protective threshold return `422` with type
`urn:wia:indoor-air-quality:threshold-less-protective`.

## §17 Demand-Controlled Ventilation Considerations

Sites that operate demand-controlled ventilation (DCV) modulate
outdoor-air intake based on occupancy or CO2 setpoints. The
operating organisation records the DCV control logic, the safety
floor on outdoor air (the minimum that DCV will permit even at
zero occupancy), and the verification that the DCV logic does not
silently violate ASHRAE 62.1 minimum outdoor-air requirements at
peak occupancy.

DCV anomalies (sensor drift causing chronically low outdoor-air
intake) are detected through the streaming subscription and
trigger remediation under §8.

## §18 Mould and Moisture Governance

Episodic mould and moisture investigations follow ISO 16000-19 for
impactor sampling and the operator's moisture-mitigation protocol
for source removal. The investigation record references both the
laboratory result and the moisture survey.

## §19 Pandemic-Response Considerations

Programmes that operated during respiratory-pandemic events
incorporate the lessons learned into their sampling strategy:
elevated outdoor-air intake during occupied hours, periodic
re-verification of HEPA-filter integrity in high-risk zones,
display of CO2 indicators visible to occupants, and integration of
zone-level occupancy data so that ventilation can be modulated to
match expected exposure.

The pandemic-response posture is recorded against the site so
that retrospective audits can distinguish business-as-usual from
elevated-response operations.

## §20 Climate-Adaptation Considerations

Sites in regions with seasonal smoke events, allergen surges, or
pollen seasons adopt climate-adaptation playbooks that switch the
outdoor-air strategy depending on outdoor air quality. The
playbook is recorded in the operator's quality dossier; activation
events emit records that link to the relevant outdoor air-quality
observations.

## §21 Sensor Drift and Recalibration

Continuous IAQ sensors drift over time as their reference cells age,
their flow paths foul, and their firmware updates introduce
calibration shifts. The operating organisation defines a
sensor-drift policy that bounds the acceptable drift between
recalibration events and that triggers recalibration when the
drift envelope is breached. The drift policy is recorded in the
operator's quality dossier and is enforced through the streaming
subscription so that observers see when a sensor's recorded drift
warrants re-acceptance of its recent submissions.

## §22 Filter Lifecycle Governance

Filter stages (PHASE-1 §3) follow lifecycle governance: scheduled
rotation cadence, post-rotation pressure-drop verification, and
end-of-life disposition. The lifecycle records are appended to the
filter-stage record and are exposed through the API so that
mechanical contractors and indoor-air auditors can verify that
filters are operated within their certified life.

## §23 Privacy of Occupant Symptom Records

Occupant symptom records (PHASE-1 §7) are personal data under the
operating jurisdiction's data-protection law. The operator records
the lawful basis (typically consent or legitimate interest under
GDPR-aligned regimes), the retention bound, and the data-subject
rights (access, correction, erasure). Erasure requests over
symptom records are honoured under the operator's subject-access
workflow, with the API removing free-text fields and substituting
a tombstone record so that aggregate counts remain consistent.

## §24 Conformance and Auditing

A programme conformant with WIA-indoor-air-quality publishes its
laboratory accreditation references, its programme code
registration, its quality dossier, and the catalogue of IAQ
findings it has released, and answers an annual self-assessment
that maps each clause of this PHASE to the programme's
implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-indoor-air-quality
- **Last Updated:** 2026-04-27
