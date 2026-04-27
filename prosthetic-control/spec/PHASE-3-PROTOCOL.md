# WIA-prosthetic-control PHASE 3 — PROTOCOL Specification

**Standard:** WIA-prosthetic-control
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern the operating
procedure of an accredited prosthetic-control programme: device
classification under the relevant medical-device regulation, clinical
fitting and follow-up cadence, calibration drift management,
clinical-trial conduct for novel control modes, post-market
surveillance, adverse-event escalation, and device wind-down. The
PROTOCOL layer binds the data shapes of PHASE-1 and the API contract
of PHASE-2 to the medical-device regulatory environment under which
prosthetic-control implementations operate.

References (CITATION-POLICY ALLOW only):

- ISO 13485:2016 (medical devices — quality management systems)
- ISO 14971:2019 (medical devices — application of risk management)
- ISO 14155:2020 (clinical investigation of medical devices)
- ISO 22523:2006 (external limb prostheses — requirements and tests)
- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27701:2019 (privacy information management)
- ISO 9001:2015 (quality management systems)
- IEC 60601-1 (medical electrical equipment — basic safety)
- IEC 62366-1 (usability engineering for medical devices)
- IEC 62304 (medical device software life cycle)
- ISO 8601 (date and time)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8446 (TLS 1.3)
- IETF RFC 9457 (Problem Details)

---

## §1 Device Classification

A prosthetic-control implementation is regulated as a medical device
under the relevant national framework. The programme records the
device's classification (typically Class IIa or Class IIb under EU
MDR-equivalent frameworks; an equivalent risk class under other
frameworks) and the notified body or competent authority that
performed the conformity assessment. Classification is recorded
against the device record and referenced from every subject record.

A device that integrates implantable components (intramuscular EMG
electrodes, peripheral-nerve stimulators, intracortical or epidural BCI
arrays) carries the implantable component's separate classification and
the surgical-team consent requirements alongside the device classification.

## §2 Clinical Fitting

Fitting is conducted by a certified prosthetist working with a
clinically responsible physician. The fitting visit captures the
informed consent of the user, the donning and doffing protocol, the
electrode-placement reference (PHASE-1 §3 channel.electrodeMapRef),
the initial calibration (PHASE-1 §6), the prescribed
follow-up schedule, and the user's performance baseline (e.g.
Box-and-Block test for upper-limb, 6-minute walk for lower-limb).

Implementations MUST refuse to publish a subject as `active` until
fitting fields are complete; partial fittings are persisted as `draft`
subjects and are excluded from analytics queries.

## §3 Calibration Drift and Recalibration

Surface-EMG signals drift over time as electrodes shift, skin
conductance changes, and the residual limb's volume varies. Pattern-
recognition decoders consequently degrade unless they are recalibrated
or adapted online. The programme defines a calibration-drift policy
that bounds the acceptable performance loss before recalibration is
required.

Implementations that adapt online (decoder.retrainPolicy =
"incremental-online" in PHASE-1 §4) MUST emit an adaptation log per
PHASE-2 §5 and MUST flag the subject for clinical follow-up when the
adaptation magnitude exceeds the configured drift envelope; flagged
subjects appear in the clinic's worklist and are not excluded from
follow-up by virtue of the device having self-adapted.

## §4 Clinical Investigations

Novel control modes (a new BCI front-end, an unproven peripheral-nerve
stimulator, a new pattern-recognition class) MUST be evaluated under
ISO 14155-aligned clinical investigations before they are released to
unrestricted clinical use. The investigation record carries the
protocol, the ethics-committee approval reference, the participating
sites, and the publication identifier of the investigation result.

Investigation participants are tracked under a separate
`investigationCohort` cohort code in the API; data from
investigation cohorts is not aggregated with post-market surveillance
data unless the investigation is explicitly extended to the
post-market period under a regulator-approved protocol.

## §5 Stimulation-Safety Envelope

Stimulation-based feedback channels (TENS, peripheral-nerve
stimulation) operate within a charge-balance and dose envelope
specified by the device's IEC 60601 dossier. The programme records
the per-channel envelope (charge per phase, charge density,
inter-pulse interval, accumulated charge per session) and the
device's certified margins. Implementations MUST enforce the envelope
in firmware, not only in the application layer; the application layer
records the envelope but does not relax it.

## §6 Cybersecurity and Software Updates

Devices that connect to clinic networks expose a managed update
endpoint over mutually-authenticated TLS 1.3 (RFC 8446). Software
updates are signed by the manufacturer's release key and verified by
the device prior to install. The programme records every update
applied to every device with the update's version, the signing key
fingerprint, and the verification outcome. Failed updates leave the
device on the prior version and trigger a clinic-side alert.

A device that loses connectivity for an extended period (the
threshold is defined in the device's risk file under §9) enters a
limited-functionality mode in which only previously-validated decoder
configurations are usable; new decoders cannot be loaded without
re-establishing the trust chain.

## §7 Records Retention

Programme records — every record defined in PHASE-1, the API audit
logs defined in PHASE-2 §13, raw acquisitions, motor-command streams,
adaptation logs, and adverse-event reports — are retained for the
period required by the operating jurisdiction's medical-device
regulation. The minimum baseline is the longer of (a) the device's
expected lifetime plus seven calendar years, or (b) the period
required by the user's national health-records law.

## §8 Time Synchronisation

Device clocks are synchronised against the clinic's reference time
service per RFC 5905 (NTPv4). Devices that operate in environments
without persistent network access carry a battery-backed real-time
clock and re-synchronise on every clinic visit; the maximum
unsynchronised interval is recorded in the programme's quality
dossier.

## §9 Risk File and Software Life Cycle

Every released device version carries an ISO 14971-aligned risk file
and an IEC 62304-aligned software-life-cycle artefact. The programme
references both files by content-address in the device record. A
device whose risk file or life-cycle artefact cannot be resolved at
publication time MUST NOT be activated for new fittings; the API
returns `urn:wia:prosthetic-control:risk-file-missing` on attempt.

## §10 Adverse-Event Escalation

Serious and life-threatening adverse events (PHASE-1 §8) escalate
through three concurrent channels: the operating clinic's risk
manager, the manufacturer's post-market surveillance team, and the
national medical-device authority. Reports to the authority follow
the timeline the authority requires (commonly within 10 to 30
calendar days of detection, with shorter windows for life-threatening
events). The API's adverse-event endpoint (PHASE-2 §11) emits the
notification and records the authority's report reference.

## §11 Usability Engineering

User-interface and clinical-workflow decisions are documented under
IEC 62366-1 usability engineering. Use-error scenarios are catalogued
during design and verified during fitting; recurring use errors that
appear in post-market surveillance trigger a usability-engineering
review and, where required, a labelling or training revision.

## §12 Programme Wind-Down

A programme that ceases operations transfers all open device fittings
to a successor programme or, where no successor exists, to a
nominated long-term archive. Devices in active use receive an
extended-support notice; the manufacturer's continued obligation to
honour the device's risk file is communicated to all users in the
form prescribed by the relevant medical-device authority.

## §13 Pediatric Considerations

Pediatric fittings carry additional governance: ethics-committee
approval for the use of investigational control modes in minors, parental
or guardian consent in addition to the user's assent where the user is
old enough to assent, growth-related re-fitting cadence (typically every
6 to 12 months for upper-limb pediatric users), and outcome instruments
adapted for the user's age. The programme records the pediatric
governance markers against each pediatric subject and flags pediatric
cohorts in queries so that adult-cohort statistics are not contaminated.

Pediatric users transition to the adult cohort at the operating clinic's
defined age threshold; the transition event is recorded against the
subject and prior pediatric observations remain accessible for
longitudinal continuity.

## §14 Outcome-Reporting Cadence

Outcome instruments (PHASE-1 §10) are administered at fitting (baseline),
at the prescribed follow-up interval (commonly week 4, week 12, and
annually thereafter), and after device re-configuration that materially
changes the user's experience. Programmes publishing externally cited
outcome statistics report mean and dispersion at the cohort level only;
individual outcomes never appear in external reports.

## §15 Cross-Border Programme Operation

Programmes that operate across borders maintain a primary jurisdiction
of medical-device registration and one or more operating jurisdictions
where clinics fit devices to users resident in those jurisdictions.
Cross-border data transfers honour the user's national health-records
law, which the operating clinic records at fitting. Devices operating
under mutual-recognition arrangements emit single sets of evidence
packages; devices operating under parallel certifications emit one
evidence package per certification regime.

## §16 Privacy and Data-Subject Rights

User rights over the records described in PHASE-1 are exercised through
the operating clinic, which is the data controller for the user's
clinical record. Requests for access, correction, or erasure are
mediated by the clinic's standard subject-access workflow; the
WIA-prosthetic-control programme acts as the data processor for the
clinic and executes the requested operation against the records held
in its API.

Erasure of clinical records carries an exception for adverse-event
records and risk-file linkages, which the operating jurisdiction's
medical-device regulation typically requires to be retained
irrespective of subject preference; the clinic explains the exception
to the user at fitting and again at any erasure request.

## §17 Conformance and Auditing

A programme conformant with WIA-prosthetic-control publishes its ISO
13485 certificate, its programme code registration, its risk-file
catalogue, and the catalogue of devices it has fitted, and answers
an annual self-assessment that maps each clause of this PHASE to the
programme's implementation. The self-assessment is reviewed during
the annual ISO 13485 surveillance audit.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-prosthetic-control
- **Last Updated:** 2026-04-27
