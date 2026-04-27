# WIA-industrial-robot PHASE 3 — PROTOCOL Specification

**Standard:** WIA-industrial-robot
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an accredited
industrial-robot programme: integrator and operator accreditation,
robot-vendor qualification, ISO 10218 task-based risk assessment,
ISO/TS 15066 collaborative-mode commissioning, ISO 9283 baseline
testing, change-control for safety configuration, calibration
cadence, incident response, occupational-safety reporting,
records retention, decommissioning custody, and programme
wind-down.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO 14001:2015 (environmental management systems)
- ISO 45001:2018 (occupational health and safety management)
- ISO 9283:1998 (robot performance test methods)
- ISO 9787:2013 (robot coordinate systems)
- ISO 10218-1 / ISO 10218-2 (robot safety)
- ISO/TS 15066:2016 (collaborative robots)
- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC TR 22100-5 (cybersecurity for industrial machinery)
- ISO 8601 (date and time)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8446 (TLS 1.3)
- IETF RFC 9457 (Problem Details)
- IEC 61131-3 (PLC programming languages)
- IEC 61784-3 (functional-safety field-bus profiles)
- IEC 62061 (functional safety of E/E/PE control systems)
- ISO 13849-1 (safety-related parts of control systems)

---

## §1 Operator and Integrator Accreditation

An operator MAY claim conformance to WIA-industrial-robot only
after a recognised accreditation body has issued a valid
certificate against ISO 9001:2015 and ISO 45001:2018 (occupational
health and safety) covering the deployment scope. Integrators
that commission collaborative-robot cells additionally maintain
ISO 10218 / ISO/TS 15066 competence documented in their quality
dossier.

## §2 Vendor Qualification

Robot vendors are qualified per the operator's vendor-management
programme: ISO 10218-1 type-certification evidence, vendor's
ISO/TS 15066 collaborative-operation declarations where relevant,
firmware update signing keys, and the vendor's incident-response
playbook. Vendors whose firmware has been the subject of a
recall or a safety advisory carry the resolved-status record so
that integrators can verify the deployed firmware's safety
status.

## §3 Task-Based Risk Assessment

Every robotic task (PHASE-1 §4) follows the ISO 10218-1 §5 task-
based risk assessment process: identification of hazardous
situations, risk-estimation, risk evaluation, and risk-reduction
measure selection. The assessment is recorded as a content-
addressed artefact and referenced from the task record. Risk
assessment is repeated at every material change to the task
(new tooling, new workspace, new program flow).

## §4 Collaborative-Mode Commissioning

Cells operating under collaborative modes (PHASE-1 §3
`iso10218SafetyClass` ∈ {hand-guiding, speed-and-separation-
monitoring, power-and-force-limiting}) are commissioned with
ISO/TS 15066 procedures: biomechanical-limit determination,
separation-monitoring sensor commissioning (light curtains, area
scanners, time-of-flight cameras), and validation runs that
exercise the worst-case approach scenarios at the cell's
expected operating speeds. Commissioning evidence is appended to
the safety-configuration record.

## §5 Safety Configuration Change Control

Changes to safety configuration (zones, speed limits,
biomechanical-limit profiles, fieldbus parameters) follow change
control: proposed change → safety-engineer review → re-validation
under §3 task-based risk assessment → safety officer approval →
controlled deployment → post-implementation verification. The
verification's outcome appends to the safety-configuration
record.

## §6 ISO 9283 Baseline Testing

Each robot is baseline-tested per ISO 9283 at commissioning:
position repeatability (RP), pose repeatability (PR), distance
accuracy (AT), path accuracy, velocity accuracy. Baseline
results are recorded against the robot record and re-evaluated
at calibration cadence. Drift beyond the operator's defined
threshold triggers a maintenance work order.

## §7 Calibration Cadence

TCP and joint calibrations are performed under ISO/IEC 17025
procedures at the cadence the operator declares. Routine TCP
re-verification typically follows tool-change events; full ISO
9283 re-testing typically follows major firmware updates and
component replacements (harmonic-drive, joint-encoder, motor).

## §8 Incident Response

Safety incidents (PHASE-1 §7) trigger the operator's incident-
response playbook: e-stop verification, witness interviews,
review of the reconstruction motion-sample window, root-cause
investigation, corrective action plan, and reporting to the
relevant occupational-safety authority for incidents of
`major-injury` severity or above.

The reconstruction motion-sample window is the critical artefact
for retrospective analysis; programmes MUST configure the window
duration to be at least long enough to capture the prior
motion that contributed to the incident (typically 1000-5000 ms).

## §9 Occupational-Safety Reporting

Programmes report safety incidents to the operating
jurisdiction's occupational-safety authority within the period
the authority requires. The report carries the incident record,
the reconstruction window, and the root-cause investigation. The
authority's report reference is recorded against the incident
record.

## §10 Cybersecurity Operations

Cybersecurity events affecting the robot controller (firmware
update verification failures, anomalous fieldbus traffic,
unexpected configuration changes) follow PHASE-1 §6 of the
adjacent industrial-IoT standard's category mapping (security-
alarm). Events of priority 1 or 2 escalate to the operator's
CSIRT under the operator's incident-response playbook.

## §11 Records Retention

Programme records — every record defined in PHASE-1, the API
audit logs, safety incidents, maintenance records, and safety
configurations — retain for a minimum of seven calendar years
from the last access of the cell. Safety-incident records of
`major-injury` severity or above retain indefinitely.

## §12 Time Synchronisation

Robot controllers, edge gateways, historian, and SCADA front-
ends synchronise per RFC 5905 (NTPv4). Sub-millisecond
synchronisation is preserved on the safety-fieldbus per IEC
61784-3 expectations independent of the WIA programme; the WIA
programme only consumes the historian's synchronised time.

## §13 Decommissioning Custody

Robot decommissioning follows a documented workflow: removal of
service date, secure erasure of any sensitive configuration on
the controller, physical disposition (re-deployment, refurbish,
recycle, destruction), and a disposition certificate appended to
the robot record per PHASE-1 §2.

## §14 Programme Wind-Down

A programme that ceases operations transfers historian archives
to a recognised long-term archive, exports CMMS history to the
operator's records-management system, and notifies regulators
and certifying bodies of the cessation. Indefinite-retention
records (major-injury safety incidents) transfer with content-
addresses preserved.

## §15 Quality Dossier

The programme's quality dossier records the vendors qualified,
the integrators contracted, the ISO 10218 / ISO/TS 15066
competence framework, the safety-configuration change-control
authority, and the deprecation history of robot firmware. The
dossier is reviewed at least annually by the operator's quality
manager.

## §16 Cross-Border Programme Operation

Multi-jurisdiction operators honour each participating
jurisdiction's occupational-safety reporting rules and product-
safety regulations applicable to the robot vendor's exports.

## §17 End-Effector Swap Discipline

End-effector swaps (PHASE-1 §10) are governed under change
control because the swap may shift the dynamic load envelope of
the robot, the biomechanical-limit profile in collaborative
modes, and the cycle-time budgeting of the affected tasks.

A swap event triggers (a) re-validation of the safety
configuration if the cell is collaborative, (b) update of the
TCP calibration if the new effector has a different geometry,
and (c) re-baselining of cycle-time budgets for tasks that
depend on the new effector. The operator's quality dossier
records the swap-discipline workflow.

## §18 Cybersecurity for Robot Controllers

Robot controllers operate under the IEC 62443 zone classification
of the cell (PHASE-1 §9). Firmware updates are signed by the
vendor's release key and verified by the controller before
install; failed verification leaves the controller on the prior
firmware and emits a security-category alarm event.

Controllers exposed to plant networks (for OPC UA Robotics
companion-spec endpoints, MES integration, or remote diagnostics)
restrict ingress through the operator's industrial-security
broker; direct external access from the public internet is not
permitted under this PHASE.

## §19 Operator Training and Authorisation

Operators interacting with the cell (programmers, machine
tenders, maintenance technicians) carry training and
authorisation records held in the operator's HR system. The
WIA-industrial-robot programme records the authorised-operator
list per cell at a level appropriate to safety-relevant
operations: who is authorised to run the cell in production
mode, who can place the cell into teach mode, who can update
safety configuration. The list is referenced from the safety
configuration record so that any change to safety-relevant
operation requires a re-binding to authorised operators.

## §20 Programme Wind-Down and Successor Handover

A programme that ceases operations transfers historian archives
to a recognised long-term archive, exports CMMS history to the
operator's records-management system, and notifies regulators
and certifying bodies of the cessation. Indefinite-retention
records (major-injury safety incidents) transfer with content-
addresses preserved.

When a cell transitions to a successor operator (acquisition,
divestiture), the programme records the successor's identifier
and the date of effective handover so that downstream consumers
can trace records across the transition.

## §21 Cross-Border Programme Operation Detail

Multi-jurisdiction operators that ship robots across borders
honour each participating jurisdiction's product-safety
regulations applicable to the vendor's exports (CE marking under
the EU Machinery Directive 2006/42/EC, OSHA-aligned hazard
controls in the United States, KC certification in the Republic
of Korea, equivalent regimes in other operating jurisdictions).
The operator's compliance lead records the applicable regimes
per cell so that downstream consumers see which authority's
requirements the cell honours.

## §22 Quality Dossier Annual Review

The operator's quality dossier (PHASE-3 §15) is reviewed at
least annually by the operator's quality manager and is read
during the annual ISO 9001 / ISO 45001 surveillance audits. The
review's outcomes are recorded as content-addressed minutes that
the public catalogue references; programmes that publish
externally cited cells attach the most recent review reference
to the evidence package's audit section.

## §23 Conformance and Auditing

A programme conformant with WIA-industrial-robot publishes its
operator and integrator accreditation references, its programme
code registration, its quality dossier, the catalogue of cells
it operates, and the cyber-posture summary, and answers an
annual self-assessment that maps each clause of this PHASE to
the programme's implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-industrial-robot
- **Last Updated:** 2026-04-27
