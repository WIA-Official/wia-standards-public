# WIA-industrial-iot PHASE 3 — PROTOCOL Specification

**Standard:** WIA-industrial-iot
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an accredited
industrial-IoT programme: operator accreditation, asset-vendor
qualification (ISO 19443-aligned for nuclear-related supply,
otherwise operator-defined), zone and conduit governance per IEC
62443, alarm-management discipline per ISA-18.2, control-loop
change control, calibration cadence, records retention,
historian time-synchronisation, decommissioning custody,
incident response, and programme wind-down.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO 14001:2015 (environmental management systems)
- ISO 45001:2018 (occupational health and safety management)
- ISO 14224:2016 (reliability and maintenance data exchange)
- ISO/IEC 17025:2017 (testing and calibration laboratories)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27019:2024 (IS controls for energy utilities)
- ISO 8601 (date and time)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8915 (NTS for NTP — Network Time Security)
- IETF RFC 8446 (TLS 1.3)
- IETF RFC 9457 (Problem Details)
- IEC 61131-3 (PLC programming languages)
- IEC 62264 (ISA-95)
- IEC 62443 (industrial automation security)
- ANSI/ISA-18.2 (alarm management)
- ANSI/ISA-101 (HMI design conventions; non-citable IEC mirror
  pending — operator may adopt the operator's national mirror)
- OPC Unified Architecture (UA)

---

## §1 Operator Accreditation

An industrial-IoT operator MAY claim conformance to
WIA-industrial-iot only after a recognised accreditation body has
issued a valid certificate against ISO 9001:2015 and against the
operator's information security management posture (ISO/IEC
27001:2022 with the IEC 62443 cybersecurity controls in scope of
the deployment). Operators that ship product to safety-critical
markets (nuclear, aerospace, medical) carry additional sector-
specific certifications.

## §2 Vendor Qualification

Asset vendors are qualified per the operator's vendor-management
programme: factory-acceptance test evidence, IEC 62443-aligned
secure-development declarations, firmware update signing keys,
and the vendor's incident-response playbook. Qualification is
recorded against the asset's `manufacturerRef` and is reviewed at
least annually.

## §3 Zone and Conduit Governance

The site's IEC 62443 zone architecture is documented in the
cyber-posture record (PHASE-1 §8). Zone boundaries are revised
through a controlled process: proposed change, security-engineer
review, IEC 62443 risk re-assessment, change-board approval, and
deployment with conduit re-verification. Submissions that
violate a zone or conduit (e.g. attempting to write into a
higher-security-level zone from a lower-level zone) return a
Problem-Details response of type
`urn:wia:industrial-iot:zone-conduit-violation`.

## §4 Alarm Management Discipline

Alarms follow ANSI/ISA-18.2 lifecycle expectations:
identification, rationalisation, design, implementation,
operation, monitoring & assessment, management of change, and
audit. The operating programme records the rationalisation
report for every priority-1 and priority-2 alarm and reviews
the alarm performance metrics (alarm rate per operator, peak
flooding rates, top-10 chronic alarms) at least monthly.

Alarm rate budgets typical of mature deployments are recorded in
the operator's quality dossier and serve as the operator's
internal performance target; the standard does not prescribe a
single numeric target because operating context varies.

## §5 Control-Loop Change Control

Loop tuning revisions follow change control: proposed change,
loop-performance impact assessment (overshoot, settling-time,
robustness), supervisor approval, controlled commissioning,
post-implementation verification. The verification's outcome is
appended to the loop's tuning history (PHASE-1 §5).

Changes to safety-related loops (loops that directly affect a
safety-instrumented function) require additional approvals from
the safety officer and follow the operator's IEC 61511-aligned
functional-safety governance.

## §6 Calibration Cadence

Sensors and instruments calibrated under ISO/IEC 17025 follow a
cadence appropriate to their service tier and the operating
environment: process-critical instruments typically every 6 to 12
months, custody-transfer instruments per the regulator's required
frequency, and non-critical sensors at the operator's defined
cadence. The cadence is recorded in the operator's quality
dossier; deviations trigger an alert on the asset's maintenance
schedule.

## §7 Records Retention

Programme records — every record defined in PHASE-1, the API
audit logs, alarm and event history, maintenance records, and
production-order traces — retain for the longer of (a) the
operating jurisdiction's regulatory requirement, or (b) the
product's expected service life plus the recall window. Safety-
related alarm and event history retains indefinitely.

## §8 Historian Time Synchronisation

Historians, edge gateways, PLCs, and SCADA front-ends
synchronise per RFC 5905 (NTPv4) or RFC 8915 (NTS for NTP) where
the operator's network supports it. Time synchronisation is
verified at every commissioning and at every major firmware
update; clock-drift events that exceed the operator's threshold
trigger an alert through the streaming subscription.

## §9 Cybersecurity Operations

Cybersecurity events (suspected intrusion, failed authentication
clusters, anomalous control-bus traffic) are reported under
PHASE-1 §6 with category `security-alarm`. Events of priority 1
or 2 escalate to the operator's CSIRT under the operator's
incident-response playbook; CSIRT's actions are recorded against
the event for post-incident review.

Programmes operating in jurisdictions with critical-infrastructure
disclosure requirements (CISA / ENISA NIS 2 / KISA / equivalent)
record the regulatory disclosure reference in the event record.

## §10 Decommissioning Custody

Asset decommissioning follows a documented workflow: removal-of-
service date, data export from historians and CMMS, secure
erasure of any sensitive configuration on the asset, physical
disposition (re-deployment, recycling, or destruction), and a
disposition certificate. The certificate is appended to the
asset record per PHASE-1 §3.

## §11 Records of Material Traceability

Production-order material-traceability records (PHASE-1 §9)
preserve input-lot to output-lot bindings for the period the
operating jurisdiction's product-liability law requires.
Programmes that operate under FDA, EMA, FSC, or comparable
regulatory authority encode the regulator's expected retention
in their quality dossier and on the production-order's
`status=complete` transition.

## §12 Time Synchronisation Audit

Sites synchronise per §8; audit cycles re-verify clock drift
budgets at the operator's defined cadence. Failed audits emit
remediation actions and are recorded in the operator's quality
dossier.

## §13 Cross-Site Federation

Operators that run multiple sites with shared APM, MES, or
historian services federate per-site records through the API.
Federation is mutually authenticated; cross-site queries are
gated by the consuming client's authorisation scope and return
`403 Forbidden` with type
`urn:wia:industrial-iot:cross-site-scope-violation` when the
scope does not include the queried site.

## §14 Programme Wind-Down

A programme that ceases operations transfers historian archives
to a recognised long-term archive, exports CMMS history to the
operator's records-management system, and notifies regulators
and certifying bodies of the cessation. Indefinite-retention
records (safety-related alarms, custody-transfer telemetry)
transfer to the long-term archive with content-addresses
preserved.

## §15 Quality Dossier

The programme's quality dossier records the vendors qualified,
the IEC 62443 architecture in force, the alarm-rationalisation
methodology, the calibration cadence policy, the cybersecurity
incident-response playbook, and the deprecation history of
asset firmware. The dossier is reviewed at least annually by
the operator's quality manager and is read during the annual
ISO 9001 / ISO/IEC 27001 surveillance audits.

## §16 Operating Personnel Health and Safety

Operating personnel (control-room operators, field technicians)
work under the operator's ISO 45001-aligned occupational-health
and safety governance. Shift-change handover events that affect
control authority are recorded at the alarm-event level so that
retrospective analyses can verify operator continuity at the
time of an incident.

## §17 Anti-Tamper Protections

Critical-asset configurations (control-loop tunings, IEC 62443
zone definitions, cybersecurity controls) are protected against
unauthorised modification through the operator's change-board
process and through cryptographic signing of configuration
exports. Tamper-detection events emit security-category alarms
under PHASE-1 §6 and escalate to CSIRT per §9.

## §18 Sustainability Reporting

Operators that publish sustainability disclosures (CSRD-aligned,
GRI, SASB, or equivalent) consume the energy / emissions
telemetry (PHASE-1 §10) and aggregate it into the disclosure
period the regulator requires. The aggregation methodology is
recorded in the operator's quality dossier and is reviewed at
the same cadence as the disclosure cycle.

## §19 Cross-Border Programme Operation

Multi-jurisdiction industrial-IoT operators honour each
participating jurisdiction's industrial-cybersecurity disclosure
rules (NIS 2, CIRCIA, KISA-equivalent). Cross-border telemetry
flows honour the source-jurisdiction's data-protection law for
operator-personnel data; only opaque references flow through the
WIA API.

## §20 Functional-Safety Coordination

Industrial deployments that include safety-instrumented
functions (per IEC 61511) coordinate the WIA-industrial-iot
records with the operator's functional-safety dossier. The
coordination is one-way: the WIA programme records reference
the SIS dossier identifiers but does not duplicate the SIS
proof-test or trip-history detail, which lives in the
functional-safety system of record.

Loops that affect a safety-instrumented function (PHASE-1 §5)
carry the SIF reference so that change-control reviews can
escalate to the operator's safety officer when a tuning or mode
change has the potential to affect the SIF.

## §21 Programme Wind-Down and Decommissioning

A programme that ceases operations transfers historian archives
to a recognised long-term archive, exports CMMS history to the
operator's records-management system, and notifies regulators
and certifying bodies of the cessation. Indefinite-retention
records (safety-related alarms, custody-transfer telemetry,
recall-relevant production-order traces) transfer to the long-
term archive with content-addresses preserved.

When a programme transitions to a successor operator (acquisition,
divestiture, joint-venture restructuring), the programme records
the successor's identifier and the date of effective handover.
The operator's quality dossier captures the handover audit so
that downstream consumers can trace records across the transition
without ambiguity.

## §22 Conformance and Auditing

A programme conformant with WIA-industrial-iot publishes its
operator accreditation references, its programme code
registration, its quality dossier, the catalogue of sites it
operates, and the cyber-posture summary, and answers an annual
self-assessment that maps each clause of this PHASE to the
programme's implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-industrial-iot
- **Last Updated:** 2026-04-27
