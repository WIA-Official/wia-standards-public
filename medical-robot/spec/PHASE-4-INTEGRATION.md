# WIA-medical-robot PHASE 4 — Integration Specification

**Standard:** WIA-medical-robot
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE specifies how a medical-robot deployment integrates the
data, APIs, and protocols of PHASEs 1–3 with broader operational
systems: hospital EHR, OR scheduling, biomedical-engineering CMMS,
clinical data warehouse, regulator audit pipelines, manufacturer
post-market surveillance, training-simulator pipelines, and the
patient-facing apps that close the procedural feedback loop.

References (CITATION-POLICY ALLOW only):
- HL7 FHIR R5 — Subscriptions, Bulk Data Access, Bundle, Procedure,
  ServiceRequest, AdverseEvent, Provenance
- HL7 v2.x messages: SIU (scheduling), ORM (orders), ORU (results)
- IHE Surgical Workflow profile (where adopted) and IHE Patient
  Care Device profile for monitoring during procedures
- ISO 13485:2016, ISO 14971:2019, IEC 62304, IEC 62366-1
- IEC 80601-2-77:2019, IEC 80601-2-78:2019
- ISO 13482:2014
- US FDA 21 CFR §820, EU MDR 2017/745
- WIA-medical-data-privacy, WIA-medical-imaging, WIA-medical-iot,
  WIA-network-security, WIA-pq-crypto, WIA-supply-chain

---

## §1 EHR integration

EHR systems consume robot-procedure data through:

- **FHIR subscription** — for procedure events, intervention
  events, and high-priority alarms in real time
- **FHIR bulk export** — for procedure summaries delivered
  daily to the EHR's results inbox
- **HL7 v2 ORU^R01** — for legacy EHRs, procedure summaries
  emitted at completion

The deployment policy declares which patterns are active.
Procedure-completion delivery includes the OR-team list, the
robot identity, the duration, the intervention-event count
by kind, and any AdverseEvent resources arising from the
procedure.

## §2 OR scheduling integration

OR scheduling systems integrate via FHIR ServiceRequest and
the IHE Surgical Workflow profile (where adopted):

- The scheduling system creates ServiceRequest resources for
  upcoming procedures
- The robot boundary verifies robot availability, calibration
  status, and end-effector inventory at the scheduled time
- A scheduling conflict (robot under maintenance, end-effector
  out-of-stock) returns `urn:wia:mrobot:problem:scheduling-conflict`
- Procedure start consumes the ServiceRequest, transitioning it
  to `status: completed` on procedure completion

## §3 Biomedical-engineering CMMS integration

The CMMS receives:

- New robot admission events
- End-effector lifetime warnings (90% of cycle limit)
- Calibration-due notifications (≥ 14 days before due)
- Verification-failure events
- Maintenance-overrun events for retrospective review
- Software-update needs and post-update re-attestation results

The CMMS posts back service records, calibration records
(PHASE 1 §7), end-effector replacements with new UDI-PI, and
decommissioning notices.

## §4 Clinical data warehouse integration

The data warehouse consumes:

- Procedure resources with the procedure-team and outcome
- Intervention-event summaries
- Telemetry summaries (downsampled)
- AdverseEvent resources

Subject identifiers are pseudonymous per PHASE 1 §11. The
warehouse may aggregate procedure outcomes by surgeon and by
robot for quality-improvement programmes; aggregation
thresholds prevent re-identification of small-volume teams.

## §5 Regulator audit integration

Regulators receive:

- AdverseEvent reports with traceability to the procedure,
  robot, end-effector, and operator
- Device-failure summaries for post-market surveillance
- Annual safety reports per IEC 62304 §8
- Recall-scope queries when a manufacturer issues a recall;
  the boundary returns the affected procedures' UDI-PI scope

The regulator-export endpoint is gated by jurisdiction-
specific release authority.

## §6 Manufacturer post-market surveillance

Manufacturers receive:

- Aggregate procedure counts by robot model
- Failure-mode distributions (force-limit violations, tele-
  link losses, calibration drift)
- Software-related issue reports per IEC 62304 §6.2
- End-effector lifetime histograms

Aggregation thresholds prevent re-identification:
distributions on cohorts of fewer than 30 procedures or 30
patients are suppressed.

## §7 Training-simulator integration

Training simulators consume:

- De-identified procedure replays (signed export per
  WIA-medical-data-privacy PHASE 1 §7)
- Telemetry traces for skill-assessment training
- Intervention-event sequences for OR-team rehearsal

Replays are delivered as a sealed bundle; the simulator
verifies the bundle signature before consumption.

## §8 Operational SLAs

| Concern                                          | Default SLA                |
|--------------------------------------------------|----------------------------|
| Procedure-start pre-check p95                    | ≤ 5 s                      |
| Telemetry ingest p95 added latency               | ≤ 50 ms                    |
| Alarm propagation to OR console p95              | ≤ 200 ms                   |
| EHR subscription delivery p95                    | ≤ 5 s                      |
| Bulk export for ≤ 1000 procedures                | ≤ 60 s                     |
| Audit chain entry available after operation      | ≤ 10 s                     |
| Calibration-due alert lead time                  | ≥ 14 days                  |

Tighter SLAs are negotiable per deployment; loosening them
requires biomedical-engineering and clinical-leadership
sign-off.

## §9 Quarterly compliance report

The boundary emits a quarterly compliance report covering:

- Total procedures by robot kind and surgeon credentials
- Intervention events by kind (force-limit, tele-link-loss,
  emergency-stop)
- Outcome distribution and AdverseEvent rate
- Calibration completion rate vs. due dates
- End-effector lifecycle: deployments, replacements, scrap
- Tele-surgery sessions by partner site
- Cross-domain references honoured vs. refused
- Audit-chain integrity check results

The report is signed and is itself in scope for the audit
chain.

## §10 Acceptance criteria

A deployment claims conformance when:

1. Every fielded robot is in the registry with current UDI
   resolution, calibration, and verification
2. Every procedure in the past quarter has a matching
   audit-chain entry with verifiable inclusion proof
3. Operator credentials are current and match the procedures
   they performed
4. EHR integration delivers within SLA across at least 95%
   of procedure events
5. IEC 80001-1 risk file is current and signed
6. Quarterly compliance report has no integrity-check
   failures

A deployment failing any of these reports the gap in its
compliance package rather than concealing it.

## §11 Common pitfalls (informative)

- **Tele-surgery clock skew** — remote-console operators on
  a different timezone experience clock-skew alarms; the
  deployment SHOULD configure boundary-clock as the
  authoritative reference for the procedure
- **End-effector recycling drift** — manufacturers occasionally
  reuse serial-number ranges; UDI-PI uniqueness verification
  must be strict
- **Operator credential expiry mid-procedure** — the boundary
  honours credentials valid at procedure-start time even if
  they expire during the procedure (to avoid forcing an abort)
- **Surgeon-handover over tele-link** — handover requires
  quorum signature; some deployments mistakenly accept a
  unilateral handover, which is rejected
- **OR network segmentation** — the OR private LAN should not
  be merged with general hospital networks; an accidental
  merge surfaces as cross-traffic on monitoring

## §12 Decommissioning

When a robot is decommissioned:

1. Active procedures complete or are transferred to alternate
   robots
2. Telemetry archives are sealed and kept for the regulatory-
   retention window
3. End effectors are inventoried; consumables go to clinical-
   waste handling
4. The robot's storage is wiped per the manufacturer's
   data-sanitisation procedure
5. Audit chain is sealed for the robot's history

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Cross-domain reference table (informative)

| Reference                  | Use site                                                 | Gate applied                                         |
|----------------------------|----------------------------------------------------------|------------------------------------------------------|
| WIA-medical-data-privacy   | every procedure record references consent                | medical-side consent + robot-side purpose            |
| WIA-medical-imaging        | imaging studies acquired during procedure                | medical-imaging-side study existence + release       |
| WIA-medical-iot            | non-robotic IoT devices used during procedure            | medical-iot-side device association                  |
| WIA-network-security       | TLS cipher-suite floor for tele-surgery                  | network-security-side floor on each session          |
| WIA-pq-crypto              | post-quantum migration phase                             | pq-crypto-side phase declaration current             |

## Annex B — Decommissioning checklist (informative)

- [ ] Active procedures completed or transferred
- [ ] End effectors inventoried and disposed
- [ ] Telemetry archives sealed
- [ ] CMMS service record finalised
- [ ] Audit chain sealed and final root published
- [ ] Manufacturer notified per warranty / service contract
- [ ] Storage wiped per manufacturer procedure

## Annex C — Conformance disclosure

Sections §1, §3, §4, §8, §10 are mandatory. §2 (OR scheduling)
is mandatory for deployments performing scheduled procedures
(typically all surgical robots). §5 (regulator audit) is
mandatory in any jurisdiction or contract requiring it. §6,
§7 are excluded with documented reason where the data flow
is not in scope.

## Annex D — Worked end-effector lifecycle (informative)

A laparoscopic stapler end-effector cycles through:

1. Manufactured with UDI-DI = `M-LAP-STAPLER-1.0`, cycle limit
   declared as 50 deployments
2. Admitted to deployment with UDI-PI = `SN-91A7`, cycle counter
   = 0
3. Used in 30 procedures over 6 months; cycle counter = 30
4. At cycle counter = 45 (90% of limit), CMMS receives
   `end-effector-life-warning` event
5. At cycle counter = 50, the boundary refuses procedure-start
   with this end-effector
6. CMMS marks the end-effector for waste-stream handling and
   updates inventory
7. Audit chain retains the end-effector's full history for the
   regulatory-retention window

This pattern surfaces deviations: an end-effector reaching
cycle limit faster than peers may indicate a clinical-team
training opportunity.

## Annex E — Surgeon-quality dashboards (informative)

The deployment may publish per-surgeon quality dashboards
internally:

- Procedure volume by code and robot kind
- Intervention-event rate (force-limit, emergency-stop)
- Procedure duration vs. peer median
- AdverseEvent rate
- Patient-reported outcome correlation (where consented)

Dashboards are gated by clinical-leadership policy; surgeon-
identifiable views are restricted to the surgeon themselves
and the clinical-leadership team. Anonymised aggregate views
may be shared with the broader OR team for quality-improvement
purposes.

## Annex F — Telemetry research-export approval (informative)

High-frequency raw telemetry export for research:

1. Researcher submits a research request with consent-bundle
   and IRB approval reference
2. Clinical-leadership reviews
3. On approval, the boundary issues a one-time research-
   export token bound to the consent bundle
4. Researcher pulls the export; the boundary records the
   pull as a high-priority audit event with researcher
   identity
5. Export expires after the deployment-declared retention
   window; researcher must renew or destroy

The pattern ensures that high-fidelity telemetry never leaves
the deployment without explicit consent + IRB + clinical-
leadership concurrence.
