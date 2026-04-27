# WIA-medical-alert-system PHASE 4 — Integration Specification

**Standard:** WIA-medical-alert-system
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE specifies how a medical-alert-system deployment
integrates the data, APIs, and protocols of PHASEs 1–3 with
broader operational systems: hospital EHR, primary-care
records, monitoring-station CRM, EMS dispatch, family-contact
apps, regulator audit pipelines, manufacturer post-market
surveillance, and the social services that close the loop on
welfare-related activations.

References (CITATION-POLICY ALLOW only):
- HL7 FHIR R5 — Subscriptions, Bulk Data Access, Bundle,
  Communication, CommunicationRequest, Encounter
- HL7 v2.x ADT messages — for hospital admit/discharge
  flow correlation
- IHE PCC profile — Patient Care Coordination
- ISO 13485:2016, ISO 14971:2019, IEC 62304, IEC 80001-1
- IEC 60601-1-8:2020, IEC 80601-2-58:2019
- EN 50134 series
- WIA-medical-data-privacy, WIA-medical-iot, WIA-medical-imaging,
  WIA-network-security, WIA-pq-crypto

---

## §1 EHR integration

EHR systems consume alarm-system data via:

- **FHIR subscription** — for activations triggering hospital
  admission via EMS dispatch; the EHR receives a heads-up
  before the patient arrives in the ED
- **FHIR bulk export** — for population-level activation
  pattern review
- **HL7 v2 ADT linkage** — admission events from EMS-
  transported PERS activations correlate with the hospital's
  ADT stream

The deployment policy declares which patterns are active.

## §2 Primary-care records integration

For ambulatory deployments, the patient's primary-care record
is updated post-resolution:

- Activation summary (with disposition)
- Vital-sign metrics captured at activation
- Clinical-team commentary if dispatched

This closes the loop so the primary-care clinician has
visibility into between-visit events.

## §3 Monitoring-station CRM integration

Monitoring stations maintain CRM records of the subjects
they cover:

- Subject demographic and clinical-history overview (per
  consent scope)
- Routing-policy preferences
- Family-contact roster
- Recent activation history

The boundary feeds these via FHIR subscription; the CRM is
the canonical operator workspace for handling activations.

## §4 EMS dispatch integration

For PERS deployments where dispatch is part of the routing
chain:

- Boundary pushes a structured dispatch payload to the
  EMS dispatch system (typically NEMSIS-style payload in
  US, similar national equivalents elsewhere)
- Payload includes location, subject identifier,
  presenting-event, vital-sign metrics, and consent for
  emergency-disclosure
- EMS arrives, performs assessment, transports if
  warranted; transport notification flows back to the
  boundary
- Hospital admission flows via HL7 v2 ADT

## §5 Family-contact app integration

The family-contact app receives notifications per the
subject's documented consent:

- Activation occurred (redacted if non-emergency)
- Acknowledgement and disposition once known
- Welfare-check fires (if subject has authorised family
  notification)

Apps are SMART-launched against the family contact's
authenticated identity provider.

## §6 Regulator audit integration

Regulators receive structured exports for:

- Adverse event reports if activation outcomes include
  patient harm
- Aggregate activation-rate reports for the deployment
- Post-market surveillance for medical-class devices
  (PERS pendants, fall detectors, nurse-call panels)

Each regulator-export endpoint is gated by a release
authority specific to the jurisdiction.

## §7 Manufacturer post-market surveillance

Manufacturers receive:

- Aggregate activation rates per device kind
- False-alarm distributions (device-malfunction false
  alarms vs. user-side false alarms)
- Battery-life and connectivity-quality histograms
- Software-related issue reports per IEC 62304 §6.2

Aggregation thresholds prevent re-identification: cohorts
fewer than 30 devices or 30 patients are suppressed.

## §8 Operational SLAs

| Concern                                          | Default SLA                |
|--------------------------------------------------|----------------------------|
| Alarm activation acknowledged at boundary p95    | ≤ 1 s                      |
| First routing attempt initiated p95              | ≤ 2 s                      |
| Monitoring-station webhook delivery p95          | ≤ 5 s                      |
| EMS dispatch delivery p95                        | ≤ 10 s                     |
| Voice-call establishment p95 to primary station  | ≤ 30 s                     |
| Welfare-check escalation                         | per EN 50134-7 cadence     |
| Audit chain entry available after operation      | ≤ 10 s                     |

Tighter SLAs are negotiable per deployment; loosening them
requires clinical-leadership and operations sign-off.

## §9 Quarterly compliance report

The boundary emits a quarterly compliance report covering:

- Total activations by device kind and trigger kind
- Routing-policy outcome distribution (acknowledged at
  level 0, 1, 2; failover events)
- False-alarm rates by device kind
- Welfare-check generation and resolution rates
- EMS dispatches and transport rates
- Cross-domain references honoured vs. refused
- Audit-chain integrity check results

The report is signed and is itself in scope for the audit
chain.

## §10 Acceptance criteria

A deployment claims conformance when:

1. Every fielded device or installation is in the registry
   with current certification status
2. Every alarm in the past quarter has a matching audit
   chain entry with verifiable inclusion proof
3. Every alarm has a resolution event or an active
   escalation
4. Monitoring-station integration delivers within SLA across
   at least 99% of activations
5. Quarterly compliance report has no integrity-check
   failures
6. Cross-domain references resolve at the partner boundary
   for ≥ 99% of bound activations

A deployment failing any of these reports the gap.

## §11 Common pitfalls (informative)

- **PERS battery exhaustion** — devices stop emitting
  heartbeat without explicit failure signal; the boundary
  detects via heartbeat-timeout welfare-check. The
  deployment SHOULD review battery-low alerts and
  proactive replacement schedule
- **Shared-device subject confusion** — pendants
  reassigned to a new patient may carry stale subject
  reference; sanitisation is mandatory before re-deployment
- **False-alarm fatigue** — fall detectors with poor
  calibration produce excessive false alarms; the
  deployment SHOULD review false-alarm rates quarterly and
  retune
- **Family-contact churn** — patients change family contacts
  frequently; the deployment SHOULD make roster maintenance
  a low-friction operation
- **EN 50134-7 cadence drift** — escalation cadences may
  drift if the routing policy is edited without review;
  the deployment SHOULD audit cadence quarterly

## §12 Decommissioning

When a deployment is decommissioned:

1. Active devices are returned to inventory or family
2. Monitoring-station coverage transferred to receiving
   service
3. EHR and primary-care records updated with deployment
   end date
4. Audit chain sealed and final root published
5. Regulator notification filed if required

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Cross-domain reference table (informative)

| Reference                  | Use site                                                  | Gate applied                                   |
|----------------------------|-----------------------------------------------------------|------------------------------------------------|
| WIA-medical-data-privacy   | every activation references the consent record            | medical-side consent + alert-side purpose      |
| WIA-medical-iot            | vital-sign metrics carried on activations                 | medical-iot device association                 |
| WIA-network-security       | TLS cipher-suite floor for monitoring-station partners    | network-security-side floor on each session    |
| WIA-pq-crypto              | post-quantum migration phase                              | pq-crypto-side phase declaration current       |

## Annex B — Decommissioning checklist (informative)

- [ ] Active devices returned or transferred
- [ ] Monitoring-station coverage transferred
- [ ] EHR and primary-care records updated
- [ ] Audit chain sealed and final root published
- [ ] Manufacturer notifications filed
- [ ] Regulator notification filed if required

## Annex C — Conformance disclosure

Sections §1, §2, §3, §8, §10 are mandatory. §4 (EMS) is
mandatory for PERS deployments where EMS dispatch is part
of the response chain. §5, §6, §7, §11 are mandatory where
the corresponding flow is offered.

## Annex D — Worked PERS-to-EMS dispatch (informative)

```
1. Subject presses pendant; device transmits activation
2. Boundary receives, creates AlarmEvent
3. First routing attempt: webhook to primary monitoring station
4. Operator picks up call; subject confirms emergency
5. Operator $triage with confirmed-emergency note
6. Operator $dispatch with EMS-dispatch payload
7. Boundary forwards to EMS dispatch system (NEMSIS or equivalent)
8. EMS arrives, performs assessment
9. EMS transports to nearest hospital
10. Hospital ADT system records admission with reference back
    to the alarm event id (HL7 v2 PV1.50 placer order number)
11. Boundary records $transfer-to-care with hospital reference
12. Activation closed; quality-improvement metrics recomputed
```

## Annex E — Welfare-check escalation worked example (informative)

```
1. Subject in assisted-living; bed-occupancy sensor reports
   no occupancy at 07:30 (outside expected wakeup window)
2. Boundary creates WelfareCheck with kind=morning-wakeup
3. First routing attempt: care-team facility-floor-staff app
4. No acknowledgement within 90 seconds
5. Escalation to facility on-call nurse
6. Nurse acknowledges, performs in-person check
7. Subject is found; reports having gone for early walk
8. Nurse $resolve with reason: subject-found-active
9. Welfare-check closed; quality-improvement records the
   resolution time and reason
```

## Annex F — Manufacturer post-market quarterly bundle (informative)

The boundary provides manufacturers with a quarterly bundle:

- Aggregate activation counts per device model
- False-alarm rate distribution (with reasons)
- Battery-life histograms
- Connectivity-quality histograms
- Software-related issue summary
- Recall-scope query results if any

Aggregation thresholds prevent re-identification of
small-volume installations (n < 30).

## Annex G — Social-services integration (informative)

For welfare-related activations (inactivity, missed welfare-
check) where there is no clinical emergency, deployments
may integrate with social services:

- Local-government adult protective services for vulnerable
  adults with patterns of welfare-check escalation
- Voluntary community-care organisations for befriending
  and reassurance follow-ups
- Family-mediation services where family-contact roster
  conflicts surface

Integration is governed by the patient's documented consent
and the deployment's social-services-partnership agreement.

## Annex H — Quality-improvement programme integration

Deployments often participate in industry-wide quality-
improvement programmes:

- Aggregate false-alarm rates reported to industry consortium
- Benchmark response-time distributions
- Best-practice routing-policy reference templates
- Adverse-event lessons-learned shared with peer deployments

Participation is governed by the deployment's quality-
improvement-partnership agreement; aggregation thresholds
prevent re-identification.
