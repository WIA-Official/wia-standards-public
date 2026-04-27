# WIA-medical-iot PHASE 4 — Integration Specification

**Standard:** WIA-medical-iot
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE specifies how a medical-IoT deployment integrates the
data, APIs, and protocols of PHASEs 1–3 with broader operational
systems: hospital EHR (Electronic Health Record), biomedical-
engineering CMMS (Computerised Maintenance Management System),
clinical data warehouse, regulator audit pipelines, manufacturer
post-market surveillance, supply-chain UDI registries, and the
patient-facing apps that close the feedback loop. It is non-
prescriptive about specific vendors but specifies the integration
*contracts* a deployment must satisfy.

References (CITATION-POLICY ALLOW only):
- HL7 FHIR R5 — Subscriptions, Bulk Data Access, Bulk Data Group
- HL7 v2.x ADT, ORU, ORM, MDM messages (for legacy EHR integration)
- IHE PCD Technical Framework — PCD-01 (Communicate PCD Data),
  PCD-04 (Subscribe to PCD Data)
- ISO 13485:2016 — QMS for medical devices
- ISO 14971:2019 — risk management
- IEC 62304:2006/A1:2015 — software life cycle
- IEC 80001-1:2021 — risk management for IT-networks
- US FDA 21 CFR §820 — Quality System Regulation
- EU MDR 2017/745
- WIA-medical-data-privacy, WIA-medical-imaging, WIA-network-security,
  WIA-pq-crypto, WIA-supply-chain — sibling WIA standards

---

## §1 EHR integration

EHR systems consume MedIoT data through one of three patterns:

- **FHIR subscription** — the EHR registers a Subscription
  resource for the patients in its care; the boundary pushes
  Observations and AlarmConditions as they happen
- **FHIR bulk export** — the EHR pulls a periodic export
  (PHASE 2 §9) for offline analytics
- **HL7 v2 PCD-01** — for EHRs lacking FHIR support, the boundary
  emits HL7 v2.x ORU^R01 messages translated from FHIR
  Observation, with IHE PCD-01 message profile

The deployment policy declares which patterns are active.
Subscription-mode EHRs receive a webhook delivery within
seconds of observation ingestion; bulk-mode EHRs receive
batched deliveries on a documented schedule.

## §2 Device registry sync

Devices admitted to the deployment carry UDIs that resolve via:

- **GUDID** (FDA Global UDI Database) — for US-cleared devices
- **EUDAMED** (EU device database) — for MDR-cleared devices
- **K-MFDS UDI** — for Korea-cleared devices
- **Manufacturer attestation** — for devices not yet in any
  national registry, with a signed manufacturer manifest

The boundary refreshes UDI metadata on a deployment-declared
cadence (typically daily). A UDI-DI removed from a national
registry triggers a maintenance review of all bound devices.

The deployment publishes its current UDI roster to the
biomedical-engineering CMMS via FHIR Device subscription; the
CMMS owns service-history records that join on `deviceRef`.

## §3 Biomedical-engineering CMMS integration

The CMMS receives:

- New device admission events
- Calibration-due notifications
- Quarantine events
- Connectivity-loss events that exceed maintenance-window
  thresholds
- Software-update needs (when the firmware identifier in the
  device record falls behind the manufacturer's published latest)

The CMMS posts back:

- Service-record references (linking to the deployment's
  service-record store)
- Calibration records (PHASE 1 §6) signed by the BME principal
- Decommissioning notices

## §4 Clinical data warehouse integration

The data warehouse consumes:

- Bulk exports (PHASE 2 §9) of Observation, AlarmCondition,
  DeviceAssociation, and Device resources
- Subject identifiers are pseudonymous per PHASE 1 §10;
  re-identification requires a linkage authorisation per
  WIA-medical-data-privacy PHASE 1 §5

The warehouse SHOULD honour the de-identification job record
attached to each export and not retain the dataset past its
declared expiry.

## §5 Regulator audit integration

Regulators (FDA MDR for adverse events, EU MDR competent
authority, K-MFDS, PMDA) receive structured exports for
post-market surveillance:

- Adverse event reports referencing the contributing
  observations and alarm conditions
- Device-failure summaries with traceability to UDI-PI for
  recall scope determination
- Annual safety reports per IEC 62304 §8 problem resolution

The regulator-export endpoint is gated by a regulator-specific
release authority; the export carries audit-chain inclusion
proofs so regulators can verify the export's integrity.

## §6 Manufacturer post-market surveillance

Manufacturers receive:

- Aggregate metric distributions for their devices (no PHI)
- Device-failure reports with UDI-PI scope
- Calibration-drift summaries (anonymised)
- Software-related issue reports per IEC 62304 §6.2

Aggregation thresholds prevent re-identification: distributions
on cohorts of fewer than 30 devices or 30 patients are
suppressed. The manufacturer's contract with the deployment
declares the data-flow scope.

## §7 Patient-facing app integration

The patient-facing app (provided by the deployment, or a
third-party app authorised under SMART) consumes:

- Active device summary (PHASE 2 §6 `$active-devices`)
- Recent observations for self-monitoring metrics
- Active alarm conditions (limited to non-clinical detail)
- Education resources tied to the patient's monitored conditions

Patient-app actions emit AuditEvents so impersonation patterns
are detectable; the patient's own access appears in the audit
chain alongside clinician access.

## §8 Operational SLAs

| Concern                                          | Default SLA                |
|--------------------------------------------------|----------------------------|
| Observation ingest p95 added latency             | ≤ 200 ms                   |
| Alarm propagation to active subscriptions p95    | ≤ 1 s (high-priority)      |
| Bulk export for ≤ 10K observations               | ≤ 60 s                     |
| EHR FHIR subscription delivery p95               | ≤ 5 s                      |
| HL7 v2 ORU emission to PCD-01 partners           | ≤ 10 s                     |
| Audit chain entry available after operation      | ≤ 10 s                     |
| Calibration-due alert lead time                  | ≥ 14 days                  |
| Connectivity-loss escalation                     | per IEC 60601-1-8 alarm tau |

Tighter SLAs are negotiable per deployment; loosening them
requires biomedical-engineering and clinical-leadership sign-off.

## §9 Quarterly compliance report

The boundary emits a quarterly compliance report covering:

- Total observations ingested per device class
- Alarm conditions by priority and resolution outcome
- Calibration completion rate vs. due dates
- Connectivity uptime per device class
- Bulk exports delivered, by partner and purpose
- Incident reports: device quarantine, buffer overflows,
  clock-skew episodes
- Cross-domain references (consent, imaging) honoured vs.
  refused

The report is signed and is itself in scope for the audit
chain so report tampering would surface in the chain.

## §10 Acceptance criteria

A deployment claims conformance when:

1. Every fielded device is in the registry with current UDI
   resolution and current calibration
2. Every observation in the past quarter has a matching audit
   chain entry with verifiable inclusion proof
3. Every alarm condition has a resolution event or an active
   escalation
4. EHR integration delivers within SLA across at least 95% of
   observations
5. IEC 80001-1 risk file is current and signed
6. Quarterly compliance report has no integrity-check failures
7. Cross-domain references resolve at the partner boundary
   for ≥ 99% of bound observations

A deployment failing any of these reports the gap in its
compliance package rather than concealing it.

## §11 Common pitfalls (informative)

- **UDI-PI reuse** — manufacturers occasionally reuse
  serial-number ranges across product cycles; the boundary
  MUST verify UDI-PI uniqueness within the manufacturer's
  declared range
- **Time zone drift** — devices with no offset clock often
  default to UTC or to a manufacturing-locale offset; the
  boundary translates to local time on display but stores
  the original offset
- **Gateway battery exhaustion** — patient-owned smartphone
  gateways occasionally fail to relay observations; deployments
  SHOULD surface relay-loss patterns to patients
- **Maintenance-window observation** — observations during a
  maintenance window are flagged so they are not interpreted as
  normal operating data
- **Alarm fatigue** — IEC 60601-1-8 priority drift; the
  deployment SHOULD review alarm-priority configuration
  quarterly to prevent fatigue

## §12 Decommissioning

When a deployment is decommissioned:

1. Active devices are returned to inventory or the manufacturer
2. Patient associations are closed with a structured reason
3. EHR integration is wound down with prior notice to clinicians
4. The data warehouse retains the data within consent scope
5. The audit chain is sealed and a final root is published
6. Regulator notification is filed where required

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Cross-domain reference table (informative)

| Reference                 | Use site                                                  | Gate applied                                    |
|---------------------------|-----------------------------------------------------------|-------------------------------------------------|
| WIA-medical-data-privacy  | every observation is gated by consent + purpose-of-use    | medical-side consent + MedIoT-side purpose      |
| WIA-medical-imaging       | imaging-gateway devices reference imaging study UID       | medical-imaging-side study existence            |
| WIA-network-security      | TLS cipher-suite floor and SOC monitoring                 | network-security-side floor on each session     |
| WIA-pq-crypto             | post-quantum migration phase                              | pq-crypto-side phase declaration current        |
| WIA-supply-chain          | UDI-DI traceability for recall scope                      | supply-chain-side device manifest               |

## Annex B — Decommissioning checklist (informative)

- [ ] All active devices returned or transferred
- [ ] Patient associations closed
- [ ] EHR subscriptions cancelled with downstream sign-off
- [ ] CMMS service records exported
- [ ] Final bulk export delivered to data warehouse
- [ ] Regulator notification filed
- [ ] Audit chain sealed and final root published
- [ ] IEC 80001-1 risk file archived

## Annex C — EHR integration patterns (informative)

```json
{
  "resourceType": "Subscription",
  "status": "active",
  "topic": "https://wia.example/SubscriptionTopic/medical-iot-alarms",
  "channel": {
    "type": "rest-hook",
    "endpoint": "https://ehr.example/wia-webhook",
    "payload": "application/fhir+json"
  },
  "filterBy": [
    {"resourceType": "AlarmCondition", "filterParameter": "subject", "value": "<patient cohort>"}
  ]
}
```

The EHR receives notification bundles; failure to acknowledge
within 24 hours suspends the subscription with operations alert.

## Annex D — Manufacturer surveillance worked example (informative)

```
GET /Manufacturer/<gln>/$device-failure-summary?period=2026-Q2 HTTP/1.1
Authorization: Bearer <manufacturer-jwt>
Accept: application/fhir+json
```

Returns:

- Aggregate failure counts grouped by UDI-DI (no UDI-PI to
  prevent re-identification of installation sites)
- Failure modes (battery, sensor drift, firmware crash)
- Time-to-failure distributions
- Counts suppressed when n < 30

## Annex E — Risk-file change management (informative)

Material changes to the deployment trigger an IEC 80001-1 risk-
file update before commit:

| Change                                | Risk-file action                  |
|---------------------------------------|-----------------------------------|
| New device class admitted             | Re-evaluate harms + control map   |
| New gateway type                      | Re-evaluate network segmentation  |
| Cipher-suite floor change             | Cross-reference network-security  |
| New EHR partner                       | Re-evaluate data-flow risks       |
| Quarterly review                      | Confirm or revise residual risks  |

Risk-file updates are signed by the clinical-engineering
director and the chief medical informatics officer.

## Annex F — Conformance disclosure

Sections §1, §2, §3, §4, §8, §10 are mandatory.
Sections §5, §6, §7 are mandatory where the deployment
operates in a jurisdiction or contract that requires the
flow; otherwise they are excluded with documented reason.
Conformance levels are declared in the capability advertisement.
