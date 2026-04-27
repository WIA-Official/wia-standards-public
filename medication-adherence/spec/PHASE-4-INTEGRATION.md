# WIA-medication-adherence PHASE 4 — Integration Specification

**Standard:** WIA-medication-adherence
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE specifies how a medication-adherence deployment
integrates the data, APIs, and protocols of PHASEs 1–3 with
broader operational systems: hospital EHR, ambulatory clinic
practice management, pharmacy systems (in-hospital pharmacy,
retail chains, mail-order), smart-device manufacturer pipelines,
clinical data warehouse, regulator audit pipelines, controlled-
substance monitoring programmes, and pharmaceutical post-market
surveillance.

References (CITATION-POLICY ALLOW only):
- HL7 FHIR R5 — Subscriptions, Bulk Data Access, Bundle,
  MedicationDispense, MedicationAdministration, AdverseEvent
- HL7 v2.x: RDS, RAS, RGV, RDE messages
- IHE Pharmacy Hospital Medication Workflow profile
- ISO 13485:2016, IEC 62304, IEC 80001-1:2021
- US 21 CFR §1300, §1311 — controlled substance management
- US FDA REMS programme
- EU Falsified Medicines Directive 2011/62/EU
- WIA-medical-data-privacy, WIA-medical-iot, WIA-medical-imaging,
  WIA-network-security, WIA-pq-crypto, WIA-supply-chain

---

## §1 EHR integration

EHR systems consume adherence data via:

- **FHIR subscription** — for clinical-alert deviations and
  administration events affecting active care plans
- **FHIR bulk export** — for adherence-summary delivery to
  the EHR's results inbox on a documented cadence
- **HL7 v2 ORU^R01** — for legacy EHRs

The deployment policy declares which patterns are active.
Non-adherence patterns surface in the EHR as care-plan tasks
for the responsible clinician.

## §2 Pharmacopoeia roster sync

Pharmacopoeia codes are sourced from:

- RxNorm (US National Library of Medicine) — daily refresh
- KD (K-MFDS Korean drug code) — daily refresh
- ATC (WHO) — quarterly refresh
- SNOMED CT pharmacy subset — quarterly refresh
- Manufacturer-attestation for new products not yet in any
  registry

The boundary refreshes on the documented cadence; the
deployment policy declares the refresh schedule. Missing
codes block dispense workflow until added; the deployment
SHOULD have an exception path for emergency formulary
addition signed by the chief pharmacist.

## §3 Pharmacy systems integration

Pharmacy systems integrate with:

- **In-hospital pharmacy** — full FHIR + HL7 v2 support;
  bidirectional flow including dispense reporting and refill
  approval
- **Retail pharmacy chains** — FHIR + chain-specific webhooks
  for refill workflow
- **Mail-order pharmacy** — FHIR + extended fulfilment
  timestamps for shipping coordination

Each pharmacy partner maintains a partner-roster entry; the
boundary verifies signed partner credentials before accepting
dispense events.

## §4 Smart-device manufacturer pipelines

Manufacturers of smart pill bottles, blister packs, and
ingestion sensors integrate via:

- Device firmware update channels (cross-reference to
  WIA-supply-chain for signed update manifests)
- Aggregated adherence metrics for product improvement
  (anonymised per WIA-medical-iot PHASE 4 §6 thresholds)
- Failure-mode reports per IEC 62304 §6.2

## §5 Clinical data warehouse integration

The data warehouse consumes:

- Bulk exports of dispense / administration / deviation /
  adherence-summary records
- Subject identifiers are pseudonymous per PHASE 1 §11
- Aggregation at population scale for outcomes research,
  pharmacoepidemiology, and value-based-care analytics

The warehouse honours de-identification job records; datasets
expire per the consent's declared period.

## §6 Regulator audit integration

Regulators receive structured reports:

- US DEA / state PMP — controlled-substance dispensation
- US FDA REMS — risk-evaluation-and-mitigation reporting for
  REMS-managed medications (clozapine, isotretinoin,
  thalidomide, etc.)
- EU Falsified Medicines Directive — pack-level verification
  records for retail dispense
- KR M-FDS 마약류 통합관리시스템 — controlled-substance
  reporting

Each regulator-export endpoint is gated by a release authority
specific to the jurisdiction.

## §7 Pharmaceutical post-market surveillance

Pharmaceutical companies receive:

- Aggregate adherence rates by medication and indication
- Adverse-event correlation with adherence patterns
  (where consented for pharmacovigilance purpose)
- Dispense channel mix by region
- New-prescription rates following formulary updates

Aggregation thresholds prevent re-identification: cohorts
fewer than 30 patients are suppressed.

## §8 Operational SLAs

| Concern                                          | Default SLA                |
|--------------------------------------------------|----------------------------|
| Prescription mutation acceptance p95              | ≤ 1 s                       |
| Dispense event ingest p95                         | ≤ 500 ms                    |
| Administration ingest p95                         | ≤ 500 ms                    |
| Clinical-alert deviation propagation p95          | ≤ 2 s                       |
| Adherence summary computation                     | ≤ 30 s for 30-day window    |
| EHR FHIR subscription delivery p95                | ≤ 5 s                       |
| Bulk export for ≤ 10K records                     | ≤ 60 s                      |
| Audit chain entry available after operation       | ≤ 10 s                      |
| Controlled-substance regulator report             | per jurisdiction (typically ≤ 24 h after dispense) |

Tighter SLAs are negotiable per deployment; loosening them
requires clinical-leadership sign-off.

## §9 Quarterly compliance report

The boundary emits a quarterly compliance report covering:

- Total prescriptions written by indication
- Dispense events by pharmacy partner
- Administration events by evidence source class
- Deviation rates by severity and medication category
- Adherence-summary distributions
- Refill workflow completion times
- Controlled-substance reporting timeliness
- Cross-domain references honoured vs. refused
- Audit-chain integrity

The report is signed and is itself in scope for the audit chain.

## §10 Acceptance criteria

A deployment claims conformance when:

1. Pharmacopoeia roster is current and verifiable
2. Every dispense in the past quarter has a matching audit
   chain entry with verifiable inclusion proof
3. Controlled-substance reporting timeliness matches the
   jurisdiction's requirement
4. EHR integration delivers within SLA across at least 95%
   of clinical-alert deviations
5. Quarterly compliance report has no integrity-check failures
6. Cross-domain references resolve at the partner boundary
   for ≥ 99% of bound records

A deployment failing any of these reports the gap in its
compliance package.

## §11 Common pitfalls (informative)

- **Pharmacopoeia drift** — manufacturers register new SKUs
  before national codes are assigned; the deployment SHOULD
  have a fast-path for emergency formulary addition
- **Dose timing tolerance** — overly-narrow tolerance windows
  generate alarm fatigue; the deployment SHOULD review timing
  windows quarterly
- **Self-reported reliability** — patient self-reports are
  inherently lower-confidence; the deployment SHOULD weight
  evidence-source mix in the adherence-summary computation
- **Refill-supply double-counting** — overlapping prescriptions
  for the same medication can produce double-count of supply;
  the boundary handles this via the active-prescription set
  computed at supply-threshold check
- **Smart-device battery exhaustion** — devices stop emitting
  events without an explicit failure signal; the boundary
  detects via heartbeat-timeout and surfaces to patient

## §12 Decommissioning

When a deployment is decommissioned:

1. Active prescriptions are transferred to the receiving system
2. Patient-paired smart devices are unbound with patient
   notification
3. Data warehouse retention continues per consent scope
4. Controlled-substance records are sealed and archived per
   regulatory retention
5. Audit chain is sealed and a final root is published

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Cross-domain reference table (informative)

| Reference                  | Use site                                                  | Gate applied                                         |
|----------------------------|-----------------------------------------------------------|------------------------------------------------------|
| WIA-medical-data-privacy   | every adherence record references consent                 | medical-side consent + adherence-side purpose       |
| WIA-medical-iot            | smart bottles / blister packs / ingestion sensors          | medical-iot device association                       |
| WIA-network-security       | TLS cipher-suite floor                                     | network-security-side floor on each session          |
| WIA-pq-crypto              | post-quantum migration phase                               | pq-crypto-side phase declaration current             |
| WIA-supply-chain           | smart-device firmware-update manifest verification         | supply-chain-side device manifest                    |

## Annex B — REMS programme integration (informative)

For REMS-managed medications (clozapine, isotretinoin,
thalidomide), the boundary verifies REMS programme
enrolment before dispense:

1. Pharmacy submits dispense intent
2. Boundary queries REMS-specific verification endpoint
3. On success, boundary records the dispense
4. On failure, dispense is refused with REMS-specific
   problem-detail URI

## Annex C — Decommissioning checklist (informative)

- [ ] Active prescriptions transferred
- [ ] Smart devices unbound
- [ ] Pharmacy partners notified
- [ ] Data warehouse retention continues per consent
- [ ] Controlled-substance records archived per retention
- [ ] Audit chain sealed and final root published
- [ ] Manufacturer notifications filed per warranty terms

## Annex D — Conformance disclosure

Sections §1, §3, §5, §8, §9, §10 are mandatory. §6 is mandatory
for jurisdictions or contracts requiring controlled-substance
reporting. §7 is mandatory where the deployment shares data
with pharmaceutical post-market surveillance partners.

## Annex E — Worked deviation escalation (informative)

A patient on tacrolimus (anti-rejection, time-critical)
misses the 8 AM dose:

1. 08:30 boundary detects no MedicationAdministration matching
   the scheduled dose
2. Boundary creates Deviation with severity: clinical-alert
3. CDS subscription delivers to renal-transplant care team
4. Care team app receives task; clinician calls patient
5. Patient takes the dose at 09:15; reports via app
6. App posts MedicationAdministration with
   evidenceSource: patient-self-report
7. Boundary records administration; deviation severity
   downgrades to "informational"
8. Adherence summary recomputes; PDC remains within band
9. Clinician closes the task with note about patient's
   morning routine

The audit chain captures every step so retrospective review
can reconstruct how the alert was handled.

## Annex F — REMS programme worked example (informative)

For a clozapine prescription (US REMS programme):

1. Prescriber submits MedicationRequest
2. Boundary verifies REMS programme enrolment for prescriber
   and patient at the REMS verification endpoint
3. On success, prescription is admitted; on failure,
   refused with REMS-specific problem-detail URI
4. Pharmacy submits MedicationDispense
5. Boundary verifies REMS programme dispensing pharmacy
   authorisation
6. Boundary records the dispense and reports to REMS-
   programme reporting per cadence

## Annex G — Pharmacy partner-roster entry (informative)

```json
{
  "partnerRosterEntry": {
    "partnerId": "urn:wia:mdadh:partner:retail-chain-x",
    "partnerKind": "retail-pharmacy-chain",
    "fhirEndpoint": "https://retail-x.example/fhir",
    "v2Endpoint": "mllp://retail-x.example:2575",
    "credentials": {
      "tlsClientCertSubject": "CN=retail-x-pharmacy, O=Retail X",
      "signingKey": "https://retail-x.example/.well-known/jwks.json"
    },
    "supportedFlows": ["dispense", "refill-approval", "transfer-in"],
    "validNotAfter": "2027-01-01T00:00:00Z"
  }
}
```

## Annex H — Pharmacovigilance partnership (informative)

A pharmaceutical company subscribed to pharmacovigilance flow
receives:

- AdverseEvent + linked-administration bundles within 24 hours
- Aggregate adherence summaries quarterly
- New-prescription analytics monthly

The partnership contract declares the data scope and the
pharmaceutical-company-side responsibilities for reporting
back to regulators (US FDA MedWatch, EudraVigilance, KR 의약품안전관리원).

## Annex I — Decommissioning checklist (informative)

- [ ] Active prescriptions transferred to receiving deployment
- [ ] Smart devices unbound with patient notification
- [ ] Pharmacy partners notified per partnership contract
- [ ] Data warehouse retention continues per consent scope
- [ ] Controlled-substance records archived per regulatory retention
- [ ] Audit chain sealed and final root published
- [ ] Manufacturer warranty / service contracts wound down

A complete decommissioning report is filed with the controlling
regulator if required by jurisdiction.
