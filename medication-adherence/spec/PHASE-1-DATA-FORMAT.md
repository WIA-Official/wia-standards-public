# WIA-medication-adherence PHASE 1 — Data Format Specification

**Standard:** WIA-medication-adherence
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for medication-
adherence systems: smart pill bottles, blister-pack sensors,
ingestion-event sensors, electronic medication-administration
records (eMAR), pharmacy-dispense events, and the patient-
reported adherence reports that feed clinical decision-making.
The shape interoperates with HL7 FHIR R5 medication resources
and with HL7 v2.x pharmacy messages so existing eMAR / pharmacy
deployments adopt this PHASE without parallel data models.

References (CITATION-POLICY ALLOW only):
- HL7 FHIR R5 — MedicationRequest (R5/medicationrequest.html),
  MedicationDispense, MedicationAdministration, MedicationStatement,
  Medication, MedicationKnowledge
- HL7 v2.x RDS (Pharmacy/Treatment Dispense), RAS (Administration),
  RGV (Give)
- IHE Pharmacy Hospital Medication Workflow (HMW)
- WHO Anatomical Therapeutic Chemical (ATC) classification
- RxNorm (US National Library of Medicine), KD code (K-MFDS),
  SNOMED CT 363761000004101 (Medicinal product)
- ISO/IEC 11073-10472:2012 — medication monitor (medication
  reminder/dispenser device specialisation)
- IEEE 11073-10101 — for vital-sign metrics that may correlate
  with adherence (e.g., blood pressure response to therapy)
- ISO 13485:2016 — for the medical-device portions
- IEC 62304:2006/A1:2015 — software life cycle
- IETF RFC 8259 (JSON), RFC 7515 (JWS), RFC 3339

---

## §1 Scope

This PHASE applies to systems that capture or infer the
patient's pattern of taking medications: home-use connected
devices (smart bottles, blister packs, inhalers, injectors),
in-hospital eMAR systems, pharmacy dispense systems, and
patient-reported tracking apps. It standardises the *shape*
of medication identity, dispense events, scheduled dose
events, ingestion or administration events, deviation records,
and adherence summaries.

The standard is jurisdiction-aware: deployments declare the
applicable pharmacy regulation (US 21 CFR §1300, EU Falsified
Medicines Directive, K-MFDS Pharmaceutical Affairs Act, JP
PMD Act, etc.) so downstream consumers apply the correct
controlled-substance handling.

## §2 Medication identity

Medications are identified by structured codes drawn from
existing pharmacopoeia:

- `medicationRef` — URN of form `urn:wia:mdadh:medication:<vocab>:<code>`
  where `vocab` is one of {`rxnorm`, `kd`, `atc`, `snomed`, `wia`}
- For combination products: list of component `medicationRef`s
  plus the strength of each
- For controlled substances: explicit `controlledScheduleRef`
  declaration (US DEA Schedule I-V, KR 향정신성의약품 분류, etc.)
- For multi-dose presentations: declared `dosesPerPresentation`
  and `presentationLot`

The boundary verifies medication codes against the deployment's
pharmacopoeia roster; unrecognised codes are rejected with
`urn:wia:mdadh:problem:medication-not-recognised`. New codes
are admitted via the pharmacopoeia roster update procedure
(PHASE 4 §2).

## §3 Prescription record (FHIR MedicationRequest)

Every adherence-tracking session is bound to a prescription:

| FHIR field        | Binding                                                         |
|-------------------|-----------------------------------------------------------------|
| `status`          | active / on-hold / cancelled / completed / stopped / draft     |
| `intent`          | order / proposal / plan / instance-order                        |
| `medication`      | Reference to a Medication or `medicationCodeableConcept`        |
| `subject`         | pseudonymous patient (cross to medical-data-privacy)           |
| `requester`       | prescribing clinician principal URN                             |
| `dispenseRequest` | quantity + repeats + initial fill + dispense interval          |
| `dosageInstruction[]` | timing (when and how often), route, dose quantity, max-per-period |
| `note[]`          | clinical notes (e.g., "take with food")                         |

Prescriptions are signed by the prescribing clinician (JWS
detached). Renewal of a prescription creates a new
MedicationRequest; the prior version is preserved in version
history because adherence reconstruction must reference the
prescription active at any past timestamp.

## §4 Dispense event (FHIR MedicationDispense)

Pharmacy dispense events:

- `dispenseId` — URN of form `urn:wia:mdadh:dispense:<pharmacy>:<seq>`
- `pharmacyRef` — dispensing pharmacy (GS1 GLN)
- `prescriptionRef` — linked MedicationRequest
- `medicationRef` — actual product dispensed (may differ from
  prescribed medication if substitution per deployment policy)
- `presentationLot` — manufacturer lot number (for recall scope)
- `dispensedAt` — RFC 3339
- `daysSupply` — declared days of supply
- `dispensedBy` — pharmacist principal URN

Dispense events feed the patient's expected dose schedule:
the boundary computes the projected end-of-supply date and
generates refill-due notifications.

## §5 Scheduled dose event

For every prescription's dosage instruction, the boundary
computes a forward schedule of expected dose events:

- `scheduledDoseId` — URN of form `urn:wia:mdadh:scheddose:<rxRef>:<seq>`
- `prescriptionRef` — parent prescription
- `expectedAt` — RFC 3339 of the expected dose time
- `windowStart`, `windowEnd` — tolerance window (e.g.,
  ±30 min for routine, ±5 min for time-critical)
- `expectedDose` — quantity + unit
- `expectedRoute` — oral, sublingual, IV, IM, subcutaneous,
  inhaled, topical, transdermal, ophthalmic, otic
- `marker` — closed enum: `routine`, `prn` (as needed),
  `loading-dose`, `tapering-dose`, `breakthrough`

Scheduled events do not confirm administration; they describe
the prescription's expectation. The actual administration
event (§6) confirms or contradicts the schedule.

## §6 Administration event (FHIR MedicationAdministration)

Each captured dose is an administration event:

- `administrationId` — URN
- `subject` — pseudonymous patient
- `prescriptionRef` — fulfilling prescription
- `scheduledDoseRef` — scheduled-dose this fulfills (or null
  if PRN / unscheduled)
- `medicationRef` — actual medication taken
- `effectiveDateTime` — RFC 3339 of actual administration
- `doseQuantity` — actual dose taken
- `route` — actual route
- `performer` — principal who administered (in-hospital: RN /
  patient self / caregiver)
- `evidenceSource` — closed enum:
  - `device-event` — smart bottle / blister / injector cap
  - `ingestion-sensor` — ingestible sensor (FDA-cleared
    digital-medicine devices)
  - `eMAR-entry` — clinician keystroke in eMAR
  - `patient-self-report` — patient app entry
  - `caregiver-report` — caregiver app entry

Each evidence source carries different reliability bands; the
adherence-summary (§9) accounts for this.

## §7 Deviation record

When a scheduled dose is missed, taken late, or taken at the
wrong dose:

- `deviationId` — URN
- `scheduledDoseRef` — affected scheduled dose
- `deviationKind` — closed enum: `missed`, `late`, `early`,
  `under-dose`, `over-dose`, `wrong-medication`, `wrong-route`
- `severity` — `informational`, `clinical-concern`, `clinical-alert`
  (mapped from prescription's marker)
- `detectedAt` — RFC 3339 of detection
- `resolvedAt` — when the deviation was acknowledged and
  resolved
- `clinicalEscalationRef` — pointer to any clinical-team
  escalation

Time-critical medications (anti-rejection, antibiotics for
sepsis, anti-arrhythmics) flag deviations as
`clinical-alert` immediately; routine medications hold for
the deployment-declared review cadence.

## §8 Refill-and-supply record

Anticipated end-of-supply triggers refill workflow:

- `refillId` — URN
- `prescriptionRef` — parent prescription
- `triggerKind` — closed enum: `days-supply-threshold`,
  `patient-request`, `pharmacy-rebalance`, `formulary-change`
- `triggerDate` — RFC 3339
- `requestedAt`, `dispensedAt` — RFC 3339 timestamps
- `refillCountAvailable` — count remaining on the prescription
- `pharmacyOptions[]` — per the patient's preferences

## §9 Adherence summary

The boundary computes adherence summaries per patient per
prescription on a documented cadence:

- `summaryId` — URN
- `subject`, `prescriptionRef` — scope
- `period` — observation period (e.g., 30 days)
- `mpr` — Medication Possession Ratio: days supplied / days
  in period (capped at 1.0)
- `pdc` — Proportion of Days Covered: days where dose was
  available and (per evidence) administered / total days
- `evidenceMix` — proportion of doses confirmed by each
  evidence source class
- `deviationCount` — count of deviations, by kind and severity
- `interpretation` — `adherent` (PDC ≥ 0.80), `partial`
  (0.50 ≤ PDC < 0.80), `non-adherent` (PDC < 0.50);
  thresholds adjustable per deployment policy

The summary is signed; downstream clinical-decision-support
tools verify the signature before consuming.

## §10 Cross-domain references

| Reference                  | Use site                                                         |
|----------------------------|------------------------------------------------------------------|
| WIA-medical-data-privacy   | every adherence record references the consent record             |
| WIA-medical-iot            | smart bottles / blister packs / ingestion sensors as MedIoT devices |
| WIA-medical-imaging        | rare cross-reference for imaging-confirmed administration         |
| WIA-network-security       | TLS cipher-suite floor for device-side connections               |
| WIA-pq-crypto              | post-quantum migration phase                                     |

## §11 Subject identifier scope

Subject identifiers are pseudonymous per WIA-medical-data-privacy
`subjectRef` shape. The medication-adherence boundary holds no
direct identifiers in audit chains.

## §12 Conformance levels

| Level     | Scope                                                                |
|-----------|----------------------------------------------------------------------|
| Surface   | data formats accepted; self-attested                                 |
| Verified  | annual third-party audit                                             |
| Anchored  | continuous evidence package + IEC 80001-1 risk file when device-coupled |

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Worked dispense + administration sequence (informative)

```json
{
  "resourceType": "MedicationDispense",
  "id": "urn:wia:mdadh:dispense:pharm-001:d-91a7",
  "status": "completed",
  "medication": {"coding": [{"system": "http://www.nlm.nih.gov/research/umls/rxnorm", "code": "314231", "display": "amlodipine 5 MG Oral Tablet"}]},
  "subject": {"reference": "urn:wia:mdp:subject:f4c2-9bd1-7a05-3e8e"},
  "performer": [{"actor": {"reference": "urn:wia:hr:pharmacist:p-77c2"}}],
  "authorizingPrescription": [{"reference": "urn:wia:mdadh:rx:r-91a7"}],
  "quantity": {"value": 30, "unit": "tablet"},
  "daysSupply": {"value": 30, "unit": "d"},
  "whenHandedOver": "2026-04-28T11:30:00+09:00"
}
```

## Annex B — Negative test vectors (informative)

| Stimulus                                              | Expected outcome                                |
|-------------------------------------------------------|-------------------------------------------------|
| Administration without prescription reference         | accepted as PRN if prescription allows; else 422 |
| Dispense for medication outside prescription          | 422 + `dispense-not-prescribed`                 |
| Time-critical missed dose                             | clinical-alert deviation + clinician escalation |
| Refill request beyond prescription's repeat count     | 403 + `refill-exhausted`                        |
| Controlled-substance dispense without DEA-equivalent  | 403 + `controlled-substance-authority-required` |
