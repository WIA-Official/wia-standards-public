# WIA-medication-adherence PHASE 2 — API Interface Specification

**Standard:** WIA-medication-adherence
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the HTTP API surface a medication-adherence
boundary exposes to pharmacy systems, in-hospital eMARs,
ambulatory clinics, patient-facing apps, smart-pill-bottle
gateways, ingestion-sensor processors, regulator audit
clients, and pharmaceutical post-market surveillance teams.
The shape is FHIR R5 RESTful, layered with adherence-specific
resources for scheduled-dose, deviation, refill, and adherence-
summary management.

References (CITATION-POLICY ALLOW only):
- HL7 FHIR R5 — RESTful API, MedicationRequest,
  MedicationDispense, MedicationAdministration, MedicationStatement,
  Subscription, Bundle, OperationOutcome
- HL7 SMART App Launch 2.2
- HL7 v2.x RDS, RAS, RGV, RDE messages
- IHE Pharmacy Hospital Medication Workflow profile
- IETF RFC 9457 (Problem Details), RFC 8615 (well-known URIs),
  RFC 7515 (JWS), RFC 7519 (JWT), RFC 8259 (JSON), RFC 3339

---

## §1 Capability discovery

```
GET /metadata HTTP/1.1
Accept: application/fhir+json
```

FHIR CapabilityStatement augmented with WIA extensions:
declared pharmacopoeia (RxNorm + KD + ATC + SNOMED CT +
deployment-vocab), evidence-source classes accepted, time-
critical medication flag rules, deviation-severity mapping,
controlled-substance handling jurisdiction.

```
GET /.well-known/wia/medication-adherence HTTP/1.1
```

Returns deployment policy summary: pharmacopoeia roster, formulary
references, dispensing-pharmacy roster, IEC 80001-1 risk-file
reference for the device-coupled portion, integration with
pharma-vigilance partners.

## §2 Prescription lifecycle

```
POST /MedicationRequest HTTP/1.1
Authorization: Bearer <prescriber-jwt>
Content-Type: application/fhir+json
WIA-Purpose-Of-Use: TREAT
```

Body is a FHIR MedicationRequest per PHASE 1 §3. Boundary
verifies:

1. Active consent for `TREAT` purpose
2. Medication code in pharmacopoeia roster
3. Prescriber is currently licensed in the deployment's
   jurisdiction
4. Controlled-substance scheduling rules satisfied

```
PUT /MedicationRequest/<id>/$cancel HTTP/1.1
PUT /MedicationRequest/<id>/$renew HTTP/1.1
PUT /MedicationRequest/<id>/$hold HTTP/1.1
```

`$renew` creates a new MedicationRequest linked to the
prior; the prior remains version-history-preserved.

## §3 Dispense events

```
POST /MedicationDispense HTTP/1.1
Authorization: Bearer <pharmacist-jwt>
Content-Type: application/fhir+json
```

Pharmacy submits the dispense; boundary verifies the
prescription is active and dispense fits the dispense-request
parameters. Lot tracking enables recall-scope queries
(PHASE 4 §6).

For controlled substances, the boundary verifies the
pharmacist's authority for the scheduled medication and
emits a regulator-trackable record (per US DEA, K-MFDS
controlled-substance reporting, etc.).

## §4 Scheduled-dose and administration

```
POST /MedicationAdministration HTTP/1.1
GET  /Patient/<self>/$dose-schedule?from=...&to=...
PUT  /MedicationAdministration/<id>/$amend HTTP/1.1
```

The boundary computes the patient's expected schedule from
active prescriptions; the schedule is queryable for app-side
reminders. Administrations from any evidence source flow to
this endpoint. Amendments allow correcting a wrongly-attributed
administration; the prior record is preserved in version
history.

## §5 Deviation handling

```
POST /Deviation HTTP/1.1
PUT  /Deviation/<id>/$acknowledge HTTP/1.1
PUT  /Deviation/<id>/$resolve HTTP/1.1
GET  /Patient/<subjectRef>/Deviation?since=...&severity=clinical-alert
```

The boundary creates deviation records automatically on
schedule-vs-administration mismatch; manual deviation
submission is allowed for caregiver-reported context.

## §6 Refill workflow

```
POST /Refill HTTP/1.1
PUT  /Refill/<id>/$approve HTTP/1.1
PUT  /Refill/<id>/$dispense HTTP/1.1
GET  /Patient/<subjectRef>/Refill?status=pending
```

Refill workflow integrates with the pharmacy system:
boundary creates a refill on supply-threshold trigger;
pharmacy approves and dispenses; boundary records the new
supply window.

## §7 Adherence summaries

```
GET /Patient/<subjectRef>/AdherenceSummary?period=30d
GET /Patient/<subjectRef>/AdherenceSummary/<summaryId>
POST /AdherenceSummary/$compute HTTP/1.1
```

`$compute` triggers an on-demand computation; the boundary
caches summaries to avoid recomputation on every read.
Summaries are signed and verifiable by clinical-decision-
support tools.

## §8 Search filtering and minimum necessary

Search endpoints filter to consented subset per WIA-medical-data-
privacy policy. Records filtered out do not appear in the response
or in counts; the response carries a `link` of `relation:
prohibited` for transparency.

## §9 Error model

| URI                                                       | Status | Meaning                                          |
|-----------------------------------------------------------|-------:|--------------------------------------------------|
| `urn:wia:mdadh:problem:medication-not-recognised`         | 422    | medication code not in pharmacopoeia roster      |
| `urn:wia:mdadh:problem:dispense-not-prescribed`           | 422    | dispense outside prescription                    |
| `urn:wia:mdadh:problem:refill-exhausted`                  | 403    | refill beyond prescription repeats               |
| `urn:wia:mdadh:problem:controlled-substance-authority-required` | 403 | pharmacist lacks scheduled-substance authority |
| `urn:wia:mdadh:problem:no-active-consent`                 | 403    | no consent matches purpose                       |
| `urn:wia:mdadh:problem:prescriber-not-licensed`           | 403    | prescriber license invalid in jurisdiction       |
| `urn:wia:mdadh:problem:device-not-paired`                 | 422    | smart device claim fails pairing verification    |

## §10 Bulk export

```
GET /$export?_type=MedicationDispense,MedicationAdministration,Deviation&purpose=HRESCH&consentBundle=<id>
```

Same minimum-necessary discipline as sibling standards.

## §11 Subject self-service

```
GET  /Patient/<self>/$adherence-overview
POST /Patient/<self>/$self-report-administration
```

The patient-facing app uses self-report to add evidence; the
boundary tags the record with `evidenceSource: patient-self-
report` so reliability bands apply.

## §12 Subscriptions for clinical-decision-support

Clinical-decision-support tools subscribe to deviations of
clinical-alert severity for their patient cohort:

```
POST /Subscription HTTP/1.1
{
  "topic": "https://wia.example/SubscriptionTopic/medication-adherence-clinical-alerts",
  "channel": {"type": "rest-hook", "endpoint": "https://cds.example/wia-webhook"},
  "filterBy": [{"resourceType": "Deviation", "filterParameter": "severity", "value": "clinical-alert"}]
}
```

Delivery uses TLS 1.3 with detached JWS in `Wia-Signature`.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Worked clinical-alert delivery (informative)

```json
{
  "resourceType": "Bundle",
  "type": "history",
  "entry": [
    {"resource": {"resourceType": "Deviation", "id": "d-91a7", "deviationKind": "missed", "severity": "clinical-alert", "scheduledDoseRef": "urn:wia:mdadh:scheddose:rx-91a7:s-15", "detectedAt": "2026-04-28T08:30:00+09:00"}},
    {"resource": {"resourceType": "MedicationRequest", "id": "rx-91a7", "status": "active", "medicationCodeableConcept": {"coding": [{"system": "http://www.nlm.nih.gov/research/umls/rxnorm", "code": "311036", "display": "tacrolimus 1 MG Oral Capsule"}]}}}
  ]
}
```

The CDS tool receives the bundle, opens a clinical task for
the renal-transplant care team, and the boundary records the
delivery in the audit chain.

## Annex B — Pagination and rate limiting (informative)

List endpoints paginate at ≤ 500 results per page. Per-token
rate limits default to 100 dispense/administration calls per
minute, 10 prescription mutations per minute. Rate-limit
refusals carry `urn:wia:mdadh:problem:rate-limited`.

## Annex C — Patient-app consent flow (informative)

The patient-facing app requests SMART-launch with scopes
appropriate to adherence (read on the patient's prescriptions,
write on self-report administrations, read on the patient's
deviations). The deployment's identity provider issues the
token; the app stores it under a secure-enclave keychain.

A patient who revokes consent at WIA-medical-data-privacy
sees the app degrade gracefully: prescriptions are no longer
visible but historical self-reports are preserved at the
boundary for audit.

## Annex D — Conformance disclosure

Sections §1, §2, §3, §4, §5, §6, §7 are mandatory. §8, §10,
§11 are mandatory where the deployment offers the corresponding
flow (warehouse, bulk-export, patient-app). §12 (Subscriptions)
is mandatory where clinical-decision-support is integrated.

## Annex E — Worked refill workflow (informative)

```
1. Boundary detects: prescription rx-91a7 has 7 days supply remaining
2. Boundary creates Refill resource with triggerKind: days-supply-threshold
3. Pharmacy partner subscribed to Refill?status=pending receives webhook
4. Pharmacy reviews; if approved, posts $approve with pharmacist signature
5. On dispense, pharmacy posts MedicationDispense linked to refill
6. Boundary updates refill.status to "completed" and recomputes supply window
```

The patient-facing app shows the refill in flight; on dispense
completion, the app receives a delivery notification and the
adherence-summary is recomputed.

## Annex F — Mail-order timing (informative)

For mail-order pharmacies, the boundary tracks shipping events:

| Event                      | Source                             |
|----------------------------|------------------------------------|
| Order placed               | pharmacy-side dispense intent      |
| Order packed               | pharmacy fulfilment                |
| Carrier pickup             | shipping carrier API               |
| In-transit                 | shipping carrier API               |
| Delivered                  | shipping carrier API + patient acknowledgement |

The supply window starts at delivery rather than at dispense
to avoid premature refill triggers during transit.

## Annex G — Adherence-summary computation cache

The boundary caches summaries per patient per prescription per
period. Cache invalidation triggers:

- New administration affecting the period
- New deviation affecting the period
- Prescription mutation
- Refill event
- Manual `$compute` invocation

Cached summaries carry a TTL; expired summaries are recomputed
on next read.

## Annex H — Idempotency and conflict handling

All write endpoints accept `Idempotency-Key`; the boundary
stores keys for 30 days. A retry of a prior request with the
same key returns the original response. A different request
body with the same key returns
`urn:wia:mdadh:problem:idempotency-conflict` (409). For batch
operations, partial successes are not supported — either the
entire batch succeeds or all entries fail with detailed
per-entry problems in the response bundle.

## Annex I — Patient-app emergency-contact mode

When a patient enables emergency-contact mode in the patient
app, deviations of `clinical-alert` severity are also delivered
to the patient's nominated emergency contact (typically a
caregiver or family member). The contact receives a redacted
notification (medication-class only, not the specific
medication) and a callback number to the deployment's clinical
team. The notification contract is governed by the patient's
explicit consent at WIA-medical-data-privacy.
