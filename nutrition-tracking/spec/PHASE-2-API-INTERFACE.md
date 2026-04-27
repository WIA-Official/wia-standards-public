# WIA-nutrition-tracking PHASE 2 — API Interface Specification

**Standard:** WIA-nutrition-tracking
**Phase:** 2 — API Interface
**Version:** 1.0
**Status:** Stable

This PHASE defines the API surface a nutrition-tracking
deployment exposes for food-database queries, intake-event
publication, hydration / supplement logging, dietary-
prescription management, dietary-pattern analytics, allergen
record management, and image-recognition workflows. The
shape is HTTP/JSON for application clients; high-rate
sensor ingestion (e.g., continuous glucose monitor
correlation) uses the streaming projection in PHASE 3.

References (CITATION-POLICY ALLOW only):
- IETF RFC 9457 (Problem Details), RFC 7515 (JWS), RFC 8259 (JSON)
- HL7 FHIR R5 — RESTful API patterns
- USDA FoodData Central — REST API conventions
- WIA-medical-data-privacy PHASE 2 — for clinical-deployment
  consent gating
- WIA-medication-adherence PHASE 2 — for drug-nutrient
  interaction surfacing

---

## §1 Food-database query

```
GET /foods/{foodRef} HTTP/1.1
Authorization: Bearer <jws-jwt>
Accept: application/json
```

Returns the food's full nutrient profile per PHASE 1 §3.

```
GET /foods?q=<text>&source=usda-fdc&limit=20
GET /foods?gtin=08801062123456
GET /foods?codexCategory=01.1.1
```

Search by free-text, GS1 GTIN (barcode scan), or Codex
category. Free-text search supports per-deployment language
preference; matching uses lexicographic + synonym expansion
where the database supports it.

```
GET /foods/{foodRef}/ingredients
```

Returns the ingredient breakdown for branded / composite
foods. Ingredients themselves resolve to `foodRef`s for
recursive analysis.

## §2 Intake event publication

```
POST /intakes HTTP/1.1
Content-Type: application/json
Idempotency-Key: 7e2c91a7-...
```

Body is a PHASE 1 §4 intake record. The boundary:

1. Validates each `foodRef` exists in the database roster
2. Computes per-item nutrient totals
3. Runs allergen-screening per PHASE 1 Annex G
4. Returns the canonicalised intake with computed totals

Anaphylactic-allergen matches return 422 with
`urn:wia:nutr:problem:allergen-conflict` and require an
explicit `overrideAcknowledged: true` in a follow-up call.

```
GET /intakes/{intakeId}
GET /intakes?subjectRef=...&from=...&to=...&mealLabel=lunch
PUT /intakes/{intakeId}     (correction within edit window)
DELETE /intakes/{intakeId}  (within edit window only)
```

Edit window is deployment-policy-controlled (typically 24h
for self-tracking, 72h for clinical with dietitian sign-off).

## §3 Hydration logging

```
POST /hydration HTTP/1.1
GET /hydration?subjectRef=...&date=2026-04-28
GET /hydration/summary?subjectRef=...&period=7d
```

Hydration summary returns total fluid intake, water
equivalents (per WHO/EFSA hydration guidelines), and
caffeine/alcohol totals.

## §4 Supplement logging

```
POST /supplements HTTP/1.1
GET /supplements?subjectRef=...&from=...
```

The boundary cross-references prescribed supplements
against active drug records in WIA-medication-adherence:
nutrient-drug interactions (e.g., calcium with
levothyroxine, vitamin K with warfarin) are surfaced as
warnings.

## §5 Dietary-prescription management

```
POST /dietary-prescriptions HTTP/1.1
Authorization: Bearer <dietitian-jwt>
```

Body is a PHASE 1 §7 prescription record. The boundary
verifies the prescriber holds an active dietitian
credential per the deployment's identity authority.

```
GET /dietary-prescriptions/{rxId}
GET /dietary-prescriptions?subjectRef=...&active=true
PUT /dietary-prescriptions/{rxId}/state    (mark inactive)
```

State enum: `active`, `superseded`, `cancelled`,
`completed`. State transitions are signed and audit-chained.

## §6 Dietary-pattern analytics

```
GET /patterns?subjectRef=...&from=2026-03-28&to=2026-04-28
```

Returns the PHASE 1 §8 pattern summary for the period.

```
GET /patterns/diet-quality?subjectRef=...&period=30d
GET /patterns/comparison?subjectRef=...&against=dri
GET /patterns/comparison?subjectRef=...&against=prescription
```

Diet-quality scores are computed per the deployment's
chosen scoring system (HEI-2020, Mediterranean, DASH).

## §7 Allergen record management

```
POST /allergens HTTP/1.1
Body: PHASE 1 §9 allergen record
```

```
GET /allergens?subjectRef=...
PUT /allergens/{allergenRecordId}/severity   (clinician-only)
DELETE /allergens/{allergenRecordId}         (with audit)
```

Allergen records are deletable only with a documented
clinical reason (e.g., negative oral-food-challenge result);
deletion is audit-chained.

## §8 Image-recognition workflow

```
POST /images HTTP/1.1
Content-Type: multipart/form-data
```

Body includes the image file and an optional intake-event
context. The boundary:

1. Stores the image in the deployment's image store
2. Submits to the recognition model
3. Returns candidate food matches with confidence scores
4. Optionally pre-fills an intake event with the
   highest-confidence match

```
GET /images/{imageEvidenceId}
PUT /images/{imageEvidenceId}/verification    (user confirms/corrects)
```

User verification updates the recognition model's training
feedback (where consented) per the deployment's
model-improvement policy.

## §9 Drug-nutrient interaction queries

```
GET /interactions?subjectRef=...
```

Returns active drug-nutrient interactions for the subject's
current medications + active intake patterns. Surfaces
clinically-significant interactions per the deployment's
interaction reference (e.g., Stockley's, Lexicomp,
KFDA 의약품 안전성 정보).

## §10 Capability discovery

```
GET /.well-known/wia/nutrition-tracking HTTP/1.1
```

Returns the capability document:

```json
{
  "wia.standardVersion": "1.0",
  "wia.implementationVersion": "app-x-2.4.1",
  "supportedDatabases": ["usda-fdc", "kr-rda", "oeff"],
  "labellingRegime": "kr-mfds",
  "allergenSet": "kr-mfds-22",
  "imageRecognitionEnabled": true,
  "supportedDietPatterns": ["dash","mediterranean","diabetic","renal","low-fodmap"],
  "manifest": "https://app-x.example/.well-known/wia/nutrition-tracking/manifest.jws"
}
```

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Idempotency

POST endpoints accept `Idempotency-Key`. Boundary stores
keys for 24 hours. Replays return the original response.

## Annex B — Pagination

List endpoints support cursor pagination. Cursors are
opaque, signed by the boundary, valid for 30 minutes.

## Annex C — Negative-test vectors (informative)

| Stimulus                                              | Expected response                                |
|-------------------------------------------------------|-------------------------------------------------|
| Intake POST referencing unknown foodRef               | 422 + `food-not-recognised`                      |
| Intake POST with anaphylactic-allergen ingredient     | 422 + `allergen-conflict`                       |
| Intake PUT outside edit window                        | 403 + `edit-window-expired`                     |
| Dietary-prescription POST by non-dietitian            | 403 + `not-authorised-prescriber`               |
| Allergen DELETE without clinical reason               | 422 + `allergen-deletion-requires-reason`       |

## Annex D — Bulk ingest

For migration from legacy systems:

```
POST /intakes/$bulk HTTP/1.1
Content-Type: application/x-ndjson
```

Each line is a PHASE 1 §4 intake record. The boundary
processes in order, returns per-line results. Bulk ingest
is gated by the deployment's bulk-quota policy.

## Annex E — Webhook subscriptions

```
POST /subscriptions HTTP/1.1
```

Event classes: `intake-published`, `prescription-active`,
`prescription-superseded`, `allergen-warning-fired`,
`pattern-summary-ready`, `interaction-detected`.

Webhook delivery uses TLS 1.3 with a detached JWS in
`Wia-Signature`.

## Annex F — Authorities and roles

| Role                  | Scope                                            |
|-----------------------|--------------------------------------------------|
| `subject`             | own data (intake/hydration/supplements/allergens) |
| `dietitian`           | dietary-prescription write + assigned-subject read |
| `clinician`           | clinical-deployment read + interaction queries   |
| `app-developer`       | deployment-only API calls (no subject data)      |
| `auditor`             | read-only across engagement scope                |

## Annex G — Worked image-recognition response

```json
{
  "imageEvidenceId": "urn:wia:nutr:image:app-x:i-2026-04-28-001",
  "candidates": [
    {"foodRef": "urn:wia:nutr:food:kr-rda:K-RICE-WHITE", "confidence": 0.92, "estimatedQuantity": {"value": 210, "unit": "g"}},
    {"foodRef": "urn:wia:nutr:food:kr-rda:K-RICE-MIXED", "confidence": 0.65, "estimatedQuantity": {"value": 220, "unit": "g"}}
  ],
  "preFilledIntakeRef": "urn:wia:nutr:intake:app-x:i-2026-04-28-014"
}
```

The user verifies or corrects via PUT
`/images/{id}/verification`; corrections feed model
improvement subject to the deployment's consent policy.

## Annex H — Capability versioning

Capability documents declare `wia.standardVersion` and
`wia.implementationVersion`. Standard-version mismatch is a
hard refusal; implementation-version mismatch is logged
but not refusing.

## Annex I — Audit-chain replay

Regulators and clinical auditors retrieve a selective
audit-chain replay at `/audit/chain` filtered by kind
(intake-published, prescription-active, allergen-warning,
etc.) and time range. Restricted kinds (e.g., clinical
overrides) require the requester's role to authorise
access.

## Annex J — Bulk-export for research

Research-deployment bulk export:

```
GET /export/intakes?subjectCohort=...&from=...&to=...
```

Honours each subject's consent's data-sharing scope; export
of non-sharing-scoped data is refused.

## Annex K — Deployment-policy override transparency

For audit clarity, every deployment-policy override
(e.g., extending the intake edit window beyond 24h,
suppressing repeat drug-nutrient interaction warnings)
is itself recorded in the capability document and
audit-chained. Subjects and clinicians querying their
deployment's policy receive the current override list
with effective dates and the issuing authority.

The capability document refresh cadence is published in
the deployment's documentation; consumers SHOULD revalidate
weekly to track policy drift.
