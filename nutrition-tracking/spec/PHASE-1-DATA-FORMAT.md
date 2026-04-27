# WIA-nutrition-tracking PHASE 1 — Data Format Specification

**Standard:** WIA-nutrition-tracking
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for nutrition-
tracking systems: food-item identity, intake events, nutrient
profile records, dietary-pattern summaries, dietary-prescription
records (clinical), and the cross-references binding food
items, intake, and patient/subject context. The shape
interoperates with USDA FoodData Central, Open Food Facts,
HL7 FHIR R5 NutritionOrder / NutritionIntake, and the WHO
Codex Alimentarius food-categorisation system so existing
nutritionist software and EHR systems adopt this PHASE
without parallel data models.

References (CITATION-POLICY ALLOW only):
- USDA FoodData Central — Food and Nutrient Database for
  Dietary Studies (FNDDS), Standard Reference (SR Legacy),
  Branded Foods, Foundation Foods
- HL7 FHIR R5 — NutritionOrder, NutritionIntake,
  NutritionProduct (R5/nutritionproduct.html)
- Codex Alimentarius (FAO/WHO) — General Standard for the
  Labelling of Pre-packaged Foods (CXS 1-1985)
- WHO/FAO Dietary Reference Intakes (DRI) framework
- US FDA 21 CFR §101 — Nutrition Facts label requirements
- EU Regulation 1169/2011 — Food Information to Consumers
- KR 식약처 식품 영양성분 표시기준 — KFDA labelling
- ISO 22000:2018 — Food safety management systems
- GS1 GTIN — for branded food product identification
- IETF RFC 8259 (JSON), RFC 7515 (JWS), RFC 3339

---

## §1 Scope

This PHASE applies to systems that record what a person eats
and drinks: consumer-facing food diaries (mobile apps,
wearable-integrated trackers), clinical-nutrition systems
(hospital meal-tracking, dietitian software), public-health
research panels, and integrated food-service systems.

The standard is jurisdiction-aware: deployments declare
the applicable nutrition-labelling regulation (US FDA,
EU 1169/2011, KR 식약처, JP Consumer Affairs Agency, etc.)
so downstream consumers apply the correct rounding rules,
RDI/DRV references, and serving-size definitions.

In scope: food identity, intake events, nutrient profiles,
dietary patterns, hydration, supplement intake, dietary
prescriptions, allergen tracking. Out of scope: detailed
recipe step-by-step instructions (out-of-domain), food
production traceability (cross-domain to WIA-food-traceability),
body-composition measurements (cross-domain to WIA-medical-iot
or wearables).

## §2 Food-item identity

Foods are identified by structured codes drawn from
existing food-database sources:

- `foodRef` — URN of form `urn:wia:nutr:food:<source>:<id>` where
  `source` is one of {`usda-fdc`, `usda-sr`, `usda-fndds`,
  `gs1`, `oeff` (Open Food Facts), `nz-foodfiles`, `kr-rda`,
  `jp-stcfood`, `eu-eurofir`, `wia` (locally curated)}
- For branded foods: `gs1Gtin` declared as the canonical
  identifier; cross-references to FDC Branded Foods
- For generic foods: cross-reference to FDC Foundation Foods
  or SR Legacy where available
- For composite/recipe foods: `componentRefs[]` enumerating
  each ingredient with quantity

The boundary verifies food references against the
deployment's food-database roster; unrecognised references
are rejected with `urn:wia:nutr:problem:food-not-recognised`
unless the request includes a `userDefinedNutrientProfile`
override (subject to dietitian review).

## §3 Nutrient profile (USDA FDC nutrient list aligned)

Each food's nutrient profile carries the USDA FDC nutrient
codes plus declared serving size:

- `servingSize` — quantity + unit (per Codex CXS 1-1985 §4.5
  serving-size declaration rules)
- `nutrients[]` — per-nutrient records: `nutrientCode`
  (FDC code, e.g., `1003` for protein), `name`, `value`,
  `unit`, `derivationCode` (analytical method per FDC
  derivation-code list), `sourceRef` (literature reference
  if calculated)

Nutrient codes covered (minimum mandatory set, US FDA Nutrition
Facts: 21 CFR §101.9 and EU 1169/2011 Annex XIII aligned):

| FDC code | Nutrient                    | Unit |
|----------|-----------------------------|------|
| 1008     | Energy                      | kcal |
| 1003     | Protein                     | g    |
| 1004     | Total lipid (fat)           | g    |
| 1258     | Fatty acids, total saturated| g    |
| 1257     | Fatty acids, total trans    | g    |
| 2000     | Sugars, total               | g    |
| 1235     | Sugars, added               | g    |
| 1079     | Fiber, total dietary        | g    |
| 1093     | Sodium, Na                  | mg   |
| 1087     | Calcium, Ca                 | mg   |
| 1089     | Iron, Fe                    | mg   |
| 1162     | Vitamin C                   | mg   |
| 1114     | Vitamin D                   | µg   |
| 1109     | Vitamin E                   | mg   |
| 1190     | Folate (DFE)                | µg   |
| 1178     | Vitamin B12                 | µg   |
| 1098     | Magnesium, Mg               | mg   |
| 1095     | Zinc, Zn                    | mg   |

Extended profiles (research-grade) include amino acid
profile (FDC 1210–1235), fatty acid profile (FDC 1257–1294),
carotenoids, individual sugars, and bioactive compounds.
Profile completeness is declared per food via `profileTier`:
`label`, `extended`, `research-grade`.

## §4 Intake event record

Each meal, snack, or beverage emits an intake event:

- `intakeId` — URN
- `subjectRef` — pseudonymous subject identifier
- `consentRef` — URN of governing consent (clinical
  deployments per WIA-medical-data-privacy)
- `consumedAt` — RFC 3339 with offset
- `mealLabel` — closed enum: `breakfast`, `lunch`, `dinner`,
  `snack-am`, `snack-pm`, `snack-evening`, `beverage`,
  `pre-workout`, `post-workout`, `unspecified`
- `items[]` — per-item records:
  - `foodRef` — URN of food item
  - `quantity` + `unit` (with WHO/FAO unit conversion to
    standard grams)
  - `preparationCode` — closed enum: `raw`, `boiled`,
    `steamed`, `fried`, `baked`, `grilled`, `roasted`,
    `microwaved`, `prepared-other`
  - `userOverrides` — explicit nutrient overrides (e.g.,
    fortified or modified preparation)
- `consumptionMethod` — `self-reported`, `barcode-scan`,
  `image-recognition`, `weighed`, `meal-plan-confirmed`,
  `passive-sensor`
- `confidence` — `high` (weighed/scanned) /
  `medium` (logged with photo or recipe) /
  `low` (estimated from text)
- `locationContext` — `home`, `restaurant`, `workplace`,
  `school`, `hospital`, `travel`, `unknown`

Per-item nutrient totals are computed at intake-time from
the food's profile and the quantity. The boundary preserves
both the source food references and the computed totals so
downstream analytics can recompute under updated reference
data.

## §5 Hydration record

Hydration is tracked separately because of its different
clinical interpretation:

- `hydrationId` — URN
- `subjectRef` — pseudonymous subject
- `consumedAt` — RFC 3339
- `volume` + `unit` — standardised to mL
- `beverageRef` — URN of beverage (water, coffee, tea,
  juice, alcoholic, etc.)
- `caffeineMg` — caffeine content if applicable
- `alcoholGrams` — pure alcohol if applicable
- `electrolyteProfile` — sodium/potassium/magnesium content
  if applicable (sports/clinical scenarios)

Total daily fluid intake is computed from hydration plus
the moisture content of solid foods (per FDC water
nutrient code 1051).

## §6 Supplement intake record

Dietary supplements (vitamins, minerals, herbal, protein
powders, etc.) carry distinct semantics:

- `supplementId` — URN
- `subjectRef`
- `consumedAt` — RFC 3339
- `supplementProductRef` — URN of supplement product
  (cross-reference to NIH Dietary Supplement Label Database,
  KR 건강기능식품 데이터베이스, or similar)
- `nutrients[]` — declared nutrient amounts per the supplement
  label
- `prescriptionRef` — URN of prescribing record if clinically
  prescribed (WIA-medication-adherence cross-reference)
- `selfReportedReason` — for non-prescribed supplements, the
  user's stated reason

Supplements are NOT mixed with the food intake totals by
default; analytics surfaces them separately so clinicians
can assess dietary-source vs. supplement-source nutrients.

## §7 Dietary-prescription record (clinical)

For clinical-nutrition deployments:

- `dietaryPrescriptionId` — URN
- `subjectRef` — pseudonymous patient
- `prescribedBy` — registered dietitian / clinician URN
- `consentRef` — URN of governing consent
- `effectiveFrom` / `effectiveUntil` — RFC 3339
- `dietPattern` — closed enum from a clinical taxonomy:
  `cardiac-protective`, `dash`, `mediterranean`,
  `low-sodium`, `low-potassium`, `renal`, `diabetic`,
  `gluten-free`, `lactose-restricted`, `low-fodmap`,
  `texture-modified`, `npo`, `enteral`, `parenteral`,
  `weight-loss-medical`, `weight-gain-medical`,
  `paediatric-failure-to-thrive`, `oncology-supportive`
- `targetNutrients[]` — per-nutrient targets (min/max/target
  with units)
- `allergens[]` — restricted allergens per the patient's
  clinical context
- `culturalPreferences` — closed enum: `vegetarian`,
  `vegan`, `halal`, `kosher`, `jain`, `buddhist`, etc.

Prescriptions are signed by the prescribing clinician
(JWS detached). Adherence analytics compare actual intake
events against prescription targets.

## §8 Dietary-pattern summary

Aggregated dietary-pattern records:

- `patternSummaryId` — URN
- `subjectRef`
- `period` — start / end RFC 3339
- `nutrientAverages[]` — per-nutrient daily mean and
  derived percentage of DRI/RDI
- `mealCadence` — average meals per day, snack frequency
- `foodGroupDistribution` — per food group (Codex categories)
  share of total energy
- `dietQualityScores` — Healthy Eating Index (HEI-2020),
  Mediterranean Diet Score, DASH Score where computable
- `comparisonRef` — reference standard the summary compares
  against (e.g., DRI, dietary prescription, peer cohort)

Pattern summaries do not authorise re-identification; they
inherit the consent of the underlying intake events.

## §9 Allergen and intolerance record

Allergen and intolerance tracking:

- `allergenRecordId` — URN
- `subjectRef`
- `allergens[]` — list per Codex Alimentarius / FALCPA
  big-9 (US) / EU Annex II:
  - `cereals-containing-gluten`
  - `crustaceans`
  - `eggs`
  - `fish`
  - `peanuts`
  - `soybeans`
  - `milk-and-dairy`
  - `tree-nuts`
  - `sulphites` (≥10 ppm)
  - `celery`, `mustard`, `sesame`, `lupin`, `molluscs`
    (EU additional)
  - `sesame-seeds` (US 2023 addition)
- `severity` per allergen: `mild`, `moderate`, `severe`,
  `anaphylactic`
- `reactionType` per allergen: `IgE-mediated`,
  `non-IgE-mediated`, `intolerance`, `auto-immune` (celiac)
- `clinicalEvidenceRef` — diagnostic evidence (skin-prick
  test, serum IgE, oral food challenge, etc.)

Active allergen records gate every intake event: an item
whose ingredients trigger a recorded allergen surfaces a
warning at logging time.

## §10 Food-image evidence

For image-recognition-driven logging:

- `imageEvidenceId` — URN
- `intakeRef` — linked intake event
- `imageHash` — SHA-256 of the captured image
- `imageURI` — storage URI
- `recognitionModelRef` — URN of the recognition model
  version
- `recognitionConfidence` — declared confidence band
- `userVerification` — `unverified`, `confirmed`, `corrected`,
  `rejected`

Images are NEVER required for tracking; deployments may
opt-out for privacy. When used, images are subject to the
deployment's privacy retention policy.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Cross-domain references (informative)

| Reference                  | Use site                                                  |
|----------------------------|-----------------------------------------------------------|
| WIA-medical-data-privacy   | clinical deployments — consent + PHI gating              |
| WIA-medication-adherence   | drug-nutrient interaction surfacing                       |
| WIA-medical-iot            | weight-tracking / glucose-monitor correlation            |
| WIA-food-traceability      | food-source provenance for high-risk items                |

## Annex B — Conformance disclosure

Sections §2, §3, §4, §9 are mandatory. §5 (hydration), §6
(supplements), §10 (image evidence) are mandatory for
deployments offering those features. §7 (dietary prescription)
is mandatory for clinical-nutrition deployments. §8 (pattern
summary) is mandatory for deployments offering analytics.

## Annex C — Worked intake event (informative)

```json
{
  "intakeId": "urn:wia:nutr:intake:app-x:i-2026-04-28-014",
  "subjectRef": "urn:wia:nutr:subject:p-x-PR-014",
  "consumedAt": "2026-04-28T08:15:00+09:00",
  "mealLabel": "breakfast",
  "items": [
    {
      "foodRef": "urn:wia:nutr:food:usda-fdc:1098693",
      "quantity": 240,
      "unit": "g",
      "preparationCode": "raw"
    },
    {
      "foodRef": "urn:wia:nutr:food:gs1:08801062123456",
      "quantity": 1,
      "unit": "serving",
      "preparationCode": "prepared-other"
    }
  ],
  "consumptionMethod": "barcode-scan",
  "confidence": "high",
  "locationContext": "home"
}
```

## Annex D — Versioning and deprecation

Versioning follows SemVer 2.0.0. Food-database roster
updates do not bump the standard version; the boundary
records the roster snapshot at each intake event so
recomputation is consistent under roster changes.

## Annex E — Conformance levels

| Level     | Scope                                                         |
|-----------|---------------------------------------------------------------|
| Surface   | data formats accepted; self-attested                          |
| Verified  | annual third-party audit                                      |
| Anchored  | continuous evidence package + signed roster commitments       |

Implementations declare their level in the capability
document. Clinical deployments typically require Verified
or Anchored.

## Annex F — Worked dietary prescription (informative)

```json
{
  "dietaryPrescriptionId": "urn:wia:nutr:rx:clinic-x:rx-001",
  "subjectRef": "urn:wia:nutr:subject:p-x-PR-014",
  "prescribedBy": "urn:wia:hr:dietitian:RD-7e2c",
  "consentRef": "urn:wia:nutr:consent:c-014-2026-q1",
  "effectiveFrom": "2026-04-28T00:00:00+09:00",
  "effectiveUntil": "2026-07-28T23:59:59+09:00",
  "dietPattern": "diabetic",
  "targetNutrients": [
    {"code": "1008", "min": 1800, "max": 2000, "unit": "kcal"},
    {"code": "1005", "min": 200, "max": 240, "unit": "g"},
    {"code": "1093", "max": 2000, "unit": "mg"},
    {"code": "1235", "max": 25, "unit": "g"}
  ],
  "allergens": ["peanuts"],
  "culturalPreferences": "halal"
}
```

## Annex G — Allergen-warning trigger flow

Each intake event runs through allergen-screening:

1. For each item's `foodRef`, retrieve the food's ingredient
   list from the database
2. Each ingredient cross-checked against the subject's
   active allergen records
3. On any match: surface a warning with severity classification
   from the allergen record
4. Severe / anaphylactic matches refuse the intake until
   user confirmation overrides; the override is itself
   audit-chained

This surfacing is real-time at logging-time; downstream
analytics also produce aggregate allergen-exposure reports.
