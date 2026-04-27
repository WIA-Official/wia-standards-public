# WIA-nutrition-tracking PHASE 4 — Integration Specification

**Standard:** WIA-nutrition-tracking
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE specifies how a nutrition-tracking deployment
integrates the data, APIs, and protocols from PHASEs 1–3
with broader operational systems: external food databases,
EHRs (clinical), wearable & continuous glucose monitor
integrations, drug-interaction reference services,
governmental nutrition-surveillance reporting, and the
common pitfalls of running a nutrition-tracking deployment
at scale.

References (CITATION-POLICY ALLOW only):
- HL7 FHIR R5 — NutritionOrder, NutritionIntake,
  Observation
- USDA FoodData Central — REST API + bulk download
- Open Food Facts — Open Database License
- KR 식약처 식품영양성분 데이터베이스
- WIA-medical-data-privacy, WIA-medication-adherence,
  WIA-medical-iot, WIA-identity-management,
  WIA-pq-crypto

---

## §1 External food-database integration

The deployment integrates with one or more external
food databases:

- **USDA FoodData Central** — REST API + quarterly bulk
  download for offline cache; the deployment caches
  Foundation, SR Legacy, and Branded Foods in full
- **KR 식약처 식품영양성분DB** — REST API; full snapshot
  for Korean-deployment cache
- **JP 食品成分データベース** — bulk download
- **Open Food Facts** — bulk download under ODbL; the
  deployment honours the share-alike obligation in any
  derivative product
- **EUROFIR (European Food Information Resource)** —
  per-jurisdiction national food composition databases
- **Custom-curated** — per-deployment additions for
  region-specific foods not in upstream databases

Database freshness is monitored daily; lag beyond 30 days
on any source triggers a maintenance ticket.

## §2 EHR integration (clinical deployments)

For clinical-nutrition deployments, the boundary
integrates with the hospital EHR via FHIR R5:

- **NutritionOrder** — dietitian prescriptions push to
  EHR as NutritionOrder resources
- **NutritionIntake** — actual intake events as
  NutritionIntake resources (or Observation, depending
  on EHR profile)
- **AllergyIntolerance** — allergen records as
  AllergyIntolerance resources
- **Patient** — pseudonym ↔ MRN binding via the EHR's
  master patient index

Push or pull is per the EHR's preference; OAuth2 + SMART
on FHIR scopes mediate access for the bidirectional
operations.

## §3 Wearable & CGM integration

For deployments correlating intake with physiological
signals:

- **Continuous glucose monitors (Dexcom, Abbott, Medtronic,
  Apollo)** — post-prandial glucose response correlation
  with carbohydrate intake; integration via
  WIA-medical-iot CGM endpoints
- **Smartwatch heart-rate** — meal timing in workout
  windows; integration via WIA-medical-iot wearable
  endpoints
- **Smart scales** — weight tracking correlated with
  caloric intake; per-deployment policy on data sharing

CGM-correlation analytics produce per-meal glucose-area-
under-curve (gAUC) summaries, surfacing high-glycaemic
foods for individual subjects.

## §4 Drug-interaction reference integration

The deployment integrates with one or more drug-nutrient
interaction databases:

- Stockley's Drug Interactions
- Lexicomp Drug Information
- KR 의약품 안전정보 (KFDA)
- NIH Office of Dietary Supplements — Drug Interactions

Reference license terms (per-vendor) govern usage;
freemium tiers may exclude certain high-significance
interactions from the deployment's surface.

## §5 Public-health surveillance reporting

For deployments contributing to public-health surveillance:

- **WHO Global Dietary Database** — anonymised aggregate
  contributions
- **NHANES (US)** — per-deployment opt-in for
  research-grade contribution
- **KR 국민건강영양조사** — per-deployment opt-in
- **JP 国民健康・栄養調査** — per-deployment opt-in

Surveillance reporting is **always** anonymised and
aggregated; per-subject data never leaves the deployment
without explicit consent.

## §6 Recipe-database integration

For deployments offering recipe analysis:

- **Edamam Recipe API** — recipe ingredient parsing
- **Spoonacular** — recipe nutrient analysis
- **Custom recipe parser** — for deployments with
  proprietary recipe libraries

Recipe-derived nutrient totals are stored as composite
food entries with `componentRefs[]` pointing to the
ingredients per PHASE 1 §2.

## §7 Operational SLAs

| Concern                                | Default SLA                |
|----------------------------------------|----------------------------|
| Food query (cached)                    | ≤ 50 ms p95                |
| Food query (upstream-fallback)         | ≤ 500 ms p95               |
| Intake POST                            | ≤ 200 ms p95               |
| Image-recognition response             | ≤ 2 s p95                  |
| CGM correlation pipeline               | ≤ 60 s after intake        |
| Webhook delivery                       | ≤ 10 s p95 to subscriber   |
| Pattern-summary generation (daily)     | ≤ 5 min                    |

## §8 Quarterly compliance report

The boundary emits a quarterly compliance report:

- Active subject count by jurisdiction
- Intake event volume by meal category
- Allergen-warning fire rate by severity
- Override-acknowledgement rate (signal for clinical
  review)
- Drug-interaction detection rate
- Image-recognition correction rate
- Food-database roster freshness
- Pseudonym re-identification request count
- Audit-chain integrity check results

For clinical deployments, the report includes per-
dietitian prescription volume and adherence statistics
(under the deployment's clinical-leadership policy).

## §9 Acceptance criteria

A deployment claims conformance when:

1. Every fielded food reference resolves to a database
   entry or a documented user-defined override
2. Every active allergen record gates intake events as
   per PHASE 3 §3
3. Every clinical-deployment dietary-prescription is
   signed by a credentialled dietitian
4. Drug-nutrient interaction screening is active for any
   subject with a linked WIA-medication-adherence
   prescription
5. Food-database roster freshness is within the
   deployment's policy limit
6. Audit chain integrity check passes for the prior
   quarter
7. Quarterly compliance report has no integrity-check
   failures

## §10 Common pitfalls (informative)

- **Roster drift** — food databases update silently;
  deployments SHOULD subscribe to release notifications
  and refresh within 30 days of release
- **Branded-food coverage gaps** — long tail of branded
  products NOT in any open database; deployments SHOULD
  collect user-submitted branded foods and curate them
  into the deployment's local extension
- **Image-recognition cultural bias** — most public
  models trained predominantly on Western foods; KR / JP
  / SE-Asia / African cuisine recognition has known gaps
  that deployments SHOULD measure and document
- **Self-report under-reporting** — well-documented
  effect (subjects under-report 10-30% by energy);
  deployments SHOULD surface this caveat in clinical
  reports
- **Recipe-decomposition error** — composite-food
  ingredient ratios are estimates; clinical use cases
  benefit from weighing ingredients separately
- **Allergen-label vagueness** — "may contain" warnings
  are not strict allergen identification; deployments
  SHOULD treat these as caution-level for severe-allergy
  subjects

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Cross-domain reference table

| Reference                  | Use site                                                |
|----------------------------|---------------------------------------------------------|
| WIA-medical-data-privacy   | clinical consent + PHI gating                            |
| WIA-medication-adherence   | drug-nutrient interaction surfacing                     |
| WIA-medical-iot            | CGM, smart-scale, wearable correlation                  |
| WIA-identity-management    | dietitian credential verification                        |
| WIA-pq-crypto              | post-quantum migration phase                             |

## Annex B — Decommissioning checklist

When a nutrition-tracking deployment winds down:

- [ ] Active subjects notified with data-export options
- [ ] Clinical deployments transfer prescriptions to
      successor system
- [ ] EHR integrations closed cleanly (NutritionOrder
      cancellations propagated)
- [ ] Image archive retention per the deployment's
      privacy policy
- [ ] Audit chain sealed and final root published
- [ ] Custom-curated food entries published as
      open-source contributions where licence allows

## Annex C — Conformance disclosure

Sections §1, §2 (clinical only), §4, §7, §8, §9 are
mandatory. §3 (CGM/wearable) is mandatory for deployments
offering correlation features. §5 (surveillance) is
optional. §6 (recipe analysis) is mandatory for
deployments offering recipe features.

## Annex D — Migration from legacy diary apps (informative)

Common migration patterns:

1. Import historical intake records from legacy app's
   export format
2. Map legacy food references to canonical USDA / OEFF /
   national-database references; flag unmappable items
   for user review
3. Recompute nutrient totals under current roster
4. Preserve original totals as `legacyComputed` for
   traceability
5. Surface migration-quality summary to user before
   committing

The migration audit trail is itself audit-chained so
post-migration reviews can examine the mapping decisions.

## Annex E — Multi-language UX considerations

Nutrition tracking deployments often serve multilingual
populations:

- Food-name search supports per-deployment language
  preferences
- Synonym expansion (e.g., 김치 ↔ kimchi ↔ Korean fermented
  cabbage) honours both the user's language and the
  database's canonical form
- Branded-food search by GTIN works language-agnostic
  (preferred for international packaged goods)
- Recipe ingredient text-parsing relies on per-language
  parsers; deployments SHOULD document covered languages

## Annex F — Halal / Kosher / Vegan compliance signals

For subjects with cultural-preference filters:

- Halal — checks alcohol content, pork derivatives,
  gelatin source, halal-certification of branded products
- Kosher — checks meat-dairy separation, kosher
  certification of branded products
- Vegan — checks animal-derived ingredients, including
  hidden derivatives (gelatin, casein, whey, honey,
  carmine)

Branded products carry certification info from upstream
sources where available; ingredient-level inference is
applied for non-branded composite foods.

## Annex G — Energy-balance integration

For deployments offering energy-balance feedback:

- Caloric intake from PHASE 1 §4 totals
- Caloric expenditure from WIA-medical-iot wearable
  cross-reference (resting metabolic rate + activity)
- Net energy balance computed daily; trends surfaced
  to the subject

The boundary's energy-balance computation is informational
and does not constitute clinical advice; clinical
deployments SHOULD route energy-balance interpretation
through registered dietitians.
