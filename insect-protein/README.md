# WIA-AGRI-025: Insect Protein Standard

> 弘益人間 (홍익인간) · Benefit All Humanity

A WIA standard for sustainable insect protein production, processing, traceability, and circular‑economy integration. The standard targets edible‑insect species authorised by the U.S. FDA GRAS pathway, the EU Novel Food Regulation (Regulation (EU) 2015/2283) and the Republic of Korea *Insect Industry Promotion Act* (곤충산업의 육성 및 지원에 관한 법률).

## 1. Scope

This standard covers four operational layers:

1. **Data formats** — JSON schemas for batches, species records, nutrition profiles, safety tests, and sustainability metrics (see `spec/PHASE-1-DATA-FORMAT.md`).
2. **API contract** — REST surface for farms, batches, certifications, and audit trails (see `spec/PHASE-2-API-INTERFACE.md`).
3. **Communication protocol** — exchange semantics between farms, processors, retailers, and regulators (see `spec/PHASE-3-PROTOCOL.md`).
4. **Integration** — supply‑chain, ESG, and traceability hooks (see `spec/PHASE-4-INTEGRATION.md`).

Out of scope: live‑feed bait insects, pet‑only feed mixes that do not enter the human food chain, and pharmaceutical extracts (covered under separate WIA pharma‑grade standards).

## 2. Target species

The standard is species‑agnostic but provides validation profiles for the species most commonly cleared for food and feed use:

- *Tenebrio molitor* (yellow mealworm) — EU novel‑food authorised 2021.
- *Locusta migratoria* (migratory locust) — EU novel‑food authorised 2021.
- *Acheta domesticus* (house cricket) — EU novel‑food authorised 2022.
- *Alphitobius diaperinus* (lesser mealworm) — EU novel‑food authorised 2023.
- *Hermetia illucens* (black soldier fly) — feed‑grade in EU since Regulation (EU) 2017/893; food‑use under review.
- *Bombyx mori* (silkworm pupa, 누에번데기) — Korea food‑grade per MFDS (2016 revision of 식품공전).
- *Protaetia brevitarsis seulensis* (white‑spotted flower chafer larva, 흰점박이꽃무지) — Korea food‑grade since 2016.

## 3. Conformance tiers

| Tier | Use case | Required artefacts | Audit cadence |
|------|----------|--------------------|---------------|
| Tier 1 — Self‑declared | Pilot farms, internal R&D | OpenAPI 3.0 contract test, JSON Schema validation report, HACCP plan | Annual self‑review |
| Tier 2 — Third‑party assessed | Commercial supply, B2B contracts | Tier 1 + accredited lab safety panel (heavy metals, microbiological), ISO 22000:2018 food‑safety records | Every 18 months |
| Tier 3 — Accredited | Public‑facing, regulated retail, school nutrition | Tier 2 + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

## 4. Layout

```
insect-protein/
├── README.md                       (this file)
├── index.html                      (interactive overview + top-nav)
├── spec/
│   ├── PHASE-1-DATA-FORMAT.md
│   ├── PHASE-2-API-INTERFACE.md
│   ├── PHASE-3-PROTOCOL.md
│   └── PHASE-4-INTEGRATION.md
├── api/typescript/                 (SDK skeleton)
├── cli/                            (POSIX shell client)
├── simulator/                      (browser-side calculators)
├── press/                          (en/ko launch articles, prompt assets)
└── ebook/                          (en + ko long-form, 8 chapters each)
```

## 5. Crosswalk to existing references

The standard is *normative for WIA conformance only*. Where data structures or governance vocabulary overlap with existing references, the field names and semantics deliberately mirror the upstream definitions so that a Tier 2/3 audit can re‑use the same evidence:

- **EU Novel Food Regulation (EU) 2015/2283** — `regulatoryStatus.eu` slot in batch records.
- **Regulation (EU) 2017/893** (processed animal protein in aquafeed) — feed‑grade flag and traceability anchor.
- **Codex Alimentarius CAC/RCP 1‑1969 (HACCP)** — hazard plan stored under `safety.haccp`.
- **ISO 22000:2018** (food safety management) — referenced by Tier 2 evidence.
- **ISO 14040:2006 / 14044:2006** (LCA) — sustainability metrics encoded under `sustainability.lca`.
- **FAO 2013** *Edible insects: future prospects for food and feed security* — definitions of FCR (feed‑conversion ratio), ECI (efficiency of conversion of ingested food).
- **FDA 21 CFR §170.30** — GRAS notice references in `regulatoryStatus.us`.
- **Korea 식품의약품안전처 식품공전 (MFDS Korean Food Code)** — authorised‑species mapping for the Korean market.
- **IPIFF Good Hygiene Practices for Insects as Food and Feed** — process‑control alignment.

Cross‑references are pointers, not normative substitutes. Implementations remain responsible for compliance with each cited body.

## 6. Related WIA standards

- `food-safety` — generic farm‑to‑fork hazard model that this standard specialises.
- `pet-nutrition` — sibling standard for feed‑grade insect ingredients in pet food.
- `traceability` — chain‑of‑custody primitives reused by §3 of PHASE‑3 here.
- `sustainability-metrics` — shared LCA encoding (see PHASE‑4 integration map).
- `circular-economy` — substrate‑sourcing rules (food‑waste bioconversion) for BSF and mealworm.

## 7. Versioning and governance

- Standard ID: `WIA-AGRI-025`.
- Current version: `1.0` (data structures stable since 2025‑12‑26).
- Versioning: SemVer applied to JSON schemas and OpenAPI documents under `spec/`.
- Open governance: change proposals open via the WIA standards repository under MIT licence; reference SDKs ship under MIT.

## 8. Quick start

```bash
# TypeScript SDK
npm install @wia/insect-protein-sdk

# POSIX CLI
WIA_API_BASE="https://api.wia-standards.org/v1/insect-protein" \
  bash cli/insect-protein.sh list-batches --species "Tenebrio molitor"
```

See `cli/README.md` for the shell client and `api/typescript/` for the typed SDK.

---

© 2025 WIA · 弘益人間 (홍익인간)
