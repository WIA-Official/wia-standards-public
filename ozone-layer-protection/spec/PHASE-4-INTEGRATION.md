# WIA-ozone-layer-protection PHASE 4 — Integration Specification

**Standard:** WIA-ozone-layer-protection
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE specifies how an ozone-layer-protection deployment
integrates the data, APIs, and protocols from PHASEs 1–3 with
broader operational systems: customs / single-window trade
systems, refrigerant licence registries, GAW/WOUDC submission
flows, satellite mission ground segments, multilateral fund
project reporting, and the cross-domain consumers that depend
on ozone-protection state.

References (CITATION-POLICY ALLOW only):
- UNEP Ozone Secretariat reporting infrastructure
- UNEP Multilateral Fund — TPMP / HPMP / KIP project frameworks
- WMO Global Atmosphere Watch programme
- WMO World Ozone and UV Radiation Data Centre (WOUDC)
- Network for the Detection of Atmospheric Composition Change (NDACC)
- WCO SAFE Framework — customs single-window
- ASHRAE Standard 34 / ISO 817 — refrigerant designation
- WIA-greenhouse-gas-accounting, WIA-supply-chain,
  WIA-network-security, WIA-pq-crypto

---

## §1 Customs / single-window integration

For NOU deployments connected to national customs systems:

- **Single-window message bus**: the NOU subscribes to
  customs-clearance events for HS codes covering controlled
  substances (HS Chapter 29 organics, including 2903 series)
- **Pre-clearance check**: customs queries the NOU for
  importer licence / quota state via PHASE 2 §10
  capability + bespoke trade-system endpoint
- **Post-clearance audit**: cleared transactions auto-create
  PHASE 1 §3 transaction records pre-populated from customs
  declaration; the NOU verifies and signs

For Article-5 parties, customs integration drives the
majority of import / export records; for non-Article-5
parties with shrinking trade volumes, manual verification
may suffice.

## §2 Refrigerant-licence registry integration

National refrigerant-licence registries integrate with
this PHASE:

- Importer licence / quota system: each licence update
  flows to PHASE 1 §3 quota tracking
- Technician certification system: each technician's
  certification status drives PHASE 1 §10 service-event
  authorisation
- Equipment register (large-scale stationary equipment):
  per-equipment refrigerant-charge inventory + leak-test
  records

For the EU, this is the F-gas Regulation 517/2014 portal +
ODS Regulation 1005/2009 portal. For the US, this is EPA
§608 + Significant New Alternatives Policy (SNAP). For
Korea, 환경부 ODS / HFC 관리 시스템.

## §3 GAW / WOUDC submission flow

For atmospheric-observation deployments:

- **GAW station registration**: each station registers in
  the WMO GAW Information System (GAWSIS) with metadata
  pinned to the deployment's substance roster
- **Data submission to WOUDC**: validated observations
  forwarded to WOUDC via the agreed exchange channel
  (FTP-based file submission for legacy compatibility,
  HTTPS-based for modern stations)
- **NDACC participation**: stations participating in NDACC
  follow the additional NDACC quality-control protocols

WOUDC submission packages include the per-station-day
observation file plus calibration metadata.

## §4 Satellite mission ground segment integration

For deployments processing satellite ozone retrievals:

- **Mission ground segment**: TROPOMI (Sentinel-5P), OMI
  (Aura), OMPS (Suomi-NPP, NOAA-20, NOAA-21) ground
  segments produce Level-1 / Level-2 products
- **Reprocessing campaigns**: mission ground segments
  release reprocessed Level-2 products on multi-year
  cycles; the deployment's observation records are
  recomputed against the latest reprocessing
- **NRT vs. offline products**: near-real-time products
  carry `provisional` quality flag; offline (validated)
  products carry `validated`

## §5 Multilateral Fund project reporting

For Article-5 parties with Multilateral Fund (MLF) projects:

- **TPMP** (Terminal Phase-out Management Plan) reporting
- **HPMP** (HCFC Phase-out Management Plan) reporting
- **KIP** (Kigali Implementation Plan) reporting
- Per-project consumption-reduction tracking against
  approved targets

MLF project reports cross-reference PHASE 1 §3 transaction
records and PHASE 1 §4 consumption calculations to evidence
project-claimed reductions.

## §6 Cross-domain integration

| Domain                        | Integration                                                      |
|-------------------------------|------------------------------------------------------------------|
| WIA-greenhouse-gas-accounting | Kigali HFC accounting alignment (CO₂-eq weighted)               |
| WIA-supply-chain              | Refrigerant supply-chain traceability (cylinder serialisation)   |
| WIA-network-security          | iPIC + Secretariat channel cipher-suite floor                    |
| WIA-pq-crypto                 | Post-quantum migration phase declarations                         |
| WIA-air-quality (planned)     | Tropospheric ozone ↔ stratospheric ozone semantic disambiguation |

## §7 Operational SLAs

| Concern                                         | Default SLA                |
|-------------------------------------------------|----------------------------|
| Substance-roster lookup                          | ≤ 100 ms p95              |
| Transaction publish (single)                     | ≤ 200 ms p95              |
| Bulk transaction submission (annual filing)      | ≤ 30 s for ≤ 100k records |
| Atmospheric observation ingest (single)          | ≤ 200 ms p95              |
| NetCDF file ingest (Level-2 satellite)           | ≤ 5 min per orbit         |
| iPIC consultation forwarding                     | ≤ 1 min                    |
| Article 7 submission acknowledgement             | per Secretariat SLA        |
| Audit-chain entry availability                   | ≤ 10 s                    |

## §8 Quarterly compliance report

The boundary emits a quarterly compliance report:

- Per-substance × party × year transaction volumes
- Consumption vs. phase-out target by substance class
- Active exemption count + actual-use vs. authorised
- Atmospheric-observation freshness (per station)
- iPIC consultation count + response timeliness
- Illegal-trade case state distribution
- Service-event volume by technician region
- Article 7 submission timeliness
- Audit-chain integrity check results

## §9 Acceptance criteria

A deployment claims conformance when:

1. Substance roster current with most recent Montreal
   Amendment
2. All transactions for the prior reporting year reported
3. Consumption recomputed and within phase-out target (or
   covered by authorised exemption)
4. Atmospheric observations (if station operator) submitted
   to WOUDC current
5. Active iPIC consultations responded within agreed
   timeline
6. Article 7 submission for the prior year delivered
7. Quarterly compliance report has no integrity-check
   failures

## §10 Common pitfalls (informative)

- **HS code drift** — controlled substances move between
  HS sub-headings on WCO revisions; deployments SHOULD
  refresh HS-code mapping at every WCO HS revision
- **Mixture / blend reporting** — refrigerant blends
  (e.g., R-410A = R-32 + R-125) must be decomposed into
  component substances for Article 7 reporting
- **GWP value drift** — IPCC AR6 GWP values differ from
  AR5; Kigali Amendment originally referenced AR4. The
  deployment SHOULD declare which IPCC report's GWP values
  are used and align with the controlling regulation's
  reference
- **Calibration lapse** — Dobson / Brewer instruments
  drift; routine calibration against WCC reference is
  essential to avoid systematic bias propagation
- **Non-Annex C substance trade with non-parties** — not
  every party of the Vienna Convention has ratified all
  amendments; trade restriction logic must check per-party
  amendment-ratification status
- **Servicing-tail double-counting** — HCFC servicing
  reserves under post-2030 schedules must be tracked
  separately to avoid double-counting against new-equipment
  consumption
- **Illegal-trade signal handling** — atmospheric anomalies
  (CFC-11 emissions resurgence 2018-2019) can hint at
  illegal production; deployments SHOULD correlate
  observation anomalies with trade data

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Cross-domain reference table

| Reference                       | Use site                                                   |
|---------------------------------|------------------------------------------------------------|
| WIA-greenhouse-gas-accounting   | Kigali HFC accounting and CO₂-eq aggregation               |
| WIA-supply-chain                | refrigerant cylinder supply-chain traceability             |
| WIA-network-security            | iPIC + Secretariat channel cipher-suite floor              |
| WIA-pq-crypto                   | post-quantum migration phase                               |
| WIA-identity-management         | NOU officer + technician credential                         |

## Annex B — Decommissioning checklist (informative)

When a national NOU restructures or transfers
responsibility:

- [ ] Substance-roster handoff to successor
- [ ] Open transaction reporting cycle handed off
- [ ] Active exemptions transferred
- [ ] iPIC contact-point updated with UNEP
- [ ] Customs integration re-bound
- [ ] Audit chain sealed for the prior NOU; new NOU
      starts fresh chain referencing the prior root
- [ ] Multilateral Fund project ownership transferred

## Annex C — Conformance disclosure

Sections §1, §2, §7, §8, §9 are mandatory for NOU
deployments. §3 is mandatory for atmospheric-observation
deployments. §4 is mandatory for satellite-data deployments.
§5 is mandatory for Article-5 parties with active MLF
projects.

## Annex D — Worked Article 7 calendar (informative)

```
Reporting year (CY 2025) compilation timeline:

2026-01-01  Reporting-year close; provisional totals available
2026-03-31  Provisional Article 7 submission for early discussion
2026-06-30  Final transaction reconciliation
2026-09-30  Article 7 deadline (per Article 7 paragraph 3)
2026-Q4     Secretariat aggregation publication
2027-Q1     Implementation Committee review of any flagged cases
```

## Annex E — Multi-language reporting

UNEP Article 7 forms are published in the UN's six official
languages (Arabic, Chinese, English, French, Russian, Spanish).
Deployments serving non-English-speaking NOU staff should
support per-language localisation; the canonical submission
to UNEP remains in the agreed working language.

## Annex F — Refrigerant-bank model

For deployments tracking refrigerant banks (the in-service
inventory of refrigerant gas in equipment):

- `bankInventory` — per-substance, per-equipment-type,
  per-region cumulative installed quantity
- `leakRate` — per-equipment-type modelled annual leak rate
- `serviceFlow` — annual recharge to compensate leaks
- `endOfLifeRecovery` — annual recovery from decommissioned
  equipment
- `destructionFlow` — recovered refrigerant destroyed

The bank model feeds long-term forecasts of phase-out
trajectory and informs HFC phase-down planning.

## Annex G — Atmospheric-anomaly correlation

For deployments that ingest both transaction and
observation data:

- Atmospheric anomalies (e.g., CFC-11 emission resurgence
  detected 2018-2019 from AGAGE network observations)
  can signal undeclared production / use
- Deployments SHOULD subscribe to atmospheric-anomaly
  feeds (NOAA, AGAGE, NDACC) and cross-reference against
  transaction data
- Anomaly investigation outcomes are themselves audit-chained

## Annex H — Public attestation surface

Anchored deployments publish:

- Capability document
- Substance-roster signature
- Annual Article 7 submission summary (party-public version)
- iPIC consultation aggregate
- Quarterly compliance report (sanitised public version)
- Audit-chain root

UNEP Ozone Secretariat consumes the surface for
Implementation Committee preparation.
