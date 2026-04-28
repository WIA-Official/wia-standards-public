# WIA-ozone-layer-protection PHASE 1 — Data Format Specification

**Standard:** WIA-ozone-layer-protection
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for ozone-layer
protection operations: ozone-depleting-substance (ODS) and
HFC inventory records, production / import / export records,
phase-out schedule declarations, atmospheric-observation
records, ground-based and satellite ozone-column observations,
total ozone column records, equivalent-stratospheric-chlorine
(EESC) tracking, and the cross-references binding party
reporting to UNEP Ozone Secretariat data flows. The shape
interoperates with the Montreal Protocol Article 7 reporting
data tables, the WMO Global Atmosphere Watch (GAW), the
NASA Total Ozone Mapping Spectrometer (TOMS) / Ozone
Monitoring Instrument (OMI) / Tropospheric Monitoring
Instrument (TROPOMI) data formats, and ECMWF Copernicus
Atmosphere Monitoring Service (CAMS).

References (CITATION-POLICY ALLOW only):
- Vienna Convention for the Protection of the Ozone Layer (1985)
- Montreal Protocol on Substances that Deplete the Ozone Layer (1987) + Amendments
  (London 1990, Copenhagen 1992, Montreal 1997, Beijing 1999, Kigali 2016)
- UNEP Ozone Secretariat Article 7 Reporting Forms
- UNEP Multilateral Fund — TPMP / HPMP / KIP project reporting
- WMO Global Atmosphere Watch (GAW) — measurement programme
- WMO Scientific Assessment of Ozone Depletion (quadrennial reports)
- ISO/IEC 17025:2017 — laboratory accreditation
- ISO 14001:2015 — environmental management systems
- ISO 23161:2018 — petroleum and gas: ozone-depleting refrigerant trace
- ASHRAE Standard 34 — refrigerant designation and safety classification
- IPCC AR6 WGI — Annex VII (radiative forcing for ODS / HFC)
- NetCDF Climate and Forecast (CF) Metadata Conventions v1.10
- OGC Sensor Observation Service (SOS) 2.0
- IETF RFC 8259 (JSON), RFC 7515 (JWS), RFC 3339

---

## §1 Scope

This PHASE applies to systems that record ozone-layer-protection
state and activities: party-government reporting systems
(Article 7 reporting under Montreal Protocol), national ozone
units (NOUs), refrigerant-management systems, fire-suppression
inventory systems, atmospheric-observation networks (Dobson,
Brewer, ozonesondes), and satellite ozone-data processing
pipelines.

The standard is regime-aware: deployments declare their
operating regime — Article 5 party (developing country with
extended phase-out timelines), non-Article 5 party (developed
country with earlier timelines), or non-party (with restricted
trade access). Each regime carries its own phase-out schedule
and reporting cadence.

In scope: ODS / HFC substance identity and global-warming-potential
(GWP) catalogue, production / import / export reporting,
consumption (defined as production + imports - exports - feedstock /
process-agent uses), critical-use exemption tracking, illegal-trade
case records, atmospheric observations, ozone-column metrics,
EESC trajectory. Out of scope: tropospheric ozone air-quality
(cross-domain to WIA-air-quality), broader greenhouse-gas accounting
beyond Kigali HFCs (cross-domain to WIA-greenhouse-gas-accounting).

## §2 ODS / HFC substance identity

Substances are identified by:

- `substanceRef` — URN of form `urn:wia:ozone:substance:<scheme>:<id>`
  where `scheme` is one of {`montreal-annex`, `cas`, `ashrae34`,
  `gwp-ar6`}
- `montrealAnnex` — closed enum: `Annex A Group I` (CFC-11,
  CFC-12, CFC-113, CFC-114, CFC-115), `Annex A Group II`
  (halons), `Annex B Group I/II/III`, `Annex C Group I` (HCFC),
  `Annex C Group II/III` (HBFC, BCM), `Annex E` (methyl bromide),
  `Annex F Group I/II` (HFC under Kigali Amendment)
- `cas` — CAS Registry Number
- `ashrae34` — ASHRAE 34 designation (e.g., R-12, R-134a, R-32,
  R-1234yf)
- `odp` — Ozone Depletion Potential (per Montreal Protocol
  reporting tables)
- `gwp100` — 100-year Global Warming Potential per IPCC AR6
- `formula` — molecular formula
- `iupacName`
- `state` — `gas`, `liquid`, `liquefied-gas` (storage state at
  STP)

The boundary verifies substance references against the
deployment's substance roster; unrecognised substances are
rejected. Roster updates align with UNEP Ozone Secretariat's
published amendments.

## §3 Production / import / export record

Per-party Article 7 data:

- `transactionId` — URN
- `partyRef` — reporting-party URN (ISO 3166-1 alpha-3 + party
  status)
- `reportingYear` — Gregorian year
- `substanceRef`
- `transactionKind` — closed enum: `production`,
  `import`, `export`, `feedstock-use`, `process-agent-use`,
  `destruction`, `essential-use`, `critical-use`,
  `quarantine-and-pre-shipment`, `laboratory-and-analytical`
- `quantityKg` — metric tonnes (Article 7 tables use metric
  tonnes; this standard normalises to kg for arithmetic)
- `partyOfOrigin` — for imports: ISO 3166-1 of origin;
  for exports: ISO 3166-1 of destination
- `partyStatus` — `party`, `non-party` (trade-restriction
  context)
- `verificationRef` — URN of verification evidence (customs
  declaration, factory production log, etc.)

Records are signed by the National Ozone Unit (NOU) and
audit-chained. Annual aggregates feed the Article 7 submission.

## §4 Consumption calculation record

Consumption = production + imports - exports - feedstock - process-agent
(per Montreal Protocol Article 1(6)):

- `consumptionId` — URN
- `partyRef`
- `reportingYear`
- `substanceRef`
- `productionTonnes` — ODP-weighted metric tonnes
- `importsTonnes`
- `exportsTonnes`
- `feedstockTonnes` — feedstock uses excluded from
  consumption per Article 1(6)
- `processAgentTonnes` — process-agent uses excluded
- `essentialUseTonnes` — exempted under Article 2 essential-use
  decisions
- `criticalUseTonnes` — methyl-bromide critical-use
- `qpsTonnes` — quarantine and pre-shipment
- `consumptionTonnes` — calculated value
- `odpWeightedTonnes` — per-substance ODP-weighted
- `co2eqWeightedTonnes` — per-substance GWP-weighted
  (Kigali Amendment HFC reporting)

Consumption is the headline metric tracked against the
party's phase-out / phase-down baseline.

## §5 Phase-out schedule declaration

Per substance × party-status, the controlling schedule:

- `scheduleId` — URN
- `substanceClass` — Annex A / B / C / E / F group
- `partyStatus` — `article-5`, `non-article-5`
- `baselineYears` — declared baseline years (averages used
  for the substance's group)
- `baselineConsumption` — ODP-weighted tonnes
- `milestones[]` — per-year reduction targets:
  - Annex A I (CFC) non-Art.5: -50% 1995, -75% 1996, -100% 1996
  - Annex A I Art.5: -100% 2010 (with later limited
    exemptions)
  - Annex C I (HCFC) non-Art.5: freeze 1996, -65% 2004,
    -75% 2010, -90% 2015, -99.5% 2020, -100% 2030
  - Annex C I Art.5: freeze 2013, -10% 2015, -35% 2020,
    -67.5% 2025, -97.5% 2030 (with limited servicing tail)
  - Annex F (HFC, Kigali) non-Art.5: -10% 2019, -40% 2024,
    -70% 2029, -80% 2034, -85% 2036
  - Annex F Art.5 Group 1: freeze 2024, -10% 2029, -30%
    2035, -50% 2040, -80% 2045
  - Annex F Art.5 Group 2 (warmer climates): freeze 2028,
    -10% 2032, -20% 2037, -30% 2042, -85% 2047
- `currentYearTarget` — the binding consumption ceiling for
  the current reporting year

The boundary cross-checks each consumption record against
the schedule; non-compliance flags trigger investigation
records.

## §6 Atmospheric observation record

Ground-based and satellite ozone observations:

- `observationId` — URN
- `stationRef` — observation site URN (WMO GAW station ID
  or satellite-instrument ID)
- `observationKind` — `total-column-ozone` (Dobson units),
  `ozone-profile` (mPa per altitude bin from ozonesonde),
  `surface-ozone` (parts per billion volume),
  `solar-uv-index`, `tropopause-temperature` (correlate)
- `instrument` — `dobson-spectrophotometer`, `brewer`,
  `ozonesonde-ECC`, `OMI`, `TROPOMI`, `OMPS`, `MLS`
- `observationTimestamp` — RFC 3339 with offset
- `latitude` / `longitude` / `altitude`
- `value` + `unit` — units per WMO standards (Dobson units
  for total column, mPa for vertical profile)
- `uncertainty` — declared per BIPM JCGM 100 GUM principles
- `qualityFlag` — closed enum: `nominal`, `provisional`,
  `processed`, `validated`, `corrected`, `withdrawn`
- `dataAccessRef` — URN to the data file (NetCDF CF-1.10
  conventions for satellite, TXT or NDJSON for sondes)

Ground-station observations follow the WMO GAW programme;
data submitted to the World Ozone and UV Radiation Data
Centre (WOUDC).

## §7 EESC trajectory record

Equivalent stratospheric chlorine (EESC) — composite metric
of stratospheric chlorine + bromine weighted by depletion
efficacy:

- `eescId` — URN
- `latitudeBand` — `mid-latitude-NH`, `mid-latitude-SH`,
  `polar-NH`, `polar-SH`, `tropical`
- `referenceYear`
- `eescPpt` — parts per trillion (chlorine equivalent)
- `componentsPpt` — per-substance contribution
- `methodology` — modelled (chemistry-transport model) vs.
  reconstructed (from atmospheric observations)
- `referenceAssessment` — citation to the WMO Scientific
  Assessment of Ozone Depletion that produced the trajectory

EESC trajectory is the composite signal used to assess
ozone-layer recovery progress. Pre-1980 levels are the
recovery target.

## §8 Critical-use exemption record

Methyl-bromide critical-use and HCFC servicing-tail
exemptions:

- `exemptionId` — URN
- `partyRef`
- `substanceRef`
- `exemptionKind` — `critical-use-MB` (Article 2H),
  `essential-use` (Article 2A-2I), `feedstock` (Article 1(6)
  exclusion), `process-agent`, `laboratory-and-analytical`,
  `qps` (quarantine and pre-shipment)
- `exemptionYearsAuthorised` — authorised period
- `quantityAuthorisedTonnes`
- `actualUseTonnes` — annual use
- `MOPDecisionRef` — Meeting-of-the-Parties decision URN
  authorising the exemption (e.g., Decision IX/6 critical-use
  criteria)
- `nominationRef` — URN of the technical nomination submitted
  to MBTOC / TEAP

## §9 Illegal-trade case record

Cross-border illegal-trade incidents:

- `caseId` — URN
- `partyRef` — reporting party
- `discoveredAt`
- `substanceRef`
- `seizureKg`
- `seizureLocation` — port / border / inland
- `caseStatus` — `under-investigation`, `prosecuted-pending`,
  `prosecuted-convicted`, `closed-no-action`, `referred-to-INTERPOL`
- `interpolNotice` — INTERPOL reference if applicable
- `mitigationActions[]` — destruction, return-to-origin,
  fine, criminal-charge

Cases are reported to UNEP via the iPIC (informal Prior
Informed Consent) network and INTERPOL Project ENVISEC /
Operation Smokestack-style coordinated enforcement actions.

## §10 Refrigerant-life-cycle record (technician operations)

For deployments tracking refrigerant handling:

- `serviceEventId` — URN
- `equipmentRef` — equipment URN
- `technicianRef` — certified technician URN (per regional
  certification: US EPA §608, EU F-gas Regulation 517/2014,
  KR 환경부 냉매 관리 자격)
- `eventKind` — `installation`, `recharge`, `repair`,
  `decommissioning`, `recovery`, `recycling`, `destruction`
- `substanceRef`
- `quantityKg`
- `containmentVerificationRef` — URN of leak-test evidence
- `disposalCertificateRef` — for destruction / recycling

Refrigerant-life-cycle records feed national F-gas reporting
and serviceable-equipment-bank inventories.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Cross-domain references (informative)

| Reference                  | Use site                                                    |
|----------------------------|-------------------------------------------------------------|
| WIA-greenhouse-gas-accounting | Kigali HFC accounting alignment                          |
| WIA-supply-chain           | refrigerant import / export traceability                    |
| WIA-network-security       | iPIC / INTERPOL data exchange security                       |
| WIA-pq-crypto              | post-quantum migration phase                                 |

## Annex B — Conformance disclosure

Sections §2, §3, §4, §5 are mandatory for party-government
reporting deployments. §6, §7 are mandatory for atmospheric-
observation deployments. §8 is mandatory for parties claiming
exemptions. §9 is mandatory for parties with enforcement
operations. §10 is mandatory for refrigerant-management
deployments.

## Annex C — Conformance levels

| Level     | Scope                                                        |
|-----------|--------------------------------------------------------------|
| Surface   | data formats accepted; self-attested                         |
| Verified  | annual third-party audit (ISO 14001 + UNEP MOP scrutiny)     |
| Anchored  | continuous evidence package + UNEP Implementation Committee  |

## Annex D — Worked Article 7 submission (informative)

```json
{
  "transactionId": "urn:wia:ozone:tx:KOR:2026-A-CFC11-import",
  "partyRef": "urn:wia:ozone:party:KOR",
  "reportingYear": 2025,
  "substanceRef": "urn:wia:ozone:substance:montreal-annex:Annex-A-CFC-11",
  "transactionKind": "import",
  "quantityKg": 0,
  "partyOfOrigin": null,
  "partyStatus": "non-article-5",
  "verificationRef": "urn:wia:ozone:verify:KOR:2025-A-CFC-customs-summary"
}
```

For phased-out substances (CFC-11 in 2025), the report
typically declares zero with a verification reference to the
customs aggregate showing zero imports.

## Annex E — Versioning and deprecation

Versioning follows SemVer 2.0.0. Montreal Protocol amendment
(e.g., new substance scheduled, baseline adjusted) triggers
minor bumps. The Kigali Amendment introduction (2019) was
the most recent major schedule extension.

## Annex F — EESC recovery target

Pre-1980 EESC level (~1.7 ppb mid-latitude) is the recovery
benchmark. Current trajectory (2026) is in steady decline;
WMO Scientific Assessment of Ozone Depletion 2022 projected
mid-latitude recovery around 2040 and Antarctic recovery
around 2066. Deployments tracking EESC reference the most
recent WMO assessment.

## Annex G — TOMS / OMI / TROPOMI data-format crosswalk

Total-column ozone retrieval products (Level-2):

- TOMS (1978-2005): HDF format, 1° × 1.25° resolution
- OMI (2004-): HDF-EOS5 / NetCDF-4, 13×24 km nadir
- TROPOMI (2017-): NetCDF-4, 5.5×7 km nadir (post-2018)
- OMPS (Suomi-NPP, NOAA-20, NOAA-21): NetCDF-4

The boundary's ingestion adapters convert each to the
canonical PHASE 1 §6 observation envelope.

## Annex H — Cross-coalition data sharing

Some atmospheric observations are shared cross-coalition
(WMO GAW, Network for the Detection of Atmospheric
Composition Change — NDACC) under FAIR data principles.
The sharing record carries the FAIR conformance metadata
(findable, accessible, interoperable, reusable) per the
GO FAIR initiative.
