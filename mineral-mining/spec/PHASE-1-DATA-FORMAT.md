# WIA-mineral-mining PHASE 1 — Data Format Specification

**Standard:** WIA-mineral-mining
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for mineral
mining operations: mining-asset registry records, exploration
and resource-estimate records, ore-grade and assay records,
production-shift records, environmental-compliance records,
worker-health records, supply-chain provenance for critical
minerals, and the cross-references binding extraction to
downstream smelting and refining. The shape interoperates
with internationally recognised resource-reporting codes
(JORC / NI 43-101 / SAMREC / PERC under the CRIRSCO
template) and supply-chain due-diligence frameworks (OECD
DDG, EU Conflict-Minerals Regulation, ICMM commitments).

References (CITATION-POLICY ALLOW only):
- CRIRSCO International Reporting Template (JORC, NI 43-101, SAMREC, PERC)
- OECD Due Diligence Guidance for Responsible Supply Chains of
  Minerals from Conflict-Affected and High-Risk Areas
- ISO 14001:2015 — Environmental management systems
- ISO 45001:2018 — Occupational health and safety
- ISO 17025:2017 — Testing and calibration laboratory competence
- ISO/IEC 27001:2022 — Information security
- ICMM Mining Principles
- IETF RFC 3339 (timestamps), RFC 7515 (JWS), RFC 8259 (JSON)
- WGS-84 — geodetic reference frame
- UN Framework Classification for Resources (UNFC) 2019

---

## §1 Scope

This PHASE applies to the data shape used by extractive
operations from exploration through stope/face production
to gate-of-mine handover, plus environmental and worker
records associated with those operations. Out of scope:
downstream concentrator chemistry (handled by metallurgy
standards), smelter mass-balance accounting (cross-domain
to refining standards), and capital-market reporting
formats (handled by the issuer's listing-authority
templates with this standard providing the underlying
data).

The standard is competent-person-aware: any record
classified as a Mineral Resource or Reserve under a CRIRSCO-
family code carries a competent-person attribution and the
qualifying code; misuse of those classifications outside
their codes is a conformance violation.

In scope: asset registry, exploration data, resource and
reserve estimate metadata, production records, environmental
sample records, worker-exposure records, equipment-state
records, supply-chain provenance handover. Out of scope:
royalty contracts, geological-model file formats (carried
opaquely as binary references), mineral-rights registries
(jurisdictional).

## §2 Mining-asset registry record

Every operation tracks the assets it operates:

| Field             | Source / Binding                                       |
|-------------------|--------------------------------------------------------|
| `assetRef`        | URN of form `urn:wia:mm:asset:<operator>:<id>`         |
| `assetType`       | `pit`, `underground-mine`, `placer`, `dredging`, `in-situ-leach`, `tailings-storage`, `processing-plant` |
| `operatorRef`     | URN of operating entity                                |
| `licenceRef`      | URN of governing mineral-rights licence               |
| `commodity[]`     | declared commodities (Cu, Au, Li, REE, Ni, Co, Zn, …) |
| `methodClass`     | `open-pit`, `underground-cut-and-fill`, `block-cave`, `room-and-pillar`, `solution-mining`, `placer` |
| `lifeOfMineEnd`   | declared end-of-mine date (planning estimate)          |
| `closureBondRef`  | URN of the closure-bond instrument                    |
| `coordinatingAuthority` | URN of the regulatory authority                  |

Asset registry changes are signed and audit-logged. Closure-
bond status changes are first-class events that propagate to
downstream provenance consumers.

## §3 Exploration data record

Exploration drives the resource picture:

- `explorationId` — URN
- `assetRef` — URN of the licence area or operating asset
- `programName` — programme identifier
- `methods[]` — closed enum: `mapping`, `geochem-soil`,
  `geochem-stream`, `geophysics-em`, `geophysics-magnetics`,
  `geophysics-gravity`, `drilling-rc`, `drilling-diamond`,
  `drilling-aircore`, `trenching`, `bulk-sample`
- `dateRange` — from/to RFC 3339
- `dataPackagesRef[]` — URIs of the underlying data packages
  (assays, lithology logs, surveys)
- `competentPersonRef` — URN of the competent person under
  the applicable code
- `reportRef` — URN of the public exploration report (where
  required by listing rules)

Exploration records do not by themselves classify resources;
they are inputs to a resource-estimate record (PHASE 1 §4).

## §4 Resource and reserve estimate record

Estimates classified under a CRIRSCO-family code:

- `estimateId` — URN
- `assetRef` — URN
- `code` — closed enum: `JORC`, `NI 43-101`, `SAMREC`, `PERC`
- `classification` — closed enum within the chosen code
  (e.g., for JORC: `inferred`, `indicated`, `measured`;
  for reserves: `probable`, `proved`)
- `commodity` — primary commodity
- `tonnes` — declared tonnes (at declared confidence)
- `gradeMean` — mean grade with units (% or g/t)
- `cutOffGrade` — economic cut-off applied
- `effectiveDate` — RFC 3339
- `competentPersonRef` — URN, with the person's
  qualification per the chosen code
- `reportRef` — URN of the public technical report
- `modifyingFactors` — declared list of factors applied for
  reserve classification

Resource and reserve records are versioned by code and
effective-date; cross-version replay reconstructs the
estimate evolution. Records claiming a code-classification
without a corresponding competent-person attribution are
refused at boundary intake.

## §5 Production record

For every shift / panel / face / pit-bench:

- `productionId` — URN
- `assetRef` — URN
- `shiftStart`, `shiftEnd` — RFC 3339
- `locationRef` — URN of the workplace within the asset
  (panel, level, bench)
- `tonnesMined` — measured tonnes (with vendor-quantified
  uncertainty)
- `gradeSampleRefs[]` — URNs of grade samples backing the
  declared mined grade (PHASE 1 §6)
- `commodityProduced[]` — for multi-commodity operations
- `equipmentRefs[]` — URNs of equipment used
- `dilutionFactor` — declared dilution applied
- `recoveryFactor` — declared recovery applied
- `signatures[]` — shift-foreman and survey signatures

Production records feed gate-of-mine reconciliation against
the resource estimate; persistent over- or under-call against
the estimate triggers PHASE 4 §6 reconciliation review.

## §6 Assay / grade-sample record

Each laboratory result is recorded:

- `sampleId` — URN
- `assetRef` — URN
- `samplingMethod` — closed enum: `chip`, `channel`, `bulk`,
  `drill-core-half`, `drill-core-quarter`, `pulp-rejects`
- `sampledAt` — RFC 3339 with offset
- `coordinates` — easting, northing, elevation (WGS-84)
- `length` — metres of the sample interval
- `assays[]` — per-element entry with method, value, unit,
  detection-limit, accreditation reference (ISO 17025)
- `qaqcRefs[]` — URNs of accompanying blanks, duplicates,
  certified reference materials (CRMs), pulp duplicates
- `laboratoryRef` — URN of the assaying laboratory

Assay records without QA/QC linkage are tagged `provisional`
and excluded from resource-estimate inputs until the QA/QC
package is reconciled.

## §7 Environmental sample record

Routine and incident environmental sampling:

- `envSampleId` — URN
- `assetRef` — URN
- `mediaType` — `surface-water`, `groundwater`, `soil`, `dust`,
  `tailings-supernatant`, `acid-mine-drainage`, `noise`, `air`
- `sampledAt` — RFC 3339 with offset
- `coordinates` — WGS-84
- `analytes[]` — analyte/value/unit/limit-of-quantification
- `regulatoryThresholdRefs[]` — URN of the regulatory
  threshold against which the sample is evaluated
- `exceedance` — boolean
- `chainOfCustodyRef` — URN of the chain-of-custody record

Exceedance events trigger PHASE 4 §7 environmental incident
workflow within a deployment-declared latency budget. The
deployment publishes its environmental-monitoring plan as a
declared schedule; missing samples on the plan are treated
as a conformance violation regardless of incident outcomes.

## §8 Worker-exposure record

Per worker per shift, the deployment records measured
exposure relevant to the mine's hazards:

- `exposureId` — URN
- `workerRef` — URN of the worker (privacy-preserving
  pseudonym; binding to PII held in a separately access-
  controlled HR system per PHASE 3 §9)
- `assetRef` — URN
- `shiftRef` — URN
- `noiseDoseDb` — equivalent continuous noise dose
- `dustExposureMgM3` — respirable dust exposure
- `silicaExposureMgM3` — respirable crystalline silica
- `radonWlm` — radon working-level-months (uranium / hard-rock)
- `dieselParticulateMgM3` — DPM (underground diesel-equipment
  operations)
- `noteRef` — narrative reference for any exceptional events

Exposures exceeding the operation's occupational limit
trigger PHASE 4 §8 worker-health follow-up workflows.

## §9 Supply-chain provenance handover

When ore or concentrate leaves the gate, a provenance
record handing it to the next custody chain is emitted:

- `handoverId` — URN
- `fromAssetRef` — URN
- `toCustodyRef` — URN of receiving party
- `handoverAt` — RFC 3339 with offset
- `commodity` — declared commodity
- `gradeDeclared` — declared head-grade (with uncertainty)
- `tonnesHandover` — declared tonnes (with uncertainty)
- `oecdDdgStatus` — declared due-diligence status per the
  OECD DDG (e.g., `green`, `red`, `requires-additional-action`)
- `incidentReports[]` — URNs of any open environmental or
  social-incident reports relevant to this batch
- `signatures[]` — shipper and receiver signatures

Handover records are the canonical entry point into
downstream WIA-supply-chain and refining standards; the
boundary refuses handover for operations whose `oecdDdgStatus`
is `red`.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Cross-domain references (informative)

| Reference                     | Use site                                                 |
|-------------------------------|----------------------------------------------------------|
| WIA-supply-chain              | downstream provenance receiver (PHASE 4 §3)              |
| WIA-environmental-monitoring  | shared schema for environmental sample evaluation        |
| WIA-occupational-safety       | worker exposure reference framework                      |

## Annex B — Conformance disclosure

Sections §2, §3, §5, §6, §7, §8, §9 are mandatory for any
operating mining asset; §4 is mandatory for any operation
declaring resource or reserve estimates publicly. A
deployment `partial` or `excluded` on §6 (Assay) or §9
(Handover) is non-conformant overall.

## Annex C — Versioning and deprecation

Versioning follows SemVer 2.0.0. Resource and reserve
records carry the originating code's version (e.g.,
`JORC 2012`); migration to a newer code edition is recorded
as a versioned re-statement, not a silent overwrite.

## Annex D — Worked handover record (informative)

A copper-concentrate handover with green DDG status:

```json
{
  "handoverId": "urn:wia:mm:handover:operator-x:h-2026-04-27-0014",
  "fromAssetRef": "urn:wia:mm:asset:operator-x:north-pit",
  "toCustodyRef": "urn:wia:logistics:carrier-y:lot-l-7791",
  "handoverAt": "2026-04-27T22:30:00+09:00",
  "commodity": "Cu",
  "gradeDeclared": {"value": 28.4, "unit": "%", "uncert": 0.3},
  "tonnesHandover": {"value": 1024.5, "unit": "t", "uncert": 1.2},
  "oecdDdgStatus": "green",
  "incidentReports": [],
  "signatures": [/* shipper, receiver JWS detached signatures */]
}
```

The receiving custody chain consumes this record into
WIA-supply-chain inflow. The originator retains a copy in
the audit chain so reconciliation across the gate is
auditable both directions.

## Annex E — Vendor extensions

Mine-fleet vendors and lab vendors may extend records with
`x-vendor-*` fields. Extensions MUST NOT contradict canonical
fields and MUST NOT be required for core conformance.

## Annex F — Time discipline cross-reference

Time fields use the discipline of PHASE 3 §6 (UTC, leap-
second handling). Records that fail clock discipline are
tagged `provisional` until backfilled.

## Annex G — Conformance level

Implementations declare conformance level (Surface / Verified
/ Anchored). Anchored requires a continuous evidence package
plus an annual audit by an ISO 14001 / ISO 45001 auditor
covering the integration contracts in PHASE 4.
