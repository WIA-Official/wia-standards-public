# WIA-HERITAGE-009 PHASE 3 — PROTOCOL Specification

**Standard:** WIA-HERITAGE-009
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern a hydrogen-
energy programme: jurisdictional licensing, ISO 14687 fuel-
quality discipline, ISO 19880-1 refuelling-station
operation, ISO 22734 electrolyser operation, IEC 60079
explosive-atmosphere governance, IEC 62282 fuel-cell
component conformance, life-cycle carbon-intensity
accounting, guarantee-of-origin issuance, transport
incident response, and end-of-life decommissioning.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO 14001:2015 (environmental management)
- ISO 14064-1:2018 (greenhouse gas accounting at the
  organisation level)
- ISO 14687:2019 (hydrogen fuel quality)
- ISO 19880-1:2020 (HRS general requirements)
- ISO 22734:2019 (water electrolysis hydrogen generators)
- ISO 26142:2010 (hydrogen detection)
- ISO 45001:2018 (occupational health and safety)
- ISO/IEC 17025:2017 (calibration laboratories)
- ISO/IEC 27001:2022 (information security management)
- ISO 8601 (date and time)
- IETF RFC 5905 (NTPv4)
- IETF RFC 9457 (Problem Details)
- IEC 60079 series (explosive atmospheres)
- IEC 62282 series (fuel cell technologies)
- SAE J2601 / J2719 / J2799
- NFPA 2 (Hydrogen Technologies Code; cited where US
  jurisdictions apply)
- KGS Code FP216 / FP217 / FP218 (Korean hydrogen-handling
  rules)
- CertifHy guarantee-of-origin scheme
- IPHE Working Group on Hydrogen Production Analysis
  (methodology reference)

---

## §1 Jurisdictional Licensing

A WIA-HERITAGE-009 operator MAY claim conformance to
WIA-HERITAGE-009 only after the operating jurisdiction's
energy / industrial-safety regulator has issued a valid
operating licence for the value-chain segments the
operator runs (KOSHA + KGS in Korea, ARB / DOE / FAA-
HMS in the US, EU PED + ATEX directives + national fuels
regulator, METI / HPGS in Japan, equivalent authorities
elsewhere). Licences are recorded against the programme;
revocation freezes the affected segment.

## §2 ISO 14687 Quality Discipline

Hydrogen for fuel-cell vehicle use follows ISO 14687:2019
Type I Grade D limits per impurity (water, CO, CO2, sulphur
species, formaldehyde, formic acid, ammonia, particulates,
total hydrocarbons, halogenated compounds, oxygen, nitrogen,
argon, helium). Per-batch quality records (PHASE-1 §4)
cite an ISO/IEC 17025-accredited laboratory; assays whose
results breach Grade D limits trigger the operator's
quality-deviation workflow, and the affected batch cannot
ship for FCEV refuelling until requalified or downgraded.

## §3 ISO 19880-1 Refuelling Station Operation

Refuelling stations operate per ISO 19880-1:2020 and the
SAE J2601 fueling protocol family (T20 / T40 / H70 / A70
profiles per vehicle tank capacity and pressure class).
Station SOP covers:

- per-vehicle communication negotiation per SAE J2799
  (where supported);
- per-fill precooling envelope per SAE J2601;
- per-fill quality-certificate citation per ISO 19880-8
  / SAE J2719 (PHASE-1 §6);
- per-fill abort criteria (pressure overshoot, temperature
  rise, vehicle-side fault, station-side fault).

Aborted fills are recorded with rationale and feed the
operator's reliability-improvement process.

## §4 ISO 22734 Electrolyser Operation

Electrolyser plants operate per ISO 22734:2019 for
water-electrolysis hydrogen generators. Operating discipline:

- per-stack performance baseline at commissioning and at
  the operator's revalidation cadence;
- per-stack degradation tracking (voltage rise at fixed
  current, efficiency degradation curve);
- per-stack stop-and-start cycle counting (relevant for
  PEM / AEM stacks where dynamic cycling drives membrane
  degradation);
- per-batch product-water consumption and per-batch
  oxygen co-product handling (oxygen vent or oxygen
  monetisation for adjacent industrial use).

## §5 IEC 60079 Explosive Atmospheres

Hydrogen-handling areas are zoned per IEC 60079 (Zone 0 /
1 / 2 for gas atmospheres). The operator's per-zone
electrical-equipment rating (Ex d / e / i / nA / nC) follows
the zone classification. Hydrogen detection per ISO 26142
covers fixed-point detectors at vent stacks, dispenser
canopies, compression skids, and storage areas; alarm
thresholds follow the operator's per-zone alarm matrix.

## §6 IEC 62282 Fuel-Cell Component Conformance

Fuel-cell modules and stacks consumed at offtake sites
(stationary CHP, vehicle propulsion) follow IEC 62282
series (Part 2 stationary modules, Part 3 stationary
applications, Part 6 micro fuel cell power systems, Part
7 single-cell test methods, Part 8 energy storage systems
for primary use). Operators record component identifiers
and conformance certificates in the offtake record.

## §7 Life-Cycle Carbon-Intensity Accounting

Per-batch carbon intensity (PHASE-1 §3 `carbonIntensity`)
follows the operator's published LCA boundary, aligned
with ISO 14064-1:2018 organisation-level GHG accounting
and the IPHE Working Group on Hydrogen Production Analysis
methodology. Boundary choices are recorded in the
operator's quality dossier:

- well-to-gate boundary for production-only operators;
- well-to-tank boundary for operators that also carry
  midstream and dispensing;
- per-segment attribution for operators that jointly
  operate production and downstream segments.

Carbon intensity values are not aggregated across batches
without consumer awareness of the boundary choice; the
guarantee-of-origin issuance cites the specific batch and
boundary.

## §8 Guarantee-of-Origin Issuance

Operators that participate in CertifHy (or the operating
jurisdiction's equivalent low-carbon / renewable hydrogen
disclosure scheme — UK Low-Carbon Hydrogen Standard, US
DOE 45V, KEA H2 GoO in Korea, equivalent rules) emit
per-batch guarantees that downstream offtakers cite in
their own LCA reporting. The operator's GoO issuance
discipline records:

- per-batch eligibility criteria check (carbon intensity
  threshold, source-mix composition);
- per-batch issuance and per-batch retirement (when the
  offtaker consumes the guarantee);
- per-period reconciliation with the GoO registry to
  prevent double-counting.

## §9 Transport Incident Response

Hydrogen transport incidents (tube-trailer rupture,
liquid-tanker spill, pipeline rupture, ammonia or LOHC
carrier event) follow the operating jurisdiction's
hazardous-materials response framework (US DOT 49 CFR,
EU ADR, Korean KGS Code, equivalent rules). The operator's
incident-response SOP covers immediate notification,
exclusion-zone establishment, source-control procedure,
and post-event investigation. Records flow into the safety
incident record (PHASE-1 §8).

## §10 End-of-Life Decommissioning

End-of-life decommissioning follows the operating
jurisdiction's industrial-decommissioning regime: site
characterisation, equipment de-energisation, hydrogen and
adjacent-gas inventory drainage, equipment dismantling,
contaminated-material disposal, and site release. For
salt-cavern storage, the operator's cavern-abandonment
plan follows the jurisdiction's well-abandonment rules.

## §11 Records Retention

Programme records — every production / quality / inventory /
refuelling / transport / safety incident record, the API
audit logs, the regulator submissions, the GoO issuance
register, and the LCA boundary documentation — retain per
the operating jurisdiction's commercial- and industrial-
safety records-retention rules. Safety-incident records
that involved injury, fatality, or major release retain
indefinitely.

## §12 Time Synchronisation

Operator clocks synchronise per RFC 5905 (NTPv4) so that
production / inventory / refuelling / transport timestamps
are consistent across the operator's facility fleet and
the partner-operator integrations.

## §13 Cross-Jurisdictional Operation

Multi-jurisdiction operators (cross-border pipeline
operators, marine ammonia carrier operators, multinational
refuelling-network operators) honour each jurisdiction's
licensing and safety rules. Per-record governing-
jurisdiction tagging supports downstream regulator-specific
reporting.

## §14 Quality Dossier

The operator's quality dossier records the licensing
references, the ISO 14687 quality programme, the ISO
19880-1 / 22734 operating SOPs, the IEC 60079 zone map,
the GoO scheme membership, the LCA boundary documentation,
and the operator's incident history. The dossier is
reviewed at least annually by the operator's quality
manager.

## §15 Pipeline-Embrittlement and Material-Compatibility Discipline

Operators that transport hydrogen through dedicated H2
pipelines or that blend hydrogen into the existing natural-
gas network manage hydrogen-induced embrittlement risk per
the operator's material-compatibility study. The study
covers:

- per-pipeline-grade material classification (X52 / X60 /
  X65 / X70 / X80 carbon steels are most vulnerable to
  hydrogen embrittlement; austenitic stainless and certain
  polymers are more tolerant);
- per-blending-percentage embrittlement-risk envelope
  derived from operator-side coupon-testing or recognised
  industry test programmes;
- per-pipeline integrity-management cadence including
  inline inspection (smart-pig runs), pressure-testing,
  and per-section embrittlement-coupon retrieval and
  destructive testing.

Operators that operate dedicated H2 pipelines record their
embrittlement-management programme in the quality dossier;
operators that blend record both their network operator's
material-compatibility study reference and the per-blending-
point sequence ramp.

## §16 Cryogenic Handling Discipline (Liquid Hydrogen)

Liquid hydrogen handling carries low-temperature exposure
risk (LH2 boils at 20.3 K at 1 atm) and oxygen-displacement
asphyxiation risk. The operator's cryogenic-handling SOP
covers:

- per-vessel cryogenic-rated material certification (per
  ISO 13985 for vehicle tanks; per the operator's pressure-
  vessel standard for storage and transport vessels);
- per-task PPE (cryogenic gloves, face shield, oxygen
  monitor on the operator);
- per-area oxygen monitoring with alarm thresholds aligned
  with the ASHRAE-equivalent indoor-air-quality discipline;
- per-vessel boil-off venting design (vent stacks at safe
  height with no vent-hood reignition risk);
- per-spill rapid-warming procedure for any uncontrolled
  release.

Cryogenic incidents (PHASE-1 §8 classification
`low-temperature-exposure` or `spill-cryogenic`) follow the
incident-response protocol of §9 plus the operator's
cryogenic medical-response procedure for any worker
exposure.

## §17 Tube-Trailer Loading and Unloading Discipline

Tube-trailer operations follow the operator's loading-and-
unloading SOP:

- per-shift driver qualification verification;
- per-load loading-station leak test;
- per-load tube-trailer pressure record at fill and at
  delivery (mass discrepancy investigation per PHASE-2 §8);
- per-load tube-trailer maintenance status check (annual
  pressure-vessel inspection, periodic ultrasonic testing
  per the operator's vessel-management programme).

## §18 Conformance and Auditing

A programme conformant with WIA-HERITAGE-009 publishes
its operating licences, the catalogue of operating
facilities, the per-batch quality summary, the safety-
incident summary at major and above, and the GoO issuance
register, and answers an annual self-assessment that maps
each clause of this PHASE to the operator's implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-HERITAGE-009
- **Last Updated:** 2026-04-28
