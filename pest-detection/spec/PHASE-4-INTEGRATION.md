# WIA-pest-detection PHASE 4 — Integration Specification

**Standard:** WIA-pest-detection
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE specifies how WIA-pest-detection integrates
with adjacent regulatory, supply-chain, agricultural-
operations, and research-data systems: IPPC International
Phytosanitary Portal and Regional Plant Protection
Organisations, EPPO PRA / database, NPPO authorities,
laboratory information management systems (LIMS),
agricultural-supply-chain traceability, MRL / PHI
binding to harvest lots, satellite imagery providers
(ESA Sentinel, USGS Landsat, NASA HLS, Planet, Maxar),
ISOBUS-equipped tractor / sprayer fleets, AgGateway
ADAPT, FAO data conventions, weather-service feeds,
and downstream food-safety pipelines (e.g. RASFF, FDA
FSIS, MFDS Food Safety Korea). It also specifies the
operational binding to companion WIA standards.

References (CITATION-POLICY ALLOW only):
- IPPC International Phytosanitary Portal (IPP) — pest reporting under ISPM 17
- IPPC ePhyto generic national system / hub
- EPPO Reporting Service, EPPO Global Database
- FAO Crop Prospects and Food Situation; FAO IPM
- Codex Alimentarius MRL portal
- WHO recommended classification of pesticides by hazard
- ISO 11783 (ISOBUS); AgGateway ADAPT framework
- GS1 GTIN / GLN / SSCC; GS1 EPCIS (for harvested-lot binding)
- HL7 FHIR R5 (where pest-related public-health data emerge, e.g. vector-borne)
- ISO 19115-2 — geographic-information metadata
- OGC SensorThings; OGC WMTS / WMS / WCS; OGC GeoTIFF / GeoPackage
- ESA Copernicus Open Access Hub; USGS Earth Explorer; NASA HLS / GIBS
- 21 CFR Part 11; EU Reg 2017/625 (official controls); EU Reg 1107/2009 (PPP)
- ISO 8601, ISO 3166, BCP 47
- IETF RFC 9110 (HTTP), RFC 7515 (JWS), RFC 8259 (JSON), RFC 8785 (JCS)

---

## §1 IPPC and RPPO integration

NPPO-filed reports propagate to:

| System / body         | Operator                           |
|-----------------------|------------------------------------|
| IPPC IPP              | IPPC Secretariat (FAO)             |
| EPPO                  | European and Mediterranean PPO     |
| NAPPO                 | North American PPO                 |
| COSAVE                | Comité Regional de Sanidad Vegetal |
| APPPC                 | Asia and Pacific PPC                |
| IAPSC                 | Inter-African Phytosanitary Council |
| OIRSA                 | Organismo Internacional Regional   |
|                       | de Sanidad Agropecuaria             |
| CPPC                  | Caribbean PPC                       |

Reports cite the IPPC submission identifier; status
updates (eradication-declared, containment-progress,
periodic) replicate to the WIA NPPO-report record.

## §2 LIMS integration

ISO 17025-accredited diagnostic laboratories integrate
through:

| LIMS profile         | Standard / vendor                              |
|----------------------|------------------------------------------------|
| HL7 v2.x OUL^R22     | laboratory result message (where the LIMS     |
|                      | uses HL7)                                     |
| AgGateway ADAPT      | agricultural lab-result exchange              |
| Custom REST / SOAP   | per LIMS vendor                                |

LIMS-emitted diagnostic results sign with the lab's
JWS key; the WIA diagnostic record stores the
signature payload and the lab's accreditation body
reference.

## §3 Supply-chain traceability

Confirmed pest events on a field unit propagate to:

| Standard / system           | Binding                                |
|-----------------------------|----------------------------------------|
| WIA-agricultural-supply-chain | lot inheritance of pest-event history |
| WIA-food-traceability       | regulator notification path            |
| GS1 EPCIS                   | event-based supply-chain trace         |
| GS1 GTIN / GLN / SSCC       | container / pallet identifier          |

A harvested lot from a field unit with an unresolved
pest event carries a `pestEventDigest` summarising the
event chain so a buyer's QA party can decide on
acceptance, sampling, or rejection.

## §4 MRL / PHI binding

Chemical-intervention decisions cite the applicable
MRL identifier:

| Authority           | MRL list                                       |
|---------------------|------------------------------------------------|
| Codex Alimentarius  | Codex MRLs                                     |
| EU                  | EU MRL database (Reg 396/2005 + Annexes)       |
| US                  | EPA MRL / tolerances (40 CFR Part 180)          |
| Japan               | MHLW Positive List                             |
| Korea               | MFDS PLS (Positive List System)                |
| Codex (Korea / JP)  | bilateral / regional alignment as published   |

The intervention record's `mrlBinding` resolves through
the authority's published list at the time of
application; subsequent MRL changes do not retro-
actively invalidate prior compliant applications, but
trigger a review event on lots not yet harvested.

## §5 Remote-sensing provider integration

| Provider             | Binding                                       |
|----------------------|-----------------------------------------------|
| ESA Sentinel-2 / -1  | Copernicus Open Access Hub (S2 MSI L1C, L2A;  |
|                      | S1 GRD)                                       |
| USGS Landsat 8 / 9   | Landsat Collection 2 (L1, L2)                  |
| NASA HLS             | Harmonized Landsat-Sentinel L30 / S30          |
| PlanetScope          | Planet API                                    |
| Maxar                | Maxar SecureWatch / GBDX                       |
| UAV multi-spectral   | local raw + GeoTIFF + AgGateway ADAPT         |

Imagery records cite the provider's tile / scene
identifier; derived indices (NDVI, NDRE, NDWI, GNDVI)
record the algorithm and reference equation.

## §6 Tractor / sprayer fleet integration (ISOBUS)

Variable-rate intervention records exchange with
ISOBUS-equipped sprayers via Task Controller messages.
Application records ingest the as-applied data
(actual rate, application time, weather summary, GPS
trace) so the audit chain records what was applied
versus what was prescribed.

## §7 Weather-service feed

Application gating (drift, washout) consumes weather
data from:

| Provider              | Service                                      |
|-----------------------|----------------------------------------------|
| WMO member services   | per-country meteorological agency             |
| ECMWF                 | open / commercial APIs                       |
| NOAA NWS              | US National Weather Service                  |
| KMA (Korea)           | open data portal                             |
| JMA (Japan)           | met agency open data                         |

Wind-speed, wind-direction, temperature, relative
humidity, and precipitation forecast / observation
windows are recorded on the application record.

## §8 Cross-domain WIA bindings

| Companion standard          | Binding purpose                                |
|-----------------------------|------------------------------------------------|
| WIA-precision-agriculture   | variable-rate prescription                     |
| WIA-agricultural-supply-chain | harvested-lot inheritance                    |
| WIA-food-traceability       | regulator notification path                    |
| WIA-agricultural-drone      | UAV survey mission binding                     |
| WIA-soil-sensor             | soil-borne pest correlation                    |
| WIA-smart-irrigation        | irrigation event ↔ disease-conducive period   |
| WIA-yield-prediction        | yield-impact estimate per pest event           |
| WIA-content-ai              | AI-pest-classification governance              |

Each binding identifies the consumed PHASE.

## §9 Long-term archival

| Authority / context | Retention                                       |
|---------------------|-------------------------------------------------|
| NPPO                | per national rules; typically ≥ 5 years         |
| EU (Reg 1107/2009)  | 10 years for pesticide application records     |
| US EPA              | per state-level rules; FIFRA retention          |
| KR MFDS PLS         | per national rules; minimum 3 years for PLS    |
| ISO 17025 lab       | ≥ 5 years post-result for traceability          |

## §10 Conformance test suite

The reference test suite covers:

- chain-of-custody completeness on a synthetic sample
- diagnostic-pathway gate enforcement (NPPO report
  blocked without lab signature)
- IPM-hierarchy enforcement on chemical-intervention
  decisions
- MRL / PHI gate on pesticide application
- alert-acknowledgement escalation (unacknowledged
  critical alert escalates after 2 hours)
- IPPC submission cross-walk on a confirmed
  quarantine-pest event
- ISOBUS task-controller round-trip on a variable-rate
  spray prescription
- audit-chain hash continuity

## §11 Internationalisation

User-facing strings (alerts, dashboards, scout app
forms) carry the BCP 47 language tag. Country-
specific regulator paths are resolved by the field-
unit's `region` (ISO 3166-2 sub-national).

## §12 Security and privacy posture

- Transport: TLS 1.3 with mutual TLS for NPPO ↔ IPPC
  exchanges; sponsor-issued mutual TLS for edge devices
- Authentication: client_credentials with key
  attestation for sponsor / lab / NPPO integrations
- At-rest: AES-256-GCM with sponsor-controlled KMS;
  per-field-unit key wrapping per ISO/IEC 27002 §8.24
- Audit: tamper-evident chain (PHASE 3 §11) exportable
  per ISO/IEC 27037 forensic-evidence guidance
- Privacy: farmer / scout identifiers opaque on cross-
  border exchange; re-linkage held by sponsor / NPPO
  under regulator-approved DPIA
- Connectivity: edge devices operate offline-first;
  observation, trap, inference, and decision records
  queue locally and submit on reconnect; queue
  integrity is hash-chained

## §13 Operational metrics

Sponsors report (informationally) on the WIA registry:

- field units active vs. fallow
- observations per field unit per week (proxy for
  surveillance intensity)
- diagnostic-pathway TAT (suspect → confirmed)
- intervention-decision IPM-tier distribution
- regulator-clock compliance rate (NPPO report timing)
- alert acknowledgement rate

## §14 Recovery and continuity

- API outage — local edge capture; sync on reconnect
- IPPC outage — queued NPPO submissions; replay on
  recovery
- LIMS outage — diagnostic results held at the lab and
  signed offline; replay on reconnect
- imagery-provider outage — local tile mirror with
  SHA-256 manifest; hot-fail to alternate provider

## Annex A — Worked end-to-end example (informative)

A grape-growing cooperative deploys a network of 60
pheromone traps for *Lobesia botrana* across 1200 ha.
Edge-AI inference on weekly orchard imagery posts to
PHASE 2 §6. A trap network exceeds the action threshold
in week 18. Sample submits to the ISO 17025 lab; PM
7/26 is run; result confirms *L. botrana*. The NPPO is
not engaged (this is an established, regulated-non-
quarantine pest in the region) but the buyer's QA
party receives the alert. The decision-record applies a
biological-tier intervention (mating disruption with
pheromone dispensers) per the IPM hierarchy. Application
records bind to the audit chain. End-of-season
yield-prediction binds the pest-event chain to the
harvested lot.

## Annex B — Conformance disclosure

Implementations declare the IPPC and RPPO bindings
supported, the LIMS protocols accepted, the imagery-
provider integrations enabled, the EPPO Code dictionary
version, and the ISOBUS task-controller capability.
Disclosure is machine-readable at `/.well-known/wia-
pest-conformance.json`.

## Annex C — Versioning

Adding a new RPPO gateway is minor; changing the IPPC
submission format is major.
