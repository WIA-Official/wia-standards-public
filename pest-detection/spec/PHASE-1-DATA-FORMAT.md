# WIA-pest-detection PHASE 1 — Data Format Specification

**Standard:** WIA-pest-detection
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for crop-
pest detection covering field-, greenhouse-, orchard-,
forestry-, and stored-product contexts. Records bind
specimen and trap observations, remote-sensing imagery,
edge-AI inference, scout reports, regulatory reportable-
pest records, and intervention-decision artefacts so an
agronomist, a national plant-protection organisation
(NPPO) inspector, or a buyer's quality-assurance party
can reconstruct the lifecycle of any pest event.

References (CITATION-POLICY ALLOW only):
- IPPC International Plant Protection Convention — ISPM 1, ISPM 5 (Glossary), ISPM 6 (Surveillance), ISPM 8 (Pest status), ISPM 17 (Pest reporting), ISPM 23 (Inspection)
- ISPM 27 — Diagnostic protocols for regulated pests; ISPM 31 — Sampling of consignments
- EPPO Diagnostic Standards (PM 7 series) and EPPO PRA (PM 5)
- EPPO Code (Bayer Code / EPPO Code) — pest taxonomy reference
- FAO Crop Production / Plant Protection guidance
- Codex Alimentarius MRL — maximum residue limits for harvested produce
- ISO 17025:2017 — testing laboratory accreditation (diagnostic labs)
- ISO 19115-2:2019 — geographic information metadata (imagery)
- ISO 19139 — geographic information metadata XML
- OGC GeoTIFF, OGC SensorThings, OGC GeoPackage
- ISO 11783 (ISOBUS) — agricultural electronics (tractor-mounted sensors)
- ISO 14961 / AgGateway ADAPT — agricultural exchange of business-process data
- ISO 8601 (date / time), ISO 3166 (country / region), BCP 47 (language)
- IETF RFC 8259 (JSON), RFC 8785 (JCS), RFC 4122 (UUID), RFC 7515 (JWS), RFC 9530 (Content-Digest)
- WIPO ST.26 — sequence listing format (where pathogen identification is genomic)
- LSID (Life Science Identifier) — taxon-identity URI scheme

---

## §1 Scope

This PHASE applies to systems that detect, classify,
report, and trigger response to plant pests (insects,
mites, plant-pathogenic fungi, oomycetes, bacteria,
viruses, viroids, phytoplasmas, parasitic plants, weeds,
and quarantine pests as defined under ISPM 5).

In scope: field-unit record, observation record, trap
record, sample record, diagnostic-result record, edge-
inference record, remote-sensing record, alert record,
intervention-decision record, NPPO reporting record,
and the cross-references binding pest events to traceable
agricultural lots.

Out of scope: animal-veterinary pest control (handled
by veterinary standards), urban structural pest control
(handled by urban-pest standards), and stored-grain
mycotoxin assays beyond pest-causation (handled by
food-safety standards).

## §2 Field-unit record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `fieldUnitRef`       | UUID (RFC 4122)                                 |
| `geometry`           | OGC Simple Features WKT or GeoJSON polygon      |
| `crs`                | EPSG identifier (e.g. EPSG:4326)                |
| `crop`               | EPPO Code or ITIS TSN; cultivar at sub-level    |
| `phenologyStage`     | BBCH-scale code (00–99 across cereals, fruit,   |
|                      | vegetables, ornamentals)                        |
| `productionSystem`   | open-field / greenhouse / tunnel / orchard /    |
|                      | nursery / forestry / stored-product             |
| `irrigationMethod`   | rainfed / drip / sprinkler / flood              |
| `ipmTier`            | per FAO IPM tiers — preventive, monitoring,     |
|                      | intervention, evaluation                        |
| `region`             | ISO 3166-2 sub-national region                  |
| `farmRef`            | upstream farm / property identifier              |
| `seasonRef`          | growing-season identifier                       |

Field units bind to the agricultural-supply-chain
standard so harvested lots inherit the pest-event
history.

## §3 Observation record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `observationRef`     | UUID                                            |
| `fieldUnitRef`       | §2                                              |
| `observationTime`    | ISO 8601 with timezone                          |
| `observerKind`       | scout / agronomist / farmer / drone / fixed-cam |
|                      | / pheromone-trap / yellow-sticky-trap / light-  |
|                      | trap / sentinel-plant / extension-officer       |
| `observerRef`        | identity (operator UUID for human; device UUID  |
|                      | for sensor)                                     |
| `pestCandidate`      | EPPO Code; multi-candidate lists allowed        |
| `count`              | numeric per unit area or per trap               |
| `severityScale`      | EPPO standardised severity per crop / pest      |
| `lifeStage`          | egg / nymph / instar-N / pupa / adult /         |
|                      | mycelial / sporulating                          |
| `mediaRef`           | URI to imagery / audio / SEM / micrograph       |
| `notes`              | free-text up to 4 KiB                           |

Observation records may stand alone (visual scout
report) or bind to a downstream sample record for lab
confirmation.

## §4 Trap record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `trapRef`            | UUID                                            |
| `kind`               | pheromone / yellow-sticky / light / pitfall /   |
|                      | malaise / interception                          |
| `lureRef`            | for pheromone: lure compound + manufacturer +   |
|                      | release-rate; freshness window                  |
| `position`           | OGC point (lat / lon / elevation)               |
| `installedAt`        | ISO 8601                                        |
| `serviceCadence`     | days between checks                             |
| `lastService`        | ISO 8601                                        |
| `replacementDue`     | ISO 8601                                        |

Trap captures bind to observation records; a single
trap record may have many observation records over its
operating life.

## §5 Sample record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `sampleRef`          | UUID                                            |
| `observationRef`     | §3 (where the sample originates from a flagged  |
|                      | observation)                                    |
| `sampleType`         | leaf / fruit / stem / root / soil / insect /    |
|                      | trap-catch / inflorescence / seed / wood        |
| `collectionTime`     | ISO 8601                                        |
| `collectorRef`       | scout / inspector identity                      |
| `chainOfCustody[]`   | events (transport temperature, container,       |
|                      | handover signature)                             |
| `labRef`             | accredited laboratory identifier (ISO 17025)    |
| `submissionTime`     | ISO 8601                                        |

## §6 Diagnostic-result record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `diagnosticRef`      | UUID                                            |
| `sampleRef`          | §5                                              |
| `methodCode`         | EPPO PM 7 protocol identifier (e.g. PM 7/119    |
|                      | for *Xylella fastidiosa*); ISPM 27 reference    |
| `methodKind`         | morphological / DNA-barcoding / qPCR / RT-PCR / |
|                      | LAMP / NGS-metagenomic / serological-ELISA      |
| `confirmedTaxon`     | EPPO Code + LSID + canonical scientific name    |
|                      | (per ITIS / GBIF / Catalogue of Life)           |
| `confidence`         | per-method qualitative / quantitative           |
| `analystRef`         | accredited analyst identity                     |
| `reportTime`         | ISO 8601                                        |
| `replicates`         | per-replicate Ct, melt, sequence accession      |
|                      | (where sequencing applied; INSDC accession)     |

Diagnostic results that confirm a quarantine pest
trigger the NPPO reporting workflow (§9).

## §7 Edge-inference record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `inferenceRef`       | UUID                                            |
| `mediaRef`           | URI to source image / spectral cube              |
| `modelRef`           | model identifier + semantic version + container |
|                      | digest                                          |
| `pipelineRef`        | preprocessing + post-processing chain reference |
| `predictions`        | per-class probability vector with EPPO Code keys|
| `confidence`         | scalar top-1 + entropy + per-class margin       |
| `provenanceRef`      | edge-device serial + firmware version           |

Edge-inference records that exceed an alert threshold
trigger an alert record (§10) but do not by themselves
enter the NPPO report. NPPO reporting requires
diagnostic confirmation (§6) per ISPM 17.

## §8 Remote-sensing record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `imageryRef`         | UUID                                            |
| `sensor`             | satellite (Sentinel-2 MSI, Sentinel-1 SAR,      |
|                      | Landsat 8/9, PlanetScope) / aerial (UAV multi-  |
|                      | / hyperspectral) / fixed (RGB / NIR / IR)        |
| `acquisitionTime`    | ISO 8601                                        |
| `bands`              | per-band centre wavelength (nm)                  |
| `coverage`           | OGC polygon                                     |
| `groundTruthRef`     | optional binding to observation record(s)       |
| `vegetationIndex`    | NDVI / NDRE / NDWI / GNDVI / EVI per pixel,     |
|                      | aggregated to field-unit if requested            |
| `sourceMetadata`     | ISO 19115-2 metadata payload URI                 |

## §9 NPPO reporting record

For pests under ISPM 17 reporting obligation:

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `reportRef`          | UUID                                            |
| `nppoRef`            | issuing NPPO identifier (ISO 3166-1 country)    |
| `regulatedPestCode`  | EPPO Code; quarantine status flag                |
| `reportType`         | first-report / periodic / outbreak / eradication-|
|                      | declared                                          |
| `eventTime`          | ISO 8601                                        |
| `affectedAreaRef`    | polygon                                         |
| `controlMeasures`    | reference list                                  |
| `submissionRef`      | IPPC submission identifier (NPPO official)       |

NPPO reports propagate to the IPPC International
Phytosanitary Portal and to relevant Regional Plant
Protection Organisations (e.g. EPPO, NAPPO, COSAVE,
APPPC, IAPSC).

## §10 Alert record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `alertRef`           | UUID                                            |
| `triggerKind`        | observation / edge-inference / sensor-threshold |
| `triggerRef`         | the upstream record that produced the alert     |
| `severity`           | informational / advisory / action / critical    |
| `recipients`         | farm operator / agronomist / extension officer  |
| `dispatchTime`       | ISO 8601                                        |
| `acknowledgement`    | recipient confirmation (signed)                  |

## §11 Intervention-decision record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `decisionRef`        | UUID                                            |
| `triggerRef`         | alert / diagnostic / NPPO directive             |
| `interventionKind`   | none / cultural / biological / mechanical /     |
|                      | chemical                                        |
| `pesticideRef`       | for chemical: ISO 1750 active-ingredient name + |
|                      | local-registry product code                     |
| `dosePlanned`        | dose per ha + application timing                |
| `mrlBinding`         | Codex / national MRL identifier                 |
| `phiDays`            | pre-harvest interval (days)                     |
| `applicatorRef`      | trained applicator credential identifier         |
| `recordOfApplication`| application record per local regulator          |

Intervention decisions cite the IPM hierarchy and the
trigger; chemical interventions cite the MRL and PHI
so a downstream lot inherits the necessary pre-harvest
interval.

## §12 Cross-domain references (informative)

- WIA-precision-agriculture — variable-rate prescription
- WIA-agricultural-supply-chain — lot inheritance
- WIA-food-traceability — regulator notification
- WIA-agricultural-drone — UAV mission binding
- WIA-soil-sensor — soil-borne pest correlation

## Annex A — Worked alert payload (informative)

```json
{
  "alertRef": "alert-2026-04-12-1834",
  "triggerKind": "edge-inference",
  "triggerRef": "inference:tomato-leafminer:0.93",
  "severity": "action",
  "recipients": ["farm:fieldA","agronomist:KE-NRT-ext-007"],
  "dispatchTime": "2026-04-12T09:14:00+03:00"
}
```

## Annex B — EPPO Code coverage

The standard requires EPPO Code at the genus + species
resolution where available. Sub-species or pathotype is
recorded on `confirmedTaxon.subspecies` when the
diagnostic protocol resolves to that level.

## Annex C — Conformance disclosure

Implementations declare the JSON-Schema URIs they
serve, the canonicalisation form (RFC 8785), and the
NPPO bindings they participate in.

## Annex D — Versioning

Field additions are minor; semantic redefinition is
major.
