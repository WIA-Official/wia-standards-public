# WIA-predictive-maintenance PHASE 1 — Data Format Specification

**Standard:** WIA-predictive-maintenance (WIA-IND-026)
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for
WIA-predictive-maintenance, the cross-OEM
predictive maintenance interoperability standard.
The records bind every asset, sensor, telemetry
stream, anomaly, prognosis (RUL), maintenance
order, and digital-twin model to documented
industrial-IoT and asset-management standards so
that operators, OEMs, MRO providers, and auditors
can coordinate maintenance without per-vendor lock-
in.

References (CITATION-POLICY ALLOW only):
- ISO 10303 STEP (Product data exchange, informative)
- ISO 10816 / ISO 20816 (Mechanical vibration evaluation)
- ISO 13374-1..-7 (Condition monitoring and diagnostics of machines)
- ISO 17359:2018 (General guidelines for condition monitoring)
- ISO 18436-1..-8 (Personnel qualification for condition monitoring)
- ISO 13379 (Data interpretation and diagnostics techniques)
- ISO 13381-1 (Prognostics general guidelines)
- ISO 14224:2016 (Reliability and maintenance data)
- ISO 55000 / 55001 / 55002 (Asset management)
- ISO/IEC 27001:2022, ISO/IEC 27017 (cloud security), ISO/IEC 27701:2019
- IEC 61508 (Functional safety, informative), IEC 61511 (Process industries)
- IEC 62443 series (Industrial cybersecurity)
- IEEE 802.1Q TSN (Time-Sensitive Networking, informative)
- OPC UA (IEC 62541) Companion Specifications (Pumps, Robotics, Machinery)
- MTConnect (ISO 23247-2 Digital Twin Manufacturing, MTConnect schema)
- MQTT v5.0, AMQP 1.0, HTTP/2 / HTTP/3
- Sparkplug B Specification (Eclipse Tahu, informative)
- Asset Administration Shell (AAS) IEC 63278-1
- IETF RFC 4122 (UUID), RFC 8259 (JSON), RFC 8785 (JCS), RFC 7515 (JWS)
- ISA-95 / IEC 62264 (Manufacturing operations), ISA-88 (Batch control)
- ANSI/ISA-101.01 (Human–machine interface)

---

## §1 Scope

This PHASE applies to records that describe
industrial assets, the sensors that monitor them,
the telemetry the sensors emit, the anomalies and
prognoses produced from the telemetry, the
maintenance orders triggered, and the digital-twin
models that close the loop.

In scope: asset record, sensor record, telemetry
stream record, signal-condition record, anomaly
record, prognosis (RUL) record, maintenance order
record, work-order completion record, twin-model
record, and the cross-references binding each
record to its OEM identity, applicable
standardisation, and asset-management policy.

Out of scope: the reliability-engineering analysis
itself (handled by the operator's reliability
team); regulatory plant-safety obligations under
IEC 61508 / 61511 / OSHA / Seveso (handled by the
operator's process-safety regime).

## §2 Asset record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `assetRef`           | UUID (RFC 4122) opaque                          |
| `lei`                | ISO 17442 LEI of the operator                   |
| `oemRef`             | OEM identity URI                                |
| `udi`                | unique device identifier (FDA UDI when medical) |
| `taxonomy`           | ISO 14224:2016 taxonomy class                   |
| `parentAsset`        | optional parent asset reference                  |
| `serviceClass`       | ISO 14224 §6 functional / operational class     |
| `lifecycleState`     | `installed`, `operating`, `idle`, `removed`     |
| `aasRef`             | optional Asset Administration Shell reference   |

`assetRef` is the only invariant identifier across
ownership transfers; OEM-internal serial numbers
are recorded but not authoritative.

## §3 Sensor record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `sensorRef`          | UUID                                            |
| `assetRef`           | this PHASE §2                                   |
| `kind`               | `accelerometer`, `temperature`, `current`,      |
|                      | `voltage`, `acoustic-emission`, `oil-debris`,   |
|                      | `pressure`, `flow`, `strain`, `image`,          |
|                      | `infrared`, `ultrasound`                        |
| `mountingPosition`   | ISO 13373-1 mounting nomenclature               |
| `range`              | physical range with units                       |
| `samplingRate`       | Hz                                              |
| `accuracy`           | per ISO 5725 / manufacturer specification       |
| `calibrationRef`     | URI to calibration record                       |

## §4 Telemetry stream record

A telemetry stream is the smallest replayable unit
of monitoring data.

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `streamRef`          | URI                                             |
| `sensorRef`          | this PHASE §3                                   |
| `transport`          | `opc-ua`, `mqtt-5`, `amqp-1.0`, `mtconnect`,   |
|                      | `http-streaming`                                 |
| `encoding`           | `json`, `cbor`, `protobuf`, `apache-avro`       |
| `unit`               | UCUM code or SI unit                            |
| `samplingPolicy`     | continuous, on-change, scheduled, event         |
| `qosClass`           | `best-effort`, `reliable`, `safety-critical`    |

## §5 Signal-condition record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `conditionRef`       | UUID                                            |
| `streamRef`          | this PHASE §4                                   |
| `feature`            | `rms-velocity`, `spectrum-1x`, `spectrum-2x`,   |
|                      | `kurtosis`, `crest-factor`, `enclosed-energy`,  |
|                      | `oil-iso-4406`, `acoustic-spectrum`             |
| `unit`               | UCUM code                                       |
| `severity`           | `none`, `acceptable`, `unsatisfactory`,         |
|                      | `unacceptable` per ISO 10816 / ISO 20816        |
| `threshold`          | configured threshold per ISO 13373-3           |
| `evaluatedAt`        | ISO 8601                                        |

## §6 Anomaly record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `anomalyRef`         | UUID                                            |
| `assetRef`           | this PHASE §2                                   |
| `triggerRef[]`       | conditionRef list                               |
| `failureModeCode`    | ISO 14224:2016 §B.2 failure mode code           |
| `severity`           | `none`, `incipient`, `developed`, `severe`,     |
|                      | `failed`                                        |
| `confidence`         | 0..1                                            |
| `detectedAt`         | ISO 8601                                        |
| `detectionAlgorithm` | URI to algorithm card                           |

## §7 Prognosis (RUL) record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `prognosisRef`       | UUID                                            |
| `assetRef`           | this PHASE §2                                   |
| `anomalyRef`         | optional this PHASE §6                          |
| `rulEstimate`        | remaining useful life with units (hours,        |
|                      | cycles, kWh)                                    |
| `confidenceInterval` | low / high bounds                               |
| `framework`          | ISO 13381-1 prognostic framework reference       |
| `modelCardRef`       | URI to model card (ISO/IEC 23053 / 23894 form)  |
| `lastUpdated`        | ISO 8601                                        |

## §8 Maintenance order record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `orderRef`           | UUID                                            |
| `assetRef`           | this PHASE §2                                   |
| `triggerRef`         | optional anomaly / prognosis reference          |
| `kind`               | `condition-based`, `predictive`, `preventive`,  |
|                      | `corrective`, `safety-critical`                 |
| `priority`           | `routine`, `urgent`, `emergency`                |
| `plannedStart`       | ISO 8601                                        |
| `plannedDuration`    | ISO 8601 duration                               |
| `assigneeRef`        | technician / contractor identifier              |
| `partsList[]`        | replacement parts with EAN / GTIN / OEM PN      |

## §9 Work-order completion record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `completionRef`      | UUID                                            |
| `orderRef`           | this PHASE §8                                   |
| `closedAt`           | ISO 8601                                        |
| `findings`           | localised technician findings                   |
| `failureModeCode`    | ISO 14224 §B.2 failure mode code                |
| `replacedParts[]`    | replaced parts with serial numbers              |
| `evidenceRef[]`      | photos, oil-analysis report, inspection log     |

## §10 Twin-model record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `twinRef`            | URI                                             |
| `assetRef`           | this PHASE §2                                   |
| `aasReference`       | Asset Administration Shell reference (IEC 63278)|
| `physicsBasis`       | declared first-principles / data-driven /       |
|                      | hybrid model                                    |
| `simulationEngine`   | engine identifier                               |
| `interfaceContract`  | OPC UA / MTConnect / FMI 3.0 binding            |
| `validityWindow`     | ISO 8601 duration since last calibration       |

## §11 Cross-domain references (informative)

- WIA-energy-management — energy-related KPIs
- WIA-supply-chain — spare-parts logistics
- WIA-cybersecurity-industrial — IEC 62443 binding
- WIA-language-bridge — multilingual work orders

## Annex A — Conformance disclosure

Implementations declare the schema versions they
support, the canonicalisation form (RFC 8785), and
the JWS key set used to sign anomaly, prognosis,
and order records.

## Annex B — Worked anomaly record (informative)

```json
{
  "anomalyRef": "f63f4f04-9a76-4d2c-9f53-2c4d09a1bbb1",
  "assetRef": "asset-072",
  "triggerRef": ["cond-rms-velocity-2026-04-28"],
  "failureModeCode": "FOF-BRG",
  "severity": "developed",
  "confidence": 0.92,
  "detectedAt": "2026-04-28T11:32:00+09:00"
}
```

## Annex C — Versioning

Field additions are minor; field removals or
semantic redefinition require a major bump
synchronised with ISO 14224 and ISO 13374
revisions.

## Annex D — Conformance level

Conformance is "Core" (asset + sensor + stream +
condition + anomaly + order) or "Full" (adds
prognosis, completion, and twin-model records).

## Annex E — Privacy

Personal data appearing in maintenance records
(technician identity, supervisor approval) is
processed under the operator's privacy regime.
Pseudonymisation is the default for cross-OEM
analytics joins.

## Annex F — Cybersecurity

Records are produced and consumed under IEC 62443
zone and conduit segregation. Telemetry from
safety-critical zones (SIL-rated) crosses
conduits only via gateway transformations
documented in the operator's cybersecurity policy.

## Annex G — Calibration cadence

Sensor calibration records carry expiry dates per
ISO 17025. Anomaly and prognosis records derived
from out-of-calibration sensors are flagged
`calibration-stale` until the sensor is
re-calibrated.

## Annex H — Failure-mode taxonomy

ISO 14224:2016 §B.2 enumerates failure modes per
equipment class. Common codes include:

| Code   | Description                                       |
|--------|---------------------------------------------------|
| FOF-BRG| Bearing seizure / failure                         |
| FOF-LUB| Lubrication-system failure                        |
| FOF-VIB| Excessive vibration                               |
| FOF-OVH| Overheating                                       |
| FOF-LOO| Looseness / misalignment                          |
| FOF-COR| Corrosion                                         |
| FOF-FAT| Fatigue cracking                                  |
| FOF-WEAR | Surface wear                                    |

Operators MAY add jurisdiction-specific extensions
under `FOF-X-<code>` namespaces; extensions do not
override the canonical taxonomy.

## Annex I — Operating context

Each anomaly carries an operating-context block:

| Field           | Source / Binding                              |
|-----------------|-----------------------------------------------|
| `loadFraction`  | 0..1 nominal load fraction                    |
| `dutyCycle`     | continuous / cyclic / intermittent / standby  |
| `processStage`  | ISA-88 procedural element if batch context    |
| `ambientTemp`   | °C; affects ISO 10816 zone interpretation     |

Operating context is informative; it accompanies
the anomaly so that auditors can reproduce the
condition under which the anomaly was raised.

## Annex J — Reliability KPI summary

Each operator publishes a monthly KPI summary:

| KPI         | Definition                                    |
|-------------|-----------------------------------------------|
| MTBF        | mean time between failures per ISO 14224      |
| MTTR        | mean time to repair                           |
| Availability| operational time / scheduled time             |
| OEE         | quality × performance × availability          |

KPI summaries reference the underlying record set
so that auditors can recompute the KPI from
primary records.

弘益人間 (Hongik Ingan) — Benefit All Humanity
