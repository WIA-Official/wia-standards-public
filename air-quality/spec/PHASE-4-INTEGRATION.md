# WIA-ENE-017: Air Quality Monitoring Standard
## PHASE 4: INTEGRATION

**Document Version:** 1.1
**Status:** Active
**Last Updated:** April 25, 2026
**Philosophy:** 弘益人間 (Hongik Ingan) · Benefit All Humanity

---

## 1. Introduction

Phase 4 specifies how a conformant WIA-ENE-017 network **reaches the outside world**: regulatory reporting, public-health alerting, smart-city control loops, scientific data lakes, citizen-sensor federation, and long-horizon archive pipelines. Phases 1–3 defined *what* we ship and *how* we ship it; Phase 4 defines *to whom* and *under what contract*.

The objective is bidirectional: a WIA station's data lands in a cardiologist's EHR, a traffic-light controller, a WMO global broker, and an open-data dashboard **without any service brokering a proprietary format**.

### 1.1 Normative references

- OGC 15-078r6 — SensorThings API 1.1
- OGC 19-086r6 — Environmental Data Retrieval (EDR) API 1.1
- WMO-306 Manual on Codes Vol. I.3 — BUFR/CREX
- WMO WIS 2.0 Technical Regulations
- OASIS CAP 1.2 — Common Alerting Protocol
- HL7 FHIR R5 — Observation, Patient, Encounter resources
- US EPA NAAQS 40 CFR 50
- WHO Global Air Quality Guidelines (2021)
- EU Directive 2008/50/EC (amended 2024/2881) — Ambient Air Quality
- Apache Parquet 2.9; Apache Iceberg 1.5; Delta Lake 3.1
- Barkjohn et al. 2021 — EPA PurpleAir correction formula

---

## 2. Integration architecture

```
                  ┌──────────────────────────────────────────┐
                  │              WIA Ingest API              │
                  │  (Phase 2 REST/WebSocket/GraphQL front)  │
                  └──────────────────────────────────────────┘
                                     │
        ┌────────────┬───────────────┼───────────────┬─────────────┐
        ▼            ▼               ▼               ▼             ▼
   SensorThings   WMO WIS 2.0    FHIR Gateway    CAP Publisher  Data Lake
   bridge         (BUFR exporter)                (Emergency)    (Parquet)
        │            │               │               │             │
        ▼            ▼               ▼               ▼             ▼
   OpenAQ /     Global broker   Hospital EHR    IPAWS / KR-EAS  Analytics /
   EEA / PurpleAir (CAMS, ECMWF) (Epic, Cerner)    / EU-EDM     Research
```

Every outward arrow is a *pull-or-push* connector that reads Phase 1 records and emits them in the target schema. The central ingest API remains the single source of truth; integrations are **idempotent readers**, never silent transformers.

---

## 3. OGC SensorThings API 1.1 bridge

### 3.1 Entity correspondence

Phase 2 (§ 6) already defined the URI mapping. Phase 4 extends it to the observation value-type:

| WIA field (Phase 1) | SensorThings property                    |
|---------------------|------------------------------------------|
| `measurements.*.value`   | `Observations.result`               |
| `measurements.*.unit`    | `Datastreams.unitOfMeasurement.symbol` |
| `measurements.*.qc_flag` | `Observations.resultQuality`        |
| `measurements.*.uncertainty` | `Observations.parameters.uncertainty` |
| `measurements.*.method`  | `Sensors.metadata.method`           |
| `location`               | `Locations.location` (GeoJSON Point) |
| `aqi.value`              | `Observations.parameters.aqi`       |

### 3.2 Observed properties

Each pollutant has a stable IRI, aligned with EEA Vocabulary `pollutant-concept`:

```
http://dd.eionet.europa.eu/vocabulary/aq/pollutant/1          # SO2
http://dd.eionet.europa.eu/vocabulary/aq/pollutant/5          # PM10
http://dd.eionet.europa.eu/vocabulary/aq/pollutant/6001       # PM2.5
http://dd.eionet.europa.eu/vocabulary/aq/pollutant/7          # O3
http://dd.eionet.europa.eu/vocabulary/aq/pollutant/8          # NO2
http://dd.eionet.europa.eu/vocabulary/aq/pollutant/10         # CO
```

This lets EEA's Reportnet 3, OpenAQ, and ECMWF CAMS ingest a WIA feed without any local mapping table.

---

## 4. WMO WIS 2.0 and BUFR export

For exchange with national meteorological services, pollutants MUST be encoded as BUFR sequence 307080 (Meteorological surface observation) extended with table B entries:

| Descriptor | Meaning              | Units  |
|------------|----------------------|--------|
| 0 15 195   | Mass concentration of PM2.5 in air | μg/m³ |
| 0 15 196   | Mass concentration of PM10 in air  | μg/m³ |
| 0 15 192   | Ozone mixing ratio              | ppb   |
| 0 15 193   | Nitrogen dioxide mixing ratio   | ppb   |
| 0 15 191   | Sulfur dioxide mixing ratio     | ppb   |
| 0 15 023   | Carbon monoxide mixing ratio    | ppm   |

Packaging:

- BUFR message (bulletin header `IUAQnn CCCC`, centre ID 220 = WIA),
- wrapped in WMO WIS 2.0 MQTT envelope with `pubsub.properties.data_id = wis2:org.wia.aq/observation/<station_id>/<timestamp>`,
- signed with the station's JWS (Phase 3 § 8.3) in `pubsub.properties.security`.

Reference tooling: `cli/wis2-publish.py` uses `ecCodes` for BUFR encoding and paho-mqtt for WIS 2.0 dissemination.

---

## 5. Health-system integration (HL7 FHIR R5)

Clinical triggers for vulnerable populations (asthma, COPD, cardiovascular) MUST emit an `Observation` resource tied to the patient's `Encounter` when their residential AQI exceeds clinician-defined thresholds.

```json
{
  "resourceType": "Observation",
  "status": "final",
  "category": [{
    "coding": [{
      "system": "http://terminology.hl7.org/CodeSystem/observation-category",
      "code": "environment"
    }]
  }],
  "code": {
    "coding": [{
      "system": "http://loinc.org",
      "code": "88293-5",
      "display": "Ambient PM2.5 concentration"
    }]
  },
  "subject": { "reference": "Patient/patient-with-asthma-0421" },
  "effectiveDateTime": "2026-04-25T14:30:00Z",
  "valueQuantity": {
    "value": 78.2, "unit": "ug/m3",
    "system": "http://unitsofmeasure.org", "code": "ug/m3"
  },
  "interpretation": [{
    "coding": [{
      "system": "http://terminology.hl7.org/CodeSystem/v3-ObservationInterpretation",
      "code": "HH", "display": "Critical high"
    }]
  }],
  "device": { "reference": "Device/WIA-AQ-KR-Seoul-001" },
  "note": [{ "text": "AQI 162 (Unhealthy). WHO 24 h AQG = 15 μg/m³." }]
}
```

LOINC codes used (stable):

| Pollutant | LOINC  | Display                               |
|-----------|--------|---------------------------------------|
| PM2.5     | 88293-5| Ambient PM2.5 mass concentration      |
| PM10      | 88295-0| Ambient PM10 mass concentration       |
| O3        | 88296-8| Ambient ozone                          |
| NO2       | 88298-4| Ambient nitrogen dioxide               |
| SO2       | 88299-2| Ambient sulfur dioxide                 |
| CO        | 88300-8| Ambient carbon monoxide                |

---

## 6. Emergency alerting — OASIS CAP 1.2

### 6.1 Triggering rules

An alert MUST be raised when **any** of the following is true over a station's declared service area:

| Pollutant | Level          | Threshold (1 h)   | CAP severity |
|-----------|----------------|-------------------|--------------|
| PM2.5     | Unhealthy      | ≥ 55 μg/m³        | Moderate     |
| PM2.5     | Very unhealthy | ≥ 150 μg/m³       | Severe       |
| PM2.5     | Hazardous      | ≥ 250 μg/m³       | Extreme      |
| O3        | Unhealthy      | ≥ 165 ppb (1 h)   | Moderate     |
| NO2       | Unhealthy      | ≥ 360 ppb         | Moderate     |
| SO2       | Unhealthy      | ≥ 305 ppb         | Moderate     |

Thresholds align with US EPA NAAQS and the WHO 2021 interim targets; regulators MAY override via `/admin/alert-thresholds` but MUST record the active override in every emitted alert's `parameter[override=true]`.

### 6.2 CAP 1.2 skeleton

```xml
<alert xmlns="urn:oasis:names:tc:emergency:cap:1.2">
  <identifier>wia-aq-2026-04-25-KR-Seoul-00421</identifier>
  <sender>alerts@airquality.wia.org</sender>
  <sent>2026-04-25T14:35:00+09:00</sent>
  <status>Actual</status>
  <msgType>Alert</msgType>
  <scope>Public</scope>
  <info>
    <category>Env</category>
    <event>Elevated PM2.5</event>
    <urgency>Expected</urgency>
    <severity>Severe</severity>
    <certainty>Observed</certainty>
    <headline>Unhealthy air — PM2.5 at 162 μg/m³</headline>
    <description>Seoul City Hall monitor WIA-AQ-KR-Seoul-001 recorded 162 μg/m³
      PM2.5 over the past hour. Sensitive groups should remain indoors.</description>
    <instruction>Close windows. N95/KF94 masks recommended outdoors.</instruction>
    <parameter><valueName>aqi</valueName><value>212</value></parameter>
    <area>
      <areaDesc>Jung-gu, Seoul (5 km radius)</areaDesc>
      <circle>37.5665,126.9780 5</circle>
    </area>
  </info>
</alert>
```

### 6.3 Downstream channels

- **United States**: IPAWS-OPEN ingest; `eventCode` `AQA` if present.
- **Korea**: KR-EAS (재난문자) gateway operated by MOIS; see `cli/kr-eas-publish.py`.
- **European Union**: EDM/EU-Alert routed via national authorities.
- **Mobile devices**: FCM/APNs topics `aq.alerts.{country}.{city}`.
- **Web**: Schema.org `SpecialAnnouncement` for SEO + browser `push`.

---

## 7. Smart-city control loops

Integrations are published as sanctioned actuator channels with explicit **rate caps** to prevent control oscillation.

| System                  | Trigger                     | Action                                | Max frequency |
|-------------------------|-----------------------------|---------------------------------------|---------------|
| Traffic signal plan     | PM2.5 > 100 μg/m³ for 15 min | Cordon pricing nudge; reroute freight | every 30 min  |
| Building HVAC (BACnet)  | Outdoor PM2.5 > 35 μg/m³    | Switch to recirculation, MERV-16 path | every 10 min  |
| Public transport ops    | NO2 > 200 ppb, O3 > 140 ppb | Add electric bus slots                | every 1 h     |
| School dismissal guide  | AQI > 151                    | Hold indoor PE; push advisory         | once per half-day |
| Road sprinklers         | PM10 > 200 μg/m³ & dry      | Trigger dust suppression              | every 2 h     |

Each actuation MUST be logged as `Action` records with `x-wia-source=aq:<station>` so post-hoc audits can separate air-quality-driven actions from other policies.

---

## 8. Data-lake sink (Parquet / Iceberg / Delta)

Long-term analytics uses a columnar store. Required schema (shared by Parquet, Iceberg, Delta):

```
readings/
├── station_id     STRING  (partition)
├── ingest_day     DATE    (partition)
├── timestamp      TIMESTAMP(UTC)
├── pollutant      STRING
├── value          DOUBLE
├── unit           STRING
├── qc_flag        STRING
├── uncertainty    DOUBLE
├── method         STRING
├── lat            DOUBLE
├── lon            DOUBLE
├── elevation      DOUBLE
├── aqi            INT
├── dominant       STRING
├── sensor_model   STRING
└── ingest_at      TIMESTAMP(UTC)
```

- **Iceberg table properties**: `format-version=2`, `write.target-file-size-bytes=134217728`, `write.parquet.compression-codec=zstd`.
- **Partitioning**: `station_id, ingest_day` (daily, 24 files/day for busy stations).
- **Maintenance**: nightly `rewrite_data_files`, weekly `expire_snapshots` (keep 30 days).
- **Reproducibility**: every Tier 1 station's daily close-out emits a `manifest.json` containing per-file SHA-256 hashes, pinned to the WIA Chain for auditability.

---

## 9. Forecast integration (CAMS, HRRR-Smoke, local LSTM)

WIA ingests forecasts as *signed third-party inputs* rather than authoritative observations:

- **CAMS Near-Real-Time**: O3/NO2/PM2.5/PM10 at 0.1° up to +96 h, refreshed twice daily.
- **NOAA HRRR-Smoke**: North-American smoke transport, 3 km, +48 h.
- **National LSTM/ConvLSTM**: local +24 h nowcasts trained on WIA station history.

Forecasts are exposed by `GET /stations/{id}/forecasts?source=cams|hrrr|wia-local&horizon=1h,6h,24h,72h`. Responses MUST include a `provenance` object (`model`, `run`, `fetched_at`, `citation`). Health advisories MUST distinguish observed vs. forecast data in UI copy.

---

## 10. Citizen-sensor federation

Low-cost networks (PurpleAir, Sensor.Community, AirGradient) are integrated under Tier 3 with **mandatory bias correction**.

### 10.1 EPA PurpleAir correction (Barkjohn 2021)

For PurpleAir PA-II CF=1 values:

```
PM2.5_corrected = 0.524 · PA_cf1 − 0.0852 · RH + 5.72      (μg/m³)
```

Applied when the sensor's meta indicates a PurpleAir family; the corrected value replaces the raw value in the WIA pipeline, the raw value is retained in `parameters.raw_cf1`, and `qc_flag` starts at `suspect` until the correlation against a reference monitor within 1 km (90 days) exceeds `R² > 0.7`.

### 10.2 Other networks

| Network             | Default QC       | Correction strategy                            |
|---------------------|------------------|------------------------------------------------|
| Sensor.Community    | Tier 3, `suspect`| Station-specific bias from co-located ref.     |
| AirGradient         | Tier 3, `valid`  | Vendor factory calibration honoured            |
| Civic DIY (Arduino) | Tier 4, `suspect`| Never promoted; visualised as "community" layer only |

---

## 11. Reporting and compliance exports

Regulatory deliverables are produced by deterministic pipelines:

- **EU Reportnet 3 (EEA eReporting)**: monthly `E1a` (validated assessment), `E2a` (UTD), and `G` (information model) INSPIRE-compliant GML packages.
- **EPA AQS Submission**: annual raw-data files in AQS Transaction Format (`RD|` records).
- **Korea KECO**: `EOD` nightly CSV via SFTP to the NAMIS system.

A checksum manifest and cryptographic witness (Phase 3 JWS + optional RFC 3161 timestamp) accompany every regulatory submission so any later tamper is detectable.

---

## 12. Observability and dashboards

- **Grafana datasource plugin** `wia-airquality-datasource` queries the REST API directly; saved dashboards stored under `dashboards/grafana/*.json`.
- **Prometheus exporter**: `:9117/metrics` exposes `wia_aq_pm25_ugm3{station="…"}` and `wia_aq_aqi{station="…"}`; scrape interval 60 s.
- **OpenTelemetry**: traces MUST carry attributes `wia.station`, `wia.pollutant`, `wia.qc_flag` for correlation with ingest-API latency alarms.
- **OpenAPI + AsyncAPI 3.0**: the streaming channels (§ 3 MQTT / § 7 WebSocket) are additionally documented in AsyncAPI for tooling like Microcks, Backstage, and Insomnia.

---

## 13. Worked scenarios

### 13.1 Hospital cardiology unit subscribes to a patient's home AQI

```bash
POST https://fhir.hospital.local/Subscription
Content-Type: application/fhir+json

{
  "resourceType": "Subscription",
  "status": "requested",
  "reason": "Home PM2.5 monitoring for CHF patient",
  "criteria": "Observation?patient=Patient/0421&code=88293-5",
  "channel": {
    "type": "websocket",
    "endpoint": "wss://fhir.hospital.local/ws",
    "payload": "application/fhir+json"
  }
}
```

The WIA FHIR gateway evaluates the patient's residence (consented geocoded address), maps to the nearest Tier 1 station, and emits Observation notifications when thresholds fire.

### 13.2 Traffic authority reroutes freight during a haze episode

```bash
curl -X POST https://traffic.seoul.gov.kr/api/policy/freight-cordon \
  -H "Authorization: Bearer $TRAFFIC_TOKEN" \
  -H "x-wia-source: aq:WIA-AQ-KR-Seoul-001" \
  -d '{"window":"2026-04-25T16:00:00+09:00/2026-04-25T22:00:00+09:00",
       "reason":"PM10 > 200 μg/m³ sustained","zones":["Jung","Jongno"]}'
```

The action is journalled; a matching entry lands in the data lake `actions` table alongside the readings that triggered it, enabling retrospective efficacy analysis.

### 13.3 Publishing to WMO WIS 2.0

```bash
python cli/wis2-publish.py \
  --station WIA-AQ-KR-Seoul-001 \
  --from 2026-04-25T00:00:00Z --to 2026-04-25T23:59:59Z \
  --centre 220 --bulletin IUAQ01 \
  --broker wis2-gb.kma.go.kr \
  --jws-key ~/.wia/stations/WIA-AQ-KR-Seoul-001.jwk
```

Produces an hourly BUFR bundle, MQTT-publishes the WIS 2.0 envelope, and writes the manifest to the data lake.

---

## 14. Privacy, consent, and ethical use

- Patient linkage requires explicit, revocable consent under HIPAA/PIPA; the FHIR gateway MUST verify an active `Consent` resource with `scope=patient-privacy` before forwarding station data.
- Fine-grained traffic behavioural inferences require ethical review; WIA Standards recommends an IRB/ethics sign-off for any deployment that cross-joins AQI to individual mobility.
- Data published under open licence (CC-BY-4.0 by default) MUST carry attribution back to the WIA station operator; commercial resale requires a separate WIA Commercial Data Licence.

---

## 15. Compliance checklist

**Phase 4 conformance requires:**

- [ ] SensorThings 1.1 bridge exposed or deterministically mappable (§ 3)
- [ ] BUFR/WIS 2.0 publisher operational for at least one WMO centre (§ 4)
- [ ] FHIR `Observation` emission with correct LOINC codes (§ 5)
- [ ] CAP 1.2 alerts published on the thresholds in § 6
- [ ] At least one smart-city actuation channel with rate cap (§ 7)
- [ ] Parquet/Iceberg data-lake schema matches § 8
- [ ] Forecast integration flagged in UI (§ 9)
- [ ] Citizen-sensor bias correction applied (§ 10)
- [ ] Regulatory export pipeline reproducible & signed (§ 11)
- [ ] Grafana / Prometheus / OpenTelemetry observability wired (§ 12)
- [ ] Consent mechanism enforced for patient linkage (§ 14)

---

## 16. References

1. OGC 15-078r6 — SensorThings API v1.1 (Part 1: Sensing)
2. OGC 19-086r6 — Environmental Data Retrieval API v1.1
3. WMO-306 Manual on Codes Vol. I.3 — BUFR/CREX
4. WMO WIS 2.0 Technical Regulations
5. OASIS CAP 1.2 — Common Alerting Protocol
6. HL7 FHIR R5 Observation / Subscription / Consent
7. WHO Global Air Quality Guidelines (2021)
8. US EPA NAAQS Primary Standards — 40 CFR 50
9. EU Ambient Air Quality Directive 2008/50/EC, as amended by 2024/2881
10. Barkjohn et al. (2021) — "Development and Application of a U.S.-Wide Correction for PM2.5 Data Collected with the PurpleAir Sensor", *Atmospheric Measurement Techniques* 14, 4617
11. Apache Iceberg 1.5, Apache Parquet 2.9, Delta Lake 3.1
12. RFC 3161 — Time-Stamp Protocol (TSP)

---

**Document Status:** ACTIVE
**Effective Date:** April 25, 2026
**Review Date:** April 25, 2028

© 2026 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
