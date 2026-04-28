# WIA-CITY-009 (smart-lighting) — Phase 2: API Interface Specification

> **Version:** 1.0.0
> **Status:** Draft
> **Phase:** 2 of 4 (API Interface)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 2 specifies the API surface a smart-lighting deployment exposes to the city operations centre, the lighting maintenance contractor, and downstream consumers (utility for energy reporting, city department for maintenance scheduling, public for outage reporting). The surface is rooted in the canonical conventions of the lighting-control domain: DALI-2 / D4i for the in-fixture digital protocol, Zhaga Book 18 for outdoor luminaire connectors, IEC 62386 series for the digital-control communications, and IES TM-30 for colour-fidelity reporting.

### 1.1 Authorization model

Authorization on a smart-lighting deployment composes:

| Concern | Mechanism | Reference |
|---------|-----------|-----------|
| Operator console identity | X.509 client certificate per ISO/IEC 27001 §A.5.16 | ISO/IEC 27001:2022 |
| Luminaire-side authorisation | DALI-2 application-controller authentication + IEC 62443-3-3 SR 1.1 | DALI-2 v1.x; IEC 62443-3-3:2013 |
| In-fixture sensor data flow | D4i (DALI-2 part 251 + 252 + 253) | IEC 62386-150/-151/-251/-252/-253 |
| Energy-metering integrity | IEC 62386-252 metadata + ANSI C12 metering data | IEC 62386-252; ANSI C12.19 |
| Per-fixture entitlement | Role-based access against the city's asset register | ISO 55001:2014 asset management |
| Privacy floor (people-detection sensors) | DPIA per ISO/IEC 29134:2023 + jurisdiction privacy law | ISO/IEC 29134:2023 |

API requests originate from authenticated operator consoles or maintenance-contractor field devices. The console signs every API request with an X.509 certificate; the certificate chain anchors at the city's asset-management PKI per ISO 55001:2014 conventions.

### 1.2 Privacy floor (occupancy and people-counting sensors)

Modern street-lighting fixtures often carry occupancy sensors, people-counting sensors, or environmental-air-quality sensors as part of D4i-compatible payloads. Endpoints that expose people-counting data MUST honour the deployment's published Data Protection Impact Assessment (DPIA, ISO/IEC 29134:2023) and the applicable jurisdiction privacy law. Aggregate counts at zone level are typically privacy-neutral; per-fixture per-second counts may not be, depending on density and re-identification risk.

---

## 2. HTTP/REST Surface

### 2.1 Base URL and discovery

```
https://<host>/wia-city-009/v1
```

Discovery document at `/.well-known/wia-city-009`:

```json
{
  "wia_city_009_version": "1.0.0",
  "deployment_id": "<UUID v4 per RFC 9562>",
  "operator": "<city-or-contractor-name>",
  "supported_protocols": ["DALI-2", "D4i", "IEC 62386"],
  "supported_connectors": ["Zhaga Book 18 socket", "Zhaga Book 11 control"],
  "asset_register_uri": "<URI>",
  "endpoints": { "luminaire": "...", "zone": "...", "schedule": "...", "fault": "..." }
}
```

### 2.2 Luminaire endpoint

```
GET    /luminaire                  → list luminaires (paginated)
GET    /luminaire/{id}             → luminaire descriptor (Phase 1 §2.2)
PUT    /luminaire/{id}/level       → set dim level (0-254 per DALI)
GET    /luminaire/{id}/telemetry   → live telemetry feed
GET    /luminaire/{id}/event-feed  → SSE stream of fault / occupancy events
```

The `level` PUT translates to a DALI-2 DAPC (Direct Arc Power Control) command on the fixture's digital bus. Telemetry includes power consumption (watts), driver temperature, dimmer position, lumen output (where the driver supports lumen feedback per IEC 62386-252), and the ambient sensor readings (when D4i sensor packets are configured).

### 2.3 Zone endpoint

```
GET    /zone                       → list zones
GET    /zone/{id}                  → zone descriptor
GET    /zone/{id}/aggregate        → zone-level aggregate (energy, occupancy, fault counts)
PUT    /zone/{id}/preset           → apply a lighting preset
```

Zones group luminaires by physical or logical boundary (e.g., a single street, a parking lot, a park area). Zone-level aggregates are the privacy-safe surface for downstream consumers; per-fixture data is gated on operator-or-contractor identity.

### 2.4 Schedule endpoint

```
GET    /schedule                   → list schedules
POST   /schedule                   → create a schedule (sunset/sunrise + dim profile)
GET    /schedule/{id}              → schedule detail
PUT    /schedule/{id}              → update
DELETE /schedule/{id}              → archive (retain history)
```

Schedules encode time-of-day or astronomic-event-driven dim profiles. Astronomic events (sunset, sunrise, civil/nautical/astronomical twilight) are computed from the deployment's geographic coordinates per the standard astronomic algorithms. Schedules cascade from city policy down to zone overrides down to per-fixture exceptions.

### 2.5 Fault endpoint

```
GET    /fault                      → list active faults
GET    /fault/{id}                 → fault detail
POST   /fault/{id}/dispatch        → assign to a maintenance contractor
PUT    /fault/{id}/resolve         → resolve with after-action notes
```

Faults flow from luminaire telemetry: driver overheat, lamp failure (open or short), photocell drift, schedule mismatch, communication loss. Each fault carries an ISO 55001 asset-management linkage so the city can track per-asset failure rates and warranty claims.

### 2.6 Energy-reporting endpoint

```
GET    /energy/report              → aggregate energy report for a billing period
GET    /energy/report/{id}/csv     → ANSI C12.19-compatible CSV for utility billing
```

Energy reports aggregate per-fixture and per-zone consumption over the operator's billing period (typically monthly). The CSV export follows ANSI C12.19 conventions so the utility's billing system can ingest it without custom translation. The standard does not invent new metering primitives; it provides the wire format to relay them.

---

## 3. Idempotency and retry semantics

Every write endpoint accepts the `Idempotency-Key` header per IETF draft `draft-ietf-httpapi-idempotency-key-header`. Hosts retain a 24-hour replay cache per console identity. Field-device retries on flaky cellular networks rely on this discipline so a maintenance crew that lost connectivity mid-operation can safely retry.

---

## 4. Pagination, filtering, and bulk export

Collection endpoints support cursor pagination (`?cursor=…&limit=…`) per IETF `draft-ietf-httpapi-link-relations`. Bulk export at `POST /exports` accepts a time window plus entity filter and returns a signed manifest with a Merkle root over the included envelopes. The manifest is signed by the operator; auditors verify the chain before admitting the export as evidence in any regulatory or warranty proceeding.

---

## 5. Health and observability

```
GET /health   → liveness
GET /ready    → readiness (includes back-end DALI gateway connectivity check)
GET /metrics  → Prometheus exposition (operator-actionable counters)
```

The `/metrics` endpoint exposes: luminaires-online ratio per zone, fault-creation rate per zone, energy-consumption distribution, schedule-adherence metric (% of fixtures matching schedule). Telemetry MUST NOT include high-cardinality labels (per-fixture identifiers, per-second values).

---

## 6. Error model

Errors return RFC 9457 problem documents. Reserved problem types relevant to Phase 2:

| Type | Status | Meaning |
|------|--------|---------|
| `…/dali-bus-fault` | 503 | The DALI bus segment for the addressed luminaire is offline. |
| `…/luminaire-not-commissioned` | 412 | The luminaire is in the asset register but not yet commissioned for control. |
| `…/dpia-violation` | 403 | The requested operation falls outside the deployment's published DPIA purpose limitations. |
| `…/d4i-sensor-misconfigured` | 422 | D4i sensor packet configuration does not match the firmware capability declared at commissioning. |
| `…/schedule-conflict` | 409 | The submitted schedule conflicts with a higher-priority city policy. |
| `…/energy-meter-reset-detected` | 422 | The luminaire's energy counter has reset; reading is bracketed by the reset event. |

---

## 7. Conformance test suite

A black-box conformance test suite is published at `https://github.com/WIA-Official/wia-smart-lighting-conformance` and walks through every Phase 2 endpoint, the DALI-2 DAPC translation, the D4i sensor-packet decode, the ANSI C12.19 CSV export, the schedule-cascade evaluation, and the bulk-export Merkle root check.

---

## 8. References

- IEC 62386-150:2018 — Digital addressable lighting interface — Part 150
- IEC 62386-151:2018 — Part 151 (control device general requirements)
- IEC 62386-251:2018 — Part 251 (memory bank for energy reporting)
- IEC 62386-252:2018 — Part 252 (memory bank for diagnostics and maintenance)
- IEC 62386-253:2020 — Part 253 (memory bank for luminaire information)
- IEC 60529:1989 — Degrees of protection (IP code) for outdoor luminaires
- DALI-2 v1.x specifications — Digital Illumination Interface Alliance
- D4i Programme — DALI-2 + IoT extension package
- Zhaga Consortium Book 18 — outdoor luminaire socket
- Zhaga Consortium Book 11 — luminaire-internal control
- ANSI C12.19:2012 — utility-industry end-device data tables
- IES TM-30-20 — Method for evaluating light source colour rendition
- IEEE 802.15.4 — Low-rate wireless networks (used by some D4i payloads)
- ISO/IEC 27001:2022 — Information security management
- ISO/IEC 29134:2023 — Privacy impact assessment
- ISO 55001:2014 — Asset management
- IEC 62443-3-3:2013 — System security requirements
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 9562 — UUIDs
- IETF RFC 8446 — TLS 1.3
- BIPM SI Brochure — Time scales (TAI / UTC / leap seconds)

---

## 9. Implementer note — operational lifecycle

A street-lighting installation has a 15-25 year asset lifecycle. The wire formats and protocol disciplines that operate it must absorb that horizon without locking the city into a single luminaire vendor. The DALI-2 + D4i + Zhaga combination is the open-standard backbone; this Phase's API surface is a thin operational layer above that backbone that lets the city manage its lighting estate without per-vendor custom integration.

弘益人間 — Benefit All Humanity.


## 10. Closing implementer note for Phase 2

Phase 2 endpoints are the day-to-day surface a city operations console interacts with. The discipline is high — every dim command audited, every fault tracked through to contractor work order, every energy report reconciled against utility metering — because the alternative is a multi-billion-fixture city lighting estate operating on tribal knowledge that does not survive a contractor change. The wire-format discipline insists on auditability as a precondition.

A first deployment that follows the runbook reaches operational stability in about 90 days. Lighter deployments compress this to 30 days; metropolitan deployments scale to 12-24 months for full asset coverage. The depth of asset-register reconciliation, contractor onboarding, and utility-billing alignment concentrated in those windows is what justifies the wire-format discipline.
