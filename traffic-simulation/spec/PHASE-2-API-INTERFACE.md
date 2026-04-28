# WIA-CITY-017 (traffic-simulation) — Phase 2: API Interface Specification

> **Version:** 1.0.0
> **Status:** Draft
> **Phase:** 2 of 4 (API Interface)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 2 specifies the API surface a WIA-CITY-017 traffic-simulation deployment exposes to traffic-management centres (TMC), urban-mobility planners, transport-research labs, and downstream consumers (V2X infrastructure, mobility-as-a-service operators, regulators). The surface is rooted in the canonical conventions of the traffic-engineering domain: SUMO and PTV VISSIM as canonical microsimulation engines, SAE J2735 for V2X message vocabulary, ISO 17572 for location referencing, and ETSI ITS-G5 for the European V2X profile.

### 1.1 Authorization model

Authorization on a traffic-simulation deployment composes:

| Concern | Mechanism | Reference |
|---------|-----------|-----------|
| Operator console identity | X.509 client certificate per ISO/IEC 27001 §A.5.16 | ISO/IEC 27001:2022 |
| TMC-side authorisation | Role-based access aligned to NTCIP 2202 system-architecture roles | NTCIP 2202 v01 |
| Per-scenario entitlement | Documented scenario-publish governance (TMC operator vs research lab vs simulation vendor) | TMDD v3.x |
| Records retention | NIST SP 800-53 Rev 5 AU controls + jurisdiction transport-records law | NIST SP 800-53 Rev 5 |
| Privacy floor (origin-destination data) | DPIA per ISO/IEC 29134:2023 + transport-data anonymisation policy | ISO/IEC 29134:2023 |

API requests originate from authenticated TMC consoles or research-lab clients. The console signs every API request with an X.509 certificate; the certificate chain anchors at the TMC's PKI per the operator's ISO/IEC 27001 access-control register.

### 1.2 Privacy floor for origin-destination matrices

Origin-destination (OD) matrices are the highest-privacy-impact data class in the standard. Per-individual OD trajectories enable re-identification with surprisingly little auxiliary information; aggregate OD matrices at zone-level (typically census tract or transport-analysis-zone) are typically privacy-neutral. Endpoints that return OD data MUST honour the deployment's published DPIA and the applicable jurisdiction privacy law (GDPR Article 32 in the EU, KR PIPA Article 29 in Korea, FAST Act §6308 in the US). Endpoints refuse OD queries finer than the documented zone resolution unless the requester carries a documented research-purpose authorisation.

---

## 2. HTTP/REST Surface

### 2.1 Base URL and discovery

```
https://<host>/wia-city-017/v1
```

Discovery document at `/.well-known/wia-city-017`:

```json
{
  "wia_city_017_version": "1.0.0",
  "deployment_id": "<UUID v4 per RFC 9562>",
  "operator": "<TMC-name>",
  "supported_engines": ["SUMO 1.x", "PTV VISSIM 2024", "Aimsun Next", "MATSim"],
  "supported_profiles": ["SAE J2735 2024", "ETSI ITS-G5 v2"],
  "iso_17572_lrs_supported": true,
  "od_zone_resolution": "transport_analysis_zone",
  "endpoints": { "scenario": "...", "run": "...", "trajectory": "...", "od_matrix": "...", "v2x": "..." }
}
```

### 2.2 Scenario endpoint

```
GET    /scenario                 → list scenarios
POST   /scenario                  → upload a scenario (network + demand + control)
GET    /scenario/{id}             → scenario descriptor (Phase 1 §2.2)
GET    /scenario/{id}/network     → network in standard format (SUMO net.xml, OpenDRIVE)
GET    /scenario/{id}/demand      → demand matrix (origin-destination at zone resolution)
PUT    /scenario/{id}/calibration → upload calibration evidence (counts, speed observations)
```

Scenarios encode the road network, the time-of-day demand pattern, the signal-control plan, and the calibration evidence used to validate the model against real-world observations. The network is delivered in either SUMO `net.xml` format or OpenDRIVE 1.7 format depending on the consumer's tooling.

### 2.3 Run endpoint

```
POST   /run                       → launch a simulation run
GET    /run/{id}                  → run state
GET    /run/{id}/feed              → SSE stream of per-step state
GET    /run/{id}/result            → aggregate result (KPIs, OD-matched trips)
DELETE /run/{id}                  → archive (retain results)
```

Runs execute scenarios with operator-specified parameters: random seed, simulation horizon, controller policy override, weather/incident overlay. The SSE stream emits per-step state at the operator's documented sampling rate (typically 1 Hz or 10 Hz). Runs are throttled per console to one concurrent execution by default; large-fleet operators may negotiate higher concurrency.

### 2.4 Trajectory endpoint

```
GET    /run/{id}/trajectory          → per-vehicle trajectory data
GET    /run/{id}/trajectory/zone/{z} → zone-aggregate trajectories (privacy-safe)
```

Per-vehicle trajectories are the highest-resolution output and carry the highest privacy risk. The endpoint returns full trajectories only to authenticated requesters with documented research-purpose authorisation; aggregate trajectories at zone level are the default response for general consumers.

### 2.5 OD-matrix endpoint

```
GET    /od-matrix/{date}/{period}            → OD matrix at zone resolution
POST   /od-matrix/{date}/{period}/disagg     → research-purpose disaggregation request (DPIA-gated)
```

The OD-matrix endpoint is the load-bearing surface for transport-planning consumers. The default zone resolution is the city's published transport-analysis-zone (TAZ) layer; finer disaggregation requires research-purpose authorisation.

### 2.6 V2X-bridge endpoint

```
GET    /v2x/messages              → SAE J2735 message stream from the simulated environment
POST   /v2x/inject                → inject a SAE J2735 message into a running simulation
```

The V2X bridge lets researchers and infrastructure operators evaluate V2X strategies (SPaT, MAP, BSM, RSA, TIM messages) in simulation before deployment. Messages follow SAE J2735 2024 with the regional adaptation (J2735 in North America, ETSI ITS-G5 with its CAM/DENM/SPATEM/MAPEM equivalents in Europe).

---

## 3. Idempotency and retry semantics

Every write endpoint accepts the `Idempotency-Key` header per IETF draft `draft-ietf-httpapi-idempotency-key-header`. Hosts retain a 24-hour replay cache per console identity. Long-running simulation runs (overnight or multi-day) rely on the discipline so reconnecting consoles can safely retry the launch operation.

---

## 4. Pagination, filtering, and bulk export

Collection endpoints support cursor pagination per IETF `draft-ietf-httpapi-link-relations`. Bulk export at `POST /exports` accepts a time window plus entity filter and returns a signed manifest with a Merkle root over the included envelopes. The manifest is signed by the operator; transport-research labs verify the chain before admitting the export as evidence in any peer-reviewed publication.

---

## 5. Health and observability

```
GET /health   → liveness
GET /ready    → readiness (includes simulation-engine connectivity check)
GET /metrics  → Prometheus exposition
```

The `/metrics` endpoint exposes: scenarios-published per month, runs-completed per day, p50/p95/p99 run wall-clock duration, V2X-messages-emitted per simulation second. Telemetry MUST NOT include high-cardinality labels (per-vehicle identifiers, per-step counters).

---

## 6. Error model

Errors return RFC 9457 problem documents. Reserved problem types relevant to Phase 2:

| Type | Status | Meaning |
|------|--------|---------|
| `…/network-malformed` | 422 | The submitted network does not parse as valid SUMO `net.xml` or OpenDRIVE. |
| `…/demand-zone-mismatch` | 422 | The demand matrix references zones not present in the city's TAZ layer. |
| `…/dpia-violation` | 403 | The requested operation falls outside the deployment's published DPIA purpose limitations. |
| `…/od-resolution-too-fine` | 403 | The requested OD resolution is finer than the operator's published default; research-purpose authorisation required. |
| `…/calibration-divergence` | 422 | The calibration evidence diverges from the simulation results beyond the operator's threshold. |
| `…/run-budget-exceeded` | 429 | The console has exceeded its concurrent-run budget. |

---

## 7. Conformance test suite

A black-box conformance test suite is published at `https://github.com/WIA-Official/wia-traffic-simulation-conformance` and walks through every Phase 2 endpoint, the SUMO `net.xml` round-trip, the OpenDRIVE 1.7 round-trip, the SAE J2735 message-stream parse, the OD-matrix DPIA enforcement, and the bulk-export Merkle root check.

---

## 8. References

- SUMO (Simulation of Urban Mobility) v1.x — Eclipse SUMO project documentation
- PTV VISSIM 2024 — PTV Group documentation
- Aimsun Next — Aimsun Group documentation
- MATSim — Multi-Agent Transport Simulation framework
- OpenDRIVE 1.7 — ASAM standard for road-network description
- SAE J2735 (2024) — DSRC Message Set Dictionary
- SAE J2945 series — DSRC system requirements
- ETSI EN 302 663 — ITS-G5 access layer
- ETSI EN 302 665 — ITS communications architecture
- ETSI EN 302 637-2 — Cooperative Awareness Messages (CAM)
- ETSI EN 302 637-3 — Decentralized Environmental Notification Messages (DENM)
- ETSI TS 103 301 — SPATEM / MAPEM
- ISO 17572-1/-2/-3 — Intelligent transport systems — Location referencing
- ISO 14813-1 — ITS service architecture
- NTCIP 1202 — Actuated traffic signal controller
- NTCIP 2202 — Internet protocol for ITS centre-to-centre
- TMDD v3.x — Traffic Management Data Dictionary
- ISO/IEC 27001:2022 — Information security management
- ISO/IEC 29134:2023 — Privacy impact assessment
- NIST SP 800-53 Rev 5 — Security and Privacy Controls
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 9562 — UUIDs
- IETF RFC 8446 — TLS 1.3
- BIPM SI Brochure — Time scales (TAI / UTC / leap seconds)

---

## 9. Implementer note — privacy floor for OD data

OD matrices have been a recurring source of re-identification incidents in transport research. The standard's privacy floor (Phase 2 §1.2) is intentionally conservative: zone-resolution aggregates are the default, and finer disaggregation requires a documented research-purpose authorisation chain. Operators who weaken the default in jurisdictions where the trade-off favours research access MUST log the weakening in the audit chain so a privacy regulator can reconstruct who saw what data and under what authorisation.

弘益人間 — Benefit All Humanity.


## 10. Closing implementer note for Phase 2

Phase 2 endpoints are the day-to-day surface a transport-research analyst or TMC operator interacts with. The discipline is high — every scenario signed, every run audited, every OD-matrix query gated on the published DPIA — because traffic-simulation results have a public-credibility-cost that retracted papers cannot recover. Standardising the wire format up-front protects honest researchers from being entangled with bad-actor claims.

A first deployment that follows the runbook reaches operational stability in about 90 days. Lighter deployments compress this to 30 days; metropolitan-scale deployments scale to 6-12 months for full calibration coverage. The depth of network-import, demand-calibration, and signal-control replication concentrated in those windows is what justifies the wire-format discipline.


## 12. Closing implementer note (Phase 2 → Phase 3 handoff)

Phase 2 endpoints for traffic-simulation are the day-to-day surface that researchers and TMC operators interact with; Phase 3 protocols govern the wire-level exchanges that flow into and out of those endpoints. The handoff is direct: every Phase 2 envelope produced or consumed at an endpoint is also a Phase 3 protocol envelope, signed and audited under the same discipline. This Phase serialises the operational request-response patterns; Phase 3 below adds the federation, calibration-evidence, and incident-injection protocol that makes the requests trustworthy across organisational boundaries.
