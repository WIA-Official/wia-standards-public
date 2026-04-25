# WIA Cryo-Monitoring Communication Protocol Specification

**Phase 3: Communication Protocol**

**Version**: 1.0.0
**Status**: Draft
**Date**: 2026-04-26
**Standard ID**: WIA-CRYO-008

---

## 1. Overview

### 1.1 Purpose

WIA Cryo-Monitoring Phase 3는 Phase 1 데이터 포맷을 운반하는 실시간 통신 프로토콜의 표준 계약을 정의합니다. 다채널 센서 메쉬, 알람 페이저, 시설 게이트웨이가 이 프로토콜을 통해 일관되게 통신합니다.

This document specifies the wire-level protocol bindings (MQTT 5.0, OPC UA, WebSocket) over which WIA Cryo-Monitoring Phase 1 payloads are transported in real time, together with the security baseline.

### 1.2 Design Goals

1. **Multi-binding** — implementations MAY pick one or more bindings (MQTT 5.0, OPC UA, WebSocket) per facility class.
2. **Failover-Aware** — alarms MUST traverse a redundant path within the deadlines defined in §4.
3. **Functional-Safety Friendly** — payload structure aligned with IEC 61508 partitioning conventions.
4. **Security First** — TLS 1.3 baseline, mutual auth via certificate or PSK.
5. **Deterministic Delivery** — bounded latency targets per topic class.

### 1.3 Scope

| In scope | Out of scope |
|----------|--------------|
| Topic / address space hierarchy | LIMS workflow integration (PHASE 4) |
| Failover and redundancy rules | Vendor PLC fieldbus mapping |
| Alarm propagation deadlines | Refrigerant valve actuation logic |
| Security baseline | Backend storage architecture |

---

## 2. Protocol Bindings

### 2.1 MQTT 5.0 (ISO/IEC 20922)

The default binding for sensor mesh telemetry. Topic hierarchy:

```
wia/cryo/{facility_id}/{room_id}/sensors/{sensor_id}/{kind}
wia/cryo/{facility_id}/{room_id}/alerts/{severity}
wia/cryo/{facility_id}/{room_id}/levels/{tank_id}
wia/cryo/{facility_id}/control/heartbeat
```

`{kind}` ∈ `temperature`, `pressure`, `level`, `selftest`. QoS:

| Topic class | QoS | Retained |
|-------------|-----|----------|
| `sensors/.../temperature` | 0 (at most once, high frequency) | no |
| `sensors/.../selftest` | 1 (at least once) | yes |
| `alerts/{severity}` | 2 (exactly once) | no |
| `levels/{tank_id}` | 1 | yes |
| `control/heartbeat` | 0 | no |

### 2.2 OPC UA (IEC 62541)

The default binding for facility-grade integration with PLCs and SCADA. Object model uses the WIA-CRYO information model (URI `https://wiastandards.com/cryo/ua`) with the following node classes mapped onto the PHASE 1 payloads:

- `WiaCryoSensorType` — extends `BaseObjectType`, holds class metadata (RTD/thermocouple/level), calibration epoch, value variable.
- `WiaCryoAlertType` — extends `AlarmConditionType`, mapped to `severity`, `acknowledged`, `confirmed`.
- `WiaCryoLevelType` — `BaseObjectType` with sampled level history.

### 2.3 WebSocket (IETF RFC 6455 / RFC 8441)

Optional binding for browser-based dashboards. Frames carry the same JSON payloads as PHASE 1 §3 with framing:

```
{"type":"reading","payload":{...}}
{"type":"alert","payload":{...}}
{"type":"level","payload":{...}}
{"type":"heartbeat","payload":{"ts":"..."}}
```

Heartbeat MUST be sent every 30 s; absence for 90 s causes the dashboard to mark the link as `degraded`.

---

## 3. Security Baseline

### 3.1 Transport

- TLS 1.3 (IETF RFC 8446) is the minimum.
- Cipher suites: `TLS_AES_256_GCM_SHA384`, `TLS_CHACHA20_POLY1305_SHA256`, `TLS_AES_128_GCM_SHA256`.
- Certificates: X.509 v3 (IETF RFC 5280); facility CAs MAY be private.

### 3.2 Mutual Authentication

- MQTT 5.0: client certificates with `clientId` matching the certificate `commonName`; PSK is acceptable for embedded sensor brokers.
- OPC UA: User Token Policy `Certificate` or `IssuedToken`.
- WebSocket: bearer JWT (IETF RFC 7519) with the same scopes defined in PHASE 2 §3.1, optionally combined with client certificate pinning.

### 3.3 Replay Protection

All event payloads include a monotonic `seq` and `observed_at`. Brokers MUST reject events with `seq` decreasing within a 60 s window or `observed_at` skewed > 30 s from broker time.

---

## 4. Latency and Failover Deadlines

| Event class | Hard deadline (publish→subscriber) | Heartbeat interval |
|-------------|-------------------------------------|--------------------|
| Critical alert (severity=critical) | 1 s | n/a |
| Warning alert (severity=warning)   | 5 s | n/a |
| Reading (default)                  | 10 s | n/a |
| Heartbeat                          | n/a  | 30 s |
| Selftest                           | 60 s | n/a |

Implementations MUST provide at least one redundant network path for `severity=critical` alerts. When a primary path's heartbeat is missed for 3 consecutive intervals, alerts MUST be re-published over the secondary path within the deadline above.

---

## 5. Failover and Redundancy

A compliant Phase 3 deployment MUST implement at least one of the following redundancy patterns:

- **Dual broker active-active** — two MQTT brokers with replicated session state; clients reconnect to the other on failure.
- **Active-standby** — a single primary broker with a standby; sensors connect to a virtual IP.
- **OPC UA aggregating server** — front a primary OPC UA server with an aggregating server that fails over to a hot replica.

Failover events themselves are recorded as `wia/cryo/{facility_id}/control/failover` MQTT messages with QoS 1 retained, carrying the time of failover and the reason code.

---

## 6. Versioning, Discovery, and Capabilities

Brokers and aggregating servers MUST publish a `wia/cryo/{facility_id}/control/capabilities` retained message with:

```json
{
  "wia_cryo_version": "1.0.0",
  "phase3_bindings": ["mqtt", "opcua"],
  "tls_version_min": "1.3",
  "selftest_interval_s": 60
}
```

Clients negotiate against this capability advertisement; mismatched major versions MUST cause the client to refuse to publish.

---

## 7. Error Handling

| Error class | Wire signal |
|-------------|-------------|
| Authentication failure | MQTT CONNACK `0x86`/`0x87`; OPC UA `Bad_UserAccessDenied` |
| Schema validation failure | OPC UA `Bad_DataEncodingInvalid`; WebSocket framed `{"type":"error", ...}` per IETF RFC 9457 |
| Quota exceeded | MQTT DISCONNECT `0x97`; OPC UA `Bad_TooManyOperations` |
| Replay rejected | MQTT DISCONNECT `0x99`; OPC UA `Bad_RequestNotAllowed` |

WebSocket bindings carry RFC 9457 problem details for any error inside the framing envelope.

---

## 8. Compliance and Certification

| Tier | Required Phase 3 surface |
|------|---------------------------|
| Tier 1 — Self-declared | At least one binding (§2) + TLS 1.3 + heartbeat + capability advertisement |
| Tier 2 — Assessed | Tier 1 + at least one redundancy pattern (§5) + signed assessor report |
| Tier 3 — Accredited | Tier 2 + WIA accreditation + ISO/IEC 17065:2012 + IEC 61508 SIL self-declaration |

---

## 9. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2026-04-26 | Initial PHASE 3 specification (this document) |

---

## 10. References

- WIA Cryo-Monitoring PHASE 1 Data Format Specification (this repository)
- WIA Cryo-Monitoring PHASE 2 SDK and Client API Specification (this repository)
- WIA Cryo-Monitoring PHASE 4 Ecosystem Integration (this repository)
- IEC 62541 (OPC UA) — Unified architecture
- ISO/IEC 20922 (MQTT 5.0) — Message Queuing Telemetry Transport
- IEC 61508 — Functional safety of electrical/electronic/programmable electronic safety-related systems
- IEC 60751 — Industrial platinum resistance thermometers
- IEC 60584-1 — Thermocouples — EMF specifications and tolerances
- ISO/IEC 17065:2012 — Conformity assessment
- IETF RFC 5280 — Internet X.509 Public Key Infrastructure Certificate and CRL Profile
- IETF RFC 6455 — The WebSocket Protocol
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 8441 — Bootstrapping WebSockets with HTTP/2
- IETF RFC 8446 — The Transport Layer Security (TLS) Protocol Version 1.3
- IETF RFC 9457 — Problem Details for HTTP APIs

---

## Annex A — Conformance Tier Matrix

WIA conformance for the WIA Cryo-Monitoring communication protocol is evaluated across three tiers as summarized in §8. Implementations MUST publish their declared tier in the capability message described in §6 via the optional `tier` field, and MUST report tier downgrade events to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references the standards listed in §10. Implementers SHOULD review the listed standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation. This cross-walk is informative only.

---

## Annex C — Reference Implementations and Test Vectors

The WIA-Official GitHub organization publishes:

- `wia-standards/standards/cryo-monitoring/api/` — broker integration helpers
- `wia-standards/standards/cryo-monitoring/cli/` — POSIX shell client demonstrating the wire contract
- `wia-standards/standards/cryo-monitoring/simulator/` — interactive simulator that emits the §2 topic hierarchy with realistic timing

A normative set of MQTT-trace and OPC UA capture pairs covering the topic classes defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector.

---

## Annex D — Open Questions and Future Work

1. **MQTT 5.0 shared subscriptions** — operational guidance for load-balancing across redundant subscribers.
2. **Time synchronization** — explicit profile (PTP / NTPv4) for sub-second deadlines defined in §4.
3. **OPC UA pub-sub binding** — when to prefer pub-sub over the classic client-server profile.
4. **Edge buffering policy** — guidance on retention windows during link partitions.
5. **Sustainability disclosure** — broker-level energy reporting fields aligned with PHASE 4.

Items in this annex are non-normative.

---

**弘益人間 · Benefit All Humanity**

© 2026 WIA — World Certification Industry Association
Licensed under MIT License


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

- **Sensor populations** — Mixed RTD (IEC 60751 Class A) and Type-T
  thermocouple (IEC 60584-1) installations are common in walk-in cryo
  rooms; implementers SHOULD record the sensor class on every sensor
  descriptor so that downstream calibration tooling can compute the
  correct uncertainty envelope per the relevant tolerance class.
- **LN2 dewar telemetry** — Capacitive level probes typically sample at
  0.1 Hz; ultrasonic probes at 1 Hz. The wire formats accommodate both
  without changing the schema.
- **Operator workstations** — Operators typically supervise 4–16
  rooms simultaneously; UI implementations SHOULD subscribe to the
  facility-level alert topic class only and lazy-fetch reading streams
  on demand. Eager-fetch designs raise broker load disproportionately
  to the operator's attention budget.
- **Audit retention** — A 7-year retention window is sufficient to
  satisfy ISO 20387:2018 audit expectations in most jurisdictions; some
  regulators require longer retention for human-derived materials, in
  which case the deployment policy MUST extend the retention window
  rather than the standard's defaults.
- **Time synchronization** — Sub-second alert deadlines depend on
  synchronized clocks. NTPv4 with stratum-2 servers is sufficient for
  the deadlines defined in PHASE 3 §4; PTP is recommended for sites
  that require deterministic interlocks.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA Cryo-Monitoring conformance.


