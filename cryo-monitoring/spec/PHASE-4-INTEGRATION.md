# WIA Cryo-Monitoring Ecosystem Integration Specification

**Phase 4: Ecosystem Integration**

**Version**: 1.0.0
**Status**: Draft
**Date**: 2026-04-26
**Standard ID**: WIA-CRYO-008

---

## 1. Overview

### 1.1 Purpose

WIA Cryo-Monitoring Phase 4는 PHASE 1–3 데이터 표준을 운영 워크플로 (LIMS/biobank/시설관리/자동알림)와 연결하는 통합 표준을 정의합니다. 시설 게이트웨이가 다중 사이트 컨트롤 플레인과 통신하는 패턴, 외부 보고서 산출 패턴, 지속가능성 공시 필드를 포함합니다.

This document specifies how a WIA Cryo-Monitoring deployment integrates with the surrounding biobanking, laboratory information management, paging, and reporting ecosystem.

### 1.2 Design Goals

1. **LIMS Friendly** — sample identifiers exposed in cryo events MUST resolve in upstream LIMS without re-keying.
2. **Audit Continuous** — provenance per W3C PROV-DM is preserved across ecosystem boundaries.
3. **Federated Without Lock-in** — every integration is a pull / push contract over open formats; no mandatory vendor product.
4. **Sustainability Aware** — energy and refrigerant disclosure fields are first-class.
5. **Operationally Conservative** — ecosystem failures MUST NOT mask alerts.

### 1.3 Scope

| In scope | Out of scope |
|----------|--------------|
| LIMS / biobank handshake | Sample destruction workflows |
| Paging / on-call adapters | Vendor-specific pager hardware |
| Sustainability disclosure fields | Carbon-credit certification |
| Multi-site federation patterns | Cross-border legal export controls |
| Reporting profile (CSV, PDF) | Layout templating engines |

---

## 2. LIMS / Biobank Handshake

### 2.1 Sample Identifier Mapping

Every PHASE 1 reading or alert that pertains to a stored sample MUST carry a `sample_ref` field with one of the following identifier classes:

- `lims:{lims_namespace}:{sample_id}`
- `biobank:{biobank_namespace}:{accession}`
- `wia:cryo:{facility_id}:{tank_id}:{slot_id}`

If a backend cannot resolve a `sample_ref`, it MUST surface the unresolved identifier as part of the response without rewriting it.

### 2.2 Synchronization

The recommended synchronization pattern is server-driven pull from the LIMS, using:

- HTTPS + OAuth 2.0 (IETF RFC 6749) bearer tokens
- ISO 8601 timestamps for incremental sync
- ETag-based conditional GET (IETF RFC 9110)

When LIMS data is unavailable, cryo backends MUST continue to operate and queue events for later reconciliation; queued events expire after 7 days unless the deployment policy extends the window.

### 2.3 Compatibility With ISO 20387:2018

Integrations operating in biobanking environments SHOULD expose the following derived metadata for ISO 20387:2018 audits:

- Sample-level cumulative time outside the temperature envelope.
- Sample-level cumulative time below an LN2 floor threshold.
- Calibration record link for each sensor that contributed to the sample's history.

---

## 3. Paging and On-Call Integration

### 3.1 Webhook Adapter

Compliant deployments MUST support outbound webhooks for alerts:

```
POST {operator_endpoint}
Content-Type: application/json
WIA-Signature: t={timestamp}; v1={hex_hmac_sha256}

{ "alert": {...}, "facility": {...} }
```

The webhook signature MUST use HMAC-SHA-256 with a per-endpoint secret. Receivers SHOULD reject signatures older than 5 minutes.

### 3.2 Email and SMS

Optional adapters for SMTP (IETF RFC 5321) and SMS (operator-specific) MAY be enabled. When enabled, they MUST NOT replace the webhook path — webhook delivery is the audit-of-record path.

### 3.3 Acknowledgement

Operator acknowledgement of an alert via the integrated paging path MUST be reflected back into the cryo-monitoring backend through the PHASE 2 `POST /alerts/{id}/ack` endpoint, preserving the operator identity.

---

## 4. Multi-Site Federation

### 4.1 Federation Model

Federation among multiple sites is an **opt-in pull** model. A consumer site reads a producer site's federation feed:

```
GET https://{producer-host}/api/v1/federation/feed
Authorization: Bearer {jwt}
```

Producers expose the same WIA Cryo Phase 1 payloads with namespace prefixes to avoid identifier collision. Federation does not change the producer's local data of record.

### 4.2 Identity

Each federated event MUST carry the producer's facility identifier and a signed `WIA-PROV` header (HMAC-SHA-256 or detached JWS per IETF RFC 7515) so that the consumer can verify origin.

### 4.3 Conflict Resolution

When a consumer reuses or merges events into local dashboards, conflicts (duplicate `event_id`, identical `observed_at` with different values) MUST resolve to the producer's canonical record; the consumer's view is read-only with respect to federated data.

---

## 5. Reporting Profile

### 5.1 CSV Profile

The reporting CSV profile follows IETF RFC 4180 with UTF-8 encoding and the following fixed header for the per-event report:

```
event_id,facility_id,sample_ref,observed_at,kind,value,unit,quality,severity,acknowledged_at,ack_by
```

### 5.2 PDF Profile

PDF reports MUST be PDF/A-3 (ISO 19005-3:2012) and embed the underlying CSV as an attachment so that downstream auditors can re-process without parsing the rendered text.

### 5.3 Localization

All reports MUST be available in at least Korean and English; additional locales follow the WIA Standards site's language list.

---

## 6. Sustainability Disclosure (Optional but Recommended)

The following optional fields MAY be reported alongside facility heartbeat events:

- `power.kwh.day` — total electrical energy consumed by the cryo monitoring infrastructure in the previous calendar day.
- `ln2.consumption.l.day` — liquid nitrogen consumption in liters per day.
- `refrigerant.gwp_total` — sum of global-warming-potential equivalent for refrigerants in service (informational).
- `water.l.day` — water consumption attributable to the cryogenic infrastructure, where applicable.

These fields are non-normative and are intended to support voluntary reporting frameworks.

---

## 7. Operational Failure Modes

The ecosystem integrations described in §2–§6 MUST NOT compromise the alert-of-record path:

- LIMS unavailability MUST NOT delay alert delivery beyond the deadlines defined in PHASE 3 §4.
- Webhook receiver outages MUST NOT cause backpressure into the alert publication path; deliveries are buffered and retried with exponential backoff.
- Federation partner outages affect read-only consumer dashboards only.

---

## 8. Compliance and Certification

| Tier | Required Phase 4 surface |
|------|---------------------------|
| Tier 1 — Self-declared | LIMS handshake (§2) or webhook adapter (§3) implemented; reporting profile (§5) available |
| Tier 2 — Assessed | Tier 1 + signed assessor report against this PHASE + at least 90 days of federation log retention if §4 is implemented |
| Tier 3 — Accredited | Tier 2 + WIA accreditation + ISO/IEC 17065:2012 conformity assessment + ISO 20387:2018 alignment for biobanking deployments |

---

## 9. Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2026-04-26 | Initial PHASE 4 specification (this document) |

---

## 10. References

- WIA Cryo-Monitoring PHASE 1 Data Format Specification (this repository)
- WIA Cryo-Monitoring PHASE 2 SDK and Client API Specification (this repository)
- WIA Cryo-Monitoring PHASE 3 Communication Protocol Specification (this repository)
- ISO 20387:2018 — Biotechnology — Biobanking — General requirements for biobanking
- ISO 19005-3:2012 — Document management — Electronic document file format for long-term preservation (PDF/A-3)
- ISO/IEC 17065:2012 — Conformity assessment
- W3C PROV-DM — provenance data model
- IETF RFC 4180 — Common Format and MIME Type for CSV Files
- IETF RFC 5321 — Simple Mail Transfer Protocol
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- IETF RFC 7515 — JSON Web Signature (JWS)
- IETF RFC 9110 — HTTP Semantics

---

## Annex A — Conformance Tier Matrix

WIA conformance for the WIA Cryo-Monitoring Phase 4 ecosystem integration surface is evaluated across three tiers as summarized in §8. Implementations MUST publish their declared tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page; tier downgrade events MUST be reported to the WIA registry within 30 days.

| Tier | Mandatory artifacts | Audit cadence |
|------|--------------------|----------------|
| Tier 1 — Self-declared | OpenAPI 3.0 contract for the §2 / §3 endpoints, sample CSV / PDF/A-3 reports per §5, webhook signature verification report | Annual self-review |
| Tier 2 — Third-party assessed | Tier 1 + signed third-party assessor report + 90-day federation retention | Every 24 months |
| Tier 3 — Accredited | Tier 2 + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, ISO 20387:2018 alignment for biobanking deployments, evidence retention ≥ 7 years | Every 12 months |

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references the standards listed in §10. Implementers SHOULD review the listed standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation. This cross-walk is informative only.

---

## Annex C — Reference Implementations and Test Vectors

The WIA-Official GitHub organization publishes:

- `wia-standards/standards/cryo-monitoring/api/` — federation feed and webhook helpers
- `wia-standards/standards/cryo-monitoring/cli/` — POSIX shell client to exercise §2 / §3 endpoints
- `wia-standards/standards/cryo-monitoring/simulator/` — interactive simulator with sample LIMS handshake fixtures

Test vectors include (a) signed webhook payloads, (b) example federation feed responses with `WIA-PROV` headers, (c) PDF/A-3 sample reports with embedded CSV. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector.

---

## Annex D — Open Questions and Future Work

1. **Federated authorization** — token-exchange profile for cross-organization read scopes.
2. **PDF/A-4** — migration path once ISO 19005-4 is widely deployed in PDF tooling.
3. **Sustainability schema convergence** — alignment with sectoral disclosure frameworks as they stabilize.
4. **Sample chain-of-custody** — first-class linkage to courier handoff events for transported samples.
5. **Localization expansion** — automated re-rendering of reports across the full WIA language list.

Items in this annex are non-normative.

---

**弘益人間 · Benefit All Humanity**

© 2026 WIA — World Certification Industry Association
Licensed under MIT License


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

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


