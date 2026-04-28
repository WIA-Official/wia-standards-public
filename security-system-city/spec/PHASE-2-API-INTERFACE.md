# WIA-CITY-014 (security-system-city) — Phase 2: API Interface Specification

> **Version:** 1.0.0
> **Status:** Draft
> **Phase:** 2 of 4 (API Interface)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 2 specifies the API surface a city-scale physical-security deployment exposes to the security operations centre (SOC), the city's emergency-management agency, and downstream consumers (regulator, third-party auditor, member-agency partner). The surface is rooted in the canonical conventions of physical-security and security-operations standards: ONVIF for video and access control on the wire, IEC 62676 series for the video-surveillance system architecture, ISO/IEC 27001 for information-security management, and ISO 22320 for emergency-management coordination.

### 1.1 Authorization model

Authorization on a city-scale security deployment composes:

| Concern | Mechanism | Reference |
|---------|-----------|-----------|
| Operator console identity | X.509 client certificate per ISO/IEC 27001 §A.5.16 (identity management) | ISO/IEC 27001:2022 |
| Device-side authorization | ONVIF Profile T security profile (ISO/IEC 27001 + IEC 62443-3-3 SR 1.1) | ONVIF Profile T 2024.06; IEC 62443-3-3:2013 |
| Per-operation entitlement | Role-based access aligned to NIST SP 800-160 Vol. 2 systems-security engineering | NIST SP 800-160 Vol. 2 |
| Cross-agency cooperation | ISO 22320 emergency-management peer trust list | ISO 22320:2018 |
| Records retention | NIST SP 800-53 Rev 5 AU controls (audit and accountability) | NIST SP 800-53 Rev 5 |
| At-rest video integrity | IEC 62676-2-31 evidence-grade encoding with cryptographic chain | IEC 62676-2-31:2019 |

API requests originate from authenticated SOC consoles whose human-user identity is bound to the city's identity-management register (often a national or municipal PKI rather than a public CA). The console signs every API request with an X.509 certificate; the certificate chain anchors at the city's PKI per ISO/IEC 27001 §A.5.17 (authentication information).

### 1.2 Privacy floor

Every Phase 2 endpoint that returns or accepts personal data (camera feed, access-control event, registered subject record) MUST honour the deployment's published Data Protection Impact Assessment (DPIA, ISO/IEC 29134:2023) and the applicable jurisdiction's privacy law (GDPR Article 32 in the EU, KR PIPA Article 29 in Korea, CCPA/CPRA in California, LGPD in Brazil). Endpoints refuse operations that would violate the published purpose limitations.

---

## 2. HTTP/REST Surface

### 2.1 Base URL and discovery

```
https://<host>/wia-city-014/v1
```

Discovery document at `/.well-known/wia-city-014`:

```json
{
  "wia_city_014_version": "1.0.0",
  "deployment_id": "<UUID v4 per RFC 9562>",
  "operator": "<operator-name>",
  "supported_profiles": ["ONVIF Profile T", "ONVIF Profile A", "ONVIF Profile C"],
  "iec_62676_compliance": "Part 1, 2, 4",
  "dpia_uri": "<URI of published DPIA>",
  "regulatory_jurisdiction": ["GDPR", "PIPA"],
  "endpoints": { "camera": "...", "access_point": "...", "incident": "...", "subject": "..." }
}
```

### 2.2 Camera endpoint

```
GET    /camera                       → list cameras
GET    /camera/{id}                  → camera descriptor (Phase 1 §2.2)
GET    /camera/{id}/stream            → live or archived stream (RTSP / SRTP / WebRTC)
GET    /camera/{id}/event-feed        → SSE stream of motion / VCA events
POST   /camera/{id}/ptz               → pan-tilt-zoom command (ONVIF Profile T)
```

The stream endpoint negotiates the transport per ONVIF Profile T: RTSP/SRTP for legacy NVR consumers, WebRTC for browser-based consoles, and ONVIF over HTTPS for inter-device exchanges. SOC consoles consume both the stream and the analytics event feed; downstream forensic systems consume only the archived stream with timestamps anchored to TAI per BIPM conventions.

### 2.3 Access-point endpoint

```
GET    /access-point/{id}            → access-point descriptor
POST   /access-point/{id}/event      → access-grant or access-denied event (ONVIF Profile A)
GET    /access-point/{id}/feed       → SSE stream of access events
POST   /access-point/{id}/lockdown   → lockdown command (incident response)
```

Access-control events follow ONVIF Profile A. The event envelope carries the credential identity (badge ID hashed with the operator's salt to prevent rainbow-table attacks), the access-point identity, the decision (granted / denied / forced), and the reason code per the operator's published reason-code register.

### 2.4 Incident endpoint

```
POST   /incident                      → declare an incident
GET    /incident/{id}                → incident state and dossier
POST   /incident/{id}/escalate        → escalate to next-tier responder
POST   /incident/{id}/close           → close incident with after-action notes
```

Incidents flow through the ISO 22320 emergency-management state machine (declared → triaged → escalated → contained → closed → after-action). Each transition emits a signed envelope. Cross-agency escalation invokes the federation handshake (Phase 3 §2) to share the incident dossier under documented jurisdictional terms.

### 2.5 Subject record endpoint (registered persons)

```
GET    /subject/{id}                 → subject descriptor
POST   /subject                       → register a subject (privacy-impacted; DPIA-gated)
DELETE /subject/{id}                 → tombstone (GDPR Article 17 erasure)
GET    /subject/{id}/access-history   → access-history feed (DPIA-gated)
```

Subject endpoints are the highest-privacy-risk part of the API. The deployment publishes a signed DPIA referencing ISO/IEC 29134:2023 and the jurisdiction-specific privacy law; endpoints refuse subject queries that fall outside the DPIA's documented purpose limitations. Erasure (DELETE) tombstones the record but preserves audit-grade access history per the operator's retention policy and the regulator's archive requirement.

### 2.6 Mass-event endpoint (gatherings, parades, evacuations)

```
GET    /mass-event/{id}              → mass-event descriptor
POST   /mass-event/{id}/wave         → emit a wave-of-people-density envelope
GET    /mass-event/{id}/dashboard    → operator's dashboard rollup
```

Mass-event envelopes carry per-zone density estimates (counts per square metre) computed by the deployment's video-content-analytics (VCA) layer. Density above the operator's safety threshold triggers a recommendation envelope to the on-scene incident commander; thresholds vary per venue and are documented in the operator's emergency-management plan.

---

## 3. Idempotency and retry semantics

Every write endpoint accepts the `Idempotency-Key` header per IETF draft `draft-ietf-httpapi-idempotency-key-header`. Hosts retain a 24-hour replay cache per console identity. Lockdown commands and incident-state transitions inherit the same retention so a console reconnecting after a transient outage can safely retry without producing duplicate physical-security actions.

---

## 4. Pagination, filtering, and bulk export

Collection endpoints support cursor pagination (`?cursor=…&limit=…`) per IETF `draft-ietf-httpapi-link-relations`. Bulk export at `POST /exports` accepts a time window plus an entity-type filter and returns a signed manifest with a Merkle root over the included envelopes. The manifest is signed by the operator and counter-signed by the regulator's auditor when the export is for regulatory inspection per ISO/IEC 27001 §A.5.30 (ICT readiness for business continuity).

---

## 5. Health and observability

```
GET /health   → liveness
GET /ready    → readiness (includes camera connectivity check)
GET /metrics  → Prometheus exposition (operator-actionable counters)
```

The `/metrics` endpoint exposes: cameras-online ratio per zone, access-event rate per access-point, incident-creation rate per zone, mass-event density distribution. Telemetry MUST NOT include high-cardinality labels (per-camera identifiers, per-subject identifiers) to preserve the privacy floor.

---

## 6. Error model

Errors return RFC 9457 problem documents. Reserved problem types relevant to Phase 2:

| Type | Status | Meaning |
|------|--------|---------|
| `…/dpia-violation` | 403 | The requested operation falls outside the deployment's published DPIA purpose limitations. |
| `…/onvif-auth-failed` | 401 | ONVIF authentication did not verify against the configured device certificates. |
| `…/lockdown-denied` | 403 | The console identity lacks lockdown authorisation per ISO/IEC 27001 access-control register. |
| `…/iec-62676-evidence-broken` | 422 | Evidence-grade encoding failed integrity check. |
| `…/rate-limited` | 429 | Per-console rate limit exceeded; retry per `Retry-After`. |
| `…/cross-agency-trust-required` | 401 | Operation requires a federated trust handshake (Phase 3 §2) before proceeding. |
| `…/erasure-conflict-with-investigation` | 409 | Erasure refused because the subject is named in an active legal-hold investigation. |

---

## 7. Conformance test suite

A black-box conformance test suite is published at `https://github.com/WIA-Official/wia-security-system-city-conformance` and walks through every Phase 2 endpoint, the ONVIF Profile T / A / C round-trip, the IEC 62676-2-31 evidence-grade encoding chain, the GDPR Article 17 erasure flow with legal-hold conflict, and the bulk-export Merkle root check. Hosts publishing `bridge_profile=Full` SHOULD additionally pass the suite's ISO 22320 cross-agency federation extension tests.

---

## 8. References

- IEC 62676-1-1:2020 — Video surveillance systems for use in security applications — System requirements
- IEC 62676-2-1:2020 — Video transmission protocols — General
- IEC 62676-2-31:2019 — Internet protocol (IP) interoperability for ONVIF-conformant systems — Profile T
- IEC 62676-4:2018 — Application guidelines
- ONVIF Profile T 2024.06 — Streaming and analytics for IP-based video systems
- ONVIF Profile A 2024.06 — Access control configuration
- ONVIF Profile C 2024.06 — Access control monitoring
- ISO/IEC 27001:2022 — Information security management
- ISO/IEC 27002:2022 — Code of practice for information security controls
- ISO/IEC 29134:2023 — Privacy impact assessment guidelines
- ISO 22320:2018 — Security and resilience — Emergency management
- IEC 62443-3-3:2013 — System security requirements and security levels
- NIST SP 800-53 Rev 5 — Security and Privacy Controls for Information Systems
- NIST SP 800-160 Vol. 2 — Systems Security Engineering — Cyber-resilient systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 9562 — Universally Unique IDentifiers (UUIDs)
- IETF RFC 8446 — TLS 1.3
- BIPM SI Brochure — Time scales (TAI / UTC / leap seconds)

---

## 9. Implementer note — operational lifecycle

A city-scale physical-security deployment integrates with municipal infrastructure that has a 20-30 year asset lifecycle (CCTV poles, fibre runs, access-control hardware). The wire formats and protocol disciplines that operate it must absorb that horizon without locking the city into a single vendor or a single regulator's interpretation. The standard's backwards-compatibility promise (within the 1.x line, no Phase field shape, no endpoint, no protocol exchange will be removed) is therefore mandatory rather than aspirational.

弘益人間 — Benefit All Humanity.


## 10. Closing implementer note for Phase 2

Phase 2 endpoints are the day-to-day surface a SOC operator
interacts with. The discipline is high — every request signed,
every operation auditable, every privacy-impacted endpoint gated
on the published DPIA — because the alternative is a city
surveillance network operating without the auditable evidence
chain that distinguishes lawful surveillance from arbitrary
surveillance. The standard insists on the discipline as a
precondition for the wire format itself.
