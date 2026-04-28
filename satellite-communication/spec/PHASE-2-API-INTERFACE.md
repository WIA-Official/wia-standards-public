# WIA-SPACE-003 (satellite-communication) — Phase 2: API Interface Specification

> **Version:** 1.0.0
> **Status:** Draft
> **Phase:** 2 of 4 (API Interface)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 2 specifies the API surface a WIA-SPACE-003 deployment exposes to ground-segment operators, mission-control consoles, and downstream consumers (regulators, broadcasters, science archives). The surface is intentionally narrow: every endpoint maps to a Phase 1 entity (Phase 1 §2). Cryptographic identity, authorization, and audit logging are pinned to the canonical space-domain conventions of CCSDS rather than terrestrial enterprise conventions.

### 1.1 Authorization model

Authorization on a satellite-communication deployment is rooted in **CCSDS 355.0-B-2 — Space Data Link Security Protocol (SDLS)**. SDLS provides authentication, integrity, and (optionally) confidentiality at the data-link layer for both telemetry (downlink) and telecommand (uplink) frames; the protocol's Security Header and Security Trailer are the canonical anchor for all Phase 2 access decisions, not bearer tokens at the application layer.

| Concern | Mechanism | Reference |
|---------|-----------|-----------|
| Telecommand authentication | SDLS Authentication Service over CCSDS 232.0-B-4 TC frames | CCSDS 355.0-B-2 §5.2; CCSDS 232.0-B-4 |
| Telecommand confidentiality | SDLS Encryption Service (AES-256-GCM under SDLS framing) | CCSDS 355.0-B-2 §5.4; FIPS 197; NIST SP 800-38D |
| Telemetry authentication | SDLS Authentication Service over CCSDS 132.0-B TM frames | CCSDS 355.0-B-2 §5.2; CCSDS 132.0-B |
| Key management | CCSDS 354.0-M-2 Space Mission Key Management | CCSDS 354.0-M-2 |
| Ground-segment console access | ISO/IEC 27001 §A.5–A.18 controls + IEC 62443-3-3 SR.1.1 | ISO/IEC 27001:2022; IEC 62443-3-3:2013 |
| Inter-agency cross-support | CCSDS 902.0-B Cross-Support Concept | CCSDS 902.0-B |

API requests originate from authenticated ground-segment consoles whose human-user identity is bound to an ISO/IEC 27001 access-control register; the console signs every API request with an X.509 certificate per IEC 62443-3-3 control SR 1.5 (authenticator management). The certificate chain anchors at the operator's PKI, not at a public CA, because operational satellite-comm trust does not reduce to public-web trust.

### 1.2 Frequency-coordination compliance

Every Phase 2 endpoint that touches an active link MUST honour the operator's filing record under **ITU-R Radio Regulations Article 22 (Space services)** and the operator's national administration filings (FCC Part 25 in the US, Ofcom satellite licensing in the UK, MIC告示 in Japan, MSIT 무선국 허가 in Korea). The deployment's `regulatoryFilings` field (Phase 1 §2.1.1) is the authoritative list; endpoints refuse operations that would violate a filed coordination record.

---

## 2. HTTP/REST Surface

### 2.1 Base URL and discovery

```
https://<host>/wia-space-003/v1
```

Discovery document at `/.well-known/wia-space-003`:

```json
{
  "wia_space_003_version": "1.0.0",
  "deployment_id": "<UUID>",
  "operator": "<operator-name>",
  "supported_orbits": ["GEO", "MEO", "LEO", "HEO"],
  "supported_bundles": ["RFC 9171 BPv7"],
  "ccsds_profiles": {
    "tm": "CCSDS 132.0-B",
    "tc": "CCSDS 232.0-B-4",
    "sdls": "CCSDS 355.0-B-2",
    "key_mgmt": "CCSDS 354.0-M-2"
  },
  "itu_filings_uri": "<URI>",
  "endpoints": { "spacecraft": "...", "link": "...", "bundle_session": "...", "telemetry_session": "..." }
}
```

### 2.2 Spacecraft endpoint

```
GET    /spacecraft                 → list of Spacecraft
GET    /spacecraft/{id}            → single Spacecraft
PUT    /spacecraft/{id}            → register / update
GET    /spacecraft/{id}/state      → current operational state
```

The `state` sub-resource carries the latest TT&C session result (Phase 1 §2.5) plus the spacecraft's current attitude solution per the operator's flight-dynamics system. The endpoint is throttled to one request per second per console; sustained polling at higher rates triggers a `429 Too Many Requests` with `Retry-After`.

### 2.3 Link endpoint

```
GET    /link/{id}                  → link descriptor
GET    /link/{id}/budget           → live link-budget telemetry
POST   /link/{id}/handover         → request a handover
```

The link descriptor carries: spacecraft and ground-station / user-terminal endpoints, frequency band, polarisation, modulation (8PSK / QPSK / 16APSK / DVB-S2X), FEC profile (DVB-S2X LDPC + BCH), nominal EIRP, nominal G/T, and the rain-fade margin used for the budget. The budget endpoint returns the live computed numbers and the predicted-degradation envelope for the next 30 minutes.

### 2.4 Bundle session endpoint

```
POST   /bundle-session              → open a BPv7 session (RFC 9171)
GET    /bundle-session/{id}         → session state
POST   /bundle-session/{id}/bundle  → submit a bundle
GET    /bundle-session/{id}/feed    → SSE stream of incoming bundles
DELETE /bundle-session/{id}         → close session
```

Bundles are framed per RFC 9171 BPv7 with the `bcb-cose` security context per RFC 9173 (Bundle Protocol Security). The Phase 2 endpoint validates the BPv7 primary block, dispatches to the ground-segment routing engine, and returns receipts for accepted bundles. Bundles whose security context fails verification produce an RFC 9457 problem document of type `https://wiastandards.com/space/problem/bundle-bpsec-failed`.

### 2.5 Telemetry session endpoint

```
POST   /telemetry-session           → open a TM session over CCSDS 132.0-B
GET    /telemetry-session/{id}      → session state + last-frame summary
GET    /telemetry-session/{id}/feed → SSE stream of TM frames (decoded)
```

Frames are decoded per CCSDS 132.0-B. The decoded payload arrives on the SSE stream as binary CBOR (RFC 8949) with a per-frame timestamp expressed in TAI per BIPM conventions to avoid leap-second ambiguity in operational reconstruction. Hosts MUST NOT expose raw SDLS-protected frame contents on this endpoint without explicit authorisation; the SDLS protection survives application-layer access control.

### 2.6 Service plan endpoint

```
GET    /service-plan                → list active service plans
POST   /service-plan                → create a plan (operator only)
GET    /service-plan/{id}           → plan detail
```

Service plans encode the per-customer entitlement (which links, which orbits, which bandwidth, which jurisdictions). They map to the operator's billing system and to the ITU-R filing record so that a customer's traffic is always coordinated with its filed allocation.

---

## 3. Idempotency and retry semantics

Every write endpoint accepts the `Idempotency-Key` header per IETF draft `draft-ietf-httpapi-idempotency-key-header`. Hosts retain a 24-hour replay cache of accepted keys per console identity. Repeated requests with the same key produce the same response without re-executing the side effect. The retention is sized so that a console reconnecting after a 12-hour ground-segment outage can safely retry every queued write without producing duplicate spacecraft commands.

---

## 4. Pagination, filtering, and bulk export

Collection endpoints support cursor pagination (`?cursor=…&limit=…`) per IETF `draft-ietf-httpapi-link-relations`. Filters use `?filter=…` with a small expression grammar documented in the discovery document. Bulk export at `POST /exports` accepts a time window plus an entity-type filter and returns a signed manifest with a Merkle root over the included envelopes per the standards bulk-export discipline; the manifest is signed by the operator and counter-signed by the auditor when the export is for regulatory inspection.

---

## 5. Health and observability

```
GET /health   → liveness
GET /ready    → readiness (includes ground-segment connectivity check)
GET /metrics  → Prometheus exposition
```

Health endpoints do not require authentication and do not count against the operator's rate limits. The `/metrics` endpoint exposes operator-actionable counters: bundles accepted/rejected per minute, TM frame loss rate per spacecraft, link-budget margin distribution per orbit, and SDLS verification failure count per console. Telemetry MUST NOT include high-cardinality labels (per-frame identifiers, per-customer identifiers); these would explode the time-series database without operational benefit.

---

## 6. Error model

Errors return RFC 9457 problem documents. Reserved problem types relevant to Phase 2:

| Type | Status | Meaning |
|------|--------|---------|
| `…/itu-filing-violation` | 403 | The requested operation would violate a filed ITU-R Article 22 coordination record. |
| `…/sdls-auth-failed` | 401 | SDLS authentication did not verify against the configured key catalogue. |
| `…/bundle-bpsec-failed` | 422 | RFC 9173 BPSec verification failed for the submitted bundle. |
| `…/link-fade-overshoot` | 503 | The link fade margin has exceeded the operator's threshold; no new traffic accepted on this link until it recovers. |
| `…/ccsds-frame-malformed` | 422 | A submitted TC frame violates CCSDS 232.0-B-4 framing rules. |
| `…/cross-support-denied` | 403 | The cross-support agreement (CCSDS 902.0-B) does not cover the requested operation. |
| `…/key-rollover-required` | 412 | The session's SDLS key has reached its rollover threshold; the console MUST request a new key per CCSDS 354.0-M-2 before retrying. |

---

## 7. Conformance test suite

A black-box conformance test suite is published at `https://github.com/WIA-Official/wia-satellite-communication-conformance` and walks through every Phase 2 endpoint, every problem-document path, the SDLS authentication flow, the BPv7 bundle round-trip, the CCSDS 132.0-B TM-decode path, and the bulk-export Merkle root check. Hosts publishing `bridge_profile=Full` SHOULD additionally pass the suite's CCSDS 902.0-B Cross-Support extension tests against a mock partner agency.

---

## 8. References

- CCSDS 132.0-B-3 — TM Space Data Link Protocol
- CCSDS 232.0-B-4 — TC Space Data Link Protocol
- CCSDS 355.0-B-2 — Space Data Link Security Protocol (SDLS)
- CCSDS 354.0-M-2 — Space Mission Key Management Concept
- CCSDS 401.0-B — Radio Frequency and Modulation Systems
- CCSDS 902.0-B — Cross-Support Concept and Reference Architecture
- CCSDS 911.1-B — SLE Forward CLTU Service
- CCSDS 911.2-B — SLE Return All Frames Service
- ITU-R Radio Regulations Article 22 — Space services
- IETF RFC 9171 — Bundle Protocol Version 7
- IETF RFC 9172 — Bundle Protocol Security
- IETF RFC 9173 — Default Security Contexts for Bundle Protocol Security
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 8949 — Concise Binary Object Representation (CBOR)
- IETF RFC 9562 — Universally Unique IDentifiers (UUIDs)
- ISO/IEC 27001:2022 — Information security management
- IEC 62443-3-3:2013 — System security requirements and security levels
- FIPS 197 — Advanced Encryption Standard (AES)
- NIST SP 800-38D — GCM mode of operation
- BIPM SI Brochure — Time scales (TAI / UTC / leap seconds)

---

弘益人間 — Benefit All Humanity.


## Implementer note — operational lifecycle

Satellite-communication implementations have a longer operational
lifecycle than most software systems: a spacecraft launched today
remains in service for 15-25 years on average, and the ground
segment that operates it must continue to honour the protocol
contracts of this Phase across that horizon. The backwards-
compatibility promise (§Phase 4 — within the 1.x line, no Phase
field shape, no endpoint, no protocol exchange will be removed)
is therefore mandatory rather than aspirational; operators rely
on it for capital-allocation decisions on hardware that has
already left the gravitational well.

弘益人間 — Benefit All Humanity.
