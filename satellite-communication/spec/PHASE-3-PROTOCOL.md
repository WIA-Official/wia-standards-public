# WIA-SPACE-003 (satellite-communication) — Phase 3: Protocol Specification

> **Version:** 1.0.0
> **Status:** Draft
> **Phase:** 3 of 4 (Protocol)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 3 specifies the on-the-wire protocols by which WIA-SPACE-003 deployments coordinate across trust boundaries — between an operator and its mission partners, between an operator and its regulator, between an operator and a downstream science archive. The protocols are layered above the CCSDS framing of Phase 2 (TM and TC frames, SDLS, BPv7) and inherit the CCSDS conventions for time, identity, and audit.

### 1.1 Time discipline

All Phase 3 protocol exchanges carry timestamps in **TAI** per BIPM SI Brochure conventions, encoded in CCSDS Unsegmented Code (CUC) per **CCSDS 301.0-B-4** for binary contexts and in RFC 3339 with explicit TAI offset for textual contexts. Leap-second jumps in UTC do not affect protocol replay defence; the canonical TAI representation is monotonic.

### 1.2 Replay defence bounds

Every protocol envelope carries a 96-bit nonce and a TAI timestamp. Receivers reject envelopes with skew greater than ±300 seconds and maintain a 600-second seen-nonce cache. The cache is persistent across ground-segment console restarts so a power cycle does not re-open the window for a previously-blocked replay. Spacecraft-side replay defence is delegated to the SDLS Anti-Replay Counter per CCSDS 355.0-B-2 §5.3, which provides equivalent protection at the data-link layer.

---

## 2. Cross-support handshake (CCSDS 902.0-B)

When two agencies cooperate on a mission (e.g., NASA Deep Space Network providing tracking for an ESA spacecraft, or KARI ground station providing downlink for a JAXA satellite during contingency), the cross-support handshake is the canonical protocol. It is a thin wrapper over CCSDS 902.0-B and CCSDS 921.1-B (SLE Service Management).

### 2.1 Handshake state machine

```
IDLE    → BIND-REQUEST     : prime agency requests service binding
BIND    → BIND-CONFIRM     : remote agency accepts, returns service ID
BOUND   → SCHEDULE         : prime agency submits per-pass schedule
ACTIVE  → SLE forward/return : actual SLE service traffic flows
ACTIVE  → UNBIND-REQUEST   : prime agency releases binding
UNBOUND → audit envelopes signed by both agencies
```

The handshake reuses the WIA-SOCIAL Phase 3 §5 receipt shape so that vendor implementations across multiple WIA-family standards share their federation library; the addition is the CCSDS-specific service identifier (forward CLTU per CCSDS 911.1-B vs return all frames per CCSDS 911.2-B vs return online frames per CCSDS 911.5-B).

### 2.2 Handshake envelope schema

```json
{
  "wia_space_003_version": "1.0.0",
  "type": "cross_support_handshake",
  "handshake_id": "cs_01HX...",
  "prime_agency": "did:wia:agency:nasa-dsn",
  "remote_agency": "did:wia:agency:esa-estrack",
  "spacecraft_id": "did:wia:spacecraft:bepicolombo",
  "sle_service": "rcf" | "raf" | "cltu" | "rocf",
  "service_window_start_tai": "RFC 3339 + TAI offset",
  "service_window_end_tai": "RFC 3339 + TAI offset",
  "ccsds_902_agreement_uri": "<URI>",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

The `ccsds_902_agreement_uri` points at the signed bilateral agreement under CCSDS 902.0-B. The handshake envelope refers to the agreement; it does not replicate it. Auditors fetch the agreement separately when they need the full terms.

---

## 3. Conjunction-warning protocol

LEO and MEO operators publish conjunction-warning envelopes (predicted close approaches between two space objects) through this protocol. The data source is the operator's flight-dynamics system or a third-party provider (CSpOC / 18 SDS for the US, EU SST for Europe, KARI 우주환경 감시센터 for Korea); the wire format is uniform regardless of source.

### 3.1 Conjunction envelope schema

```json
{
  "wia_space_003_version": "1.0.0",
  "type": "conjunction_warning",
  "warning_id": "cw_01HX...",
  "primary_object": { "norad_id": 12345, "name": "...", "operator": "did:wia:operator:..." },
  "secondary_object": { "norad_id": 67890, "name": "...", "operator": "did:wia:operator:..." },
  "tca_tai": "<closest-approach time>",
  "miss_distance_m": 740,
  "miss_distance_uncertainty_m": 380,
  "probability_of_collision": 1.4e-5,
  "issuing_authority": "did:wia:cspoc",
  "issued_at_tai": "<TAI timestamp>",
  "expires_at_tai": "<TAI timestamp>",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

### 3.2 Operational discipline

Conjunction warnings are the highest-priority traffic class on the deployment. Hosts MUST: deliver every warning to the affected operator within 60 seconds of issuance; emit a warning envelope to operations when delivery latency exceeds the threshold; and persist every warning to the audit log for at least 10 years (the typical regulator-archive horizon for orbital safety records).

Operators receiving a conjunction warning evaluate manoeuvre options under their flight-dynamics system; the standard does not adjudicate manoeuvre vs no-manoeuvre. It does require the operator to publish a `conjunction_response` envelope so the chain of decision is auditable downstream.

---

## 4. Frequency-coordination protocol

Spectrum coordination across operators flows through this protocol. The data source is ITU-R BR (Radiocommunication Bureau) for primary-jurisdiction allocations, and the operator's national administration for the operator's filed records (FCC IBFS in the US, Ofcom satellite licensing register in the UK, MSIT 무선국 데이터베이스 in Korea, MIC 周波数管理 system in Japan).

### 4.1 Coordination envelope schema

```json
{
  "wia_space_003_version": "1.0.0",
  "type": "coordination_request",
  "request_id": "fc_01HX...",
  "filing_administration": "USA" | "GBR" | "KOR" | "JPN" | ...,
  "filing_record_id": "<ITU-R notification ID>",
  "frequency_band_mhz": [ 14000.0, 14500.0 ],
  "polarization": "RHCP" | "LHCP" | "linear",
  "service_area_iso6709": [ "+37.5665+126.9780/", ... ],
  "service_window_start_tai": "<TAI>",
  "service_window_end_tai": "<TAI>",
  "interference_budget_dbW": -160,
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

The `interference_budget_dbW` is the maximum interference power the operator is willing to accept from peer transmissions before triggering a coordination dispute. Disputes flow through the operator's national administration to ITU-R BR per Article 22 dispute-resolution conventions; the protocol envelope is the wire format for dispute correspondence.

---

## 5. Mission-event protocol

Operationally significant events (apogee burn, deorbit burn, payload deployment, attitude anomaly, propulsion-system stress) flow through this protocol so that mission partners and regulators have a uniform feed.

### 5.1 Mission-event envelope schema

```json
{
  "wia_space_003_version": "1.0.0",
  "type": "mission_event",
  "event_id": "me_01HX...",
  "spacecraft_id": "did:wia:spacecraft:...",
  "event_class": "manoeuvre" | "anomaly" | "deployment" | "ground_safety_incident" | "spectrum_event",
  "event_subclass": "<short token>",
  "occurred_at_tai": "<TAI>",
  "operator_summary": "<short prose>",
  "telemetry_evidence_uri": "<URI of TM session reference>",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

Mission-event envelopes feed the operator's incident-response workflow and (when the event class is `ground_safety_incident`) the operator's national administration for required reporting. The standard does not replace the formal incident-report process; it provides the wire format that survives translation between the operator's tooling and the regulator's intake portal.

---

## 6. Cross-orbit handover protocol

When a service spans multiple orbits (e.g., a LEO downlink rolling over to a GEO downlink during gap-filling), the cross-orbit handover protocol coordinates the source and destination ground-segment elements.

### 6.1 Handover state machine

```
PENDING   : prime ground segment requests handover; predicted handover time emitted
COMMITTED : both source and destination acknowledge; service migration scheduled
EXECUTED  : service migration completed; latency-of-handover measured
RECONCILED: audit envelopes from both sides written to the audit log
```

Handover envelopes carry the per-orbit link descriptors (Phase 2 §2.3), the predicted glass-to-glass latency delta, and the customer notification chain so that customer applications can adapt their buffer strategies during the transition.

### 6.2 Handover failure modes

| Failure | Detection | Operator response |
|---------|-----------|-------------------|
| Source link fade exceeds threshold mid-handover | Link-budget telemetry | Abort handover; restore source link if possible; otherwise emit service-interruption envelope |
| Destination cannot accept (capacity exhausted) | Destination ack carries refusal reason | Try alternate destination; failing that, accept service interruption |
| SDLS key rollover during handover window | Key-management envelope | Coordinate key rollover before handover; fall back to encrypted-only fail-safe |
| Cross-support agreement does not cover destination | CCSDS 902.0-B agreement check | Refuse handover; propose alternate destination with covered agreement |

---

## 7. Audit log discipline

Every Phase 3 protocol envelope is written to an append-only log replicated across at least two storage backends. Retention is sized to the longest applicable regulatory window: 10 years for orbital-safety records, 7 years for spectrum-coordination records, 5 years for general operational records. The audit log is exposed via a federated query endpoint at `GET /audit?from=…&to=…&type=…` so an auditor reconstructing an incident can verify the chain without trusting the operator's current state.

---

## 8. Cross-standard composition

Phase 3 composes with: WIA-OMNI-API for operator and console identity, WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for the federation receipt shape, and WIA-INTENT for declaring the highest-level mission intent. The composition lets a single satellite-communication operator running multiple WIA-family standards reuse one identity, signature, audit, and federation machinery rather than maintaining separate parallel implementations.

---

## 9. Conformance test coverage

The Phase 3 conformance suite walks through: cross-support handshake against a mock partner agency, conjunction-warning round-trip with mock CSpOC data, frequency-coordination request-and-response, mission-event publication with audit-log assertion, cross-orbit handover happy path, and every failure mode in §6.2.

---

## 10. References

- CCSDS 301.0-B-4 — Time Code Formats
- CCSDS 355.0-B-2 — Space Data Link Security Protocol (SDLS)
- CCSDS 902.0-B — Cross-Support Concept and Reference Architecture
- CCSDS 911.1-B — SLE Forward CLTU Service
- CCSDS 911.2-B — SLE Return All Frames Service
- CCSDS 911.5-B — SLE Return Online Frames Service
- CCSDS 921.1-B — SLE Service Management
- ITU-R Radio Regulations Article 22 — Space services
- ITU-R Recommendation S.1503 — Functional description of software for assessing non-GSO interference
- ISO 24113 — Space systems — Space debris mitigation requirements
- ISO/IEC 27001:2022 — Information security management
- IEC 62443-3-3:2013 — System security requirements and security levels
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
