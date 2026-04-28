# WIA-CITY-014 (security-system-city) — Phase 3: Protocol Specification

> **Version:** 1.0.0
> **Status:** Draft
> **Phase:** 3 of 4 (Protocol)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 3 specifies the on-the-wire protocols by which a city-scale physical-security deployment coordinates across trust boundaries — between the city's SOC and partner agencies (police, fire, EMS, neighbouring city SOCs), between the SOC and the regulator, between the SOC and downstream forensic labs. The protocols are layered above the Phase 2 API surface and inherit ISO 22320 emergency-management discipline, ISO/IEC 27001 information-security controls, and IEC 62443-3-3 operational-technology security requirements.

### 1.1 Time discipline

All Phase 3 protocol exchanges carry timestamps in **TAI** per BIPM SI Brochure conventions (RFC 3339 with explicit TAI offset). Leap-second jumps in UTC do not affect protocol replay defence; the canonical TAI representation is monotonic. Forensic-grade timestamps additionally carry a documented uncertainty per BIPM JCGM 100 (Guide to the Expression of Uncertainty in Measurement) so a court can reason about timestamp reliability.

### 1.2 Replay defence bounds

Every protocol envelope carries a 96-bit nonce and a TAI timestamp. Receivers reject envelopes with skew greater than ±300 seconds and maintain a 600-second seen-nonce cache. The cache is persistent across console restarts so a power cycle does not re-open the window for a previously-blocked replay.

---

## 2. Cross-agency federation handshake

When the SOC cooperates with a partner agency (police, fire, EMS, federal investigator) on an active incident, the federation handshake is the canonical protocol. It composes ISO 22320 emergency-management coordination with the WIA-SOCIAL Phase 3 §5 receipt shape so that vendor implementations across multiple WIA-family standards share their federation library.

### 2.1 Handshake state machine

```
IDLE     → BIND-REQUEST   : SOC requests cooperation with partner agency
BIND     → BIND-CONFIRM   : partner accepts; trust-list entry exchanged
BOUND    → INCIDENT-SHARE : SOC shares incident dossier under documented purpose
ACTIVE   → JOINT-OPS      : both sides exchange operational envelopes
ACTIVE   → UNBIND-REQUEST : either side releases; remaining work transferred
UNBOUND  → audit envelopes signed by both
```

### 2.2 Handshake envelope schema

```json
{
  "wia_city_014_version": "1.0.0",
  "type": "cross_agency_handshake",
  "handshake_id": "ca_01HX...",
  "soc_agency": "did:wia:soc:seoul-met",
  "partner_agency": "did:wia:police:seoul-met",
  "purpose": "incident_response" | "joint_investigation" | "evidence_handover",
  "scope_uri": "<URI of signed cooperation agreement>",
  "iso22320_role": "primary" | "supporting" | "coordinating",
  "valid_until_tai": "<TAI>",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

The `scope_uri` points at the signed bilateral cooperation agreement (under the city's general legal framework or the relevant national emergency-management law, e.g. Korean 재난 및 안전관리 기본법 for Korea). The handshake envelope refers to the agreement; it does not replicate it.

---

## 3. Incident-dossier exchange protocol

When the SOC shares an incident dossier with a partner agency, the exchange follows a documented protocol that preserves chain-of-custody evidence integrity while honouring the privacy floor of Phase 2 §1.2.

### 3.1 Dossier envelope schema

```json
{
  "wia_city_014_version": "1.0.0",
  "type": "incident_dossier",
  "dossier_id": "id_01HX...",
  "incident_id": "inc_01HX...",
  "shared_with": "did:wia:police:seoul-met",
  "shared_at_tai": "<TAI>",
  "items": [
    {
      "item_id": "ev_01HX...",
      "kind": "video_clip" | "access_event" | "subject_record" | "vca_observation",
      "content_uri": "<URI of evidence-grade encoded content>",
      "content_hash_sha384": "0x...",
      "iec_62676_evidence_grade": true,
      "redaction_applied": "<redaction-policy reference>",
      "purpose_limitation": "<purpose-limitation reference>"
    }
  ],
  "chain_of_custody": [
    { "actor": "did:wia:operator:soc-shift-3", "action": "captured", "at_tai": "..." },
    { "actor": "did:wia:operator:soc-supervisor", "action": "approved-share", "at_tai": "..." }
  ],
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

### 3.2 Operational discipline

- **Evidence-grade encoding**: video clips MUST use IEC 62676-2-31 evidence-grade encoding with cryptographic chain. The encoding is the on-the-wire form; downstream forensic labs verify the chain before admitting the clip as evidence.
- **Redaction**: faces of bystanders unrelated to the incident MUST be redacted per the published redaction policy before sharing outside the SOC. Redaction is logged in chain-of-custody.
- **Purpose limitation**: every shared item carries a purpose-limitation reference; receivers MUST refuse to use the item outside the limitation.
- **Retention**: the receiving agency adopts the SOC's retention policy or a more conservative one. The handshake envelope (§2.2) declares the receiver's retention policy at bind time.

---

## 4. Mass-event coordination protocol

When the SOC's video-content-analytics layer detects mass-event density approaching a safety threshold, the coordination protocol notifies the on-scene incident commander (who may be a city emergency-management official, a police precinct commander, or a private-venue security director).

### 4.1 Density-warning envelope schema

```json
{
  "wia_city_014_version": "1.0.0",
  "type": "density_warning",
  "warning_id": "dw_01HX...",
  "venue_id": "venue_01HX...",
  "zone_id": "zone_01HX...",
  "density_persons_per_sqm": 4.2,
  "density_threshold_persons_per_sqm": 4.0,
  "trend": "rising" | "stable" | "falling",
  "estimated_time_to_critical_threshold_seconds": 120,
  "issued_at_tai": "<TAI>",
  "expires_at_tai": "<TAI>",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

### 4.2 Operational latency budget

The SOC publishes a maximum end-to-end latency budget for density warnings (typically 5 seconds from VCA detection to the on-scene incident commander). Hosts emit a `latency_breach` envelope to operations when delivery exceeds the threshold.

### 4.3 Recommendation rather than command

The SOC issues a recommendation envelope; the on-scene incident commander makes the operational decision. The standard does not adjudicate: it provides the wire format that lets the recommendation be issued, received, acted upon (or refused), and audited later.

---

## 5. Mutual-aid coordination protocol

When an incident exceeds the SOC's response capacity, mutual-aid coordination invokes neighbouring SOCs or specialised agencies. The protocol follows ISO 22320 mutual-aid conventions and reuses the federation handshake envelope of §2.

### 5.1 Mutual-aid request schema

```json
{
  "wia_city_014_version": "1.0.0",
  "type": "mutual_aid_request",
  "request_id": "ma_01HX...",
  "requesting_soc": "did:wia:soc:seoul-met",
  "responding_soc_options": ["did:wia:soc:incheon", "did:wia:soc:gyeonggi"],
  "requested_capabilities": ["additional_camera_streams", "incident_command_support"],
  "estimated_duration_minutes": 240,
  "incident_reference": "inc_01HX...",
  "issued_at_tai": "<TAI>",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

Responding SOCs accept or refuse via signed response envelopes. Acceptance establishes a federation handshake (§2) for the duration of the mutual-aid event.

---

## 6. Audit log discipline

Every Phase 3 protocol envelope is written to an append-only log replicated across at least two storage backends. Retention is sized to the longest applicable regulatory window: typically 5-7 years for incident records, 30 days for routine surveillance footage (under GDPR data-minimisation), and per-evidence-item retention extending to the close of any active legal-hold investigation. The audit log is exposed via a federated query endpoint at `GET /audit?from=…&to=…&type=…` so an auditor reconstructing an incident can verify the chain without trusting the SOC's current state.

---

## 7. Cross-standard composition

Phase 3 composes with: WIA-OMNI-API for SOC-operator and partner-agency identity, WIA-AIR-SHIELD for runtime trust list and key rotation, WIA-SOCIAL Phase 3 §5 for the federation receipt shape, and WIA-INTENT for declaring SOC operational intent (which informs the regulator's monitoring without exposing operational detail).

---

## 8. Conformance test coverage

The Phase 3 conformance suite walks through: federation handshake against a mock partner agency, incident-dossier exchange with chain-of-custody assertion, density-warning round-trip with latency-budget assertion, mutual-aid request with multi-responder negotiation, and the audit-log replication invariant.

---

## 9. References

- ISO 22320:2018 — Security and resilience — Emergency management
- ISO/IEC 27001:2022 — Information security management
- ISO/IEC 27002:2022 — Information security controls
- ISO/IEC 27037:2012 — Guidelines for identification, collection, acquisition, and preservation of digital evidence
- ISO/IEC 27050 series — Electronic discovery
- ISO/IEC 29134:2023 — Privacy impact assessment
- IEC 62676-2-31:2019 — Evidence-grade IP interoperability
- IEC 62443-3-3:2013 — System security requirements
- NIST SP 800-86 — Guide to integrating forensic techniques into incident response
- NIST SP 800-160 Vol. 2 — Cyber-resilient systems
- BIPM JCGM 100 — Guide to the Expression of Uncertainty in Measurement
- BIPM SI Brochure — Time scales (TAI / UTC / leap seconds)
- IETF RFC 9562 — UUIDs
- IETF RFC 8446 — TLS 1.3

---

弘益人間 — Benefit All Humanity.


## 10. Operational case studies — typical incident classes

Three incident classes recur in city-scale deployments and shape the
protocol-layer design:

**Active-shooter / armed-incident class**: highest-priority traffic,
sub-5-second latency budget for camera feeds to police incident
commander, automatic invocation of mutual-aid handshakes with
neighbouring SOCs. Audit retention extends to the close of any
criminal proceeding (typically 5-15 years).

**Mass-gathering safety class** (concerts, parades, sporting
events): density-warning protocol (§4) is the load-bearing path,
with VCA tuning calibrated against the venue's pre-published
density thresholds. Audit retention is the standard 30-day window
for public spaces unless an incident occurs.

**Missing-person class**: subject-record federation with
neighbouring jurisdictions is the load-bearing path, with strict
purpose limitation (Phase 2 §1.2) and erasure-on-resolution
discipline. Subject records that cross a national border require
explicit cross-border data-transfer authorisation per the source
jurisdiction's privacy law.

## 11. Closing protocol note

The Phase 3 protocol layer balances incident-response responsiveness
against audit rigour and privacy floor. The default discipline
favours rigour and privacy; operators can opt out of specific
defaults in jurisdictions where the trade-off favours responsiveness
(e.g., active-shooter class), and the opt-out itself is recorded in
the audit chain.
