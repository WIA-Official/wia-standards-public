# WIA-CITY-017 (traffic-simulation) — Phase 3: Protocol Specification

> **Version:** 1.0.0
> **Status:** Draft
> **Phase:** 3 of 4 (Protocol)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 3 specifies the on-the-wire protocols by which a WIA-CITY-017 deployment coordinates across trust boundaries — between TMC and research partners, between TMC and V2X infrastructure, between TMC and neighbouring jurisdictions, between TMC and the regulator. The protocols are layered above the Phase 2 API surface and inherit ISO 14813-1 ITS service architecture conventions, NTCIP 2202 centre-to-centre protocols, and TMDD v3.x data-dictionary conventions.

### 1.1 Time discipline

All Phase 3 protocol exchanges carry timestamps in **TAI** per BIPM SI Brochure conventions (RFC 3339 with explicit TAI offset). Simulation steps are timestamped relative to scenario start; real-world coordination uses absolute TAI. Microsimulation typically runs at 0.1 s steps; the standard preserves sub-second resolution throughout the protocol stack.

### 1.2 Replay defence bounds

Every protocol envelope carries a 96-bit nonce and a TAI timestamp. Receivers reject envelopes with skew greater than ±300 seconds and maintain a 600-second seen-nonce cache. The cache is persistent across console restarts so a power cycle does not re-open the window for a previously-blocked replay.

---

## 2. SAE J2735 V2X message protocol

The V2X bridge (Phase 2 §2.6) injects and receives SAE J2735 messages. Phase 3 specifies the canonical encoding and the simulation-time-to-real-time mapping.

### 2.1 Supported message types

| Message | SAE J2735 PSID | Encoding | Use in simulation |
|---------|----------------|----------|-------------------|
| BSM (Basic Safety Message) | 0x20 | UPER per ASN.1 | per-vehicle state at 10 Hz |
| MAP (intersection geometry) | 0x82 | UPER per ASN.1 | static infrastructure description |
| SPaT (signal phase and timing) | 0x82 | UPER per ASN.1 | signal-controller state, 10 Hz |
| RSA (road safety alert) | 0x83 | UPER per ASN.1 | incident-injection in scenario |
| TIM (traveler information message) | 0x8003 | UPER per ASN.1 | dynamic traveller information |
| PSM (personal safety message) | 0x27 | UPER per ASN.1 | vulnerable road user representation |

### 2.2 ETSI ITS-G5 regional adaptation

European deployments use ETSI ITS-G5 with semantically-equivalent messages: CAM (Cooperative Awareness Message) for BSM-equivalent state; DENM (Decentralized Environmental Notification Message) for incident reporting; SPATEM for signal phase and timing; MAPEM for intersection geometry. The protocol envelope wraps the regional encoding and declares the region in a `regional_profile` field so consumers can dispatch correctly.

### 2.3 ASN.1 module compatibility

The standard does not invent ASN.1 modules; it consumes the published SAE J2735 and ETSI ITS-G5 modules verbatim. Bridge implementations validate against the canonical modules at `https://github.com/WIA-Official/wia-traffic-simulation-asn1` which mirror the SAE and ETSI publications.

---

## 3. NTCIP 2202 centre-to-centre protocol

When the TMC running the simulation cooperates with a neighbouring TMC (e.g., a regional traffic-management partnership), the centre-to-centre protocol follows NTCIP 2202.

### 3.1 Inter-centre handshake

```
IDLE     → TS-2 BIND-REQUEST    : prime TMC requests centre-to-centre session
BIND     → TS-2 BIND-CONFIRM    : remote TMC accepts; trust-list entry exchanged
BOUND    → DATA-EXCHANGE        : TMDD-formatted envelopes flow both directions
ACTIVE   → SUSPEND / RESUME     : either side temporarily pauses (e.g., maintenance window)
ACTIVE   → UNBIND-REQUEST       : either side terminates the session
UNBOUND  → audit envelopes signed by both
```

### 3.2 TMDD message profile

The protocol carries TMDD v3.x message types: signal-control plans, incident reports, lane-closure announcements, congestion-event notifications. The TMDD message is wrapped in the standards envelope for signature, replay defence, and audit; the TMDD payload is preserved verbatim so legacy consumers continue to consume it directly.

### 3.3 Operational latency budget

Inter-centre data exchange targets 5-second end-to-end latency for incident reports and 30-second latency for routine traffic-state updates. Hosts emit a `latency_breach` envelope when delivery exceeds the budget so operations can investigate.

---

## 4. Calibration-evidence protocol

The calibration evidence chain is the most rigorous part of the protocol layer because simulation results have been a recurring source of policy-debate disputes. The standard requires every published simulation result to include a calibration-evidence envelope linking to real-world observations (loop-detector counts, Bluetooth-MAC trajectory matches, INRIX/HERE probe-vehicle aggregates) plus a documented goodness-of-fit metric (typically GEH < 5 for >85% of count locations per UK Highways Agency conventions).

### 4.1 Calibration envelope schema

```json
{
  "wia_city_017_version": "1.0.0",
  "type": "calibration_evidence",
  "scenario_id": "scn_01HX...",
  "observation_period_start_tai": "<TAI>",
  "observation_period_end_tai": "<TAI>",
  "observation_sources": [
    { "kind": "loop_detector", "operator": "...", "count_locations": 142 },
    { "kind": "bluetooth_mac", "operator": "...", "match_locations": 38 },
    { "kind": "probe_vehicle", "operator": "INRIX", "segment_count": 8742 }
  ],
  "goodness_of_fit": { "metric": "GEH", "threshold": 5, "fraction_below_threshold": 0.91 },
  "operator_summary": "Calibration meets UK Highways Agency convention for forecast-grade microsimulation.",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

### 4.2 Calibration disputes

When a downstream consumer disputes the calibration claim, the audit log carries every calibration envelope ever published for the scenario. The consumer can re-run the goodness-of-fit calculation against the same observation sources without trusting the publisher's current claim. The dispute-resolution process is human (typically through transport-research peer review or regulator inquiry); the standard provides the auditable wire format.

---

## 5. Incident-injection protocol

Researchers and operators inject incidents into running simulations to evaluate response strategies (signal-timing adjustments, dynamic traveller information, V2X message broadcast). The incident-injection protocol formalises the wire format for these operations.

### 5.1 Incident envelope schema

```json
{
  "wia_city_017_version": "1.0.0",
  "type": "incident_injection",
  "run_id": "run_01HX...",
  "incident_id": "inc_01HX...",
  "kind": "lane_closure" | "vehicle_breakdown" | "weather_event" | "special_event",
  "location_iso17572": "<ISO 17572 location reference>",
  "start_simulation_time": 3600,
  "end_simulation_time": 7200,
  "severity": "minor" | "moderate" | "severe",
  "expected_response_strategy": "signal_retiming + RSA broadcast",
  "issued_at_tai": "<TAI>",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

### 5.2 Multi-jurisdiction injection

Incidents that span jurisdictions (a freeway crossing two TMC boundaries) require coordinated injection; the protocol uses the §3 inter-centre handshake to coordinate and emits a federated audit chain across both TMCs.

---

## 6. Audit log discipline

Every Phase 3 protocol envelope is written to an append-only log replicated across at least two storage backends. Retention is sized to the longest applicable regulatory window: typically 5 years for transport-planning records, 10 years for safety-related calibration evidence, and per-incident retention extending to the close of any active investigation.

---

## 7. Cross-standard composition

Phase 3 composes with: WIA-OMNI-API for TMC-operator and research-partner identity, WIA-AIR-SHIELD for runtime trust list and key rotation, WIA-SOCIAL Phase 3 §5 for the federation receipt shape, and WIA V2X (companion standard) when the simulation drives real-world V2X infrastructure rather than only modelling it.

---

## 8. Conformance test coverage

The Phase 3 conformance suite walks through: SAE J2735 message-injection round-trip with mock vehicle stack, NTCIP 2202 inter-centre handshake against mock partner TMC, calibration-evidence publication with goodness-of-fit re-computation, incident-injection with multi-jurisdiction federation, and the audit-log replication invariant.

---

## 9. References

- SAE J2735 (2024) — DSRC Message Set Dictionary
- SAE J2945 series — DSRC system requirements
- ETSI EN 302 663 — ITS-G5 access layer
- ETSI EN 302 637-2 / -3 — CAM / DENM
- ETSI TS 103 301 — SPATEM / MAPEM
- ISO 14813-1 — ITS service architecture
- ISO 17572-1/-2/-3 — Location referencing
- NTCIP 2202 — Internet protocol for ITS centre-to-centre
- NTCIP 1202 — Actuated traffic signal controller
- TMDD v3.x — Traffic Management Data Dictionary
- UK Highways Agency Design Manual for Roads and Bridges (DMRB) — modelling conventions including GEH metric
- ISO/IEC 27001:2022 — Information security management
- BIPM SI Brochure — Time scales (TAI / UTC / leap seconds)
- IETF RFC 8446 — TLS 1.3

---

弘益人間 — Benefit All Humanity.


## 10. Glossary expansion

Microsimulation: per-vehicle simulation at sub-second resolution, typically driven by a car-following model and a lane-changing model. Mesosimulation: per-platoon simulation at second resolution, trading detail for speed. Macrosimulation: per-link aggregate simulation at minute resolution, suitable for regional transport models. OD matrix: origin-destination matrix listing trips between zones. TAZ: Traffic Analysis Zone, the planning-grade spatial unit. GEH: Geoffrey E. Havers statistic, a goodness-of-fit metric standard in traffic engineering. SPaT/MAP: Signal Phase and Timing / intersection map V2X message pair. CAM/DENM: ETSI ITS-G5 Cooperative Awareness Message / Decentralized Environmental Notification Message.

## 11. Implementer note — calibration discipline

The calibration-evidence chain (§4) is the single most-asked-for artefact in traffic-simulation policy debates. A simulation result without published calibration evidence has no traction in regulator review or peer review; the standard makes calibration-evidence publication mandatory rather than optional, and the goodness-of-fit threshold (typically GEH < 5 for >85% of count locations per UK Highways Agency conventions) is the bright line between forecast-grade simulation and uncalibrated speculation.


## 12. Closing protocol note for traffic-simulation

The Phase 3 protocol layer for traffic-simulation balances policy-laboratory rigour against operational responsiveness. Microsimulation runs that feed signal-retiming decisions need calibration-grade evidence; V2X-injection runs that feed RSU-deployment decisions need ETSI ITS-G5 / SAE J2735 fidelity; cross-jurisdiction runs that span TMC boundaries need the federation handshake. Each protocol exchange in this Phase is signed, audited, and replay-defended so the simulation results survive the political cycles that commission them.
