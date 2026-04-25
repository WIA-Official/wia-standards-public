# WIA-FUSION Phase 3: Protocol Specification

**Version:** 1.0.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This specification defines the operational protocols for nuclear fusion reactors, including plasma operation sequences, AI control protocols, and safety procedures.

---

## 2. Operation Protocols

### 2.1 Plasma Startup Sequence

```yaml
Protocol: STARTUP-001
Duration: 20-60 seconds
Phases:
  1. Pre-conditioning (T-60s to T-0s):
     - Vacuum verification: < 1×10⁻⁶ Pa
     - Wall conditioning: Glow discharge
     - Cryopump activation
     - Diagnostic checkout

  2. Magnetic Field Ramp-up (T+0s to T+5s):
     - Toroidal field: 0 → 5.3 T
     - Ramp rate: 1.0 T/s
     - Poloidal field shaping

  3. Gas Injection (T+2s to T+3s):
     - Deuterium puff: 10²⁰ atoms
     - Pressure target: 0.1-1 Pa

  4. Plasma Breakdown (T+3s to T+5s):
     - ECRH pre-ionization: 1-2 MW
     - Loop voltage: 10-20 V
     - Breakdown criteria: Ip > 100 kA

  5. Current Ramp-up (T+5s to T+15s):
     - Target current: 15 MA
     - Ramp rate: 1 MA/s
     - Flux consumption monitoring

  6. Auxiliary Heating (T+10s onwards):
     - NBI activation: gradual ramp
     - ICRH/ECRH tuning
     - Temperature monitoring

Success Criteria:
  - Plasma current: > 10 MA
  - Central temperature: > 5 keV
  - Stored energy: > 100 MJ
  - No disruptions during ramp-up
```

### 2.2 Steady-State Operation Protocol

```yaml
Protocol: STEADY-001
Duration: 300-1000+ seconds
Objective: Maintain stable burning plasma

Control Parameters:
  Core Temperature:
    target: 10-15 keV
    tolerance: ±10%
    feedback: ECRH power modulation

  Density:
    target: 1.0×10²⁰ /m³
    tolerance: ±15%
    feedback: Pellet injection rate

  Plasma Current:
    target: 15 MA
    tolerance: ±5%
    feedback: Inductive + bootstrap

  Beta:
    target: 2.5%
    limit: < 3.5% (stability margin)
    feedback: Heating power reduction

AI Control Loop:
  cycle_time_ms: 1
  inputs:
    - Real-time diagnostics
    - Predictive models
    - Stability indicators
  outputs:
    - Heating power adjustments
    - Shape control commands
    - Fuel injection rate

  optimization_targets:
    - Maximize Q-factor
    - Minimize disruption risk
    - Maintain wall protection
```

### 2.3 Soft Landing Shutdown Protocol

```yaml
Protocol: SHUTDOWN-001
Duration: 60-120 seconds
Objective: Controlled plasma termination

Sequence:
  1. Heating Power Reduction (T+0s to T+30s):
     - Ramp rate: 50% per minute
     - Maintain plasma control
     - Monitor stored energy decrease

  2. Current Ramp-down (T+30s to T+60s):
     - Target: 0 MA
     - Ramp rate: 0.5 MA/s
     - Vertical stability control

  3. Fuel Cutoff (T+45s):
     - Stop gas puffing
     - Stop pellet injection
     - Allow density decay

  4. Magnetic Field Reduction (T+60s to T+90s):
     - Toroidal field ramp-down
     - Poloidal field neutralization

  5. Post-shot Analysis (T+90s onwards):
     - Data archival
     - Component inspection
     - Performance logging

Success Criteria:
  - No disruption during shutdown
  - Wall heat load within limits
  - Complete current termination
```

---

## 3. AI Control Protocol

### 3.1 Real-time Control Architecture

```
┌─────────────────────────────────────────────────────────┐
│                   AI Control System                      │
├─────────────────────────────────────────────────────────┤
│                                                          │
│  ┌──────────┐    ┌──────────┐    ┌──────────────────┐  │
│  │ Sensors  │───▶│ AI Core  │───▶│ Actuator Control │  │
│  └──────────┘    └──────────┘    └──────────────────┘  │
│       │              │                    │             │
│       ▼              ▼                    ▼             │
│  ┌──────────┐  ┌───────────┐    ┌──────────────────┐  │
│  │ 10 kHz   │  │ Neural    │    │ Heating Systems  │  │
│  │ Sampling │  │ Network   │    │ - NBI            │  │
│  │          │  │ Inference │    │ - ECRH/ICRH      │  │
│  └──────────┘  │ < 1 ms    │    │ Shape Coils      │  │
│                └───────────┘    │ Gas Injection    │  │
│                                 └──────────────────┘  │
└─────────────────────────────────────────────────────────┘
```

### 3.2 Disruption Prediction Protocol

```yaml
Protocol: AI-DISRUPT-001
Latency: < 10 ms prediction to action

Input Features:
  - Plasma current (Ip)
  - Internal inductance (li)
  - Beta normalized
  - Locked mode amplitude
  - Radiation peaking factor
  - Edge density gradient
  - q95 safety factor
  - Greenwald fraction
  - MHD activity spectrum

Neural Network Architecture:
  type: LSTM + Attention
  layers: 4
  hidden_units: 256
  training_shots: 50,000+
  accuracy: > 95%
  false_positive_rate: < 5%

Prediction Thresholds:
  GREEN (< 30%):
    action: Continue monitoring
    update_rate: 100 Hz

  YELLOW (30-70%):
    action: Prepare mitigation
    update_rate: 1 kHz
    alerts: Control room notification

  RED (> 70%):
    action: Initiate mitigation
    update_rate: 10 kHz
    response_time: < 30 ms
```

### 3.3 Automatic Mitigation Protocol

```yaml
Protocol: AI-MITIGATE-001

Mitigation Hierarchy:
  Level 1 - Soft Mitigation:
    trigger: Disruption risk 50-70%
    actions:
      - Reduce heating power by 20%
      - Increase edge cooling
      - Modify plasma shape
    success_rate: 80%

  Level 2 - Active Mitigation:
    trigger: Disruption risk 70-90%
    actions:
      - Rapid heating power reduction
      - ECRH mode stabilization
      - Controlled density reduction
    success_rate: 60%

  Level 3 - Massive Gas Injection (MGI):
    trigger: Disruption risk > 90% OR imminent disruption
    actions:
      - Inject 10²³-10²⁴ atoms (Ar, Ne, or D₂)
      - Rapid radiation cooling
      - Runaway electron suppression
    response_time: < 10 ms
    success_criteria: Wall protection achieved
```

---

## 4. Safety Protocols

### 4.1 Emergency Shutdown Protocol

```yaml
Protocol: EMERGENCY-001
Classification: Safety-Critical

Triggers:
  - Unmitigated disruption
  - Cooling system failure
  - Vacuum breach
  - Magnet quench
  - Radiation alarm
  - Fire detection
  - Seismic event (> 0.1g)
  - Manual emergency stop

Sequence (< 5 seconds total):
  1. T+0 ms: Emergency signal received
  2. T+10 ms: MGI triggered
  3. T+100 ms: All heating systems OFF
  4. T+500 ms: Plasma terminated
  5. T+1000 ms: Fast magnetic field dump initiated
  6. T+5000 ms: Facility safe state achieved

Post-Emergency:
  - Automated damage assessment
  - Radiation survey
  - Component inspection
  - Incident report generation
```

### 4.2 Tritium Management Protocol

```yaml
Protocol: TRITIUM-001

Inventory Limits:
  - In-vessel: < 1 kg
  - Storage: < 3 kg
  - Daily throughput: < 100 g

Containment:
  Primary: Vacuum vessel
  Secondary: Glove boxes
  Tertiary: Building containment

Monitoring:
  - Real-time tritium monitors
  - Stack release monitoring
  - Groundwater sampling
  - Personnel dosimetry

Emergency Response:
  trigger: Tritium release > 1 Ci/m³
  actions:
    - Isolate affected area
    - Activate ventilation
    - Personnel evacuation
    - Environmental monitoring
```

### 4.3 Radiation Monitoring Protocol

```yaml
Protocol: RADIATION-001

Monitoring Points:
  - Neutron flux monitors (in-vessel)
  - Gamma monitors (biological shield)
  - Activation monitors (components)
  - Area monitors (facility)
  - Personnel dosimeters

Alarm Levels:
  NORMAL: < 1 µSv/h (facility areas)
  ALERT: 1-10 µSv/h
  HIGH: 10-100 µSv/h
  EVACUATE: > 100 µSv/h

Neutron Activation:
  - Component activation tracking
  - Decay heat calculation
  - Waste classification
  - Decommissioning planning
```

---

## 5. Communication Protocol

### 5.1 Inter-facility Data Exchange

```yaml
Protocol: FUSION-COMM-001
Standard: Based on IMAS (ITER)

Message Format:
  header:
    version: "1.0"
    sender: "KSTAR"
    recipient: "ITER"
    timestamp: ISO8601
    message_type: enum[data|control|alert]
    priority: enum[low|normal|high|critical]

  payload:
    compression: gzip
    encryption: AES-256-GCM
    signature: ECDSA
    data: <binary or JSON>

Transport:
  - Primary: Dedicated fiber network
  - Backup: Encrypted internet
  - Latency: < 100 ms (international)
```

### 5.2 Real-time Control Network

```yaml
Protocol: RT-CONTROL-001

Network Architecture:
  Type: Deterministic Ethernet
  Latency: < 100 µs
  Jitter: < 10 µs
  Redundancy: Dual-ring topology

Message Priority:
  1. Safety interlocks (highest)
  2. Plasma control
  3. Diagnostics
  4. Data logging (lowest)

Synchronization:
  Protocol: IEEE 1588 PTP
  Accuracy: < 1 µs
```

---

## 6. Compliance Checklist

```
□ Startup sequence validated
□ Steady-state control tested
□ Shutdown procedure verified
□ Disruption prediction calibrated
□ Mitigation systems armed
□ Emergency procedures trained
□ Tritium handling certified
□ Radiation monitoring active
□ Communication links tested
□ Documentation complete
```

---

**弘益人間 - Benefit All Humanity**

© 2025 WIA - World Certification Industry Association

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-FUSION is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/WIA-FUSION/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-FUSION/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-FUSION/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.
