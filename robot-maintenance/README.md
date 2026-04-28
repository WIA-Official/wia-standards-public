# WIA-ROBOT-MAINT-001 — Robot Maintenance

> 로봇 유지보수 표준 — 로봇 유지보수 · 예지보전 · 진동 분석 · OEE · IoT 진단

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
robot maintenance hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- ISO 17359:2018 (Condition monitoring + diagnostics)
- ISO 13373 series (Vibration monitoring)
- ISO 18436 series (Personnel competence)
- ISO 10816 / ISO 20816 (Vibration severity)
- IEC 60300-3-11 (RCM)
- ISO 55000 series (Asset management)
- VDI 4499 (Digital factory + simulation)
- OPC UA Companion Specification for Robotics
- MIMOSA OSA-CBM 3.3.1

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. Predictive Maintenance Foundations
2. Condition Monitoring per ISO 17359
3. Vibration Analysis per ISO 20816
4. Asset Management per ISO 55000
5. MIMOSA OSA-CBM Architecture
6. OEE and Performance Metrics
7. Digital Twin per VDI 4499
8. Robot Specific Diagnostics

## CLI

A POSIX shell helper is published at `cli/robot-maintenance.sh`. The helper ships
sample envelope generators with no dependencies beyond `jq` and POSIX
shell so integrators can exercise the contract before wiring real
backends.

## Reference simulator

A browser-based reference simulator is published at `simulator/index.html`.
The simulator surfaces every Phase 1 envelope and walks every Phase 2
endpoint exchange so adopters can study the contract behaviour before
integrating with a real backend.

## Conformance

A standard is conformant when:

1. Every Phase 1 envelope it emits validates against the published
   JSON Schema for that envelope class
2. Every Phase 2 endpoint it exposes honours the documented status
   codes, content shapes, and error envelopes
3. Every Phase 3 protocol exchange it participates in honours the
   handshake order, signature requirements, and audit hook contract
4. Phase 4 ecosystem composition reaches the required cross-standard
   capabilities (audit transport, identity, federation) per the
   Phase 4 §4 capability matrix

## Governance

Maintained under the WIA Standards public-benefit governance model.
See `https://wiastandards.com/governance/` for committee composition,
voting rules, and the per-standard release calendar.

弘益人間 — Benefit All Humanity.
