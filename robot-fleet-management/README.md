# WIA-ROBOT-FLEET-001 — Robot Fleet Management

> 로봇 군집 관리 표준 — 로봇 함대 관리 · 다중 로봇 · 작업 할당 · 텔레메트리

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
robot fleet management hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- ISO 22166-1:2021 Modular service robots - General Requirements
- ROS 2 (Robot Operating System) Iron Irwini
- VDA 5050 v2.0 (AGV interface)
- OPC UA Companion Spec for Robotics
- ISO 8373:2021 (Robotics vocabulary)
- ISA-95 (Enterprise-Control System Integration)
- MQTT 5.0 per OASIS
- OpenRMF (Open Robotics Middleware Framework)
- ISO 18646 series (Robot performance)

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. Fleet Management Architecture
2. VDA 5050 AGV Interface
3. Multi-Robot Task Allocation
4. Path Planning per Multi-Agent
5. Telemetry and Observability
6. Map and Localization Sharing
7. Charging and Battery Management
8. Operations and Safety per ISO 22166

## CLI

A POSIX shell helper is published at `cli/robot-fleet-management.sh`. The helper ships
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
