# WIA-XR-VR-001 — Virtual Reality

> 가상 현실 표준 — 가상 현실 · OpenXR · 6DoF · 햅틱 · 사이버멀미 완화

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
virtual reality hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- Khronos OpenXR 1.0
- W3C WebXR Device API
- Khronos glTF 2.0 / KTX 2.0
- ISO/IEC 18039 (Mixed Augmented Reality)
- IEEE 3079.1 (Cybersickness mitigation)
- IEEE 2888 (Cyber Real Interfaces)
- WCAG 2.2 + W3C XR Accessibility User Requirements
- ANSI/IEEE C95.1-2019 (RF Safety)
- ANSI Z80.36-2016 (Virtual reality eyewear)

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. OpenXR Runtime Architecture
2. 6DoF Tracking Technologies
3. Display Technologies
4. Haptic Feedback Systems
5. Cybersickness Mitigation IEEE 3079.1
6. Audio Spatial Rendering
7. Authoring and Game Engines
8. Health Safety per ANSI Z80.36

## CLI

A POSIX shell helper is published at `cli/virtual-reality.sh`. The helper ships
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
