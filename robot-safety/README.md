# WIA-ROBOT-SAFETY-001 — Robot Safety

> 로봇 안전 표준 — 로봇 안전 · ISO 10218 · 협동 로봇 · 안전 정격 모니터링 정지

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
robot safety hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- ISO 10218-1:2025 (Industrial robot safety - robot)
- ISO 10218-2:2025 (Robot system + integration)
- ISO/TS 15066:2016 (Collaborative robots)
- ISO 13482:2014 (Personal care robots)
- ISO 13849-1:2023 (Safety-related control)
- IEC 62061:2021 (Functional safety of safety-related E/E/PE control)
- ISO 12100:2010 (Risk assessment + risk reduction)
- ANSI/RIA R15.06-2012
- IEC 60204-1:2016 (Electrical equipment of machines)

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. ISO 10218 Industrial Robot Safety
2. Collaborative Robots ISO/TS 15066
3. Risk Assessment ISO 12100
4. Functional Safety per ISO 13849
5. IEC 62061 Safety-Related E/E/PE
6. Safety-Rated Stop Categories
7. ANSI/RIA R15.06 + R15.08 AMR
8. Safety Functions and Diagnostic Coverage

## CLI

A POSIX shell helper is published at `cli/robot-safety.sh`. The helper ships
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
