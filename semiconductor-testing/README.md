# WIA-SEMI-TEST-001 — Semiconductor Testing

> 반도체 시험 표준 — ATE · 웨이퍼 프로빙 · BIST · 신뢰성 시험 · JEDEC

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
semiconductor testing hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- JEDEC JESD22 series (Reliability Test Methods)
- JEDEC JESD47 (Stress-Test-Driven Qualification)
- JEDEC JESD89 (Single Event Effects)
- IEEE 1450 STIL (Standard Test Interface Language)
- IEEE 1149.x JTAG family
- IEEE 1500 (Embedded Core Test)
- IEEE 1687 IJTAG
- AEC-Q100 / Q104 (Automotive)
- MIL-STD-883 (Military / Aerospace)

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. ATE Test Program Architecture
2. Wafer Probing
3. Built-In Self Test (BIST)
4. Boundary Scan IEEE 1149 family
5. Stress Tests per JEDEC JESD22
6. Reliability Qualification JESD47
7. Single Event Effects per JESD89
8. AEC-Q100 Automotive Qualification

## CLI

A POSIX shell helper is published at `cli/semiconductor-testing.sh`. The helper ships
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
