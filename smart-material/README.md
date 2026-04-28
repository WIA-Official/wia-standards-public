# WIA-MAT-SMART-001 — Smart Material

> 스마트 물질 표준 — 스마트 물질 · 압전체 · 자기변형체 · ER/MR 유체 · 능동 제어

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
smart material hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- IEEE 176-1987 (Standard on Piezoelectricity)
- ASTM F2229-19 (Smart materials data exchange)
- ISO 23932 (Adaptive structures)
- IEC 60068 series (Environmental testing)
- MIL-STD-810H (Environmental engineering)
- JIS K 6253 (Hardness of vulcanized + thermoplastic rubber)
- ISO 13007 series (Ceramic tiles)
- ASME Y14.46 (Product definition for additive manufacturing)
- ISO 19840 (Coatings)

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. Piezoelectric Materials per IEEE 176
2. Magnetostrictive Materials
3. Electrorheological / Magnetorheological Fluids
4. Shape Memory Composites
5. Active Control Systems
6. Active Constrained Layer Damping
7. Photochromic and Electrochromic
8. Industrial Applications and Standards

## CLI

A POSIX shell helper is published at `cli/smart-material.sh`. The helper ships
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
