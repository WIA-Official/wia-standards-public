# WIA-MAT-SM-001 — Shape Memory Material

> 형상 기억 물질 표준 — 형상 기억 합금 · NiTi · 형상 기억 폴리머 · 의료 디바이스

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
shape memory material hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- ASTM F2004-17 (NiTi Transformation Temperature DSC)
- ASTM F2005-21 (Standard Terminology for NiTi)
- ASTM F2063-18 (NiTi Rod Wire Tube Strip Bar)
- ASTM F2516-22 (Tension Testing of NiTi Alloy)
- ISO 14630 (Implants for surgery)
- ISO 10993 series (Biocompatibility)
- ASTM E384 (Microhardness)
- ISO 6892-1 (Metallic materials Tensile testing)
- ASTM F2828 (Cantilever bend NiTi)

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. Shape Memory Alloy Foundations
2. NiTi (Nitinol) Properties
3. Material Characterization ASTM F2004
4. Mechanical Testing ASTM F2516
5. Shape Memory Polymers (SMP)
6. Medical Device Applications
7. Aerospace Actuator Applications
8. Biocompatibility per ISO 10993

## CLI

A POSIX shell helper is published at `cli/shape-memory-material.sh`. The helper ships
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
