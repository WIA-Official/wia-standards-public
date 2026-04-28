# WIA-MAT-SH-001 — Self Healing Material

> 자가 치유 물질 표준 — 자가 치유 물질 · 마이크로캡슐 · DA 반응 · 분자 동역학

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
self healing material hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- ASTM E2899-19 (Fracture toughness test)
- ASTM D5379 (Shear properties of composite materials)
- ASTM F3001-14 (Additive manufacturing materials)
- ISO 6721 series (Plastics - dynamic mechanical properties)
- ASTM D790 (Flexural Properties of Plastics)
- ASTM E647-15 (Fatigue crack growth rate)
- ISO 75 / ASTM D648 (HDT)
- BS EN ISO 527 (Tensile properties)
- Society for Composite Materials standards

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. Microcapsule-Based Self-Healing
2. Vascular Self-Healing Networks
3. Diels-Alder Reversible Polymers
4. Supramolecular Polymers
5. Shape-Memory Assisted Healing
6. Self-Healing Concrete
7. Mechanical Test Methods
8. Industrial Applications

## CLI

A POSIX shell helper is published at `cli/self-healing-material.sh`. The helper ships
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
