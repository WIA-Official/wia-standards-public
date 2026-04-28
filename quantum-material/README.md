# WIA-QM-001 — Quantum Material

> 양자 물질 표준 — 양자 물질 · 위상 절연체 · 초전도체 · 고체 NMR · STM

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
quantum material hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- ASTM F1372-93 (Sheet Resistance Sheet Carriers)
- ASTM E1448-19 (Hall mobility)
- ISO 23034:2024 (Nanotechnology)
- ISO/IEC 80004-13:2017 (Nano-objects)
- IEC 62607 series (Nanomanufacturing characterisation)
- IUPAC Solid State Chemistry recommendations
- Crystallographic Information File (CIF) per IUCr
- ICSD Inorganic Crystal Structure Database
- Materials Project database citation guidelines

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. Topological Insulators
2. Superconductivity Foundations
3. High-Tc Cuprates
4. Iron-Based Superconductors
5. Topological Superconductors
6. Two-Dimensional Materials
7. Crystallographic Characterization
8. First-Principles DFT

## CLI

A POSIX shell helper is published at `cli/quantum-material.sh`. The helper ships
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
