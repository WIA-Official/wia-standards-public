# WIA-ENV-014 — Wetland Conservation

> 습지 보전 표준 — 람사르 · 습지 평가 · 생태계 복원 · 모니터링

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
wetland conservation hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- Ramsar Convention (1971)
- Ramsar Strategic Plan 2016-2024
- Ramsar Resolution VIII.6 / IX.1
- ISO 14040 / 14044 (LCA)
- Convention on Biological Diversity (CBD)
- Aichi Biodiversity Targets
- Kunming-Montreal Global Biodiversity Framework (2022)
- EU Habitats Directive 92/43/EEC
- US Clean Water Act §404

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. Ramsar Convention Framework
2. Wetland Classification Systems
3. Wetland Ecological Assessment
4. Restoration Methodology
5. Carbon Sequestration in Wetlands
6. Ramsar Wise-Use Implementation
7. Biodiversity Monitoring CBD
8. Climate Adaptation for Wetlands

## CLI

A POSIX shell helper is published at `cli/wetland-conservation.sh`. The helper ships
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
