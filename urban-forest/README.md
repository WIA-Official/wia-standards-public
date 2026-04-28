# WIA-ENV-013 — Urban Forest

> 도시숲 표준 — 도시 수림 · 캐노피 · 생태계 서비스 · 탄소 격리

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
urban forest hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- i-Tree Eco v6.0 (USDA Forest Service)
- ISO 14040:2006 / 14044:2006 (LCA)
- FAO Voluntary Guidelines on Urban and Peri-Urban Forestry
- UN-Habitat Global Public Space Programme
- ISA BMP (International Society of Arboriculture Best Management Practices)
- ANSI A300 (Tree Care Operations)
- ISO 14001:2015 EMS
- EU Biodiversity Strategy 2030

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. i-Tree Eco Methodology
2. Tree Inventory Standards
3. Canopy Cover Analysis
4. Ecosystem Services Valuation
5. Carbon Sequestration Accounting
6. Biodiversity Indicators
7. Tree Risk Management ISO 31000
8. Urban Heat Island Mitigation

## CLI

A POSIX shell helper is published at `cli/urban-forest.sh`. The helper ships
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
