# WIA-ENE-041 — Rare Earth Mining

> 희토류 채굴 표준 — 희토류 자원 · 광산 안전 · 환경 영향 평가 · 분리 정제

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
rare earth mining hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- ISO 14001:2015 EMS
- ISO 14040:2006 / 14044:2006 LCA
- ICMM Mining Principles 2024
- IRMA Standard for Responsible Mining v2.0 2024
- Towards Sustainable Mining (TSM) Protocol
- Equator Principles 4 (2020)
- OECD Due Diligence for Conflict-Free Mineral Supply Chains
- EU Critical Raw Materials Act (Reg 2024/1252)
- USGS Mineral Commodity Summaries 2024

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. Rare Earth Element Geology
2. Mining Methods Selection
3. Hydrometallurgical Processing
4. Tailings Management
5. Environmental Impact Assessment
6. Radiological Protection
7. Responsible Sourcing per OECD
8. EU CRMA Compliance

## CLI

A POSIX shell helper is published at `cli/rare-earth-mining.sh`. The helper ships
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
