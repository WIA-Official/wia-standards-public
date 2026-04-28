# WIA-ENE-031 — Zero Energy Building

> 제로에너지 건물 표준 — 제로에너지 빌딩 · 건물 단위 ZEB · K-ZEB · ASHRAE · LEED

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
zero energy building hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- ASHRAE 90.1-2022
- ASHRAE 189.1-2020 (High-Performance Green Buildings)
- ASHRAE 169-2021 (Climate Data)
- EU EPBD Recast Directive (EU) 2024/1275
- Korea Green Building Construction Support Act
- PHIUS Passive Building Standard 2021
- Living Building Challenge 4.0
- ISO 52000 series Energy Performance of Buildings
- DOE Common Definition of ZEB

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. ZEB Definitions per DOE / IEA SHC
2. ASHRAE 90.1-2022 Compliance
3. Passive House per PHIUS 2021
4. Korea K-ZEB Certification
5. EU EPBD Recast 2024 ZEB
6. Net-Zero Carbon Building
7. Renewable Energy Integration
8. Living Building Challenge 4.0

## CLI

A POSIX shell helper is published at `cli/zero-energy-building.sh`. The helper ships
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
