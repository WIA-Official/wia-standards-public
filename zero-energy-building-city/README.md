# WIA-ENE-ZEC-001 — Zero Energy Building and City

> 제로에너지 빌딩·도시 표준 — ZEB·ZEC · ASHRAE 90.1 · IEEE 1547 · 그리드 상호작용

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
zero energy building and city hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- ASHRAE 90.1-2022
- ASHRAE 189.1-2020
- ASHRAE 169-2021 Climate zones
- IEEE 1547-2018 (DER interconnection)
- IEC 61850-7-420 (DER comm)
- OpenADR 3.0
- ISO 52000-1:2017 (EPB framework)
- ISO 52016-1:2017 (Energy needs)
- EU EPBD recast Directive (EU) 2024/1275

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
3. EPB per ISO 52000 series
4. DER Interconnection per IEEE 1547-2018
5. Grid-Interactive Buildings per DOE GEB
6. OpenADR 3.0 Demand Response
7. Net-Zero City Frameworks
8. EU EPBD Recast Directive

## CLI

A POSIX shell helper is published at `cli/zero-energy-building-city.sh`. The helper ships
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
