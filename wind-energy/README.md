# WIA-ENE-015 — Wind Energy

> 풍력 에너지 표준 — 풍력 발전 · IEC 61400 · 그리드 코드 · 해상풍력 · 보조계통

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
wind energy hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- IEC 61400-1:2019 (Design requirements)
- IEC 61400-3-1:2019 (Offshore design)
- IEC 61400-12-1:2022 (Power performance)
- IEC 61400-21-1:2019 (Power quality)
- IEC 61400-25 series (Communications)
- IEEE 1547-2018 (DER interconnection)
- NERC PRC-024-3 (Frequency-voltage ride through)
- DNV-ST-0145 / DNV-OS-J101 offshore
- ISO 81400-4 (Gearboxes)

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. Wind Turbine Design IEC 61400-1
2. Offshore Wind per IEC 61400-3-1
3. Wind Resource Assessment
4. Power Performance per IEC 61400-12-1
5. Power Quality per IEC 61400-21-1
6. Communications IEC 61400-25
7. Gearbox Reliability ISO 81400-4
8. Offshore Foundation Design

## CLI

A POSIX shell helper is published at `cli/wind-energy.sh`. The helper ships
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
