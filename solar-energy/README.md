# WIA-ENE-014 — Solar Energy

> 태양광 에너지 표준 — 태양광·태양열 발전 · IEC 61215 · IEEE 1547 · NEC 690 · O&M

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
solar energy hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- IEC 61215 (PV module qualification)
- IEC 61730 (PV module safety)
- IEC 62446 (PV system documentation)
- IEC 61724-1:2021 (PV monitoring)
- IEEE 1547-2018 (DER interconnection)
- NEC 690 (NFPA 70 Article 690)
- UL 1741 SA / SB
- ISO 9806:2017 (Solar thermal collectors)
- ASHRAE 93-2010 (Solar collector testing)

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. PV Module Qualification IEC 61215
2. PV Safety per IEC 61730
3. Inverter Interconnection per IEEE 1547-2018
4. PV Performance Monitoring per IEC 61724-1
5. Solar Resource Assessment
6. Bifacial PV Energy Yield
7. PV O&M per IEC 62446
8. Solar Thermal per ISO 9806

## CLI

A POSIX shell helper is published at `cli/solar-energy.sh`. The helper ships
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
