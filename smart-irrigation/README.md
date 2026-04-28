# WIA-AGRI-IRR-001 — Smart Irrigation

> 스마트 관개 표준 — 정밀 관개 · ISOBUS · FAO 56 ETo · 물 사용 효율 · IoT

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
smart irrigation hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- ISO 11783 ISOBUS series
- FAO Irrigation and Drainage Paper 56 (ETo)
- ASABE EP505 (Microirrigation Filtration)
- ISO 24566-1:2024 (Irrigation system performance)
- IEC 61512 series (Batch control)
- EU Water Framework Directive 2000/60/EC
- USDA NRCS Conservation Practice Standard 442
- OGC SensorThings API 1.1
- ASABE EP527 (Sprinkler Irrigation)

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. ISOBUS per ISO 11783
2. Reference Evapotranspiration FAO 56
3. Drip Irrigation Design
4. Sprinkler Irrigation per ASABE EP527
5. Soil Moisture Sensors
6. Variable-Rate Irrigation
7. Water Quality Management
8. Smart Controllers and IoT

## CLI

A POSIX shell helper is published at `cli/smart-irrigation.sh`. The helper ships
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
