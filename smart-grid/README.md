# WIA-ENE-027 — Smart Grid

> 스마트 그리드 표준 — 스마트 그리드 · IEC 61850 · DERMS · AMI · 사이버 보안

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
smart grid hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- IEC 61850 series (Communication networks)
- IEC 62351 series (Power systems security)
- IEEE 2030 series (Smart Grid Interoperability)
- IEEE 1547-2018 (DER interconnection)
- IEC 60870-5-104 (SCADA)
- DNP3 per IEEE 1815-2012
- NERC CIP-002-9 / 003-9 / 005-7 / 007-6 / 010-4
- FERC Order 2222 (DER aggregation)
- OpenADR 3.0

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. IEC 61850 Communication Architecture
2. Power System Security IEC 62351
3. DER Interconnection IEEE 1547-2018
4. FERC Order 2222 DER Aggregation
5. Advanced Metering Infrastructure
6. Wide-Area Monitoring (WAMS)
7. NERC CIP Bulk Electric System
8. OpenADR 3.0 Demand Response

## CLI

A POSIX shell helper is published at `cli/smart-grid.sh`. The helper ships
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
