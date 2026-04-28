# WIA-CITY-001 — Smart City

> 스마트 시티 표준 — 도시 데이터 플랫폼 · 거버넌스 · KPI · 시민 참여

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
smart city hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- ISO 37120:2018 (City indicators)
- ISO 37122:2019 (Smart city indicators)
- ISO 37123:2019 (Resilient cities indicators)
- ITU-T Y.4900 series
- ISO/IEC 30141:2018 (IoT Reference Architecture)
- ISO/IEC 21972:2020 (Linked-data context for smart-city KPIs)
- ETSI TS 103 463 (Key Performance Indicators)
- UN-Habitat Global Urban Monitoring Framework

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. ISO 37120 City Indicators
2. ISO 37122 Smart City Indicators
3. ISO/IEC 30141 IoT Reference Architecture
4. ITU-T Y.4900 KPI Framework
5. Linked Data per ISO/IEC 21972
6. Urban Resilience per ISO 37123
7. Citizen Participation Platforms
8. Smart-City Cybersecurity

## CLI

A POSIX shell helper is published at `cli/smart-city.sh`. The helper ships
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
