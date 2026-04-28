# WIA-GOV-001 — Smart Government

> 스마트 거버넌스 표준 — 디지털 정부 · GovTech · eIDAS · 시민 디지털 신원 · 개방데이터

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
smart government hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- ISO/IEC 38505 (Governance of data)
- ISO/IEC 27001:2022
- EU eIDAS Regulation (EU) 910/2014 + Reg 2024/1183 (eIDAS 2)
- UN E-Government Survey 2024
- World Bank GovTech Maturity Index 2024
- OECD Recommendation on Digital Government 2014
- Open Government Partnership (OGP) Action Plans
- W3C DCAT 3.0 + DCAT-AP 3.0
- OpenAPI Specification 3.1

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. Digital Government Maturity
2. eIDAS 2.0 EU Digital Identity Wallet
3. Open Data per W3C DCAT
4. API Government per OpenAPI 3.1
5. Citizen Single Sign-On
6. Public Procurement Standardisation
7. Smart City Government Composability
8. Digital Public Infrastructure

## CLI

A POSIX shell helper is published at `cli/smart-government.sh`. The helper ships
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
