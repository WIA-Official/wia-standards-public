# WIA-FIN-WEALTH-001 — Wealth Management

> 자산 관리 표준 — 자산 관리 · 포트폴리오 · MiFID II · 적합성 평가 · ESG 통합

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
wealth management hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- ISO 20022 (Financial messaging)
- ISO 22222 (Personal financial planning)
- EU MiFID II Directive 2014/65/EU + MiFIR Reg 600/2014
- EU PRIIPs Reg 1286/2014 + Delegated Reg 2017/653
- EU Sustainable Finance Disclosure Reg 2019/2088 (SFDR)
- EU Taxonomy Reg 2020/852
- FATF Rec 24 (Beneficial Ownership)
- Basel III/IV per BCBS Frameworks
- GIPS 2020 (Global Investment Performance Standards)

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. MiFID II Suitability Assessment
2. ISO 22222 Personal Financial Planning
3. Portfolio Construction MPT
4. PRIIPs KID Disclosure
5. Sustainable Finance per SFDR + EU Taxonomy
6. AML / KYC per FATF
7. Performance Reporting per GIPS 2020
8. Wealth-Tech Open Banking

## CLI

A POSIX shell helper is published at `cli/wealth-management.sh`. The helper ships
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
