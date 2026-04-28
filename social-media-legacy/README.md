# WIA-SOC-001 — Social Media Legacy

> 소셜 미디어 레거시 표준 — 디지털 유산 · 사후 계정 · DSA · GDPR Art 17/20 · 디지털 추모

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
social media legacy hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- EU Digital Services Act (Reg 2022/2065)
- GDPR Art 17 (Right to erasure) + Art 20 (Portability)
- GDPR Art 89 (Archival in public interest)
- ePrivacy Directive 2002/58/EC
- Library of Congress Web Archive standards
- IPTC NewsML-G2 Section 5.5 archival
- ISO 14721:2012 OAIS Reference Model
- Estate-of-deceased per US Uniform Probate Code
- Korea Personal Information Protection Act (PIPA)

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. Digital Legacy Frameworks
2. EU DSA Posthumous Provisions
3. Right to Erasure GDPR Art 17
4. Data Portability per GDPR Art 20
5. Web Archive Standards
6. Memorialization Policies
7. OAIS Reference Model ISO 14721
8. National Probate Frameworks

## CLI

A POSIX shell helper is published at `cli/social-media-legacy.sh`. The helper ships
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
