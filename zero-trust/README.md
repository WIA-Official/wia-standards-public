# WIA-SEC-ZT-001 — Zero Trust

> 제로 트러스트 표준 — 신뢰 0 검증 모델 · NIST SP 800-207 · CISA ZTMM · 정책 결정/집행

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
zero trust hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- NIST SP 800-207 (Zero Trust Architecture)
- NIST SP 800-207A (Multi-cloud)
- CISA Zero Trust Maturity Model v2.0
- DoD Zero Trust Reference Architecture v2.0
- NIST SP 800-204D (Service Mesh)
- NIST SP 800-53 Rev 5
- ISO/IEC 27001:2022
- ISO/IEC 27017 / 27018
- Cloud Security Alliance CCM v4

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. NIST SP 800-207 Foundations
2. Trust Algorithm Inputs
3. PEP Deployment Patterns
4. CISA ZTMM Pillars
5. Service Mesh per NIST SP 800-204D
6. Identity Federation OIDC/SAML
7. Continuous Authentication
8. Microsegmentation Patterns

## CLI

A POSIX shell helper is published at `cli/zero-trust.sh`. The helper ships
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
