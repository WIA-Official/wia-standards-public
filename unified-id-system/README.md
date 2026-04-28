# WIA-ID-001 — Unified ID System

> 통합 ID 체계 표준 — 통합 신원 · OIDC · DID · 제로 트러스트 정합 · 라이프사이클

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
unified id system hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- OpenID Connect Core 1.0 + Discovery 1.0
- OAuth 2.1 (draft-ietf-oauth-v2-1)
- IETF RFC 9068 (OAuth JWT Profile)
- FIDO2 / WebAuthn per W3C
- SCIM 2.0 per RFC 7643 / 7644
- W3C Decentralized Identifiers (DID) 1.0
- W3C Verifiable Credentials Data Model 2.0
- ISO/IEC 24760 series (Identity management framework)
- NIST SP 800-63-3 (Digital Identity Guidelines)

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. OIDC and OAuth 2.1 Foundations
2. FIDO2 / WebAuthn Passkeys
3. SCIM 2.0 Lifecycle
4. Decentralized Identifiers per W3C DID 1.0
5. Verifiable Credentials per W3C VC 2.0
6. Identity Assurance per NIST 800-63-3
7. Privacy-Preserving Federation
8. Identity Management Framework ISO/IEC 24760

## CLI

A POSIX shell helper is published at `cli/unified-id-system.sh`. The helper ships
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
