# WIA-TLS-LITE — TLS Lite

> TLS Lite 표준 — 경량 TLS · IoT · 임베디드 · TLS 1.3 부분집합 · DTLS

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
tls lite hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- IETF RFC 8446 (TLS 1.3)
- IETF RFC 9147 (DTLS 1.3)
- IETF RFC 7250 (Raw Public Keys)
- IETF RFC 8392 (CWT — CBOR Web Token)
- IETF RFC 7252 (CoAP)
- IETF RFC 8030 (Constrained Headers)
- NIST FIPS 140-3
- ISO/IEC 19772:2020 (Authenticated encryption)
- ISO/IEC 18033-7 (Tweakable block ciphers)

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. TLS 1.3 Subset Profile
2. DTLS 1.3 over CoAP
3. Raw Public Key Authentication
4. CWT and CBOR Envelopes
5. Constrained Device Crypto
6. FIPS 140-3 Validation Path
7. Side-Channel Resistance
8. OTA and Trust List Bootstrap

## CLI

A POSIX shell helper is published at `cli/tls-lite.sh`. The helper ships
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
