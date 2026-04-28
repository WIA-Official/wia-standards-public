# WIA-AI-ASR-001 — Speech Recognition

> 음성 인식 표준 — 음성→텍스트 · ASR · 단대단 · 다국어 · 접근성

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
speech recognition hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- ISO/IEC 22989:2022 (AI concepts)
- ISO/IEC 23053:2022 (AI ML framework)
- W3C Voice Browser Working Group SRGS 1.0
- W3C Speech Recognition API draft
- ITU-T P.863 POLQA
- IEEE 743 (transmission quality measurements)
- BCP 47 / RFC 5646 language tags
- EU AI Act (Reg 2024/1689)
- WCAG 2.2 Section 1.2.6 sign-language

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. Acoustic Model Fundamentals
2. Language Modeling per Transformer
3. End-to-End Models
4. Speech Recognition API Web Standards
5. Multilingual ASR
6. Speech Quality Metrics ITU-T P.863
7. Privacy and On-Device
8. Accessibility per WCAG and EU EAA

## CLI

A POSIX shell helper is published at `cli/speech-recognition.sh`. The helper ships
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
