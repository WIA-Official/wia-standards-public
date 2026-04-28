# WIA-HEALTH-SLEEP — WIA Sleep

> WIA 수면 표준 — 수면 모니터링 · 폴리솜노그래피 · 웨어러블 · AASM 스코어링

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
wia sleep hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- AASM Manual for the Scoring of Sleep + Associated Events v3.0 (2024)
- ICSD-3-TR International Classification of Sleep Disorders 3rd Ed
- ASTM F3038-21 (Wearable sleep tracker accuracy)
- ISO 27017 / 27018 (Cloud security + privacy)
- ISO 13485 (Medical devices QMS)
- FDA 21 CFR 880 + 882 (Medical devices)
- EU MDR 2017/745
- HIPAA Privacy Rule per 45 CFR 164.500-534
- GDPR Art 9 special category data

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. AASM Sleep Scoring v3.0 (2024)
2. Polysomnography (PSG)
3. Sleep Disorders ICSD-3-TR
4. Wearable Sleep Trackers
5. Sleep Apnea Detection
6. Circadian Rhythm Monitoring
7. Privacy and Data Governance
8. Clinical Validation per FDA 21 CFR 880

## CLI

A POSIX shell helper is published at `cli/wia-sleep.sh`. The helper ships
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
