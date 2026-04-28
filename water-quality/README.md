# WIA-ENV-017 — Water Quality

> 수질 표준 — 수질 모니터링 · WHO·EPA 음용수 · IoT · 수처리 · WHO GDWQ

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
water quality hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- WHO Guidelines for Drinking-water Quality 4th Edition + 1st Addendum 2017
- US EPA National Primary Drinking Water Regulations 40 CFR 141
- ISO 5667 series (Water sampling)
- ISO 7027 (Turbidity)
- EPA Method 200.7 / 200.8 (ICP / ICP-MS)
- EPA Method 524.2 / 525.2 (VOC / SVOC)
- Codex Alimentarius CXS 108-1981 (Bottled water)
- ISO 17025 (Lab competence)
- USGS National Field Manual

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. WHO GDWQ Framework
2. EPA Primary Drinking Water Regulations
3. Water Sampling per ISO 5667 series
4. Microbial Indicators
5. Chemical Analysis ICP / ICP-MS
6. VOC / SVOC Analysis EPA 524.2 / 525.2
7. Sensor Network IoT Monitoring
8. Treatment Technologies and Compliance

## CLI

A POSIX shell helper is published at `cli/water-quality.sh`. The helper ships
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
