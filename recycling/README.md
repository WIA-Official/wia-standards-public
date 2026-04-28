# WIA-ENE-023 — Recycling

> 재활용 표준 — 순환경제 · 자원회수 · EPR · 분리수거 · 매립 회피

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
recycling hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- ISO 14040:2006 / 14044:2006 (LCA)
- ISO 14021:2016 (Self-declared environmental claims)
- ISO 17088:2021 (Compostable plastics)
- EN 13432:2000 (Compostability)
- EU Waste Framework Directive 2008/98/EC
- EU Single-Use Plastics Directive 2019/904
- Basel Convention (1989)
- ISO 16221:2001 (Marine biodegradability)
- Ellen MacArthur Foundation Circular Economy Framework

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. Waste Hierarchy per EU 2008/98/EC
2. Material Recovery Pathways
3. EPR Schemes Per Stream
4. Composting Standards
5. Plastics Identification Codes
6. Circular Economy per EMF
7. Material Flow Accounting
8. Marine Litter and SUP Directive

## CLI

A POSIX shell helper is published at `cli/recycling.sh`. The helper ships
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
