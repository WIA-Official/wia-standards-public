# WIA-ENE-026 — Radioactive Waste Management

> 방사성 폐기물 관리 표준 — 방사성 폐기물 분류·저장·심지층 처분 · IAEA · 사용후핵연료

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
radioactive waste management hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- IAEA GSG-1 (Classification of Radioactive Waste)
- IAEA SSR-5 (Disposal of Radioactive Waste)
- IAEA SSG-23 (Safety Case for Geological Disposal)
- IAEA Joint Convention on Spent Fuel and Radioactive Waste Safety
- IAEA SSR-6 (Transport of Radioactive Material)
- ICRP Publication 122 (Radiological Protection)
- NRC 10 CFR Part 60 / 61 / 63
- EU Council Directive 2011/70/Euratom
- OECD/NEA Forum on Stakeholder Confidence

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. IAEA GSG-1 Classification
2. Spent Fuel Management
3. Geological Disposal Safety Case
4. Transport of Radioactive Materials
5. Decommissioning per IAEA WS-G-2.1
6. Stakeholder Engagement OECD/NEA
7. Radiological Protection ICRP 103/122
8. Long-Term Stewardship

## CLI

A POSIX shell helper is published at `cli/radioactive-waste.sh`. The helper ships
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
