# WIA-EDU-019 — STEM Education

> STEM 교육 표준 — 과학·기술·공학·수학 교육 · NGSS · CSTA · 평가 프레임

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
stem education hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- Next Generation Science Standards (NGSS)
- CSTA K-12 Computer Science Standards 2017
- ISTE Standards for Students 2024
- NCTM Principles to Actions
- OECD PISA 2025 Framework
- TIMSS 2023 Assessment Framework
- ABET Engineering Accreditation Criteria 2024-25
- ITEEA Standards for Technological and Engineering Literacy (STEL)
- UNESCO STEM and Gender Advancement (SAGA)

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. NGSS Three-Dimensional Learning
2. CSTA K-12 Computer Science
3. ISTE Standards 2024 for Students
4. NCTM Mathematical Proficiency
5. PISA 2025 Scientific Literacy
6. Engineering Design Process
7. STEM Equity per UNESCO SAGA
8. Assessment per ABET / TIMSS

## CLI

A POSIX shell helper is published at `cli/stem-education.sh`. The helper ships
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
