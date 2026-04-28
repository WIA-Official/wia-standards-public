# WIA-AI-RECO-001 — Recommendation AI

> 추천 AI 표준 — 추천 시스템 · 협업 필터링 · 콘텐츠 기반 · 다중목적 · 공정성

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
recommendation ai hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- ISO/IEC 22989:2022 (AI concepts)
- ISO/IEC 23053:2022 (AI ML framework)
- ISO/IEC 24029-1/2 (Robustness)
- ISO/IEC 42001:2023 (AI management system)
- EU AI Act (Reg 2024/1689)
- NIST AI Risk Management Framework 1.0
- GDPR Art 22 (Automated decision-making)
- OECD AI Principles
- ACM RecSys Best Practices 2023

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. Collaborative Filtering Foundations
2. Content-Based Approaches
3. Deep Learning RecSys
4. Multi-Objective Optimisation
5. Fairness and Bias Auditing
6. Explainability per ISO/IEC 42001
7. Privacy Preserving Recsys
8. Online Evaluation A/B Testing

## CLI

A POSIX shell helper is published at `cli/recommendation-ai.sh`. The helper ships
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
