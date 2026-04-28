# WIA-ENV-015 — Soil Restoration

> 토양 복원 표준 — 토양 정화 · 오염 평가 · 자연 기반 해결

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
soil restoration hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- ISO 11074:2015 (Soil quality vocabulary)
- ISO 17025 (Lab competence)
- ISO/TC 190 series
- USDA Soil Taxonomy (12th Ed)
- EPA Method 9080 / 8260D / 8270E
- EU Soil Strategy 2030
- EU Soil Monitoring Law (2023 proposal COM(2023) 416)
- Codex Alimentarius CXC 79-2019
- ASTM E1739 (Risk-Based Corrective Action)

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. ISO 11074 Soil Quality Vocabulary
2. Site Investigation per ISO 18400
3. Contaminant Analysis per EPA Methods
4. Risk Assessment per ASTM E1739
5. Bioremediation Technologies
6. Phytoremediation Pathways
7. Heavy-Metal Stabilisation
8. EU Soil Strategy 2030 Compliance

## CLI

A POSIX shell helper is published at `cli/soil-restoration.sh`. The helper ships
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
