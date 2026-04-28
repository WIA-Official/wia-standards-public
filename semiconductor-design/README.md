# WIA-SEMI-DESIGN-001 — Semiconductor Design

> 반도체 설계 표준 — 반도체 회로 설계 · IP 코어 · DRC/LVS · 검증 · 테이프아웃

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
semiconductor design hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- IEEE 1801-2024 (UPF Power Intent)
- IEEE 1685-2022 (IP-XACT)
- Accellera SystemVerilog 2017 + UVM 1.2
- IEC 61508-3 (Functional safety)
- ISO 26262-11 (Hardware in road vehicles)
- IEEE P2851 (Functional Safety Reference)
- ITU-T X.1054 (Information security gov)
- IEC 62443-4-1 (Secure product development)
- OASIS SBOM CycloneDX 1.6

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. RTL Design Methodology
2. Verification per UVM 1.2
3. Static Timing Analysis
4. Power Intent per IEEE 1801 UPF
5. DFT and ATPG
6. Physical Design DRC/LVS
7. Functional Safety per ISO 26262
8. Hardware Trojan and Supply-Chain Hardening

## CLI

A POSIX shell helper is published at `cli/semiconductor-design.sh`. The helper ships
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
