# WIA-RUST-LEARN-PRIMER — Rust Learn

> Rust 학습 표준 — Rust 언어 학습 · 소유권 · 라이프타임 · 동시성 · 표준 라이브러리

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
rust learn hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- Rust Reference (rust-lang.github.io/reference)
- Rust Edition 2024
- ISO/IEC 9899:2018 (C standard for FFI alignment)
- Cargo Book (doc.rust-lang.org/cargo)
- Rustlings Exercise Pack 5.6
- Rust by Example
- The Rust Programming Language Book 3rd Edition (Klabnik + Nichols 2023)
- Rust Reference Library
- IEEE 754-2019 (Floating-point arithmetic)

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. Ownership Borrowing Lifetimes
2. Type System and Traits
3. Collections and Iterators
4. Error Handling
5. Modules Crates Cargo
6. Testing and Documentation
7. Concurrency Patterns
8. Async Runtimes

## CLI

A POSIX shell helper is published at `cli/rust-learn.sh`. The helper ships
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
