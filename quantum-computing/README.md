# WIA-QC-001 — Quantum Computing

> 양자 컴퓨팅 표준 — 양자 컴퓨팅 · 큐비트 · 양자 회로 · NISQ · 오류 정정

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
quantum computing hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- IEEE 1671 series (test interoperability)
- ISO/IEC AWI 4879 (Quantum computing terminology)
- OpenQASM 3.0
- QIR (Quantum Intermediate Representation) Alliance Spec
- Cirq + Qiskit + PennyLane runtime APIs
- NIST PQC Round 4 (CRYSTALS-Kyber/Dilithium/SPHINCS+/Falcon)
- ETSI TS 103 619 (Quantum-safe key exchange)
- GSMA Post-Quantum Cryptography migration guidelines
- ITU-T Y.3800 series (Quantum Key Distribution)

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. Qubit Foundations
2. Universal Gate Sets
3. OpenQASM 3.0 Programming
4. NISQ Algorithms
5. Quantum Error Correction
6. Quantum Supremacy and Advantage
7. Quantum Chemistry Applications
8. Post-Quantum Cryptography Migration

## CLI

A POSIX shell helper is published at `cli/quantum-computing.sh`. The helper ships
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
