# WIA-EDU-006 — Virtual Classroom

> 가상 교실 표준 — 원격 교육 · LMS · LRS · LTI · WebRTC · 접근성

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
virtual classroom hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- IMS Global LTI Advantage 1.3
- IEEE 9274.1.1 xAPI
- ADL cmi5 v1.0
- SCORM 2004 4th Edition
- WCAG 2.2 AA
- ISO/IEC 19796-1 Quality management
- EU Web Accessibility Directive 2016/2102
- ADA Title II / III
- IETF RFC 8866 SDP / WebRTC

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. LTI Advantage 1.3 Foundations
2. xAPI / cmi5 Learning Records
3. WebRTC Real-Time Classrooms
4. Accessibility per WCAG 2.2 AA
5. LMS Integration via LTI 1.3
6. Proctoring and Assessment Integrity
7. Privacy per FERPA + GDPR Art 8
8. Multi-Modal Learning Telemetry

## CLI

A POSIX shell helper is published at `cli/virtual-classroom.sh`. The helper ships
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
