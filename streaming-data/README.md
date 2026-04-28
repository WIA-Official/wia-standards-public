# WIA-DATA-001 — Streaming Data

> 스트리밍 데이터 표준 — 이벤트 스트리밍 · Kafka · Pulsar · Flink · Beam · CDC

## Scope

This standard documents the WIA Standards canonical envelope shape,
API surface, protocol exchanges, and ecosystem integration for
streaming data hosts that need to interoperate across
jurisdictions, vendors, and operational contexts. The standard
composes with the wider WIA Standards family to inherit cross-
standard composition, audit transport, and federation handshakes
without per-standard re-implementation.

## Normative references

- Apache Kafka KRaft + KIP-848
- Apache Pulsar 3.0 LTS
- Apache Flink 1.18 + FLIP-27 source
- Apache Beam 2.55 model
- CloudEvents Specification 1.0.2
- AsyncAPI Specification 3.0
- Schema Registry per Confluent + Apicurio
- Debezium 2.7 CDC connectors
- ISO 20022 (financial messaging — for BFSI streaming)

## Phase organisation

The standard is published in four Phase files:

- `spec/PHASE-1-DATA-FORMAT.md` (or PHASE-1-FOUNDATION.md) — wire-format envelopes
- `spec/PHASE-2-API.md` (or PHASE-2-DATA.md) — REST + JSON-RPC surface
- `spec/PHASE-3-PROTOCOL.md` — protocol exchanges and federation
- `spec/PHASE-4-INTEGRATION.md` — ecosystem composition and compliance

## eBook companion

A learner-focused 8-chapter eBook is published in both Korean and
English at `ebook/ko/` and `ebook/en/`:

1. Apache Kafka Architecture
2. Apache Pulsar Architecture
3. Stream Processing per Apache Flink
4. Apache Beam Unified Model
5. CloudEvents per CNCF
6. AsyncAPI Specification 3.0
7. Schema Evolution and Registry
8. Change Data Capture per Debezium

## CLI

A POSIX shell helper is published at `cli/streaming-data.sh`. The helper ships
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
