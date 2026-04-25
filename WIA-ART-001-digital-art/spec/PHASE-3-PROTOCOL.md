# WIA-ART-001: Phase 3 - Protocol Specification

**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 3 specifies the wire-level protocols, signed-manifest container, transport protections, and real-time collaboration protocol used by WIA-ART-001 implementations. The Phase 3 design composes well-established IETF and W3C primitives rather than redefining them, so that conformant implementations can rely on broadly available libraries and tooling.

---

## 2. Signed Manifest Container

The canonical signed manifest follows COSE per RFC 9052 with CBOR encoding per RFC 8949. JOSE alternatives (JWS per RFC 7515) are recognised for clients that prefer JSON encoding.

The manifest carries:

- A reference to the artwork bytes (URI plus content-addressed hash, typically SHA-256 per FIPS 180-4).
- The metadata block (authors, creation time, rights, language) as defined in Phase 2.
- Provenance assertions following W3C PROV-O.
- One or more signatures, each with explicit algorithm identifier (RFC 8032 EdDSA, NIST FIPS 186-5 ECDSA, RFC 8017 RSA-PSS).
- Optional creator-attestation assertions following W3C VC Data Model 2.0.
- Optional C2PA-compatible assertions for cross-standard interoperability.

### 2.1 Canonical Hash

The artwork content hash is computed over the artwork bytes after canonicalisation:

- Raster images: PNG re-encoded with deflate level 9 and sRGB chunk normalised (PNG canonicalisation profile per ISO 15948 considerations).
- Vector graphics: SVG normalised per the W3C SVG canonicalisation guidance.
- 3D models: glTF 2.0 with deterministic JSON ordering and embedded buffers in canonical Base64.
- Video: original bytes (no canonicalisation).
- Audio: original bytes (no canonicalisation).

Hash algorithm: SHA-256 by default; SHA3-256 acceptable when the deploying jurisdiction or organisation requires it.

---

## 3. Data Exchange Protocol

### 3.1 Message Format

JSON message envelope:

```json
{
  "version": "1.0",
  "type": "wia-art-001",
  "timestamp": "2026-04-26T08:14:32Z",
  "payload": { },
  "manifest_uri": "https://cdn.example.com/art/.../manifest.cose",
  "signatures": [
    {
      "alg": "EdDSA",
      "kid": "did:web:tanaka.art#key-1",
      "sig": "base64..."
    }
  ]
}
```

Timestamps follow ISO 8601:2019. The signature covers the canonicalised payload and the manifest_uri.

### 3.2 Authentication

Authentication of clients follows Phase 2 §4. Authentication of artwork content follows §2 above.

### 3.3 Rate Limiting

Rate limits are enforced at the API gateway and exposed via the IETF rate-limit-headers conventions. The reference defaults are:

- 1,000 requests per hour per client for the free tier.
- 10,000 requests per hour per client for the pro tier.
- 100,000 requests per hour for enterprise deployments with an SLA.

These defaults are deployment policy and may be overridden in operating organisations' published terms.

---

## 4. Transport Security

| Layer | Reference | Notes |
|-------|-----------|-------|
| TCP transport | RFC 9293 | TCP standard |
| TLS | RFC 8446 (1.3); RFC 5246 (1.2 minimum) | Required for all wire traffic |
| Certificate | RFC 5280 (X.509 v3) | Required SAN with DNS or URI for service identity |
| HTTP semantics | RFC 9110 | Request/response semantics |
| HTTP/2 | RFC 9113 | Required for browser-side clients |
| HTTP/3 over QUIC | RFC 9114 / RFC 9000 | Optional, recommended for mobile |
| WebSocket | RFC 6455 | Real-time updates |
| Trace context | W3C Trace Context | Distributed tracing |

Certificate pinning is recommended for studio devices and trusted partners; OCSP stapling per RFC 6066 §8 is mandatory for all public-facing endpoints.

---

## 5. Real-Time Collaboration Protocol

### 5.1 Operational Transformation Stream

For collaborative editing, the protocol uses a CRDT (Conflict-free Replicated Data Type) over a WebSocket channel. Operations are:

```json
{
  "session_id": "sess_123",
  "client_id": "client_abc",
  "lamport_clock": 12345,
  "op": "stroke",
  "args": { "tool": "brush", "color_oklch": [0.7, 0.15, 240], "path": "[...]" },
  "predecessor": "..."
}
```

CRDT state is canonicalised at each merge boundary and hashed for verification at session checkpoints.

### 5.2 Session Identity

Each collaborative session has a stable identifier. Participants are identified by their DID (W3C DID Core 1.0) when authenticated; pseudonymous participants are identified by an ephemeral session-scoped identifier.

### 5.3 Conflict Resolution

Conflict resolution uses a deterministic merge based on Lamport clocks and tie-breaking by participant DID. The resolution algorithm is publicly documented so that any client implementing the algorithm produces identical results from identical input streams.

---

## 6. Subscription and Event Stream Protocol

For one-way streams (e.g., a marketplace subscribing to new-artwork events), Server-Sent Events per the W3C EventSource specification or WebSocket per RFC 6455 are acceptable. Event payloads are JSON envelopes as described in §3.1, and the event stream itself is rate-limited.

---

## 7. Reference Standards Alignment

| Concern | Reference |
|---------|-----------|
| Signed-manifest container | RFC 9052 (COSE), RFC 8949 (CBOR), RFC 7515 (JWS) |
| Hash | FIPS 180-4 (SHA-2), FIPS 202 (SHA-3) |
| Signing | RFC 8032 (EdDSA), NIST FIPS 186-5 (ECDSA), RFC 8017 (RSA-PSS) |
| Identity | W3C DID Core 1.0, W3C VC Data Model 2.0 |
| Provenance | W3C PROV-O |
| Catalog | W3C DCAT v3 |
| Discovery markup | schema.org VisualArtwork / CreativeWork |
| Image canonicalisation | ISO 15948 (PNG) considerations |
| 3D content | Khronos glTF 2.0 |
| Video | ISO/IEC 14496-10 (AVC), ISO/IEC 23008-2 (HEVC), AOMedia AV1 |
| Audio | ISO/IEC 14496-3 (AAC), RFC 6716 (Opus), ISO/IEC 11172-3 (MP3) |
| Transport | RFC 9293 (TCP), RFC 8446 (TLS 1.3), RFC 9110 (HTTP), RFC 9113 (HTTP/2), RFC 9114 (HTTP/3) |
| Real-time | RFC 6455 (WebSocket), W3C EventSource (SSE) |
| Rate-limit headers | IETF rate-limit-headers conventions |
| Locale | BCP 47 (RFC 5646), Unicode CLDR |
| Time | ISO 8601:2019 |

All references conform to the WIA Citation & Veracity Policy v1.0 §2.1 ALLOW.

---

## 8. Conformance

A Phase 3 implementation is conformant when:

1. Signed manifests are emitted in COSE/CBOR or JWS/JSON with explicit algorithm identifiers.
2. Hash and signature algorithms match the §7 references and are declared in the manifest.
3. TLS 1.3 (or 1.2 minimum) is supported on all wire endpoints.
4. Real-time collaboration uses the documented CRDT protocol with deterministic conflict resolution.
5. Event streams use WebSocket or Server-Sent Events with the §3.1 envelope.

---

## 9. Implementation Appendix

### 9.1 CRDT details

The reference CRDT is a sequence-CRDT extended with attribute layers. Each operation carries:

- A globally unique operation identifier (Lamport clock × participant DID).
- A reference to the predecessor operation (for causality preservation).
- The operation type and its arguments.
- A signature over the canonicalised operation by the participant.

The merge function preserves causality, deterministic ordering on concurrent edits, and signature integrity. State is checkpointed at session boundaries; checkpoints are signed and become part of the artwork's provenance graph.

### 9.2 Replay and reconstruction

Any conformant client can reconstruct the artwork state from the operation log alone. Reconstruction is bit-identical across implementations, which makes the operation log itself a verifiable provenance artefact: the hash of the canonicalised operation log is a natural identifier for the collaborative session.

### 9.3 Network resilience

The protocol assumes unreliable networks. Key resilience properties:

- **Out-of-order delivery** — operations are merged by causality, not arrival order.
- **Duplicate delivery** — operation identifiers make duplicates trivially detectable.
- **Disconnection** — clients buffer operations locally and reconcile on reconnect.
- **Partition** — partitioned clients continue editing locally; merge resolves on rejoin.

### 9.4 Privacy in collaborative sessions

Participants who are minors in jurisdictions with online-safety rules (UK Online Safety Act 2023, EU DSA, COPPA, etc.) trigger the privacy-by-default settings of WIA-CHILD-001 even within a collaborative session. The collaborative session provider is responsible for honouring those defaults; non-honouring is a conformance failure.

### 9.5 Bandwidth shaping

For low-bandwidth links, operations are batched and delta-compressed. The reference compression uses Zstandard (RFC 8478) at level 9 over UTF-8 JSON. CBOR encoding (RFC 8949) is an acceptable alternative when client and server negotiate it.

### 9.6 Trace context propagation

Every protocol message carries a W3C Trace Context `traceparent` so that distributed traces span the client, server, and downstream services. Traces include the operation identifier so an individual collaborative edit can be located in the trace store.

### 9.7 Test suite

The Phase 3 conformance test suite covers:

- Manifest signing and verification with each supported algorithm family.
- Cross-implementation manifest interoperability (round-trip through COSE/CBOR and JOSE/JSON).
- CRDT merge correctness on synthetic concurrent-edit scenarios.
- TLS handshake compatibility against the reference test endpoints.
- Rate-limit-header compliance.

The test suite is published as an open package; any implementation can self-assess prior to a formal review.

### 9.8 Cryptographic Agility

Implementations support multiple algorithm families simultaneously and select per-manifest based on the deploying organisation's cryptographic policy. The reference policy:

- **Default** — Ed25519 signatures (RFC 8032) and SHA-256 hashes (FIPS 180-4).
- **Legacy hardware** — ECDSA P-256 (NIST FIPS 186-5) and SHA-256.
- **High-assurance** — SHA3-512 (FIPS 202) and Ed25519, with hybrid post-quantum signatures (FIPS 204 ML-DSA) added once the implementation has completed integration testing.

Algorithm selection is recorded in the manifest, and verifiers MUST refuse to silently substitute algorithms.

### 9.9 Public-Key Resolution

Public keys are resolved through DIDs (W3C DID Core 1.0) by default. The reference resolution path uses did:web for organisations with public DNS infrastructure and did:key for self-certifying keys. Cached public keys carry their freshness indicator so verifiers know when to refresh.

### 9.10 Long-Term Verifiability

Manifests signed today must verify decades from now. The combination of explicit algorithm identifiers (so the verifier knows what the signer used), open hash and signature primitives with public test vectors, and content addressing of the artwork bytes, makes long-term verification robust against changes in the verifier's runtime environment.

---

**弘益人間 (Benefit All Humanity)**
*© 2025 WIA*
