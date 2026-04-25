# WIA-CITY-017 (traffic-simulation) — Phase 3: Protocol Specification

> **Version:** 1.0.0
> **Status:** Official
> **Phase:** 3 of 4 (Protocol)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 3 specifies the wire protocols by which WIA-CITY-017 engines exchange data with **roadside infrastructure**, **traffic-management centres**, **vehicles or driving simulators**, and **other simulation engines** participating in a federated study.

The objective is to give a deterministic mapping of simulation events and outputs to the relevant ISO ITS message families and to fix the operational interoperability points where a simulation can be replayed against a real-world deployment.

### 1.1 Protocol stack

```
+---------------------------------------------------+
| WIA-CITY-017 application semantics (Phase 1+2)    |
+---------------------------------------------------+
| Mediation: engine, GDF mapper, SPaT/MAP encoder   |
+---------------------------------------------------+
| Transport (one of):                               |
|   ISO/TS 19091 (V2I / I2V)                        |
|   ISO 19082 (roadside module ↔ signal controller) |
|   ISO 14825 (GDF import / export)                 |
|   CoAP/UDP/IP    | HTTP/TCP/IP    | TLS / DTLS    |
+---------------------------------------------------+
| Physical: PoE-Ethernet / fibre / cellular / 5.9GHz|
+---------------------------------------------------+
```

---

## 2. ISO 14825 GDF Bridge

### 2.1 Import

Engines MUST accept ISO 14825 GDF 5.0 import documents over the Phase-2 endpoint `/networks/{id}:import-gdf`. The import process:

1. Validates the GDF schema and feature catalog version.
2. Maps GDF feature types to WIA-CITY-017 *Network* node and link types.
3. Preserves the GDF feature identifier on every produced node and link.
4. Reports unmapped feature types as informative warnings without aborting the import.

### 2.2 Export

Engines MUST emit ISO 14825 GDF 5.0 export documents over the Phase-2 endpoint `/networks/{id}:export-gdf`. The export carries:

- The complete topology (nodes, links, lanes).
- The complement of attributes that survive round-trip.
- A header identifying the engine version and the WIA-CITY-017 schema version.

### 2.3 Round-trip fidelity

Round-trip fidelity is measured by re-import of an exported GDF. Engines MUST achieve fidelity ≥ 99% on the canonical conformance suite published with this specification.

---

## 3. ISO/TS 19091 V2I / I2V Bridge

### 3.1 MAP messages

The *MAP* message (ISO/TS 19091 §6.1) describes the geometry and topology of a signalised intersection. WIA-CITY-017 engines MUST emit MAP messages that:

- Match the canonical Phase-1 *Intersection* descriptor's `nodeId`.
- Encode every *Movement* and *AllowedManeuver* in the MAP message body.
- Pass the ISO/TS 19091 conformance test vectors when bytewise compared.

### 3.2 SPaT messages

The *SPaT* (Signal Phase and Timing) message (ISO/TS 19091 §6.2) describes the current and predicted state of every signal phase at an intersection. WIA-CITY-017 engines MUST emit SPaT messages every 100 ms during a real-time replay, or at the scenario step rate (typically 1 Hz) during a non-real-time export.

### 3.3 Replay binding

When an engine is bound to a real-world deployment for hardware-in-the-loop testing, the SPaT stream replaces the live SPaT feed of the affected intersection. The deployment MUST maintain a privileged channel that re-asserts the live SPaT in case of engine disconnection within 1 s.

---

## 4. ISO 19082 Roadside Module Bridge

### 4.1 Data frames

ISO 19082:2025 specifies the data elements and data frames between roadside modules (RSM) and signal controllers (SC). WIA-CITY-017 engines MUST emit data frames for:

- *Operational state* (running, faulted, manual override).
- *Detector inputs* (presence detectors, induction loops, video detection).
- *Phase requests* (preemption, priority, demand).

### 4.2 Mapping to engine state

| WIA-CITY-017 entity | ISO 19082 element |
|---------------------|-------------------|
| `Intersection.phases[].id` | Phase identifier |
| `Vehicle trace at stop bar` | Detector input frame |
| `Emergency vehicle preemption` | Priority request frame |
| `Bus priority request` | Priority request frame |
| `Faulted lamp / detector` | Operational-state frame |

### 4.3 Latency budgets

End-to-end latency from engine event to ISO 19082 frame emission MUST not exceed 50 ms 95th percentile during a real-time replay.

---

## 5. Vehicle Trace Streaming

### 5.1 Push semantics

Vehicle traces are streamed to subscribers either as bulk bundles (Phase 1 §3.3) or as live frames over RFC 8895-style server-sent events. Live frames are CBOR records with the structure of a single state record from the trace bundle, prefixed with the vehicle identifier and step time.

### 5.2 Pull semantics

Bulk bundles are downloaded over HTTP/2 or HTTP/3 with `Range` header support per RFC 9110 §14. Implementations SHOULD support resumable downloads.

### 5.3 Privacy

When the trace originates from a real-world deployment, the privacy controls of Phase 1 §5 apply at every step of the streaming pipeline. The stream MUST refuse to advertise `vehicleId` values that would re-identify a subject under the operator's privacy policy.

---

## 6. Engine Federation Protocol

### 6.1 Discovery

A federation root document is a JSON object served at the project's `/.well-known/wia-traffic-federation` endpoint. The document lists partner project URIs, supported export formats, and the federation key (COSE_Sign1 verifying key).

### 6.2 Token exchange

Tokens issued for federation MUST use the OAuth 2.1 token-exchange grant (RFC 8693). The exchanged token's audience is the partner project URI; its scope is restricted to the partner-allowed verb set.

### 6.3 Run synchronisation

Federated runs synchronise via:

- A *master* engine that holds the canonical RNG seed.
- Periodic *checkpoint* messages carrying the engine state hash.
- A *catch-up* protocol that uses HTTP range requests to reconcile diverged engines.

### 6.4 Result aggregation

Each engine in a federation produces a Phase-1 *Result* object. The federation aggregator computes the project-level metrics from the union of *Result* objects.

---

## 7. Identity, Time, and Cryptography

### 7.1 Identity

- **Engines and roadside modules**: X.509 v3 certificates (RFC 5280) issued by the project PKI.
- **Operators**: federated identity over OAuth 2.1 (RFC 9700) and OpenID Connect.

### 7.2 Time

Real-time replay against deployed infrastructure MUST use NTPv4 with NTS (RFC 5905, RFC 8915). Discrepancies > 100 ms MUST raise an audit event.

### 7.3 Cryptography

| Layer | Algorithm | Reference |
|-------|-----------|-----------|
| TLS / RTSPS | TLS 1.3 cipher suites | RFC 8446 |
| DTLS | DTLS 1.3 | RFC 9147 |
| OSCORE | AES-CCM-16-64-128 | RFC 8613 §12 |
| COSE signature | ES256, EdDSA | RFC 9053 |
| Trace encryption | AES-256-GCM | ISO/IEC 18033-3 |

---

## 8. Failure Handling

### 8.1 GDF import failure

A GDF import that fails ISO 14825 schema validation MUST raise the `traffic/network-incompatible-gdf` problem and MUST NOT partially commit any node or link.

### 8.2 SPaT/MAP export failure

An export that fails ISO/TS 19091 conformance MUST raise the `traffic/intersection-spat-incompatible` problem with the offending intersection identifier and a list of failing fields.

### 8.3 Engine crash mid-run

Engine crashes MUST flush the current step's vehicle traces to durable storage before exiting where possible. On restart, the engine MUST refuse to resume a partially completed run; instead, it MUST advertise the run as `state=ABORTED` and require a fresh launch.

### 8.4 Network partition during federation

A federation partition exceeding the configured *catch-up* timeout MUST escalate to the federation arbiter. The arbiter chooses one of: rejoin (if the lagging engine can catch up within a bounded window), checkpoint-rollback, or run termination.

---

## 9. Conformance Profiles

### 9.1 Baseline profile (P-B)

A P-B engine MUST:

- Implement Phase-2 HTTP/REST surface.
- Pass GDF round-trip fidelity ≥ 99%.
- Pass ISO/TS 19091 MAP and SPaT export against the canonical conformance vectors.
- Emit COSE-signed result bundles.

### 9.2 Real-time profile (P-RT)

A P-RT engine MUST additionally:

- Sustain the §3.2 SPaT 100 ms emission cadence.
- Sustain the §4.3 ISO 19082 50 ms emission cadence.
- Maintain time-sync ≤ 50 ms vs. the reference clock.

### 9.3 Federated profile (P-F)

A P-F engine MUST additionally:

- Implement §6 federation protocol.
- Implement OAuth 2.1 token exchange (RFC 8693).
- Maintain checkpoint cadence ≤ 10 s during federated runs.

---

## 10. References

1. RFC 5280 — *X.509 PKI Certificate and CRL Profile.*
2. RFC 5905; RFC 8915 — *NTPv4, NTS.*
3. RFC 7252; RFC 7641 — *CoAP, OBSERVE.*
4. RFC 8446; RFC 9147 — *TLS 1.3, DTLS 1.3.*
5. RFC 8613 — *OSCORE.*
6. RFC 8693 — *OAuth 2.0 Token Exchange.*
7. RFC 8949 — *CBOR.*
8. RFC 9052; RFC 9053 — *COSE.*
9. RFC 9110; RFC 9457 — *HTTP semantics, problem details.*
10. RFC 9700 — *OAuth 2.1 (BCP).*
11. ISO 14817-1:2015; ISO 14817-2:2015; ISO 14817-3:2017.
12. ISO 14825:2011.
13. ISO 17572 (all parts).
14. ISO 19082:2025.
15. ISO/TS 19091:2017/2019.
16. ISO 21217:2020.
17. ISO/IEC 18033-3:2010.
18. FIPS 180-4; FIPS 197.

---

## 11. Detailed Conformance Tests

### 11.1 GDF round-trip test

The conformance test suite includes a canonical GDF 5.0 document with ~5,000 features. The engine MUST:

1. Import the document.
2. Emit a Phase-1 *Network* descriptor.
3. Re-export the network as GDF.
4. Compare the re-exported GDF against the original; the matching ratio MUST be ≥ 99% on a per-feature basis.

Reportable mismatches are tracked under three categories: *topology* (node/link count, connectivity), *attributes* (lane count, speed limit, permitted classes), and *geometry* (vertex count, vertex displacement). The 99% threshold applies to the combined score across categories.

### 11.2 SPaT/MAP byte-wise vector test

A canonical set of intersections (small, mid-size, complex) is published with byte-wise expected MAP and SPaT message outputs. The engine MUST emit byte-equivalent messages for each intersection at each phase transition over a 60-second simulated run.

### 11.3 ISO 19082 frame test

The engine emits a 30-second sequence of ISO 19082 data frames against a virtual signal controller. The frames MUST match a canonical reference sequence under a tolerated timing skew of ±10 ms.

### 11.4 NTS time-sync test

The engine MUST sustain ≤ 50 ms time deviation against a reference NTS-protected NTPv4 server over a 1-hour run while subjected to a network jitter profile of 10 ms 95th-percentile.

### 11.5 Federation token-exchange test

The engine MUST successfully obtain a federation-scoped token via RFC 8693 token exchange, exercise an export against a partner project, and verify the returned bundle's COSE_Sign1 signature against the federation key listed in the partner project's federation document.

---

## 12. Profile Selection Guidance

Operators choose a conformance profile based on the deployment's role:

- **Off-line study (P-B)** — calibration, scenario exploration, what-if analyses; latency budgets relaxed.
- **Real-time replay (P-RT)** — TMC dashboarding, observatory; strict latency budgets, no controller binding.
- **Hardware-bound (P-RT + HiL)** — bound to a real controller; same latency as P-RT plus the §3.3 fallback assertion.
- **Federated (P-F)** — multi-engine studies; checkpoint cadence and federation token discipline.

A deployment MAY combine profiles (e.g. P-RT + P-F). Combined profiles MUST surface every constituent tag in Phase 4 §12.
