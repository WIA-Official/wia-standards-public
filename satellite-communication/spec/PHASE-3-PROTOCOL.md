# WIA-SPACE-003 (satellite-communication) — Phase 3: Protocol Specification

> **Version:** 1.0.0
> **Status:** Official
> **Phase:** 3 of 4 (Protocol)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 3 specifies the wire protocols by which WIA-SPACE-003 deployments exchange data between **spacecraft**, **ground stations**, **user terminals**, **gateways**, and the **management plane**. The objective is to give a deterministic mapping of every Phase-2 verb to the relevant transport, and to fix the operational interoperability points between the deployment, the IETF *Delay-Tolerant Networking* family, and the OSI reference model that organises the entire stack.

### 1.1 Protocol stack

```
+---------------------------------------------------+
| WIA-SPACE-003 application semantics (Phase 1+2)   |
+---------------------------------------------------+
| Mediation: gateway, bundle agent, link planner    |
+---------------------------------------------------+
| Transport (one of):                               |
|   RFC 9171 BPv7 over TCPCL / UDPCL / LTPCL        |
|   CoAP/UDP/IP                                     |
|   HTTP/TCP/IP                                     |
|   TLS / DTLS                                      |
+---------------------------------------------------+
| Physical: RF (L–Q bands) / optical / waveguide    |
+---------------------------------------------------+
```

The OSI layering of ISO/IEC 7498-1:1994 is preserved; each WIA artefact is associated with one or more OSI layers as documented in §6.

---

## 2. Bundle Protocol (RFC 9171)

### 2.1 Bundles as first-class objects

Bundles are first-class objects on the WIA-SPACE-003 wire. A bundle is a sequence of CBOR-encoded blocks per RFC 9171 §4.1. The primary block carries:

- Bundle Protocol version (= 7).
- Bundle processing flags.
- Source, destination, report-to EIDs.
- Creation timestamp.
- Lifetime.

Implementations MUST support the IPN URI scheme (RFC 9171 §4.2.5.2). Implementations MAY support the `dtn:` scheme.

### 2.2 Convergence layers

The following convergence layers are recognised:

| Convergence | Reference | Use |
|-------------|-----------|-----|
| TCPCL | IETF DTN WG TCP convergence-layer specification | Reliable inter-gateway transfer over TCP |
| UDPCL | IETF DTN WG UDP convergence-layer specification | Lossy / lower-latency transfer |
| LTPCL | IETF DTN WG LTP convergence-layer specification | Very-long-delay deep-space |

Each convergence layer's wire format defers to its IETF specification; the WIA descriptor ties a bundle-session to a chosen convergence layer.

### 2.3 Default security context

Bundle authenticity and confidentiality follow RFC 9173 *Default Security Context for the Bundle Protocol*. The deployment MUST use the BIB (Bundle Integrity Block) and BCB (Bundle Confidentiality Block) when the bundle traverses any untrusted segment.

### 2.4 Custody and reports

Custody transfer (where supported by the operator) and status reports (delivery, forward, acknowledgement) follow the RFC 9171 control-flow rules. The Phase-1 *BundleSession* descriptor records the custody-transfer policy chosen for the session.

### 2.5 Lifetime

The bundle lifetime field is honoured strictly. Expired bundles MUST raise the `space/bundle-lifetime-expired` problem on the management plane and MUST trigger an audit entry.

---

## 3. Telemetry and Tele-command Sessions

### 3.1 Frame profile

Telemetry and tele-command sessions ride on operator-chosen telemetry frames. The Phase-1 *TelemetrySession* descriptor records the frame profile name and version. The wire frame format itself is owned by the operator's space-link tooling and is not redefined by WIA-SPACE-003.

### 3.2 Encryption at transport

When `encryptionAtTransport.algorithm ≠ NONE`, the operator-chosen encryption MUST use AES-256-GCM or AES-128-GCM with a key managed under ISO 11770-2 (symmetric) or ISO 11770-3 (asymmetric).

### 3.3 Tele-command authorisation

Tele-commands carry an additional authorisation envelope: a COSE_Sign1 signature (RFC 9052) over the canonical command bytes, produced by a privileged operator key. Spacecraft MUST verify the signature before acting on the command.

### 3.4 Replay protection

Tele-commands include a strictly monotonic sequence number tied to the spacecraft's command counter. Replays are rejected at the spacecraft; the gateway MUST ensure that sequence numbers are not reused.

---

## 4. Identity, Time, and Cryptography

### 4.1 Identity

- **Spacecraft, ground stations, user terminals**: X.509 v3 certificates (RFC 5280) issued by the deployment PKI.
- **Operators**: federated identity over OAuth 2.1 (RFC 9700) and OIDC.

### 4.2 Time

Real-time operations MUST be slaved to NTPv4 with NTS (RFC 5905, RFC 8915). Spacecraft on-board time is reconciled against ground reference at every tracking pass.

### 4.3 Cryptographic algorithms

| Layer | Algorithm | Reference |
|-------|-----------|-----------|
| TLS / HTTP | TLS 1.3 cipher suites | RFC 8446 |
| DTLS | DTLS 1.3 | RFC 9147 |
| OSCORE | AES-CCM-16-64-128 | RFC 8613 §12 |
| COSE signature | ES256, EdDSA | RFC 9053 |
| BPv7 BIB / BCB | per RFC 9173 default security context | RFC 9173 |
| At-rest storage | AES-256-GCM | ISO/IEC 18033-3, FIPS 197 |

Implementations MUST refuse cipher suites whose IETF status is "not recommended" for new deployments.

---

## 5. Failure Handling

### 5.1 Loss of contact

Loss of contact between a spacecraft and a ground station for more than the configured no-contact timeout MUST raise an alarm on the management plane. The bundle agent MUST hold queued bundles within their lifetime budget and MUST refuse new bundles whose lifetime would exceed the resumed-contact horizon by an unsafe margin.

### 5.2 Cryptographic failure

Failed BIB or BCB verification, expired certificate, or OCSP "revoked" status MUST cause the offending bundle to be discarded with an audit entry.

### 5.3 Tele-command rejection

Tele-commands that fail signature verification, sequence checks, or operator-policy gates MUST be rejected at both the gateway and the spacecraft. The audit log MUST record the rejection reason.

### 5.4 Gateway crash

Gateway crashes MUST flush in-flight session state to durable storage before exiting where possible. On restart, the gateway MUST refuse to silently resume sessions; instead, sessions MUST be re-established under explicit operator action.

---

## 6. OSI Mapping

| OSI layer | WIA-SPACE-003 artefact | Notes |
|-----------|------------------------|-------|
| 1 (Physical) | RF or optical waveform | Owner: operator + national radio regulation |
| 2 (Data Link) | Frame profile of TM/TC sessions | §3.1 |
| 3 (Network) | BPv7 EIDs and routing | RFC 9171 |
| 4 (Transport) | Convergence layer (TCPCL, UDPCL, LTPCL) | §2.2 |
| 5 (Session) | BundleSession (Phase 1 §2.6) | — |
| 6 (Presentation) | CBOR / JSON, COSE | RFC 8949 / 9052 |
| 7 (Application) | Phase-2 verbs | §4 of Phase 2 |

The OSI mapping is informative for human readers and normative for the conformance checklist of Phase 4 §6.

---

## 7. Conformance Profiles

### 7.1 Baseline profile (P-B)

A P-B gateway MUST:

- Implement HTTP/REST surface (Phase 2 §2).
- Implement BPv7 (RFC 9171) ingress and egress when the deployment uses bundle convergence.
- Implement RFC 9173 default security context.
- Issue COSE_Sign1 audit-log digests per Phase 4 §3.

### 7.2 Constrained-terminal profile (P-CT)

A P-CT gateway MUST additionally:

- Implement CoAP surface (Phase 2 §3).
- Implement OSCORE (RFC 8613) for end-to-end protection.
- Implement CoRE Resource Directory registration (RFC 9176).

### 7.3 Deep-space profile (P-DS)

A P-DS gateway MUST additionally:

- Implement LTPCL convergence layer.
- Carry per-spacecraft mission time-base reconciliation.
- Carry custody-transfer policy in BundleSession descriptors.

---

## 8. Conformance Vectors

The conformance suite includes:

- **BPv7 bundle round-trip** vectors per RFC 9171 §11.
- **BIB / BCB protection** vectors per RFC 9173 §10.
- **CoAP OBSERVE** vectors per RFC 7641 §6.
- **TLS 1.3 cipher inventory** test against the RFC 8446 mandatory profile.
- **NTS time-sync** test against an RFC 8915 reference server.
- **Tele-command replay** test that intentionally replays a command and asserts rejection.

A conformant gateway MUST pass every vector in the suite for its declared profile set.

---

## 9. Latency and Reliability Budgets

Operators MUST publish, per service plan, the following budgets:

- 95th-percentile end-to-end latency for management-plane HTTP calls.
- 95th-percentile end-to-end latency for bundle delivery (per orbital regime).
- Per-month availability commitment.
- Per-day bundle delivery commitment.

These budgets are recorded in the Phase-1 *ServicePlan* descriptor and surfaced through the discovery document.

---

## 10. References

1. ISO/IEC 7498-1:1994 — *OSI Basic Reference Model.*
2. ISO/IEC 18033-3:2010 — *Block ciphers.*
3. ISO/IEC 27001:2022; ISO/IEC 27037:2012.
4. ISO 11770-2; ISO 11770-3 — *Key management.*
5. IEC 62443-3-3:2013.
6. RFC 4838 — *DTN Architecture.*
7. RFC 5280 — *X.509 PKI Certificate and CRL Profile.*
8. RFC 5905; RFC 8915 — *NTPv4, NTS.*
9. RFC 7252; RFC 7641; RFC 7959 — *CoAP family.*
10. RFC 8446; RFC 9147 — *TLS 1.3, DTLS 1.3.*
11. RFC 8613 — *OSCORE.*
12. RFC 8949 — *CBOR.*
13. RFC 9052; RFC 9053 — *COSE.*
14. RFC 9110; RFC 9457 — *HTTP semantics, problem details.*
15. RFC 9171; RFC 9173 — *BPv7 and default security context.*
16. RFC 9176 — *CoRE Resource Directory.*
17. RFC 9700 — *OAuth 2.1.*
18. FIPS 180-4 — *Secure Hash Standard.*
19. FIPS 197 — *Advanced Encryption Standard.*

---

## 11. Detailed Conformance Tests

### 11.1 BPv7 round-trip

The conformance suite includes canonical bundles built with each combination of convergence layer (TCPCL, UDPCL, LTPCL) and security context (RFC 9173 BIB only, BIB+BCB, neither). The gateway MUST round-trip every combination without payload corruption.

### 11.2 BIB / BCB protection vectors

Test vectors derived from RFC 9173 §10 test the gateway's handling of the *Default Security Context for the Bundle Protocol*. The gateway MUST reproduce expected canonical bytes for the BIB and BCB on each test bundle.

### 11.3 CoAP OBSERVE

A long-running CoAP OBSERVE subscription on a synthetic telemetry resource MUST be sustained for 24 hours without false re-registration; observers MUST receive notifications with monotonically increasing 24-bit Observe values.

### 11.4 Tele-command replay

The conformance suite intentionally replays a canonical tele-command. The gateway MUST reject the replay with the `space/forbidden-zone` problem code or an equivalent vendor-specific code, and MUST emit an audit entry that surfaces the replay reason.

### 11.5 NTS time-sync

The gateway MUST sustain ≤ 50 ms time deviation against a reference NTS-protected NTPv4 server over a 24-hour run while subjected to a network jitter profile of 10 ms 95th-percentile.

### 11.6 OSI mapping audit

For each artefact in the deployment, the gateway MUST emit a discovery hint binding the artefact to its OSI layer per Phase 3 §6. The hint surfaces under `/.well-known/wia-space-osi-mapping` and is validated by the conformance suite.

---

## 12. Profile Selection Guidance

Operators choose a conformance profile based on the deployment's role:

- **Off-line catalog management (P-B)** — catalog, contact-window planning, link-budget calculations; no real-time traffic.
- **IoT-class user terminals (P-CT)** — constrained-terminal profile; CoAP and OSCORE on the user side.
- **Deep-space missions (P-DS)** — long-delay support; LTP convergence layer; mission time-base reconciliation.

A deployment MAY combine profiles. Combined profiles MUST surface every constituent tag in Phase 4 §10.
