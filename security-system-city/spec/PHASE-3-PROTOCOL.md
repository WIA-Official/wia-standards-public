# WIA-CITY-014 (security-system-city) — Phase 3: Protocol Specification

> **Version:** 1.0.0
> **Status:** Official
> **Phase:** 3 of 4 (Protocol)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 3 specifies the wire protocols used by WIA-CITY-014 between **cameras**, **recorders**, **access controllers**, **sensors**, **gateways**, and the **management plane**. The objective is to give a deterministic mapping of every Phase-2 verb to the relevant transport, and to fix the operational interoperability points required by the IEC 62676 series (video) and IEC 60839 series (alarm and access control).

### 1.1 Protocol stack

```
+---------------------------------------------------+
| WIA-CITY-014 application semantics (Phase 1+2)    |
+---------------------------------------------------+
| Mediation: gateway, evidence builder, alarm bus   |
+---------------------------------------------------+
| Transport (one of):                               |
|   IEC 62676-2-3 (RTP/RTSP)                        |
|   IEC 60839-5-x (alarm transmission)              |
|   IEC 60839-7-x (alarm transmission protocols)    |
|   CoAP/UDP/IP    | HTTP/TCP/IP   | TLS / DTLS     |
+---------------------------------------------------+
| Physical: PoE-Ethernet / fibre / sub-GHz / cellular|
+---------------------------------------------------+
```

---

## 2. Video Transmission (IEC 62676-2-3)

### 2.1 Session control

RTSP 2.0 (RFC 7826) over TLS 1.3 (RFC 8446) — referred to as RTSPS — is the only conforming session-control protocol for new deployments. RTSP 1.0 (RFC 2326) is preserved only for legacy interoperability and MUST be tunnelled over TLS regardless.

### 2.2 Media transport

- **RTP** (RFC 3550) for live and recorded video.
- **AVC** (ISO/IEC 14496-10) and **HEVC** (ISO/IEC 23008-2) are the two normative codecs. Other codecs MAY be supported but MUST be reported as informative in the stream descriptor.
- **RTP payload formats** follow IETF specifications: RFC 6184 (H.264/AVC), RFC 7798 (H.265/HEVC).
- **SRTP** (RFC 3711) MAY be used as the media-plane protection layer; RTSPS already covers session control.

### 2.3 Recording control

Recorder-side recording is controlled through the Phase-2 HTTP surface. The wire format for recorded segment retrieval follows IEC 62676-2-3 export rules; segment containers use ISO/IEC 14496-12 (ISO Base Media File Format).

### 2.4 Operational performance category

Stream descriptors carry the IEC 62676-4 category (Phase 1 §2.3). Mid-stream transitions (e.g. zoom level, tilt) that change the achievable category MUST be reflected by an SDP attribute `a=wia-city-014:category-changed:<NEW>`.

---

## 3. Access-Control Transmission (IEC 60839-5/-7)

### 3.1 Wired alarm transmission

The IEC 60839-5-1 series specifies *alarm transmission system requirements*. Conforming transmission units MUST satisfy:

- IEC 60839-5-2 — *Requirements for the transceiver units.*
- IEC 60839-5-3 — *Requirements for the alarm transmission protocols.*

WIA-CITY-014 layers these under a common encapsulation document that maps each Phase-1 *Alarm* event to a single IEC 60839-5-3 *primitive* (notification, ack, query, response).

### 3.2 IP-based alarm transmission

For IP-native deployments, the gateway translates Phase-1 *Alarm* objects into TLS-protected JSON or CBOR payloads pushed over RFC 9110 HTTP. CoAP push (RFC 7641 OBSERVE) is the constrained-side equivalent. End-to-end protection uses COSE_Sign1 (RFC 9052).

### 3.3 Door command framing

Door commands (`security:control-door`) follow the IEC 60839-11-2 *system performance* response-time bounds:

- Grade 1 / 2: response within 2.0 s.
- Grade 3: response within 1.0 s.
- Grade 4: response within 0.5 s.

The gateway MUST measure the round-trip from issuance of the verb to the controller-acknowledged door state change and surface the measurement in the audit log (IEC 62443-3-3 SR 6.1).

---

## 4. Sensor Transmission

### 4.1 Wired sensors

Wired sensors follow IEC 60839-1 system topology (loop, branch, addressable). The wire-level framing is implementation-defined, but the gateway MUST normalise sensor events into the Phase-1 *Sensor* descriptor and the Phase-1 *Alarm* descriptor for any condition crossing the alarm threshold.

### 4.2 Wireless sensors

Wireless sensors operate over ISO/IEC 14543-3-10 (energy-harvesting WSP) or IEEE 802.15.4-based mesh stacks. The gateway MUST authenticate every wireless sensor with a credential rooted in the site PKI (RFC 5280).

### 4.3 Tamper handling

Sensor tamper events are first-class alarms with `category=TAMPER`. Tamper alarms MUST be raised within 1 s of detection and MUST NOT be silenceable from the operator HTTP surface; clearing requires a physical confirmation event tied to the sensor's lifecycle record.

---

## 5. Identity, Time, and Cryptography

### 5.1 Identity

- **Devices**: X.509 v3 certificates (RFC 5280) issued by the site PKI. Certificate templates carry a *device-identity* OID and bind to the IEC 62676-2-3 device-MAC reference (where present).
- **Operators**: federated identity over OAuth 2.1 (RFC 9700) and OIDC. Group claims MUST resolve to a IEC 62443-2-1 personnel registry.

### 5.2 Time

Recording timestamps MUST be slaved to a time source with documented accuracy ≤ 50 ms relative to UTC. Conforming sources:

- NTPv4 with NTS — RFC 5905 / RFC 8915.
- PTP (IEEE 1588 — informative) where the site has a grandmaster.

Time discontinuities (leap seconds, DST changes, manual adjustments) MUST be logged and MUST NOT silently rewrite recorded segment timestamps.

### 5.3 Cryptography

| Layer | Algorithm | Reference |
|-------|-----------|-----------|
| TLS / RTSPS | TLS 1.3 cipher suites | RFC 8446 |
| DTLS | DTLS 1.3 | RFC 9147 |
| OSCORE | AES-CCM-16-64-128 | RFC 8613 §12 |
| COSE signature | ES256, EdDSA | RFC 9053 |
| At-rest video | AES-256-GCM | ISO/IEC 18033-3 |
| Key management | ISO 11770-2 (symmetric); ISO 11770-3 (asymmetric) | ISO 11770-2/-3 |

Implementations MUST NOT enable cipher suites whose IETF status is "not recommended" for new deployments (e.g. RC4, 3DES) and MUST disable RTSP transport without TLS.

---

## 6. Evidence Bundling (ISO/IEC 27037)

### 6.1 Bundle structure

An evidence bundle is a ZIP archive (informative container; the structure is normative) with:

- `manifest.json` — Phase-1 *Alarm* + *Camera* + *AccessPoint* references; bundle UUID; creation timestamp; PKI chain pointer.
- `manifest.cose` — COSE_Sign1 detached signature over `manifest.json`.
- `clip-<index>.mp4` — video clip(s).
- `events.jsonl` — newline-delimited Phase-1 events.
- `tooling.json` — tool versions, gateway firmware hash, IEC 62676-2-3 RTP capture parameters.

### 6.2 Hashing

`manifest.json` references each evidence file by SHA-256 (FIPS 180-4) digest. Verification of any file MUST recompute the digest and compare against the manifest before acceptance.

### 6.3 Custody record

The custody record tracks acquisition, transport, and chain-of-custody operations per ISO/IEC 27037 §7. Each entry has an actor (operator URI), a time, a device, and a method. The list is append-only; deletions are recorded as new entries with the type `revocation`.

---

## 7. Lockdown Propagation

### 7.1 Site lockdown

A site lockdown is a coordinated state change affecting all access points, all live video subscriptions, and the alarm policy. Lockdown propagation MUST:

1. Issue an HTTP `POST /sites/{siteId}:lockdown` command requiring `security:lockdown` verb.
2. Cascade lock commands to every Phase-1 *AccessPoint* whose `fireSafetyLink=FAIL-SECURE`.
3. Refuse cascade for any access point with `fireSafetyLink=FAIL-SAFE` or `doorClass=emergency-egress`.
4. Notify every connected operator session via the event bus.
5. Record an IEC 62443-3-3 SR 5.4 audit entry.

### 7.2 Lockdown release

Release follows the same pathway in reverse and requires a justification field.

### 7.3 Latency

Lockdown propagation latency MUST not exceed 3.0 s 99th percentile measured from command receipt to last access-point acknowledgement.

---

## 8. Failure Handling

### 8.1 Lost camera

Cameras unreachable for > 30 s MUST be marked `state=unreachable` and MUST surface a Phase-1 *Alarm* with `category=CYBER` and `severity=MAJOR` for any zone whose `operationalRequirement` requires that camera.

### 8.2 Lost recorder

If a recorder loses storage capacity below the IEC 62676-1 §6 reservation (typically 24 h of retention), the gateway MUST raise an alarm with `category=CYBER`, `severity=MAJOR`.

### 8.3 Cryptographic failures

Failed signature verification, expired certificate, OCSP "revoked" status — each is a critical condition that produces an alarm and disables the affected entity until reissued.

---

## 9. Conformance Profiles

### 9.1 Baseline profile (P-B)

A P-B gateway MUST:

- Implement RTSPS + RTP/AVC (ISO/IEC 14496-10).
- Implement HTTP/REST surface (Phase 2 §2).
- Implement IEC 60839-11-1 access-control bridge with ≥ Grade 2.
- Implement COSE-signed evidence bundles (§6).

### 9.2 Extended profile (P-E)

A P-E gateway MUST additionally:

- Implement RTP/HEVC (ISO/IEC 23008-2).
- Implement IEC 62676-4 operational-category attestation.
- Implement IEC 60839-7 IP-based alarm transmission.
- Implement IEC 62443-3-3 SR 1, SR 2, SR 5, SR 6 controls.

### 9.3 City-grade profile (P-C)

A P-C gateway MUST additionally:

- Implement Site lockdown propagation with §7.3 latency budget.
- Implement multi-site federation through the management plane.
- Implement evidence bundling with hardware-backed COSE keys (TPM 2.0 or HSM).

---

## 10. References

1. RFC 2326; RFC 7826 — *RTSP 1.0 / 2.0.*
2. RFC 3550 — *RTP: A Transport Protocol for Real-Time Applications.*
3. RFC 3711 — *The Secure Real-time Transport Protocol (SRTP).*
4. RFC 5280 — *Internet X.509 PKI Certificate and CRL Profile.*
5. RFC 5905; RFC 8915 — *NTPv4 and Network Time Security.*
6. RFC 6184; RFC 7798 — *RTP payload for H.264 / H.265.*
7. RFC 7252; RFC 7641; RFC 7959 — *CoAP family.*
8. RFC 8446; RFC 9147 — *TLS 1.3, DTLS 1.3.*
9. RFC 8613 — *OSCORE.*
10. RFC 9052; RFC 9053 — *COSE.*
11. RFC 9110; RFC 9457 — *HTTP semantics, problem details.*
12. RFC 9700 — *OAuth 2.1 (BCP).*
13. IEC 62676-1:2013; IEC 62676-2-1:2013; IEC 62676-2-3:2013; IEC 62676-4:2025; IEC 62676-5-1:2024.
14. IEC 60839-1:2014; IEC 60839-11-1:2013; IEC 60839-11-2:2014; IEC 60839-5-1; IEC 60839-5-2; IEC 60839-5-3; IEC 60839-7-1.
15. IEC 62443-2-1; IEC 62443-3-3:2013.
16. ISO/IEC 14496-10; ISO/IEC 14496-12; ISO/IEC 23008-2.
17. ISO/IEC 18033-3:2010.
18. ISO/IEC 27001:2022; ISO/IEC 27037:2012.
19. ISO 11770-2; ISO 11770-3.
20. FIPS 180-4 — *Secure Hash Standard.*
21. FIPS 197 — *Advanced Encryption Standard.*
