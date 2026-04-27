# WIA-medical-iot PHASE 3 — Protocol Specification

**Standard:** WIA-medical-iot
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE specifies the protocols binding the data formats
(PHASE 1) and the API surface (PHASE 2) to operational exchanges:
device authentication, link-layer encryption, time discipline,
clock-synchronisation between devices and the boundary, IEC 80001-1
risk-network bring-up, audit-chain construction, and incident
handling for connectivity loss or device compromise.

References (CITATION-POLICY ALLOW only):
- IETF RFC 8446 (TLS 1.3), RFC 7525 (TLS recommendations)
- IETF RFC 7252 (CoAP) and RFC 8323 (CoAP over TCP/TLS/WebSockets)
- IETF RFC 8613 (OSCORE) — Object Security for Constrained RESTful
  Environments, used with CoAP for constrained devices
- IETF RFC 9162 (Certificate Transparency 2.0 pattern) — for tamper-
  evident audit chain construction
- IEEE 802.11-2020 (Wi-Fi), IEEE 802.15.4 (Zigbee), Bluetooth Core 5.4
- 3GPP TS 33.501 (5G security architecture)
- IEC 80001-1:2021 — risk management
- IEC 62304:2006/A1:2015 — software life cycle
- HL7 SMART App Launch 2.2

---

## §1 Authentication

Devices, gateways, clinicians, biomedical engineers, and patient
self-service principals authenticate using JWS-signed JWTs issued
by the deployment's identity authority. Token claims:

- `iss`, `sub`, `aud`, `iat`, `exp`
- `wia.role` — one of {`device`, `gateway`, `clinician`, `bme`,
  `patient`, `auditor`}
- `wia.deviceRef` — for device tokens, the URN of the bound device
- `wia.scope[]` — operation-class scopes
- `wia.holderRef` — for clinician/BME tokens, the organisation
  whose patients/devices may be acted upon
- `cnf` — confirmation claim binding the token to a TLS client
  certificate (RFC 8705 mTLS-bound JWT)

Device tokens are short-lived (≤ 24 hours); rotation is automated
via the device-registry (PHASE 4 §2). Clinician tokens require a
SMART-launch grant against the EHR.

## §2 Constrained device profile (CoAP + OSCORE)

For battery-powered or otherwise constrained devices, the
boundary accepts the IETF constrained profile:

- CoAP over UDP with DTLS 1.3 for transport
- OSCORE (RFC 8613) for object-security between the device and
  the boundary, surviving gateway translation
- CBOR-encoded payloads with a JSON-LD context that maps onto
  the FHIR JSON model

OSCORE security contexts are bound to the device's `deviceRef`
and rotated on a deployment-declared cadence. Replay protection
uses the OSCORE sequence number; replay outside the sequence
window is rejected and audit-logged.

## §3 General-purpose device profile (HTTP + TLS 1.3)

For mains-powered or higher-bandwidth devices:

- HTTP/1.1 or HTTP/2 over TLS 1.3
- Server certificate from the deployment's PKI; optional client
  certificate for mTLS (required for high-priority alarms and
  control-plane operations)
- JSON-encoded FHIR payloads
- Connection re-use across observations to keep handshake cost low
- Server-Sent Events for streaming alarms and tele-monitoring

The cipher-suite floor cross-references WIA-network-security:
deployments in the `hybrid` migration phase configure
TLS 1.3 with hybrid groups (per WIA-pq-crypto PHASE 3 §3).

## §4 Personal-health device profile (Bluetooth LE)

Personal-health devices (consumer-grade glucose meters, blood
pressure monitors, scales) typically reach the boundary via a
patient-owned smartphone gateway. The link is:

- Bluetooth LE with the Bluetooth GATT profile defined by the
  IEEE 11073-20601 application profile and the device-class GATT
  service (e.g., Continuous Glucose Monitoring 0x181F)
- The smartphone gateway runs the deployment's authorised
  patient-facing app, which authenticates the patient (SMART
  patient launch) and translates IEEE 11073 into FHIR
- The translated FHIR is delivered to the boundary over TLS 1.3

The gateway, not the device, is the principal for authentication.
The boundary records the gateway-device pairing in the
`gatewayRef` field of the connectivity binding (PHASE 1 §7).

## §5 Audit chain

Every observation ingest, alarm transition, calibration,
device-registration mutation, association change, and bulk
export emits an AuditEvent. AuditEvents form a per-deployment
hash chain:

```
chain_input = SHA-256(prev_chain_root || canonical(event))
chain_root_t = chain_input
```

Canonicalisation uses RFC 8785 JSON Canonicalisation Scheme.
The chain root is sealed once per UTC day; sealed roots MAY be
published to a Certificate-Transparency-style log (RFC 9162
pattern) for jurisdictions that require public auditability of
medical-device interactions.

For high-volume continuous monitoring (ECG, capnography), the
chain is sharded by device-ref hash prefix to keep sealing
throughput within operational SLAs; the sharding is itself
audited so an after-the-fact reshard is detectable.

## §6 Time discipline

Clocks synchronise to authoritative sources:

- Boundary: NTPv4 stratum-2 (or PTPv2 IEEE 1588 for facilities
  requiring sub-millisecond synchronisation, e.g., catheterisation
  lab telemetry)
- Devices on AC mains power: NTPv4 stratum-3 against the boundary
- Battery-powered devices: time set on each successful handshake;
  device-clock drift is captured in the observation envelope

A device whose declared `effectiveDateTime` is more than the
deployment-declared skew tolerance from boundary time has the
observation accepted but flagged `interpretation: "clock-skew"`.
Persistent skew triggers a maintenance ticket via PHASE 4 §6.

## §7 Connectivity loss and store-and-forward

Devices disconnected from the boundary buffer observations
locally up to the device's documented buffer capacity:

- On reconnection, the device replays buffered observations in
  sequence; the boundary accepts replay only if the device's
  signature on each observation chains back to a session that
  was active during the buffered interval
- Buffer overflow drops the oldest observations; the device emits
  a `buffer-overflow` annotation that the boundary records
- For alarm conditions, buffer-overflow triggers a higher-priority
  reconnection re-attempt schedule per IEC 60601-1-8

## §8 Device compromise handling

When a device is suspected compromised (out-of-policy firmware,
unexpected behaviour, lost-and-found scenario):

- Biomedical engineering submits a `device-quarantine` record
  binding the `deviceRef` and an incident reference
- Boundary refuses further observations from the device until
  re-attestation
- Active patient associations are dissolved with explicit
  `dissolutionReason: "device-quarantine"`
- Re-attestation requires fresh UDI-PI scan + clean firmware
  attestation + biomedical-engineer two-person sign-off

The quarantine event is part of the audit chain; downstream
research datasets containing observations from a quarantined
device's pre-quarantine window are flagged for review.

## §9 Replay protection

All write endpoints accept `Idempotency-Key`; the boundary
stores keys for 30 days. Replays return the original response;
conflicts return `urn:wia:miot:problem:idempotency-conflict`.

CoAP/OSCORE replay protection uses the OSCORE sequence number
window. TLS-protected sessions rely on sequence numbers within
the session; cross-session replay is rejected via the
`Idempotency-Key`.

## §10 IEC 80001-1 risk-network bring-up

Each new MedIoT deployment publishes an IEC 80001-1 risk file
covering:

- The clinical use cases the network supports
- The risk-control measures (segmentation, monitoring, alarms)
- Residual risks and clinical-acceptance signatures
- Maintenance windows and emergency procedures

The risk file is referenced by the capability document (PHASE 2
§1). Material changes to the network (new device class, new
gateway, IT-side configuration changes) require a risk-file
update before the change is committed.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Worked CoAP/OSCORE exchange (informative)

```
Request: CoAP CON POST coaps://boundary.example/Observation/$ingest
Token: 0x91a7
Content-Format: application/fhir+cbor
Object-Security: <OSCORE option with seq=1024>
Payload: <CBOR-encoded FHIR Observation per PHASE 1 §3>
```

```
Response: CoAP ACK 2.04 Changed
Token: 0x91a7
Object-Security: <OSCORE option with seq=1024>
Payload: <empty>
```

The boundary appends the observation to the chain; the device
considers the observation persisted only after receiving the
ACK with matching token and OSCORE sequence acknowledgement.

## Annex B — Algorithm choices (informative)

| Concern               | Default                            | Notes                                       |
|-----------------------|------------------------------------|---------------------------------------------|
| Token signing         | ES256                              | mTLS-bound (RFC 8705)                       |
| TLS                   | 1.3 (RFC 8446)                     | hybrid groups when in WIA-pq-crypto hybrid  |
| OSCORE AEAD           | AES-CCM-16-64-128                  | RFC 8613 mandatory                          |
| Audit hash            | SHA-256 (RFC 6234)                 |                                             |
| Symmetric at rest     | AES-256-GCM                        |                                             |

Quantum-resistance migration is governed by WIA-pq-crypto.

## Annex C — Negative-test vectors (informative)

| Stimulus                                                | Expected outcome                                  |
|---------------------------------------------------------|---------------------------------------------------|
| Token without `wia.deviceRef` for device principal      | 403 + token-malformed                             |
| OSCORE replay outside sequence window                   | refused; audit-logged                             |
| Device clock skew beyond tolerance                      | accepted, flagged `clock-skew`                    |
| Buffer-overflow replay with broken signature chain      | refused; quarantine-warning emitted               |
| Quarantined device attempting to ingest                 | 403 + `device-quarantined`                        |

## Annex D — Reconnect storm mitigation (informative)

A network outage that drops many devices at once produces a
reconnect storm when connectivity is restored. The boundary
applies admission control:

- Reconnects are admitted at a deployment-declared rate
  (typically 100 reconnects/sec/cluster)
- Devices receive a `Retry-After` header on overflow with
  a jitter-randomised back-off
- Critical devices (high-priority alarm-bearing) are admitted
  with priority 0; non-critical devices admit on priority 1
- Buffered observations from priority-0 devices are processed
  first; priority-1 observations queue behind

This prevents the boundary itself from becoming the next
failure mode after a network outage.

## Annex E — Firmware update protocol (informative)

Firmware updates flow through the biomedical-engineering
control plane:

1. Manufacturer publishes an update with a signed manifest
   (cross-reference to WIA-supply-chain)
2. BME imports the update into the deployment's update
   repository
3. Devices poll for updates on a documented cadence; the
   poll authenticates the device and returns the manifest
4. Device verifies the manifest signature against the
   manufacturer's published key
5. Device installs in a maintenance window declared by BME
6. Post-install, device re-attests its firmware identifier
   to the boundary; the boundary updates the Device
   resource's `softwareRef`

A device that fails post-install attestation is quarantined
(PHASE 3 §8) pending BME investigation.
