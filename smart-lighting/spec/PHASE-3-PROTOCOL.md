# WIA-CITY-009 (smart-lighting) — Phase 3: Protocol Specification

> **Version:** 1.0.0
> **Status:** Official
> **Phase:** 3 of 4 (Protocol)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 3 specifies the wire protocols that carry WIA-CITY-009 control and telemetry between **luminaires**, **control gear**, **sensors**, **gateways**, and **management plane**. The objective is to provide deterministic mappings between abstract operations defined in Phase 2 and concrete byte-level behaviour on:

1. **DALI bus (wired)** — IEC 62386-101 Manchester-coded 1200 baud bus.
2. **Wireless extensions** — IEC 62386-104 / -220 carriers (Thread, Bluetooth-mesh, sub-GHz).
3. **Building management plane** — ISO/IEC 14908-1 *Control Network Protocol* (CNP) and IP-based gateways.
4. **Energy-harvesting sensors** — ISO/IEC 14543-3-10 wireless short-packet (WSP).

Each protocol layer is treated as a transport for the canonical Phase-1 payloads. Implementations MUST preserve semantic equivalence under change of carrier.

### 1.1 Protocol stack

```
+-------------------------------------------------+
| WIA-CITY-009 application semantics (Phase 1+2)  |
+-------------------------------------------------+
| Mediation: gateway, mapper, scene engine        |
+-------------------------------------------------+
| Transport (one of):                             |
|   IEC 62386-101  | IEC 62386-104 | ISOIEC 14908 |
|   ISO/IEC 14543-3-10 (WSP)                      |
|   CoAP/UDP/IP    | HTTP/TCP/IP   | TLS / DTLS   |
+-------------------------------------------------+
| Physical: 1200 baud DALI / 2.4 GHz / sub-GHz /  |
|   PoE-Ethernet / fibre / power-line             |
+-------------------------------------------------+
```

---

## 2. DALI Bus (IEC 62386-101)

### 2.1 Physical layer

The DALI bus is a half-duplex differential pair carrying nominal 16 V DC supply and Manchester-encoded data at 1200 baud. The relevant IEC 62386-101 ed. 3.0 parameters are:

| Parameter | Value |
|-----------|-------|
| Idle voltage | 9.5–22.5 V |
| Logical "0" | High level (≥ 9.5 V) |
| Logical "1" | Low level (−6.5 to +6.5 V differential) |
| Bit rate | 1200 baud ± 10% |
| Manchester half-bit | 416.67 μs |
| Maximum stretch | 600 mA (Type 1 power supply) |
| Cable resistance budget | ≤ 2 Ω round trip per device |

### 2.2 Frame formats

WIA-CITY-009 uses three IEC 62386-102 frame types verbatim:

- **Forward frame, 16-bit (control gear)**: start-bit (1) || address byte (8) || command byte (8) || stop (2 idle bits).
- **Forward frame, 24-bit (control device, IEC 62386-103)**: start-bit (1) || event_scheme (3) || address (5) || instance_byte (8) || data_byte (8) || stop.
- **Backward frame, 8-bit**: start-bit (1) || answer_byte (8) || stop.

Bit-timing tolerances and inter-frame gaps follow IEC 62386-101 ed. 3.0 §8.2.

### 2.3 Addressing

| Type | Address byte (binary) | Purpose |
|------|-----------------------|---------|
| Short | `0AAAAAA0` to `0AAAAAA1` | Unicast to short address 0..63 |
| Group | `100GGGG0` to `100GGGG1` | Group 0..15 |
| Broadcast | `1111111x` | All devices |
| Broadcast unaddressed | `1111110x` | Devices without short address |
| Special command | `101CCCC1` | Configuration / query commands |

Selector bit *S* (bit 0 of address byte) distinguishes "level command" (S=0) from "command opcode" (S=1) — see IEC 62386-102 §11.

### 2.4 Mapping to Phase-2 verbs

| Phase-2 op | DALI frame | Notes |
|------------|------------|-------|
| `recall scene` | `100GGGGS` + `0001 NNNN` | Scene recall NNNN |
| `set level` | `0AAAAAA0` + `0..254` | Direct arc-power |
| `query device type` | `0AAAAAA1` + `1001 1001` | DEVICE_TYPE (0x99) |
| `query status` | `0AAAAAA1` + `1001 0000` | QUERY_STATUS (0x90) |
| `identify device` | `1100 0001` + `00100101` | IDENTIFY (subset) |

A gateway MUST translate the Phase-2 *long-running operation* status into the IEC 62386-102 §10 *fading transition* timing model so that returned ETags reflect actual luminaire arc-power transitions.

---

## 3. Wireless Carriers (IEC 62386-104 / -220)

### 3.1 Carrier choice

IEC 62386-104 enumerates Thread, Bluetooth-mesh, and other 2.4 GHz / sub-GHz short-packet transports as alternative carriers. Each carrier provides:

- A *bridge entity* that exposes a logical IEC 62386-102 control gear over the air.
- A *bearer-specific frame mapping* preserving the address, command, and data fields of the wired frame.
- A *retransmission and acknowledgement layer* compatible with the bearer-native MAC layer (e.g. IEEE 802.15.4 ACK frames on Thread-bearing radios).

WIA-CITY-009 implementations are bearer-agnostic at the application layer; bearer choice is signalled through the Phase-1 `controlGear.transportProfile` field.

### 3.2 Bluetooth-mesh profile (subset of IEC 62386-104 ed. 1)

When the bearer is Bluetooth-mesh, the gateway MUST:

1. Advertise the *Light Lightness Server*, *Light CTL Server*, *Light Hue/Saturation Server*, and *Generic OnOff Server* models in the device composition data.
2. Subscribe each lighting model to a publish-address corresponding to the IEC 62386 group ID.
3. Translate every IEC 62386-102 forward frame into the equivalent Bluetooth-mesh access message.
4. Surface telemetry through the *Light LC Server* and the publish-period mechanism.

### 3.3 Thread/IPv6 profile

When the bearer is Thread (an IPv6-over-802.15.4 stack), the gateway MUST:

1. Operate as an RFC 9176 CoAP Resource Directory client.
2. Carry IEC 62386-102 frames inside CoAP POST bodies with media-type `application/cbor` and the CDDL schema published with this specification.
3. Use OSCORE (RFC 8613) for end-to-end protection of every command.

### 3.4 Reliability semantics

For wireless bearers, the application-level guarantee is at-least-once delivery with idempotent commands. *Set arc-power* and *recall scene* are idempotent; *step up / step down* MUST be wrapped with a sequence number and replayed only after explicit ETag re-read by the controller.

---

## 4. ISO/IEC 14543-3-10 Wireless Short-Packet (Energy Harvesting)

### 4.1 Use cases

Energy-harvesting input devices (push buttons, occupancy detectors, window contacts) transmit short telegrams of 6, 7, 14, or up to 28 bytes. WIA-CITY-009 maps these to *Sensor* events of Phase 1 §2.5.

### 4.2 Telegram structure (ISO/IEC 14543-3-10 §6.4)

```
[ Sync (1B) | Length (1B) | DataType (1B) | EncryptedPayload | CRC (2B per ITU-T V.41) ]
```

The *DataType* enumeration relevant to lighting is:

| DataType | Meaning | Maps to |
|----------|---------|---------|
| `RPS` (Repeated Switch) | Push-button event | `lighting:control` recall scene / level step |
| `1BS` (1-byte sensor) | Single-bit state | Occupancy ON/OFF |
| `4BS` (4-byte sensor) | Multi-byte sensor | Illuminance, temperature, humidity |
| `VLD` (Variable Length) | TEDS or large payload | ISO/IEC/IEEE 21451 TEDS |

### 4.3 Security

The high-security profile of ISO/IEC 14543-3-10 §11 (AES-128 in CCM* mode with rolling nonce) MUST be used for any deployment carrying access-control or safety-relevant events.

---

## 5. ISO/IEC 14908 Control Network Protocol Bridging

### 5.1 Building automation integration

A WIA-CITY-009 site is often part of a wider building automation network running ISO/IEC 14908-1 (control network protocol, formerly known as a published ANSI / consortium specification). Phase-3 mandates a deterministic bridge between WIA-CITY-009 entities and ISO/IEC 14908 *Functional Profile* objects.

### 5.2 Bridge object mapping

| WIA-CITY-009 entity | ISO/IEC 14908 object | Notes |
|---------------------|----------------------|-------|
| Zone (Phase 1 §2.2) | Group / Channel | Uses ISO/IEC 14908-3 channel ID |
| Luminaire | Standardized Lamp Actuator | Standard Network Variable types from ISO/IEC 14908-1 |
| Sensor | Standardized Sensor | Standard Network Variable types as above |
| Scene | Standardized Scene Controller | IEC 62386-102 scene 0..15 ↔ ISO/IEC 14908 scene index |

### 5.3 Latency budgets

End-to-end command latency from a CoAP `POST` at the management plane to an arc-power transition at a luminaire MUST satisfy:

- 95th percentile ≤ 200 ms on a fully wired DALI deployment.
- 95th percentile ≤ 500 ms when crossing a wireless bearer (Thread, BLE-mesh).

These latency budgets are normative for the §6 conformance suite.

---

## 6. Conformance Profiles

### 6.1 Wired-only profile (P-W)

A P-W gateway MUST:

- Implement IEC 62386-101 ed. 3.0 physical layer and IEC 62386-102 / -103 frame parsers.
- Pass §3 wireless mappings as a no-op (no wireless bridge).
- Pass §5 ISO/IEC 14908 bridging when the site descriptor declares a CNP integration.

### 6.2 Wireless-extended profile (P-WX)

A P-WX gateway MUST additionally:

- Implement at least one IEC 62386-104 wireless carrier (Thread or BLE-mesh).
- Maintain a 1:1 model mapping such that every IEC 62386-102 forward frame is reproduced over the air with identical address, command, and data fields.
- Provide the OSCORE protection of §3.3.

### 6.3 Energy-harvesting profile (P-EH)

A P-EH gateway MUST additionally:

- Implement ISO/IEC 14543-3-10 receive-only stack with the high-security profile.
- Maintain a sensor catalogue keyed by EnOcean Equipment Profile (EEP) reference.
- Translate received telegrams into Phase-1 *Sensor* events within 250 ms of receipt.

---

## 7. Failure Handling

### 7.1 Bus failure

A DALI bus failure (idle below 2 V or above 22.5 V for > 100 ms, per IEC 62386-101 §8.4.4) MUST raise a Phase-2 problem-detail of type `lighting/dali-bus-fault` with `severity: critical`, and the IEC 62443-3-3 SR 6.1 audit log entry MUST be written within 1 s.

### 7.2 Lost wireless bridge

If a wireless bridge loses RF link for > 30 s, the gateway MUST mark the affected control gears as `state: unreachable` and serve cached state to readers with `Cache-Control: stale-while-revalidate` per RFC 9111.

### 7.3 Emergency lighting (IEC 62386-202)

Device type 1 (self-contained emergency luminaires) MUST NOT be silenced or commanded `OFF` outside of the IEC 62386-202 §11.2 maintenance window. The gateway MUST refuse such commands with HTTP 409 / CoAP 4.09 and the problem code `lighting/emergency-protected`.

---

## 8. Time and Synchronization

Scheduling and astronomical events require time accuracy at or below 1 s. WIA-CITY-009 mandates one of:

- Network Time Security per RFC 8915 over RFC 5905 (NTPv4), or
- Precision Time Protocol per IEEE 1588-2019 mapped via gateway when the site has a PTP grandmaster (ISO/IEC 14908 PTP profile is out of scope).

Astronomical events (sunrise, sunset) are computed against the IANA Time Zone Database tz data release in force at the time of computation.

---

## 9. References

1. IEC 62386-101 ed. 3.0 — *DALI System components.*
2. IEC 62386-102 ed. 3.0 — *DALI Control gear.*
3. IEC 62386-103 ed. 2.0 — *DALI Control devices.*
4. IEC 62386-104 — *DALI Wireless / alternative wired transports.*
5. IEC 62386-202 — *DALI Self-contained emergency lighting.*
6. IEC 62386-209 — *DALI Colour control gear.*
7. IEC 62386-220 — *DALI Wireless networking subsystem.*
8. IEC 62443-3-3:2013 — *System security requirements and security levels.*
9. ISO/IEC 14543-3-10:2012 — *Wireless short-packet (WSP) protocol.*
10. ISO/IEC 14908-1:2012 — *Control network protocol — Protocol stack.*
11. ISO/IEC 14908-3:2012 — *Power line channel specification.*
12. ISO/IEC/IEEE 21451-1:2010 — *Common functions, communication protocols, and TEDS formats.*
13. ISO/IEC/IEEE 21451-5:2010 — *Wireless transducer interface.*
14. RFC 5905 — *Network Time Protocol Version 4.*
15. RFC 7252 — *The Constrained Application Protocol (CoAP).*
16. RFC 7641 — *Observing Resources in CoAP.*
17. RFC 7959 — *Block-Wise Transfers in CoAP.*
18. RFC 8613 — *Object Security for Constrained RESTful Environments (OSCORE).*
19. RFC 8915 — *Network Time Security for the Network Time Protocol.*
20. RFC 8949 — *Concise Binary Object Representation (CBOR).*
21. RFC 9111 — *HTTP Caching.*
22. RFC 9147 — *DTLS 1.3.*
23. RFC 9176 — *CoRE Resource Directory.*
24. ITU-T V.41 — *Code-independent error-control system.*
