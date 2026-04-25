# WIA Livestock Tracking — Phase 3 Protocol Specification

**Standard:** WIA-AGRI-009
**Version:** 1.1.0
**Status:** ✅ Deep-Published
**Last updated:** 2026-04-25
**Conforms to:** ISO 11784/11785 (LF FDX-B), ISO 14223 (advanced transponders), ISO 24631 (conformance + performance), GS1 EPC Gen2 v2 (UHF), MQTT 5 (OASIS, 2019), CoAP (RFC 7252) + OSCORE (RFC 8613), LoRaWAN 1.1, Bluetooth 5.4 Mesh, TLS 1.3 (RFC 8446), Hyperledger Fabric 2.5, ICAR ADE 1.7.4, OIE WOAH WAHIS 2.0.

---

## 1. Communication protocol matrix

| Protocol | Tier             | Range       | Use case                            | Power profile |
|----------|------------------|-------------|-------------------------------------|---------------|
| LF RFID 134.2 kHz (ISO 11784/11785 FDX-B) | Identification | 0–30 cm | Race-side reader, slaughter scanner | Reader powered |
| UHF RFID 860–960 MHz (EPC Gen2)          | Identification | 1–10 m | Yard/race batch scan | Reader powered |
| BLE 5.4 Mesh                               | Local sensor mesh | 0–100 m | Collar ↔ farm gateway | Coin-cell years |
| LoRaWAN 1.1 (KR920 / EU868 / AU915)        | Pasture telemetry | 2–15 km | GPS collar uplink, remote farms | Single CR2 cell, multi-year |
| LTE-M / NB-IoT                             | Carrier-backed | National | Transport vehicle telematics | Rechargeable |
| MQTT 5 over TLS 1.3                        | Backend         | Internet | Real-time fan-out | Server |
| CoAP + OSCORE (RFC 8613)                   | Constrained     | Internet | Battery-bound bolus / ear-tag | Tiny |
| HTTPS / REST                                | Backend         | Internet | API + EPCIS capture | Server |

---

## 2. RFID standards in depth

### 2.1 ISO 11784/11785 (LF, mandatory)

134.2 kHz, FDX-B encoding, 64-bit identifier:

```
Bit 0     : Animal flag (1 = animal application)
Bit 1-15  : Reserved
Bit 16-26 : Manufacturer / national bureau code (ICAR-allocated)
Bit 27-37 : Country code (ISO 3166-1 numeric, "410" for Korea)
Bit 38-65 : Unique national identifier (12-digit)
```

A typical Korea-issued tag: <code>410-000123456789</code> — country `410`, manufacturer-allocated 12-digit serial. ISO 24631-2 specifies the conformance test; ISO 24631-3 specifies the performance test (read range, transmission integrity).

ICAR-managed manufacturer code allocations are listed at <https://www.icar.org/Documents/Codes/manufacturer.pdf>.

### 2.2 ISO 14223 (advanced transponders)

ISO 14223 defines bidirectional LF transponders with on-tag memory (up to 2 KB), allowing on-tag storage of veterinary events when network connectivity is intermittent (mountain pasture, transit). The wire is identical to ISO 11784/11785 for backward compatibility with installed readers.

### 2.3 EPC Gen2 v2 UHF

860–960 MHz (region-specific: KR866, EU868, US915). EPC binary encoding for SGTIN-198 lets a herd-scale read happen in one drive-by:

```
EPC: urn:epc:id:sgtin:8801234.056789.001234
URI: https://id.gs1.org/01/08801234567896
```

UHF complements LF — UHF reads at gate scale, LF resolves the canonical animal identifier.

---

## 3. MQTT 5 backend

### 3.1 Topic structure

```
livestock/{tenant}/{holdingId}/animal/{animalId}/location
livestock/{tenant}/{holdingId}/animal/{animalId}/health
livestock/{tenant}/{holdingId}/animal/{animalId}/welfare
livestock/{tenant}/{holdingId}/animal/{animalId}/alerts
livestock/{tenant}/{holdingId}/herd/summary
livestock/{tenant}/{holdingId}/gateway/{gatewayId}/state
```

### 3.2 Message format (CBOR-encoded JSON shape)

```json
{
  "deviceId":"DEV-COLLAR-7AA","ts":"2026-04-25T14:22:00+09:00",
  "data":{"lat":33.4996,"lng":126.5312,"hdop":1.2,
           "activity":"GRAZING","stepCount":4321,
           "ruminationMin":127,"bodyTempC":38.8},
  "battery":{"voltageMv":3015,"percent":78,"firmware":"1.4.2"}
}
```

### 3.3 QoS strategy

| QoS | Use case                      | Rationale                                        |
|-----|-------------------------------|--------------------------------------------------|
| 0   | 60 s heartbeat                | Cheap; broker tracks last-seen                   |
| 1   | Location, activity, body temp | At-least-once; dedup by (animalId, ts)           |
| 2   | Disease alerts, geofence exit | Exactly-once; loss not acceptable                |

### 3.4 MQTT 5 features the standard depends on

- **Session Expiry Interval** — collars surviving multi-day pasture cycles keep their broker session.
- **Payload Format Indicator** — separates CBOR sensor frames from JSON dashboards.
- **Subscription Identifier** — multiplexes per-herd filters per gateway.
- **AUTH packet + Reason Codes** — surfaces enhanced-auth failures in gateway logs.

---

## 4. LoRaWAN 1.1 telemetry

### 4.1 Device profile

```json
{
  "dev_eui":"0018B20000000001",
  "join_eui":"70B3D57ED0000001",
  "nwk_s_enc_key":"<rotated per session, LoRaWAN 1.1>",
  "frequency_plan":"KR920","class":"A","data_rate":"SF7BW125",
  "fcnt_resets_allowed":false
}
```

### 4.2 Uplink payload (12 bytes, GPS frame)

```
Byte 0     : Message type (0x01 = GPS, 0x02 = activity, 0x03 = vitals)
Byte 1-4   : Latitude (signed int32, scaled by 1e7)
Byte 5-8   : Longitude (signed int32, scaled by 1e7)
Byte 9-10  : Altitude (signed int16, metres)
Byte 11    : Battery (uint8, percent)

Example bytes: 01 16 5A 0E 4B 78 9A BC DE 01 F4 4E
              type lat-int32        lng-int32   alt   batt
```

### 4.3 Downlink commands

```
0x02 <interval-uint16 minutes>   # change uplink interval
0x03 <on/off>                    # activate buzzer / vibration
0x10 <new-firmware-version>      # OTA pointer
```

ChirpStack 4 / The Things Stack v3 servers decode and re-publish to the MQTT topic tree from §3.

---

## 5. Bluetooth 5.4 Mesh (farm-yard mesh)

BLE 5.4 mesh handles ear-tag temperature sensors that need ~50 m range and decade-long batteries. Mesh nodes self-organise; gateway nodes bridge to MQTT. Provisioning uses the OOB QR code printed on the ear tag.

---

## 6. CoAP + OSCORE for boluses

Rumen boluses (e.g., Allflex SmartBolus) operate from a single coin cell for 5+ years. CoAP (RFC 7252) over UDP plus OSCORE (RFC 8613) deliver authenticated, encrypted packets at &lt; 200 byte overhead.

```
CON POST /animal/KOR-01-123456789/vitals
Content-Format: application/cbor
OSCORE: kid=0x01,iv=0x...
Payload (CBOR): {"t":38.9,"u":"Cel","ts":1714032668}
```

Block-wise transfer (RFC 7959) carries multi-hour temperature profiles without IP fragmentation.

---

## 7. Edge inference (TinyML on the collar)

Modern collars run on-device classifiers (gait → grazing/ruminating/walking; rumination minutes; oestrus detection). Recommended runtime: TensorFlow Lite Micro on Cortex-M4F. The classifier output is a small label + confidence score sent over LoRaWAN, drastically reducing payload vs raw 6-axis IMU traces.

```c
// Pseudocode — classifier label byte
uint8_t label = run_classifier(imu_window_64);
// 0=resting 1=grazing 2=ruminating 3=walking 4=oestrus 5=lameness
```

---

## 8. Data security baseline

| Layer                | Required                                                                  |
|----------------------|---------------------------------------------------------------------------|
| Backend transport    | TLS 1.3 with TLS_AES_256_GCM_SHA384 or TLS_CHACHA20_POLY1305_SHA256        |
| Edge transport       | OSCORE on CoAP, AES-128-CCM on LoRaWAN                                    |
| Token format         | EdDSA JWT, audience-restricted, 30 min lifetime, DPoP-bound (RFC 9449)    |
| Application signing  | Ed25519 (RFC 8032) over RFC 8785 canonical JSON                           |
| Key management       | HSM-backed, RFC 7517 JWKS rotation, 90-day max                            |
| RFID tag spoofing    | Mitigated via challenge-response on ISO 14223 advanced tags               |

---

## 9. Blockchain anchoring

### 9.1 Hyperledger Fabric chaincode

```solidity
// Solidity equivalent for WIA Chain (PoA) anchoring
pragma solidity ^0.8.20;

contract LivestockRegistry {
    struct Animal {
        string animalId;
        string rfidTag;
        uint64 birthDate;
        address owner;
        bytes32 currentHash;
    }
    mapping(bytes32 =&gt; Animal) public animals;
    event AnimalRegistered(string animalId, address owner);
    event AnimalTransferred(string animalId, address from, address to);
    event AnimalSlaughtered(string animalId, uint64 atUnix);

    function registerAnimal(string memory id, string memory rfid, uint64 birthDate) external {
        bytes32 key = keccak256(bytes(id));
        require(animals[key].birthDate == 0, "exists");
        animals[key] = Animal(id, rfid, birthDate, msg.sender, bytes32(0));
        emit AnimalRegistered(id, msg.sender);
    }

    function updateHash(string memory id, bytes32 hash) external {
        bytes32 key = keccak256(bytes(id));
        require(animals[key].owner == msg.sender, "not owner");
        animals[key].currentHash = hash;
    }

    function transfer(string memory id, address to) external {
        bytes32 key = keccak256(bytes(id));
        require(animals[key].owner == msg.sender, "not owner");
        emit AnimalTransferred(id, msg.sender, to);
        animals[key].owner = to;
    }
}
```

Endorsement policy on Fabric: <code>AND('Producers.member','Veterinarians.member','Authority.member')</code> — no single party can rewrite history.

### 9.2 PoA + BSC anchoring

WIA Chain (PoA) anchors a Merkle root of every 30-second ingest window onto BSC mainnet, giving public auditors a one-click attestation without exposing private veterinary detail.

---

## 10. Interoperability

### 10.1 GS1 EPCIS for cross-org events

```xml
<EPCISDocument>
  <EPCISBody><EventList>
    <ObjectEvent>
      <eventTime>2026-04-25T14:22:00Z</eventTime>
      <epcList><epc>urn:icar:animal:kor.01.123456789</epc></epcList>
      <action>OBSERVE</action>
      <bizStep>https://ref.gs1.org/cbv/BizStep-arriving</bizStep>
      <readPoint><id>urn:epc:id:sgln:880456.70000.0</id></readPoint>
    </ObjectEvent>
  </EventList></EPCISBody>
</EPCISDocument>
```

### 10.2 ICAR ADE 1.7.4 endpoints

ICAR Animal Data Exchange covers genealogy, productivity, lactation, reproduction, conformation, body condition, and weighing. Endpoints follow the ADE REST profile; Lely / DeLaval / CRV / Allflex consume natively.

### 10.3 OIE WOAH WAHIS 2.0

Notifiable disease events route to WAHIS 2.0 (<https://wahis.woah.org/>) within 24 h per OIE Terrestrial Animal Health Code Chapter 1.1. The WIA gateway signs the payload with the National Designated Authority's key.

### 10.4 Korea regulatory integration

| System                              | Endpoint                        | Trigger                              |
|-------------------------------------|---------------------------------|--------------------------------------|
| 축산물이력제 (MAFRA)               | `POST /mafra/v1/livestock/...` | Birth, movement, slaughter           |
| 가축전염병예방법 신고             | `POST /mafra/v1/disease/notify`| Notifiable disease (FMD, AI, ASF)    |
| 축평원 (KAPE) 한우 등급             | `POST /kape/v1/grading`        | Slaughter grading event              |
| 농림축산검역본부 (APQA)            | `POST /apqa/v1/import-export`  | Import/export inspection             |

---

## 11. Failure modes the protocol layer must handle

- **Collar offline (mountain pasture)** — buffer 24 h locally; replay with original timestamps when reconnecting.
- **Late events** — use `cache/...` topic prefix per WIS-style convention; do NOT overwrite `origin/...`.
- **Clock drift** — NTP every 6 h; ingest rejects readings with `|now − ts| &gt; 600 s` (livestock cycles tolerate 10-min lag).
- **RFID collision in race** — anti-collision per ISO 18000-63 Q-protocol; up to 1000 reads/s.
- **Sensor swap** — emit EPCIS `AssociationEvent` so audit trail reconciles old → new device id.
- **Replay attacks** — OSCORE sequence-number window + JWT JTI cache.

---

**弘益人間 — Benefit All Humanity**
*WIA-AGRI-009 · © 2026 MIT License*
