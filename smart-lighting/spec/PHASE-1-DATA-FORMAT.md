# WIA-CITY-009 (smart-lighting) — Phase 1: Data Format Specification

> **Version:** 1.0.0
> **Status:** Official
> **Phase:** 1 of 4 (Data Format)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This document specifies the canonical data formats used by WIA-CITY-009 smart-lighting deployments. The formats describe **luminaires**, **control gear**, **scenes**, **sensor inputs**, and **telemetry events** in a way that interoperates with the IEC 62386 (DALI) family, the ISO/IEC/IEEE 21451 transducer-interface family, ISO/IEC 14543-3-10 wireless short-packet networks, ISO/IEC 14908 control-network protocol, and the IEC 60598 luminaire safety baseline.

### 1.1 Goals

- Provide a JSON / CBOR canonical encoding that is loss-free with respect to the IEC 62386 device-information model.
- Support both wired (Manchester-coded DALI bus per IEC 62386-101 ed. 3.0) and wireless transports listed in IEC 62386-104 (e.g. Thread, Bluetooth-mesh) and IEC 62386-220 (wireless).
- Carry the IEEE/ISO/IEC 21451 *Transducer Electronic Data Sheet* (TEDS) so that any compliant sensor can self-describe.
- Comply with IEC 62443-3-3 system security requirements (zoning, identifiers, integrity classes) at the data-payload layer.

### 1.2 Non-goals

- Defining a new physical layer. Physical signalling defers to IEC 62386-101.
- Defining new luminaire safety thresholds. Photobiological and electrical safety defer to IEC 60598-1 and IEC 62471.
- Defining radio-frequency channel plans. Channel plans defer to ISO/IEC 14543-3-10 §6 (sub-GHz energy-harvesting) and to the relevant national radio regulations.

---

## 2. Top-level Object Model

A WIA-CITY-009 deployment serializes into five root entities:

```
SmartLightingSite ──┬── Zone[]
                    ├── Luminaire[]
                    ├── ControlGear[]
                    ├── Sensor[]
                    └── Scene[]
```

Each entity has a **stable URI** (RFC 3986 syntax) of the form
`wia-lighting://<site-id>/<entity-type>/<entity-id>` where `<entity-type>` ∈ {`zone`, `luminaire`, `cg`, `sensor`, `scene`}.

### 2.1 Site

```json
{
  "type": "SmartLightingSite",
  "version": "1.0.0",
  "siteId": "<UUID v4 per RFC 9562>",
  "operator": {
    "name": "string",
    "iec62443ZoneId": "string"
  },
  "geographicScope": {
    "iso6709": "+37.5326-126.9905/",
    "altitudeMeters": "number"
  },
  "createdAt": "<RFC 3339 date-time>",
  "updatedAt": "<RFC 3339 date-time>"
}
```

The `iec62443ZoneId` field carries the cyber-security zone identifier defined by IEC 62443-3-3 §SR 5.1 (Network Segmentation). The `iso6709` field follows ISO 6709:2008 string representation of latitude/longitude/altitude.

### 2.2 Zone

A *Zone* is a logical group of luminaires that share a control intent. Zones map cleanly onto IEC 62386 *groups* (0–15 short addresses) and onto ISO/IEC 14908-1 *channel domains*.

```json
{
  "type": "Zone",
  "zoneId": "string",
  "name": {"en": "Lobby", "ko": "로비"},
  "parentZoneId": "string|null",
  "boundary": {
    "indoor": "boolean",
    "iso19115Geometry": "<GeoJSON Point|Polygon|MultiPolygon>"
  },
  "controlPolicy": "OCC|TIME|LUX|HCL|MANUAL",
  "iec62386GroupId": "uint8 (0..15) | null"
}
```

`controlPolicy` enumerations:

| Code | Meaning | Reference |
|------|---------|-----------|
| `OCC` | Occupancy-driven | ISO/IEC/IEEE 21451-1 transducer model |
| `TIME` | Schedule / astronomical clock | ISO 8601-1 calendar; sunrise/sunset per local time-zone database |
| `LUX` | Closed-loop daylight harvesting | IEC 62386-303 (occupancy / lux input devices, where applicable) |
| `HCL` | Human-centric / circadian | IEC 62386-209 colour-control gear; CIE colorimetry |
| `MANUAL` | Manual override | IEC 62386-301 push-button input device |

### 2.3 Luminaire

A *Luminaire* combines one IEC 60598 lamp assembly with one or more IEC 62386 control gears.

```json
{
  "type": "Luminaire",
  "luminaireId": "string",
  "manufacturer": "string",
  "model": "string",
  "iec60598Class": "I|II|III",
  "iec60598PhotobiologicalRiskGroup": "0|1|2|3",
  "rated": {
    "voltageVolts": "number",
    "currentAmperes": "number",
    "powerWatts": "number",
    "luminousFluxLumens": "number",
    "ccTKelvin": "number|range",
    "criRa": "number"
  },
  "controlGearRefs": ["<ControlGear URI>"],
  "ingressProtection": "IP20|IP44|IP65|IP67",
  "ipRatingBasis": "IEC 60529:2013"
}
```

The `iec60598PhotobiologicalRiskGroup` field carries the IEC 62471:2006 *Photobiological safety of lamps and lamp systems* exempt-to-RG3 classification.

### 2.4 ControlGear

A ControlGear is the IEC 62386-102 (control gear) entity. Wired gear behaves per the IEC 62386-101 ed. 3.0 physical layer (Manchester coding, 1200 baud, ±16 V differential bus). Wireless gear behaves per IEC 62386-104 / -220.

```json
{
  "type": "ControlGear",
  "controlGearId": "string",
  "deviceType": "0|1|2|3|4|5|6|7|8|9|10|...",
  "deviceTypeLabel": "<as IEC 62386-2xx series>",
  "addressing": {
    "shortAddress": "uint8 (0..63) | null",
    "groups": "uint16-bitmask",
    "scenes": "uint16-bitmask"
  },
  "transport": "WIRED|WIRELESS",
  "transportProfile": "IEC62386-101|IEC62386-104|IEC62386-220|IEC62386-104+THREAD|IEC62386-104+BTMESH",
  "memoryBanks": {
    "bank0": "<base64 raw bytes per IEC 62386-102 §4.3>",
    "bank1": "<base64 raw bytes per IEC 62386-102 §4.4>"
  },
  "supportedExtensions": ["209", "207", "202"]
}
```

`deviceType` integer values follow the IEC 62386 ed. mapping; for example `1` denotes self-contained emergency lighting (Part 202), `8` denotes colour control gear (Part 209), and `2` denotes discharge lamps (Part 203).

### 2.5 Sensor

Sensors carry an embedded TEDS object so that any IEEE/ISO/IEC 21451-compliant transducer is self-describing.

```json
{
  "type": "Sensor",
  "sensorId": "string",
  "modality": "OCCUPANCY|ILLUMINANCE|COLOR_TEMP|MOTION|CO2|TEMPERATURE|HUMIDITY|VOC",
  "transport": "WIRED|WIRELESS",
  "transportProfile": "IEC62386-303|ISOIEC14543-3-10|ISOIEC14908",
  "teds": {
    "metaTedsId": "uint8 (per ISO/IEC/IEEE 21451-2 §6.3)",
    "physicalUnits": "<SI units encoded per ISO/IEC/IEEE 21451-2 Table 7>",
    "uncertainty": "number (std. dev., per JCGM 100:2008 GUM)",
    "samplingPeriodSeconds": "number"
  }
}
```

Energy-harvesting wireless sensors (e.g. EnOcean) follow ISO/IEC 14543-3-10:2012 *Wireless short-packet (WSP) protocol optimized for energy harvesting* §5 link layer and §6 application layer.

### 2.6 Scene

A *Scene* is a deterministic recall state, mapped 1-to-1 onto IEC 62386 scenes 0–15.

```json
{
  "type": "Scene",
  "sceneId": "string",
  "iec62386SceneNumber": "uint8 (0..15)",
  "members": [
    {
      "luminaireId": "string",
      "level": "uint8 (0..254 per IEC 62386-102)",
      "ccTKelvin": "uint16 (per IEC 62386-209)",
      "fadeTime": "uint8 (0..15 per IEC 62386-102 §9.5)"
    }
  ]
}
```

The `level` value 255 (`MASK`) carries the "do not change" semantics defined in IEC 62386-102 §11.3.

---

## 3. Canonical Encodings

WIA-CITY-009 mandates two interchangeable encodings:

1. **JSON** — RFC 8259, UTF-8, NFC-normalized strings (Unicode 15.0).
2. **CBOR** — RFC 8949, with the *Concise Data Definition Language* (RFC 8610) schema published alongside this specification.

Both encodings MUST be byte-equivalent under round-trip via the canonical CBOR deterministic encoding rules (RFC 8949 §4.2). Implementations MAY use *CBOR Object Signing and Encryption* (COSE, RFC 9052/9053) for end-to-end authenticity at the data-payload layer.

### 3.1 Time

All timestamps use RFC 3339 *date-time* form with a time-zone offset. Astronomical events (sunrise, sunset, civil twilight) are computed against a local civil-time reference and serialized in the same RFC 3339 form. CBOR encodings MAY use tag 0 (RFC 8949 §3.4.1) for date-time.

### 3.2 Identifiers

All entity identifiers use RFC 9562 (UUID) version 4 unless a hardware-bound identifier is required, in which case version 5 (name-based, SHA-1) over the IEC 62386-102 *random address* (memory bank 0, bytes 0x04–0x06) is used.

### 3.3 Numeric units

All physical quantities use SI base units (metre, second, candela, kelvin, etc.) per ISO/IEC 80000-1:2009. Photometric quantities follow ISO/IEC 80000-7:2019 *Light and radiation*. CBOR encodings carry the unit as an explicit tag using IANA-registered numeric tags only.

---

## 4. Versioning and Compatibility

### 4.1 Schema version

The `version` field in every root entity uses semantic versioning (semver 2.0.0) and is independent of the IEC 62386 part edition. A WIA-CITY-009 v1.x payload MUST be parseable by any v1.y implementation where y ≥ x. Breaking changes increment the major version.

### 4.2 Edition mapping

This Phase-1 specification has been authored against:

- IEC 62386-101 ed. 3.0 (2024) — System components
- IEC 62386-102 ed. 3.0 (2024) — Control gear
- IEC 62386-103 ed. 2.0 (2024) — Control devices
- IEC 62386-104 ed. 1.0 — Wireless and alternative wired transports
- IEC 62386-209 ed. 1.x — Colour control
- ISO/IEC/IEEE 21451-1:2010 — Common functions
- ISO/IEC/IEEE 21451-2:2010 — Wired transducer interface
- ISO/IEC/IEEE 21451-5:2010 — Wireless transducer interface
- ISO/IEC 14543-3-10:2012 — Wireless short-packet protocol
- ISO/IEC 14908-1:2012 — Control network protocol
- IEC 62443-3-3:2013 — System security requirements

Future IEC 62386 part publications are pulled in by reference; implementations SHOULD report the effective edition in the `controlGear.transportProfile` field.

---

## 5. Examples

### 5.1 Daylight-harvesting office zone

```json
{
  "type": "Zone",
  "zoneId": "lobby-east",
  "name": {"en": "Lobby (east)", "ko": "동측 로비"},
  "boundary": {"indoor": true, "iso19115Geometry": "<GeoJSON omitted>"},
  "controlPolicy": "LUX",
  "iec62386GroupId": 3
}
```

### 5.2 Circadian (HCL) member luminaire

```json
{
  "type": "Luminaire",
  "luminaireId": "lum-1015",
  "manufacturer": "<vendor>",
  "model": "<vendor model>",
  "iec60598Class": "I",
  "iec60598PhotobiologicalRiskGroup": "1",
  "rated": {
    "voltageVolts": 230,
    "currentAmperes": 0.18,
    "powerWatts": 41.4,
    "luminousFluxLumens": 4200,
    "ccTKelvin": [2700, 6500],
    "criRa": 90
  },
  "controlGearRefs": ["wia-lighting://hq/cg/cg-1015a"],
  "ingressProtection": "IP44",
  "ipRatingBasis": "IEC 60529:2013"
}
```

---

## 6. References

1. IEC 62386-101 ed. 3.0 — *Digital addressable lighting interface — Part 101: General requirements — System components.*
2. IEC 62386-102 ed. 3.0 — *Part 102: General requirements — Control gear.*
3. IEC 62386-103 ed. 2.0 — *Part 103: General requirements — Control devices.*
4. IEC 62386-104 — *Part 104: General requirements — Wireless and alternative wired system components.*
5. IEC 62386-209 — *Part 209: Particular requirements for control gear — Colour control (device type 8).*
6. ISO/IEC/IEEE 21451-1:2010 — *Common functions, communication protocols, and Transducer Electronic Data Sheet (TEDS) formats.*
7. ISO/IEC/IEEE 21451-2:2010 — *Transducer to microprocessor communication protocols.*
8. ISO/IEC/IEEE 21451-5:2010 — *Wireless communication protocols and Transducer Electronic Data Sheet (TEDS) formats.*
9. ISO/IEC 14543-3-10:2012 — *Information technology — Home electronic systems — Wireless short-packet protocol.*
10. ISO/IEC 14908-1:2012 — *Information technology — Control network protocol — Part 1: Protocol stack.*
11. IEC 62443-3-3:2013 — *Industrial communication networks — Network and system security — System security requirements and security levels.*
12. IEC 60598-1:2020 — *Luminaires — Part 1: General requirements and tests.*
13. IEC 62471:2006 — *Photobiological safety of lamps and lamp systems.*
14. IEC 60529:2013 — *Degrees of protection provided by enclosures (IP Code).*
15. ISO 6709:2008 — *Standard representation of geographic point location by coordinates.*
16. ISO 8601-1:2019 — *Date and time — Representations for information interchange.*
17. ISO/IEC 80000-1:2009; ISO/IEC 80000-7:2019 — *Quantities and units.*
18. RFC 3339 — *Date and Time on the Internet.*
19. RFC 3986 — *Uniform Resource Identifier (URI): Generic Syntax.*
20. RFC 8259 — *The JavaScript Object Notation (JSON) Data Interchange Format.*
21. RFC 8610 — *Concise Data Definition Language (CDDL).*
22. RFC 8949 — *Concise Binary Object Representation (CBOR).*
23. RFC 9052; RFC 9053 — *CBOR Object Signing and Encryption (COSE).*
24. RFC 9562 — *Universally Unique IDentifiers (UUIDs).*
