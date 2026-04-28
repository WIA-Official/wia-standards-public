# WIA-CITY-014 (security-system-city) — Phase 1: Data Format Specification

> **Version:** 1.0.0
> **Status:** Official
> **Phase:** 1 of 4 (Data Format)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This document specifies the canonical data formats used by the WIA-CITY-014 security-system-city deployments. The formats describe **video surveillance subsystems**, **electronic access-control subsystems**, **alarm and intrusion-detection subsystems**, and the **integrated event bus** that ties them together. The formats interoperate with the IEC 62676 series (video surveillance), the IEC 60839 series (electronic access-control systems), and the ISO/IEC 27000-family information-security management baseline.

### 1.1 Goals

- Provide a JSON / CBOR canonical encoding that captures every entity referenced by IEC 62676-1 *operational requirements* and IEC 62676-2 *video transmission requirements*.
- Carry the *operational performance categories* introduced by IEC 62676-4:2025 (overview, outline, discern, perceive, characterize, validate, scrutinize) as first-class properties on each viewpoint.
- Preserve the IEC 60839-11-1 *electronic access-control system requirements* model for doors, readers, credentials, and access points.
- Encode the chain-of-custody fields required by ISO/IEC 27037 *Guidelines for identification, collection, acquisition and preservation of digital evidence*.

### 1.2 Non-goals

- Defining new image-quality metrics. Pixel-density and DORI-style classifications defer to IEC 62676-4.
- Defining new compression. Video codec semantics defer to ISO/IEC 23008-2 (HEVC) and ISO/IEC 14496-10 (AVC).
- Defining cryptographic primitives beyond reference. Cryptography defers to ISO/IEC 18033-3 and FIPS 197/180-4.

---

## 2. Top-level Object Model

A WIA-CITY-014 site serializes into seven root entities:

```
SecuritySite ──┬── Zone[]
               ├── Camera[]
               ├── AccessPoint[]
               ├── Sensor[]
               ├── Alarm[]
               ├── Recorder[]
               └── Operator[]
```

Each entity has a stable URI of the form `wia-sec://<site-id>/<entity-type>/<entity-id>`.

### 2.1 SecuritySite

```json
{
  "type": "SecuritySite",
  "version": "1.0.0",
  "siteId": "<UUID v4 per RFC 9562>",
  "operator": {
    "name": "string",
    "iso27001Scope": "string",
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

The `iso27001Scope` field carries the boundary statement of the operator's ISO/IEC 27001 information-security management system.

### 2.2 Zone

```json
{
  "type": "Zone",
  "zoneId": "string",
  "name": {"en": "Lobby", "ko": "로비"},
  "parentZoneId": "string|null",
  "boundary": {
    "indoor": "boolean",
    "iso19115Geometry": "<GeoJSON omitted>"
  },
  "iec60839GradingClass": "1|2|3|4",
  "operationalRequirement": "OBSERVE|RECOGNIZE|IDENTIFY|INSPECT"
}
```

The `iec60839GradingClass` field expresses the security grading class defined in IEC 60839-11-1 §5 *Grading*. The `operationalRequirement` enumeration aligns with IEC 62676-4 §5 video-surveillance operational requirements.

### 2.3 Camera

A *Camera* is a surveillance image-capture device. Its descriptor carries enough metadata for an integrator to verify operational performance against IEC 62676-4 categories.

```json
{
  "type": "Camera",
  "cameraId": "string",
  "manufacturer": "string",
  "model": "string",
  "sensor": {
    "resolutionPixels": {"width": "uint", "height": "uint"},
    "pixelPitchMicrometers": "number",
    "frameRateMaxFps": "number"
  },
  "lens": {
    "focalLengthMm": "number",
    "fNumber": "number",
    "irisType": "FIXED|MANUAL|DC|P-IRIS"
  },
  "codec": {
    "profile": "AVC-Main|AVC-High|HEVC-Main|HEVC-Main10",
    "container": "MP4|MKV|RTP",
    "isoIecReference": "ISO/IEC 14496-10|ISO/IEC 23008-2"
  },
  "operationalCategoryAchieved": "OBSERVE|RECOGNIZE|IDENTIFY|INSPECT",
  "ip": {
    "ingressProtection": "IP66|IP67",
    "ipRatingBasis": "IEC 60529:2013"
  },
  "transport": "RTP|HLS|DASH|MJPEG",
  "transportProfile": "IEC 62676-2-1|IEC 62676-2-2|IEC 62676-2-3"
}
```

The `operationalCategoryAchieved` value MUST be supported by a camera-by-zone calibration record verifiable per IEC 62676-4 §6 procedure.

### 2.4 AccessPoint

An *AccessPoint* models a door or barrier under electronic access control per IEC 60839-11-1.

```json
{
  "type": "AccessPoint",
  "accessPointId": "string",
  "doorClass": "internal|external|emergency-egress",
  "iec60839Grade": "1|2|3|4",
  "readers": [
    {
      "readerId": "string",
      "modality": "RFID|BLE|NFC|BIOMETRIC|MULTI",
      "credentialFormat": "ISO 14443|ISO/IEC 7816|ISO/IEC 18092|ISO/IEC 19794"
    }
  ],
  "lockType": "MAGNETIC|ELECTRIC-STRIKE|MOTORIZED",
  "egressDevice": "REQUEST-TO-EXIT|TURNSTILE|MANTRAP",
  "fireSafetyLink": "FAIL-SAFE|FAIL-SECURE",
  "openTimeBudgetMs": "uint",
  "alarms": ["FORCED-OPEN", "HELD-OPEN", "TAMPER", "DENY"]
}
```

Biometric-modality readers MUST attach an ISO/IEC 19794 record reference describing the biometric data interchange format used.

### 2.5 Sensor

```json
{
  "type": "Sensor",
  "sensorId": "string",
  "modality": "PIR|GLASS-BREAK|VIBRATION|MAGNETIC|SMOKE|CO|GAS|TEMP|HUMIDITY",
  "iec60839ComponentClass": "I|II|III|IV",
  "transport": "WIRED|WIRELESS",
  "transportProfile": "IEC 60839-5-x|IEC 60839-7-x|ISO/IEC 14543-3-10",
  "lifecycle": {
    "installDate": "<RFC 3339 date>",
    "maintenanceIntervalDays": "uint",
    "lastVerified": "<RFC 3339 date-time>"
  }
}
```

The `iec60839ComponentClass` field follows the component classification of the IEC 60839-1 *General requirements* for alarm systems.

### 2.6 Alarm

```json
{
  "type": "Alarm",
  "alarmId": "string",
  "severity": "INFO|MINOR|MAJOR|CRITICAL",
  "category": "INTRUSION|FIRE|LIFE-SAFETY|TAMPER|CYBER",
  "originRefs": ["<Sensor URI>", "<AccessPoint URI>", "<Camera URI>"],
  "raisedAt": "<RFC 3339 date-time>",
  "acknowledgedAt": "<RFC 3339 date-time | null>",
  "clearedAt": "<RFC 3339 date-time | null>",
  "evidence": {
    "videoClipUri": "<URI | null>",
    "cohExportUri": "<URI per ISO/IEC 27037 §7 chain-of-custody export | null>"
  }
}
```

### 2.7 Recorder

A *Recorder* is the storage and retention engine, typically a network video recorder (NVR) or a video-management system (VMS).

```json
{
  "type": "Recorder",
  "recorderId": "string",
  "capacityTerabytes": "number",
  "retentionDays": "uint",
  "encoding": "ISO/IEC 23008-2|ISO/IEC 14496-10",
  "encryptionAtRest": {
    "algorithm": "AES-256-GCM|AES-128-GCM",
    "keyProvenance": "ISO 11770-2|ISO 11770-3"
  },
  "iec62676Reference": "IEC 62676-2-3"
}
```

### 2.8 Operator

An *Operator* is a human or service principal that interacts with the system. Identity is asserted through the federated-identity layer of Phase 4.

```json
{
  "type": "Operator",
  "operatorId": "string",
  "role": "VIEWER|GUARD|SUPERVISOR|ADMINISTRATOR|AUDITOR",
  "iso27001RoleRef": "string",
  "trainingRecord": {
    "iso30401Reference": "string",
    "completedAt": "<RFC 3339 date-time>"
  }
}
```

---

## 3. Canonical Encodings

WIA-CITY-014 mandates two interchangeable encodings:

1. **JSON** — RFC 8259, UTF-8, NFC-normalized strings.
2. **CBOR** — RFC 8949, with the *Concise Data Definition Language* (RFC 8610) schema published alongside.

Both encodings MUST be byte-equivalent under round-trip via the canonical CBOR deterministic encoding rules (RFC 8949 §4.2). End-to-end protection of payloads uses *CBOR Object Signing and Encryption* (COSE, RFC 9052/9053).

### 3.1 Time

All timestamps use RFC 3339 *date-time*. Recording timestamps MUST be slaved to a time source with documented accuracy ≤ 50 ms relative to UTC; the recommended sources are NTPv4 with NTS (RFC 5905, RFC 8915) or PTP (IEEE 1588 — informative reference where applicable).

### 3.2 Identifiers

UUID v4 (RFC 9562) is the default. Recorders and cameras MAY additionally surface their hardware-bound identifier as RFC 9562 v5 over the IEC 62676-2 device-MAC reference.

### 3.3 Spatial

GeoJSON is conveyed as ISO 19115-1 metadata reference; geometric primitives follow ISO 19107 *Spatial Schema* informal mapping (informative).

### 3.4 Image and video

Image-quality metrics MUST be expressed using the IEC 62676-4 *operational performance category* labels and the supporting pixel-density bands; the latter are computed in pixels per metre in the principal scene plane and stored with the camera descriptor.

---

## 4. Privacy Controls

Personal data captured by cameras, biometric readers, or behavioural sensors MUST be classified per the operator's data-protection regime. The data format reserves the following machine-readable fields:

- `Camera.privacyMaskRegions`: array of polygons in normalised image coordinates that the recorder MUST blank during retention.
- `AccessPoint.biometricRetentionDays`: integer, default 0 — biometric templates SHOULD NOT be retained beyond a single transaction unless an explicit lawful-basis record is attached.
- `Recorder.dataSubjectAccessEndpoint`: URI for subject-access requests.

ISO/IEC 27701 *Privacy Information Management* extensions may be referenced from the same record.

---

## 5. Versioning and Compatibility

The `version` field uses semver 2.0.0. Phase 1 v1.x payloads are backwards-compatible within the major version. Edition references for normative work were:

- IEC 62676-1:2013 — System requirements.
- IEC 62676-2-1:2013 — Video transmission protocol.
- IEC 62676-2-3:2013 — RTP-based interoperability.
- IEC 62676-4:2025 — Application guidelines (operational performance categories).
- IEC 62676-5-1:2024 — Image quality.
- IEC 60839-11-1:2013 — Electronic access-control systems.
- IEC 60839-1:2014 — Alarm and electronic security systems — General requirements.
- ISO/IEC 27001:2022 — Information security management systems — Requirements.
- ISO/IEC 27002:2022 — Information security controls.
- ISO/IEC 27037:2012 — Guidelines for digital-evidence handling.
- ISO/IEC 27701:2019 — PIM extension.

---

## 6. Examples

### 6.1 IDENTIFY-class lobby camera

```json
{
  "type": "Camera",
  "cameraId": "cam-lobby-01",
  "manufacturer": "<vendor>",
  "model": "<vendor model>",
  "sensor": {"resolutionPixels": {"width": 3840, "height": 2160}, "pixelPitchMicrometers": 1.45, "frameRateMaxFps": 30},
  "lens": {"focalLengthMm": 4.2, "fNumber": 1.6, "irisType": "P-IRIS"},
  "codec": {"profile": "HEVC-Main", "container": "RTP", "isoIecReference": "ISO/IEC 23008-2"},
  "operationalCategoryAchieved": "IDENTIFY",
  "ip": {"ingressProtection": "IP66", "ipRatingBasis": "IEC 60529:2013"},
  "transport": "RTP",
  "transportProfile": "IEC 62676-2-3"
}
```

### 6.2 Grade-3 perimeter intrusion sensor

```json
{
  "type": "Sensor",
  "sensorId": "pir-perim-04",
  "modality": "PIR",
  "iec60839ComponentClass": "III",
  "transport": "WIRED",
  "transportProfile": "IEC 60839-5-x",
  "lifecycle": {
    "installDate": "2026-04-01",
    "maintenanceIntervalDays": 180,
    "lastVerified": "2026-04-21T09:11:00+09:00"
  }
}
```

---

## 7. References

1. IEC 62676-1:2013 — *Video surveillance systems for use in security applications — System requirements.*
2. IEC 62676-2-1:2013 — *Part 2-1: Video transmission protocols — General.*
3. IEC 62676-2-3:2013 — *Part 2-3: Video transmission protocols — IP interoperability based on RTP services.*
4. IEC 62676-4:2025 — *Part 4: Application guidelines.*
5. IEC 62676-5-1:2024 — *Part 5-1: Image quality.*
6. IEC 60839-1:2014 — *Alarm and electronic security systems — Part 1: General requirements.*
7. IEC 60839-11-1:2013 — *Part 11-1: Electronic access-control systems — System and components requirements.*
8. IEC 60529:2013 — *Degrees of protection provided by enclosures (IP Code).*
9. ISO 6709:2008 — *Standard representation of geographic point location by coordinates.*
10. ISO 11770-2; ISO 11770-3 — *Information technology — Security techniques — Key management.*
11. ISO 14443 (all parts) — *Identification cards — Contactless integrated circuit cards — Proximity cards.*
12. ISO/IEC 7816 (all parts) — *Identification cards — Integrated circuit cards.*
13. ISO/IEC 14496-10:2014 — *Information technology — Coding of audio-visual objects — Part 10: Advanced video coding.*
14. ISO/IEC 18033-3:2010 — *Encryption algorithms — Block ciphers.*
15. ISO/IEC 18092 — *Near Field Communication.*
16. ISO/IEC 19794 (all parts) — *Biometric data interchange formats.*
17. ISO/IEC 23008-2:2020 — *High-Efficiency Video Coding (HEVC).*
18. ISO/IEC 27001:2022; ISO/IEC 27002:2022 — *Information security management systems.*
19. ISO/IEC 27037:2012 — *Guidelines for identification, collection, acquisition and preservation of digital evidence.*
20. ISO/IEC 27701:2019 — *Privacy information management systems.*
21. RFC 3339; RFC 5905; RFC 8259; RFC 8610; RFC 8915; RFC 8949; RFC 9052; RFC 9053; RFC 9562.


## Implementer note — operational lifecycle

Municipal physical-security infrastructure has a 20-30 year asset
lifecycle. The wire-format discipline must absorb that horizon
without locking the city into a single vendor or single regulator
interpretation. The Phase-level backwards-compatibility promise
is therefore mandatory rather than aspirational; cities make
capital-allocation decisions on hardware that will outlast many
political cycles, and the standard has to remain trustworthy
through those transitions.
