# WIA-SPACE-003 (satellite-communication) — Phase 1: Data Format Specification

> **Version:** 1.0.0
> **Status:** Official
> **Phase:** 1 of 4 (Data Format)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This document specifies the canonical data formats used by WIA-SPACE-003 satellite-communication deployments. The formats describe **space-asset descriptors**, **link budgets**, **bundle traffic**, **telemetry / tele-command sessions**, and **ground-segment / user-segment composition** in a way that interoperates with the IETF *Delay-Tolerant Networking* family (RFC 4838, RFC 9171), the OSI reference model (ISO/IEC 7498-1:1994), the ISO/IEC 27000-family information-security baseline, and the IEC 62443-family operational-technology baseline.

### 1.1 Goals

- Provide a JSON / CBOR canonical encoding for each entity that an integrator may need to reason about across orbital regimes (GEO, MEO, LEO, HEO) and across layers (physical, link, network, transport, application).
- Carry RFC 9171 *Bundle Protocol Version 7* (BPv7) primary block and canonical block references as first-class objects so that bundles produced or consumed by a deployment can be correlated with the WIA descriptors.
- Express link budgets, modulation profiles, and access-method metadata in a form that survives round-trip with the operator's link-planning tooling.

### 1.2 Non-goals

- Defining new physical-layer waveforms. Waveform selection defers to national radio regulations and to existing space-domain consortium documents.
- Defining new orbital-mechanics models. Orbit propagation defers to the operator's existing tooling and to the relevant aerospace conventions.
- Defining new encryption algorithms. Cryptography defers to ISO/IEC 18033-3 (block ciphers) and FIPS 197 (AES) / FIPS 180-4 (SHA family).

---

## 2. Top-level Object Model

A WIA-SPACE-003 deployment serializes into seven root entities:

```
SpaceCommunicationsDeployment ──┬── Spacecraft[]
                                ├── GroundStation[]
                                ├── UserTerminal[]
                                ├── Link[]
                                ├── BundleSession[]
                                ├── TelemetrySession[]
                                └── ServicePlan[]
```

Each entity has a stable URI of the form `wia-space://<deployment-id>/<entity-type>/<entity-id>`.

### 2.1 SpaceCommunicationsDeployment

```json
{
  "type": "SpaceCommunicationsDeployment",
  "version": "1.0.0",
  "deploymentId": "<UUID v4 per RFC 9562>",
  "operator": {
    "name": "string",
    "iso27001Scope": "string",
    "iec62443ZoneId": "string"
  },
  "regulatoryFilings": ["string (URI of regulatory record)"],
  "createdAt": "<RFC 3339 date-time>",
  "updatedAt": "<RFC 3339 date-time>"
}
```

The `iso27001Scope` field carries the boundary statement of the operator's ISO/IEC 27001 ISMS. The `regulatoryFilings` list points to operator-controlled records of national radio-regulation filings; the document itself does not redistribute regulatory content.

### 2.2 Spacecraft

```json
{
  "type": "Spacecraft",
  "spacecraftId": "string",
  "noradId": "uint",
  "internationalDesignator": "string",
  "constellation": "string|null",
  "regime": "GEO|MEO|LEO|HEO|MOLNIYA|LUNAR|DEEP-SPACE",
  "orbit": {
    "epoch": "<RFC 3339 date-time>",
    "semiMajorAxisKm": "number",
    "eccentricity": "number",
    "inclinationDeg": "number",
    "raanDeg": "number",
    "argPerigeeDeg": "number",
    "meanAnomalyDeg": "number"
  },
  "payloads": [
    {
      "payloadId": "string",
      "kind": "TRANSPONDER|REGENERATIVE|OPTICAL|SDR",
      "uplinkBands": ["L|S|C|X|Ku|Ka|V|Q|OPTICAL"],
      "downlinkBands": ["L|S|C|X|Ku|Ka|V|Q|OPTICAL"]
    }
  ]
}
```

Orbit elements use the classical osculating-element notation. Implementations MAY translate to and from TLE-style mean elements; the round-trip MUST preserve epoch and reference frame.

### 2.3 GroundStation

```json
{
  "type": "GroundStation",
  "groundStationId": "string",
  "iso6709": "+37.5326-126.9905+50/",
  "antennas": [
    {
      "antennaId": "string",
      "diameterMeters": "number",
      "gainDbi": "number",
      "bands": ["L|S|C|X|Ku|Ka|V|Q|OPTICAL"],
      "polarization": "LHCP|RHCP|LINEAR-V|LINEAR-H|DUAL"
    }
  ],
  "iec62443ZoneId": "string"
}
```

### 2.4 UserTerminal

```json
{
  "type": "UserTerminal",
  "terminalId": "string",
  "kind": "VSAT|HAND-HELD|MOBILE|FIXED|MARITIME|AERONAUTICAL|IoT",
  "antenna": {
    "diameterMeters": "number",
    "gainDbi": "number",
    "steerable": "boolean",
    "bands": ["L|S|C|Ku|Ka"]
  },
  "transport": "WIRED-BACKHAUL|WIRELESS-BACKHAUL|DIRECT",
  "iec62443ZoneId": "string"
}
```

### 2.5 Link

A *Link* is a directed channel between a Spacecraft and a GroundStation or UserTerminal. The Link descriptor carries enough information to reproduce the operator's link-budget calculation.

```json
{
  "type": "Link",
  "linkId": "string",
  "fromEndpoint": "<URI of source endpoint>",
  "toEndpoint": "<URI of destination endpoint>",
  "band": "L|S|C|X|Ku|Ka|V|Q|OPTICAL",
  "centerFrequencyHz": "number",
  "bandwidthHz": "number",
  "modulation": "BPSK|QPSK|8PSK|16APSK|32APSK|64APSK|256APSK|OFDM|OPTICAL-DPSK",
  "coding": "string (canonical name of FEC scheme)",
  "rollOffFactor": "number",
  "linkBudget": {
    "eirpDbW": "number",
    "freeSpaceLossDb": "number",
    "atmosphericLossDb": "number",
    "rainAttenuationDb": "number",
    "antennaGainRxDbi": "number",
    "noiseTemperatureK": "number",
    "cnrDb": "number"
  }
}
```

The link-budget structure is canonical; implementations MUST be able to round-trip the Link descriptor through their internal link-budget tooling.

### 2.6 BundleSession

A *BundleSession* represents a logical conversation between two BPv7 endpoints. Phase-1 carries enough metadata to correlate bundles produced by an operator-side BPv7 implementation with the deployment.

```json
{
  "type": "BundleSession",
  "bundleSessionId": "string",
  "sourceEndpointId": "string (RFC 9171 EID)",
  "destinationEndpointId": "string (RFC 9171 EID)",
  "convergenceLayer": "TCPCL|LTPCL|UDPCL|BLEUSAS",
  "lifetimeMillis": "uint",
  "bundleProtocolVersion": 7,
  "bundleProtocolFlags": "uint64-bitmask"
}
```

### 2.7 TelemetrySession

```json
{
  "type": "TelemetrySession",
  "telemetrySessionId": "string",
  "spacecraftId": "string",
  "groundStationId": "string",
  "kind": "TM|TC",
  "frameVersion": "string",
  "transferFrameRateBps": "number",
  "encryptionAtTransport": {
    "algorithm": "AES-256-GCM|AES-128-GCM|NONE",
    "keyProvenance": "ISO 11770-2|ISO 11770-3|OPERATOR-DEFINED"
  }
}
```

### 2.8 ServicePlan

A *ServicePlan* describes a contracted service offered by the deployment to a downstream operator or tenant.

```json
{
  "type": "ServicePlan",
  "planId": "string",
  "serviceLevel": "BEST-EFFORT|STANDARD|PRIORITY|EMERGENCY",
  "throughputCommitMbps": "number",
  "latencyTargetMs": "number",
  "availabilityCommit": "number (0..1)",
  "iec62443SecurityLevel": "1|2|3|4"
}
```

---

## 3. Canonical Encodings

WIA-SPACE-003 mandates two interchangeable encodings:

1. **JSON** — RFC 8259, UTF-8, NFC-normalised strings.
2. **CBOR** — RFC 8949, with the *Concise Data Definition Language* (RFC 8610) schema published alongside.

Both encodings MUST be byte-equivalent under round-trip via the canonical CBOR deterministic encoding rules (RFC 8949 §4.2). End-to-end protection of payloads uses *CBOR Object Signing and Encryption* (COSE, RFC 9052 / 9053).

### 3.1 Time

Timestamps use RFC 3339 *date-time*. Real-time operations against a deployed system MUST be slaved to NTPv4 with NTS (RFC 5905, RFC 8915). For deep-space operations, the operator's mission time-base MAY override; the chosen base MUST be recorded in the deployment descriptor.

### 3.2 Identifiers

UUID v4 (RFC 9562) is the default for WIA descriptors. Bundle Protocol endpoint identifiers (EIDs) follow the RFC 9171 §4.2.5 schemes (`dtn:`, `ipn:`).

### 3.3 Coordinate conventions

Geographic coordinates use ISO 6709:2008. Spacecraft state vectors use the operator's chosen reference frame; the frame name MUST be recorded in the orbit descriptor.

---

## 4. Versioning and Compatibility

The `version` field uses semver 2.0.0. Phase 1 v1.x payloads are backwards-compatible within the major version. Edition references for normative work were:

- ISO/IEC 7498-1:1994 — *Open Systems Interconnection — Basic Reference Model.*
- ISO/IEC 18033-3:2010 — *Encryption algorithms — Block ciphers.*
- ISO/IEC 27001:2022; ISO/IEC 27002:2022 — *Information security management.*
- IEC 62443-3-3:2013 — *System security requirements and security levels.*
- RFC 4838 — *Delay-Tolerant Networking Architecture.*
- RFC 9171 — *Bundle Protocol Version 7.*
- RFC 9173 — *BPv7 default security context.*

---

## 5. Privacy and Operator-Confidentiality Controls

Operator-confidential data (link budgets, regulatory filings, customer service plans) carry a `confidentialityClass` field; downstream tooling MUST honour the class when serialising or exporting.

```json
{
  "confidentialityClass": "OPEN|OPERATOR-CONFIDENTIAL|REGULATORY-RESTRICTED|EXPORT-CONTROLLED"
}
```

---

## 6. References

1. ISO/IEC 7498-1:1994 — *OSI Basic Reference Model.*
2. ISO/IEC 18033-3:2010 — *Block ciphers.*
3. ISO/IEC 27001:2022; ISO/IEC 27002:2022 — *ISMS.*
4. ISO 11770-2; ISO 11770-3 — *Key management.*
5. ISO 6709:2008 — *Geographic point location.*
6. IEC 62443-3-3:2013 — *System security requirements and security levels.*
7. RFC 3339 — *Date and Time on the Internet.*
8. RFC 4838 — *Delay-Tolerant Networking Architecture.*
9. RFC 5905; RFC 8915 — *NTPv4, NTS.*
10. RFC 8259 — *JSON.*
11. RFC 8610 — *CDDL.*
12. RFC 8949 — *CBOR.*
13. RFC 9052; RFC 9053 — *COSE.*
14. RFC 9171; RFC 9173 — *Bundle Protocol v7 and security context.*
15. RFC 9562 — *UUIDs.*
16. FIPS 180-4; FIPS 197 — *SHA, AES.*

---

## 7. Frequency Band Conventions

The following band labels are used throughout WIA-SPACE-003 descriptors. Band names are purely descriptive; specific frequency boundaries are governed by the operator's regulatory filing.

| Label | Indicative range | Typical use |
|-------|------------------|-------------|
| L | ~1–2 GHz | Mobile-satellite, GNSS |
| S | ~2–4 GHz | TT&C, mobile-satellite |
| C | ~4–8 GHz | Fixed-satellite, broadcast |
| X | ~8–12 GHz | Government/military, deep-space |
| Ku | ~12–18 GHz | Fixed-satellite, broadcast |
| Ka | ~26–40 GHz | High-throughput satellites |
| V | ~40–75 GHz | Inter-satellite, gateway feeder |
| Q | ~30–50 GHz | Inter-satellite, gateway feeder |
| OPTICAL | 850–1550 nm and visible | Optical inter-satellite, free-space optical |

Operators using a label outside this list MUST register the local extension in the Phase-1 *SpaceCommunicationsDeployment* descriptor under a dedicated `bandLabelRegistry` field.

## 8. Convergence-Layer Selection Heuristics

The following heuristics describe how an operator typically chooses a BPv7 convergence layer for a given orbital regime. The heuristics are informative; the binding choice is recorded in the Phase-1 *BundleSession* descriptor.

| Regime | Latency band | Recommended convergence | Notes |
|--------|--------------|-------------------------|-------|
| LEO | 20–40 ms RTT | TCPCL or UDPCL | Frequent contact handover |
| MEO | 50–150 ms RTT | TCPCL | Stable contact windows |
| GEO | 240–280 ms RTT | TCPCL | Long stable contacts |
| HEO / Molniya | Variable | LTPCL or TCPCL | Depends on perigee dwell |
| Lunar | ≥ 1300 ms RTT | LTPCL | Long round-trip |
| Deep-space | Tens of seconds to minutes | LTPCL | LTP designed for the regime |

## 9. Service-Plan Tier Conventions

WIA-SPACE-003 defines four canonical tier labels for service plans: BEST-EFFORT, STANDARD, PRIORITY, EMERGENCY. Operators MAY add a vendor-specific suffix (e.g. `PRIORITY-GOLD`) but MUST preserve the canonical prefix so that downstream tooling can compare tiers across operators. Emergency tier service plans MUST be available only to government or treaty-organised disaster-response tenants and MUST be subject to the operator's emergency-traffic policy.
