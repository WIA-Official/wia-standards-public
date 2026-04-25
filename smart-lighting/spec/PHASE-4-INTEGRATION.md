# WIA-CITY-009 (smart-lighting) — Phase 4: Integration Specification

> **Version:** 1.0.0
> **Status:** Official
> **Phase:** 4 of 4 (Integration)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 4 covers the **integration** of WIA-CITY-009 smart-lighting deployments with the surrounding building, city, and information ecosystems. The scope includes:

1. Building-management systems (BMS) and IoT platforms.
2. Energy-management systems and grid-side demand-response programmes.
3. Information security, identity, and audit infrastructure.
4. Privacy and data-governance regimes.
5. Photobiological and human-centric lighting integration.
6. Conformance testing and field commissioning.

The goal is to give a single normative checklist that an operator can use to bring a WIA-CITY-009 system into a real building or city environment without ambiguity.

---

## 2. Building-Management Systems (BMS)

### 2.1 BACnet, ISO/IEC 14908, and OPC UA bridges

Three interoperability surfaces are recognised:

- **ISO 16484-5 (BACnet) — informative reference**: gateways MAY expose a BACnet IP server view; mapping uses the IEC 62386-102 device-type to BACnet object-type matrix published by the WIA gateway profile registry. The mapping itself is informative because BACnet is referenced by an external standards body, not by the ALLOW citation list of this document.
- **ISO/IEC 14908 control network protocol**: normative bridge described in Phase 3 §5.
- **OPC UA (IEC 62541)**: normative bridge through the *OPC UA for Devices (DI)* and *OPC UA Companion for IEC 62386* profiles. The companion profile maps every Phase-1 entity onto an OPC UA object node and every Phase-2 verb onto an OPC UA method node.

### 2.2 IEC 62541 (OPC UA) profile binding

| WIA-CITY-009 entity | OPC UA node class |
|---------------------|-------------------|
| Site | Object (`SiteType`) |
| Zone | Object (`ZoneType`) |
| Luminaire | Object (`LuminaireType`) |
| ControlGear | Object (`ControlGearType`) |
| Sensor | Object (`SensorType`) |
| Scene | Object (`SceneType`) |
| Recall scene | Method (`RecallScene`) |
| Set level | Method (`SetLevel`) |
| Telemetry | Variable + DataChange subscription |

OPC UA security policy MUST be at least `Basic256Sha256` (IEC 62541-7 §5.2). For deployments with constrained gateways, the *AES256_Sha256_RsaPss* profile is recommended.

---

## 3. Energy-Management and Grid Integration

### 3.1 Demand response

Operators integrating WIA-CITY-009 with utility-side demand-response programmes MUST:

1. Expose an aggregate *power-curtail capability* per site, expressed as a step function of dimming level (0..254) versus expected power drawn (in W), measured under IEC 60598 rated conditions.
2. Accept a curtailment instruction encoded as a Phase-2 long-running operation, with payload fields `targetPowerW`, `rampSeconds`, and `holdSeconds`.
3. Refuse curtailments that would violate the IEC 62386-202 emergency-lighting reservation or the photobiological floor configured in §6.

### 3.2 Astronomical and weather inputs

Integration with sunrise/sunset and weather (cloud cover, precipitation) feeds is recommended for outdoor lighting. The data interchange MUST use:

- IANA Time Zone Database for civil time.
- ISO 19115-1:2014 metadata when GIS context is required.
- WMO BUFR (informative) or vendor-neutral JSON schemas published by the relevant national meteorological office.

### 3.3 Submetering

Each control gear of class IEC 62386-252 (energy-reporting gear) MUST be polled at intervals not exceeding 60 s when submetering is enabled. Aggregated readings MUST be persisted with provenance per ISO 19115-1 lineage fields.

---

## 4. Identity, Access, and Cybersecurity

### 4.1 Identity provisioning

Every device, gateway, and operator account MUST hold a credential that maps to:

- **Device**: an X.509 v3 certificate (RFC 5280) with a *device-identity* OID issued by the site PKI, bound to the IEC 62386 random address (Phase 1 §3.2).
- **Operator**: a federated identity asserted via OpenID Connect 1.0 (built on OAuth 2.1, RFC 9700), with subject claim resolvable to a IEC 62443-2-1 personnel registry.

### 4.2 Zoning and segmentation

Network segmentation MUST follow IEC 62443-3-2 risk-assessment outputs. The site descriptor `iec62443ZoneId` field (Phase 1 §2.1) names the zone; cross-zone traffic uses conduits with at least these enforcement points:

- Authenticated firewall (RFC 8907 TACACS+ or RFC 8907-equivalent for management plane).
- Per-conduit allow-list of Phase-2 verbs.
- Audit log per IEC 62443-3-3 SR 6.1.

### 4.3 Update integrity

Gateway and control-gear firmware updates MUST be:

- Signed under a code-signing certificate distinct from the device-identity certificate.
- Verified by the device prior to install, using IETF SUIT (RFC 9019, RFC 9124) manifests where the device is constrained.
- Logged with the manifest digest, source, and timestamp.

### 4.4 Cryptographic algorithms

| Layer | Algorithm | Reference |
|-------|-----------|-----------|
| TLS / DTLS | TLS 1.3 cipher suites | RFC 8446, RFC 9147 |
| OSCORE | AES-CCM-16-64-128 | RFC 8613 §12 |
| COSE signature | ES256, EdDSA | RFC 9053 |
| TEDS protection | AES-128-CCM* | ISO/IEC/IEEE 21451-2 |
| WSP high-security | AES-128-CCM* with rolling nonce | ISO/IEC 14543-3-10 §11 |

Implementations MUST track the IETF Crypto-Forum Research Group recommendations and refuse cipher suites that have been deprecated in TLS 1.3 BCP documents.

---

## 5. Privacy and Data Governance

### 5.1 Personal data minimisation

Lighting telemetry can contain personal information (presence, room occupancy, behavioural patterns). Operators MUST classify these as personal data under the applicable jurisdictional regime and implement:

- Storage minimisation: 30-day default retention for raw occupancy logs, with aggregation thereafter.
- Purpose limitation: telemetry use restricted to declared operational purposes.
- Subject-access: Phase-2 endpoints `/sites/{siteId}/data-subject-access:request` MUST return a signed export within 30 days.

### 5.2 Pseudonymisation

Sensor identifiers used in long-term archives MUST be pseudonymised with a keyed hash (HMAC-SHA-256, FIPS 198-1) where the key is rotated annually and held by the operator's data-protection function.

### 5.3 Cross-border transfer

Sites operating across jurisdictions MUST declare lawful-basis and onward-transfer mechanisms as a machine-readable policy under `/sites/{siteId}/.well-known/data-policy`.

---

## 6. Human-Centric and Photobiological Integration

### 6.1 IEC 62471 risk-group reservation

Every luminaire's photobiological risk group (Phase 1 §2.3 `iec60598PhotobiologicalRiskGroup`) limits the maximum spectrum that the gateway can apply. The gateway MUST refuse spectrum requests that would push the luminaire above its declared risk group.

### 6.2 Circadian (HCL) lighting

Circadian schedules are first-class scenes. The recommended encoding follows the *melanopic equivalent daylight illuminance (mEDI)* metric from CIE S 026/E:2018 *System for Metrology of Optical Radiation for ipRGC-Influenced Responses to Light* (informative reference; the gateway MUST report mEDI when sensor fusion permits).

### 6.3 Flicker

Maximum flicker (percent flicker, flicker index) MUST conform to the limits set by IEC TR 61547 and IEC 62386-102 §10.4 fading transitions. Drivers operating below these limits MUST declare so in their TEDS metadata.

### 6.4 Emergency egress

Egress lighting integration follows IEC 60598-2-22 *Particular requirements — Luminaires for emergency lighting* and IEC 62386-202. The Phase-2 verb `lighting:safety-override` is gated by IEC 62443-3-3 SR 5.4.

---

## 7. Field Commissioning Checklist

A WIA-CITY-009 site MUST pass each of the following before being declared *operational*:

| # | Test | Reference |
|---|------|-----------|
| 1 | Bus voltage and current within IEC 62386-101 ed. 3.0 §8.2 | DALI bus measurement |
| 2 | Random-address discovery completes for all expected devices | IEC 62386-102 §11.2 |
| 3 | Phase-2 OpenAPI document validates against published schema | OpenAPI 3.1 |
| 4 | TLS / DTLS cipher inventory restricted to TLS 1.3 | RFC 8446 / RFC 9147 |
| 5 | OSCORE protection enabled on every CoAP endpoint | RFC 8613 |
| 6 | OAuth 2.1 token introspection produces audit entries | RFC 9700, RFC 7662 |
| 7 | At least one occupancy and one illuminance sensor calibrated | ISO/IEC/IEEE 21451-2 §7 |
| 8 | Photobiological risk-group floor enforced for every luminaire | IEC 62471 |
| 9 | Emergency-luminaire reservation enforced on group commands | IEC 62386-202 |
| 10 | IEC 62443-3-3 SR 1, SR 2, SR 6, SR 7 controls implemented | IEC 62443-3-3 |
| 11 | Time synchronisation accuracy ≤ 1 s | RFC 5905, RFC 8915 |
| 12 | Round-trip latency in §3.5 of Phase 3 met for 95th percentile | Phase 3 §5.3 |

The checklist results MUST be stored as a signed Phase-1 *Site* attribute and surfaced under `/sites/{siteId}/health`.

---

## 8. City-Scale Considerations

### 8.1 Outdoor-lighting overlay

City-scale outdoor lighting (street, plaza, tunnel) is treated as one or more zones with `boundary.indoor = false`. The astronomical schedule (sunrise, sunset, civil twilight) is computed per geographic centroid and stored as the recommended schedule baseline. Adaptive dimming is mandatory.

### 8.2 Light pollution and dark-sky compliance

Sites near scientific or environmental dark-sky zones MUST honour the upper-hemispheric flux limit declared in the site descriptor `lightPollutionPolicy` field. Recommended limits follow IEC TR 63337 (informative) and IES TM-15-11 (informative); WIA-CITY-009 expresses both as numeric upper bounds in lumens emitted above the horizontal.

### 8.3 Interoperability with WIA-CITY-014 (security-system-city)

When a site combines WIA-CITY-009 lighting with WIA-CITY-014 security/CCTV, the lighting gateway MUST accept *security-driven activation* events as Phase-2 long-running operations and MUST log every activation with cross-reference to the originating WIA-CITY-014 alarm identifier.

---

## 9. Conformance Tags

A site descriptor MUST advertise a list of conformance tags that summarise the achieved profile:

| Tag | Meaning |
|-----|---------|
| `wia-city-009/v1/wired` | Wired DALI profile per Phase 3 §6.1 |
| `wia-city-009/v1/wireless` | Wireless extension profile per Phase 3 §6.2 |
| `wia-city-009/v1/eh` | Energy-harvesting sensors profile per Phase 3 §6.3 |
| `wia-city-009/v1/iec62541` | OPC UA bridge per §2.2 |
| `wia-city-009/v1/iec62443-zl1` | Security level 1 per IEC 62443-3-3 |
| `wia-city-009/v1/hcl` | Circadian / human-centric lighting per §6.2 |
| `wia-city-009/v1/dr` | Demand-response capable per §3.1 |

Conformance tags are informative for human readers and normative for automated discovery.

---

## 10. References

1. IEC 60598-1:2020 — *Luminaires — General requirements and tests.*
2. IEC 60598-2-22 — *Particular requirements — Emergency luminaires.*
3. IEC 62386-101 ed. 3.0; -102 ed. 3.0; -103 ed. 2.0; -104; -202; -209; -220; -252.
4. IEC 62443-2-1; IEC 62443-3-2; IEC 62443-3-3 — *Industrial communication networks security.*
5. IEC 62471:2006 — *Photobiological safety of lamps and lamp systems.*
6. IEC 62541-7 — *OPC Unified Architecture — Part 7: Profiles.*
7. IEC TR 61547 — *Equipment for general lighting purposes — EMC immunity requirements.*
8. ISO 16484-5 — *Building automation and control systems — Data communication protocol (BACnet); informative reference.*
9. ISO 19115-1:2014 — *Geographic information — Metadata.*
10. ISO/IEC 14543-3-10:2012 — *Wireless short-packet protocol.*
11. ISO/IEC 14908-1; -3 — *Control network protocol.*
12. ISO/IEC/IEEE 21451-1; -2; -5 — *Smart transducer interface.*
13. CIE S 026/E:2018 — *System for Metrology of Optical Radiation for ipRGC-Influenced Responses to Light* (informative).
14. RFC 5280 — *Internet X.509 Public Key Infrastructure Certificate and CRL Profile.*
15. RFC 5905 — *Network Time Protocol Version 4.*
16. RFC 7662 — *OAuth 2.0 Token Introspection.*
17. RFC 8446 — *TLS 1.3.*
18. RFC 8613 — *OSCORE.*
19. RFC 8915 — *Network Time Security.*
20. RFC 8949 — *CBOR.*
21. RFC 9019 — *A Firmware Update Architecture for IoT Devices.*
22. RFC 9052; RFC 9053 — *COSE.*
23. RFC 9111 — *HTTP Caching.*
24. RFC 9124 — *A Manifest Information Model for Firmware Updates in IoT Devices.*
25. RFC 9147 — *DTLS 1.3.*
26. RFC 9176 — *CoRE Resource Directory.*
27. RFC 9700 — *OAuth 2.1.*
28. FIPS 180-4 — *Secure Hash Standard.*
29. FIPS 198-1 — *Keyed-Hash Message Authentication Code (HMAC).*
