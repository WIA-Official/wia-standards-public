# WIA-CITY-014 (security-system-city) — Phase 4: Integration Specification

> **Version:** 1.0.0
> **Status:** Official
> **Phase:** 4 of 4 (Integration)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 4 covers the **integration** of WIA-CITY-014 deployments with surrounding building, city, and information ecosystems. The scope includes:

1. Information-security management (ISO/IEC 27000-family) and risk treatment.
2. Identity, access, and operator-station hardening.
3. Privacy and data-governance regimes (ISO/IEC 27701).
4. Building-management and command-centre interoperability.
5. Public-sector emergency, fire, and life-safety integration.
6. Conformance and field commissioning.
7. City-scale federation, edge-AI, and cross-discipline integration with WIA-CITY-009 (smart-lighting), WIA-CITY-017 (traffic-simulation), and beyond.

The objective is to give a single normative checklist that an operator can use to bring a WIA-CITY-014 system into a real city environment without ambiguity.

---

## 2. Information Security Management

### 2.1 ISO/IEC 27001 alignment

A WIA-CITY-014 deployment MUST be operated within a documented ISO/IEC 27001:2022 information-security management system (ISMS). The ISMS scope statement (Phase 1 §2.1 `iso27001Scope`) MUST cover:

- Personnel with operator, supervisor, administrator, or auditor roles.
- All gateways, recorders, and storage media that hold video or biometric data.
- All network conduits between zones (per IEC 62443-3-2 risk assessment).

### 2.2 ISO/IEC 27002 controls

Operators MUST select and document at minimum the following ISO/IEC 27002:2022 controls:

| Control area | Examples |
|--------------|----------|
| 5 — Organisational | A.5.1 policies; A.5.7 threat intelligence; A.5.34 privacy |
| 6 — People | A.6.1 screening; A.6.3 awareness, training |
| 7 — Physical | A.7.4 monitoring; A.7.5 environmental threats |
| 8 — Technological | A.8.5 secure authentication; A.8.16 monitoring activities; A.8.24 cryptography use |

### 2.3 IEC 62443 alignment

For deployments that are part of an industrial control system or operational-technology environment, the IEC 62443-3-3 system security level (SL-T) for the security-services zone MUST be ≥ SL-2. SL-3 is recommended for sites holding biometric data.

### 2.4 Risk assessment

The site descriptor MUST reference an IEC 62443-3-2 risk-assessment document under `iec62443.riskAssessmentRef`. Re-assessment cadence MUST not exceed 24 months and MUST follow significant change events (new zone, new credential modality, new federation).

---

## 3. Identity, Access, and Operator Hardening

### 3.1 Identity provisioning

- **Devices**: X.509 v3 (RFC 5280), Subject DN with the device-identity OID, Subject Alternative Name carrying the IEC 62676 device-MAC reference.
- **Operators**: federated identity over OAuth 2.1 (RFC 9700) and OpenID Connect 1.0; subject claim resolves to a IEC 62443-2-1 personnel registry.

### 3.2 Multi-factor

Operator login MUST require multi-factor authentication. Conforming factors include:

- Possession factor: a smart card / cryptographic token with X.509 client certificate (RFC 8446 mTLS).
- Inherence factor: ISO/IEC 19794-compliant biometric template held on a tamper-resistant token (e.g. ISO/IEC 7816 chip card).
- Knowledge factor: PIN / passphrase managed per ISO/IEC 27002 A.5.17.

### 3.3 Privileged operations

Privileged operations (`security:lockdown`, `security:clear-alarm` with mass-clear semantics, evidence export with key-export option) MUST require step-up authentication and MUST be logged with the second factor evidence.

### 3.4 Operator-station hardening

Operator stations MUST:

- Run a maintained operating system with the vendor's current security update level.
- Disable inbound services other than the Phase-2 client.
- Apply IEC 62443-3-3 SR 1.5 (audit), SR 1.7 (multi-factor), and SR 5.1 (zoning).

---

## 4. Privacy and Data Governance

### 4.1 ISO/IEC 27701

Operators handling personal data through cameras, biometric readers, or behavioural analytics MUST extend the ISMS to ISO/IEC 27701:2019 privacy information management.

### 4.2 Data classification

Each Phase-1 entity has an implicit data classification:

| Entity | Classification |
|--------|----------------|
| `Camera` raw video | Personal data |
| `Camera` privacy-masked video | Personal data (masked) |
| `AccessPoint` events | Personal data |
| `Sensor` aggregate counts | Anonymous |
| `Operator` identifiers | Personal data of personnel |

### 4.3 Retention

Default retention is 30 days for raw video, 90 days for access-control events, 7 years for audit log (per typical ISO/IEC 27001 audit cycles), and 0 days for biometric templates outside of the live transaction. Operators MAY override retention only with a documented lawful-basis record.

### 4.4 Subject access

The Phase-2 endpoint `/sites/{siteId}/data-subject-access:request` MUST return a signed export within 30 days. The export bundles all identified personal data with the same evidence-bundle structure as Phase 3 §6.

### 4.5 Cross-border transfer

Sites operating across jurisdictions MUST declare lawful-basis and onward-transfer mechanisms as a machine-readable policy under `/sites/{siteId}/.well-known/data-policy`.

---

## 5. Building-Management and Command Centre Interoperability

### 5.1 ISO/IEC 14908 / OPC UA / BACnet bridges

WIA-CITY-014 sites typically integrate with building automation. The recognised surfaces:

- **ISO/IEC 14908-1** control network protocol (normative bridge).
- **IEC 62541 OPC UA** (normative bridge with OPC UA Companion for IEC 62443).
- **ISO 16484-5 BACnet** (informative reference, not part of the ALLOW citation list).

### 5.2 OPC UA model

| WIA-CITY-014 entity | OPC UA node class |
|---------------------|-------------------|
| Site | Object (`SecuritySiteType`) |
| Zone | Object (`SecurityZoneType`) |
| Camera | Object (`CameraType`) |
| AccessPoint | Object (`AccessPointType`) |
| Recorder | Object (`RecorderType`) |
| Alarm | Object + DataChange |
| Lockdown command | Method (`Lockdown`) |

OPC UA security policy MUST be at least `Basic256Sha256` (IEC 62541-7 §5.2). Hardware-backed key material is recommended.

### 5.3 Integrated command centre

A command-centre console aggregating multiple WIA-CITY-014 sites MUST:

- Maintain a per-site identity context separate from operator credentials.
- Filter alarms by zone and by Phase-1 *operationalRequirement* per site policy.
- Surface evidence bundles as detached signed artefacts per Phase 3 §6.

---

## 6. Emergency, Fire, and Life-Safety Integration

### 6.1 Fire-alarm interface

Fire alarms detected by IEC 60839-1-classified sensors MUST cascade to:

- Unlock all `fireSafetyLink=FAIL-SAFE` access points.
- Override `security:lockdown` for `doorClass=emergency-egress`.
- Notify all operator sessions with `severity=CRITICAL`.

### 6.2 Life-safety reservation

Life-safety devices and protocols are **never** subject to the operator HTTP rate-limit nor to the audit-log read-restriction. The system MUST guarantee:

- Latency to door unlock on fire alarm ≤ 1 s for Grade 3, ≤ 0.5 s for Grade 4.
- Audit entries reach durable storage within 5 s of event (IEC 62443-3-3 SR 6.1).

### 6.3 Public-safety liaison

Sites operating with a regulatory liaison to public-safety services MUST expose a separately authenticated read-only feed of alarm summaries, with privacy-masked video clip references gated by an explicit authorisation record. The interface uses the Phase-2 surface with a dedicated audience claim.

---

## 7. Field Commissioning Checklist

A WIA-CITY-014 site MUST pass each of the following before being declared *operational*:

| # | Test | Reference |
|---|------|-----------|
| 1 | All cameras' achieved IEC 62676-4 category meets zone requirement | IEC 62676-4:2025 |
| 2 | Recording timestamps slaved to NTPv4 + NTS or equivalent ≤ 50 ms | RFC 5905, RFC 8915 |
| 3 | RTSPS / TLS 1.3 cipher inventory verified | RFC 8446 |
| 4 | OAuth 2.1 token introspection produces audit entries | RFC 9700, RFC 7662 |
| 5 | Multi-factor enforced for operator login | ISO/IEC 27002 A.8.5 |
| 6 | At least one camera and one access point per zone | site policy |
| 7 | All access points satisfy IEC 60839-11-1 grade ≥ zone grade | IEC 60839-11-1 |
| 8 | Tamper alarm latency ≤ 1 s | Phase 3 §4.3 |
| 9 | Lockdown propagation 99th-percentile ≤ 3 s | Phase 3 §7.3 |
| 10 | Evidence bundle signing chain validates end-to-end | Phase 3 §6 |
| 11 | ISO/IEC 27001 ISMS scope statement covers the site | ISO/IEC 27001 §4.3 |
| 12 | IEC 62443-3-3 SR 1, SR 2, SR 5, SR 6 implemented | IEC 62443-3-3 |
| 13 | ISO/IEC 27037 chain-of-custody export validates | ISO/IEC 27037 §7 |
| 14 | Privacy masks applied where required by §4.2 | ISO/IEC 27701 |
| 15 | Fire-alarm cascade meets §6.2 latency targets | IEC 60839-1 |

The checklist results MUST be stored as a signed Phase-1 *Site* attribute and surfaced under `/sites/{siteId}/health`.

---

## 8. Cyber-resilience and Incident Response

### 8.1 Detection

The site MUST run continuous detection that combines:

- IEC 62443-3-3 SR 6 audit-log analytics for anomalous operator behaviour.
- Camera firmware integrity via hardware-rooted attestation (TPM 2.0 or vendor secure-element).
- Time-source integrity monitoring (NTS-protected NTPv4 clients refusing unsigned servers).

### 8.2 Containment

Containment playbooks MUST include:

- Issuing `security:lockdown` from a privileged station.
- Revoking compromised operator credentials via the OAuth introspection endpoint with token replay refused per RFC 7662.
- Pausing recorder export endpoints to preserve forensics integrity until the incident scope is bounded.

### 8.3 Recovery

Recovery procedures MUST:

- Re-run §7 commissioning checklist for affected zones.
- Re-attest device firmware before re-enabling.
- Document the incident in the ISO/IEC 27001 corrective-action register.

---

## 9. City-Scale Considerations

### 9.1 Federation

Sites federate through the management plane. A federation document MUST specify:

- The list of partner sites (URIs).
- The verb-level cross-site authorisation matrix.
- The privacy-masking and data-handling policies that survive federation.

### 9.2 Cross-discipline integration

WIA-CITY-014 commonly integrates with:

- **WIA-CITY-009 (smart-lighting)** — security-driven lighting activation; fire-alarm-driven lighting overrides.
- **WIA-CITY-017 (traffic-simulation)** — incident-driven traffic re-routing; emergency-vehicle pre-emption.
- **WIA-CITY-008 (smart-parking)** — credential-driven gate entry events that double as access-point events.
- **WIA-CITY-006 (public-transit)** — station-level CCTV integrated with platform alerting.

### 9.3 Edge analytics

Camera-side AI analytics (object detection, attribute extraction) MUST:

- Run on hardware classified at ≥ IEC 62443-4-2 device security level matching the zone.
- Surface results as Phase-1 *Sensor* events with provenance referencing the model identifier and version.
- Refuse to emit personal-data attributes (e.g. demographic inference) unless the zone policy explicitly enables them.

### 9.4 Cross-jurisdiction operations

Cities operating WIA-CITY-014 across regulatory regions MUST hold a documented mapping of each region's lawful-basis regime to the system's data-handling levers. Changes to retention, biometric usage, or federated export MUST flow through the change-control process required by ISO/IEC 27001 §8.

---

## 10. Conformance Tags

A site descriptor MUST advertise a list of conformance tags that summarise the achieved profile:

| Tag | Meaning |
|-----|---------|
| `wia-city-014/v1/baseline` | Phase 3 §9.1 baseline profile |
| `wia-city-014/v1/extended` | Phase 3 §9.2 extended profile |
| `wia-city-014/v1/city-grade` | Phase 3 §9.3 city-grade profile |
| `wia-city-014/v1/iso27001` | ISMS-covered |
| `wia-city-014/v1/iso27701` | PIM-covered |
| `wia-city-014/v1/iec62443-sl2` | Security level 2 |
| `wia-city-014/v1/iec62443-sl3` | Security level 3 |
| `wia-city-014/v1/iec60839-grade-3` | Access-control grade 3 |
| `wia-city-014/v1/edge-ai` | Edge analytics profile per §9.3 |

Conformance tags are normative for automated discovery and informative for human readers.

---

## 11. References

1. IEC 60529:2013 — *IP Code.*
2. IEC 60839-1:2014; IEC 60839-5-1; IEC 60839-5-2; IEC 60839-5-3; IEC 60839-7-1; IEC 60839-11-1:2013; IEC 60839-11-2:2014.
3. IEC 62443-2-1; IEC 62443-3-2; IEC 62443-3-3; IEC 62443-4-2.
4. IEC 62541-7 — *OPC UA — Profiles.*
5. IEC 62676-1; -2-1; -2-3; -4:2025; -5-1:2024.
6. ISO 11770-2; 11770-3 — *Key management.*
7. ISO 14443; ISO/IEC 7816 — *Identification cards / smart cards.*
8. ISO/IEC 14908-1 — *Control network protocol.*
9. ISO/IEC 14496-10; -12; ISO/IEC 23008-2 — *AVC, ISOBMFF, HEVC.*
10. ISO/IEC 18033-3 — *Block ciphers.*
11. ISO/IEC 19794 (all parts) — *Biometric data interchange.*
12. ISO/IEC 27001:2022; ISO/IEC 27002:2022; ISO/IEC 27037:2012; ISO/IEC 27701:2019.
13. ISO 19115-1:2014 — *Geographic information — Metadata.*
14. RFC 5280 — *X.509 PKI Certificate and CRL Profile.*
15. RFC 5905; RFC 8915 — *NTPv4, NTS.*
16. RFC 7252; RFC 7641 — *CoAP, OBSERVE.*
17. RFC 7662 — *OAuth Token Introspection.*
18. RFC 8446; RFC 9147 — *TLS 1.3, DTLS 1.3.*
19. RFC 8613 — *OSCORE.*
20. RFC 9019; RFC 9124 — *SUIT manifests.*
21. RFC 9052; RFC 9053 — *COSE.*
22. RFC 9111 — *HTTP Caching.*
23. RFC 9700 — *OAuth 2.1.*
24. FIPS 180-4 — *SHA family.*
25. FIPS 197 — *AES.*
