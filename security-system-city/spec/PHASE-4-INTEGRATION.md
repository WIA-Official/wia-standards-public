# WIA-CITY-014 (security-system-city) — Phase 4: Integration Specification

> **Version:** 1.0.0
> **Status:** Draft
> **Phase:** 4 of 4 (Integration)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 4 specifies how a city-scale physical-security deployment integrates with the broader municipal-services ecosystem: emergency-management platforms (CAD systems, dispatch radios, EMS), forensic-lab pipelines, cross-agency coordination tooling, and regulatory-reporting flows. The integration is layered: ONVIF and IEC 62676 carry the device-level interoperability; ISO 22320 carries the emergency-management coordination; ISO/IEC 27001 carries the information-security baseline; the city's privacy law carries the consent and purpose-limitation floor.

---

## 2. Bridge profiles

### 2.1 Bridge to legacy NVR / VMS deployments

Most city deployments already run a video-management system (VMS) — Genetec Security Center, Milestone XProtect, Avigilon Control Center, Hanwha Wisenet WAVE, Bosch BVMS. The bridge profile maps Phase 2 endpoints to VMS-native primitives without requiring the legacy stack to be replaced.

| Phase 2 endpoint | VMS primitive | Bridge mapping |
|------------------|---------------|----------------|
| `GET /camera/{id}/stream` | VMS RTSP/SRTP feed | direct passthrough; ONVIF Profile T metadata preserved |
| `GET /camera/{id}/event-feed` | VMS analytics API | ONVIF Profile T analytics events translated to standard envelope |
| `POST /access-point/{id}/event` | VMS access-control bridge | ONVIF Profile A events translated; legacy proprietary protocols (HID OSDP, OSDP v2.2) wrapped in the bridge |
| `POST /incident` | VMS bookmark + investigation case | bridge creates VMS investigation case with Phase 1 incident envelope as evidence |

The bridge container ships at `https://github.com/WIA-Official/wia-security-system-city-bridges/vms-bridge` with reference implementations for the five major VMS platforms.

### 2.2 Bridge to CAD (Computer-Aided Dispatch) systems

The CAD bridge maps Phase 2 incident envelopes to the dispatch system's native call format. Reference bridges:

- **CentralSquare Public Safety CAD** — used by ~40% of US municipal dispatch operations
- **Hexagon Intergraph CAD** — used by ~25% of US municipal dispatch + many international deployments
- **Tyler Technologies New World CAD** — used by ~15% of US municipal dispatch
- **Korean 119 통합신고시스템** — Korea National Fire Agency unified emergency reporting
- **Japan 高度道路交通システム** — Japan ITS-integrated emergency dispatch

### 2.3 Bridge to forensic-lab pipelines

Forensic labs consume IEC 62676-2-31 evidence-grade encoded clips with the chain-of-custody envelope from Phase 3 §3. The bridge maps the incident-dossier exchange to the lab's case-management system (typically a vendor-specific system with documented intake API).

### 2.4 Bridge to access-control vendors

Access-control hardware vendors (HID, ASSA ABLOY, Suprema, ZKTeco, Hanwha) speak proprietary or semi-open protocols. The bridge profile maps Phase 2 access-point endpoints to:

- **OSDP v2.2** (Open Supervised Device Protocol) — open standard; primary target
- **HID OSDP** — HID-flavoured OSDP with extensions
- **Wiegand** — legacy 26/34/N-bit protocol; wrapped in OSDP-shaped envelope for uniformity
- **Mercury Security MS-ICS** — common access-control platform
- **Suprema BioStar** — biometric access-control with BIOS protocol

---

## 3. Privacy and regulatory integration

### 3.1 Jurisdiction mapping

The standard maps to per-jurisdiction privacy and surveillance frameworks:

| Jurisdiction | Privacy law | Surveillance law | Bridge artefact |
|--------------|-------------|------------------|-----------------|
| EU | GDPR Article 32 + EU Charter Article 7-8 | EU Member State surveillance law | bridges/eu-gdpr.json |
| US (federal) | None (sectoral) | 4th Amendment + per-state | bridges/us-federal.json |
| US (California) | CCPA/CPRA | California state surveillance law | bridges/us-ca.json |
| Korea | PIPA Article 25 | 통합방위법 + 영상정보처리기기 운영ㆍ관리 가이드라인 | bridges/kr-pipa.json |
| Japan | APPI | 個人情報保護法 + 警察法 | bridges/jp-appi.json |
| UK | UK GDPR + DPA 2018 | RIPA 2000 + IPA 2016 | bridges/uk-gdpr.json |

### 3.2 DPIA publication and review cadence

Every conformant deployment publishes a Data Protection Impact Assessment (DPIA, ISO/IEC 29134:2023) with documented purpose limitations and risk-mitigation measures. The DPIA is reviewed at least annually and on every material change to the surveillance footprint (new camera zone, new analytics capability, new agency partner).

### 3.3 Subject-rights integration

GDPR Article 15 (right of access), Article 16 (rectification), Article 17 (erasure), Article 18 (restriction), and Article 20 (portability) map to specific Phase 2 endpoints; KR PIPA Article 35 정보주체의 권리 maps to the same set with documented translations. The discovery document declares the operator's enabled subject rights per jurisdiction.

### 3.4 Lawful-intercept compatibility

Jurisdictions requiring lawful intercept on surveillance feeds declare the requirement in the discovery document. Sessions in those jurisdictions emit a notice envelope on session start; the actual intercept happens through a separate signed channel that audit-logs every access.

---

## 4. Cross-standard composition

This Phase composes with adjacent WIA-family standards:

- **WIA-OMNI-API** — SOC-operator and partner-agency identity (X.509 + DID); the Phase 2 X.509 chain anchors against WIA-OMNI-API records when the city deploys WIA-family identity
- **WIA-AIR-SHIELD** — runtime trust list and key rotation for cross-agency federation
- **WIA-SOCIAL Phase 3 §5** — federation receipt shape reused for cross-agency cross-support
- **WIA-INTENT** — declaration of SOC operational intent so regulators can monitor at intent-level without exposing operational detail
- **WIA-ACCESSIBILITY** — when the SOC's public-facing tooling (incident-information portals, missing-person announcements) must be accessible per W3C WCAG 2.2 AA

---

## 5. Operational deployment runbook

A first city-scale physical-security deployment that reaches production typically follows the runbook:

| Phase | Activity | Duration |
|-------|----------|----------|
| Day 0 | Reference container stood up; conformance suite run | 1 day |
| Day 1-14 | Initial DPIA published; legal review with city counsel | 2 weeks |
| Day 15-30 | First camera-zone bridged from VMS to standard envelopes; ONVIF Profile T verified | 2 weeks |
| Day 31-60 | First access-control system bridged via OSDP v2.2 | 4 weeks |
| Day 61-75 | First cross-agency federation handshake with city police; cooperation agreement signed | 2 weeks |
| Day 76-90 | Mass-event coordination protocol exercised against a planned public event | 2 weeks |
| Day 91+ | Production cutover with shadow operation through Day 90; legacy retained as fallback | open-ended |

Lighter deployments (small municipalities with single-zone surveillance) compress this to 30 days; larger deployments (capital cities with multi-thousand-camera networks) may take 6-12 months for full bridge coverage.

---

## 6. Compliance and certification

The standard maps to:

- **ISO/IEC 27001:2022** — information security management
- **ISO/IEC 27701:2019** — privacy information management (extension of 27001)
- **ISO 22320:2018** — emergency management
- **IEC 62676 series** — video surveillance system requirements
- **IEC 62443-3-3:2013** — system security for the operational-technology side
- **NIST SP 800-53 Rev 5** — security and privacy controls
- **NIST SP 800-86** — forensic techniques in incident response
- **ASIS International POA Standard** — physical-security operations
- **ASIS International FPSM Standard** — facility physical-security measures

Operators publish a signed conformance attestation envelope that names which compliance frames they claim and which audit evidence supports each claim.

---

## 7. Roadmap

| Version | Focus |
|---------|-------|
| 1.0.0 | Initial publication: ONVIF Profile T/A/C, IEC 62676-2-31 evidence-grade, ISO 22320 emergency-management federation |
| 1.1.x | Additive: more VMS bridges, more CAD bridges, deeper analytics-event taxonomy |
| 1.2.x | Additive: mass-event coordination as a first-class envelope class with venue-management vendor bridges |
| 1.3.x | Additive: subject-rights automation for high-volume jurisdictions (GDPR, PIPA) |
| 2.0.0 (no earlier than 2028) | Possible breaking change: post-quantum signature suite migration |

The standard is maintained by the WIA Standards Committee. Change proposals follow the WIA RFC process; breaking changes require a two-thirds Committee vote plus a 12-month deprecation window per IETF RFC 8594 / 9745.

---

## 8. References

- ISO/IEC 27001:2022 — Information security management
- ISO/IEC 27002:2022 — Information security controls
- ISO/IEC 27701:2019 — Privacy information management
- ISO/IEC 27037:2012 — Digital evidence preservation
- ISO/IEC 29134:2023 — Privacy impact assessment
- ISO 22320:2018 — Emergency management
- IEC 62676-1-1:2020 — Video surveillance system requirements
- IEC 62676-2-31:2019 — IP interoperability
- IEC 62676-4:2018 — Application guidelines
- IEC 62443-3-3:2013 — System security requirements
- ONVIF Profile T / A / C 2024.06 — IP-video and access-control profiles
- NIST SP 800-53 Rev 5 — Security and Privacy Controls
- NIST SP 800-86 — Forensic techniques
- NIST SP 800-160 Vol. 2 — Cyber-resilient systems
- ASIS International POA 2018 — Protection of Assets
- W3C WCAG 2.2 — Web Content Accessibility Guidelines
- IETF RFC 8446 — TLS 1.3
- IETF RFC 8594 — sunset HTTP header
- IETF RFC 9745 — deprecation HTTP header

---

## 9. Closing implementer note

City-scale physical security is one of the most politically sensitive infrastructures in the WIA family. The wire-format discipline does not adjudicate the policy questions (how much surveillance is appropriate, what consent model applies, which agencies should cooperate); it provides the auditable wire format that lets the policy decisions be implemented, observed, and corrected over time without locking the city into a single vendor's view of the world.

A first deployment that follows the runbook reaches production in about 90 days. The depth of legal, privacy, and operational work concentrated in those 90 days is what justifies the wire-format discipline: an inspector arriving in year five with a regulatory inquiry, or a court reviewing an incident, can reconstruct any operational decision back to the signed envelope that originated it.

弘益人間 — Benefit All Humanity.


## 10. Compliance attestation envelope schema

```json
{
  "wia_city_014_version": "1.0.0",
  "type": "compliance_attestation",
  "deployment_id": "<UUID>",
  "claimed_compliance_frames": ["ISO/IEC 27001:2022", "ISO/IEC 27701:2019", "ISO 22320:2018", "GDPR Article 32", "KR PIPA Article 29"],
  "audit_evidence_uris": ["<URI>", "..."],
  "auditor_identity": "did:wia:auditor:soc-2-firm",
  "audit_completed_at_tai": "<TAI>",
  "valid_until_tai": "<TAI>",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

The attestation envelope is published on the operator's discovery
document so a regulator or partner agency can verify the claimed
compliance frames before federation handshakes proceed.


## 11. Glossary expansion for Phase 4

VMS: Video Management System, the central platform for surveillance footage. NVR: Network Video Recorder, the storage tier of a VMS. CAD: Computer-Aided Dispatch system used by emergency-response operations. EAM: Enterprise Asset Management. OSDP: Open Supervised Device Protocol, an open access-control protocol superseding Wiegand. ONVIF: Open Network Video Interface Forum, the open IP-video interoperability consortium. DPIA: Data Protection Impact Assessment per ISO/IEC 29134:2023. RIPA / IPA: UK Regulation of Investigatory Powers Act / Investigatory Powers Act, the UK lawful-intercept regime.

## 12. Implementer note — politically sensitive infrastructure

City-scale physical security is one of the most politically sensitive infrastructures in the WIA family. The wire-format discipline does not adjudicate the policy questions (how much surveillance, what consent, which agencies cooperate); it provides the auditable wire format that lets the policy decisions be implemented, observed, and corrected over time without locking the city into a single vendor's view of the world. The discipline is what distinguishes lawful surveillance with auditable evidence chain from arbitrary surveillance.
