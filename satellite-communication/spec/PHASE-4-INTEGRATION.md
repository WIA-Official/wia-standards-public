# WIA-SPACE-003 (satellite-communication) — Phase 4: Integration Specification

> **Version:** 1.0.0
> **Status:** Draft
> **Phase:** 4 of 4 (Integration)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 4 specifies how a WIA-SPACE-003 deployment integrates with the broader satellite-communication ecosystem: ground-segment vendor stacks (Kratos, SES Networks, Telesat NMS, KARI Mission Control, JAXA Tracking Network), bridge profiles to legacy Space Link Extension (SLE) deployments, regulatory-reporting pipelines, debris-mitigation programs, and the Earth-station consortium tooling that complements the operator's own infrastructure.

---

## 2. Bridge profiles

### 2.1 Bridge to legacy SLE deployments

Most operational ground stations already speak SLE. The bridge profile maps Phase 2 endpoints to SLE service primitives without requiring the legacy stack to be replaced.

| Phase 2 endpoint | SLE primitive | Bridge mapping |
|------------------|---------------|----------------|
| `POST /telemetry-session` | RAF / RCF BIND | service ID and SCID populated from Phase 1 spacecraft descriptor |
| `GET /telemetry-session/{id}/feed` | RAF / RCF transfer | SLE-PDU transfer-data wrapped as CBOR for the SSE stream |
| `POST /bundle-session/{id}/bundle` (uplink) | CLTU TRANSFER-DATA | bundle bytes passed through the CLTU service |
| `POST /link/{id}/handover` | SLE service-management peer-abort + new BIND | coordination via SLE Service Management per CCSDS 921.1-B |

The bridge container ships at `https://github.com/WIA-Official/wia-satellite-communication-bridges/sle-bridge` with reference implementations against the CCSDS Java SLE API and the OS3 SLE API.

### 2.2 Bridge to flight-dynamics systems

Flight-dynamics systems (FDS) generate orbit determinations, manoeuvre plans, and conjunction predictions. The bridge profile maps Phase 1 spacecraft state and Phase 3 conjunction-warning envelopes to common FDS exchange formats:

- **CCSDS Navigation Data Messages** (CCSDS 502.0-B Orbit Data Messages, 503.0-B-1 Tracking Data Messages, 504.0-B Attitude Data Messages, 508.0-B Conjunction Data Messages) — primary interchange format
- **STK / FreeFlyer / GMAT** — domain-specific tools for visualisation and what-if analysis

### 2.3 Bridge to terrestrial backhaul

Operators connect ground stations to mission control over MPLS, IP-VPN, or dedicated dark fibre. The bridge profile carries Phase 2 envelopes over standard TLS 1.3 (RFC 8446) on the terrestrial leg; the SDLS protection on the space leg survives independently.

### 2.4 Bridge to user-terminal mass-market platforms

Mass-market user terminals (VSAT, marine VSAT, aviation Ku/Ka, mobile direct-to-device) integrate via DVB-S2X (ETSI EN 302 307-2) and DVB-RCS2 (ETSI EN 301 545-2) for the air interface and via Phase 2 envelopes for the operator-facing control plane. The bridge profile names the supported terminal classes per operator in the discovery document.

---

## 3. Regulatory-reporting integration

### 3.1 ITU-R BR notification flow

Operators submit advance publication, coordination, and notification under Article 22 to ITU-R BR via the BR's electronic submission system (currently SpaceCap / SpaceQry / e-Submission of Filings). The Phase 4 reporting bridge maps the operator's filing record to the BR submission format, attaches the supporting link-budget evidence (Phase 1 §2.4), and tracks the BR response chain.

### 3.2 National administration filings

Each national administration runs its own filing pipeline:

| Administration | System | Bridge artefact |
|----------------|--------|-----------------|
| FCC (US) | International Bureau Filing System (IBFS) | bridges/fcc-ibfs.json |
| Ofcom (UK) | Satellite licensing register | bridges/ofcom-sat.json |
| MIC (Japan) | 周波数管理 (frequency management) system | bridges/mic.json |
| MSIT (Korea) | 무선국 데이터베이스 (radio-station DB) | bridges/msit-radio.json |
| BNetzA (Germany) | Frequency-management database | bridges/bnetza.json |
| ANATEL (Brazil) | Satellite-network registry | bridges/anatel.json |

Each bridge documents the per-administration field mapping and the filing-response polling cadence.

### 3.3 Debris-mitigation reporting (ISO 24113 + IADC)

Operators submit debris-mitigation plans under ISO 24113 Space debris mitigation requirements; the IADC (Inter-Agency Space Debris Coordination Committee) provides the international coordination forum. The Phase 4 reporting bridge maps the operator's deorbit-plan envelope (Phase 1 §2.6) to ISO 24113 conformance evidence and to the IADC standardised reporting template.

### 3.4 Lawful-intercept compatibility

Jurisdictions requiring lawful intercept on satellite-internet user traffic (US under CALEA, EU under ETSI ES 201 671, KR under 통신비밀보호법) declare the requirement in the discovery document. Sessions in those jurisdictions emit a notice envelope to participants on session start; the actual intercept happens through a separate signed channel that audit-logs every access, and the SDLS protection of the space-segment leg is unaffected.

---

## 4. Cross-standard composition

This Phase composes with adjacent WIA-family standards:

- **WIA-OMNI-API** — operator and console identity (X.509 + DID); the Phase 2 X.509 chain is anchored against WIA-OMNI-API records when applicable
- **WIA-AIR-SHIELD** — runtime trust list maintenance and key rotation; SDLS keys rotated under CCSDS 354.0-M-2 are also tracked through the WIA-AIR-SHIELD audit chain for cross-standard visibility
- **WIA-SOCIAL Phase 3 §5** — federation receipt shape reused for cross-agency cross-support handshakes
- **WIA-INTENT** — outermost-layer declaration of mission intent so the operations team can verify the workload's intent matches the spacecraft's actual operations
- **WIA Quantum Communication** — composes when the deployment runs satellite QKD; the QKD-derived key may be rotated into SDLS via a documented bridge

---

## 5. Operational deployment runbook

A first satellite-communication deployment that reaches production typically follows the runbook:

| Phase | Activity | Duration |
|-------|----------|----------|
| Day 0 | Reference container stood up; conformance suite run | 1 day |
| Day 1-7 | SDLS key onboarding under CCSDS 354.0-M-2; ITU filing record imported | 1 week |
| Day 8-21 | First spacecraft commissioning; TM/TC bridge to vendor stack verified | 2 weeks |
| Day 22-45 | First cross-support agreement under CCSDS 902.0-B; partner-agency handshake exercised | 3-4 weeks |
| Day 46-60 | Conjunction-warning subscription wired to CSpOC / EU SST / KARI | 2 weeks |
| Day 61-90 | Production cutover with shadow operation through Day 60; legacy retained as fallback | 4 weeks |

The 90-day timeline accommodates conformance-suite passes, ITU coordination, and the regulator-notification cadence typical for space-segment licensing. Lighter deployments (small-sat operators with limited orbital regimes) compress this to 30 days.

---

## 6. Compliance and certification

The standard maps to:

- **ISO 24113** — space debris mitigation requirements
- **ISO/IEC 27001:2022** — information security management for ground-segment operations
- **IEC 62443-3-3:2013** — system security for the operational-technology side of mission control
- **CCSDS 350.0-G** — security guide for mission planners
- **CCSDS 350.4-G** — CCSDS guide for security architecture
- **ECSS-E-ST-50** series — space communications standards (European Cooperation for Space Standardization)
- **NIST SP 800-160 Vol. 2** — systems security engineering, applicable to ground-segment systems

Operators publish a signed conformance attestation envelope that names which of these compliance frames they claim and which audit evidence supports the claim.

---

## 7. Roadmap

| Version | Focus |
|---------|-------|
| 1.0.0 | Initial publication: SDLS / TM / TC / BPv7 stable; ITU-R + national-administration bridges |
| 1.1.x | Additive: more national-administration bridges; richer DVB-S2X / DVB-RCS2 modulation profiles |
| 1.2.x | Additive: ISL / optical-link reference profile; satellite-QKD reference integration |
| 1.3.x | Additive: full quantum-internet composition with WIA Quantum Network for intercontinental key distribution |
| 2.0.0 (no earlier than 2028) | Possible breaking change: post-quantum signature suite migration following NIST PQC selection |

The standard is maintained by the WIA Standards Committee. Change proposals follow the WIA RFC process; breaking changes require a two-thirds Committee vote plus a 12-month deprecation window per IETF RFC 8594 / 9745.

---

## 8. References

- CCSDS 350.0-G — Security Guide for Mission Planners
- CCSDS 350.4-G — CCSDS Guide for Secure System Engineering
- CCSDS 502.0-B — Orbit Data Messages
- CCSDS 503.0-B-1 — Tracking Data Message
- CCSDS 504.0-B — Attitude Data Messages
- CCSDS 508.0-B — Conjunction Data Messages
- CCSDS 902.0-B — Cross-Support Concept and Reference Architecture
- CCSDS 921.1-B — SLE Service Management
- CCSDS 354.0-M-2 — Space Mission Key Management
- ITU-R Radio Regulations Article 22 — Space services
- ITU-R BR e-Submission system documentation
- ETSI EN 302 307-2 — DVB-S2X
- ETSI EN 301 545-2 — DVB-RCS2
- ETSI ES 201 671 — Lawful Intercept
- ECSS-E-ST-50 — Space communications standards series
- ISO 24113 — Space debris mitigation
- ISO/IEC 27001:2022
- IEC 62443-3-3:2013
- NIST SP 800-160 Vol. 2 — Developing cyber-resilient systems
- IETF RFC 8446 — TLS 1.3
- IETF RFC 8594 — sunset HTTP header
- IETF RFC 9745 — deprecation HTTP header

---

弘益人間 — Benefit All Humanity.


## 9. Closing implementer note

Satellite communications is one of the few infrastructures where the
software stack must remain operationally correct across decades of
hardware lifecycle. A spacecraft launched today may still be in
service in 2040; the wire formats and protocol disciplines that
operate it must absorb that horizon without locking the operator
into a single vendor or a single regulator's interpretation.

This standard is intentionally conservative on protocol invention
and intentionally rigorous on audit. The CCSDS family already has
the wire formats — TM, TC, SDLS, BPv7 — that the space domain has
agreed on; the standard's contribution is the wrapper-level
discipline that makes those wire formats survive the multi-agency
multi-jurisdiction multi-vendor reality of modern operations.

A first deployment that follows the runbook in §5 reaches production
in about 90 days. The depth of audit and compliance work
concentrated in those 90 days is what justifies the wire-format
discipline: an inspector arriving in year five with a regulatory
inquiry can reconstruct any operational decision back to the
signed envelope that originated it.

弘益人間 — Benefit All Humanity.


## 10. Glossary expansion for Phase 4

CCSDS: Consultative Committee for Space Data Systems, the international standardisation body for space-segment data systems. SLE: Space Link Extension, the canonical CCSDS service framework. RAF / RCF / CLTU / ROCF: SLE service primitives for return all frames, return channel frames, forward command-link transmission units, and return online channel frames respectively. SDLS: Space Data Link Security per CCSDS 355.0-B-2. ITU-R BR: International Telecommunication Union Radiocommunication Bureau. IADC: Inter-Agency Space Debris Coordination Committee. DVB-S2X / DVB-RCS2: ETSI second-generation satellite broadcasting and return-channel standards.

## 11. Implementer note — multi-decade horizon

Spacecraft operate for 15-25 years on average; the ground-segment software that operates them must remain compatible across that horizon. The standards backwards-compatibility promise (within the 1.x line, no Phase field shape, no endpoint, no protocol exchange will be removed) is therefore mandatory rather than aspirational. Operators making capital-allocation decisions on hardware already in orbit need confidence that the wire format will outlast the silicon.
