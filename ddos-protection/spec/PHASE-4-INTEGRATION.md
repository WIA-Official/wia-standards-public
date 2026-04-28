# WIA-ddos-protection PHASE 4 — INTEGRATION Specification

**Standard:** WIA-ddos-protection
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how a DDoS-protection operator
integrates with the systems that surround the
detection-and-mitigation lifecycle: the upstream
transit providers and their DOTS endpoints; the
scrubbing-centre vendors and their BGP-redirect /
GRE / VxLAN tunnels; the CDN edge and its security
products; the national CERT for incident reporting;
the supervisory authority for the operating
jurisdiction (CISA / SEC / ENISA / KrCERT-CC / KR
ISMS-P auditors); the sector ISAC for threat-
intelligence sharing; the MANRS observatory for
routing-security attestation; the threat-
intelligence platforms; the operator's customer-
facing notification channel; the external auditor
and the ISO/IEC 27001 certification body; and the
long-term archive that preserves incident artefacts
past the active retention horizon.

References (CITATION-POLICY ALLOW only):

- IETF RFC 9132 (DOTS Signal), RFC 8783 (DOTS Data),
  RFC 8973 (DOTS Architecture)
- IETF RFC 8955 (BGP FlowSpec), RFC 7999 (BGP
  Blackhole Community)
- IETF BCP 38 / RFC 2827, BCP 84 / RFC 3704
- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022, ISO/IEC 27035-1/-2/-3
- ISO/IEC 17021-1:2015, ISO/IEC 17065:2012
- ISO 8601
- W3C Verifiable Credentials Data Model 2.0
  (optional)
- NIST SP 800-61 Rev 3
- NIST SP 800-189 (RPKI / ROA / BGPsec)
- NIST SP 800-150 (threat-information sharing)
- US CISA DDoS guide
- ENISA Threat Landscape, NIS2 Directive (EU)
  2022/2555 Articles 23 to 24 incident reporting
- MITRE ATT&CK Enterprise
- MANRS programme
- KR ISMS-P, KR 정보통신망법, KrCERT-CC

---

## §1 Upstream Transit Provider Integration

The operator integrates with each upstream transit
provider through:

- The bilateral peering or transit agreement.
- The provider's DOTS server endpoint (RFC 9132)
  and the operator's registered DOTS client
  identifier.
- The provider's BGP FlowSpec acceptance policy and
  the rate-limit on FlowSpec rules from the
  customer-side.
- The provider's blackhole community (RFC 7999) and
  the operator's published /32 blackhole policy.
- The provider's emergency-contact roster (the
  network-operator-of-watch for after-hours).

## §2 Scrubbing-Centre Vendor Integration

For cloud-based scrubbing-centre vendors the
operator's integration covers:

- The vendor's BGP-redirect anycast endpoints (the
  vendor announces the operator's prefix from the
  scrubbing-centre's anycast network during attack;
  the clean traffic is tunnelled back via GRE /
  VxLAN / IPsec).
- The vendor's DDoS console for active-attack
  visibility.
- The vendor's ASN attribution feed for source-
  attribution enrichment.
- The vendor's API for programmatic attack-policy
  changes.

## §3 CDN Edge Integration

Where the operator fronts services through a CDN:

- The CDN's edge rate-limiting product is the
  application-layer mitigation surface.
- The CDN's bot-management product feeds the
  operator's classification record.
- The CDN's TLS-fingerprint and IP-reputation feeds
  enrich the operator's detection.
- The CDN's edge cache preserves origin
  availability during volumetric attacks.

## §4 National CERT and NIS2 Integration

For EU-jurisdiction operators subject to NIS2
(Directive (EU) 2022/2555) — essential and important
entities in sectors covered by NIS2 Annex I and
Annex II:

- Article 23 incident-notification: significant
  incidents are reported to the CSIRT or competent
  authority — early warning within 24 hours;
  incident notification within 72 hours; final
  report within one month.
- Article 24 voluntary notification of significant
  cyber threats.

For US-jurisdiction operators the equivalent CISA
notification under CIRCIA (the Cyber Incident
Reporting for Critical Infrastructure Act of 2022)
+ sector-specific reporting (e.g., FCC for
telecommunications operators, SEC for public
companies under 17 CFR 229.106 / 8-K Item 1.05
material cybersecurity-incident reporting).

For KR-jurisdiction operators the KrCERT-CC
notification under 정보통신망법 Article 48-3 침해사고
신고 + 전자금융거래법 incident reporting where
applicable.

## §5 Sector ISAC Integration

The operator participates in the relevant sector
ISAC (FS-ISAC for financial services, H-ISAC for
healthcare, Auto-ISAC for automotive, K-ISAC for
KR-jurisdiction) for threat-intelligence sharing
under NIST SP 800-150 + the ISAC's published
operating procedures. STIX 2.1 + TAXII 2.1 are the
canonical machine-to-machine sharing wire formats.

## §6 MANRS Observatory Integration

For network-operator-class operators the MANRS
observatory tracks the operator's compliance with
the MANRS Network Operator Actions (filtering, anti-
spoofing, coordination, global validation). The
operator's MANRS participant identifier is published
on the observatory; periodic re-attestations are
exercised.

## §7 Threat-Intelligence Platform Integration

The operator's TIP integrates feeds from:

- Spamhaus DROP / EDROP / SBL / XBL / PBL.
- Team Cymru BGP feeds (bogon list, full BGP table
  comparison).
- Sector ISACs.
- National-CERT advisories.
- Commercial threat-intelligence vendors.
- The operator's own honeypot and dark-net
  observation.

The TIP normalises feeds to STIX 2.1 and feeds the
detection and classification records.

## §8 Customer-Facing Notification Channel

The operator's customer-facing notification covers:

- Status-page updates (the operator's published
  service-status surface with the current incident
  state, expected duration, and remediation
  progress).
- Email / SMS / Slack notification per the customer's
  configured channel.
- Telephone bridge for high-severity incidents.
- Post-incident report delivered to the customer
  within the operator's published SLA.

## §9 External Audit and ISMS Certification

The operator's ISMS is certified against ISO/IEC
27001:2022 with the scope explicitly extending to
the DOTS, scrubbing, and detection endpoints.
Compliance with ISO/IEC 27035-1 + 27035-2 +
27035-3 incident-management standards is asserted.
The certification body operates under ISO/IEC
17021-1; the conformity-assessment body for WIA-
ddos-protection operates under ISO/IEC 17065.

## §10 Long-Term Archival Integration

Records governed by the operator's retention
horizons (ISO/IEC 27035-2 §10 incident-record
retention; KR ISMS-P 보존 의무; the operator's
internal three-to-five-year retention) are migrated
to the long-term archive at the close of the
active retention window. The archive preserves the
attack-detection record, the DOTS exchange record,
the mitigation-action record, the post-incident
analysis record, the BCP-attestation record, the
cooperation correspondence record, and the audit-
event trail.

## §11 Sectoral Resilience-Reporting Integration

For sector-specific operators the resilience-
reporting integration extends:

- DORA Regulation (EU) 2022/2554 — for financial
  entities the major-ICT-incident reporting under
  Article 19 + threat-led penetration testing
  under Article 26 (TIBER-EU framework).
- US Reg SCI — for SCI entities the immediate
  notification (Rule 1002) and member disclosure
  (Rule 1003).
- KR 전자금융거래법 + 전자금융감독규정 — for KR-
  jurisdiction financial operators the FSC / FSS
  notification.
- Telecommunications-sector operators integrate
  with the sector regulator (FCC in US; BNetzA /
  Member-State NRA in EU; KCC + MSIT in KR) under
  the sector's published incident-reporting regime.

## §12 Identity-Federation and Access Integration

For multi-tenant scrubbing operators and managed-
security-service providers the operator integrates
with the customer's identity-federation surface
(SAML 2.0 + OAuth 2.1 + OpenID Connect 1.0) so
that customer administrators access the DDoS
console under the customer's home-realm
authentication. The customer's authorisation
discipline (admin / operator / read-only)
constrains the API surface available to the
federated identity.

## §13 Cross-Operator Cooperation Discipline

Distributed attacks frequently span multiple operator
boundaries; the operator's cross-operator cooperation
discipline covers:

- The bilateral routing-security peers (MANRS
  participants).
- The multi-operator FIRST (Forum of Incident
  Response and Security Teams) network for
  CSIRT-to-CSIRT exchange.
- The Trusted Introducer programme for European
  CSIRTs.
- The operator's participation in the Anti-Phishing
  Working Group (APWG) and the Spamhaus / Shadowserver
  observation networks.
- The voluntary publication of attack signatures
  through MISP (Malware Information Sharing
  Platform) where the operator participates.

## §14 Resilience-Drill and Tabletop Exercise Integration

The operator's drill schedule integrates with:

- The upstream provider's joint-drill schedule.
- The sector-coordination drill (FS-ISAC's annual
  CAPS exercise; CISA's Cyber Storm; ENISA's Cyber
  Europe; KrCERT-CC's 사이버 위기 대응 훈련).
- The customer's joint tabletop where the operator
  manages a critical-infrastructure customer.

## §15 IoT and Edge-Device Vendor Coordination

Where the operator's threat surface includes IoT-
botnet-driven attacks (Mirai-class, Mozi-class,
Bashlite-class) the operator coordinates with:

- The compromised-device vendor for firmware-update
  push (where the vendor maintains an OTA channel).
- The operator's residential ISP customers for
  customer-notification programmes (the operator's
  walled-garden discipline).
- The IoT certification programmes (UK PSTI Act
  2022, EU Cyber Resilience Act regulatory regime
  once in force) which set baseline cybersecurity
  expectations for IoT devices entering the EU
  market.

## §16 Conformance

Implementations claiming PHASE-4 conformance maintain
the upstream-transit and scrubbing-centre integrations,
exercise the national-CERT and NIS2 (where
applicable) reporting obligations, integrate with
the sector ISAC and MANRS observatory where the
operator's role calls for participation, hold the
ISO/IEC 27001 certification, exercise the IoT-vendor
coordination discipline where the threat surface
warrants, and operate the long-term archival
integration described above.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-ddos-protection
- **Last Updated:** 2026-04-28
