# WIA-ddos-protection PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-ddos-protection
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format
layer for WIA-ddos-protection. The standard covers
persistent record shapes for the lifecycle of a
distributed-denial-of-service (DDoS) protection
programme — the protected service inventory and the
protected-asset record; the network-and-application
baseline traffic profile; the threat-and-attack
record (volumetric, protocol-state-exhaustion,
application-layer); the DOTS (DDoS Open Threat
Signaling) signal channel + data channel record; the
mitigation-action record (rate-limiting, traffic
scrubbing, BGP FlowSpec announcement, blackhole
routing, anycast diversion, captcha challenge); the
post-incident analysis record; the BCP 38 ingress-
filtering and BCP 84 multihomed-network filtering
attestation; the cooperation record with the operator's
upstream transit provider, scrubbing-centre vendor,
and CDN; and the supervisory-and-CERT correspondence
record.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID) and IETF RFC 4122 (UUID URN)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27002:2022 (information security controls)
- ISO/IEC 27035-1:2023 + 27035-2:2023 + 27035-3:2024
  (information security incident management)
- IETF RFC 4732 (Internet Denial-of-Service
  Considerations)
- IETF RFC 8499 (DNS Terminology, the canonical
  vocabulary for DNS-amplification attacks)
- IETF RFC 8612 (DDoS Open Threat Signaling
  Requirements)
- IETF RFC 8782 / 9132 (DOTS Signal Channel
  Specification — RFC 9132 obsoletes 8782)
- IETF RFC 8783 (DOTS Data Channel)
- IETF RFC 8973 (DDoS-Open-Threat-Signaling
  Architecture)
- IETF RFC 8955 (BGP FlowSpec, the modern
  replacement for RFC 5575)
- IETF RFC 5575 (BGP FlowSpec original; cited as
  the legacy reference where deployed)
- IETF RFC 7039 (Source Address Validation
  Improvement (SAVI))
- IETF RFC 8174 / 2119 (key-words for normative
  language)
- IETF BCP 38 / RFC 2827 (Network Ingress Filtering)
- IETF BCP 84 / RFC 3704 (Ingress Filtering for
  Multihomed Networks)
- IETF RFC 8259 (JSON), RFC 9457 (Problem Details)
- NIST SP 800-61 Rev 3 (Computer Security Incident
  Handling Guide)
- NIST SP 800-189 (Resilient Interdomain Traffic
  Exchange — BGP origin validation, ROA, RPKI, BGPsec)
- NIST SP 800-53 Rev 5 (security and privacy
  controls)
- US CISA DDoS guide and the CISA Stop Ransomware
  campaign DDoS supplements
- ENISA Threat Landscape (the European Union Agency
  for Cybersecurity's annual report) and ENISA
  Guidelines on Network and Information Security
- MITRE ATT&CK Enterprise (Tactic TA0040 Impact;
  Technique T1498 Network Denial of Service; T1499
  Endpoint Denial of Service)
- KR ISMS-P (정보보호 및 개인정보보호 관리체계 인증)
  framework operated by KISA
- KR 정보통신망법 + KR 전자금융감독규정 + KR-CERT
  (KrCERT/CC) cooperation discipline

---

## §1 Scope

This PHASE defines persistent shapes for the
artefacts a DDoS-protection operator (an enterprise's
security team, a managed-security-service provider,
a CDN edge operator, a scrubbing-centre vendor, an
ISP transit provider, or a national CERT) maintains:

- The protected-service inventory.
- The traffic-baseline record.
- The attack-detection record.
- The DOTS signal-and-data-channel record.
- The mitigation-action record.
- The post-incident analysis record.
- The BCP 38 / BCP 84 attestation record.
- The vendor and partner cooperation record.
- The supervisory and CERT correspondence record.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
operatorName         : string (legal name)
operatorRole         : enum ("enterprise-security" |
                       "managed-security-service-
                       provider" | "cdn-edge" |
                       "scrubbing-centre" |
                       "isp-transit" | "national-
                       cert" | "user-defined")
operatorAsn          : string (the operator's
                       autonomous-system number,
                       AS-format ASN; absent for
                       non-routed operators)
operatorJurisdiction : array of string (ISO 3166-1)
governingFrameworks  : array of enum ("RFC-4732" |
                       "RFC-8499" | "RFC-8612" |
                       "RFC-9132-DOTS-SIGNAL" |
                       "RFC-8783-DOTS-DATA" |
                       "RFC-8955-BGP-FLOWSPEC" |
                       "BCP-38-RFC-2827" |
                       "BCP-84-RFC-3704" |
                       "NIST-SP-800-61R3" |
                       "NIST-SP-800-189" |
                       "NIST-SP-800-53R5" |
                       "ISO-IEC-27001" |
                       "ISO-IEC-27035" |
                       "MITRE-ATTACK-T1498" |
                       "MITRE-ATTACK-T1499" |
                       "ENISA-THREAT-LANDSCAPE" |
                       "CISA-DDOS-GUIDE" |
                       "KR-ISMS-P" |
                       "KR-정보통신망법" |
                       "KR-CERT-COOPERATION" |
                       "user-defined")
programmeStatus      : enum ("design" | "operating"
                       | "limited-rollout" |
                       "wind-down" | "archived")
```

## §3 Protected-Service Inventory Record

```
protectedService:
  serviceId          : string (uuidv7)
  serviceName        : string
  serviceClass       : enum ("public-web" | "api-
                       endpoint" | "authoritative-
                       dns" | "recursive-dns" |
                       "mail-receiver" | "voip-sbc"
                       | "game-server" | "iot-mqtt"
                       | "infrastructure-routing" |
                       "user-defined")
  publicEndpointSet  : array of object (per-
                       endpoint IPv4 / IPv6 prefix,
                       transport protocol, port, and
                       fully-qualified domain name)
  asnOriginatedFrom  : string (the operator's ASN)
  upstreamTransitRef : array of string (the upstream
                       transit AS-set the operator
                       receives prefixes through)
  scrubbingCentreRef : array of string (the
                       scrubbing-centre vendor
                       references — for cloud-based
                       traffic scrubbing the
                       operator's contract reference
                       is recorded)
  cdnRef             : array of string (the CDN
                       provider references; absent
                       for non-CDN-fronted services)
  recoveryTimeObjective : object (the operator's
                       documented RTO under attack)
```

## §4 Traffic-Baseline Record

```
trafficBaseline:
  baselineId         : string (uuidv7)
  serviceRef         : string (PHASE-1 §3)
  measuredFrom       : string (ISO 8601)
  measuredUntil      : string (ISO 8601)
  pps                : object (mean and 99th
                       percentile packets-per-second)
  bps                : object (mean and 99th
                       percentile bits-per-second)
  cps                : object (mean and 99th
                       percentile new-connections-
                       per-second)
  rpsApplication     : object (mean and 99th
                       percentile HTTP-requests-per-
                       second; absent for non-web
                       services)
  geoDistribution    : array of object (per-ISO-
                       3166-1 country traffic
                       fraction)
  asnDistribution    : array of object (per-source-
                       ASN traffic fraction)
  baselineMethod     : enum ("rolling-30-day-
                       window" | "exponentially-
                       weighted-moving-average" |
                       "seasonal-arima" |
                       "peer-comparison" | "user-
                       defined")
```

## §5 Attack-Detection Record

```
attackDetection:
  detectionId        : string (uuidv7)
  serviceRef         : string
  detectedAt         : string (ISO 8601 instant;
                       millisecond precision
                       recommended)
  attackVectorKind   : enum ("volumetric-udp-flood"
                       | "volumetric-icmp-flood" |
                       "volumetric-syn-flood" |
                       "volumetric-amplification-
                       reflection-dns" |
                       "volumetric-amplification-
                       reflection-ntp" |
                       "volumetric-amplification-
                       reflection-memcached" |
                       "volumetric-amplification-
                       reflection-cldap" |
                       "volumetric-amplification-
                       reflection-ssdp" |
                       "protocol-state-exhaustion-
                       tcp-state" | "protocol-state-
                       exhaustion-tls-handshake" |
                       "application-layer-http-
                       slowloris" | "application-
                       layer-http-flood" |
                       "application-layer-rapid-
                       reset-cve-2023-44487" |
                       "application-layer-quic-
                       flood" | "application-layer-
                       dns-water-torture" |
                       "low-and-slow" |
                       "carpet-bombing" |
                       "user-defined")
  peakRateObserved   : object (peak pps / bps / cps
                       / rps observed during the
                       attack window)
  durationSeconds    : integer (the detection-to-
                       attack-end window)
  attackerProfile    : object (source-ASN
                       distribution; geographic
                       distribution; the
                       operator's identification of
                       the booter / stresser
                       service or botnet family
                       reference where attribution
                       is possible)
  mitreTechniqueRef  : array of string (MITRE
                       ATT&CK Enterprise technique
                       identifiers — T1498 sub-
                       techniques for network DoS,
                       T1499 sub-techniques for
                       endpoint DoS)
```

## §6 DOTS Signal-and-Data-Channel Record

The DOTS protocol carries the operator's request
for upstream mitigation between the DOTS client
(at the protected operator) and the DOTS server
(at the upstream mitigation provider) per RFC 9132
(signal channel) and RFC 8783 (data channel):

```
dotsRecord:
  dotsExchangeId     : string (uuidv7)
  detectionRef       : string (PHASE-1 §5)
  dotsClientId       : string (the operator's DOTS
                       client identifier)
  dotsServerRef      : string (the upstream
                       mitigation provider's DOTS
                       server reference)
  signalChannelKind  : enum ("rfc-9132-coap" |
                       "user-defined")
  dataChannelKind    : enum ("rfc-8783-restconf" |
                       "user-defined")
  mitigationScope    : object (the IPv4 / IPv6
                       target prefixes the operator
                       requests mitigation for)
  mitigationLifetime : integer (seconds the
                       operator requests mitigation
                       to remain active)
  exchangedAt        : string (ISO 8601)
  outcomeKind        : enum ("accepted" |
                       "accepted-with-conflict" |
                       "rejected" |
                       "withdrawn-by-client" |
                       "expired")
```

## §7 Mitigation-Action Record

```
mitigationAction:
  actionId           : string (uuidv7)
  detectionRef       : string
  actionKind         : enum ("rate-limiting-edge" |
                       "rate-limiting-syn-cookies"
                       | "rate-limiting-quic-
                       initial-rtt-validation" |
                       "scrubbing-centre-divert" |
                       "scrubbing-centre-bgp-
                       redirect" |
                       "scrubbing-centre-gre-
                       tunnel-back" |
                       "scrubbing-centre-vxlan-
                       tunnel-back" |
                       "bgp-flowspec-announcement"
                       | "bgp-blackhole-
                       community-rfc-7999" |
                       "anycast-diversion" |
                       "geo-blocking" |
                       "asn-blocking" |
                       "captcha-challenge" |
                       "javascript-challenge" |
                       "tls-fingerprint-block" |
                       "user-defined")
  appliedAt          : string (ISO 8601)
  releasedAt         : string (ISO 8601; absent
                       until released)
  effectivenessMetric : object (post-action peak
                       rate observed; legitimate-
                       traffic false-positive rate
                       observed)
  reviewerRef        : string (the operator's
                       incident-commander or auto-
                       mitigation engine reference)
```

## §8 Post-Incident Analysis Record

```
postIncidentAnalysis:
  analysisId         : string (uuidv7)
  detectionRef       : string
  analysisKind       : enum ("root-cause-attack-
                       reconstruction" |
                       "mitigation-effectiveness-
                       review" | "control-gap-
                       review" | "vendor-cooperation
                       -review" | "user-defined")
  conductedAt        : string (ISO 8601)
  findingsRef        : string (URI of the findings
                       narrative — covers the NIST
                       SP 800-61 Rev 3 Detection
                       and Analysis + Containment
                       Eradication and Recovery +
                       Post-Incident Activity
                       phases)
  correctiveActions  : array of object (planned
                       remediation, owner, due
                       date, completion status)
```

## §9 BCP 38 / BCP 84 Attestation Record

The operator's BCP 38 ingress-filtering and BCP 84
multihomed-network-filtering attestation is the
upstream-policy declaration that source-address-
spoofed traffic does not leave the operator's
network. The attestation aligns with the MANRS
(Mutually Agreed Norms for Routing Security)
discipline:

```
bcpAttestation:
  attestationId      : string (uuidv7)
  reportedAt         : string (ISO 8601)
  bcp38Compliant     : boolean
  bcp84Compliant     : boolean
  manrsParticipantId : string (the MANRS programme
                       identifier; absent if not a
                       MANRS participant)
  rpkiOrigin         : enum ("validated" | "not-
                       found" | "invalid" |
                       "user-defined") (per NIST
                       SP 800-189; the operator's
                       RPKI ROA / origin-validation
                       posture)
  bgpsecRollout      : enum ("rolled-out" |
                       "partial-rollout" | "not-
                       rolled-out") (per NIST SP
                       800-189)
```

## §10 Cooperation and Correspondence Record

```
cooperationRecord:
  recordId           : string (uuidv7)
  partnerKind        : enum ("upstream-transit-
                       provider" | "scrubbing-
                       centre-vendor" | "cdn-
                       provider" | "national-cert"
                       | "law-enforcement" |
                       "abuse-mailbox-correspondence"
                       | "isac" | "user-defined")
  partnerRef         : string
  exchangedAt        : string (ISO 8601)
  contentRef         : string (URI of the
                       correspondence — DOTS
                       request, telephone-bridge
                       summary, NCSC / KrCERT-CC
                       cooperation note, etc.)
  outcomeKind        : enum ("acknowledged" |
                       "in-progress" | "closed-
                       with-finding" | "closed-no-
                       further-action" |
                       "user-defined")
```

## §11 Conformance

Implementations claiming PHASE-1 conformance maintain
the records defined above for the operator's
protected services, exercise BCP 38 / BCP 84
ingress-filtering on the operator's egress, exercise
the DOTS signal-channel exchange with at least one
upstream mitigation provider, and preserve the
incident records under the operating jurisdiction's
recordkeeping discipline (ISO/IEC 27035-2 §10
record retention; KR ISMS-P 보존 의무; the operator's
internal retention horizon, typically three to five
years).

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-ddos-protection
- **Last Updated:** 2026-04-28
