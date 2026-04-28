# WIA-ddos-protection PHASE 3 — PROTOCOL Specification

**Standard:** WIA-ddos-protection
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern a
DDoS-protection operator: the traffic-baseline and
anomaly-detection discipline; the volumetric /
protocol-state-exhaustion / application-layer attack-
classification discipline; the DOTS signal-and-data
channel discipline (RFC 9132 + RFC 8783); the
mitigation-application discipline (rate-limiting,
scrubbing-centre diversion, BGP FlowSpec, BGP
blackhole, anycast diversion, captcha challenge);
the BCP 38 + BCP 84 ingress-filtering discipline
that prevents the operator's network from being a
source of spoofed traffic; the RPKI / BGPsec
discipline (NIST SP 800-189) that mitigates BGP
hijack and route-leak attacks against the operator's
prefixes; the post-incident analysis discipline
under NIST SP 800-61 Rev 3 + ISO/IEC 27035; the
threat-intelligence integration discipline (MITRE
ATT&CK); and the supervisory / CERT cooperation
discipline.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO/IEC 27001:2022 + 27002:2022 + 27035-1/-2/-3
- IETF RFC 4732, 8499, 8612, 8782, 9132, 8783, 8973
- IETF RFC 8955, 5575, 7039, 7011 (IPFIX)
- IETF RFC 7999 (BGP Blackhole Community), RFC 9234
- IETF BCP 38 / RFC 2827, BCP 84 / RFC 3704
- NIST SP 800-61 Rev 3 (Computer Security Incident
  Handling Guide)
- NIST SP 800-189 (Resilient Interdomain Traffic
  Exchange — RPKI / ROA / BGPsec)
- NIST SP 800-53 Rev 5 (controls)
- NIST SP 800-150 (Cyber Threat Information Sharing)
- US CISA DDoS guide
- ENISA Threat Landscape, ENISA NIS2 Directive
  guidance
- MITRE ATT&CK Enterprise (T1498, T1499 sub-
  techniques)
- MANRS (Mutually Agreed Norms for Routing Security)
- KR ISMS-P, KR 정보통신망법, KR-CERT (KrCERT/CC)

---

## §1 Traffic-Baseline and Detection Discipline

The operator maintains traffic baselines (PHASE-1 §4)
on a rolling window. Detection is layered:

- Network-layer detection — pps / bps / cps anomalies
  against the baseline trigger the network-layer
  alert.
- Application-layer detection — rps anomalies, error-
  rate anomalies, latency spikes, and per-URL
  anomalies trigger the application-layer alert.
- DNS-layer detection — query-rate anomalies and the
  signatures of DNS amplification and DNS water-
  torture attacks per RFC 8499 trigger the DNS
  alert.
- Multi-tenant cross-correlation — anomalies
  observed across multiple protected services with
  shared upstream-transit fingerprints raise a
  carpet-bombing alert.

Detection thresholds are documented and reviewed
on the operator's published cadence so that
threshold-decay does not produce missed alerts.

## §2 Attack-Classification Discipline

The classification discipline maps detected anomalies
to one of the canonical attack classes:

- Volumetric — UDP flood, ICMP flood, SYN flood,
  amplification-reflection (DNS, NTP, memcached,
  CLDAP, SSDP). The operator applies the published
  amplification-factor ranges to estimate the
  reflector population.
- Protocol-state-exhaustion — TCP-state exhaustion,
  TLS-handshake exhaustion, QUIC-flood, HTTP/2
  Rapid Reset (CVE-2023-44487).
- Application-layer — HTTP flood, slowloris,
  application-specific resource exhaustion (search-
  query flooding, login-form flooding, cart-flood-
  on-checkout).
- Low-and-slow — sub-baseline rate sustained over
  long windows that exhausts state.
- Carpet-bombing — fragmented attacks against many
  small targets across the operator's prefix.

Each classification routes to the appropriate
mitigation chain (PHASE-3 §4).

## §3 DOTS Signal-and-Data-Channel Discipline

The DOTS protocol orchestrates upstream mitigation
(RFC 9132 + RFC 8783):

1. The operator's DOTS client maintains a long-lived
   data-channel session with each upstream
   mitigation provider, declaring aliases (named
   target prefixes / port ranges) and baseline
   ACLs.
2. At attack detection the DOTS client raises a
   signal-channel mitigation request referencing the
   alias (or directly the target prefix), the
   requested lifetime, and the trigger event.
3. The DOTS server returns the mitigation-status
   reports; the operator monitors `mitigation-
   start`, `mitigation-active`, and `mitigation-
   terminating` transitions.
4. When the attack ends the operator withdraws the
   request via DELETE on the signal channel; the
   provider releases the mitigation.
5. The exchange is recorded in PHASE-1 §6 with the
   audit-event for forensic reconstruction.

## §4 Mitigation-Application Discipline

Mitigation chains by attack class:

- Volumetric → upstream scrubbing-centre divert
  (BGP-redirect via /24 or /32 anycast announcement,
  GRE / VxLAN tunnel back), BGP FlowSpec rules
  (RFC 8955) at upstream transit, BGP blackhole
  community (RFC 7999) for /32 source-of-attack
  preferences.
- Protocol-state-exhaustion → SYN cookies (RFC 4987-
  era, now baseline), TCP-state limits, QUIC initial-
  RTT validation, TLS handshake rate-limiting.
- Application-layer → edge rate-limiting per source-
  IP / per-cookie / per-fingerprint, CDN edge JS
  challenge, captcha challenge, TLS-fingerprint
  blocklists, web-application-firewall rules.
- Low-and-slow → connection-time-limit enforcement,
  per-source state-track limits.
- Carpet-bombing → /24-aware FlowSpec rules,
  upstream provider direct intervention.

The operator's policy-decision point evaluates the
classification, the mitigation chain, the false-
positive risk, and the legitimate-traffic impact
before applying. Four-eyes approval applies for
mitigations that affect more than the attacked
prefix.

## §5 BCP 38 / BCP 84 Ingress-Filtering Discipline

The operator enforces source-address validation on
ingress to prevent spoofed traffic from leaving the
operator's network:

- BCP 38 (RFC 2827) — at the customer-aggregation
  interface, the operator enforces uRPF strict-mode
  or ACL-based ingress filtering matching the
  customer's allocated prefix.
- BCP 84 (RFC 3704) — for multihomed customers the
  operator applies uRPF feasible-path mode or
  explicit ACL.
- SAVI (RFC 7039) — at IPv6 host-edge interfaces
  the operator enforces source-address validation
  against the host-binding table.

The MANRS programme provides the multi-operator
attestation framework; the operator's MANRS
participant identifier (where applicable) is recorded
in PHASE-1 §9.

## §6 RPKI / BGPsec Discipline

Per NIST SP 800-189 the operator's BGP-route hygiene
covers:

- ROA (Route Origin Authorisation) — every prefix
  the operator originates is covered by a ROA in
  the relevant Regional Internet Registry's
  repository (ARIN, RIPE NCC, APNIC, AFRINIC,
  LACNIC).
- ROV (Route Origin Validation) — the operator's
  ingress eBGP filters out RPKI-invalid
  announcements at the AS-edge.
- BGPsec rollout — the operator advances the BGPsec
  rollout per the NIST SP 800-189 phased-rollout
  guidance where the operator's BGP speakers
  support BGPsec.
- RFC 9234 BGP Roles + ASPA (when ratified) extend
  the discipline.

## §7 Post-Incident Analysis Discipline

Post-incident analysis follows NIST SP 800-61 Rev 3
+ ISO/IEC 27035-3:

- Detection and analysis — the operator
  reconstructs the attack signature, the source-ASN
  distribution, the mitigation effectiveness, and
  the legitimate-traffic false-positive rate.
- Containment, eradication, recovery — the operator
  documents each step taken and the elapsed time.
- Post-incident activity — lessons-learned review
  with the operator's incident-response team,
  the upstream mitigation provider, and the affected
  customer.
- The findings feed corrective-action items in
  PHASE-1 §8 with documented owners and due dates.

## §8 Threat-Intelligence Integration Discipline

The operator integrates with threat-intelligence
sources to enrich detection and attribution:

- MITRE ATT&CK Enterprise mappings — every
  detected attack is mapped to T1498 / T1499 sub-
  techniques where applicable.
- Sector ISAC feeds (FS-ISAC, H-ISAC, Auto-ISAC,
  K-ISAC) where the operator is a member.
- National CERT advisories (KrCERT-CC, US-CERT,
  JPCERT-CC, NCSC) — the operator subscribes and
  applies advisory-driven blocklists.
- Open-source feeds (Spamhaus DROP / EDROP, Team
  Cymru bogon list, the operator's own honeypot
  observations).

## §9 Supervisory and CERT Cooperation Discipline

The operator's cooperation discipline:

- National CERT — the operator notifies the national
  CERT (KrCERT-CC for KR; CISA for US; ENISA-
  coordinated CSIRTs Network for EU) when an
  incident exceeds the jurisdictional notification
  threshold.
- NIS2 Directive (Directive (EU) 2022/2555) — for
  EU-essential and important entities, significant
  incidents are reported under NIS2 Articles 23 to
  24 within the published timeframes (early
  warning within 24 hours; incident notification
  within 72 hours; final report within one month).
- KR 정보통신망법 + 전자금융감독규정 — for KR-
  jurisdiction operators the relevant incident-
  reporting timeline applies.
- Law-enforcement cooperation — the operator's
  legal function manages disclosure under MLAT or
  the operating jurisdiction's data-disclosure
  regime.

## §10 Identity, Time and Audit Discipline

NTPv4 stratum-2 or better is the operator's clock
baseline. Audit-events are emitted for every
detection, DOTS exchange, mitigation-action,
mitigation-release, BCP-attestation update, and
cooperation correspondence. Audit logs are integrity-
protected per the operator's tamper-evident
mechanism.

## §11 Operational-Resilience and Drill Discipline

The operator's drill discipline:

- Quarterly tabletop exercises against canonical
  attack scenarios.
- Annual full-stack drill exercising the DOTS
  exchange, the scrubbing-centre divert, the BGP
  FlowSpec rule, and the application-layer
  mitigation.
- Red-team / purple-team engagements where the
  operator's security team simulates a DDoS
  campaign to test detection and response.

## §12 Anycast and Geographic-Diversity Discipline

The operator's anycast deployment discipline:

- Anycast is exercised at the operator's edge — the
  same prefix is announced from geographically-
  diverse PoPs so that volumetric traffic is
  fragmented across ingress nodes.
- Per-PoP capacity ratios are documented; a PoP-
  level failure must not cascade.
- Anycast-divergence monitoring detects suboptimal
  routing and triggers operator action.

## §13 Bot-Pattern and Application-Surface Discipline

For operators with significant application-layer
exposure:

- Bot-fingerprint surveillance — TLS-fingerprint
  (JA3 / JA4 / JA4H), HTTP-header signature, and
  behaviour signature feed the classification.
- Account-takeover resistance — credential-stuffing
  surveillance is layered on top of the DDoS
  detection so that the operator's identity stack
  is not the single point of failure under
  application-layer attack.
- Web-application-firewall integration — OWASP
  ModSecurity Core Rule Set + the operator's custom
  rules feed the application-layer mitigation.

## §14 Conformance

Implementations claiming PHASE-3 conformance enforce
the discipline at every relevant decision point,
maintain the BCP 38 / BCP 84 + RPKI ROA + ROV
posture on the operator's network, exercise the DOTS
signal and data channels with at least one upstream
mitigation provider, satisfy the NIS2 / national-
CERT notification timelines (where applicable), and
exercise the drill discipline on the published
cadence.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-ddos-protection
- **Last Updated:** 2026-04-28
