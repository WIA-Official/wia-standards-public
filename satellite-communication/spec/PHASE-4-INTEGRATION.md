# WIA-SPACE-003 (satellite-communication) — Phase 4: Integration Specification

> **Version:** 1.0.0
> **Status:** Official
> **Phase:** 4 of 4 (Integration)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 4 covers the **integration** of WIA-SPACE-003 deployments with surrounding terrestrial networks, regulatory regimes, security infrastructure, and downstream tenant-facing applications. The scope includes:

1. Information-security management.
2. Identity, access, and operator-station hardening.
3. Regulatory and export-control compliance.
4. Terrestrial network integration (Internet, private MPLS, IoT clouds).
5. Tenant-facing service plans and billing.
6. Conformance and field commissioning.
7. Cross-discipline integration with adjacent WIA standards.

The objective is to give a single normative checklist that an operator can use to bring a WIA-SPACE-003 deployment into a real-world environment without ambiguity.

---

## 2. Information Security Management

### 2.1 ISO/IEC 27001 alignment

A WIA-SPACE-003 deployment MUST be operated within an ISO/IEC 27001:2022 information-security management system. The ISMS scope statement (Phase 1 §2.1 `iso27001Scope`) MUST cover:

- Personnel with operator, supervisor, administrator, or auditor roles.
- All gateways, ground stations, and storage media that hold tele-command keys, customer service plans, or telemetry.
- All network conduits between zones (per IEC 62443-3-2 risk assessment).

### 2.2 ISO/IEC 27002 controls

Operators MUST select and document at minimum the following ISO/IEC 27002:2022 controls:

| Area | Examples |
|------|----------|
| 5 — Organisational | A.5.1 policies; A.5.7 threat intelligence |
| 6 — People | A.6.3 awareness, training |
| 7 — Physical | A.7.4 monitoring |
| 8 — Technological | A.8.5 secure authentication; A.8.16 monitoring activities; A.8.24 cryptography use |

### 2.3 IEC 62443 alignment

For deployments that bind to operational technology (controllers, antenna positioners, ground-station automation), the IEC 62443-3-3 system security level (SL-T) for the bound zone MUST be ≥ SL-2. SL-3 is recommended for tele-command paths.

### 2.4 Risk assessment

The deployment descriptor MUST reference an IEC 62443-3-2 risk-assessment document under `iec62443.riskAssessmentRef`. Re-assessment cadence MUST not exceed 24 months and MUST follow significant changes (new band, new spacecraft, new tenant class).

---

## 3. Identity, Access, and Operator Hardening

### 3.1 Identity provisioning

- **Spacecraft, ground stations, user terminals**: X.509 v3 (RFC 5280) certificates from the deployment PKI.
- **Operators**: federated identity over OAuth 2.1 (RFC 9700) and OpenID Connect 1.0; subject claim resolves to a IEC 62443-2-1 personnel registry.

### 3.2 Multi-factor

Operator login MUST require multi-factor authentication. Conforming factors include:

- Possession factor: smart card / cryptographic token with X.509 client certificate (RFC 8446 mTLS).
- Inherence factor: biometric template held on a tamper-resistant token.
- Knowledge factor: PIN / passphrase per ISO/IEC 27002 A.5.17.

### 3.3 Privileged operations

Privileged operations (`space:tc-send` for safety-critical commands, `space:plan-administration` for tenant boundary changes) MUST require step-up authentication and MUST be logged with the second-factor evidence.

### 3.4 Operator-station hardening

Operator stations MUST:

- Run a maintained operating system at the vendor's current security update level.
- Disable inbound services other than the Phase-2 client.
- Apply IEC 62443-3-3 SR 1.5 (audit), SR 1.7 (multi-factor), SR 5.1 (zoning), SR 6 (audit log).

---

## 4. Regulatory and Export-Control Compliance

### 4.1 Radio regulations

Every link defined in Phase-1 MUST reference the operator's regulatory filing under `regulatoryFilings`. The filing reference is an opaque string controlled by the operator; WIA-SPACE-003 does not redistribute regulatory content.

### 4.2 Export controls

Where the deployment carries export-controlled technical data (cryptographic algorithm parameters, telemetry frame profiles, antenna patterns), the affected entities MUST be marked with `confidentialityClass=EXPORT-CONTROLLED` (Phase 1 §5). Operators MUST maintain an export-control register listing every controlled artefact.

### 4.3 Sanctions and embargoes

Tenant on-boarding MUST include a sanctions / embargo screen against the operator's compliance lists. Service plans for screened-failed tenants MUST be refused with the problem code `space/regulatory-filing-missing` if the failure is regulatory or with a vendor-specific code if the failure is screening-based.

---

## 5. Terrestrial Network Integration

### 5.1 Internet exit

Bundle traffic that exits to the Internet MUST be delivered through a hardened gateway that enforces:

- TLS 1.3 (RFC 8446) on every tenant-facing interface.
- Per-tenant verb scoping per Phase 2 §4.
- Egress filtering aligned with the operator's data-loss-prevention policy.

### 5.2 IoT cloud integration

User terminals classified as `IoT` MAY relay through an IoT cloud aggregator. The aggregator MUST:

- Use OSCORE (RFC 8613) end-to-end where the terminal supports it.
- Use COSE_Sign1 (RFC 9052) for tenant-side authenticity claims.
- Honour the per-tenant audit and retention policy.

### 5.3 Private MPLS / SD-WAN integration

For tenants with private MPLS or SD-WAN bindings, the gateway MUST:

- Maintain per-tenant routing tables.
- Apply per-tenant Quality-of-Service policies.
- Surface link health to the tenant's NMS through the discovery document.

### 5.4 Backhaul redundancy

Ground-stations with multiple backhaul paths MUST surface the path inventory through the deployment descriptor. The gateway MUST honour the tenant-configured failover policy.

---

## 6. Tenant-Facing Service Plans and Billing

### 6.1 Service-plan model

Tenant service-plan parameters (throughput, latency, availability, security level) live in Phase-1 *ServicePlan* descriptors. Plans are versioned by semver; major version bumps require tenant acceptance.

### 6.2 Quota enforcement

Quota enforcement uses the IETF `RateLimit-*` headers on the HTTP surface and CoAP option `MaxAge` semantics on the CoAP surface. Quota exhaustion returns HTTP 429 / CoAP 4.29 with a `Retry-After` header and the problem code `space/service-plan-quota-exceeded`.

### 6.3 Billing

Billing data exchange is out of scope for WIA-SPACE-003 v1. Operators integrating with billing systems MUST surface the billing data through a separate authenticated channel and MUST NOT mix billing identifiers with operational identifiers.

---

## 7. Field Commissioning Checklist

A WIA-SPACE-003 deployment MUST pass each of the following before being declared *operational*:

| # | Test | Reference |
|---|------|-----------|
| 1 | All spacecraft / ground stations / user terminals registered | Phase 1 |
| 2 | TLS 1.3 cipher inventory verified on all surfaces | RFC 8446 |
| 3 | OAuth 2.1 token introspection produces audit entries | RFC 9700, RFC 7662 |
| 4 | Multi-factor enforced for operator login | ISO/IEC 27002 A.8.5 |
| 5 | NTPv4 + NTS time-sync ≤ 50 ms vs. reference | RFC 5905, RFC 8915 |
| 6 | BPv7 conformance vectors (Phase 3 §8) pass | RFC 9171 |
| 7 | RFC 9173 default security context vectors pass | RFC 9173 |
| 8 | Tele-command replay test rejects replayed commands | Phase 3 §3.4 |
| 9 | ISO/IEC 27001 ISMS scope statement covers the deployment | ISO/IEC 27001 §4.3 |
| 10 | IEC 62443-3-3 SR 1, SR 2, SR 5, SR 6 implemented | IEC 62443-3-3 |
| 11 | Regulatory filing reference present per link | §4.1 |
| 12 | Export-control register up to date | §4.2 |
| 13 | Tenant on-boarding screen passes for all tenants | §4.3 |

The checklist results MUST be stored as a signed Phase-1 *Deployment* attribute and surfaced under `/deployments/{id}/health`.

---

## 8. Cyber-resilience and Incident Response

### 8.1 Detection

Continuous detection combines:

- IEC 62443-3-3 SR 6 audit-log analytics for anomalous operator behaviour.
- Spacecraft on-board attestation of firmware integrity (where supported).
- Time-source integrity monitoring (NTS-protected NTPv4 clients refusing unsigned servers).
- Anomalous tele-command pattern monitoring with operator-defined thresholds.

### 8.2 Containment

Containment playbooks MUST include:

- Disabling tele-command issuance from a privileged station.
- Revoking compromised operator credentials via the OAuth introspection endpoint with token replay refused per RFC 7662.
- Pausing bundle ingress at the gateway to preserve forensic integrity.

### 8.3 Recovery

Recovery procedures MUST:

- Re-run §7 commissioning checklist for affected entities.
- Re-attest spacecraft on-board firmware where the platform supports it.
- Document the incident in the ISO/IEC 27001 corrective-action register.

---

## 9. Cross-Discipline Integration

### 9.1 WIA-CITY-009 (smart-lighting) — outdoor IoT

Outdoor smart-lighting terminals that backhaul through satellite IoT links integrate via the WIA-SPACE-003 user-terminal surface. The lighting gateway holds a service-plan token and presents user-terminal identity at the WIA-SPACE-003 gateway.

### 9.2 WIA-CITY-014 (security-system-city) — remote sites

Remote security sites that lack terrestrial backhaul MAY backhaul through WIA-SPACE-003. Evidence bundles produced by the WIA-CITY-014 deployment are wrapped in BPv7 bundles for delivery to the central management plane.

### 9.3 WIA-CITY-017 (traffic-simulation) — observatory data

Traffic-simulation observatories that ingest live infrastructure data from remote regions MAY pull data through the WIA-SPACE-003 backhaul path.

### 9.4 Future cross-discipline integrations

The deployment descriptor reserves a `crossDiscipline` field for additional integrations. The field is an array of objects, each with `partnerStandard` (string) and `partnerScope` (string) fields, that MUST be honoured by the gateway when issuing tenant tokens.

---

## 10. Conformance Tags

A deployment descriptor MUST advertise a list of conformance tags that summarise the achieved profile:

| Tag | Meaning |
|-----|---------|
| `wia-space-003/v1/baseline` | Phase 3 §7.1 baseline profile |
| `wia-space-003/v1/constrained-terminal` | Phase 3 §7.2 constrained-terminal profile |
| `wia-space-003/v1/deep-space` | Phase 3 §7.3 deep-space profile |
| `wia-space-003/v1/iso27001` | ISMS-covered |
| `wia-space-003/v1/iec62443-sl2` | Security level 2 |
| `wia-space-003/v1/iec62443-sl3` | Security level 3 |
| `wia-space-003/v1/bpv7` | RFC 9171 BPv7 enabled |
| `wia-space-003/v1/bpv7-secure` | RFC 9173 default security context enabled |
| `wia-space-003/v1/iot-aggregator` | IoT cloud aggregator profile per §5.2 |

Conformance tags are normative for automated discovery and informative for human readers.

---

## 11. Operational Risk Considerations

### 11.1 Tele-command risk

Tele-commands directly affect spacecraft behaviour. Operators MUST:

- Use a privileged operator role for any safety-critical command.
- Maintain a fallback path that re-asserts the spacecraft's safe-mode behaviour.
- Restrict tele-command issuance to bounded geographic ground-segment domains.

### 11.2 Regulatory risk

Regulatory filings expire and change. Operators MUST track filing expiry through the `regulatoryFilings` reference and MUST refuse new links whose filing is missing or expired.

### 11.3 Export-control risk

Export-controlled data MUST NOT leave the operator's compliance domain except under documented authorisation. The `confidentialityClass=EXPORT-CONTROLLED` field on Phase-1 entities is the normative discriminator.

---

## 12. Long-term Archival

Archival of operational data (audit logs, telemetry, contact records) MUST follow the operator's retention policy. WIA-SPACE-003 reserves the following fields on every archive bundle:

- `archiveSchemaVersion` — semver
- `producedBy` — gateway identity
- `confidentialityClass` — see Phase 1 §5
- `digestAlgorithm` — SHA-256 (FIPS 180-4) by default
- `signature` — COSE_Sign1 over the bundle

Archive bundles MUST be verifiable by a third-party auditor without access to operator-side secrets.

---

## 13. References

1. IEC 62443-2-1; IEC 62443-3-2; IEC 62443-3-3.
2. ISO/IEC 7498-1:1994 — *OSI Basic Reference Model.*
3. ISO/IEC 18033-3:2010 — *Block ciphers.*
4. ISO 11770-2; ISO 11770-3 — *Key management.*
5. ISO/IEC 27001:2022; ISO/IEC 27002:2022; ISO/IEC 27037:2012; ISO/IEC 27701:2019.
6. ISO 6709:2008 — *Geographic point location.*
7. RFC 4838 — *DTN Architecture.*
8. RFC 5280 — *X.509 PKI Certificate and CRL Profile.*
9. RFC 5905; RFC 8915 — *NTPv4, NTS.*
10. RFC 7252; RFC 7641 — *CoAP, OBSERVE.*
11. RFC 7662 — *OAuth Token Introspection.*
12. RFC 8446; RFC 9147 — *TLS 1.3, DTLS 1.3.*
13. RFC 8613 — *OSCORE.*
14. RFC 8949 — *CBOR.*
15. RFC 9019; RFC 9124 — *SUIT manifests.*
16. RFC 9052; RFC 9053 — *COSE.*
17. RFC 9110; RFC 9457 — *HTTP semantics, problem details.*
18. RFC 9171; RFC 9173 — *BPv7 and default security context.*
19. RFC 9176 — *CoRE Resource Directory.*
20. RFC 9700 — *OAuth 2.1.*
21. FIPS 180-4 — *Secure Hash Standard.*
22. FIPS 197 — *Advanced Encryption Standard.*
