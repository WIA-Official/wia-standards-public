# WIA-TIME-011 (Historical Integrity) — Phase 4: Integration Specification

> **Version:** 1.0.0
> **Status:** Official
> **Phase:** 4 of 4 (Integration)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 4 covers the **integration** of WIA-TIME-011 deployments with surrounding archival, governance, regulatory, and information-security ecosystems. The scope includes:

1. Information-security management (ISO/IEC 27000-family).
2. Identity, access, and operator-station hardening.
3. Privacy and data-governance regimes.
4. Long-term archival institutions and government records administrations.
5. Public-sector evidence handling and digital forensics.
6. Conformance testing and field commissioning.
7. Cross-discipline integration with adjacent WIA standards.

The objective is to give a single normative checklist that an operator can use to bring a WIA-TIME-011 deployment into a real-world environment without ambiguity.

---

## 2. Information Security Management

### 2.1 ISO/IEC 27001 alignment

A WIA-TIME-011 deployment MUST be operated within an ISO/IEC 27001:2022 information-security management system. The ISMS scope statement (Phase 1 §2.1 `iso27001Scope`) MUST cover the TSA, the gateways, the transparency-log instances, and any storage media holding evidence records.

### 2.2 ISO/IEC 27002 controls

Operators MUST select and document at minimum the following ISO/IEC 27002:2022 controls:

| Area | Examples |
|------|----------|
| 5 — Organisational | A.5.1 policies; A.5.7 threat intelligence; A.5.32 intellectual property |
| 6 — People | A.6.1 screening; A.6.3 awareness, training |
| 7 — Physical | A.7.4 monitoring; A.7.5 environmental threats |
| 8 — Technological | A.8.5 secure authentication; A.8.16 monitoring activities; A.8.24 cryptography use; A.8.28 secure coding |

### 2.3 IEC 62443 alignment

Where a WIA-TIME-011 deployment is bound to operational technology (industrial archives, regulated SCADA records), the IEC 62443-3-3 system security level (SL-T) for the bound zone MUST be ≥ SL-2.

### 2.4 Risk assessment

The deployment descriptor MUST reference an IEC 62443-3-2 risk-assessment document. Re-assessment cadence MUST not exceed 24 months and MUST follow significant changes (new hash algorithm, new TSA, new transparency log).

---

## 3. Identity, Access, and Operator Hardening

### 3.1 Identity provisioning

- **TSA**: X.509 v3 certificate (RFC 5280) with the *id-kp-timeStamping* extended key usage.
- **Transparency log**: EC key with public-key fingerprint surfaced through the discovery document.
- **Operators**: federated identity over OAuth 2.1 (RFC 9700) and OpenID Connect 1.0; subject claim resolves to a IEC 62443-2-1 personnel registry.

### 3.2 Multi-factor

Operator login MUST require multi-factor authentication per ISO/IEC 27002 A.8.5.

### 3.3 Privileged operations

Privileged operations (`history:configure-log`, `history:author-verifier`, evidence-record renewal) MUST require step-up authentication and MUST be logged with the second-factor evidence.

### 3.4 Operator-station hardening

Operator stations MUST:

- Run a maintained operating system at the vendor's current security update level.
- Disable inbound services other than the Phase-2 client.
- Apply IEC 62443-3-3 SR 1.5 (audit), SR 1.7 (multi-factor), SR 5.1 (zoning), SR 6 (audit log).

---

## 4. Privacy and Data Governance

### 4.1 ISO/IEC 27701 PIM

Operators handling personal data through events MUST extend the ISMS to ISO/IEC 27701:2019 PIM.

### 4.2 Data classification

Each Phase-1 entity has an implicit data classification:

| Entity | Default classification |
|--------|------------------------|
| `Event` raw content | Personal-data-may-be-present |
| `Event.contentDigest` | Anonymous (digest only) |
| `TimeStampToken` | Operator-confidential |
| `EvidenceRecord` | Operator-confidential |
| `TransparencyLog` entries | Public-verifiable |

### 4.3 Subject access

Where personal data is referenced through a digest, subject-access requests MUST surface (a) the digest itself, (b) the existence of the bound TSA token, and (c) whether the original content is retained at the operator's archive. Subject-access endpoints align with the operator's PIM record.

### 4.4 Cross-border transfer

Sites operating across jurisdictions MUST declare lawful-basis and onward-transfer mechanisms as a machine-readable policy under `/scopes/{id}/.well-known/data-policy`.

---

## 5. Long-term Archival and Records Administration

### 5.1 OAIS reference

The deployment SHOULD align long-term archival operations with the OAIS reference model (informative reference outside the narrow ALLOW citation list); WIA-TIME-011's Phase-1 entities map cleanly onto OAIS information packages.

### 5.2 Archival information packages

Phase-1 EvidenceRecord objects function as the *Archival Information Package* (AIP) integrity layer. Archives MUST:

- Validate each AIP's evidence record on ingest.
- Schedule renewal per Phase 3 §3.3 ahead of any algorithm or certificate expiry.
- Surface AIP integrity status through the operator's archival catalog.

### 5.3 Government records

For government-records administration, the deployment MUST:

- Bind to a TSA whose policy OID is recognised by the regulator.
- Surface the regulator's evidence-handling standard reference (e.g. ISO/IEC 27037 §7) on every evidence record.
- Maintain an immutable audit log of every administrative action.

---

## 6. Public-sector Evidence Handling

### 6.1 ISO/IEC 27037 alignment

Evidence handling follows ISO/IEC 27037:2012. Each Phase-1 *Verifier* of kind `EVIDENCE-COLLECTOR` or `EXAMINER` MUST be bound to a documented role per ISO/IEC 27037 §7.

### 6.2 ISO/IEC 27042 analysis

Analysis and interpretation of digital evidence follow ISO/IEC 27042:2015. The analysis report MUST reference the Phase-1 entities used as inputs and MUST be signed with COSE_Sign1.

### 6.3 Chain of custody

The chain-of-custody record is appended on every evidence-handling action and is subject to the append-only invariant of the bound transparency log.

---

## 7. Field Commissioning Checklist

A WIA-TIME-011 deployment MUST pass each of the following before being declared *operational*:

| # | Test | Reference |
|---|------|-----------|
| 1 | TSA certificate has *id-kp-timeStamping* EKU | RFC 3161 §2.3 |
| 2 | TSA clock traceable per ISO/IEC 18014-4 | ISO/IEC 18014-4 |
| 3 | TLS 1.3 cipher inventory verified on all surfaces | RFC 8446 |
| 4 | OAuth 2.1 token introspection produces audit entries | RFC 9700, RFC 7662 |
| 5 | Multi-factor enforced for operator login | ISO/IEC 27002 A.8.5 |
| 6 | Transparency-log monitor alarms on inconsistency | RFC 9162 §6 |
| 7 | Hash-agile pipeline rejects deprecated algorithms | Phase 3 §6.3 |
| 8 | Renewal cadence test passes ahead of certificate expiry | Phase 3 §3.3 |
| 9 | ISO/IEC 27001 ISMS scope statement covers the deployment | ISO/IEC 27001 §4.3 |
| 10 | IEC 62443-3-3 SR 1, SR 2, SR 5, SR 6 implemented | IEC 62443-3-3 |
| 11 | Evidence-handling roles bound per ISO/IEC 27037 §7 | ISO/IEC 27037 |
| 12 | Privacy-by-design controls applied to event ingest | ISO/IEC 27701 |

The checklist results MUST be stored as a signed Phase-1 *Scope* attribute and surfaced under `/scopes/{id}/health`.

---

## 8. Cyber-resilience and Incident Response

### 8.1 Detection

Continuous detection combines:

- IEC 62443-3-3 SR 6 audit-log analytics.
- Transparency-log monitor protocol (Phase 3 §4.4).
- TSA certificate-status monitoring (OCSP RFC 6960, CRL RFC 5280).
- Hash-algorithm watch (NIST status changes).

### 8.2 Containment

Containment playbooks MUST include:

- Disabling new TSP requests at the gateway.
- Revoking compromised operator credentials via the OAuth introspection endpoint with token replay refused per RFC 7662.
- Pausing transparency-log submissions while the inconsistency is investigated.

### 8.3 Recovery

Recovery procedures MUST:

- Re-run §7 commissioning checklist for affected scopes.
- Re-issue certificates and re-run renewal where the incident affected the TSA.
- Document the incident in the ISO/IEC 27001 corrective-action register.

---

## 9. Cross-Discipline Integration

### 9.1 WIA-CITY-014 (security-system-city)

Evidence bundles produced by WIA-CITY-014 deployments MAY anchor their integrity through a WIA-TIME-011 scope. The WIA-CITY-014 evidence-bundle manifest references the WIA-TIME-011 token URI; verifiers retrieve the token through the Phase-2 surface.

### 9.2 WIA-AI-021 (vision-ai)

Inference results that contribute to consequential decisions MAY anchor their integrity through a WIA-TIME-011 scope. The vision-AI deployment manifest references the WIA-TIME-011 evidence-record URI.

### 9.3 WIA-CITY-017 (traffic-simulation)

Calibration and validation reports of traffic-simulation engines MAY anchor their integrity through a WIA-TIME-011 scope to provide a third-party-verifiable record of the calibration cycle.

### 9.4 Future integrations

The deployment descriptor reserves a `crossDiscipline` field for additional integrations.

---

## 10. Conformance Tags

A scope descriptor MUST advertise a list of conformance tags that summarise the achieved profile:

| Tag | Meaning |
|-----|---------|
| `wia-time-011/v1/baseline` | Phase 3 §7.1 |
| `wia-time-011/v1/transparency` | Phase 3 §7.2 |
| `wia-time-011/v1/long-term-archive` | Phase 3 §7.3 |
| `wia-time-011/v1/iso27001` | ISMS-covered |
| `wia-time-011/v1/iso27037` | Digital-evidence-handling-aligned |
| `wia-time-011/v1/iec62443-sl2` | Security level 2 |
| `wia-time-011/v1/iso18014-traced` | TSA traceable per ISO/IEC 18014-4 |

Conformance tags are normative for automated discovery and informative for human readers.

---

## 11. Operational Risk Considerations

### 11.1 Time-source compromise

A compromise of the TSA's time source invalidates every token issued under the affected window. The deployment MUST monitor time-source integrity with an independent traceability chain per ISO/IEC 18014-4.

### 11.2 Hash algorithm break

A break in a deployed hash algorithm requires emergency renewal of every evidence record using the algorithm. The deployment MUST publish a renewal SLA per scope.

### 11.3 Transparency-log compromise

A compromise of a transparency log breaks the append-only invariant and MUST trigger immediate disablement of new submissions and notification of bound scopes' verifiers.

### 11.4 Subject re-identification

Although Phase-1 events store only digests, paired access to the TSA and to the original content can support re-identification. The privacy controls of Phase 1 §5 are normative for any deployment processing personal data.

---

## 12. References

1. IEC 62443-2-1; IEC 62443-3-2; IEC 62443-3-3.
2. ISO/IEC 18014 (all parts) — *Time-stamping services.*
3. ISO/IEC 18033-3:2010 — *Block ciphers.*
4. ISO/IEC 27001:2022; ISO/IEC 27002:2022; ISO/IEC 27037:2012; ISO/IEC 27042:2015; ISO/IEC 27701:2019.
5. RFC 3161 — *TSP.*
6. RFC 4998 — *ERS.*
7. RFC 5280; RFC 6960 — *X.509, OCSP.*
8. RFC 5652; RFC 5816 — *CMS, ESSCertIDv2.*
9. RFC 5905; RFC 8915 — *NTPv4, NTS.*
10. RFC 7662 — *OAuth Token Introspection.*
11. RFC 8446 — *TLS 1.3.*
12. RFC 8949 — *CBOR.*
13. RFC 9052; RFC 9053 — *COSE.*
14. RFC 9110; RFC 9457 — *HTTP, problem details.*
15. RFC 9162 — *CT v2.*
16. RFC 9700 — *OAuth 2.1.*
17. FIPS 180-4; FIPS 197; FIPS 202 — *SHA, AES.*

---

## 13. Adoption Notes

### 13.1 Phased rollout

Operators bringing a WIA-TIME-011 deployment online for the first time should phase the adoption:

1. Stand up the TSA and verify the *id-kp-timeStamping* EKU and the ISO/IEC 18014-4 traceability chain.
2. Stand up the HTTP/REST surface (Phase 2 §2) without transparency-log binding.
3. Bind a transparency log per Phase 2 §4 once the operator has selected a log instance with a documented policy.
4. Enable evidence-record renewal once the operator's archival institution has approved the renewal cadence.
5. Run §7 commissioning checklist before serving production tenants.

Each phase MUST be gated by a documented change-management event in the ISO/IEC 27001 ISMS.

### 13.2 Migration from RFC 6962 v1 to RFC 9162 v2

Deployments that previously relied on a Certificate Transparency v1 log (RFC 6962) MAY migrate to RFC 9162 v2. Migration MUST:

- Run both logs in parallel for at least one full renewal cadence.
- Bind every evidence record to the v2 log going forward.
- Schedule a documented end-of-life for the v1 log.

### 13.3 Algorithm transition

When transitioning hash algorithms (e.g. SHA-256 → SHA-384), the deployment MUST:

- Submit every active evidence record to a renewal cycle that uses the new algorithm.
- Maintain dual-hash support during the transition window.
- Surface the transition status through the health endpoint.

### 13.4 Multi-jurisdiction operations

Deployments operating across multiple jurisdictions MUST hold a documented mapping of each region's lawful-basis regime to the system's data-handling levers. Changes to retention, lawful basis, or federated export MUST flow through the change-control process required by ISO/IEC 27001 §8.

### 13.5 Audit cadence

Operators MUST schedule:

- Quarterly internal audits of the audit log.
- Annual external audits aligned with the ISMS audit cycle.
- Ad-hoc audits in response to any incident affecting time-source integrity, certificate validity, or transparency-log consistency.
