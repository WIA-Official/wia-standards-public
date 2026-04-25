# WIA-TIME-014 (Data Time Transport) — Phase 4: Integration Specification

> **Version:** 1.0.0
> **Status:** Official
> **Phase:** 4 of 4 (Integration)
> **Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 4 covers the **integration** of WIA-TIME-014 deployments with surrounding archival, governance, regulatory, storage-security, and information-security ecosystems. The scope includes:

1. Information-security management (ISO/IEC 27000-family).
2. Storage security per ISO/IEC 27040.
3. Identity, access, and operator-station hardening.
4. Privacy and data-governance regimes.
5. Long-term archival institutions and government records administrations.
6. Conformance testing and field commissioning.
7. Cross-discipline integration with adjacent WIA standards.

The objective is to give a single normative checklist that an operator can use to bring a WIA-TIME-014 deployment into a real-world environment without ambiguity.

---

## 2. Information Security Management

### 2.1 ISO/IEC 27001 alignment

A WIA-TIME-014 deployment MUST be operated within an ISO/IEC 27001:2022 information-security management system. The ISMS scope statement (Phase 1 §2.1 `iso27001Scope`) MUST cover the gateway, the bound TSA, all storage volumes, and any operator station accessing the deployment.

### 2.2 ISO/IEC 27002 controls

Operators MUST select and document at minimum the following ISO/IEC 27002:2022 controls:

| Area | Examples |
|------|----------|
| 5 — Organisational | A.5.1 policies; A.5.7 threat intelligence |
| 6 — People | A.6.1 screening; A.6.3 awareness |
| 7 — Physical | A.7.4 monitoring; A.7.5 environmental threats |
| 8 — Technological | A.8.5 secure authentication; A.8.10 information deletion; A.8.16 monitoring activities; A.8.24 cryptography use |

### 2.3 IEC 62443 alignment

Where a WIA-TIME-014 deployment is bound to operational technology, the IEC 62443-3-3 system security level (SL-T) for the bound zone MUST be ≥ SL-2.

### 2.4 Risk assessment

The deployment descriptor MUST reference an IEC 62443-3-2 risk-assessment document. Re-assessment cadence MUST not exceed 24 months and MUST follow significant changes (new tier, new TSA, new jurisdiction).

---

## 3. Storage Security (ISO/IEC 27040)

### 3.1 Storage scope

The deployment's `iso27040StorageScope` (Phase 1 §2.1) MUST cover every storage volume operated by the deployment. The operator MUST maintain an inventory of media types, manufacturer references, encryption modes, and sanitisation procedures.

### 3.2 Sanitisation

Physical sanitisation follows ISO/IEC 27040:2024 §6.7. The deployment MUST track each sanitisation event and MUST refuse re-use of media for non-equivalent classification levels until sanitisation is documented as complete.

### 3.3 Cryptographic erasure

Cryptographic erasure relies on the operator's key-management procedure (ISO 11770-2 / ISO 11770-3). The deployment MUST refuse retrieval of any deposit whose key has been erased and MUST surface the erasure as a deposit attribute.

### 3.4 Encryption at rest

Every storage volume MUST encrypt data at rest with AES-256-GCM (preferred) or AES-128-GCM. Key rotation cadence MUST be at most 12 months for symmetric volume keys; asymmetric encryption-at-rest schemes follow the ISO 11770-3 cadence.

---

## 4. Identity, Access, and Operator Hardening

### 4.1 Identity provisioning

- **Depositors and auditors**: federated identity over OAuth 2.1 (RFC 9700) and OpenID Connect 1.0.
- **Storage volumes**: X.509 v3 (RFC 5280) certificates from the deployment PKI.
- **TSA**: external X.509 v3 certificate with the *id-kp-timeStamping* EKU.

### 4.2 Multi-factor

Operator login MUST require multi-factor authentication per ISO/IEC 27002 A.8.5.

### 4.3 Privileged operations

Privileged operations (`data-time:revoke-capsule`, retention-policy edits, tier transitions) MUST require step-up authentication and MUST be logged with second-factor evidence.

### 4.4 Operator-station hardening

Operator stations MUST run a maintained operating system at the vendor's current security update level, disable inbound services other than the Phase-2 client, and apply IEC 62443-3-3 SR 1.5, SR 1.7, SR 5.1, SR 6.

---

## 5. Privacy and Data Governance

### 5.1 ISO/IEC 27701 PIM

Operators handling personal data MUST extend the ISMS to ISO/IEC 27701:2019 PIM.

### 5.2 Data classification

Each Phase-1 entity has an implicit data classification:

| Entity | Default classification |
|--------|------------------------|
| `Deposit.contentDigest` | Anonymous (digest only) |
| Deposit raw content | Personal-data-may-be-present |
| `Capsule.releasePolicy` | Operator-confidential |
| `EvidenceRecord` | Operator-confidential |
| `StorageVolume.physicalLocations` | Operator-confidential |

### 5.3 Subject access

Subject-access endpoints align with the operator's PIM record. Subject-access requests for capsuled content MUST be evaluated against the capsule's release policy; pre-release subject access MUST be refused unless the policy explicitly permits it.

### 5.4 Cross-border transfer

Sites operating across jurisdictions MUST declare lawful-basis and onward-transfer mechanisms as a machine-readable policy under `/domains/{id}/.well-known/data-policy`.

---

## 6. Long-term Archival and Records Administration

### 6.1 OAIS reference

The deployment SHOULD align long-term archival operations with the OAIS reference model (informative reference outside the narrow ALLOW citation list); WIA-TIME-014's Phase-1 entities map cleanly onto OAIS information packages.

### 6.2 Archival information packages

Phase-1 deposits and evidence records function together as the *Archival Information Package* (AIP) integrity layer. Archives MUST validate every AIP's evidence record on ingest and SHOULD schedule renewal ahead of any algorithm or certificate expiry.

### 6.3 Government records

For government-records administration, the deployment MUST bind to a TSA whose policy OID is recognised by the regulator and surface the regulator's evidence-handling standard reference (e.g. ISO/IEC 27037 §7) on every evidence record.

---

## 7. Field Commissioning Checklist

A WIA-TIME-014 deployment MUST pass each of the following before being declared *operational*:

| # | Test | Reference |
|---|------|-----------|
| 1 | TSA certificate has *id-kp-timeStamping* EKU | RFC 3161 §2.3 |
| 2 | TSA clock traceable per ISO/IEC 18014-4 | ISO/IEC 18014-4 |
| 3 | TLS 1.3 cipher inventory verified on all surfaces | RFC 8446 |
| 4 | OAuth 2.1 token introspection produces audit entries | RFC 9700, RFC 7662 |
| 5 | Multi-factor enforced for operator login | ISO/IEC 27002 A.8.5 |
| 6 | Capsule workflow tests pass (Phase 3 §3) | Phase 3 §8 |
| 7 | Tier-transition tests pass (Phase 3 §4) | Phase 3 §8 |
| 8 | ISO/IEC 27040 sanitisation procedure documented | ISO/IEC 27040 §6.7 |
| 9 | Cryptographic erasure procedure documented | ISO 11770 |
| 10 | ISO/IEC 27001 ISMS scope statement covers the deployment | ISO/IEC 27001 §4.3 |
| 11 | IEC 62443-3-3 SR 1, SR 2, SR 5, SR 6 implemented | IEC 62443-3-3 |
| 12 | ISO/IEC 27701 PIM extends ISMS for personal data | ISO/IEC 27701 |

The checklist results MUST be stored as a signed Phase-1 *Domain* attribute and surfaced under `/domains/{id}/health`.

---

## 8. Cyber-resilience and Incident Response

### 8.1 Detection

Continuous detection combines:

- IEC 62443-3-3 SR 6 audit-log analytics.
- Storage-volume health monitoring (capacity, latency, integrity).
- TSA certificate-status monitoring (OCSP RFC 6960, CRL RFC 5280).
- Hash-algorithm watch (NIST status changes).

### 8.2 Containment

Containment playbooks MUST include:

- Disabling new deposits at the gateway.
- Pausing tier transitions while the incident scope is bounded.
- Revoking compromised operator credentials via the OAuth introspection endpoint with token replay refused per RFC 7662.

### 8.3 Recovery

Recovery procedures MUST re-run §7 commissioning checklist for affected domains, re-issue certificates and re-run renewal where the incident affected the TSA, and document the incident in the ISO/IEC 27001 corrective-action register.

---

## 9. Cross-Discipline Integration

### 9.1 WIA-TIME-011 (Historical Integrity)

Every deposit's TimeStampToken MAY be cross-anchored in a WIA-TIME-011 transparency log to provide a public-verifiable record of the deposit's existence and time.

### 9.2 WIA-CITY-014 (security-system-city)

Long-term retention of evidence bundles produced by WIA-CITY-014 deployments MAY use WIA-TIME-014 capsules to enforce regulatory retention horizons.

### 9.3 WIA-AI-021 (vision-ai)

Inference results that contribute to consequential decisions MAY be deposited into a WIA-TIME-014 domain so that the result remains verifiable across model retirements.

### 9.4 WIA-CITY-017 (traffic-simulation)

Calibration and validation reports of traffic-simulation engines MAY be deposited into a WIA-TIME-014 domain to provide a third-party-verifiable record of the calibration cycle.

---

## 10. Conformance Tags

A domain descriptor MUST advertise a list of conformance tags that summarise the achieved profile:

| Tag | Meaning |
|-----|---------|
| `wia-time-014/v1/baseline` | Phase 3 §7.1 |
| `wia-time-014/v1/constrained-depositor` | Phase 3 §7.2 |
| `wia-time-014/v1/long-term-archive` | Phase 3 §7.3 |
| `wia-time-014/v1/iso27001` | ISMS-covered |
| `wia-time-014/v1/iso27040` | Storage-security covered |
| `wia-time-014/v1/iso27701` | PIM-covered |
| `wia-time-014/v1/iec62443-sl2` | Security level 2 |
| `wia-time-014/v1/iec62443-sl3` | Security level 3 |

Conformance tags are normative for automated discovery and informative for human readers.

---

## 11. Operational Risk Considerations

### 11.1 Time-source compromise

A compromise of the TSA's time source invalidates every token issued under the affected window. The deployment MUST monitor time-source integrity with an independent traceability chain per ISO/IEC 18014-4.

### 11.2 Algorithm break

A break in a deployed hash algorithm requires emergency renewal of every evidence record using the algorithm. The deployment MUST publish a renewal SLA per domain.

### 11.3 Storage-media failure

Storage-media failures MUST trigger automatic re-placement on redundant volumes when configured, and MUST surface the failure as a Phase-1 *Auditor*-visible event regardless of replacement success.

### 11.4 Premature release

A premature release (release before the policy condition is satisfied) MUST be impossible by design; gateways MUST refuse the release operation when any condition is unmet. A premature release detected post hoc is a critical incident requiring immediate escalation.

### 11.5 Late release

A late release (release after the latest-release-at horizon) is a process incident; the gateway MUST surface late releases as audit-log warnings and the operator's incident-response procedure governs the next steps.

---

## 12. Adoption Notes

### 12.1 Phased rollout

Phase the adoption:

1. Stand up the gateway and the bound TSA.
2. Configure storage volumes and ISO/IEC 27040 controls.
3. Onboard the first set of depositors with `data-time:deposit` only.
4. Enable capsule workflows for a pilot release policy.
5. Enable tier transitions and renewal cadence.
6. Run §7 commissioning checklist before opening to production tenants.

### 12.2 Federation

Multi-domain federation uses a federation document at `/.well-known/wia-data-time-federation`. The document lists partner domain URIs, supported export formats, and the federation key (COSE_Sign1 verifying key).

### 12.3 Migration from prior archival systems

When migrating from a non-WIA archival system, the deployment MUST:

- Compute a fresh content digest for every migrated artefact.
- Issue a new RFC 3161 token at the time of migration; the migration time becomes the artefact's bound time anchor in the WIA-TIME-014 deployment.
- Preserve any prior provenance records as informative attachments.

---

## 13. References

1. IEC 62443-2-1; IEC 62443-3-2; IEC 62443-3-3.
2. ISO/IEC 18014 (all parts); ISO/IEC 18033-3:2010.
3. ISO/IEC 27001:2022; ISO/IEC 27002:2022; ISO/IEC 27037:2012; ISO/IEC 27040:2024; ISO/IEC 27042:2015; ISO/IEC 27701:2019.
4. ISO 11770-2; ISO 11770-3.
5. RFC 3161; RFC 4998; RFC 5280; RFC 5816; RFC 6960; RFC 7662; RFC 8446; RFC 8915; RFC 8949; RFC 9052; RFC 9053; RFC 9110; RFC 9147; RFC 9162; RFC 9176; RFC 9457; RFC 9700.
6. FIPS 180-4; FIPS 197; FIPS 202.
