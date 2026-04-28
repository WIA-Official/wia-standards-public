# WIA-FIN-004 — Phase 3: Protocol

> RegTech canonical Phase 3: protocols (KYC/KYB + AML monitoring + sanctions screening + trade surveillance + real-time monitoring).

# WIA-FIN-004: RegTech Standard Specification v1.0

**Version:** 1.0.0
**Status:** Final
**Date:** January 2025
**Authors:** WIA Standards Committee
**Organization:** World Certification Industry Association (WIA)

---

## Abstract

This specification defines the WIA-FIN-004 RegTech Standard for regulatory technology and compliance automation in financial services. It provides comprehensive data formats, API specifications, security protocols, and integration guidelines for implementing automated compliance, AML/KYC verification, and real-time monitoring systems.

**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Scope](#2-scope)
3. [Normative References](#3-normative-references)
4. [Terms and Definitions](#4-terms-and-definitions)
5. [Data Formats](#5-data-formats)
6. [API Specifications](#6-api-specifications)
7. [Security Requirements](#7-security-requirements)
8. [Integration Guidelines](#8-integration-guidelines)
9. [Compliance](#9-compliance)
10. [Examples](#10-examples)

---


## 7. Security Requirements

### 7.1 Transport Security

- **MUST** use TLS 1.3 or higher for all communications
- **MUST** validate SSL/TLS certificates
- **SHOULD** implement certificate pinning

### 7.2 Data Encryption

- **MUST** encrypt sensitive data at rest using AES-256
- **MUST** encrypt PII (Personally Identifiable Information) end-to-end
- **SHOULD** implement field-level encryption for high-sensitivity data

### 7.3 Access Control

- **MUST** implement role-based access control (RBAC)
- **MUST** enforce principle of least privilege
- **SHOULD** implement multi-factor authentication (MFA)

### 7.4 Audit Logging

- **MUST** log all compliance-related activities
- **MUST** include timestamp, user ID, action, and result in logs
- **MUST** protect audit logs from tampering (immutability)
- **SHOULD** retain logs for minimum 7 years

---



## 9. Compliance

### 9.1 Regulatory Compliance

This standard supports compliance with:

- **US:** FinCEN, SEC, CFTC regulations
- **EU:** GDPR, MiFID II, AMLD5
- **UK:** FCA, PRA regulations
- **APAC:** MAS, HKMA, JFSA regulations

### 9.2 Data Privacy

Implementations **MUST** comply with:

- GDPR (General Data Protection Regulation)
- CCPA (California Consumer Privacy Act)
- Other applicable data privacy laws

### 9.3 Standards Compliance

This standard is compatible with:

- ISO 27001 (Information Security)
- SOC 2 Type II (Service Organization Controls)
- PCI DSS (Payment Card Industry Data Security Standard)

---




---

## A.1 KYC / KYB onboarding protocol

KYC / KYB onboarding protocol covers the per-jurisdiction identity-verification envelope (US BSA Customer Identification Program per 31 CFR 1020.220; EU 6th AML Directive 2018/1673; KR Specific Financial Information Act + Real-Name Confirmation; JP APG; UK MLR 2017), the document-verification envelope (passport, national-ID, driver's licence per ICAO Doc 9303 + the per-jurisdiction equivalent), the liveness-detection envelope (NIST FRVT-tested liveness detection per ISO/IEC 30107-3), and the beneficial-ownership envelope per FATF Recommendation 24 (UBO chain to natural-person owners). The protocol exposes the per-customer risk-rating outcome that gates ongoing-due-diligence cadence.

## A.2 AML transaction-monitoring protocol

Transaction-monitoring protocol covers the rule-engine-of-record envelope (canonical AML scenarios — structuring, smurfing, round-amount cash, rapid-pass-through, geographic-anomaly, dormant-account-reactivation, peer-group-anomaly), the ML-anomaly-detection envelope (semi-supervised + supervised models with the per-model precision-recall trade-off documented), the alert-triage envelope (SLA from alert-creation to first-response), the SAR / STR drafting envelope (per-jurisdiction template + auto-fill from event data), and the regulator-filing envelope (FinCEN / FIU-NL / KOFIU / FIU-IND / equivalents) with the per-jurisdiction filing format (FinCEN BSA E-Filing / goAML XML schema / per-FIU equivalents).

## A.3 Sanctions-screening protocol

Sanctions-screening protocol covers the per-list refresh cadence (OFAC SDN re-pull within 24 hours of the daily issuance; UN Consolidated within 24 hours; EU sanctions within 24 hours; UK OFSI within 24 hours; per-jurisdiction equivalents within the documented SLA), the matching-algorithm envelope (Jaro-Winkler / Levenshtein / phonetic / Soundex / Cologne / Metaphone with the per-list-tuned threshold), the per-match triage envelope, and the reporting-to-regulator envelope (US OFAC via the Reporting and Compliance Office; EU competent authority; KR FSC equivalents). The protocol bans batch-only screening: the operator screens at every customer-onboarding, every transaction-initiation, and on every sanctions-list update.

## A.4 Trade-surveillance protocol

Trade-surveillance protocol covers the market-abuse scenarios per EU MAR (insider dealing, market manipulation including spoofing / layering / quote-stuffing / wash-trading), the US Reg-NMS / Reg-SHO market-conduct framework, the FINRA OATS reporting framework, the cross-market-of-record envelope (which trading venues the surveillance covers), the alert-aggregation envelope (deduplicating cross-strategy alerts on the same trader / instrument / window), and the SAR / SUSTR drafting envelope per-jurisdiction.

## A.5 Real-time-monitoring protocol

Real-time-monitoring protocol covers the streaming-event ingestion envelope (Apache Kafka / AWS Kinesis / Pulsar with the operator's documented retention), the stream-processing envelope (Flink / Spark Streaming / Kafka Streams with the per-job state-of-record envelope), the per-event latency target (typical p50 less than 100 ms, p99 less than 1000 ms for alerting; per-event cost-of-record envelope for billing), the event-replay envelope (reprocess from the documented offset on rule update), and the back-pressure envelope.

## A.6 Replay and integrity defence

Standard 96-bit nonce + 300-second skew window + 600-second seen-nonce cache for the management-plane control traffic. Per-event audit-trail records are signed at registration time and the signature chain is anchored into the per-tenant Merkle tree; chain breaks invalidate the audit envelope and trigger a forensic-review event. Travel-rule envelopes are signed per IETF JOSE (RFC 7515 JWS / RFC 7516 JWE) with the per-VASP-pair recipient-encryption envelope.

## A.7 Beneficial-ownership and UBO-resolution protocol

Beneficial-ownership protocol per FATF Recommendation 24 / 25 covers the UBO chain construction (corporate-ownership graph traversal from the customer entity to natural-person owners with the per-jurisdiction percentage threshold — typically 25% per FATF; lower in some jurisdictions like Australia 25% / UK 25% / KR 25%), the public-registry envelope (UK Companies House People with Significant Control register; EU national registers per 5th AML Directive 2018/843; KR Beneficial Ownership Register from KFIU), the per-link verification envelope (registry-based / customer-attested / third-party-verified), the conflict-resolution envelope when multiple sources disagree, and the periodic-revalidation cadence per the customer's risk tier.

## A.8 Trade-based-money-laundering protocol

TBML (Trade-Based Money Laundering) protocol covers the trade-finance scenarios per FATF TBML guidance (over-invoicing, under-invoicing, multiple invoicing, phantom shipments, ghost trades, mis-classification of goods, falsification of documents), the documentary-credit envelope (Letter of Credit / Standby LC / Documentary Collection per UCP 600 / ISP98 / URC 522), the Bill of Lading envelope, the per-shipment screening envelope (consignor / consignee / vessel / port-of-loading / port-of-discharge / goods description against sanctions and dual-use-goods controls), and the trade-finance-portal integration envelope.

## A.9 Periodic-due-diligence and refresh protocol

Periodic-due-diligence protocol covers the per-customer revalidation cadence (annual for low-risk; semi-annual for medium-risk; quarterly for high-risk; event-driven on KYC trigger events — sanctions hit, PEP-status change, jurisdiction-rating change, transactional-anomaly), the document-of-record refresh envelope (customer ID re-verification on configurable cadence; UBO refresh per Phase 3 §A.7), the customer-survey envelope where the operator's policy requires it, the EDD escalation envelope where adverse media or news-feed signals triggers a deeper review, and the result-of-record envelope (no-change / risk-rating-changed / restrictions-applied / relationship-terminated).


---

## Z.1 Glossary, conformance, governance

A companion glossary at `https://wiastandards.com/regtech/glossary/`
expands every term used throughout this Phase. The conformance
test suite at `https://github.com/WIA-Official/wia-regtech-conformance`
walks every endpoint and protocol exchange. The reference container
at `wia/regtech-host:1.0.0` ships every regtech envelope and
endpoint with mock data so integrators can exercise the contract
before wiring real backends. The companion CLI at
`cli/regtech.sh` ships sample envelope generators with no
dependencies beyond `jq` and POSIX shell.

## Z.2 Cross-standard composition (recap)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 §5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations.

## Z.3 Implementation runbook

A first implementation typically follows: stand up the reference
container; run the conformance suite against it end-to-end;
replace the mock backend with a real backend one endpoint at a
time; wire audit-log replication out to the operations sink;
onboard a single trusted peer for federation; expand to multiple
peers; promote to production with the warning-envelope subscription
enabled and the runbook in §Z.5 followed.

## Z.4 Backwards-compatibility promise

Within the 1.x line, every Phase 1 envelope shape, every Phase 2
endpoint, and every Phase 3 protocol exchange MUST remain reachable
and MUST continue to honour the documented status codes and content
shapes. Hosts MAY add optional fields and new envelopes; hosts MUST
NOT remove existing ones. Breaking changes ride a major version
bump with a 12-month deprecation window per IETF RFC 8594 and 9745
and require a two-thirds Committee vote.

## Z.5 Closing implementer note

This Phase fits inside the WIA Standards four-Phase architecture:
Phase 1 envelopes are the wire-format contract; Phase 2 surfaces
them through HTTPS; Phase 3 wraps them in protocol exchanges that
cross trust boundaries; Phase 4 integrates with the broader
ecosystem. Regtech deployments that follow this layering
inherit the per-Phase conformance gates and the cross-standard
composition behaviour described in Z.2.

弘益人間 — Benefit All Humanity.
