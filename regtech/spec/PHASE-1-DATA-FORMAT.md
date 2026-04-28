# WIA-FIN-004 — Phase 1: Data Format

> RegTech canonical Phase 1: compliance-event + risk-score-and-ML + sanctions/PEP + Travel-Rule + audit-trail envelopes.

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


## 5. Data Formats

### 5.1 Compliance Event Schema

```json
{
  "$schema": "http://json-schema.org/draft/2020-12/schema",
  "type": "object",
  "properties": {
    "eventId": {
      "type": "string",
      "format": "uuid",
      "description": "Unique event identifier"
    },
    "timestamp": {
      "type": "string",
      "format": "date-time",
      "description": "ISO 8601 timestamp"
    },
    "eventType": {
      "type": "string",
      "enum": ["transaction", "customer_action", "system_event"]
    },
    "jurisdiction": {
      "type": "string",
      "pattern": "^[A-Z]{2}$",
      "description": "ISO 3166-1 alpha-2 country code"
    },
    "riskScore": {
      "type": "number",
      "minimum": 0,
      "maximum": 100
    },
    "complianceFlags": {
      "type": "object",
      "properties": {
        "requiresReview": {"type": "boolean"},
        "requiresSAR": {"type": "boolean"},
        "requiresCTR": {"type": "boolean"}
      }
    }
  },
  "required": ["eventId", "timestamp", "eventType", "jurisdiction", "riskScore"]
}
```

### 5.2 Customer Record Schema

```json
{
  "type": "object",
  "properties": {
    "customerId": {
      "type": "string",
      "description": "Unique customer identifier"
    },
    "personalInfo": {
      "type": "object",
      "properties": {
        "fullName": {"type": "string"},
        "dateOfBirth": {"type": "string", "format": "date"},
        "nationality": {"type": "string", "pattern": "^[A-Z]{2}$"}
      },
      "required": ["fullName"]
    },
    "verification": {
      "type": "object",
      "properties": {
        "kycStatus": {
          "type": "string",
          "enum": ["pending", "verified", "rejected", "expired"]
        },
        "verificationDate": {"type": "string", "format": "date-time"},
        "documentType": {
          "type": "string",
          "enum": ["passport", "drivers_license", "national_id"]
        },
        "documentNumber": {"type": "string"}
      }
    },
    "riskProfile": {
      "type": "object",
      "properties": {
        "riskLevel": {
          "type": "string",
          "enum": ["low", "medium", "high", "prohibited"]
        },
        "isPEP": {"type": "boolean"},
        "sanctionsMatch": {"type": "boolean"},
        "lastReviewDate": {"type": "string", "format": "date-time"}
      }
    }
  },
  "required": ["customerId", "personalInfo", "verification", "riskProfile"]
}
```

### 5.3 Transaction Record Schema

```json
{
  "type": "object",
  "properties": {
    "transactionId": {"type": "string"},
    "timestamp": {"type": "string", "format": "date-time"},
    "amount": {"type": "number", "minimum": 0},
    "currency": {"type": "string", "pattern": "^[A-Z]{3}$"},
    "customerId": {"type": "string"},
    "type": {
      "type": "string",
      "enum": ["deposit", "withdrawal", "transfer", "payment"]
    },
    "jurisdiction": {"type": "string", "pattern": "^[A-Z]{2}$"},
    "metadata": {"type": "object"}
  },
  "required": ["transactionId", "timestamp", "amount", "currency", "customerId", "type"]
}
```

### 5.4 Regulatory Report Schema

```json
{
  "type": "object",
  "properties": {
    "reportId": {"type": "string"},
    "reportType": {
      "type": "string",
      "enum": ["sar", "ctr", "eft", "atr"]
    },
    "jurisdiction": {"type": "string", "pattern": "^[A-Z]{2}$"},
    "filingDate": {"type": "string", "format": "date-time"},
    "data": {"type": "object"},
    "narrative": {"type": "string"},
    "submissionStatus": {
      "type": "string",
      "enum": ["draft", "pending", "submitted", "accepted", "rejected"]
    },
    "confirmationNumber": {"type": "string"}
  },
  "required": ["reportId", "reportType", "jurisdiction", "data"]
}
```

---




---

## A.1 Compliance-event envelope

The Phase 1 envelope groups compliance events by category (KYC — customer onboarding identity verification; KYB — business onboarding; AML transaction-monitoring alert; sanctions screening hit; PEP screening hit; suspicious-activity report — SAR / STR; politically-exposed-person periodic review; trade-surveillance alert; market-abuse alert; Best-Execution outlier; OFAC/EU/UN/UK/KR sanctions list update; Travel Rule data exchange per FATF Recommendation 16) with the canonical fields: event identifier (UUID v7), event timestamp at nanosecond precision, originating-system identifier, jurisdiction code per ISO 3166-1, regulator-of-record reference, related entities (customer, account, transaction, counterparty), and the chain-of-decisions envelope.

## A.2 Risk-score and ML descriptor

Risk-score descriptors carry: scoring model identifier, model version, feature-set envelope (which inputs the model consumes), output-score range and meaning, calibration-quality envelope, model-explainability envelope (SHAP-style attributions for regulatory review), the false-positive / false-negative envelope from the operator's validation set, and the model-drift-monitoring envelope. ML-model-deployment descriptors additionally carry the model-lineage envelope, the training-data-of-record envelope (with the appropriate data-protection envelope), the bias-and-fairness envelope per the operator's responsible-AI policy, and the model-card-of-record per the IEEE 7000-style transparency standard.

## A.3 Sanctions-and-PEP-screening envelope

Sanctions-and-PEP screening envelopes carry: list-source-of-record (UN Security Council Consolidated; EU sanctions list; OFAC SDN; UK OFSI; KR KOFIU; JP MOF; WB Listings of Ineligible Firms; commercial PEP-list providers like Dow Jones / Refinitiv / LexisNexis Bridger), per-list version timestamp, screening-engine identifier, per-screen match-score envelope, fuzzy-match-policy envelope (algorithm — Jaro-Winkler / Levenshtein / phonetic + threshold), false-positive / false-negative envelope, and the per-hit triage-decision envelope (clear / refer / freeze / block).

## A.4 Travel-rule envelope

Travel-rule envelopes per FATF Recommendation 16 carry: originator entity (name, address, account-of-record), beneficiary entity (name, address, account-of-record), the value transferred (amount + ISO 4217 currency), the originating- and beneficiary-VASP identifiers (or institution identifiers per the per-jurisdiction equivalent), the per-jurisdiction threshold envelope (FATF default $1000-3000 USD-equivalent; per-jurisdiction lower thresholds in some markets), and the secure-data-exchange envelope (IVMS 101 + the per-VASP encryption-and-signing envelope).

## A.5 Audit-trail envelope

Audit-trail envelopes carry: per-event-of-record signature chain anchored into a Merkle tree per-tenant per-day, the immutable-event-store-of-record envelope (per-day Merkle-root commitments to the operator's chosen blockchain or to a public certificate-transparency-style log per RFC 9162), the regulator-access envelope (which regulator can read which fields under which legal basis), the data-residency envelope (which jurisdiction holds the canonical store), and the retention envelope per the per-jurisdiction record-keeping rules (typically 5-7 years for AML data; longer for sanctions-investigation records).

## A.6 Customer-and-counterparty descriptor

Customer-and-counterparty descriptors carry: customer identifier (UUID v7), legal-entity identifier per ISO 17442 LEI where applicable, beneficial-ownership chain to natural-person owners per FATF Recommendation 24, jurisdiction-of-incorporation per ISO 3166-1, business-class (per the GICS / NACE / KSIC industry classification), the per-jurisdiction risk-rating envelope (low / medium / high / prohibited), the customer-due-diligence (CDD) tier (simplified / standard / enhanced), the periodic-review cadence (annual for low-risk; semi-annual for medium-risk; quarterly for high-risk; event-driven for prohibited transitions), and the per-relationship product/service catalogue.

## A.7 Suspicious-activity-report descriptor

SAR / STR descriptors follow the per-jurisdiction filing template (FinCEN BSA E-Filing per the FinCEN BSA SAR XML schema; goAML-XML for FATF-aligned FIUs; KOFIU per the Korean Specific Financial Information Act; FIU-IND per India PMLA; per-jurisdiction equivalents) with the canonical fields: filing-institution identifier, filing-date, narrative summary, suspect (subject) details, related transactions and their dates / amounts / counterparties, related accounts, the suspicious-activity-type catalogue per the per-jurisdiction codes, the dollar-amount summary (or per-jurisdiction currency equivalent), the prior-SAR cross-reference where applicable, and the supporting-document attachment manifest.

## A.8 Cross-border-payment envelope

Cross-border-payment envelopes capture the ISO 20022 message type (pacs.008 / pacs.009 / pacs.004 / pain.001 / pain.013 / camt.052 / camt.053 / camt.054 — the relevant flows for credit transfer, financial-institution credit transfer, payment return, customer credit-transfer initiation, customer payment-status report, account reporting), the per-message originator and beneficiary BIC plus IBAN where applicable, the per-message structured-remittance-information field, the FATF Recommendation 16 Travel Rule compliance envelope per Phase 1 §A.4, and the per-jurisdiction reporting threshold envelope (CTR threshold per BSA at $10,000; per-jurisdiction equivalents in the operator's compliance matrix).


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
