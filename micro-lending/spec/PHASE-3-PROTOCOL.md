# WIA-micro-lending PHASE 3 — Protocol Specification

**Standard:** WIA-micro-lending
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE defines the operational protocols binding
records and API resources into auditable lifecycles:
customer onboarding (KYC / CDD), product / disclosure
lifecycle, application-decisioning workflow, group-
loan operating rhythm, disbursement and repayment
flows, restructuring decision discipline, write-off
governance, regulatory reporting cadence, customer-
protection enforcement, and the audit-event chain. The
protocols are designed so a central-bank examiner, an
FIU compliance officer, an IFC investor's E&S audit,
or an external SPI4 social-performance audit can
reconstruct any account from the event log.

References (CITATION-POLICY ALLOW only):
- BIS — Basel Core Principles for Effective Banking Supervision
- BIS — Microfinance activities and the Core Principles
- FATF Recommendations (most recent revision)
- FATF Recommendation 16 — wire transfers
- ITU-T X.1216 — security framework for digital financial services
- IASB IFRS 9 — Financial Instruments (impairment)
- US Reg E (12 CFR 1005), Reg Z (12 CFR 1026)
- EU PSD2 (2015/2366); EU PSD3 (current)
- SPI4 (CERISE+SPTF) — Universal Standards
- Client Protection Pathway (CPP) — customer protection principles
- ISO/IEC 27037 — digital evidence preservation
- IETF RFC 5424 (Syslog), RFC 7515 (JWS), RFC 8785 (JCS)

---

## §1 Customer onboarding (KYC / CDD)

```
draft → identity-collected → screened → verified → enrolled → active
                                  │
                                  └→ rejected → re-attempt | refused
```

CDD per FATF Recommendation 10 captures: name, date of
birth, residential address, nationality, occupation,
identity-document evidence, and (where applicable)
source of funds. Enhanced Due Diligence (Recommendation
12) applies to PEPs. Simplified Due Diligence is
permitted only on regulator-approved low-risk profiles.

## §2 Product / disclosure lifecycle

```
product-draft → product-approved → product-active → product-superseded
                                       │
                                       └→ disclosure-published →
                                          customer-acceptance →
                                          loan-eligible-on-product
```

Disclosure templates regenerate on price / fee /
schedule changes; outstanding loans operate on the
disclosure they consented to.

## §3 Application-decisioning

```
intake → screened → scored → decided → notified
              │            │
              └→ blocked    └→ referred → manual review
```

Application decisions follow a four-eyes principle
where the credit limit exceeds the auto-approve
threshold; approval requires a different operator
than the originator.

## §4 Group-loan operating rhythm

| Step                     | Cadence                                      |
|--------------------------|----------------------------------------------|
| Group meeting            | weekly / fortnightly per group constitution  |
| Repayment collection     | at meeting                                   |
| Savings deposit          | at meeting                                   |
| New-loan approval        | at meeting                                   |
| Default escalation       | at meeting + escalation per policy           |

The protocol records meeting attendance; missing the
meeting more than the policy threshold triggers a
group-level intervention before an individual default
is recorded.

## §5 Credit-scoring governance

| Concern                  | Contract                                       |
|--------------------------|------------------------------------------------|
| Model identity           | semantic version + container digest pinned     |
| Explainability           | per-feature contribution exposed where bound   |
| Bias monitoring          | demographic-parity / equal-opportunity         |
|                          | metrics computed on a hold-out                 |
| Drift                    | per-week feature-distribution comparison       |
| Approval rate disparities | reported to the model-governance committee     |

A model release without bias-monitoring evidence
cannot move to production.

## §6 Disbursement and repayment flows

```
disbursement: instructed → rail-accepted → rail-settled → ledger-posted
                                  │
                                  └→ rail-rejected → cancelled

repayment:    received → rail-acknowledged → ledger-posted →
                allocated (principal / interest / fees / late-fee)
```

ISO 20022 message identifiers map to disbursement /
repayment events. Ledger postings reconcile end-of-day
against rail-operator statements; mismatches raise
stewardship tasks.

## §7 Restructuring decision discipline

| Trigger                        | Decision protocol                       |
|--------------------------------|-----------------------------------------|
| 30-day arrears                 | field-officer follow-up; counsel        |
| 60-day arrears                 | branch-manager review; restructuring    |
|                                | proposal optional                        |
| 90-day arrears                 | restructuring committee; IFRS 9 stage    |
|                                | 2 / 3 classification review              |
| Customer-initiated hardship    | hardship-letter + supporting evidence    |
| Force-majeure (regulator-      | regulator-issued moratorium honoured     |
| declared event)                | per directive                           |

Restructuring decisions sign with the approving
authority's key; restructuring inflates Stage 2 / 3
provisions per IFRS 9 ECL.

## §8 Write-off governance

```
proposed → reviewed (committee) → approved → posted → recovered (optional)
```

Write-off does not extinguish the customer's
obligation; recovery efforts continue per the
sponsor's collections policy and applicable consumer-
protection rules. Write-off events emit IFRS 9 Stage 3
recognition and feed the regulatory PAR report.

## §9 Regulatory reporting cadence

| Report                       | Cadence              | Authority         |
|------------------------------|----------------------|-------------------|
| PAR aging                    | monthly              | central bank       |
| Large-exposure               | monthly / quarterly  | central bank       |
| Suspicious-transaction (STR) | event-driven         | FIU                |
| Cash-transaction (CTR)       | over threshold       | FIU                |
| Consumer-protection metrics  | annually              | consumer regulator |
| Social-performance           | annually              | sponsor / investor|

Submissions sign with the implementation's key and
record the regulator-gateway acknowledgement
identifier.

## §10 Customer-protection enforcement

The protocol enforces the Client Protection Pathway
principles:

1. appropriate product design and delivery
2. prevention of over-indebtedness
3. transparency
4. responsible pricing
5. fair and respectful treatment
6. privacy of client data
7. mechanism for complaint resolution

Per-loan protocol gates:

- pre-disbursement disclosure with localised APR
- over-indebtedness check (debt-to-income, repayment
  capacity)
- responsible pricing (effective APR within band per
  product version)
- collections-conduct policy (no harassment; permitted
  hours; permitted contacts)
- complaint-handling SLA

## §11 Audit event chain

| Field          | Meaning                                                 |
|----------------|---------------------------------------------------------|
| `eventId`      | UUID                                                    |
| `eventTime`    | ISO 8601 with timezone                                  |
| `actor`        | identity (officer / customer / rail / regulator)        |
| `resourceRef`  | URI of the resource that changed                        |
| `action`       | created / signed / disbursed / collected / restructured |
| `priorHash`    | SHA-256 of the prior event payload                      |
| `signature`    | RFC 7515 JWS over the canonical event payload (RFC 8785)|

## §12 Recipient rights (privacy)

Customer-data rights honoured (per applicable law —
GDPR / K-PIPA / CCPA / LGPD / PIPL):

- access
- rectification
- erasure (subject to regulator-required retention)
- portability
- restriction
- objection (e.g. to direct marketing)

Rights events emit dedicated audit entries.

## §13 Reproducibility

A loan decision is `reproducible-strong` when the
input payload, the model version, the container
digest, the feature transformations, and the policy
gates are all content-addressed; `reproducible-weak`
when any is absent.

## Annex A — Worked over-indebtedness gate (informative)

A returning customer applies for a top-up loan; their
current debt-service-to-income ratio is 38 %. The
policy threshold is 35 %. The application moves to
`referred` with reason `dso-over-threshold`. A field
officer requests fresh income evidence; the customer
declines a smaller loan and the application closes.

## Annex B — Conformance disclosure

Implementations declare the audit-chain schema
version, the JWS algorithm registry, the model-
governance evidence available per the implementation's
scoring stack, and the sanctions-list update cadence.

## Annex C — Versioning

Field additions are minor; semantic redefinition is
major.

## Annex D — Operator-credential binding

| Credential                | Source                                |
|---------------------------|---------------------------------------|
| Field-officer / loan-officer | sponsor training + national rules   |
| KYC verifier              | sponsor + regulator (where required)  |
| Restructuring approver    | sponsor; segregation from originator   |
| FIU compliance officer    | per national appointment              |
| Customer-protection officer | sponsor / regulator                  |

A signing event without an active credential is rejected.

## Annex E — Time-source declaration

Audit-chain timestamps cite the time-source authority
(NTP stratum-1, NIST, KASI, KRISS, PTB).

## Annex F — Out-of-band collections conduct

The protocol records:

- contact attempts (channel, time, outcome)
- contact-frequency caps (per regulator)
- visit-time limits (e.g. daylight hours per local rule)
- third-party contact restrictions

Violations of conduct rules log as customer-
protection incidents and feed the protection-metrics
report.

## Annex G — Cooling-off and right-to-cancel

Per Reg Z (US), EU Consumer Credit Directive, and
analogous national rules, customers may carry a
right-to-cancel within a defined window after
contract signature:

| Jurisdiction context     | Cooling-off window               |
|--------------------------|----------------------------------|
| EU CCD                   | 14 calendar days                 |
| US Reg Z home-equity     | 3 business days                  |
| KR consumer credit       | 7 days                           |
| Sponsor-internal policy  | per-product disclosure           |

Cancellation events emit dedicated audit entries; if
funds were already disbursed, the customer's
repayment of principal proceeds; interest / fees waive
per the disclosure.

## Annex H — Stewardship-task SLA

| Task                          | Acknowledge  | Resolve         |
|-------------------------------|--------------|-----------------|
| Sanctions-screening hit       | 1 hour       | 1 business day  |
| Reconciliation mismatch       | 4 hours      | 2 business days |
| Customer-protection complaint | 1 business   | per regulator    |
|                               | day          | (typ. ≤ 30 days) |
| FIU follow-up                 | per FIU      | per FIU          |
| Bureau-update mismatch        | 1 business   | 5 business       |
|                               | day          | days             |

SLAs are sponsor-tunable; the configured table is
recorded on the stewardship-policy reference record.