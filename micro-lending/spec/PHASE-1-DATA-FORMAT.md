# WIA-micro-lending PHASE 1 — Data Format Specification

**Standard:** WIA-micro-lending
**Phase:** 1 — Data Format
**Version:** 1.0
**Status:** Stable

This PHASE defines the canonical data format for
micro-lending operations covering individual and
group-loan products, mobile-money disbursement and
repayment, KYC / AML records, customer-protection
disclosures, loan-cycle events, restructuring and
write-off, regulatory reporting, and outcomes
attribution. The format aligns with the BIS / Basel
guidance on microfinance, FATF Recommendations on
KYC / AML, ISO 20022 for payment messages, and the
SMART Campaign / Client Protection Pathway customer-
protection principles.

References (CITATION-POLICY ALLOW only):
- BIS — Basel Core Principles for Effective Banking Supervision (BCP)
- BIS — Microfinance activities and the Core Principles for Effective Banking Supervision
- FATF Recommendations (most recent revision) — international AML / CFT standards
- ISO 20022 — Universal financial industry message scheme
- ISO 4217 — currency codes
- ISO 17442 — Legal Entity Identifier (LEI)
- ISO 18092 / ISO 14443 — NFC for low-end payment terminals
- ITU-T X.1216 — security framework for digital financial services
- IFC Performance Standards (PS 1, PS 2)
- SPI4 (CERISE+SPTF) — Universal Standards for Social and Environmental Performance Management
- Client Protection Pathway (formerly SMART Campaign Client Protection Principles)
- IASB IFRS 9 — Financial Instruments (impairment, ECL)
- US Reg E (12 CFR Part 1005) — electronic fund transfers
- US Reg Z (12 CFR Part 1026) — Truth in Lending
- EU PSD2 (Directive 2015/2366) and PSD3 (current proposal)
- IETF RFC 8259 (JSON), RFC 8785 (JCS), RFC 4122 (UUID), RFC 7515 (JWS), RFC 9530 (Content-Digest)

---

## §1 Scope

This PHASE applies to systems that originate, service,
collect, and report on micro-loans for individuals,
solidarity groups, village banking units, micro-
enterprises, and small-and-medium enterprises (SMEs)
where the principal balance is below the regulator's
microfinance threshold.

In scope: customer record, loan-product record, loan-
account record, group record (solidarity / VSLA /
self-help group), application record, disbursement
record, repayment record, restructuring record, write-
off record, customer-protection disclosure record, and
regulatory-report record. Out of scope: large-ticket
commercial lending (handled by general banking
standards) and microinsurance (handled by the
microinsurance standard).

## §2 Customer record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `customerRef`        | UUID (RFC 4122)                                 |
| `kind`               | `individual`, `sole-proprietorship`,            |
|                      | `solidarity-member`, `vsla-member`, `sme`,      |
|                      | `cooperative`                                   |
| `legalName`          | localised label (BCP 47); for entities the     |
|                      | registered name                                  |
| `idVerification`     | KYC verification record reference                |
| `nationalIdHash`     | one-way hash of the national identifier         |
| `incorporationCountry`| ISO 3166-1 alpha-3 (where entity)               |
| `lei`                | ISO 17442 (where issued, typically SMEs)         |
| `taxId`              | per-jurisdiction tax identifier                  |
| `pepStatus`          | per FATF — `none`, `domestic`, `foreign`,        |
|                      | `international-organisation`                     |
| `sanctionsScreening` | last-screen reference + outcome                  |
| `riskRating`         | low / medium / high (per FATF risk-based)        |
| `householdRef`       | for poverty-attribution analysis                  |
| `groupRef[]`         | for solidarity-loan members                      |

Customer records hold opaque references; PII lives in
a sponsor-controlled vault under FATF Recommendation
10 (CDD).

## §3 Loan-product record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `productRef`         | URI                                             |
| `productKind`        | `individual-loan`, `group-loan`, `vsla-loan`,   |
|                      | `digital-credit`, `nano-loan`, `agri-loan`,     |
|                      | `sme-loan`, `housing-loan-micro`                |
| `currency`           | ISO 4217                                        |
| `principalRange`     | min / max in `currency`                         |
| `termRange`          | min / max in months                             |
| `interestModel`      | flat / declining / hybrid; nominal vs. APR      |
|                      | (per Reg Z disclosure where bound)              |
| `feeSchedule`        | origination fee, processing fee, insurance      |
|                      | fee, application fee — disclosed                 |
| `effectiveAprBand`   | publishable APR per Reg Z / equivalent          |
| `repaymentFrequency` | weekly, fortnightly, monthly                     |
| `gracePeriod`        | days                                            |
| `collateralPolicy`   | none / cash-secured / asset-secured /            |
|                      | guarantor / movable-asset (per IFC MAR Toolkit) |

Product records are versioned; rate / fee changes are
versioned and bound to disclosures.

## §4 Loan-account record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `loanAccountRef`     | UUID                                            |
| `customerRef`        | §2                                              |
| `productRef`         | §3                                              |
| `groupRef`           | §5 (where group-loan)                           |
| `principal`          | numeric in `currency`                           |
| `interestRate`       | per period                                      |
| `termMonths`         | integer                                         |
| `disbursementDate`   | ISO 8601                                        |
| `firstRepaymentDate` | ISO 8601                                        |
| `maturityDate`       | ISO 8601                                        |
| `outstandingBalance` | numeric (current)                               |
| `parStatus`          | current / 1-30 / 31-60 / 61-90 / >90 (PAR>90)   |
| `accountStatus`      | active / restructured / written-off / settled    |

`parStatus` (Portfolio at Risk) buckets follow CGAP
microfinance reporting conventions; ≥30 days reflects
an at-risk classification.

## §5 Group record (solidarity / VSLA)

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `groupRef`           | UUID                                            |
| `kind`               | `solidarity-group`, `vsla`, `shg`, `cooperative`|
|                      | (cooperative as commercial entity)              |
| `members[]`          | customer-references with role (chair, secretary,|
|                      | treasurer, member)                              |
| `meetingCadence`     | weekly / fortnightly / monthly                   |
| `groupGuaranteePolicy`| joint-and-several / first-loss-pool / mutual    |
|                      | guarantee fund                                   |
| `groupAccountRef`    | for the group's pooled savings / loans          |

Group records bind member loans through `groupRef[]`
on the customer record; default by one member impacts
the group's collective record per the policy.

## §6 Application record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `applicationRef`     | UUID                                            |
| `customerRef`        | §2                                              |
| `productRef`         | §3                                              |
| `requestedPrincipal` | numeric                                         |
| `applicationChannel` | branch / agent / mobile-app / USSD / agent-     |
|                      | network / web                                    |
| `creditScoringInput` | model identifier + version + container digest    |
| `creditScoreOutput`  | risk band + per-feature contribution (where     |
|                      | the model is explainable)                       |
| `decision`           | approved / declined / referred / withdrawn       |
| `decisionReason`     | controlled list (collateral / income / DTI /    |
|                      | screening / withdrawn-by-customer)               |
| `decisionTime`       | ISO 8601                                        |

Decisions cite the model version so post-hoc audit
can reproduce the score from the recorded inputs.

## §7 Disbursement record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `disbursementRef`    | UUID                                            |
| `loanAccountRef`     | §4                                              |
| `amount`             | numeric in `currency`                           |
| `disbursementChannel`| cash-counter / mobile-money / bank-transfer /   |
|                      | prepaid-card / agent-network                     |
| `paymentInstrumentRef`| ISO 20022 instrument identifier                  |
| `disbursementTime`   | ISO 8601                                        |
| `feeDeducted`        | numeric                                         |
| `feeDisclosureRef`   | customer-protection disclosure (§10)             |

Disbursement events emit ISO 20022 payment messages
(pacs.008 for credit transfer where bank-rail; mobile-
money rails carry the operator-specific equivalent).

## §8 Repayment record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `repaymentRef`       | UUID                                            |
| `loanAccountRef`     | §4                                              |
| `amount`             | numeric                                         |
| `principalAllocation`| numeric                                         |
| `interestAllocation` | numeric                                         |
| `feeAllocation`      | numeric                                         |
| `lateFeeAllocation`  | numeric                                         |
| `paymentChannel`     | per disbursement-channel list                    |
| `paymentTime`        | ISO 8601                                        |
| `valueDate`          | ISO 8601                                        |

Allocation rules follow the disclosure document; if
the customer's payment is short the allocation
priority is published in the loan agreement.

## §9 Restructuring and write-off records

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `restructuringRef`   | UUID                                            |
| `loanAccountRef`     | §4                                              |
| `kind`               | `re-aging`, `term-extension`, `principal-      |
|                      | reduction`, `interest-waiver`, `payment-holiday`|
| `decisionDate`       | ISO 8601                                        |
| `effectiveTerms`     | new principal / interest / term                  |
| `accountingTreatment`| per IFRS 9 stage (1 / 2 / 3)                     |

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `writeOffRef`        | UUID                                            |
| `loanAccountRef`     | §4                                              |
| `amountWrittenOff`   | numeric                                         |
| `decisionDate`       | ISO 8601                                        |
| `recoveryRef[]`      | post-write-off recoveries                        |

## §10 Customer-protection disclosure record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `disclosureRef`      | UUID                                            |
| `customerRef`        | §2                                              |
| `loanAccountRef`     | §4                                              |
| `language`           | BCP 47 tag                                       |
| `apr`                | publishable APR                                  |
| `totalCostOfCredit`  | numeric                                         |
| `repaymentSchedule`  | structured schedule                              |
| `feesEnumerated`     | line-item list                                   |
| `disputeMechanismRef`| ombudsman / customer-redress contact             |
| `acceptanceMethod`   | wet-ink / e-sign / verbal-witnessed              |
| `acceptedAt`         | ISO 8601                                        |

Disclosures sign with the implementation's signing key
so post-hoc compliance review confirms the document
the customer accepted.

## §11 Regulatory-report record

| Field                | Source / Binding                                |
|----------------------|-------------------------------------------------|
| `reportRef`          | UUID                                            |
| `regulatorRef`       | central bank / supervisor identifier             |
| `reportType`         | par-aging / large-exposure / suspicious-         |
|                      | transaction-report (STR) / cash-transaction-     |
|                      | report (CTR) / FIU-filing                       |
| `period`             | ISO 8601 interval                                |
| `submissionTime`     | ISO 8601                                        |
| `submissionRef`      | regulator gateway identifier                     |

## §12 Cross-domain references (informative)

- WIA-mobile-payment — for disbursement / repayment rails
- WIA-cross-border-payment — for international remittance
- WIA-kyc-aml — for the screening pipeline
- WIA-credit-scoring — for model governance
- WIA-financial-inclusion — for outcomes attribution

## Annex A — Worked group-loan record (informative)

```json
{
  "loanAccountRef": "la-2026-04-12-007",
  "customerRef": "cu-2026-001",
  "groupRef": "g-village-bukit-005",
  "principal": 50000,
  "currency": "KES",
  "interestRate": {"model":"declining","periodicRate":0.025},
  "termMonths": 12,
  "outstandingBalance": 49000,
  "parStatus": "current"
}
```

## Annex B — Conformance disclosure

Implementations declare the JSON-Schema URIs they
serve, the ISO 20022 message profiles supported, the
PAR-bucket conventions used, and the IFRS 9 stage
classification rules.

## Annex C — Versioning

Field additions are minor; semantic redefinition is
major.
