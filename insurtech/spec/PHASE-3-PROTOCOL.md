# WIA-insurtech PHASE 3 — PROTOCOL Specification

**Standard:** WIA-insurtech
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an accredited
insurtech operator: regulator authorisation and prudential
supervision, product filing and approval, underwriting governance,
claim handling discipline, anti-fraud governance, consumer
protection and treating-customers-fairly discipline, reinsurance
governance, sanctions screening, complaint handling, recordkeeping,
and operator wind-down or run-off.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO 22301:2019 (business continuity management)
- ISO 31000:2018 (risk management)
- ISO 37001:2016 (anti-bribery management systems)
- ISO 19600:2014 / 37301:2021 (compliance management)
- ISO/IEC 17025:2017 (calibration / testing laboratories,
  cited where forensic adjusters operate accredited labs)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27701:2019 (privacy information management)
- IFRS 17 (Insurance Contracts)
- Solvency II Directive (EU); Insurance Capital Standard (IAIS);
  RBC frameworks (US, JP, KR, CA)
- IAIS Insurance Core Principles
- FATF Recommendations (AML/CFT)
- ISO 8601 (date and time)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8446 (TLS 1.3)

---

## §1 Regulator Authorisation

An insurer or MGA MAY claim conformance to WIA-insurtech only
after the prudential regulator (or the relevant licensing
authority for MGAs) has issued a valid licence covering the
lines of business the operator writes. Licence amendments are
recorded against the programme; amendments narrow the lines of
business the API will accept new policies under.

## §2 Solvency and Capital

Operators report solvency and capital adequacy at the cadence
the regulator requires (typically quarterly QRTs under Solvency
II, NAIC RBC filings annually with quarterly supplements, KICS
filings under the Korean regime). The submission cadence and
the responsible chief actuary are recorded in the programme's
quality dossier.

Operators that fall below the regulator's solvency capital
requirement (SCR) trigger the regulator's recovery-plan workflow;
the recovery plan is recorded against the programme and
referenced in subsequent regulator submissions.

## §3 Product Filing and Approval

Insurance products are filed with the regulator (or the
state-level filing authority in the US) before any policy can be
bound under the product code. The product register holds product
codes, the regulator filing references, and the policy-form
references that the product issues. Product withdrawals follow
the regulator's runoff procedure for in-force policies.

## §4 Underwriting Governance

Underwriting models (rules engines, machine-learning models)
that influence consumer outcomes (declines, tier shifts, premium
multipliers) are governed by the operator's model-risk-
management framework. The framework records:

- model identifier and version,
- training data lineage (for ML models),
- monitoring metrics (calibration drift, disparate-impact
  metrics in jurisdictions that require them),
- review cadence by the operator's model-risk committee,
- adverse-action explanation templates that the model emits at
  decision time (PHASE-1 §7).

Models that fail review enter remediation; declines emitted by
a model under remediation are routed to human underwriter
referral.

## §5 Claim Handling Discipline

Claims are handled per the operator's published claim-service
standards: acknowledgement within the operator's published
window, first contact with the claimant within the operator's
published window, and a target settlement window per peril and
per claim complexity tier. Standards are recorded in the
operator's quality dossier and exposed to consumers through the
programme's discovery document.

Complex claims (litigation, suspected fraud, large losses above
the operator's threshold) follow escalated workflows with
mandatory peer review and senior-claim-handler signoff.

## §6 Anti-Fraud Governance

Suspected-fraud investigations follow the operator's special-
investigations-unit (SIU) protocol and respect the regulator's
fraud-bureau notification requirements. Confirmed fraud cases
emit notifications to the relevant fraud bureau (the IFB in the
UK, the NICB in the US, equivalent bureaux in other
jurisdictions) and update the consumer's status only after
appeal rights have been exhausted.

## §7 Consumer Protection and TCF

Treating-customers-fairly principles govern product design,
distribution, and claims. The operator records its TCF
framework in the quality dossier and conducts at least annual
TCF reviews that test products against the operator's customer
outcomes statement.

Vulnerable consumers (per the regulator's vulnerable-consumer
guidance) receive enhanced support during quote, claim, and
complaint workflows; the operator records the vulnerability
flagging policy and trains front-line staff against it.

## §8 Reinsurance Governance

Reinsurance counterparties are subject to the operator's
counterparty risk framework: minimum financial-strength rating,
collateral arrangements when below threshold, and the operator's
maximum aggregated exposure to any one reinsurer. Treaty
renewals and cancellations follow the operator's treaty-
renewal calendar and are recorded against each treaty.

Facultative cessions are subject to per-risk pre-binding
authorisation through the operator's reinsurance team;
unauthorised facultative cessions are flagged in the operator's
reconciliation workflow.

## §9 Sanctions Screening

All parties (policyholders, insureds, beneficiaries, claimants,
reinsurers, third-party payees) are screened against the
applicable sanctions lists at intake and at the operator's
ongoing screening cadence (typically daily for high-risk lines).
Screening hits trigger the operator's sanctions workflow:
investigation, regulator notification when required, and policy
or claim disposition consistent with the sanctions law.

## §10 Complaint Handling

Complaints are recorded against the policy or claim, classified
per the operator's complaint taxonomy, and resolved within the
regulator's required window. Operators report complaint
volumes and themes to the regulator at the cadence the
regulator requires (typically annually or biannually) and to
their internal TCF committee.

Complaints that escalate to the operator's external dispute
resolution body (the Financial Ombudsman Service in the UK,
the National Association of Insurance Commissioners' channels
in the US, equivalent bodies elsewhere) carry the body's
reference identifier in the complaint record.

## §11 Records Retention

Programme records — every record defined in PHASE-1, the API
audit logs, the consumer-disclosure artefacts, the underwriting-
decision rationales, the claim files, the reinsurance bordereaux,
the regulator submissions, and the complaint files — retain per
the regulator's required window. Life-business records and
liability claims with long-tail exposure retain indefinitely.

## §12 Time Synchronisation

Operator clocks synchronise per RFC 5905 (NTPv4) against a
national-metrological-laboratory stratum-1 service so that
timestamped records — premium-receipt times, claim-notification
times, sanctions-screening times — are consistent across the
operator's systems and consistent with reinsurer ledgers.

## §13 Cross-Jurisdictional Programme Operation

Multi-jurisdiction operators honour each participating
jurisdiction's licensing rules, market-conduct rules,
data-protection rules, and prudential rules. Per-policy records
carry the governing jurisdiction so that downstream consumers
can apply the right legal regime.

## §14 Run-Off and Resolution

Operators that enter run-off cease writing new policies, manage
claims through closure, and report run-off progress to the
regulator at the regulator's required cadence. Operators in
resolution operate under the resolution authority's direction
and may transfer policies to a successor under the regulator's
portfolio-transfer process; the API enforces freeze conditions
on transferred policies.

## §15 Quality Dossier

The operator's quality dossier records the regulator
authorisations, the model-risk-management framework, the SIU
protocol, the TCF framework, the reinsurance counterparty
register, the complaint taxonomy, and the operational events
the operator has experienced. The dossier is reviewed at least
annually by the operator's chief risk officer.

## §16 Operational Resilience and Business Continuity

Operators that fall under the regulator's operational-resilience
expectations (DORA in the EU, the operational-resilience
framework in the UK, equivalent regimes elsewhere) record their
important business services, the impact tolerances per service,
the third-party dependencies, and the incident-response
playbooks in the operational-resilience dossier.

Resilience-impacting incidents (PAS unavailability, claim-
handling outage, telematics-feed loss) trigger the operator's
incident-response playbook, are recorded against the
programme, and are reported to the regulator within the
regulator's required notification window.

## §17 Internal Audit and Three Lines of Defence

The operator's three-lines-of-defence model is recorded in the
quality dossier: front-line risk-owners (line 1), the
compliance and risk-management functions (line 2), and the
internal audit function (line 3). Internal audit reports
covering insurtech-relevant risks are referenced from the
operator's annual audit plan; remediation actions are tracked
to closure in the quality dossier.

## §18 Climate-Risk Disclosure

Operators that fall under climate-risk disclosure expectations
(TCFD-aligned regimes, ISSB IFRS S2, jurisdictional climate-
disclosure rules) record their physical and transition risk
analyses, the per-line scenario testing, and the climate-
related underwriting policies that flow from the analysis.

## §19 Outsourcing and Critical-Service-Provider Oversight

Outsourced functions (PAS hosting, claim-handling TPAs, BPO
services, cloud infrastructure) are subject to the operator's
outsourcing policy: due-diligence prior to onboarding, service-
level agreements with measurable outcomes, exit and
substitutability arrangements, and ongoing monitoring of
provider performance.

Critical service providers (those whose failure would impair
the operator's important business services within the
impact-tolerance window of §16) are designated and reported to
the regulator where the regulator's outsourcing rules require
notification.

## §20 Sustainability and Net-Zero Commitments

Operators that have published sustainability or net-zero
commitments (NZIA-aligned, TCFD-aligned, ISSB IFRS S2-aligned)
record the commitment text, the per-line underwriting policies
that flow from the commitment, and the measurement framework
the operator uses to track progress against the commitment.
Commitment progress is reported externally on the cadence the
operator has committed to.

## §21 Cyber-Underwriting Discipline

Operators that write standalone cyber lines or provide cyber
endorsements to other lines record their cyber-underwriting
methodology in the quality dossier: the threat-intelligence
sources they consume, the assumed-attacker capability tiers,
and the systemic-event aggregate-exposure caps they apply to
prevent silent-cyber accumulation across the operator's book.

## §22 Conformance and Auditing

A programme conformant with WIA-insurtech publishes its
licensing references, its product register, its quality
dossier, and the catalogue of consumer disclosures issued, and
answers an annual self-assessment that maps each clause of this
PHASE to the operator's implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-insurtech
- **Last Updated:** 2026-04-28
