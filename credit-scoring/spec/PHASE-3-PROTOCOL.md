# WIA-credit-scoring PHASE 3 — PROTOCOL Specification

**Standard:** WIA-credit-scoring
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern a
credit-scoring operator: the FCRA permissible-purpose
discipline that gates the operator's consumer-credit-
report inquiries; the model-risk discipline that
governs the development, validation, deployment, and
monitoring of the scoring model; the fair-lending
discipline that surfaces disparate-impact concerns
before a feature or model enters production; the
consumer-disclosure discipline that produces the
adverse-action notices required by ECOA and FCRA, the
consumer-credit pre-contractual information required
by EU CCD, and the equivalent KR-statutory
disclosures; the GDPR Article 22(3) right-to-
explanation discipline; and the model-monitoring
discipline that detects covariate shift, score-
stability drift, and outcome disparities over time.

References (CITATION-POLICY ALLOW only):

- ISO 9001:2015 (quality management systems)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 42001:2023 (AI management system)
- ISO/IEC 22989:2022, 23053:2022, 24029-2:2023
- IETF RFC 5905 (NTPv4), RFC 9421 (HTTP Message
  Signatures), RFC 9457 (Problem Details)
- US ECOA (15 USC 1691), Regulation B (12 CFR Part
  1002, including 1002.4 general rules, 1002.5
  request for information, 1002.6 evaluation, 1002.7
  rules concerning extensions of credit, 1002.9
  notifications, 1002.13 information for monitoring
  purposes)
- US FCRA (15 USC 1681), Regulation V (12 CFR Part
  1022, including 1022.40 to 1022.43 furnisher
  duties)
- US FHA (42 USC 3601)
- US TILA (15 USC 1601), Regulation Z (12 CFR Part
  1026, including 1026.43 ability-to-repay)
- US CFPB Examination Manual (ECOA, FCRA,
  unfair-deceptive-abusive-acts-or-practices)
- EU CCD recast (Directive (EU) 2023/2225) Articles
  6, 9, 18, 26
- EU Mortgage Credit Directive (Directive 2014/17/EU)
  Articles 18 to 20
- EU AI Act 2024 (Regulation (EU) 2024/1689) Annex
  III §5(b), Articles 8 to 17 (high-risk AI system
  obligations), Article 13 (transparency and
  provision of information to deployers), Article
  14 (human oversight), Article 15 (accuracy,
  robustness, cybersecurity)
- EU GDPR Articles 5, 6, 12 to 22, 22(3), 24, 25,
  32, 35
- KR Credit Information Use and Protection Act
- KR 금융소비자보호법
- Basel Committee on Banking Supervision Principles
  for the Sound Management of Operational Risk and
  the BCBS sound-practices guidance on model risk
  governance
- NIST AI RMF 1.0

---

## §1 FCRA Permissible-Purpose Discipline

The operator's consumer-credit-report inquiries against
a credit-bureau are gated by the FCRA permissible
purposes (15 USC 1681b):

- §1681b(a)(3)(A) — extension of credit to the
  consumer.
- §1681b(a)(3)(B) — review or collection of an
  existing account.
- §1681b(a)(3)(F) — legitimate business need in
  connection with a transaction initiated by the
  consumer.
- §1681b(a)(3)(E) — employment purposes (with the
  consumer's written authorisation per §1681b(b)).

Inquiries outside the permissible purposes are
rejected at the operator's policy-decision point; the
audit-event records the rejected inquiry. The
operator's furnisher-duty discipline (Regulation V 12
CFR 1022.40 to 1022.43) governs the operator's role
when furnishing tradeline information to a bureau.

## §2 ECOA Adverse-Action and Effects-Test Discipline

The ECOA discipline:

- §1002.5 — the operator MUST NOT request information
  that ECOA prohibits in the application form (race,
  colour, religion, national origin, sex, marital
  status, age outside specific safe-harbour uses;
  receipt of public-assistance income; rights
  exercised under the Consumer Credit Protection
  Act) except for HMDA / Regulation B 12 CFR 1002.13
  monitoring purposes.
- §1002.7(d) — special purpose credit programmes
  follow the §1002.8 requirements.
- §1002.9 — adverse-action notice within 30 days of
  the application, carrying the principal reasons.
- The effects-test discipline — the operator's
  monitoring identifies disparities in approval
  rates, pricing, or terms across protected classes
  and triggers the operator's fair-lending review
  when disparities exceed the operating threshold.

## §3 Model-Risk Discipline (Basel + NIST AI RMF +
       ISO/IEC 42001)

The model-risk lifecycle:

1. Definition — the model's purpose, target variable,
   in-scope population, and out-of-scope edge cases
   are documented before training begins.
2. Training-data discipline — the training data set
   is documented (population, sampling method, time
   window, feature engineering, label provenance);
   the dataset's representation across protected
   classes is reviewed for sufficiency.
3. Validation — out-of-time and out-of-sample
   validation, sensitivity analysis, fairness
   metrics (demographic-parity ratio, equal-
   opportunity differential, false-negative-rate
   differential), and an independent challenge
   review by a function organisationally separate
   from model development (Basel sound-practices).
4. Approval — the model-risk committee documents the
   acceptance criteria, the residual risks, and the
   monitoring discipline; the EU AI Act Article 9
   risk-management system, Article 10 data-and-data-
   governance, Article 11 technical-documentation,
   Article 14 human-oversight, and Article 15
   accuracy-robustness-cybersecurity disciplines all
   apply for high-risk credit-scoring AI systems.
5. Deployment — the model server emits the model
   version, the input feature snapshot, and the
   score-with-reason-codes; the operator's audit log
   captures every score computed.
6. Monitoring — covariate shift (PSI), performance
   stability (KS / AUC), and fairness metrics are
   monitored on the operating cadence; thresholds
   trigger the operator's revalidation or retirement
   workflow.

## §4 Fair-Lending Discipline

Disparate-treatment review — the operator's pricing
and underwriting policies are reviewed to ensure that
no policy discriminates on a protected basis.
Disparate-impact review — the operator's empirical
outcome data is reviewed for unjustified disparities;
features and rules with disparate-impact concerns are
either removed or supported by a less-discriminatory-
alternative analysis. The CFPB's UDAAP and ECOA
examinations exercise the discipline; the operator's
records of review, remediation, and challenger-model
analyses are preserved as compliance artefacts.

## §5 Consumer-Disclosure Discipline

Consumer disclosures cover the four moments at which
the consumer interacts with the credit decision:

- Pre-application — the operator's product
  disclosures (TILA Reg Z, EU CCD Article 6 pre-
  contractual information, KR 금융소비자보호법
  설명의무) explain the product's terms.
- Application — the operator collects only ECOA-
  permitted information.
- Decision — for declines or counter-offers the
  adverse-action notice (ECOA Reg B 12 CFR 1002.9,
  FCRA 15 USC 1681m, EU CCD Article 18 rejection
  reasoning, KR Credit Information Use and Protection
  Act rejection reasoning) carries the principal
  reasons, the bureau reference, and the redress
  channel.
- Post-decision — for GDPR-regulated operators the
  Article 22(3) right-to-explanation surface explains
  the logic of the automated decision; for FCRA-
  regulated operators the consumer's right to a free
  credit-report copy is exercised through 15 USC
  1681j(a).

## §6 GDPR Article 22 Discipline

For fully automated credit decisions:

- Article 22(1) — the consumer has the right not to
  be subject to a fully automated decision producing
  legal or similarly significant effects.
- Article 22(2)(a) — entered into a contract.
- Article 22(2)(b) — authorised by Member-State law
  with safeguards.
- Article 22(2)(c) — based on the consumer's explicit
  consent.
- Article 22(3) — meaningful information about the
  logic involved, the significance of the decision,
  the right to obtain human intervention, and the
  right to express a point of view.

The operator's Article 22 procedure carries the
reasoned-explanation template, the human-intervention
SLA, and the right-to-contest workflow.

## §7 KR Credit Information Use and Protection Discipline

KR Credit Information Use and Protection Act
(신용정보의 이용 및 보호에 관한 법률) Articles 32 and
33 govern the use of credit information; the
operator's discipline carries the consent record, the
purpose-limitation discipline, the third-party
disclosure record (for credit-bureau-supplied
information), and the consumer's right to receive a
rejection-reasoning explanation. KR 금융소비자보호법
설명의무 (the financial-consumer protection
explanation duty) applies to the lender's product
explanation at the application moment.

## §8 Identity, Time and Audit Discipline

NTPv4 stratum-2 or better is the operator's clock
baseline. Audit-events are emitted for every score
computation, every credit-bureau inquiry (recorded
under the FCRA permissible purpose), every adverse-
action notice delivery, every consumer dispute, and
every Article 22(3) request. Audit logs are integrity-
protected per the operator's chosen tamper-evident
mechanism; HIPAA-equivalent integrity controls apply
to the operator's PII / NPI substrate.

## §9 Model-Monitoring and Drift Discipline

The operator's monitoring surface produces three
families of metric on the operating cadence:

- Stability metrics — population-stability index
  (PSI) on the input feature distribution; the
  operator's threshold for triggering revalidation
  is documented in the model-risk record.
- Performance metrics — KS / AUC / Gini on out-of-
  time samples; calibration plots; expected-loss
  versus observed-loss reconciliation. Material
  performance degradation triggers the operator's
  retire-or-revalidate workflow.
- Fairness metrics — demographic-parity ratio,
  equal-opportunity differential, false-negative-
  rate differential across protected classes;
  thresholds aligned with the operator's fair-
  lending policy. Material disparities trigger the
  operator's challenger-model and remediation
  workflow.

Monitoring outputs feed the model-risk committee's
quarterly review. The EU AI Act Article 72 post-
market monitoring discipline applies for high-risk
credit-scoring AI systems and provides additional
field-performance reporting to the EU AI database.

## §10 Incident-Response and Breach-Notification Discipline

The operator's incident-response plan covers:

- PII / NPI breach (FCRA 15 USC 1681w disposal of
  records, GDPR Article 33 / 34 breach notification,
  KR Credit Information Use and Protection Act
  Article 39-4 leakage notification, KR PIPA Article
  34 leakage notification) — the operator notifies
  the supervisory authority and the affected
  consumers within the statutory window.
- Model-failure incident (a deployed model returns
  scores that materially diverge from validation
  baseline, or a data-feed corruption produces
  systematically biased input vectors) — the
  operator pauses scoring, reverts to the previous
  model version, and reports the incident under the
  EU AI Act Article 73 serious-incident reporting
  channel where the AI Act applies.
- Adverse-action-notice failure — undelivered or
  delayed adverse-action notices are reported and
  remediated under the operator's ECOA Reg B 12
  CFR 1002.9 compliance procedure.

## §11 Conformance

Implementations claiming PHASE-3 conformance enforce
the discipline at every relevant decision point,
maintain the model-risk records on the Basel + NIST
AI RMF + ISO/IEC 42001 baseline, deliver the adverse-
action notices within the operating jurisdiction's
mandated time-window, exercise the GDPR Article 22(3)
right-to-explanation discipline, and surface fair-
lending disparities at the operating cadence.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-credit-scoring
- **Last Updated:** 2026-04-28
