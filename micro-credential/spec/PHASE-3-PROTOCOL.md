# WIA-micro-credential PHASE 3 — Protocol Specification

**Standard:** WIA-micro-credential
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE defines the operational protocols binding
records and API resources into auditable lifecycles:
issuer onboarding and accreditation, credential-class
publication and amendment, learner-evidence capture,
issuance lifecycle (offer → accept → bind → publish),
revocation and suspension, presentation construction
with selective disclosure, verification policy
evaluation, framework re-mapping cadence, and the
audit-event chain. Protocols are framed so an
accreditation auditor (national QA agency / EQF body)
or a recognising employer can reconstruct the full
lifecycle from the event log.

References (CITATION-POLICY ALLOW only):
- W3C Verifiable Credentials Data Model 2.0
- W3C Status List 2021
- 1EdTech Open Badges 3.0
- 1EdTech CLR 2.0
- ENQA Standards and Guidelines for Quality Assurance in the European Higher Education Area (ESG 2015)
- Bologna Process / Lisbon Recognition Convention
- ISO 21001 — Educational organisations management system
- ISO/IEC 19796-1 — Quality management for learning, education, and training
- IETF RFC 5424 (Syslog), RFC 7515 (JWS), RFC 8785 (JCS)
- ISO/IEC 27037 — digital evidence preservation

---

## §1 Issuer onboarding and accreditation

```
applicant → submitted → reviewed → accredited → active → renewed
                                       │
                                       └→ rejected → re-submission
```

Issuer accreditation is performed by the relevant
national authority (e.g. UK Quality Assurance Agency,
KCUE / KU on behalf of MoE for Korean higher
education, ANECA for Spain). The accreditation event
binds the issuer to one or more EQF / NQF levels.
Accreditations expire and renew; an expired issuer
cannot publish new credential classes.

## §2 Credential-class publication

```
draft → reviewed → published → versioned → retired
            │
            └→ rejected (returned to draft)
```

Class publication requires an internal QA review per
ESG 2015 and the issuer's quality manual; rejection
moves the class back to draft. Versioning is mandatory
on any change to learning-outcomes, EQF level,
assessment method, or credit value.

## §3 Learner-evidence capture

| Evidence kind        | Capture protocol                                 |
|----------------------|--------------------------------------------------|
| Proctored exam       | identity verification + invigilation log +       |
|                      | item-bank fairness sampling + answer-sheet hash  |
| Portfolio            | learner-curated submissions + assessor rubric    |
|                      | + sample-of-work integrity attestation            |
| Capstone project     | rubric scoring + plagiarism check artefact        |
| Observation          | structured observation form + observer credential|
| E-assessment         | item-response logs + answer-time distribution    |
|                      | + cheating-detection signal                      |

Evidence records are signed by the assessor and bound
to the issuance via `evidenceRef[]`. QA samples
re-grade a fixed proportion of evidence per class to
calibrate assessor agreement (Cohen's κ or analogous
coefficient is recorded).

## §4 Issuance lifecycle

```
prepared → offer-issued → wallet-bound → published → revoked / suspended / expired
```

Offer-issued events emit a credential offer (OID4VCI).
The wallet completes the flow with a key-binding
proof; the issuer publishes the issuance to the
recipient's wallet and writes the status-list bit.

`prepared` events are reversible (no recipient bound
yet); `wallet-bound` is irreversible save by revocation.

## §5 Revocation and suspension

| Reason                       | Action                            |
|------------------------------|-----------------------------------|
| Issuer error (wrong recipient)| revoke + re-issue                 |
| Fraud (recipient misrepresentation)| revoke; AE-equivalent record |
| Credential withdrawn (regulator)| revoke; trust-list propagation   |
| Class superseded             | suspend; advise upgrade           |
| Investigation                | suspend until decision            |

Revocations and suspensions are written to the W3C
Status List 2021 entry; verifiers fetch the list and
evaluate the bit at presentation-time.

## §6 Presentation construction

```
verifier-presentation-request → wallet-credential-match →
  selective-disclosure-construction → holder-binding-proof →
  VP-submission
```

The wallet enforces minimum-disclosure: it presents
only the claims required by the verifier's input
descriptors. Holder-binding (cnf claim) prevents a
presentation from being replayed by a third party that
does not control the holder's key.

## §7 Verification policy evaluation

A verifier evaluates the presentation against:

1. issuer DID resolution and trust-list inclusion
2. proof-validation (signature, expiry, key-binding)
3. status-list bit check (revoked / suspended / valid)
4. expiry / `validUntil` evaluation
5. claim-policy match (e.g. EQF level ≥ N, ESCO skill
   present, evidence-quality threshold)
6. policy-specific overrides (e.g. employer accepts
   suspended credentials with steward review)

Outcomes record the evaluating policy version.

## §8 Framework re-mapping cadence

| Framework           | Re-map trigger                                  |
|---------------------|-------------------------------------------------|
| EQF revision        | new EQF document publication                    |
| ESCO release        | each ESCO version (~annual)                     |
| ISCED revision      | each ISCED revision                              |
| National NQF change | per national authority                          |

Re-mapping events emit audit entries; recognising
parties may consume the new mapping or pin to the
historic mapping per their own policy.

## §9 Audit event chain

| Field          | Meaning                                                 |
|----------------|---------------------------------------------------------|
| `eventId`      | UUID                                                    |
| `eventTime`    | ISO 8601 with timezone                                  |
| `actor`        | identity (issuer / assessor / wallet / verifier)        |
| `resourceRef`  | URI of the resource that changed                        |
| `action`       | created / signed / published / revoked / verified       |
| `priorHash`    | SHA-256 of the prior event payload                      |
| `signature`    | RFC 7515 JWS over the canonical event payload (RFC 8785)|

## §10 Data-protection and recipient rights

Recipient rights honoured:

- access to all evidence held by issuer
- rectification of recipient identity attributes
- erasure: issuance status-list entry remains for
  verifier integrity, but personally-identifiable
  payload is tombstoned
- portability: VC export to a wallet of the recipient's
  choice

Privacy-rights events emit dedicated audit events.

## §11 Reproducibility

Issuance reproducibility is `strong` when issuer key,
class version, evidence digests, and issuance policy
version are all content-addressed; `weak` when any is
absent. The reproducibility tier is recorded so
recognising parties can decide on acceptance.

## §12 Trust frameworks

Verifiers consume one or more trust frameworks:

| Framework           | Operator                                         |
|---------------------|--------------------------------------------------|
| EBSI                | EU Blockchain Services Infrastructure             |
| eIDAS / eIDAS 2.0   | EU electronic identification (EUDI Wallet)        |
| KR DTAB             | Korea Digital Trust Authority list                |
| GAIA-X              | EU federated trust                                |
| Sponsor-internal    | per-employer / per-regulator trust list           |

Trust framework selection determines whose issuer DIDs
the verifier accepts.

## Annex A — Worked credential issuance (informative)

A vocational training provider, accredited under the
national NQF as level 4, completes a 12-week
"Introduction to SQL" cohort. The provider's QA cycle
samples 10 % of learner portfolios for re-grading; the
inter-rater κ exceeds the policy threshold. Each
learner who passes the assessment receives a credential
offer over OID4VCI; the wallet binds the credential to
the learner's DID. Three months later an employer
verifies the credential under the OID4VP profile; the
verification policy requires NQF level ≥ 3, status =
valid, and ESCO skill `data-management:sql:basic`. The
verification record outcome is `valid`.

## Annex B — Conformance disclosure

Implementations declare the audit-chain schema
version, the JWS algorithm registry, the W3C VC proof
types supported, the OID4VCI / OID4VP versions
implemented, and the trust frameworks consumed.

## Annex C — Versioning

Field additions are minor; semantic redefinition is
major.

## Annex D — Time-source declaration

Audit-chain timestamps cite the time-source authority
(NTP stratum-1, NIST, KASI, KRISS, PTB).

## Annex E — Assessor-credential binding

| Credential                 | Source                             |
|----------------------------|------------------------------------|
| Higher-education assessor  | university appointment              |
| Vocational assessor        | national vocational authority       |
| Industry-practitioner      | professional body certification     |
| Language proficiency       | per-test publisher (e.g. ALTE-aligned) |

A learner-evidence record cannot be signed by an
assessor without an active credential.

## Annex F — Inter-rater reliability sampling

Per ESG 2015 the issuer's QA cycle samples a fixed
proportion of evidence for re-grading by an independent
assessor. Inter-rater reliability is computed as
Cohen's κ (binary classifications) or Krippendorff's α
(ordinal scales). Threshold values are recorded on the
QA-policy reference record:

| Coefficient        | Acceptable threshold      |
|--------------------|---------------------------|
| Cohen's κ (binary) | ≥ 0.7                     |
| Krippendorff's α   | ≥ 0.667                    |
| Per-class accuracy | ≥ 0.85 vs. consensus       |

Reliability under threshold suspends the credential
class until the issuer remediates assessor training.

## Annex G — Outage and recovery (operational)

| Component outage         | Behaviour                              |
|--------------------------|----------------------------------------|
| Issuer signing service   | new issuances queued; offered on       |
|                          | recovery; status-list fallback         |
| Status-list endpoint     | verifiers consume cached list per      |
|                          | cache-control; degraded confidence     |
| DID-resolution service   | verifiers walk cached DID Documents    |
| Wallet                   | recipient-controlled; sponsor-side     |
|                          | offer queues until recipient online    |
| Verifier                 | per-policy retry / fallback            |

## Annex H — Anti-fraud detection

Implementations record signals indicating possible
fraud at issuance and verification:

- multiple wallets binding the same recipient identity
- implausible learning-velocity (course-completion
  faster than the notional learning hours allow)
- proctoring-flag accumulation on a recipient
- evidence-digest reuse across recipients
- presentation replay attempts (cnf-claim mismatch
  against verifier audience)

Detected signals open a stewardship task; pending the
review the issuance moves to suspended state.