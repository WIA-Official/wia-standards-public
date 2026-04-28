# WIA-privacy PHASE 3 — Protocol Specification

**Standard:** WIA-privacy
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE specifies the protocols binding the data format
(PHASE 1) to the API surface (PHASE 2): authentication of
data-subjects, DPO/CPO, processors, sub-processors, and
regulators; identity-verification proportionality for DSRs;
the consent-evidence chain; lawful-basis switching protocol;
breach-notification escalation; cross-border-transfer
mechanism validation; audit-chain construction; time
discipline; failure modes.

References (CITATION-POLICY ALLOW only):
- IETF RFC 8446 (TLS 1.3), RFC 7515 (JWS), RFC 7517 (JWK), RFC 9162 (Certificate Transparency 2.0)
- ISO/IEC 27701:2019 — PIMS process discipline
- ISO/IEC 29184:2020 — notice + consent chain
- ISO/IEC 29134:2023 — DPIA discipline
- ISO/IEC 27559:2022 — de-identification framework
- W3C DID Core 1.0 — for portable subject identity (where deployed)
- WIA-pq-crypto PHASE 3 — for ML-KEM/ML-DSA migration profiles

---

## §1 Authentication

DPO / CPO, processors, sub-processors, regulators, and
data-subjects authenticate using JWS-signed JWTs from the
deployment's identity authority. Token claims:

| Claim         | Source                                                 |
|---------------|--------------------------------------------------------|
| `iss`         | identity-provider URL                                  |
| `aud`         | the boundary URL                                       |
| `sub`         | principal URN                                          |
| `iat` / `exp` | per RFC 7519                                           |
| `wia.role`    | `data-subject`, `dpo`, `cpo`, `processor-admin`, `subprocessor`, `regulator`, `auditor`, `customer-admin` |
| `wia.scope`   | operation-class scopes                                 |
| `wia.subjectRef` | for subject-bound tokens                            |
| `cnf`         | mTLS certificate-thumbprint binding (regulator + auditor) |

Subject self-service tokens are short-lived (15 min). DPO /
CPO write tokens are short-lived (15 min). Regulator tokens
require mTLS binding so government CA chains gate access.

## §2 DSR identity-verification proportionality

DSRs require identity verification proportionate to the
sensitivity of the request, balancing privacy (avoid asking
for more PII than necessary) with security (avoid
imposter-driven exfiltration):

| Request                        | Verification floor                                            |
|--------------------------------|---------------------------------------------------------------|
| Withdrawal of consent          | account session + recovery channel (email / SMS link)          |
| Access to general data         | account session + recovery channel                             |
| Access to sensitive data       | account + step-up (FIDO2 / out-of-band)                        |
| Erasure                        | account + step-up                                              |
| Portability                    | account + step-up                                              |
| Authenticated party request    | DPA-defined verification (legal guardian, attorney, agent)     |

The boundary records the verification artefact (hash) for
audit. Verification failures emit a `dsr-verification-refused`
audit-chain entry.

## §3 Consent-evidence chain

Each consent operation produces a chained evidence record:

1. Notice version active at presentation (URN +
   notice signature)
2. Capture form / surface evidence (form-snapshot hash,
   audio-file hash for verbal, etc.)
3. Subject identifier at capture (with binding to the
   subject's pseudonym)
4. Consent record itself (JWS-signed)
5. Audit-chain entry referencing all of the above

Replaying any consent reproduces the entire chain so a
regulator can reconstruct what notice + presentation the
subject saw before granting consent. Notice mutations (per
PHASE 2 §8) preserve historical versions for replay.

## §4 Lawful-basis switching

A processing activity's lawful basis (Art. 6 GDPR / K-PIPA §15)
may change as circumstances change (e.g., from
`legitimate-interests` to `consent` for marketing after
PECR / CAN-SPAM / KISA opt-in tightening):

1. Controller proposes the switch with rationale
2. DPO / CPO sign-off recorded
3. Affected RoPA mutation declared
4. If switching to `consent`: re-consent campaign with
   subject communications evidence
5. If switching from `consent`: subject-notification per
   regime requirements (some regimes require explicit
   notice of basis change)

The transition is audit-chained; subjects' affected
consents are tagged with the basis change.

## §5 Breach-notification escalation

Breach notifications follow a regime-specific escalation:

```
Discovery
  ↓ (≤ 30 min)
DPO / CPO + security-incident commander notified
  ↓ (≤ 6 h)
Initial assessment: regulator-notification threshold met?
  ↓ (≤ 24 h)
Regulator pre-notification (if scope warrants early signal)
  ↓ (≤ 72 h GDPR / K-PIPA)
Regulator formal notification with available facts
  ↓ (≤ 30 days follow-up; per regime)
Final notification with full root-cause analysis
  ↓
Subject notification per Art. 34 / K-PIPA §34 / state laws
```

Each step emits an audit-chain entry signed by the
responsible authority. Late notifications are accepted but
flagged for regulator review.

## §6 Cross-border-transfer mechanism validation

Mechanism validation runs at every cross-border processing:

- **Adequacy decisions**: validity refreshed when the
  source jurisdiction publishes adequacy-decision changes
  (e.g., EU-US DPF status); deployments cache the latest
  decision text and revalidate against the publishing
  authority's surface
- **SCC**: deployments verify the SCC version is current
  (2021/914 superseded 2010/87; future versions tracked)
- **BCR**: validity follows the BCR's own renewal cadence
- **Adequacy-decision suspension**: triggers an immediate
  audit-chain entry; processing pauses until alternative
  mechanism is in place

The boundary's mechanism-validity cache refreshes daily;
manual revalidation supported via `POST /transfers/$revalidate`.

## §7 Audit chain

Every state transition emits an AuditEvent appended to a
Merkle audit log:

`kind` enum:
- `inventory-published` / `inventory-mutated`
- `ropa-published` / `ropa-state-changed`
- `consent-given` / `consent-withdrawn`
- `dsr-received` / `dsr-state-changed` / `dsr-fulfilled` /
  `dsr-refused`
- `dpia-published` / `dpia-state-changed`
- `breach-opened` / `breach-state-changed` /
  `breach-regulator-notified` / `breach-subject-notified`
- `transfer-published` / `transfer-mechanism-revalidated` /
  `transfer-mechanism-expired`
- `subprocessor-added` / `subprocessor-removed` /
  `subprocessor-changed`
- `notice-published` / `notice-superseded`
- `regulator-submission-acknowledged`

Anchored deployments mirror the audit chain to a
regulator-trusted witness on a declared cadence.

## §8 Subject-pseudonym lifecycle

Pseudonymous subject identifiers (`dataSubjectRef`) follow:

- Issued at first interaction (account creation, anonymous
  visitor cookie, etc.)
- Stable across sessions within the deployment
- Bound to PII via a separate access-controlled vault
- Re-identification requires documented operational reason
  (DSR fulfillment, breach notification, legal hold) and
  is audit-chained at `kind=subject-reidentified`
- Withdrawn on subject erasure: vault binding removed, the
  pseudonym retained as an opaque token in audit chain only

Cross-deployment portability (Art. 20 GDPR): on subject
request, the controller exports the subject's data in a
structured, commonly used, machine-readable format
(typically JSON-LD with the W3C DPV vocabulary).

## §9 Time discipline

Boundary clock: NTPv4 stratum-2. Submitter clocks (subject
mobile devices, DSR-form servers) are unverified; the
boundary records the boundary-receipt timestamp as the
authoritative time. Regulator-deadline timers reference
boundary-receipt timestamps.

## §10 Cryptographic-suite registry

| Concern                  | Default                             | Notes                                |
|--------------------------|-------------------------------------|--------------------------------------|
| Token signing            | ES256                               | mTLS-bound for regulator             |
| TLS                      | 1.3 (RFC 8446)                      | hybrid groups via WIA-pq-crypto      |
| Audit-chain hash         | SHA-256                             |                                      |
| Consent JWS signature    | PS256 or ES256                      |                                      |
| Notice signature         | ES256 by DPO / CPO                  |                                      |
| Evidence-artifact hash   | SHA-256                             |                                      |

Post-quantum migration follows WIA-pq-crypto PHASE 3 phase
declarations.

## §11 Failure modes

| Failure                                       | Behaviour                                       |
|-----------------------------------------------|-------------------------------------------------|
| Identity-provider JWKS unreachable            | Cached keys honoured until cache expiry        |
| Notice-store unreachable for consent capture  | Refuse consent (no consent without notice)     |
| Identity-verification service unreachable     | Defer DSR until service recovers; clock paused |
| Audit-chain write failure                     | Operation rejected (consistency requirement)    |
| Cross-border mechanism check unreachable      | Refuse transfer-publish; cached state honoured  |
| Regulator-submission webhook unreachable      | Queue + retry with deployment-policy backoff    |

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Worked breach-notification timeline (informative)

```
2026-04-28T03:14:00+09:00  T0   Detection (security-incident raised)
2026-04-28T03:30:00+09:00  +0:16 DPO + incident-commander notified
2026-04-28T07:00:00+09:00  +3:46 Initial scope assessment complete
2026-04-28T11:30:00+09:00  +8:16 Regulator pre-notification (KISA)
2026-04-29T03:14:00+09:00  +24h  Containment confirmed
2026-04-30T03:14:00+09:00  +48h  Subject-notification list finalised
2026-05-01T03:14:00+09:00  +72h  KISA formal notification submitted
2026-05-01T03:14:00+09:00  +72h  Subject notification dispatch begins
                                  (5일 within K-PIPA §34)
2026-05-28T03:14:00+09:00  +30d  Final regulator follow-up + root-cause
```

The audit chain captures every step with the responsible
authority's signature.

## Annex B — Worked DSR identity-verification (informative)

A subject submits an erasure DSR via web form:

1. Subject authenticated via account session
2. Step-up triggered (FIDO2 challenge)
3. FIDO2 attestation verified against the subject's
   registered authenticators
4. Verification artifact (attestation hash + timestamp)
   recorded
5. DSR state advances from `verifying` to `processing`
6. Regulatory deadline timer started

If FIDO2 fails (lost authenticator), the boundary falls
back to recovery channel + manual DPO review per the
deployment's policy.

## Annex C — Lawful-basis switch worked example (informative)

A controller switches from `legitimate-interests` to
`consent` for direct-marketing email:

1. RoPA entry mutation drafted with new `lawfulBases`
2. DPO sign-off recorded
3. Re-consent campaign launched (subjects given clear notice
   that ongoing receipt requires affirmative consent)
4. Consent grace window (e.g., 30 days) declared
5. Subjects who don't consent within window: marketing
   ceases; subjects retained for legitimate-interests
   processing only (account communications, etc.)
6. Audit-chain captures the basis change + per-subject
   re-consent outcome

## Annex D — Conformance levels

| Level     | Scope                                                                      |
|-----------|----------------------------------------------------------------------------|
| Surface   | structural conformance to PHASEs 1–3                                       |
| Verified  | annual third-party ISO/IEC 27701 audit                                    |
| Anchored  | continuous evidence package + regulator-witnessed audit-chain              |

## Annex E — Cross-domain handshake protocol

For domain-specific consent gating (e.g., medical-data-privacy):

1. Domain-specific consent record carries a back-reference
   to a privacy-domain consent (this PHASE)
2. Privacy-domain consent withdrawal propagates to bound
   domain-specific consents via webhook
3. Domain boundary verifies privacy-domain consent state
   at every operation; revoked privacy-domain consent
   refuses domain operations

## Annex F — Subject-rights legal-basis matrix

| Right        | GDPR        | K-PIPA       | CCPA            | Notes                            |
|--------------|-------------|--------------|------------------|----------------------------------|
| Access        | Art. 15     | §35          | §1798.100/110   |                                  |
| Rectification | Art. 16     | §36          | §1798.106       |                                  |
| Erasure       | Art. 17     | §37          | §1798.105       | "right to be forgotten" common term |
| Restriction   | Art. 18     | §37(2)       | n/a              |                                  |
| Portability   | Art. 20     | §35(3)       | §1798.130(a)(2) |                                  |
| Objection     | Art. 21     | §37          | §1798.135       | opt-out of sale (CCPA)           |
| Automated-decision objection | Art. 22 | §37-2 | n/a            |                                  |
| Limit use of sensitive data | n/a (consent) | n/a (consent) | §1798.121 | CPRA-introduced |

## Annex G — Notice-version-pinning worked example

```
1. Notice v3.2 published at 2026-04-01
2. Subject visits and grants consent → consent record
   references notice v3.2 by URN
3. Notice v3.3 published at 2026-04-15 (updated retention
   periods)
4. Existing consents under v3.2 remain valid (notice change
   without lawful-basis change)
5. New consents reference v3.3
6. If notice change is material (lawful basis or scope
   broadens), re-consent triggered
```
