# WIA-open-banking PHASE 4 — Integration Specification

**Standard:** WIA-open-banking
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE describes how an open-banking deployment integrates the
data, APIs, and protocols from PHASEs 1–3 with adjacent systems:
core banking, regulator interfaces, OB Directory services, fraud
and sanctions services, customer authentication channels (mobile
banking app, hardware tokens), payment-rail clearing, and the
TPP ecosystem. It is non-prescriptive about specific vendors; it
specifies the integration *contracts* a deployment must satisfy.

References (CITATION-POLICY ALLOW only):
- OBIE Open Banking UK Directory and Conformance Tool
- EU PSD2 Open Banking Europe Directory (per Berlin Group)
- KFSC MyData Standard API + KFSC Directory
- WIA-payment-system (PHASE 1–4) — for clearing handoff
- WIA-mobile-payment (PHASE 1–4) — for wallet-bound SCA
- WIA-medical-data-privacy — referenced for cross-domain identifier
  handling (some open-banking flows touch medical-billing)

---

## §1 Core-banking integration

The ASPSP's core-banking system is the system of record. The
boundary integration contract:

- Account look-ups (by IBAN, domestic account number) flow through
  a thin adapter
- Balance, transaction, standing-order, and direct-debit reads are
  performed at consent-permission granularity; the adapter filters
  to consented permissions before returning to the boundary
- Payment-initiation calls trigger the core-banking's payment-
  posting workflow; finality posting follows the relevant clearing
  rail per WIA-payment-system
- Status callbacks from the clearing rail update the payment-
  initiation record and, where the consent permits, propagate
  notifications to the TPP

Core-banking systems do not reach outside the boundary directly;
all external traffic flows through the boundary so the audit
chain captures every TPP interaction.

## §2 OB Directory integration

EU/UK deployments integrate with the relevant OB Directory:

- **OBIE Open Banking Directory (UK)** — issues software statements,
  verifies eIDAS QWAC/QSealC chains
- **Open Banking Europe Directory (Berlin Group)** — equivalent for
  EU member states under PSD2
- **KFSC Directory (KR)** — issues certificates and software
  statements for KR 마이데이터 TPPs

The deployment's TPP-registration flow (PHASE 2 §1) verifies the
software statement against the directory's published JWKS. A
directory unreachable beyond the deployment's cache window
suspends new TPP onboarding while existing TPPs continue.

## §3 Regulator interface

Regulators (EU national CAs, UK FCA, KR FSC / FSS, BR BCB, AU ACCC,
JP FSA) request audit windows during investigations. The
integration contract:

- Regulator requests are signed; the boundary's audit chain records
  the request itself so regulator-initiated audits are auditable
- The boundary returns the AuditEvents intersecting the request,
  the daily roots, and the inclusion proofs
- Periodic reporting (e.g., quarterly transaction-volume reports
  required by FCA Open Banking Implementation Trustee, monthly
  reports under KR FSC) is emitted on schedule

The deployment policy enumerates the periodic reports required
under each regime so the boundary's reporting engine knows what
to produce when.

## §4 Customer authentication channels

SCA channels include:

- **Mobile banking app** (decoupled SCA via push notification +
  in-app biometric/PIN) — most common modern channel
- **Hardware token** (possession factor with OTP) — legacy but still
  in use
- **SMS OTP** — permitted but increasingly deprecated due to SIM-
  swap fraud; deployments SHOULD migrate away
- **FIDO2 / WebAuthn** — modern strong possession+inherence factor
  for browser-based flows
- **간편인증 (KR)** — PASS / Naver / Kakao authenticator apps for
  KR domestic SCA

Each channel's integration is signed; the boundary refuses SCA
evidence from unrecognised channels.

## §5 Payment-rail clearing handoff (WIA-payment-system)

Authorised payment initiations hand off to clearing:

- The PIS record references the resulting WIA-payment-system
  instruction record
- The UETR is preserved across the handoff so end-to-end
  reconstruction works
- Settlement status (accepted-for-settlement, settled, returned,
  rejected) propagates back to the TPP via PHASE 2 §8 notifications

The two standards share the audit chain so cross-domain
reconstruction (TPP → ASPSP → clearing → settlement) works
without repeating the audit emission.

## §6 Fraud and sanctions integration

Fraud engines and sanctions screening run as adjacent micro-services:

- Fraud screening is opt-in under the SCA's exemption regime per
  EBA Guidelines GL/2018/04 (Transaction Risk Analysis exemption);
  flows under TRA exemption skip SCA and are gated by the fraud
  engine's score
- Sanctions screening is mandatory for cross-border PIS instructions

Service unavailability has tiered consequences: fraud engine
unavailable suspends TRA exemption (SCA required); sanctions
unavailable suspends cross-border PIS while domestic flows
continue.

## §7 TPP-onboarding workflow

The TPP-onboarding workflow:

1. TPP registers with the relevant directory (OBIE / Berlin Group / KFSC)
2. TPP obtains its software statement and eIDAS or domestic
   certificates
3. TPP's developer team integrates against the ASPSP's sandbox
   (out of regulatory scope of this PHASE)
4. TPP submits production credentials via PHASE 2 §1
5. ASPSP boundary validates and onboard the TPP within the
   deployment's onboarding SLA (typically ≤ 2 business days for
   directory-verified TPPs)
6. TPP starts production traffic

A TPP whose licence is suspended is automatically suspended at
the boundary; existing consents are paused; in-flight calls
complete; new operations refuse.

## §8 Operational SLAs

| Concern                                          | Default SLA              |
|--------------------------------------------------|--------------------------|
| TPP onboarding (directory-verified)              | ≤ 2 business days        |
| Consent setup p95 added latency                  | ≤ 200 ms                 |
| SCA invocation (decoupled) timeout               | ≤ 5 minutes              |
| AIS account-information read p95                 | ≤ 500 ms                 |
| PIS payment-initiation acceptance                | ≤ 1 s                    |
| CoF response                                     | ≤ 500 ms                 |
| Notification webhook delivery                    | ≤ 30 s                   |
| Audit chain entry available after operation      | ≤ 1 s                    |

Tighter SLAs negotiable per deployment; loosening requires
operational sign-off.

## §9 Acceptance criteria

A deployment claims conformance when:

1. Every active consent has SCA evidence on file with valid
   dynamic linking (for PIS consents)
2. Every TPP currently active has an unrevoked directory entry
   and unexpired eIDAS / domestic certificates
3. Every payment-initiation record in the past quarter has a
   matching WIA-payment-system clearing record
4. Every cross-border PIS has sanctions screening evidence
5. Quarterly regulatory reports submitted on time
6. FAPI 2.0 conformance test suite passes against the production
   endpoints
7. eIDAS TSL refresh / KFSC directory refresh is current
8. Quarterly compliance report has no integrity-check failures

A deployment failing any of these reports the gap in its compliance
package rather than concealing it.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Common pitfalls (informative)

- **eIDAS TSL drift** — national TSLs change as new TSPs are added
  or removed; the deployment's TSL cache MUST refresh at least daily
- **Software-statement expiry** — software statements have a short
  lifetime (≤ 30 days for OBIE); the deployment SHOULD surface
  expiring statements to TPPs in advance
- **SCA-channel substitution drift** — customer may rotate their
  decoupled-SCA channel (e.g., new mobile device); existing consents
  remain valid but require re-binding before next SCA invocation
- **Notification-webhook URL drift** — TPPs occasionally change
  webhook URLs without updating their software statement; the
  deployment SHOULD surface delivery failures to operations
- **TRA-exemption rate drift** — TRA usage must stay below
  Article 19 thresholds; the deployment SHOULD monitor exemption
  rate quarterly and surface trends to compliance

## Annex B — Decommissioning (informative)

When an ASPSP is decommissioned:

1. All active consents are notified to be revoked or migrated to
   the receiving custodian
2. In-flight payment initiations either complete (clear) or are
   cancelled with structured reasons
3. Final daily root is sealed; the chain is exported to the
   receiving custodian
4. The OB Directory entry is updated to "transferred" with the
   receiving custodian's identifier
5. Coalition partners (ASPSPs in cross-border PIS) are notified

The decommissioning manifest is itself an audit event in the chain.

## Annex C — Quarterly compliance report (informative)

The boundary emits a quarterly compliance report covering:

- Active consents by regime, scope, and permission
- SCA invocations, exemptions invoked, dynamic-linking failures
- TPP onboarding / off-boarding events
- Payment-initiation volumes by clearing rail and outcome
- AIS access volumes and permission-filter actions
- TRA-exemption rate vs. fraud-rate threshold
- eIDAS TSL refresh status / KFSC directory refresh status
- Audit-chain integrity check results
- Per-regime regulatory-report submission status

The report is signed and is itself in scope for the audit chain
so that report tampering would surface in the chain.

## Annex D — Decommissioning (informative)

When a deployment is decommissioned:

1. Active consents are notified to be revoked or migrated
2. In-flight payment initiations either complete or are cancelled
3. TPP webhook notifications are flushed; final notifications are
   acknowledged before the chain is sealed
4. Final daily root is sealed; the chain is exported to the receiving
   custodian
5. The OB Directory entry is updated to "transferred"

The decommissioning manifest is itself an audit event in the chain.

## Annex E — Common pitfalls

TPP webhook URL drift, eIDAS TSL refresh lag, SCA-channel substitution, TRA-exemption-rate drift — see WIA-payment-system PHASE 4 Annex C for the financial-domain pitfalls list which applies here too.

## Annex F — TPP-licensure verification cadence (informative)

TPP licences are revocable; the boundary verifies licence status:

- At every API call (cached at deployment-policy interval, typically 1 hour)
- On registry-published licence-revocation notifications (push)
- At quarterly directory refresh (full reconciliation)

A TPP whose cached licence is stale is refused with a fresh
verification on the next call.
