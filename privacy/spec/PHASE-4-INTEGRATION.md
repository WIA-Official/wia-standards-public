# WIA-privacy PHASE 4 — Integration Specification

**Standard:** WIA-privacy
**Phase:** 4 — Integration
**Version:** 1.0
**Status:** Stable

This PHASE specifies how a privacy-control deployment
integrates the data, APIs, and protocols from PHASEs 1–3
with broader operational systems: identity / authentication,
EHR / clinical, payment, marketing-automation, customer-data-
platform (CDP), HRIS, regulator portals, breach-notification
clearinghouses, B2B subprocessor disclosure systems, and the
common pitfalls of running a privacy-control deployment at
scale.

References (CITATION-POLICY ALLOW only):
- ISO/IEC 27701:2019 — PIMS extends 27001 governance
- ISO/IEC 29184:2020 — notice + consent surfaces
- W3C DPV (Data Privacy Vocabulary)
- WIA-identity-management, WIA-medical-data-privacy,
  WIA-medication-adherence, WIA-payment-system,
  WIA-network-security, WIA-pq-crypto

---

## §1 Identity / authentication integration

The deployment integrates with the broader identity stack:

- **Subject identity**: cross-reference WIA-identity-management
  PHASE 4 §3 for FIDO2 / OIDC / DID-bound subject portfolios
- **DPO / CPO identity**: separate role-bound principal
  with credential lifecycle owned by the deployment's
  governance authority
- **Regulator identity**: government-CA-issued mTLS
  certificates pinned to the deployment's regulator-
  trust-store
- **Customer-administrator identity (B2B)**: per-customer
  IdP federation; SAML / OIDC supported

Identity-binding mutations (subject merge / split / pseudonym
rotation) are themselves audit-chained.

## §2 EHR / clinical integration

For deployments serving clinical-care contexts:

- **Cross-domain consent**: WIA-medical-data-privacy PHASE 1 §3
  references this PHASE's consent records; clinical operations
  verify both standards' consents
- **DSR fulfillment**: clinical records subject to specialised
  fulfillment (e.g., HIPAA designated record set rules in US,
  K-PIPA 의료법 specific record-keeping)
- **Breach notification**: HIPAA breach-notification rule
  applies in addition to this PHASE's regime when PHI involved
- **EHR vendor relationship**: Business Associate Agreement
  (BAA) tracked as a sub-processor record

## §3 Payment / financial integration

For deployments processing payment data:

- **Cross-domain consent**: WIA-payment-system PHASE 1
  references this PHASE for marketing-related consent;
  service-provision consent typically rests on
  contractual lawful basis
- **PCI DSS scope**: payment data inventory carries
  PCI DSS scope tags so segregation is enforceable
- **AML / sanctions**: legitimate-interests basis with
  documented safeguards; subject objection refused with
  legal-obligation override

## §4 Marketing-automation integration

For deployments operating marketing-automation systems:

- **Consent-state propagation**: marketing-automation
  systems subscribe to webhook events
  (`consent-given` / `consent-withdrawn`) for marketing
  purposes; consent state syncs in near real-time
- **Suppression-list integration**: erasure-fulfilled
  subjects added to suppression list per
  WIA-identity-management PHASE 4 §6
- **Per-channel consent granularity**: email, SMS, push,
  postal each potentially separate consent records

## §5 Customer-data-platform (CDP) integration

For deployments operating CDPs:

- **Profile-derivation lineage**: every derived profile
  attribute references its source data and lawful basis
- **Profile freshness**: derivations stale beyond the
  retention period for source data are recomputed or
  decommissioned
- **Profile-sharing controls**: third-party sharing
  governed by per-profile consent or the CCPA
  do-not-sell flag

## §6 HRIS integration

For deployments handling employee data (HRIS as a
controller-side processing relationship):

- **Lawful basis**: typically `contract` for employment
  performance + `legal-obligation` for tax / labour law
- **Consent restrictions**: employment-context consent
  often invalid (power imbalance); deployments SHOULD
  prefer alternative bases
- **Retention by employment status**: per-jurisdiction
  retention requirements (e.g., 5 years post-separation
  KR labour law)

## §7 Regulator portal integration

For deployments with formal regulator submissions:

- **EU**: SA breach-notification portals (per Member State)
- **KR**: KISA breach-notification + 개인정보보호위원회
  reporting
- **US**: per-state Attorney General breach portals
- **CA**: California Privacy Protection Agency portal
- **JP**: 個人情報保護委員会 (PPC)
- **BR**: ANPD portal
- **CN**: CAC submission system
- **SG**: PDPC portal

The boundary records each portal's submission requirements,
authentication discipline, and acknowledgement format. A
submission's acknowledgement is itself audit-chained.

## §8 Breach-notification clearinghouses

Some jurisdictions (notably US states) have shared breach-
notification clearinghouses. The deployment's breach
record references the clearinghouse submission and the
clearinghouse's acknowledgement.

## §9 B2B subprocessor disclosure

For deployments that themselves are processors serving
multiple business customers:

- **Public sub-processor list**: published at a stable
  URL per customer-DPA convention
- **Customer notification**: per-customer webhook +
  email notice when sub-processor list changes
- **Objection window**: per-DPA, typically 30 days
- **Material-change definition**: declared in the DPA

## §10 Operational SLAs

| Concern                                 | Default SLA                         |
|-----------------------------------------|-------------------------------------|
| Consent-capture latency                 | ≤ 200 ms p95                        |
| Consent-withdrawal propagation          | ≤ 60 s to subscribed downstream     |
| DSR acknowledgement                     | per regime (e.g., GDPR ≤ 30 days)   |
| Breach detection → DPO notification      | ≤ 30 min                            |
| Breach detection → regulator submission  | per regime (GDPR 72h)               |
| RoPA export                             | ≤ 60 s for typical controller       |
| Webhook delivery                        | ≤ 10 s p95 to subscriber            |
| Audit-chain entry availability          | ≤ 10 s                              |

## §11 Quarterly compliance report

The boundary emits a quarterly compliance report:

- Active subject count by jurisdiction
- Consent give / withdraw rates by purpose
- DSR volume by request type and regime
- DSR fulfillment timeliness vs. regulatory deadlines
- DPIA status by triggering activity
- Breach incidents by kind / severity / state
- Cross-border-transfer count by mechanism
- Sub-processor list mutations
- Regulator submission count + acknowledgement rate
- Audit-chain integrity check results

For Anchored deployments, the report is regulator-witnessed.

## §12 Acceptance criteria

A deployment claims conformance when:

1. RoPA covers every active processing activity
2. Privacy notices reference active RoPA entries
3. Consents reference active notice versions
4. DSR fulfillment SLA met for the prior quarter
5. DPIAs current for high-risk activities
6. Breach-notification timeline met for any prior-quarter
   incident
7. Cross-border-transfer mechanisms valid for active
   transfers
8. Sub-processor public list current
9. Audit-chain integrity check passes for the prior
   quarter

## §13 Common pitfalls (informative)

- **Notice-version drift** — notice updates without
  consent-version pinning lead to "consent under unknown
  notice" disputes; always pin
- **Consent-withdrawal latency** — marketing systems that
  cache consent state cause complaints; subscribe to
  webhooks and refresh on every send
- **DSR queue backlog** — peak-period DSR floods exceed
  manual capacity; deployments SHOULD invest in DSR
  workflow automation early
- **Sub-processor list drift** — listing a vendor as
  current after they're decommissioned breaches
  Article 28 transparency
- **Cross-border mechanism expiry** — SCC 2010/87 deprecated
  2022-12-27 caught many deployments; track expiry actively
- **Identity-vault hardening gap** — pseudonym ↔ PII vault
  is the single highest-value target; harden disproportionately
- **Lawful-basis-shopping** — switching basis to avoid
  consent-withdrawal effects is a regulator red flag;
  document switches with strong rationale
- **Children's data identification gap** — under-N rules
  vary (US 13, KR 14, EU 16 baseline / 13-16 per Member
  State, BR 18); detection-based gating preferred over
  reliance on declarations
- **Sensitive-data inference gap** — derived sensitive
  data (e.g., health from purchase history) often missed
  in inventory; CDP integration MUST surface inferred
  sensitivity

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Cross-domain reference table

| Reference                  | Use site                                                |
|----------------------------|---------------------------------------------------------|
| WIA-identity-management    | subject + DPO authentication, FIDO2 step-up             |
| WIA-medical-data-privacy   | clinical-domain consent inheritance                     |
| WIA-medication-adherence   | medication-domain consent inheritance                   |
| WIA-medical-imaging        | imaging-domain consent inheritance                      |
| WIA-payment-system         | payment-domain marketing consent                        |
| WIA-network-security       | security incident → breach-notification escalation       |
| WIA-pq-crypto              | post-quantum migration phase                            |
| WIA-supply-chain           | sub-processor BAA / DPA tracking                         |

## Annex B — Decommissioning checklist (informative)

When a privacy-control deployment winds down:

- [ ] All active consents migrated to successor or withdrawn
- [ ] All open DSRs handed off to successor or completed
- [ ] All open breach records handed off
- [ ] Final RoPA snapshot archived under retention policy
- [ ] Sub-processors notified
- [ ] Customer-tenants notified (B2B)
- [ ] Regulator notifications per regime
- [ ] Audit chain sealed and final root published

## Annex C — Conformance disclosure

Sections §1, §10, §11, §12 are mandatory.
§2 (EHR) is mandatory for deployments with clinical
relationships. §3 (payment) is mandatory for those with
payment-data scope. §4-6 mandatory per the deployment's
scope. §7 (regulator portals) is mandatory for any
deployment subject to formal regulatory submission. §9
(B2B subprocessor) is mandatory for processor-role
deployments.

## Annex D — Worked DPIA trigger conditions (informative)

GDPR Art. 35 triggers (jurisdictional variations apply):

- Systematic + extensive evaluation including profiling
  with significant effects
- Large-scale processing of special-category or criminal
  data
- Systematic monitoring of public area on a large scale
- DPA's published "blacklist" criteria

K-PIPA §33 영향평가 trigger:

- 5만명 이상 민감정보 / 고유식별정보 처리
- 50만명 이상 정보주체 처리 + 다른 시스템 연계
- 100만명 이상 정보주체 처리

CCPA / CPRA does not require DPIA but the deployment may
voluntarily produce one to address proportionality.

## Annex E — Multi-language notice presentation

Deployments serving multiple jurisdictions present notices
in subject-preferred language:

- BCP 47 language tag detection per subject
- Per-language notice version pinning
- Translation provenance (who translated, against which
  source version)
- Subject-initiated language change preserves consent
  while updating presentation

## Annex F — DPO / CPO escalation chain

```
Operational privacy issue
  → Privacy office staff
  → DPO / CPO
  → Executive privacy committee
  → Board / regulator (depending on severity)
```

The boundary's escalation routes are configurable per the
deployment's governance policy. Routing decisions are
audit-chained.

## Annex G — Children's-data discipline

For deployments with under-N subjects:

- Age-gating at sign-up flow per jurisdiction (US 13,
  KR 14, EU 13-16 by Member State, BR 18 with parental
  consent for minors)
- Parental consent capture per regime (US COPPA verifiable
  parental consent methods; KR 법정대리인 동의)
- Reduced data collection by default for minors (data
  minimisation Art. 5(1)(c) applies even more strictly)
- Marketing restrictions (per regime)
- Enhanced erasure rights (Art. 17(1)(f))

The boundary's age-gating decisions are audit-chained.

## Annex H — Public attestation surface

Anchored deployments publish an attestation surface:

- Capability document (this PHASE §10)
- Sub-processor list (PHASE 1 §9)
- Public privacy notice
- Annual transparency report (DSR / breach / cross-border
  aggregate)
- Audit-chain root

Customers and regulators consume the surface as the
canonical conformance signal.
