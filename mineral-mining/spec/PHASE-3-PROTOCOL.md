# WIA-mineral-mining PHASE 3 — Protocol Specification

**Standard:** WIA-mineral-mining
**Phase:** 3 — Protocol
**Version:** 1.0
**Status:** Stable

This PHASE specifies the protocols binding the data format
(PHASE 1) to the API surface (PHASE 2): authentication of
operators, laboratories, competent persons, and regulators;
audit-chain construction; time discipline; QA/QC linkage
enforcement; OECD DDG status calculation; worker-privacy
contract; cryptographic signing; and post-quantum migration.

References (CITATION-POLICY ALLOW only):
- IETF RFC 8446 (TLS 1.3), RFC 7515 (JWS), RFC 7517 (JWK),
  RFC 9162 (Certificate Transparency 2.0)
- ISO 17025:2017 — laboratory accreditation
- OECD Due Diligence Guidance for Responsible Supply Chains
- ICMM Mining Principles
- WIA-pq-crypto PHASE 3 — for ML-KEM/ML-DSA migration profiles

---

## §1 Authentication

Operators, laboratories, competent persons, regulators, and
HSE officers authenticate using JWS-signed JWTs issued by the
deployment's identity authority. Token claims:

- `iss` — issuing authority URN
- `sub` — operator/system URN
- `aud` — boundary URN
- `iat`, `exp` — RFC 3339 with offset
- `wia.role` — one of the roles in PHASE 2 Annex H
- `wia.scope[]` — operation-class scopes
- `wia.cpId` — for competent-person tokens, the qualifying
  registration URN

Tokens are short-lived (typically 5 minutes for write tokens,
30 minutes for read-only). Long-lived credentials are
forbidden for any write-capable role.

## §2 Laboratory accreditation binding

Laboratory tokens carry `wia.iso17025Scopes[]` listing the
analyte/method combinations the laboratory is currently
accredited for. The token issuer (the deployment's identity
authority) refreshes the scope list from the accrediting
body's published register on a declared cadence; tokens
issued for an out-of-scope analyte/method are refused at
boundary intake. Accreditation suspension or withdrawal
propagates to outstanding tokens via the credentials
service's revocation surface.

## §3 Competent-person binding

Competent-person tokens carry the qualifying-registration
URN and the codes the person is qualified under (JORC,
NI 43-101, SAMREC, PERC). The boundary refuses an estimate
filing whose declared code is not in the competent person's
qualified list. Removal or suspension of a person's
qualification revokes outstanding tokens and propagates a
follow-up request to estimates that the person had signed
within the deployment's reconciliation window.

## §4 Audit chain

Every boundary state transition is appended to a Merkle
audit log:

- `entryId` — URN
- `parent` — prior `entryId` SHA-256
- `at` — RFC 3339 with offset
- `actor` — authority URN making the transition
- `kind` — closed enum: `asset-registered`, `asset-updated`,
  `closure-bond-mutated`, `exploration-received`,
  `estimate-filed`, `production-received`, `assay-received`,
  `qaqc-linked`, `environmental-received`, `exceedance-flagged`,
  `exposure-received`, `handover-received`, `incident-opened`,
  `incident-closed`, `regulator-witnessed`
- `payloadHash` — SHA-256 of the canonical JSON payload
- `signature` — JWS by the actor

Anchored deployments mirror the audit chain to a regulator-
trusted witness on a declared cadence; missing-mirror
escalations are publicly disclosed as part of the conformance
evidence package.

## §5 QA/QC linkage enforcement

Assay records (PHASE 1 §6) declare QA/QC packages
(blanks, duplicates, CRMs, pulp duplicates). The boundary
runs a reconciliation job:

- For each assay, verify the cited QA/QC package exists
- Compare CRM values against certified means within the
  declared confidence band
- Flag duplicate-pair variance against the laboratory's
  declared precision spec
- Mark the assay `validated` on full pass; `provisional` on
  pending; `rejected` on fail with a follow-up request to the
  laboratory and operator

QA/QC reconciliation results are themselves audit-chained.
Reconciliation is fully transparent: the deployment's QA/QC
acceptance criteria are published in the capability document
so partners and regulators can verify the regime in use.

## §6 Time discipline

All record timestamps use RFC 3339 with explicit offset.
Boundary clock is disciplined to a national-laboratory time
reference; drift outside declared bound triggers a
`boundary-clock-degraded` capability flag and tags subsequent
records `provisional` until recovered. Records carrying
local-time-without-offset are rejected at boundary intake.

## §7 Transport security

All endpoints require TLS 1.3 (RFC 8446) with a deployment-
declared cipher-suite list. Mutual TLS is required for
laboratory, regulator, and bulk-export endpoints. Certificate
revocation is published through the deployment's revocation
surface aligned with WIA-network-security PHASE 3.

## §8 Worker-privacy contract

Worker identifiers in PHASE 1 §8 exposure records are
deployment-issued pseudonyms, not direct PII. The binding
between pseudonym and the underlying HR identity is held in
a separate, access-controlled service ("HR vault"). Access
to the HR vault is restricted to:

- The worker (their own records)
- Authorised health-and-safety personnel under a signed
  consent record
- Statutory health authorities under a regulator
  counter-signature

The HR vault publishes audit logs of every access to a
secondary regulator-trusted witness; these logs are
themselves part of the conformance evidence package.

## §9 OECD DDG status calculation

The `oecdDdgStatus` field on handover records (PHASE 1 §9) is
computed by a declared rule set:

- `red` — open critical incident on the asset within the
  declared lookback window, or a regulator-issued
  prohibition; or upstream OECD-flagged red supplier
- `requires-additional-action` — open non-critical incident,
  or pending regulator review
- `green` — no open critical incidents and no pending
  regulator actions

The rule set is deployment-policy and is published in the
capability document. Changes to the rule set require a
regulator counter-signature and emit a `ddg-rules-mutated`
audit-chain entry.

## §10 Replay protection

Production POST and assay POST require `Idempotency-Key`;
the boundary stores keys for 30 days. Replays within that
window with the same body return the original response;
different bodies return `urn:wia:mm:problem:idempotency-conflict`.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Annex A — Cryptographic signature suite

Default signature algorithms are ECDSA P-256 with SHA-256 for
operator and lab tokens, and EdDSA (Ed25519) for audit-chain
entries. Deployments may declare alternative suites in the
capability document; partners verify suite compatibility on
initial connection.

## Annex B — Post-quantum migration

The standard supports a phased PQC migration aligned with
WIA-pq-crypto PHASE 3:

- Phase A — classical-only (current default)
- Phase B — hybrid (classical + ML-KEM, classical + ML-DSA)
- Phase C — PQ-only

Mining-operation timelines are typically multi-year; the
deployment declares its target migration phase per role
class in the capability document.

## Annex C — Audit-chain replication

Anchored deployments replicate audit-chain entries to a
regulator-trusted witness on a declared cadence (typically
hourly for high-criticality operations, daily for routine
production). Replication uses a Merkle batch envelope
witnessed by the regulator authority.

## Annex D — Cipher-suite floors

Endpoints accept only TLS 1.3 cipher suites with forward
secrecy. The cipher-suite floor is published in the
capability document; partners verify compatibility before
exchange.

## Annex E — Negative-test vectors for protocol layer

| Stimulus                                              | Expected outcome                              |
|-------------------------------------------------------|-----------------------------------------------|
| Lab token submitting analyte outside ISO 17025 scope  | 422 + lab-scope-mismatch                      |
| Competent-person token signing under non-qualified code | 422 + cp-not-qualified                       |
| Audit-chain entry with broken parent hash             | rejected at append; boundary alerts           |
| Time-tag without offset                               | 422 + time-discipline-violation               |
| HR vault access without consent record                | 403; logged to regulator-trusted witness      |

## Annex F — Algorithm registry

The deployment maintains an algorithm registry naming the
specific cipher and signature algorithms in use per role
class. The registry is published in the capability document
and tracked across PQ migration phases for partner
verification and regulator audit.

## Annex G — Boundary-clock health

The boundary publishes a clock-health record in the capability
document including the primary and secondary time sources,
the most recent successful sync, and the current drift
estimate. Partners verify clock health before accepting
time-sensitive products.

## Annex H — Worked QA/QC reconciliation

A worked example for a copper-assay batch:

1. Lab submits assays for ten samples plus a CRM, two
   blanks, and three duplicate pairs
2. Boundary verifies the CRM falls within the certified
   ±2σ window (deployment-declared)
3. Boundary verifies blanks fall below the laboratory's
   declared method detection limit
4. For each duplicate pair, boundary computes the relative
   percent difference (RPD) and compares against the
   laboratory's declared precision spec
5. On full pass: assays move to `validated`; on partial
   fail: affected assays move to `provisional` pending
   re-analysis
6. Reconciliation outcome is appended to the audit chain
   and reflected in the lab's reconciliation report at
   `/reconciliation` (PHASE 2 Annex G)

A persistent CRM bias triggers a follow-up to the lab and
the operator; the operator may elect to re-route work to a
backup laboratory until the issue is resolved.

## Annex I — Worker pseudonym lifecycle

Pseudonyms in the exposure record (PHASE 1 §8) follow this
lifecycle:

- Issued on worker engagement with the operation
- Stable across shifts and assets within the operation
- Re-issued (a new pseudonym) on a documented re-engagement
  after a long absence; the previous pseudonym is retained
  in the HR vault binding for continuity of medical history
- Retired on permanent separation; HR-vault binding is
  retained per the operation's records-retention policy

Cross-operation pseudonym portability is out of scope; a
worker engaged by multiple operators receives separate
pseudonyms per operation.

## Annex J — Negative-test vectors for laboratory binding

| Stimulus                                              | Expected outcome                              |
|-------------------------------------------------------|-----------------------------------------------|
| Lab submitting analyte after accreditation suspension | 422; token revoked                            |
| Lab submitting method outside declared scope          | 422 + lab-scope-mismatch                      |
| CRM result outside certified window                   | assays in batch flagged `provisional`         |
| Duplicate-pair RPD exceeding precision spec           | affected pair flagged for re-analysis         |
| Chain-of-custody URN unreachable                      | sample held for chain reconciliation          |

## Annex K — Regulator counter-signature lifecycle

Regulator counter-signatures (PHASE 2 Annex C) follow this
lifecycle:

- Mutation request emitted by the operator with all
  supporting evidence URNs
- Regulator reviews and either counter-signs (issuing a
  detached JWS) or refuses (issuing a refusal note)
- Boundary applies the mutation only on counter-signed
  acceptance
- Counter-signatures are themselves audit-chained at
  `kind=regulator-witnessed`
- Refusals are audit-chained at `kind=regulator-refusal`
  with the regulator's refusal note URN

A mutation that has been pending counter-signature beyond
the deployment-declared timeout (typically 30 days) is
elevated to regulator escalation; the operator and regulator
both receive notification.
