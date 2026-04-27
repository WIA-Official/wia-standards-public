# WIA-intent-lang PHASE 3 — PROTOCOL Specification

**Standard:** WIA-intent-lang
**Phase:** 3 — PROTOCOL
**Version:** 1.0
**Status:** Stable

This document defines the protocols that govern an
intent-language programme: grammar versioning and stewardship,
SHACL constraint shape stewardship, intent-author onboarding,
intent linting and approval workflow, planner-runtime
qualification, execution-agent qualification, evaluator
qualification, fallback-handler discipline, recordkeeping, and
programme wind-down.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time)
- ISO 9001:2015 (quality management systems)
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 27701:2019 (privacy information management)
- ISO/IEC 25010:2011 (systems and software quality models;
  cited where intent acceptance criteria reuse the standard's
  quality characteristics)
- IETF RFC 5905 (NTPv4)
- IETF RFC 8446 (TLS 1.3)
- IETF RFC 9457 (Problem Details)
- W3C SHACL
- W3C JSON-LD 1.1

---

## §1 Programme Registration

An operator MAY claim conformance to WIA-intent-lang only after
publishing an intent registry that the operator's authoring
tools and runtimes target. The registry's domain scopes
(PHASE-1 §2) are recorded against the programme; intents whose
domain does not overlap any of the programme's scopes are
rejected at the linter stage.

## §2 Grammar Stewardship

The canonical predicate grammar (PHASE-1 §5) is versioned by a
working group named in the operator's quality dossier. Minor
revisions remain backwards-compatible; major revisions go
through a deprecation window of at least 12 calendar months
before authoring tools are required to migrate.

Grammar revisions are content-addressed and exposed through the
grammar endpoint (PHASE-2 §9). Authoring tools pin the grammar
revision they emit against; intents that reference an unpinned
grammar revision are rejected by the linter.

## §3 SHACL Stewardship

SHACL constraint shapes encode the intent registry's policy:
the budget-envelope plausibility rules, the per-domain-scope
predicate vocabularies, and the operator's safety constraints
(e.g. forbidding intents whose acceptance criteria depend on
non-deterministic external services without a fallback policy).

SHACL revisions follow the same versioning rules as the
grammar. Linter failures (PHASE-2 §4) cite the failing
constraint paths so that authors can correct the intent without
a round-trip with the SHACL stewardship working group.

## §4 Intent Linting and Approval

Intents move from `draft` to `approved` through the linter
gateway. The linter performs:

- syntactic validation against the pinned grammar;
- SHACL validation against the pinned SHACL revision;
- budget-envelope plausibility checking against the operator's
  per-domain-scope norms;
- duplicate-intent detection against the recent intent
  history;
- author-authorisation check against the operator's IDP.

Intents that pass the linter advance to `approved`; intents
that fail receive a Problem-Details response with the
constraint paths in question.

## §5 Planner Runtime Qualification

Planner runtimes are qualified per programme. Qualification
requires:

- demonstrated capability to materialise representative
  intents from each domain scope without manual intervention;
- documented behaviour for fallback policies (PHASE-1 §3);
- alignment with the operator's plan-recordkeeping rules;
- conformance attestation from the operator's planner-runtime
  certifying body.

Planners that lose qualification are removed from the registry;
intents that referenced the removed planner are re-planned
through a successor planner before execution.

## §6 Execution Agent Qualification

Execution agents are qualified per action kind that the
operator's intent vocabulary names. Qualification requires:

- documented action contracts (parameter shapes, success
  predicates, retry semantics, idempotency guarantees);
- conformance attestation from the operator's execution-agent
  certifying body;
- per-agent IDP credentials that bind the agent to the action
  kinds it is authorised to perform.

Execution events emitted by an unqualified agent are rejected
at the trace endpoint with type
`urn:wia:intent-lang:agent-not-qualified`.

## §7 Evaluator Qualification

Evaluators that score traces against acceptance criteria are
qualified per evaluation methodology. Qualification covers the
evaluator's deterministic behaviour, the per-criterion
explainability of the evaluator's verdicts, and the evaluator's
treatment of inconclusive cases.

## §8 Fallback Handler Discipline

Intents whose `fallbackPolicy` requires `request-human-decision`
escalate to a human operator through the operator's notification
infrastructure; the human's decision is recorded against the
intent and is treated as part of the audit chain. Programmes
configure an escalation timeout per fallback policy and notify
the intent author when the timeout elapses.

## §9 Recordkeeping

Programme records — every record defined in PHASE-1, the API
audit logs, the grammar and SHACL revisions in force at the
time of intent processing, the fallback-handler decisions, and
the conformance attestations of qualified runtimes / agents /
evaluators — retain per the operator's recordkeeping policy.
Intents that supported regulator-attested processes (financial
reporting, healthcare workflows, public-safety operations)
retain indefinitely.

## §10 Time Synchronisation

Programme clocks synchronise per RFC 5905 (NTPv4) so that
intent / plan / trace / evaluation timestamps are consistent
across the programme's runtime fleet.

## §11 Privacy

Intent author identity is held in the operator's IDP, never in
the intent registry; the intent record's `authorRef` is an
opaque token. Intent goal text and predicate source text MAY
contain references that, in some operating contexts, qualify
as personal data; the operator's data-protection policy
records the per-domain-scope handling (de-identification,
access controls, retention).

## §12 Quality Dossier

The operator's quality dossier records the grammar and SHACL
stewardship working groups, the planner / agent / evaluator
certifying bodies, the qualified runtime / agent / evaluator
register, the programme's incident history, and the operator's
remediation actions.

## §13 Programme Freeze and Wind-Down

Programmes that freeze (no new intents accepted, in-flight
intents drained) record the freeze rationale and the expected
wind-down date. Wind-down archives the intent registry to the
operator's long-term archive (PHASE-4 §10) and notifies
authoring tools so that they can redirect new authoring to a
successor programme where one exists.

## §14 Observation-Artefact Privacy and Retention

Observation artefacts (PHASE-1 §9) are governed by the
operator's privacy-class matrix:

- `public` artefacts MAY be shared without restriction.
- `operator-internal` artefacts are restricted to the operator's
  audit and analytics consumers.
- `subject-restricted` artefacts contain personal data and are
  gated by the operator's data-protection workflow; subject-
  access requests are honoured through the operator's CRM.
- `regulator-restricted` artefacts are released only to the
  regulator's authorised consumer or under court order.

Programmes record the per-class retention window in the quality
dossier. Retention windows that elapse trigger eligibility for
purge; purge events are recorded against the artefact so that
downstream audit reviews can see that a once-existing artefact
has been intentionally removed.

## §15 Compensation and Idempotency Discipline

Compensations (PHASE-1 §10) are subject to the operator's
compensation-strategy register. Each action kind that is
eligible for compensation declares:

- the compensation algorithm (saga-style compensating
  transaction, semantic rollback, manual-intervention escalation);
- the compensation timeout beyond which manual intervention is
  required;
- the idempotency key strategy that the compensation respects so
  that a partially-applied compensation can be safely retried.

Compensations whose action kind has no registered strategy
escalate to the operator's manual-intervention queue and are
recorded as `manual-intervention-required`.

## §16 Per-Domain-Scope Safety Constraints

Domain scopes (PHASE-1 §2) carry per-scope safety constraints
that the operator's SHACL stewardship working group encodes.
Examples:

- `infrastructure-ops` domain constraints forbid intents that
  delete resources without an explicit recovery path.
- `business-process` domain constraints require intents whose
  acceptance criteria depend on financial reconciliation to
  emit explicit reconciliation-evidence references.
- `robotics-mission` domain constraints require intents whose
  actions move physical assets to declare bounding-box and
  collision-budget envelopes.

## §17 Author Onboarding and Authorisation

Authors are onboarded through the operator's IDP. Onboarding
records the author's role (intent author, intent reviewer,
intent approver, fallback handler) and the per-domain-scope
authorisation that the role grants. Authors that lose role
authorisation (departure, role change, training expiry) are
demoted to read-only access; in-flight intents authored by a
demoted author retain their original `authorRef` for audit
continuity but cannot be advanced past their current status by
the demoted author.

## §18 Reviewer-and-Approver Discipline

Intents whose domain scope or budget envelope exceeds the
operator's per-author authorisation require a separate reviewer
and approver. The reviewer-and-approver workflow is recorded
against the intent so that audit reviewers can confirm that
high-risk intents went through the operator's elevated-risk
approval process before reaching `approved` status. Reviewers
and approvers are subject to the operator's segregation-of-
duties rules: the same author cannot be reviewer or approver
of their own intent.

## §19 Plan Re-Planning Discipline

When an in-flight intent encounters a planner failure (for
example a runtime that raises an irrecoverable error mid-
execution), the intent's plan may be superseded by a fresh
plan from a successor planner. Re-planning preserves the
intent's identity and acceptance criteria; the trace records
the planner transition so that audit reviewers can correlate
pre-transition and post-transition behaviour against the
single intent.

## §20 Cross-Programme Intent Federation

Operators that participate in cross-programme intent federation
(intents whose action graphs span multiple operators' execution
agents) follow a federation protocol: the originating
operator's planner publishes the action graph to the federation
broker, partner operators receive the action subgraphs they
own, and the originating operator reconciles trace events from
each partner into the unified trace. Federation participation
is recorded against the programme; audit reviewers can resolve
the unified trace to its constituent partner traces.

## §21 Reproducibility and Replay Discipline

Programmes that support replay (re-executing a closed intent
against the same inputs to verify reproducibility) record the
replay parameters separately from the original trace so that
audit reviewers can compare replay outcomes against the
original outcome without confusion. Replays of intents whose
actions have side effects (production-system mutations,
financial movements) follow the operator's replay-safety
policy: dry-run mode by default, full-execution replay only
under explicit operator authorisation.

## §22 Conformance and Auditing

A programme conformant with WIA-intent-lang publishes its
grammar and SHACL stewardship references, its qualified runtime
register, its quality dossier, and the catalogue of intent
domain scopes it supports, and answers an annual self-
assessment that maps each clause of this PHASE to the
programme's implementation.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 3 — PROTOCOL
- **Status:** Stable
- **Standard:** WIA-intent-lang
- **Last Updated:** 2026-04-28
