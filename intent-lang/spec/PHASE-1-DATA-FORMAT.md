# WIA-intent-lang PHASE 1 — DATA-FORMAT Specification

**Standard:** WIA-intent-lang
**Phase:** 1 — DATA-FORMAT
**Version:** 1.0
**Status:** Stable

This document defines the canonical data-format layer for
WIA-intent-lang. The standard covers the persistent shapes that
flow through declarative-intent description, parsing, planning,
execution, evaluation, and audit. WIA-intent-lang is consumed
by application teams that author intents (high-level statements
of "what should happen"), by planning runtimes that translate
intents into concrete actions, by execution agents that perform
the actions, and by audit and observability platforms that
reconstruct what was intended versus what occurred.

References (CITATION-POLICY ALLOW only):

- ISO 8601 (date and time representation)
- ISO/IEC 11578 (UUID)
- ISO/IEC 14977:1996 (extended Backus-Naur form, EBNF — used to
  describe the canonical intent grammar)
- ISO/IEC 19505 (UML 2.5 superstructure — used as the reference
  metamodel for intent diagrams)
- ISO/IEC 27001:2022 (information security management)
- IETF RFC 4122 (UUID URN)
- IETF RFC 5234 (Augmented BNF for Syntax Specifications, ABNF)
- IETF RFC 8259 (JSON)
- IETF RFC 9457 (Problem Details)
- W3C XML 1.1 (used for legacy intent-set import only)
- W3C JSON-LD 1.1 (used for the optional semantic-overlay
  serialisation)
- W3C SHACL (used for intent validation against constraint
  shapes)
- OpenAPI Specification 3.1 (used by adapter manifests)

---

## §1 Scope

This PHASE defines persistent shapes for the artefacts produced
and consumed by an intent-language programme. Implementations
covered include:

- Intent authoring tools (visual diagramming editors, command-
  line authoring tools, IDE plug-ins).
- Intent compilers and validators that transform authored
  intents into the canonical interchange representation.
- Planners that materialise intents into action graphs.
- Execution agents that perform actions and emit observation
  records.
- Evaluation and audit platforms that compare observed outcomes
  against intended outcomes.

Generative-language model fine-tuning pipelines, model-weight
artefact catalogues, and runtime-cost optimisation engines are
addressed in adjacent WIA standards and are out of scope here.

## §2 Programme Identifier

```
programmeId          : string (uuidv7)
programmeOperator    : string (institutional identifier of the
                         operator that maintains the intent
                         registry)
programmeRegistered  : string (ISO 8601 / RFC 3339)
domainScopes         : array of enum ("infrastructure-ops" |
                         "data-pipeline" | "business-process" |
                         "supply-chain" | "robotics-mission" |
                         "creative-workflow" | "research-
                           experiment" | "user-defined")
intentLanguageMinor  : string (semantic-version minor that the
                         operator's compilers understand;
                         compilers MUST interpret intents
                         labelled at this minor or earlier)
programmeStatus      : enum ("draft" | "operating" |
                         "frozen" | "archived")
```

## §3 Intent Record

The intent is the canonical declarative artefact. It is a JSON
document conforming to the canonical schema, with optional
JSON-LD context for semantic overlays.

```
intent:
  intentId           : string (uuidv7)
  programmeId        : string (uuidv7)
  authoredAt         : string (ISO 8601 / RFC 3339)
  authorRef          : string (opaque token mapped in the
                         operator IDP to the human or service
                         author)
  title              : string (single-line human-readable
                         summary, ≤ 80 characters)
  goal               : string (multi-line declarative
                         statement of the desired outcome)
  preconditions      : array of Predicate
  postconditions     : array of Predicate
  invariants         : array of Predicate (predicates that
                         MUST hold throughout execution)
  acceptanceCriteria : array of Predicate (predicates that
                         confirm the intent's success after
                         execution)
  budgetEnvelope     : object (per-resource caps — e.g. wall-
                         clock, monetary, computational, energy,
                         carbon — with currency / unit codes)
  fallbackPolicy     : enum ("abort-and-rollback" |
                         "abort-no-rollback" |
                         "degrade-and-continue" |
                         "request-human-decision")
  intentStatus       : enum ("draft" | "approved" |
                         "scheduled" | "in-flight" |
                         "completed" | "failed" |
                         "withdrawn")
```

The operator's intent linter (PHASE-3 §4) checks every intent
for SHACL constraint conformance and for budget-envelope
plausibility before the intent advances past `draft`.

## §4 Predicate Record

Predicates are the units of state and outcome assertion. They
are expressed in a typed predicate language defined in PHASE-1
§5; the predicate record stores the parsed AST and the source
text.

```
predicate:
  predicateId        : string (uuidv7)
  intentId           : string (uuidv7)
  role               : enum ("precondition" |
                         "postcondition" | "invariant" |
                         "acceptance")
  sourceText         : string (UTF-8; the human-authored
                         predicate text)
  parsedAst          : object (the predicate's parsed AST
                         conforming to the grammar in §5)
  contextOverlayRef  : string (URI of the JSON-LD context
                         overlay; absent for predicates
                         expressed only in the canonical
                         language)
```

## §5 Predicate Grammar (ABNF Excerpt)

```
predicate     = atom / "(" predicate ")" / and-expr / or-expr /
                not-expr / quantifier
atom          = identifier 1*( "." identifier ) "(" args ")"
and-expr      = predicate 1*( WSP "AND" WSP predicate )
or-expr       = predicate 1*( WSP "OR" WSP predicate )
not-expr      = "NOT" WSP predicate
quantifier    = ( "FOR-ALL" / "EXISTS" ) WSP variable WSP
                "IN" WSP collection WSP ":" WSP predicate
identifier    = ALPHA *( ALPHA / DIGIT / "-" / "_" )
args          = [ value *( "," WSP value ) ]
value         = literal / variable / atom
literal       = string / number / boolean / iso-8601-instant
```

The canonical grammar is published as a versioned artefact
under `/spec/grammar/`; minor revisions remain backwards-
compatible with prior-minor compilers.

## §6 Plan Record

Planners materialise intents into action graphs. The plan
captures the action set, the dependencies, the assigned
execution agents, and the planner's confidence indicators.

```
plan:
  planId             : string (uuidv7)
  intentId           : string (uuidv7)
  plannerVersion     : string (planner identifier and version)
  plannedAt          : string (ISO 8601)
  actionGraph        : object (DAG of actions; node identifiers,
                         action kinds, parameters, success
                         predicates)
  estimatedBudget    : object (per-resource estimate with
                         confidence intervals)
  fallbackPlanRef    : string (URI of the fallback plan; absent
                         when no fallback is registered)
  planStatus         : enum ("draft" | "approved" |
                         "scheduled" | "executing" |
                         "completed" | "aborted" |
                         "superseded")
```

Plan revisions are append-only; superseded plans remain
addressable for retrospective comparison.

## §7 Execution Trace Record

```
executionTrace:
  traceId            : string (uuidv7)
  planId             : string (uuidv7)
  beganAt            : string (ISO 8601)
  endedAt            : string (ISO 8601; absent until ended)
  actorRef           : string (institutional identifier of the
                         execution agent)
  actionEvents       : array of ActionEvent
  budgetConsumed     : object (per-resource actuals)
  outcome            : enum ("succeeded" | "partially-met" |
                         "failed" | "aborted" |
                         "human-deferred")

ActionEvent:
  eventAt            : string (ISO 8601)
  actionId           : string (the DAG node identifier)
  eventKind          : enum ("started" | "succeeded" |
                         "failed" | "retried" | "skipped" |
                         "compensated")
  observationRef     : string (content-addressed URI of the
                         observation artefact, e.g. a structured
                         response, a log fragment, an image,
                         a measurement)
```

## §8 Evaluation Record

The evaluator compares the execution trace against the intent's
acceptance criteria and the planner's estimated budget. The
evaluation record carries the per-criterion verdict and the
overall verdict.

```
evaluation:
  evaluationId       : string (uuidv7)
  traceId            : string (uuidv7)
  evaluatedAt        : string (ISO 8601)
  perCriterionVerdict: array of object (criterion identifier,
                         predicate evaluated, verdict
                         {satisfied|partially-satisfied|
                          violated|inconclusive}, evidenceRefs)
  budgetVerdict      : enum ("within-envelope" |
                         "envelope-exceeded" |
                         "envelope-exceeded-with-rationale")
  overallVerdict     : enum ("intent-met" | "intent-partially-met"
                         | "intent-failed" |
                         "evaluation-inconclusive")
```

## §9 Observation Artefact Reference

Action events (PHASE-1 §7) carry an `observationRef` that
content-addresses the artefact the execution agent produced.
Programmes record the artefact-class metadata so that downstream
evaluators and auditors can interpret the artefact without
bespoke knowledge of the action.

```
observationArtefact:
  artefactId         : string (uuidv7)
  traceId            : string (uuidv7)
  actionId           : string (DAG node identifier)
  capturedAt         : string (ISO 8601 / RFC 3339)
  artefactKind       : enum ("structured-response-json" |
                         "log-fragment" | "image" | "audio" |
                         "video" | "measurement-series" |
                         "shell-transcript" | "model-output" |
                         "human-attestation" | "user-defined")
  artefactRef        : string (content-addressed URI of the
                         artefact bytes)
  artefactDigest     : string (SHA-256)
  artefactSizeBytes  : integer
  privacyClass       : enum ("public" | "operator-internal" |
                         "subject-restricted" |
                         "regulator-restricted")
  retentionWindow    : enum ("trace-only" | "intent-bound" |
                         "regulatory-required")
```

Artefact retention windows derive from the operating programme's
records-management policy and the per-domain-scope regulatory
expectations. Artefacts whose retention window is `trace-only`
are eligible for purge once the trace has reached terminal
status; artefacts that supported a regulator-attested process
retain per the regulator's required window.

## §10 Compensation Record

Action events with `eventKind=compensated` (PHASE-1 §7) record
the compensating action that the execution agent performed when
an action failed and the plan's fallback policy required a
rollback. The compensation record links the failed action and
the compensating action so that auditors can reconstruct the
unwind chain.

```
compensation:
  compensationId     : string (uuidv7)
  traceId            : string (uuidv7)
  failedActionId     : string (DAG node identifier of the
                         action that failed)
  compensatingActionId : string (DAG node identifier of the
                         compensating action; introduced into
                         the DAG by the planner's compensation
                         strategy)
  compensationOutcome: enum ("compensated" |
                         "compensation-failed" |
                         "manual-intervention-required")
  rationaleRef       : string (content-addressed URI of the
                         planner's compensation rationale; the
                         rationale text is consumed by audit
                         reviewers)
```

## §11 Review-and-Approve Record

Intents that exceed the operator's per-author authorisation
require a separate review and approval (PHASE-3 §18). The
review-and-approve record captures the workflow.

```
reviewAndApprove:
  recordId           : string (uuidv7)
  intentId           : string (uuidv7)
  reviewerRef        : string (opaque IDP token)
  reviewedAt         : string (ISO 8601)
  reviewVerdict      : enum ("approved-for-approval" |
                         "changes-requested" | "rejected")
  reviewNotesRef     : string (URI of the reviewer's notes)
  approverRef        : string (opaque IDP token; absent until
                         the approver acts)
  approvedAt         : string (ISO 8601; absent until approval)
  approvalVerdict    : enum ("approved" | "rejected")
  segregationOfDutiesAttested : boolean (true when the operator's
                         IDP confirmed reviewer != approver !=
                         author)
```

## §12 Replay Record

Replays (PHASE-3 §21) emit a separate trace plus a replay
record that links replay outcome to original outcome.

```
replay:
  replayId           : string (uuidv7)
  originalTraceId    : string (uuidv7)
  replayTraceId      : string (uuidv7)
  replayMode         : enum ("dry-run" | "full-execution")
  replayedAt         : string (ISO 8601)
  replayAuthorisedBy : string (opaque IDP token; required for
                         `full-execution` mode)
  outcomeDelta       : enum ("identical" | "deviated" |
                         "inconclusive")
```

## §13 Conformance

Implementations claiming PHASE-1 conformance emit each of the
records defined above for every intent processed and honour the
predicate-grammar rules in §5.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 1 — DATA-FORMAT
- **Status:** Stable
- **Standard:** WIA-intent-lang
- **Last Updated:** 2026-04-28
