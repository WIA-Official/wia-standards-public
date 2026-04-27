# WIA-intent-lang PHASE 4 — INTEGRATION Specification

**Standard:** WIA-intent-lang
**Phase:** 4 — INTEGRATION
**Version:** 1.0
**Status:** Stable

This document defines how an intent-language programme
integrates with the systems that surround it: authoring tools
(IDE plug-ins, visual diagram editors, command-line authoring
tools), the operator's IDP for author identity, planner-runtime
catalogues, execution-agent catalogues, evaluator catalogues,
fallback-handler infrastructure (notification systems, on-call
rotas, ticketing systems), observability platforms, and
long-term archives that hold intent registries beyond programme
wind-down. It also defines the evidence-package format that
bundles a programme's intent record set for external audit.

References (CITATION-POLICY ALLOW only):

- IETF RFC 8259 / 9457 / 8615 / 8288 / 9421
- ISO/IEC 27001:2022 (information security management)
- ISO/IEC 17065:2012 (conformity-assessment bodies)
- ISO 8601 (date and time)
- W3C SHACL
- W3C JSON-LD 1.1
- W3C Verifiable Credentials Data Model 2.0 (optional)

---

## §1 Authoring Tool Integration

Authoring tools target the API at `/v1/intents` with the
authoring tool's client certificate; the certificate's subject
identifies the tool to the operator's audit trail. Authoring
tools fetch the active grammar and SHACL artefacts from the
endpoints in PHASE-2 §9 and pin the revisions they emit
against. Authoring tools that emit intents under a stale
grammar or SHACL pinning receive a refresh notification through
the streaming subscription.

## §2 Identity Provider Integration

The operator's IDP federates author identity. Authors obtain
opaque `authorRef` tokens by authenticating to the IDP; the
authoring tool exchanges the tokens for intent registry
authorisation through the operator's authorisation service.
IDP integrations support the AAL tiers required by the
operator's per-domain-scope authorisation matrix.

## §3 Planner Runtime Catalogue Integration

Planner runtime catalogues advertise qualified planners
(PHASE-3 §5). The integration carries the catalogue's
identifier, the per-planner qualification chain, the per-domain-
scope coverage, and the catalogue's notification endpoint for
planner deprecations.

## §4 Execution Agent Catalogue Integration

Execution agent catalogues advertise qualified agents
(PHASE-3 §6). The integration carries the catalogue's
identifier, the per-agent action-kind coverage, the per-agent
operational-region binding, and the catalogue's notification
endpoint for agent deprecations.

## §5 Evaluator Catalogue Integration

Evaluator catalogues advertise qualified evaluators (PHASE-3
§7). The integration carries the catalogue's identifier and
the per-evaluator methodology coverage so that intents can
select appropriate evaluators per acceptance criterion.

## §6 Fallback Handler Integration

Fallback handlers receive escalations from intents whose
`fallbackPolicy` requires `request-human-decision` or whose
evaluation outcome is `intent-failed`. The integration record
carries the handler's identifier (notification system,
ticketing queue, on-call rota), the per-domain-scope routing,
and the escalation timeout per fallback policy.

## §7 Observability Platform Integration

Observability platforms consume execution traces (PHASE-1 §7)
and emit per-intent dashboards for the operator's analytics
team. The integration is read-only; the observability platform
does not write back into the intent registry. Observability
queries that exceed the operator's per-tenant rate limit are
throttled.

## §8 Evidence Package Format

```
evidence/
  manifest.json                 — package manifest (signed, see §9)
  programme.json                — programme record
  grammar/                      — grammar revisions in force at
                                   the cited interval
  shacl/                        — SHACL revisions in force at
                                   the cited interval
  intents/                      — per-intent records and
                                   predicates
  plans/                        — per-intent plan history
  execution-traces/             — per-plan trace and action
                                   events
  evaluations/                  — per-trace evaluations
  fallback-decisions/           — fallback-handler decisions
                                   recorded against the intent
  audit/                        — API audit log excerpts
```

The package is content-addressable; the manifest is signed by
the operator's HTTP-message-signature key (RFC 9421) and
counter-signed by the programme's quality manager when the
package supports an external audit.

## §9 Manifest and Signatures

Verification tools recompute file digests, compare to the
manifest, and reject the package on mismatch with type
`urn:wia:intent-lang:evidence-mismatch`.

## §10 well-known URI Discovery

A conformant programme exposes a discovery document at
`/.well-known/wia-intent-lang` that links to the API root, the
grammar / SHACL endpoints, the qualified runtime register, and
the published quality dossier.

## §11 Long-Term Archive Integration

Programmes designate a long-term archive that holds intents
and traces beyond programme wind-down. Quarterly deposits
round-trip content-addresses; on wind-down, remaining records
transfer to the archive with content-addresses preserved.

## §12 Cross-Standard Linkage

Programmes that consume adjacent WIA standards (workflow-
orchestration, data-lineage, ai-agent-governance) emit cross-
standard linkage records that name the consuming standard and
the version under which the linkage is claimed.

## §13 Verifiable-Credential Re-Issuance (optional)

Operators that wish to expose attestations (planner-runtime
qualification, execution-agent qualification, ISO/IEC 27001
certification) to consumers of W3C Verifiable Credentials MAY
re-issue the attestations as Verifiable Credentials under the
Data Model 2.0 specification. Re-issuance is optional; the
canonical record remains the JSON evidence-package manifest.

## §14 Streaming Heartbeat

SSE subscribers receive a heartbeat every 30 seconds with
`Last-Event-ID` resume support; subscribers that disconnect
during long-running execution traces resume from the last seen
event identifier without losing visibility of priority-1
events.

## §15 Backwards-Compatibility Guarantee

PHASE-4 minor revisions remain backwards-compatible with prior-
minor clients. Major revisions go through a deprecation window
of at least one full grammar major-version release cycle so
that authoring tools and runtimes have time to migrate.

## §16 Migration from Pre-Standard Records

Programmes that operated before WIA-intent-lang reached version
1.0 MAY migrate historical intent registries by emitting
synthetic intent records with a `legacyImport` flag. Synthetic
intents are accepted by the public catalogue but are not
eligible for evidence-package generation without contemporaneous
re-validation under the in-force SHACL revision.

## §17 Reader Tooling

Programmes MAY publish supplementary reader tools (visual
intent timelines, predicate-evaluation explanations,
budget-vs-actuals charts, fallback-decision diaries) alongside
the canonical evidence package; the tools are non-normative.

## §18 Public Catalogue and Aggregator Feeds

Programmes that publish a public catalogue of representative
intents (for community learning, benchmarking, or research)
emit an Atom or JSON Feed listing intents with their evidence-
package manifest digests, the goal text, the domain scope, and
the overall verdict at the time of publication. The feed
respects the operator's privacy and confidentiality policy.

## §19 Compensation-Strategy Catalogue Integration

Compensation-strategy catalogues advertise per-action-kind
compensation algorithms (PHASE-3 §15). The integration record
carries the catalogue's identifier, the per-strategy
applicability matrix (which planner-runtimes and execution-
agent versions can honour the strategy), and the catalogue's
notification endpoint for strategy deprecations.

## §20 Audit-Reviewer Workflow Integration

External audit reviewers (programme auditors, regulator
inspectors) consume audit-trail exports through dedicated
client certificates. The export carries the intent / plan /
trace / evaluation / compensation chain for the audit window
the auditor was commissioned for. Audit reviewers do not
write back into the intent registry; remediation actions
flow back through the operator's quality-management system.

## §21 Quality-Aggregate Endpoint Integration

External quality-benchmarking services that compare intent-
language programmes consume aggregate endpoints (PHASE-2 §18)
and publish per-domain success-rate, budget-overrun-rate, and
fallback-trigger-rate league tables. The integration is
read-only and is rate-limited; benchmark service certificates
are bound to the operator's terms-of-use that the benchmark
operator has agreed.

## §22 Federated Programme Operation

Operators that run multiple intent registries (per business
unit, per regulatory jurisdiction, per technology stack)
integrate through a federation adapter that translates between
per-registry WIA records and a combined view. The adapter
honours each registry's domain-scope authorisation matrix so
that federated queries respect the source registry's safety
constraints.

## §23 Cross-Programme Federation Broker Integration

Operators that participate in cross-programme intent federation
(PHASE-3 §20) integrate with a federation broker that routes
action subgraphs between partner programmes. The integration
record carries the broker's identifier, the partner-programme
catalogue, the per-partner authorisation matrix, and the
trace-reconciliation protocol that the broker honours. Brokers
are typically operated by a programme consortium and are
themselves audited under the operator's outsourcing policy.

## §24 Workflow Hand-Off to Adjacent Standards

Programmes that hand off completed intents to adjacent
WIA-orchestration or WIA-data-lineage programmes integrate
through a hand-off adapter that translates the intent /
plan / trace / evaluation set into the adjacent programme's
record vocabulary. The adapter preserves content-addresses so
that downstream consumers can resolve back to the originating
intent registry without ambiguity.

## §25 Conformance and Sunset

A programme conformant with PHASE-4 has integrated successfully
with at least one authoring tool, at least one IDP, the
qualified planner / agent / evaluator catalogues, the operator's
fallback-handler infrastructure, at least one observability
platform, and at least one long-term archive, and has published
at least one externally citable evidence package.

Sunsetting an integration is announced via the well-known
discovery document at least 90 calendar days before removal.

---

**Document Information:**

- **Version:** 1.0
- **Phase:** 4 — INTEGRATION
- **Status:** Stable
- **Standard:** WIA-intent-lang
- **Last Updated:** 2026-04-28
