# PHASE 1 — Data Format

> Roadmap canonical envelopes: milestone, dependency, release,
> progress update, and commitment change. All envelopes are signed
> with Ed25519 over the canonical JSON form (RFC 8785 JCS).

## 1.1 Milestone envelope

The `milestone` envelope is the smallest unit of public commitment
in this standard. It carries a project identifier, a name, a
planned date, a current confidence level, and a scope summary —
the same five fields that any narrative roadmap implicitly
contains, but expressed in a form that can be verified six months
later without a screenshot.

```
{
  "wia_roadmap_version": "1.0.0",
  "type": "milestone",
  "milestone_id": "ULID",
  "name": "Post-quantum cryptographic readiness",
  "project_id": "did:wia:project:...",
  "owner": "did:wia:org:...",
  "planned_date": "2026-09-30",
  "confidence_pct": 65,
  "scope_summary": "Hybrid Kyber-768 / X25519 negotiation in TLS 1.3 stack",
  "tags": ["security", "pqc", "tls"],
  "external_links": [],
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

`confidence_pct` is the publishing organisation's calibrated belief
that the milestone will be delivered by `planned_date`. The number
is not a marketing figure; it is a forecast that the organisation
will be measured against. Organisations that habitually publish
optimistic confidence and then slip will accumulate a verifiable
track record of optimism, which downstream consumers can weight
in their own planning.

## 1.2 Dependency envelope

```
{
  "wia_roadmap_version": "1.0.0",
  "type": "dependency",
  "dependency_id": "ULID",
  "milestone_id": "ULID",
  "depends_on": "ULID (own or external milestone)",
  "depends_on_owner": "did:wia:org:... (if external)",
  "criticality": "hard" | "soft",
  "rationale": "free text",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

A `hard` dependency means the milestone cannot be delivered
without the dependency being delivered first; a `soft` dependency
indicates a preference that allows degraded delivery if the
dependency slips. The standard is deliberately conservative on
external dependencies: declaring an external dependency is a
public statement that one organisation is committing to consume
another organisation's work, which is itself a form of
accountability.

## 1.3 Release envelope

```
{
  "wia_roadmap_version": "1.0.0",
  "type": "release",
  "release_id": "ULID",
  "milestone_id": "ULID",
  "version": "semver",
  "released_at": "RFC 3339",
  "evidence_refs": [
    "git:sha-of-release-tag",
    "doi:10.../...",
    "https://..."
  ],
  "scope_delivered": "free text",
  "scope_deferred":  "free text",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

The `scope_delivered` and `scope_deferred` split is mandatory.
"Released" is rarely binary in real roadmaps; the release envelope
captures both what the milestone delivered and what was deliberately
moved out of the milestone, so that downstream consumers can
evaluate the actual versus planned scope without re-reading
narrative release notes.

## 1.4 Progress update envelope

Progress updates are emitted on a cadence chosen by the publishing
organisation, typically weekly or bi-weekly during active
development. They are not a substitute for the release envelope;
they are a continuous signal that allows downstream consumers to
compare current state against `planned_date` and `confidence_pct`.

```
{
  "wia_roadmap_version": "1.0.0",
  "type": "progress_update",
  "update_id": "ULID",
  "milestone_id": "ULID",
  "captured_at": "RFC 3339",
  "completion_pct": 0,
  "blockers": [
    {
      "blocker_id": "ULID",
      "kind": "external_dependency_slip" | "scope_clarification"
            | "resource_unavailable" | "technical_blocker",
      "summary": "free text"
    }
  ],
  "confidence_pct_revised": 65,
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

`completion_pct` is intentionally a coarse measure (intervals of
five or ten are common in practice). Fine-grained completion
percentages are signal-poor and tend to encourage gaming.

## 1.5 Commitment change envelope

The commitment change envelope is the most sensitive envelope in
the standard. It is the public acknowledgement that a previously
published milestone has slipped, changed scope, or been dropped.

```
{
  "wia_roadmap_version": "1.0.0",
  "type": "commitment_change",
  "change_id": "ULID",
  "milestone_id": "ULID",
  "kind": "slip" | "scope_change" | "drop" | "split" | "merge",
  "from_planned_date": "2026-09-30",
  "to_planned_date":   "2026-12-15",
  "from_scope_summary": "...",
  "to_scope_summary":   "...",
  "rationale": "free text",
  "approved_by": "did:wia:officer:...",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

The `rationale` field is mandatory and is intended to be readable
by stakeholders six months after the change. "Resource constraints"
is not a useful rationale; "engineering team prioritised regulatory
compliance work in Q3" is. The standard does not enforce rationale
quality, but the public, signed nature of the rationale creates
soft pressure toward honesty.

## 1.6 Milestone confidence calibration

Organisations that publish milestones SHOULD periodically publish
a `calibration_report` envelope summarising historical confidence
versus actual delivery for the past 24 months. The report is a
form of intellectual honesty: an organisation that has historically
overshot its own confidence numbers is acknowledging the bias
publicly so that consumers can weight it appropriately.

## 1.7 Identifiers

- `milestone_id`, `release_id`, `update_id`, `change_id`,
  `dependency_id` are ULIDs (Crockford base32, monotonically
  sortable, unique within the publishing organisation's namespace).
- `project_id` and `owner` use the DID Core (W3C DID Core 1.0)
  format; the specific DID method is the publisher's choice.
- `planned_date` and `to_planned_date` use ISO 8601 calendar dates;
  events in flight use RFC 3339 timestamps.

## 1.8 References

The Phase 1 envelopes are intentionally aligned with:

- **W3C DID Core 1.0** for identity references
- **RFC 8785** (JCS) for canonical JSON
- **RFC 8949** (CBOR) for optional alternative encoding
- **ISO 8601** / **RFC 3339** for time
- **SemVer 2.0.0** for the `version` field of releases

## 1.9 Worked example: a milestone life cycle

The following sequence illustrates the typical envelope flow for a
single milestone from initial commitment to delivery.

```
T+0  milestone           {confidence_pct: 70, planned: 2026-09-30}
T+30 progress_update     {completion: 15}
T+60 progress_update     {completion: 30, blockers: []}
T+90 progress_update     {completion: 40, blockers: [external dep]}
T+95 commitment_change   {kind: slip, to_planned: 2026-11-15,
                          rationale: external dep slip}
T+120 progress_update    {completion: 60}
T+150 progress_update    {completion: 85}
T+170 release            {version: 1.0.0,
                          scope_delivered: ...,
                          scope_deferred: small subset to v1.1}
```

Each envelope is signed by the same publishing organisation;
together they form the immutable history of this milestone. A
consumer asking "did the organisation honour its original 2026-09-30
commitment?" can answer the question from the envelope stream
alone, without depending on the organisation's narrative blog post
about the slip.

## 1.10 Tagging discipline

The `tags` field is unbounded but conventional tags emerge over
time within communities of practice (security, accessibility,
internationalisation, regulatory, performance, reliability). The
standard does not enforce a tag taxonomy but does recommend that
tag identifiers be lower-case, hyphen-separated, and stable across
roadmap versions to support cross-organisation tag-based queries.

## 1.11 External link discipline

`external_links` may include any URL the publishing organisation
considers authoritative for the milestone — design documents,
public discussion threads, regulatory references. Links are not
fetched at envelope-validation time; they are advisory pointers
for human consumers. Link rot is the publisher's responsibility,
not the standard's.

## 1.12 Multi-organisation milestone aggregation

When several organisations publish related milestones (e.g., browser
vendors all committing to the same web-platform feature), an
aggregator may publish a `milestone_aggregate` envelope referencing
the underlying milestones by identifier. The aggregate carries no
new commitment; it is a curated view that downstream consumers
(developer-facing dashboards, journalists, regulators) can subscribe
to instead of subscribing to every upstream publisher individually.

```
{
  "wia_roadmap_version": "1.0.0",
  "type": "milestone_aggregate",
  "aggregate_id": "ULID",
  "aggregator_id": "did:wia:org:web-platform-tests",
  "summary": "WCAG 2.2 AA across major browsers",
  "components": [
    { "milestone_id": "...", "owner": "did:wia:org:chromium" },
    { "milestone_id": "...", "owner": "did:wia:org:gecko" },
    { "milestone_id": "...", "owner": "did:wia:org:webkit" }
  ],
  "captured_at": "RFC 3339",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

The aggregator's signature establishes accountability for the
curation; the underlying organisations remain accountable for their
own milestones. A consumer that distrusts the aggregator can still
verify each underlying milestone independently.

## 1.13 Milestone tier discipline

Organisations operating large roadmaps SHOULD adopt a tier
discipline so that downstream consumers can filter by importance
without re-reading every milestone:

- **strategic** — board-level commitment, typically 1–3 per year
- **major** — leadership-level commitment, typically 10–30 per year
- **minor** — team-level commitment, typically 100+ per year

The tier appears as a top-level `tier` field on the milestone
envelope. Tiers are advisory; the standard does not enforce a
specific count per tier, but the organisation that publishes 100
"strategic" milestones in a year has effectively rendered the
tier meaningless to its consumers.

## 1.14 Localisation

Milestone names, scope summaries, and rationales MAY be
localised. The standard supports localisation via parallel
`_i18n` fields:

```
{
  "name": "Post-quantum cryptographic readiness",
  "name_i18n": {
    "ko": "양자내성 암호화 준비",
    "ja": "ポスト量子暗号への対応",
    "zh": "后量子密码就绪"
  }
}
```

The canonical signed form contains the localisations as part of
the envelope; signature verification is over the full envelope
including the `_i18n` block, so localisations cannot be
retroactively edited without invalidating the signature.

## 1.15 Audit and tamper evidence

Every envelope's signature covers the canonical JSON form
excluding the signature itself. The publisher SHOULD maintain an
append-only Merkle log of all emitted envelopes and publish the
log root daily via a `daily_attestation_root` envelope so that
consumers can detect retroactive modification of historical
envelopes. The Merkle discipline is what allows a regulator
examining a six-year-old milestone to verify the milestone existed
in that form on its emission date without requiring the regulator
to have captured the envelope at emission time.

## 1.16 Investor-facing extension

For organisations subject to material-disclosure obligations
(public companies, regulated entities), milestones tagged
`investor-material` carry an additional `investor_material_block`
sub-envelope binding the milestone to the disclosure framework
applicable in the publisher's jurisdiction (US SEC 8-K item 7.01,
EU MAR Article 17, KR 자본시장법 정보공시, JP 適時開示). The
binding does not constitute filing — the publisher remains
responsible for actually filing — but it provides a machine-
readable trail from the operational milestone to the disclosure
event.

## Operational coda

Adoption of this standard is incremental: an organisation can
begin by publishing a single milestone envelope and incrementally
expand to dependencies, releases, progress updates, commitment
changes, and federation. The standard is intentionally additive.
There is no minimum subset of envelope types that must be emitted
to participate; the only requirement is that the envelopes that
are emitted be properly signed and structurally correct.

The cumulative behaviour of all such organisations is a
roadmap commons that no single party owns and that downstream
consumers can analyse without negotiating bilateral access. That
commons is the public-good outcome the standard exists to enable.

弘益人間 — Benefit All Humanity. The roadmap is a public document by
intention and a verifiable document by construction.
