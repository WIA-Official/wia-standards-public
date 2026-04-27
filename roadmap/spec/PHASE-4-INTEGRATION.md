# PHASE 4 — Integration

> Integration with issue trackers (GitHub, GitLab, Linear, Jira),
> OKR systems, regulatory filing portals, and consumer-facing
> dashboards. Designed so that one signed envelope stream feeds
> many downstream consumers without re-keying.

## 4.1 GitHub Projects bridge

Most open-source organisations track milestones in GitHub Projects.
The reference bridge subscribes to the Projects v2 GraphQL API,
maps GitHub status fields to standard envelope fields, and emits
`milestone`, `release`, `progress_update`, and `commitment_change`
envelopes from the GitHub state stream.

```
github_status     →  envelope_type
"in progress"     →  progress_update with completion_pct
"done"            →  release
"closed (won't fix)" →  commitment_change kind=drop
"due date changed"   →  commitment_change kind=slip
```

The bridge is operated by the publishing organisation; GitHub
itself does not need to upgrade for the standard to be useful.

## 4.2 GitLab Epics bridge

GitLab Epics map similarly: each Epic becomes a milestone, Epic
state changes become progress updates and commitment changes,
Epic closure becomes a release. The reference bridge uses the
GitLab GraphQL API.

## 4.3 Linear bridge

Linear's Project model maps directly to the standard's milestone
envelope. Linear cycles map to progress updates. Linear's GraphQL
API is the bridge's source.

## 4.4 Jira bridge

Atlassian Jira's Initiative / Epic / Story hierarchy maps as:
Initiative → milestone, Epic → sub-milestone, Story → not exposed
(too granular for roadmap). Jira's REST API is the bridge's
source.

## 4.5 OKR system integration

OKR systems (WorkBoard, Lattice, 15Five, Quantive, internal
spreadsheets) publish quarterly objectives that organisations
use as the source of truth for cross-functional commitments. The
standard provides an `okr_binding` envelope that ties a milestone
to one or more OKR identifiers:

```
{
  "wia_roadmap_version": "1.0.0",
  "type": "okr_binding",
  "binding_id": "ULID",
  "milestone_id": "ULID",
  "okr_period": "2026Q3",
  "okr_objective_id": "...",
  "okr_key_result_ids": ["...", "..."],
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

The binding lets a downstream observer answer "which milestones
support which OKRs" without manual reconciliation.

## 4.6 Regulatory filing integration

Regulatory bodies increasingly require roadmap-style commitments
for accessibility (EU EAA / KR 장애인차별금지법), security (NIST
SSDF, EU CRA), and sustainability (CSRD, GHG Protocol). The
standard provides a `regulatory_commitment` envelope that ties a
milestone to a regulatory framework and an obligation
identifier:

```
{
  "wia_roadmap_version": "1.0.0",
  "type": "regulatory_commitment",
  "commitment_id": "ULID",
  "milestone_id": "ULID",
  "framework": "EU-EAA" | "EU-CRA" | "NIST-SSDF" | "CSRD" | "...",
  "obligation_ref": "framework-specific identifier",
  "signed_by_officer": "did:wia:officer:cco-...",
  "signature": { "alg": "Ed25519", "value": "..." }
}
```

The dual signature (publisher tenant key + responsible officer)
establishes accountability for the regulatory commitment.

## 4.7 Consumer-facing dashboard integration

Investor-facing roadmap dashboards consume read-only credentials
plus the freshness contract defined in Phase 3. The combination
ensures consumer apps display authentic, fresh, scope-bound data
without needing the publisher's full signing keys.

## 4.8 WIA family integration

This standard plugs into the broader WIA family at three points:

- **WIA-OMNI-API** — credential storage for organisational
  identities
- **WIA Standards** — the underlying framework this roadmap
  describes
- **WIA-AIR-SHIELD** — transport hardening for federation
  endpoints exposed to public networks

The integration is loosely coupled: organisations that do not use
the WIA family can still implement this standard end-to-end.

## 4.9 Korean integration notes

For Korean organisations subject to the 정보공시 의무 (information
disclosure obligation under 자본시장법) for material development
plans, the standard's `commitment_change` envelope provides the
canonical record of any material schedule revision. The
`approved_by` field maps to the responsible 공시담당자 identity.
The KRX disclosure portal can consume the envelope stream
directly via the bulk export endpoint (Phase 2 §2.12).

For Korean public-sector projects subject to the 공공데이터법
roadmap publication obligation, the standard's milestone envelope
is the natural format for data.go.kr metadata registration.

## 4.10 European integration notes

EU CRA (Cyber Resilience Act) requires manufacturers to publish
vulnerability handling and update commitments. The standard's
milestone + commitment_change envelopes are the natural format
for the CRA's product-lifecycle commitment disclosure.

EU CSRD (Corporate Sustainability Reporting Directive) requires
forward-looking sustainability commitments. The standard's
`regulatory_commitment` envelope (§4.6) provides the binding to
CSRD obligation identifiers.

## 4.11 Worked example: cross-jurisdictional accessibility

A multi-national publishing house commits to WCAG 2.2 AA
compliance for its consumer products by 2027-06-30. The publisher
emits:

- a `milestone` envelope with `tags: [accessibility, wcag-2.2-aa]`;
- a `regulatory_commitment` envelope binding the milestone to
  EU-EAA Article 4 obligations;
- a parallel `regulatory_commitment` binding to KR 장애인차별금지법
  Article 21 obligations (Korean accessibility law);
- quarterly `progress_update` envelopes;
- a final `release` envelope on delivery.

Three regulators, two languages, one envelope stream.

## 4.12 Reference list

- GitHub Projects v2 GraphQL API
- GitLab GraphQL API
- Linear GraphQL API
- Atlassian Jira REST API
- EU EAA — European Accessibility Act
- EU CRA — Cyber Resilience Act
- EU CSRD — Corporate Sustainability Reporting Directive
- NIST SSDF — Secure Software Development Framework
- KR 자본시장법 정보공시 의무
- KR 장애인차별금지법
- KR 공공데이터법

## 4.13 Operational considerations

Bridge implementations SHOULD emit envelopes on a publish-on-change
basis rather than a fixed cadence; spurious updates pollute the
audit trail and inflate replay-cache pressure. Implementations
SHOULD also de-duplicate within a 60-second window to absorb
transient issue-tracker write retries.

## 4.14 Backwards compatibility

Pre-standard roadmap publication systems (WordPress posts,
Confluence pages, Notion documents, README sections) MAY
continue to operate in parallel with the standard envelope
stream during a transitional window. The signed envelopes are
canonical for verification; the human-readable narrative is
canonical for communication.

弘益人間 — Benefit All Humanity. Integration exists so that the same
roadmap commitment can satisfy a regulator, an investor, a
journalist, and a downstream open-source maintainer without
requiring the publisher to maintain four separate stories.

## 4.15 Aggregator integration patterns

Beyond the issue-tracker bridges (§4.1–§4.4), aggregator services
operate as a class of integration in their own right. An
aggregator subscribes to multiple publishers' federated streams,
applies curation logic, and emits a single derivative stream that
downstream consumers find more convenient to consume than the raw
upstream streams.

Examples of aggregator services in the wild:

- web-platform-tests-style cross-browser feature trackers
- post-quantum cryptographic readiness consortia
- accessibility compliance trackers across regulator frameworks
- regulatory commitment dashboards for investor advocacy groups
- industry-association status trackers (CNCF maturity, OpenSSF
  Best Practices badges, Linux Foundation project healthchecks)

Each aggregator's curation logic is its own intellectual property;
the standard provides only the envelope format.

## 4.16 Investor relations integration

Public companies subject to material-disclosure obligations
typically operate an investor-relations function that consumes
roadmap commitments and translates them into regulatory filings
(US 8-K item 7.01 for material agreements, UK FCA continuous
obligations, EU MAR Article 17, KR 자본시장법 정보공시, JP 適時
開示). The standard's `investor_material_block` (Phase 1 §1.16)
provides the binding from the operational milestone to the
investor-relations workflow, so that when a milestone slips, the
IR team has a machine-readable signal rather than a Slack ping
from engineering.

## 4.17 Regulator portal binding

Regulator portals (KRX KIND, SEC EDGAR, ESMA OAM, FSA EDINET)
consume tabular filings in jurisdiction-specific formats. The
standard provides a `regulator_portal_export` envelope that
packages the relevant `milestone`, `commitment_change`, and
`regulatory_commitment` envelopes into the format the named
portal accepts. The export is signed by both the publisher and
the responsible disclosure officer so that the chain of custody
is complete.

## 4.18 OKR system bidirectional sync

Some OKR systems (notably WorkBoard, Quantive) support
bidirectional sync via webhooks. The standard's `okr_binding`
envelope (§4.5) supports both publish-from-OKR and publish-to-OKR
flows so that an organisation that uses an OKR system as the
canonical record of strategic intent can mirror its OKRs into
the roadmap envelope stream and have the OKR system updated when
a milestone slips.

弘益人間 — Benefit All Humanity. Integration exists so that one signed
roadmap commitment can serve a regulator, an investor, an
employee, and a downstream open-source maintainer without forcing
the publisher to maintain four separate stories.

## 4.19 Conformance maturity model

A reference conformance maturity model is published with three
levels:

- **Bridge-only** — translates from an existing system (GitHub,
  GitLab, Linear, Jira) into envelopes; does not author
  envelopes natively.
- **Native** — authors envelopes directly from internal planning
  tools without an upstream issue-tracker bridge.
- **Native + Federation** — additionally participates in
  federation handshakes and publishes a federation telemetry
  stream.

The maturity level is self-declared in the capability
advertisement and is not a gating requirement for participation.
A small organisation publishing milestones from a spreadsheet via
a hand-written script is a fully valid Bridge-only participant.

## 4.20 Final implementer guidance

Begin with Phase 1 envelope generation from your existing planning
system (almost certainly an issue tracker) and Phase 2 read
endpoints. Treat federation, OKR binding, and regulator-portal
export as Phase 4 work that follows after the read path is solid.
Resist the temptation to model every internal planning artefact
as a milestone; the standard is designed for commitments that
matter to people outside the publisher's organisation, not for
internal task management.

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
