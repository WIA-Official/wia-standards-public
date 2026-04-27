# WIA Roadmap Standard

> Open standard for cross-organisation development roadmap publication,
> milestone tracking, dependency declaration, and progress attestation.

**Philosophy**: 弘益人間 — Benefit All Humanity

---

## Why this standard exists

Roadmaps are the most-shared, least-interoperable artefact in
software development. Every organisation publishes them — to
investors, partners, customers, regulators, journalists — but every
organisation publishes them in a different format. The result is a
landscape where comparing two organisations' commitments to, say,
post-quantum cryptographic readiness or accessibility compliance
requires manual reading of free-form prose.

WIA Roadmap defines a small set of signed envelopes (`milestone`,
`dependency`, `release`, `progress_update`, `commitment_change`)
that turn a roadmap into a machine-comparable artefact without
forcing organisations to abandon their narrative roadmap pages. A
narrative roadmap and a signed envelope stream coexist; the
narrative is for humans, the envelopes are for tooling.

---

## 4-Phase architecture

| Phase | Scope |
|-------|-------|
| 1 — Data Format | Milestone, dependency, release, progress, commitment-change envelopes |
| 2 — API Interface | HTTP surface for publish, query, stream, subscription |
| 3 — Federation Protocol | Cross-organisation dependency declaration, replay defence, scope binding |
| 4 — Integration | GitHub Issues / Projects, GitLab Epics, Linear, Jira, OKR systems, regulator filings |

---

## Quick start

```bash
# Run a reference roadmap publishing host
docker run -p 8080:8080 wia/roadmap-host:1.0.0

# Subscribe to live milestone updates from a project
curl -N "http://localhost:8080/roadmap/stream?project_id=did:wia:project:wia-standards" \
     -H "Accept: text/event-stream"
```

---

## CLI

A reference CLI ships under `cli/roadmap.sh` with subcommands
`validate`, `milestone`, `dependency`, `release`, `progress`, `info`.

---

## Companion standards

* **GitHub Issues / Projects** — primary upstream issue tracker source
* **GitLab Epics** — alternative upstream
* **Linear / Jira** — enterprise issue trackers
* **OKR systems** — organisational alignment binding
* **WIA-OMNI-API** — credential storage for organisational identities
* **WIA Standards** — the underlying framework this roadmap describes

---

## Conformance levels

| Level | Required surfaces |
|-------|-------------------|
| Minimal | Phase 1 envelopes, Phase 2 publish + query |
| Core | Plus Phase 3 federation, dependency declaration |
| Full | Plus Phase 4 issue-tracker bridges, OKR binding |

---

MIT License — © 2025 WIA (World Certification Industry Association)

弘益人間 — Benefit All Humanity.
