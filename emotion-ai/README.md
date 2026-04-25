# Emotion Ai

WIA standard `emotion-ai` — see <https://wiastandards.com> for the full standards catalogue.

This README accompanies the normative specification under `spec/`. It introduces the standard at a glance and points to the conformance, layout, and governance materials that an implementer needs to begin work.

---

## Conformance tiers

WIA conformance for **emotion-ai** is evaluated across three tiers, mirrored in every PHASE document under `spec/`:

| Tier | Scope | Audit cadence |
|------|-------|---------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days of detection. The conformity assessment process for Tier 3 is aligned with ISO/IEC 17065:2012 and depends on the documentary evidence retention policy described in the per-PHASE Annex A under `spec/`.

## Layout

```
standards/emotion-ai/
├── README.md            # this document
├── index.html           # human-readable landing page
├── simulator/           # interactive browser-based simulator
├── spec/                # PHASE-1..N normative specifications
├── api/                 # reference TypeScript SDK skeleton
├── cli/                 # POSIX shell client
├── press/               # press kit (article + DALL·E prompts)
└── ebook/{en,ko}/      # eight-chapter ebook editions
```

The PHASE documents under `spec/` are the normative source. Code under `api/`, `cli/`, and `simulator/` is informative reference material that demonstrates the contract; production implementations may diverge as long as they preserve the PHASE contract.

## Open governance

Comments, proposals, and conformance reports are accepted via the GitHub issues tracker on the WIA-Official organization. Major version bumps follow the WIA Standards governance process documented at <https://wiastandards.com/governance>.

---

**弘益人間 · Benefit All Humanity** — © 2026 WIA. Licensed under MIT.
