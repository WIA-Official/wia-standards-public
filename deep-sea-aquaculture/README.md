# WIA-AGRI-031: Deep-Sea Aquaculture Standard

## Overview

Standard for sustainable deep-sea aquaculture operations, species management, and ocean farming. The specification covers data formats for site registration, species inventory, feed and harvest events, and the operational telemetry needed to evaluate environmental impact alongside commercial yield.

The PHASE documents under `spec/` are the normative source. This README provides only the table of contents and the conformance + governance entry points.

## TypeScript SDK

```bash
npm install @wia/deep-sea-aquaculture-sdk
```

---

© 2025 WIA · 홍익인간 (弘益人間)

---

## Conformance tiers

WIA conformance for **deep-sea-aquaculture** is evaluated across three tiers, mirrored in every PHASE document under `spec/`:

| Tier | Scope | Audit cadence |
|------|-------|---------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page.

## Layout

```
standards/deep-sea-aquaculture/
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
