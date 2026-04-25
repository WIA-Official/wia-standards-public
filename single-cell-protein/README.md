# WIA-AGRI-034: Single Cell Protein Standard

## Overview

The WIA Single Cell Protein Standard provides protocols for producing protein-rich biomass from microorganisms for food and feed applications.

## Key Features

- **Fermentation Management**: Control production parameters
- **Quality Assurance**: Monitor nutritional and safety metrics
- **Downstream Processing**: Optimize recovery and purification
- **Application Development**: Create market-ready products
- **Sustainability Assessment**: Measure environmental impact
- **Regulatory Compliance**: Track approval status

## Installation

```bash
npm install @wia/single-cell-protein
```

## Usage Example

```typescript
import { SingleCellProteinClient } from '@wia/single-cell-protein';

const client = new SingleCellProteinClient({
  apiKey: 'your-api-key',
});

// Start production
const production = await client.createProduction({
  organism: 'yeast',
  strain: 'Candida utilis',
  substrate: { type: 'methanol', carbonSource: 'methanol' },
});

// Monitor fermentation
await client.recordProcessMetrics({
  productionId: production.productionId,
  biomassDensity: 35.5,
  specificGrowthRate: 0.15,
});
```

## License

MIT License

---

© 2025 SmileStory Inc. / WIA
弘익人間 (홍익인간) · Benefit All Humanity

---

**홍익인간 (弘益人間)** - Benefit All Humanity 🌍

---

## Conformance tiers

WIA conformance for **single-cell-protein** is evaluated across three tiers, mirrored in every PHASE document under `spec/`:

| Tier | Scope | Audit cadence |
|------|-------|---------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days of detection. The conformity assessment process for Tier 3 is aligned with ISO/IEC 17065:2012 and depends on the documentary evidence retention policy described in the per-PHASE Annex A under `spec/`.

## Layout

```
standards/single-cell-protein/
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
