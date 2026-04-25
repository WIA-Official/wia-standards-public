# WIA-AGRI-030: Urban Farming Standard

## Overview

The WIA Urban Farming Standard provides protocols for managing urban agriculture operations, from rooftop gardens to vertical farms.

## Key Features

- **Farm Management**: Track urban farms and their operations
- **Production Planning**: Schedule crops and rotations
- **Community Engagement**: Manage education and volunteer programs
- **Resource Optimization**: Monitor water, energy, and soil health
- **Market Access**: Connect with CSA programs and local markets
- **Sustainability Tracking**: Measure environmental impact

## Installation

```bash
npm install @wia/urban-farming
```

## Usage Example

```typescript
import { UrbanFarmingClient } from '@wia/urban-farming';

const client = new UrbanFarmingClient({
  apiKey: 'your-api-key',
});

// Register urban farm
const farm = await client.createFarm({
  name: 'Rooftop Haven',
  type: 'rooftop',
  area: 200,
  growingMethod: ['hydroponics', 'container'],
});

// Record harvest
await client.recordProduction({
  farmId: farm.farmId,
  crop: 'lettuce',
  yieldAmount: 25,
  quality: 'premium',
});

// Create CSA program
await client.createCSAProgram({
  farmId: farm.farmId,
  name: 'Weekly Veggie Box',
  members: 50,
  shareSize: 'medium',
});
```

## License

MIT License

---

© 2025 SmileStory Inc. / WIA
홍익인간 (弘益人間) (홍익인간) · Benefit All Humanity

## Standard scope

An open standard for managing urban agriculture operations from rooftop gardens to community vertical farms.

## Conformance levels

| Level | Description |
|-------|-------------|
| **Conformant** | Implementation publishes valid records against the OpenAPI contract in `api/openapi-3.0.yaml`. |
| **Audited** | Implementation passes all WIA conformance tests in `cli/urban-farming.sh validate`. |
| **Certified** | Implementation has registered with WIA at https://cert.wiastandards.com and exposes a public credential. |

## Repository layout

```
urban-farming/
├── README.md          ← you are here
├── index.html         ← landing page (EN/KO toggle, dark theme)
├── simulator/         ← interactive simulator
├── spec/              ← Phase 1–4 specifications
├── api/               ← OpenAPI 3.0 contract + TS/Python SDKs
├── cli/               ← urban-farming command-line tool
├── ebook/             ← long-form documentation (EN + KO)
└── press/             ← editorial articles + DALLE prompts
```

## Quick start

```bash
# Install the CLI
chmod +x cli/urban-farming.sh
sudo ln -s "$(pwd)/cli/urban-farming.sh" /usr/local/bin/urban-farming

# Configure
export WIA_URBAN_FARMING_API_KEY="your-api-key"
```

## Governance

- All citations follow [`docs/CITATION-POLICY.md`](../../docs/CITATION-POLICY.md): only verifiable primary sources (ISO, IEC, IEEE, RFC, W3C, HL7) appear in body and spec.
- The standard graduates to **Deep Published v3** only when `validate-published-v3.sh` passes 21/21 (17 size/structure + 4 veracity gates).
- Implementations seeking external citation MUST reference the Deep Published version of the standard.

## License

MIT — see repository root.

弘益人間 (홍익인간) · Benefit All Humanity
