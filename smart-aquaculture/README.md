# WIA Smart Aquaculture Standard

> Water quality monitoring, fish health tracking, and sustainable seafood production

## Overview

The WIA Smart Aquaculture Standard defines standardized data formats for smart aquaculture systems, including water quality monitoring, fish health tracking, feeding automation, and sustainable seafood production.

## Key Features

- **Water Quality Monitoring**: Real-time temperature, salinity, dissolved oxygen, pH, ammonia
- **Fish Health Tracking**: Growth rates, mortality, disease monitoring, vaccinations
- **Feeding Automation**: Smart feeding schedules, feed conversion ratios
- **Biomass Management**: Population density, weight distribution, harvest projections
- **Sustainability**: Environmental impact tracking, resource efficiency

## Quick Start

### TypeScript SDK

```bash
npm install @wia/smart-aquaculture
```

```typescript
import { SmartAquacultureClient } from '@wia/smart-aquaculture';

const client = new SmartAquacultureClient({
  baseUrl: 'https://api.example.com/aquaculture',
  apiKey: 'your-api-key'
});

// Get farm information
const farm = await client.getFarm('farm-uuid-123');
console.log(farm.data);

// Submit water quality data
const result = await client.submitWaterQuality(waterQualityData);
```

## Documentation

- [Phase 1: Data Format](spec/PHASE-1-DATA-FORMAT.md)
- [Phase 2: API Interface](spec/PHASE-2-API-INTERFACE.md)
- [Phase 3: Protocol](spec/PHASE-3-PROTOCOL.md)
- [Phase 4: Integration](spec/PHASE-4-INTEGRATION.md)

## Supported Species

- **Fish**: Salmon, tilapia, sea bass, flatfish
- **Shellfish**: Oysters, mussels, clams
- **Crustaceans**: Shrimp, prawns, crab
- **Aquatic Plants**: Seaweed, algae

## CLI

A reference CLI ships under `cli/smart-aquaculture.sh` with subcommands
`validate`, `farm`, `water-quality`, `fish-health`, `feeding`, `harvest`, `info`.

## Conformance levels

| Level | Required surfaces |
|-------|-------------------|
| Minimal | Phase 1 envelopes, Phase 2 publish + query |
| Core | Plus Phase 3 federation, FAO ALDFI vocabulary alignment, ASC certification mapping |
| Full | Plus environmental impact tracking, traceability chain to retail (Global GAP-style) |

## Companion standards

* **FAO ALDFI** — Aquatic Living Resources Data and Information Network vocabularies
* **ASC** — Aquaculture Stewardship Council certification
* **BAP** — Best Aquaculture Practices
* **Global GAP** — aquaculture supply-chain traceability
* **WIA-OMNI-API** — credential storage for farm operator identities
* **WIA Water Scarcity Response** — sister standard for cross-watershed coordination

## License

MIT License - © 2025 WIA Standards Committee

**홍익인간 (弘益人間) - Benefit All Humanity**
