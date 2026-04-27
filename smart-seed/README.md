# WIA Smart Seed Standard

> Digital seed passports, germination testing, and variety registration

## Overview

The WIA Smart Seed Standard provides standardized data formats for smart seed systems, variety registration, germination testing, seed certification, digital seed passports, and intellectual property protection.

## Key Features

- **Variety Registration**: Plant variety protection, UPOV integration, breeder rights
- **Germination Testing**: ISTA/AOSA compliant testing protocols
- **Seed Certification**: OECD seed schemes, quality assurance
- **Digital Passports**: QR codes, blockchain traceability, verifiable credentials
- **Quality Metrics**: Germination rate, purity, moisture, vigor index

## Quick Start

### TypeScript SDK

```bash
npm install @wia/smart-seed
```

```typescript
import { SmartSeedClient } from '@wia/smart-seed';

const client = new SmartSeedClient({
  baseUrl: 'https://api.example.com/seed',
  apiKey: 'your-api-key'
});

// Get seed variety information
const variety = await client.getVariety('550e8400-e29b-41d4-a716-446655440000');
console.log(variety.data);

// Scan QR code for seed passport
const passport = await client.scanQRCode('QR-12345');
console.log(passport.data);
```

## Documentation

- [Phase 1: Data Format](spec/PHASE-1-DATA-FORMAT.md)
- [Phase 2: API Interface](spec/PHASE-2-API-INTERFACE.md)
- [Phase 3: Protocol](spec/PHASE-3-PROTOCOL.md)
- [Phase 4: Integration](spec/PHASE-4-INTEGRATION.md)

## Certifications Supported

- **OECD Seed Schemes**: Cereals, maize, vegetables
- **ISTA**: International Seed Testing Association
- **UPOV**: Plant variety protection
- **Organic**: Organic seed certification

## CLI

A reference CLI ships under `cli/smart-seed.sh` with subcommands
`validate`, `variety`, `lot`, `germination-test`, `passport`, `info`.

## Conformance levels

| Level | Required surfaces |
|-------|-------------------|
| Minimal | Phase 1 envelopes, Phase 2 publish + query |
| Core | Plus Phase 3 federation, ISTA test alignment, UPOV variety registration mapping |
| Full | Plus OECD seed scheme certification bridges, blockchain traceability for passport, organic seed certification |

## Companion standards

* **ISTA** — International Rules for Seed Testing
* **UPOV** — Plant variety protection convention
* **OECD Seed Schemes** — international cereal / maize / vegetable schemes
* **AOSA** — Association of Official Seed Analysts (US)
* **WIA Smart Breeding** — sister standard for breeding program integration
* **WIA Smart Farm** — sister standard for farm-side seed deployment

## License

MIT License - © 2025 WIA Standards Committee

**홍익인간 (弘益人間) - Benefit All Humanity**
