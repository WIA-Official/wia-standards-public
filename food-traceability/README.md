# WIA Food Traceability Standard

> Blockchain-verified farm-to-fork traceability

## Overview

The WIA Food Traceability Standard enables complete product tracking throughout the food supply chain, from farm to consumer, using blockchain technology and verifiable credentials.

## Key Features

- **Complete Traceability**: Full farm-to-fork tracking with immutable records
- **Blockchain Integration**: Ethereum, Hyperledger Fabric support
- **QR Codes**: Consumer-facing product verification
- **Verifiable Credentials**: W3C VC standard compliance
- **Event Tracking**: Harvest, processing, packaging, transport, retail, sale

## Quick Start

### TypeScript SDK

```bash
npm install @wia/food-traceability
```

```typescript
import { TraceabilityClient } from '@wia/food-traceability';

const client = new TraceabilityClient({
  baseUrl: 'https://api.example.com/traceability',
  apiKey: 'your-api-key'
});

// Get complete trace chain
const chain = await client.getTraceChain('product-id-123');
console.log(chain.data);

// Scan QR code
const trace = await client.scanQRCode('QR-12345');
```

## Documentation

- Phase 1: Data Format (see spec directory)
- Phase 2: API Interface
- Phase 3: Protocol
- Phase 4: Integration

## Blockchain Support

- **Ethereum**: Public blockchain for transparency
- **Hyperledger Fabric**: Private/permissioned networks
- **Polygon**: Low-cost transactions
- **WIA Chain**: Purpose-built for agriculture

## License

MIT License - © 2025 WIA Standards Committee

**홍익인간 (弘益人間) - Benefit All Humanity**

## Conformance Tiers

| Tier      | Scope                                                    |
|-----------|----------------------------------------------------------|
| Surface   | data formats accepted; self-attested                     |
| Verified  | annual third-party audit against PHASE documents         |
| Anchored  | continuous evidence package per Annex G                  |

## Reference Standards

This standard cites only ALLOW sources per the WIA citation policy:
ISO/IEC, IEEE, RFC, W3C. Schemas use JSON Schema 2020-12 and OpenAPI
3.1. Signatures use Sigstore (DSSE envelope, Rekor transparency log).

## Open Governance

Issues and proposals are tracked at github.com/WIA-Official/wia-standards/
issues with the `food-traceability` label. The WIA Standards working group reviews
open issues at the start of every minor release cycle.

弘益人間 (Hongik Ingan) — Benefit All Humanity
