# WIA-green-infrastructure Green Infrastructure

> Cross-vendor data and protocol layer for Green Infrastructure.

## Overview

WIA-green-infrastructure defines standardized data formats and protocol semantics for
Green Infrastructure. The standard exposes four PHASE documents covering data
format, API interface, protocol, and integration concerns.

## Documentation

- spec/PHASE-1-DATA-FORMAT.md — schemas
- spec/PHASE-2-API-INTERFACE.md — REST + streaming APIs
- spec/PHASE-3-PROTOCOL.md — message protocol
- spec/PHASE-4-INTEGRATION.md — integration patterns

## Conformance Tiers

| Tier      | Scope                                                    |
|-----------|----------------------------------------------------------|
| Surface   | data formats accepted; self-attested                     |
| Verified  | annual third-party audit                                 |
| Anchored  | continuous evidence package per Annex G                  |

## Reference Standards

This standard cites only ALLOW sources per the WIA citation policy:
ISO/IEC, IEEE, RFC, W3C. Schemas use JSON Schema 2020-12 and OpenAPI
3.1. Signatures use Sigstore (DSSE envelope, Rekor transparency log).

## License

MIT License — © 2025 WIA Standards Committee

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Quick Start

```bash
npm install @wia/green-infrastructure
```

```typescript
import { Client } from '@wia/green-infrastructure';
const client = new Client({ baseUrl: 'https://api.example.com/wia-green-infrastructure' });
await client.submitRecord(record);
```

## Reference Implementations

- TypeScript SDK: `api/typescript/`
- Schemas: JSON Schema 2020-12 in `spec/PHASE-1-DATA-FORMAT.md`
- Conformance vectors: `tests/phase-vectors/`

## Compliance Standards

- ISO/IEC 27001:2022 — information security management
- ISO/IEC 27018:2019 — PII protection in public clouds
- ISO/IEC 17065:2012 — product certification bodies
- OpenAPI Specification 3.1, JSON Schema 2020-12
- IETF RFC 9700 (OAuth 2.1), RFC 9457 (Problem Details)
- Sigstore (DSSE envelope, Rekor transparency log)

弘益人間 (Hongik Ingan) — Benefit All Humanity
