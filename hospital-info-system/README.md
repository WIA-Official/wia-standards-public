# WIA-MED-007: Hospital Information System Standard

**Status:** ✅ Complete
**Version:** 1.0.0

## Overview

WIA-MED-007 defines standards for Hospital Information Systems (HIS) including patient management, scheduling, admissions, and hospital workflows.

## Quick Start

```typescript
import { createClient } from '@wia/med-007';

const his = createClient({
  baseUrl: 'https://api.wia.live/his',
  apiKey: 'your-api-key'
});

const patients = await his.getPatients();
```

## Features

- 👤 Patient Registration & Management
- 📅 Appointment Scheduling
- 🏥 Admission & Discharge
- 🛏️ Bed Management
- 📊 Department Operations

**홍익인간 (弘益人間) - Benefit All Humanity**
© 2025 WIA | MIT License

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
issues with the `hospital-info-system` label. The WIA Standards working group reviews
open issues at the start of every minor release cycle.

弘益人間 (Hongik Ingan) — Benefit All Humanity

## Quick Start

```bash
npm install @wia/hospital-info-system
```

```typescript
import { HospitalClient } from '@wia/hospital-info-system';
const client = new HospitalClient({ baseUrl: 'https://api.example.com/wia-hospital-info-system' });
await client.submitPatientRecord(record);
```

## Reference Implementations

- TypeScript SDK: `api/typescript/`
- Schemas: JSON Schema 2020-12 in `spec/PHASE-1-DATA-FORMAT.md`
- Conformance vectors: `tests/phase-vectors/`
