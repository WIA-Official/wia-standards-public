# WIA Food Safety Standard

> HACCP compliance, testing, and incident management

## Overview

The WIA Food Safety Standard provides standardized data formats for food safety management, including hazard analysis, testing protocols, inspections, certifications, and incident reporting.

## Key Features

- **Hazard Analysis**: Biological, chemical, physical, and allergen hazards
- **Laboratory Testing**: Microbiological, chemical residue, allergen testing
- **Inspections**: Facility audits, compliance scoring, corrective actions
- **Certifications**: HACCP, ISO 22000, FSSC 22000, organic
- **Incident Management**: Outbreak tracking, recalls, root cause analysis

## Quick Start

### TypeScript SDK

```bash
npm install @wia/food-safety
```

```typescript
import { FoodSafetyClient } from '@wia/food-safety';

const client = new FoodSafetyClient({
  baseUrl: 'https://api.example.com/food-safety',
  apiKey: 'your-api-key'
});

// Submit test results
const result = await client.submitTestResult(testData);

// Report food safety incident
const incident = await client.reportIncident(incidentData);
```

## Documentation

- Phase 1: Data Format (see spec directory)
- Phase 2: API Interface
- Phase 3: Protocol
- Phase 4: Integration

## Compliance Standards

- **HACCP**: Hazard Analysis Critical Control Points
- **ISO 22000**: Food safety management systems
- **FSSC 22000**: Food Safety System Certification
- **FDA FSMA**: Food Safety Modernization Act
- **EU Regulation 178/2002**: General food law

## License

MIT License - © 2025 WIA Standards Committee

**홍익인간 (弘益人間) - Benefit All Humanity**

## Conformance Tiers

| Tier      | Scope                                                      | Audit cadence |
|-----------|------------------------------------------------------------|---------------|
| Surface   | data formats accepted; no formal audit                     | self-attested |
| Verified  | annual third-party audit against this PHASE document       | annual        |
| Anchored  | continuous evidence package per Annex G; SBOM signed       | continuous    |

Implementations declare their tier in the OpenAPI document via the
`x-wia-conformance-tier` extension field. Auditors consume the field to
determine the appropriate review depth without re-running the entire
test matrix.

## Reference Implementations

- TypeScript SDK: `api/typescript/`
- Schemas: JSON Schema Draft 2020-12 in `spec/PHASE-1-DATA-FORMAT.md`
- Conformance vectors: `tests/phase-vectors/`

## Layout

```
food-safety/
├── README.md           # this file
├── index.html          # standard landing page
├── spec/
│   ├── PHASE-1-DATA-FORMAT.md
│   ├── PHASE-2-API-INTERFACE.md
│   ├── PHASE-3-PROTOCOL.md
│   └── PHASE-4-INTEGRATION.md
├── api/                # SDK (TypeScript)
├── cli/                # WIA CLI tooling
├── ebook/{en,ko}/      # 8-chapter publishable e-book
└── press/              # press kit and release notes
```

## Open Governance

Issues and proposals are tracked at
https://github.com/WIA-Official/wia-standards/issues with the
`food-safety` label. The WIA Standards working group reviews open
issues at the start of every minor release cycle and publishes the
resulting decision log alongside the release notes.

弘益人間 (Hongik Ingan) — Benefit All Humanity
