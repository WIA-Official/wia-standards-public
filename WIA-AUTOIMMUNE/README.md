# WIA-AUTOIMMUNE Standard

🛡️ **Treg-Microbiome Axis Autoimmune Disease Interoperability**

[![WIA Standard](https://img.shields.io/badge/WIA-Official%20Standard-06B6D4)](https://wia.live/autoimmune)
[![Version](https://img.shields.io/badge/version-1.0.0-blue)](https://github.com/WIA-Official/wia-standards)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

---

> **홍익인간 (弘益人間) - Benefit All Humanity** - "Benefit All Humanity"
>
> The WIA-AUTOIMMUNE Standard unifies fragmented autoimmune research by establishing the Treg-Microbiome Axis as the foundation for immune tolerance restoration.

---

## Overview

The WIA-AUTOIMMUNE Standard provides a comprehensive framework for:

- **Treg Functional Assessment** - Standardized evaluation of regulatory T cell count, FOXP3 expression, and suppressive function
- **Microbiome Analysis** - SCFA-producing bacteria quantification and dysbiosis scoring
- **Disease Activity Tracking** - Interoperable scoring for RA, SLE, MS, T1D, IBD, and more
- **Flare Prediction** - Early warning system based on Treg-Microbiome Axis biomarkers
- **Treatment Recommendations** - Evidence-based personalized interventions (low-dose IL-2, FMT, dietary)

## The Treg-Microbiome Axis Discovery

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  💡 Unified Principle: Immune Tolerance = Treg Function + Gut Microbiome   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   Gut Microbiome                         Regulatory T Cells                 │
│   ┌───────────┐                          ┌───────────────┐                 │
│   │ Butyrate  │─────── SCFA ─────────▶   │ FOXP3+       │                 │
│   │ Producers │        Production        │ Stabilization │                 │
│   └───────────┘                          └───────────────┘                 │
│        │                                        │                          │
│        │                                        │                          │
│        ▼                                        ▼                          │
│   ┌───────────┐                          ┌───────────────┐                 │
│   │ Gut       │◀── Immune ──────────────▶│ Suppressive   │                 │
│   │ Barrier   │   Regulation             │ Function      │                 │
│   └───────────┘                          └───────────────┘                 │
│                                                                             │
│   DISRUPTION IN AUTOIMMUNITY:                                               │
│   • Reduced SCFA producers → Impaired Treg function                        │
│   • Increased intestinal permeability → Systemic inflammation              │
│   • Th17/Treg imbalance → Loss of self-tolerance                           │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Supported Diseases

| Code | Disease | Primary Activity Score |
|------|---------|----------------------|
| RA | Rheumatoid Arthritis | DAS28 |
| SLE | Systemic Lupus Erythematosus | SLEDAI |
| MS | Multiple Sclerosis | EDSS |
| T1D | Type 1 Diabetes | HbA1c |
| IBD | Inflammatory Bowel Disease | CDAI/Mayo |
| PSO | Psoriasis | PASI |
| HT | Hashimoto's Thyroiditis | TPO Ab |
| GD | Graves' Disease | TRAb |

## Quick Start

### Installation

```bash
npm install @wia/autoimmune-sdk
```

### TypeScript SDK Usage

```typescript
import { WiaAutoImmuneClient } from '@wia/autoimmune-sdk';

const client = new WiaAutoImmuneClient({
  apiKey: 'your-api-key',
  environment: 'production'
});

// Create comprehensive assessment
const assessment = await client.assessments.create({
  patient_id: 'patient-uuid',
  disease_type: 'RA',
  immune_panel: {
    treg_count: 45.2,
    foxp3_expression: 68.5,
    suppressive_function: 42.0,
    autoantibodies: [
      { name: 'RF', value: 85, positive: true },
      { name: 'Anti-CCP', value: 320, positive: true }
    ],
    cytokines: { il6: 28.5, il17: 45.2 }
  },
  microbiome_sample_id: 'sample-123'
});

// Get Treg status
const tregStatus = await client.treg.getStatus('patient-uuid');
console.log(`Treg Function: ${tregStatus.treg.suppressive_function}%`);
console.log(`Functional Status: ${tregStatus.functional_status}`);

// Predict flare risk
const flareRisk = await client.predictions.predictFlare({
  patient_id: 'patient-uuid',
  include_microbiome: true
});
console.log(`Flare Risk: ${flareRisk.flare_probability}%`);
console.log(`Risk Level: ${flareRisk.risk_level}`);

// Get treatment recommendations
const recommendations = await client.treatment.getRecommendations({
  patient_id: 'patient-uuid'
});
console.log('Recommended treatments:', recommendations.primary_recommendations);
```

### CLI Usage

```bash
# Install CLI
curl -sL https://wia.live/install/autoimmune | bash

# Configure
wia-autoimmune config

# Create assessment
wia-autoimmune assess patient-123 --disease RA

# Get Treg status
wia-autoimmune treg patient-123 --format summary

# Predict flare
wia-autoimmune flare patient-123

# Calculate TMAS score
wia-autoimmune tmas patient-123
```

## Standard Components

### Phase 1: Data Format
Standardized JSON schemas for autoimmune profiles, Treg status, and microbiome data.

```json
{
  "@context": ["https://wia.live/autoimmune/v1"],
  "type": ["AutoimmuneProfile"],
  "patient_id": "uuid",
  "disease_type": "RA",
  "immune_markers": {
    "treg": {
      "count": { "value": 45.2, "unit": "cells/μL" },
      "foxp3_expression": 68.5,
      "suppressive_function": 42.0
    },
    "th17_treg_ratio": 3.2
  },
  "microbiome": {
    "dysbiosis_score": 62,
    "scfa_levels": { "butyrate": 45.2 }
  }
}
```

### Phase 2: API Interface
RESTful API with OpenAPI 3.1 specification for all Treg-Microbiome Axis operations.

```
POST /api/v1/autoimmune/assess         - Create comprehensive assessment
GET  /api/v1/autoimmune/treg-status    - Get Treg functional status
GET  /api/v1/autoimmune/microbiome     - Get microbiome analysis
POST /api/v1/autoimmune/flare/predict  - Predict flare risk
POST /api/v1/autoimmune/treatment/recommend - Get personalized recommendations
```

### Phase 3: Clinical Protocols
Evidence-based protocols for:
- Microbiome Restoration (12-16 week program)
- Low-dose IL-2 Treg Expansion
- Integrated Treg-Microbiome Intervention
- Flare Early Warning System
- Deep Remission Achievement

### Phase 4: Integration
- **EHR**: FHIR R4, HL7 v2.x integration
- **Labs**: Quest, LabCorp, specialty microbiome labs
- **Research**: OMOP CDM, i2b2, TriNetX
- **Clinical Trials**: CDISC SDTM/ADaM
- **Wearables**: HealthKit, Google Fit

## TMAS Score (Treg-Microbiome Axis Score)

```
TMAS = (Treg_Score × 0.4) + (Microbiome_Score × 0.4) + (Activity × 0.2)

Interpretation:
80-100: Excellent tolerance, remission likely
60-79:  Good tolerance, stable disease
40-59:  Impaired tolerance, active disease
20-39:  Poor tolerance, high flare risk
0-19:   Severely impaired, urgent intervention
```

## API Reference

### Base URL
```
Production: https://api.wia.live/autoimmune/v1
Staging:    https://api-staging.wia.live/autoimmune/v1
```

### Authentication
```http
Authorization: Bearer <api_key>
```

### Example Request
```bash
curl -X POST https://api.wia.live/autoimmune/v1/flare/predict \
  -H "Authorization: Bearer $API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "patient_id": "patient-uuid",
    "include_microbiome": true,
    "prediction_horizon_days": 30
  }'
```

## Directory Structure

```
WIA-AUTOIMMUNE/
├── spec/
│   ├── PHASE-1-DATA-FORMAT.md      # JSON schemas
│   ├── PHASE-2-API-INTERFACE.md    # OpenAPI specification
│   ├── PHASE-3-PROTOCOL.md         # Clinical protocols
│   └── PHASE-4-INTEGRATION.md      # EHR/Lab integration
├── api/
│   └── typescript/
│       ├── src/
│       │   ├── types.ts            # Type definitions
│       │   └── index.ts            # SDK implementation
│       └── package.json
├── cli/
│   └── wia-autoimmune.sh           # CLI tool
├── README.md
└── install.sh
```

## Evidence Base

The Treg-Microbiome Axis approach is supported by:

1. **Treg Dysfunction** (2024-2025)
   - Autoimmune diseases show Treg functional defects, not just count reduction
   - GRAIL E3 ligase activity correlates with disease severity
   - CAR-Treg therapies showing promise in early trials

2. **Microbiome-Immune Crosstalk**
   - Butyrate stabilizes FOXP3 expression
   - SCFA producers depleted across all major autoimmune diseases
   - FMT shows efficacy in IBD and emerging data in RA

3. **Integrated Therapies**
   - Low-dose IL-2 selectively expands Tregs
   - Dietary fiber increases SCFA production
   - Combined approaches may offer synergistic benefits

## Contributing

We welcome contributions! Please see our [Contributing Guide](../../CONTRIBUTING.md).

## License

MIT License - see [LICENSE](../../LICENSE) for details.

---

**© 2026 WIA (World Certification Industry Association)**

홍익인간 (弘益人間) · Benefit All Humanity

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
