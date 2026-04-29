# WIA-CHRONIC-PAIN Standard

🧠 **Neuroplasticity Reversal Chronic Pain Interoperability**

[![WIA Standard](https://img.shields.io/badge/WIA-Official%20Standard-06B6D4)](https://wia.live/chronic-pain)
[![Version](https://img.shields.io/badge/version-1.0.0-blue)](https://github.com/WIA-Official/wia-standards)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

---

> **홍익인간 (弘益人間) - Benefit All Humanity** - "Benefit All Humanity"
>
> The WIA-CHRONIC-PAIN Standard transforms chronic pain management by establishing the **Neuroplasticity Reversal** paradigm - chronic pain represents maladaptive neural changes that can be reversed through targeted multimodal intervention.

---

## Overview

The WIA-CHRONIC-PAIN Standard provides a comprehensive framework for:

- **Pain Phenotyping** - Classification (nociceptive, neuropathic, nociplastic, mixed)
- **Central Sensitization Assessment** - CSI, QST, temporal summation, CPM
- **Neuroplasticity Reversal Index (NRI)** - Quantify reversal potential
- **Non-Invasive Neuromodulation** - TMS, tDCS protocols
- **Multimodal Treatment** - CBT, exercise, mindfulness integration
- **Opioid-Sparing Strategies** - Safe tapering with alternative management

## The Neuroplasticity Reversal Discovery

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  💡 Unified Principle: Chronic Pain = Maladaptive Neuroplasticity          │
│                        → REVERSIBLE through targeted intervention           │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   Acute Pain                    Chronic Pain              Reversal         │
│   ┌─────────┐                   ┌─────────┐              ┌─────────┐       │
│   │ Normal  │  ──Sensitization──▶│Maladaptive│──Treatment──▶│ Restored│    │
│   │ Brain   │                   │ Changes  │              │  Brain  │      │
│   └─────────┘                   └─────────┘              └─────────┘       │
│                                                                             │
│   What Changes:                 What Reverses:                              │
│   • Gray matter ↓ (ACC, PFC)    • CBT + Exercise = Gray matter ↑           │
│   • Central sensitization ↑     • TMS/tDCS = Cortical modulation           │
│   • Pain catastrophizing ↑      • Pain education = Cognitive shift         │
│   • Fear-avoidance ↑            • Graded exposure = Function ↑             │
│                                                                             │
│   KEY INSIGHT: The brain CAN change back!                                  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Pain Classifications

| Code | Type | Mechanism | Examples |
|------|------|-----------|----------|
| NOC | Nociceptive | Tissue damage signal | Arthritis, acute injury |
| NEU | Neuropathic | Nerve damage/dysfunction | Diabetic neuropathy, PHN |
| NOP | Nociplastic | Central sensitization | Fibromyalgia, chronic LBP |
| MIX | Mixed | Multiple mechanisms | CRPS, failed back surgery |

## Quick Start

### Installation

```bash
npm install @wia/chronic-pain-sdk
```

### TypeScript SDK Usage

```typescript
import { WiaChronicPainClient, calculateNRI, interpretNRI } from '@wia/chronic-pain-sdk';

const client = new WiaChronicPainClient({
  apiKey: 'your-api-key',
  environment: 'production'
});

// Create comprehensive assessment
const assessment = await client.assessments.create({
  patient_id: 'patient-uuid',
  pain_type: 'nociplastic',
  pain_scores: {
    nrs_current: 7,
    bpi_severity: 6.5,
    bpi_interference: 7.2
  },
  duration_months: 36,
  sensitization_data: {
    csi_score: 62,
    temporal_summation_present: true
  }
});

// Get Neuroplasticity Reversal Index
const nri = await client.neuroplasticity.getIndex('patient-uuid');
console.log(`NRI: ${nri.current_index}/100`);
console.log(`Reversal Potential: ${nri.reversal_potential}%`);
console.log(`Trend: ${nri.trend}`);

// Get neuromodulation plan
const neuromodPlan = await client.neuromodulation.createPlan({
  patient_id: 'patient-uuid',
  target_symptoms: ['pain_intensity', 'central_sensitization'],
  preferences: { clinic_based: true }
});
console.log(`Recommended: ${neuromodPlan.primary_modality.type}`);
console.log(`Sessions: ${neuromodPlan.primary_modality.expected_sessions}`);

// Get treatment recommendations
const recommendations = await client.treatment.getRecommendations('patient-uuid');
console.log('Treatment plan:', recommendations.primary_recommendations);

// Calculate NRI locally
const localNRI = calculateNRI(62, 45, 30, 36);
const interpretation = interpretNRI(localNRI);
console.log(`Local NRI: ${localNRI}, ${interpretation.description}`);
```

### CLI Usage

```bash
# Install CLI
curl -sL https://wia.live/install/chronic-pain | bash

# Configure
wia-chronic-pain config

# Create assessment
wia-chronic-pain assess patient-123 --pain-type nociplastic

# Get central sensitization status
wia-chronic-pain sensitization patient-123

# Calculate NRI
wia-chronic-pain nri patient-123 --format summary

# Get neuromodulation plan
wia-chronic-pain neuromod patient-123

# Get treatment recommendations
wia-chronic-pain recommend patient-123

# Opioid risk assessment
wia-chronic-pain opioid-risk patient-123

# Create taper plan
wia-chronic-pain taper patient-123
```

## Standard Components

### Phase 1: Data Format
Standardized JSON schemas for chronic pain profiles, sensitization data, and neuroimaging.

```json
{
  "@context": ["https://wia.live/chronic-pain/v1"],
  "type": ["ChronicPainProfile"],
  "patient_id": "uuid",
  "pain_type": "nociplastic",
  "pain_assessment": {
    "nrs": { "current": 7 },
    "duration_months": 36,
    "locations": [{"body_region": "lower_back"}]
  },
  "central_sensitization": {
    "csi_score": { "part_a": 62, "interpretation": "severe" },
    "temporal_summation": { "present": true, "ratio": 2.8 },
    "neuroplasticity_index": 72
  }
}
```

### Phase 2: API Interface
RESTful API with OpenAPI 3.1 specification.

```
POST /api/v1/chronic-pain/assess              - Create pain assessment
GET  /api/v1/chronic-pain/sensitization       - Get sensitization status
GET  /api/v1/chronic-pain/neuroplasticity/index - Get NRI
POST /api/v1/chronic-pain/neuromodulation/plan - Generate neuromod plan
POST /api/v1/chronic-pain/treatment/recommend  - Get recommendations
GET  /api/v1/chronic-pain/opioid-risk         - Opioid risk assessment
POST /api/v1/chronic-pain/opioid-taper/plan   - Create taper plan
```

### Phase 3: Clinical Protocols
Evidence-based protocols for:
- Comprehensive Pain Assessment (CSI, QST, psychosocial)
- Non-Invasive Neuromodulation (TMS, tDCS protocols)
- Cognitive Behavioral Therapy for Pain
- Graded Exercise Therapy
- Opioid Tapering with Multimodal Support
- Integrated Multimodal Program

### Phase 4: Integration
- **EHR**: FHIR R4, HL7 v2.x integration
- **Pain Clinics**: Neuromodulation devices, QST systems
- **PDMP**: Prescription Drug Monitoring integration
- **Digital Therapeutics**: Pain apps, telehealth
- **Research**: CDISC, IMMPACT, NIH HEAL Initiative

## Neuroplasticity Reversal Index (NRI)

```
NRI = 100 - [(CS×0.35) + (PSY×0.25) + (STRUCT×0.25) + (DUR×0.15)]

Where:
• CS = Central Sensitization (0-100)
• PSY = Psychosocial factors (0-100)
• STRUCT = Structural brain changes (0-100)
• DUR = Duration factor (0-100)

Interpretation:
80-100: Excellent reversal potential (early intervention)
60-79:  Good reversal potential (standard protocols)
40-59:  Moderate potential (intensive intervention)
20-39:  Limited potential (prolonged approach)
0-19:   Challenging (focus on function)
```

## Neuromodulation Protocols

| Modality | Target | Indication | Sessions |
|----------|--------|------------|----------|
| rTMS | M1, DLPFC | Neuropathic, fibromyalgia | 10-20 |
| tDCS | Motor cortex | Fibromyalgia, low back pain | 10-20 |
| TENS | Peripheral | Localized pain, adjunct | Ongoing |

## API Reference

### Base URL
```
Production: https://api.wia.live/chronic-pain/v1
Staging:    https://api-staging.wia.live/chronic-pain/v1
```

### Authentication
```http
Authorization: Bearer <api_key>
```

### Example Request
```bash
curl -X POST https://api.wia.live/chronic-pain/v1/neuromodulation/plan \
  -H "Authorization: Bearer $API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "patient_id": "patient-uuid",
    "target_symptoms": ["pain_intensity", "central_sensitization"],
    "preferences": {"clinic_based": true}
  }'
```

## Directory Structure

```
WIA-CHRONIC-PAIN/
├── spec/
│   ├── PHASE-1-DATA-FORMAT.md      # JSON schemas
│   ├── PHASE-2-API-INTERFACE.md    # OpenAPI specification
│   ├── PHASE-3-PROTOCOL.md         # Clinical protocols
│   └── PHASE-4-INTEGRATION.md      # EHR/Device integration
├── api/
│   └── typescript/
│       ├── src/
│       │   ├── types.ts            # Type definitions
│       │   └── index.ts            # SDK implementation
│       └── package.json
├── cli/
│   └── wia-chronic-pain.sh         # CLI tool
├── README.md
└── install.sh
```

## Evidence Base

The Neuroplasticity Reversal approach is supported by:

1. **Brain Structure Recovery**
   - Gray matter loss in chronic pain is reversible
   - CBT + exercise → measurable gray matter increase
   - Treatment response correlates with structural normalization

2. **Central Sensitization Reversal**
   - TMS/tDCS can modulate cortical excitability
   - Reduced temporal summation after neuromodulation
   - Improved descending inhibition (CPM)

3. **Opioid-Sparing Outcomes**
   - Multimodal programs achieve 50%+ opioid reduction
   - Maintained or improved function
   - Reduced healthcare utilization

## Contributing

We welcome contributions! Please see our [Contributing Guide](../../CONTRIBUTING.md).

## License

MIT License - see [LICENSE](../../LICENSE) for details.

---

**© 2026 WIA (World Certification Industry Association)**

홍익인간 (弘益人間) · Benefit All Humanity

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
