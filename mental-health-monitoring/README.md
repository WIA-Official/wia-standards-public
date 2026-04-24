# 🧠 WIA-MENTAL-HEALTH

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


**Mental Health Monitoring Standard**

*Integrated mental health assessment, treatment, and monitoring through neuroplasticity-centered care*

---

## Overview

WIA-MENTAL-HEALTH is a comprehensive standard for mental health data interoperability, assessment protocols, and treatment integration. The core principle is that **neuroplasticity promotion** serves as the unified pathway to mental health recovery across all therapeutic modalities.

```
Fragmented Approaches    →    Unified Principle           →    Universal Solution
Dozens of therapies      →    Neuroplasticity Promotion   →    Mental Health Recovery
```

## Key Features

- 📊 **Standardized Assessment** - PHQ-9, GAD-7, PCL-5, and more with unified 0.0-1.0 scoring
- 🧬 **Biomarker Integration** - Cortisol, inflammatory markers, HRV, sleep, gut-brain axis
- 🧘 **Psychedelic Therapy Protocols** - Psilocybin, MDMA, ketamine with safety frameworks
- 📱 **Digital Therapeutics** - FDA-cleared apps, AI chatbots, VR therapy integration
- 🚨 **Crisis Detection** - Real-time risk assessment with 988 Lifeline integration
- 🔒 **Privacy-First** - HIPAA/GDPR compliant with AES-256 encryption

## Quick Start

### Installation

```bash
# Install the CLI tool
./install.sh

# Or install the TypeScript SDK
cd api/typescript
npm install
```

### CLI Usage

```bash
# Calculate PHQ-9 and GAD-7 scores
wia-mental-health score --phq9 "2,1,2,1,0,1,0,1,0" --gad7 "1,2,1,0,1,1,0"

# Crisis screening
wia-mental-health crisis --suicide 0 --selfharm 0

# Biomarker analysis
wia-mental-health biomarker --cortisol 15 --hrv 35 --sleep 88

# Validate data file
wia-mental-health validate patient-data.json
```

### TypeScript SDK

```typescript
import {
  createClient,
  calculateMentalHealthIndex,
  determineCrisisLevel
} from '@wia/mental-health';

// Create SDK client
const client = createClient({ apiKey: 'your-api-key' });

// Calculate mental health scores
const scores = calculateMentalHealthIndex(
  [2, 1, 2, 1, 0, 1, 0, 1, 0], // PHQ-9 responses
  [1, 2, 1, 0, 1, 1, 0]        // GAD-7 responses
);

console.log(scores.depressionScore);
// { value: 0.33, rawScore: 9, severity: 'mild', instrument: 'PHQ-9' }

// Submit assessment
const result = await client.submitAssessment({
  subjectId: 'patient-12345',
  assessmentType: 'monitoring',
  instruments: ['PHQ-9', 'GAD-7'],
  responses: { ... }
});
```

## Specification Phases

| Phase | Description | Status |
|-------|-------------|--------|
| [PHASE-1](spec/PHASE-1-DATA-FORMAT.md) | Data Format | ✅ Complete |
| [PHASE-2](spec/PHASE-2-API-INTERFACE.md) | API Interface | ✅ Complete |
| [PHASE-3](spec/PHASE-3-PROTOCOL.md) | Protocol | ✅ Complete |
| [PHASE-4](spec/PHASE-4-INTEGRATION.md) | Integration | ✅ Complete |

## Data Schema

### Mental Health Index

```json
{
  "mental_health_index": {
    "depression_score": {
      "value": 0.45,
      "instrument": "PHQ-9",
      "raw_score": 12,
      "severity": "moderate"
    },
    "anxiety_score": {
      "value": 0.28,
      "instrument": "GAD-7",
      "raw_score": 8,
      "severity": "mild"
    },
    "ptsd_score": {
      "value": 0.0,
      "instrument": "PCL-5",
      "meets_criteria": false
    },
    "wellbeing_score": {
      "value": 0.72,
      "instrument": "WHO-5"
    },
    "resilience_score": {
      "value": 0.65,
      "instrument": "CD-RISC"
    }
  }
}
```

### Score Normalization

All scores are normalized to 0.0-1.0 range:

| Score Range | Interpretation |
|-------------|----------------|
| 0.00 - 0.20 | Minimal/None |
| 0.21 - 0.40 | Mild |
| 0.41 - 0.60 | Moderate |
| 0.61 - 0.80 | Moderately Severe |
| 0.81 - 1.00 | Severe |

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/assessments` | POST | Submit assessment |
| `/assessments/history/{id}` | GET | Get assessment history |
| `/assessments/crisis-screen` | POST | Crisis screening |
| `/treatments/recommend` | POST | Get treatment recommendations |
| `/treatments/sessions` | POST | Log treatment session |
| `/biomarkers` | POST | Submit biomarker data |

## Treatment Protocols

### Evidence-Based Modalities

- **Psychotherapy**: CBT, DBT, ACT, EMDR, IPT
- **Pharmacotherapy**: SSRIs, SNRIs, mood stabilizers
- **Neuromodulation**: TMS, rTMS, tDCS, neurofeedback
- **Complementary**: Mindfulness, meditation, exercise
- **Psychedelic**: Psilocybin, MDMA, ketamine (clinical settings)
- **Digital**: FDA-cleared apps, VR therapy, AI chatbots

### Stepped Care Model

```
STEP 4: Specialist/Intensive Treatment
       ├── Inpatient care
       ├── Intensive outpatient
       └── Psychedelic-assisted therapy

STEP 3: High-Intensity Interventions
       ├── Individual CBT/DBT/EMDR
       ├── Medication + therapy
       └── TMS/tDCS neuromodulation

STEP 2: Low-Intensity Interventions
       ├── Guided self-help (digital)
       ├── Group therapy
       └── Peer support

STEP 1: Universal Prevention
       ├── Psychoeducation
       ├── Lifestyle interventions
       └── Stress management apps
```

## Privacy & Security

- **Encryption**: AES-256-GCM at rest, TLS 1.3 in transit
- **Compliance**: HIPAA, GDPR, PIPEDA
- **Pseudonymization**: Required for all patient identifiers
- **Consent Management**: Granular, time-limited, withdrawable

## Integration

- **EHR Systems**: Epic, Cerner, Allscripts (FHIR R4)
- **Wearables**: Apple HealthKit, Google Fit, Fitbit, Garmin, Oura
- **Telehealth**: Zoom for Healthcare, Doxy.me, Teladoc
- **Crisis Services**: 988 Lifeline, mobile crisis teams

## Resources

- [Try Simulator](simulator/)
- [TypeScript SDK](api/typescript/)
- [CLI Tool](cli/wia-mental-health.sh)
- [Full Specification](spec/)

## 🌍 Philosophy

**홍익인간 (弘益人間)** - Benefit All Humanity

This standard embodies the Korean philosophy of 弘益人間, aiming to benefit all humanity through innovation and standardization.

## Contributing

We welcome contributions! Please see our [contribution guidelines](https://github.com/WIA-Official/wia-standards/blob/main/CONTRIBUTING.md).

## License

MIT License - © 2025 WIA (World Certification Industry Association)

---

**홍익인간 (弘益人間) - Benefit All Humanity**

*WIA - World Certification Industry Association*

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
