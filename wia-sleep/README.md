# WIA-SLEEP

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


## World Interoperability for Sleep Optimization

**Version:** 1.0.0 | **Status:** Active | **Category:** Healthcare/MED

홍익인간 (弘益人間) - Benefit All Humanity

---

## Overview

WIA-SLEEP is a comprehensive standard for sleep health data interoperability and personalized sleep optimization. Moving beyond traditional "sleep hygiene" approaches, this standard embraces **Chronobiological Personalization** and **Circadian-Metabolic Synchronization**.

### Key Paradigm Shifts

| Traditional | WIA-SLEEP Approach |
|-------------|-------------------|
| One-size-fits-all recommendations | Chronotype-based personalization |
| Sleep duration focus | Sleep architecture optimization |
| Generic sleep hygiene | Circadian phase-aware interventions |
| Isolated sleep data | Integrated metabolic synchronization |

---

## Features

- **Chronotype Assessment** - MCTQ-based classification with biological marker validation
- **Circadian Phase Tracking** - DLMO estimation, entrainment status, alertness prediction
- **Sleep Architecture Analysis** - Stage percentages, cycle detection, quality metrics
- **Personalized Optimization** - Light therapy, timing recommendations, intervention planning
- **Device Interoperability** - Mappings for Oura, Apple Watch, WHOOP, PSG systems
- **Healthcare Integration** - FHIR R4 and HL7 v2.x mappings

---

## Directory Structure

```
wia-sleep/
├── README.md                          # This file
├── schemas/                           # JSON Schemas
│   ├── wia-sleep-record.schema.json   # Main sleep record schema
│   ├── chronotype-assessment.schema.json
│   ├── sleep-architecture.schema.json
│   ├── circadian-markers.schema.json
│   └── metabolic-sync.schema.json
├── api/
│   ├── openapi.yaml                   # OpenAPI 3.0 specification
│   └── examples/                      # API examples
│       ├── sleep-record-example.json
│       ├── chronotype-result.json
│       └── optimization-plan.json
├── sdk/
│   └── typescript/                    # TypeScript SDK
│       ├── src/
│       │   ├── index.ts
│       │   ├── types/                 # Type definitions
│       │   ├── services/              # Core services
│       │   └── utils/                 # Utility functions
│       ├── package.json
│       └── tsconfig.json
├── algorithms/                        # Algorithm specifications
│   ├── chronotype-classification.md
│   ├── circadian-phase-estimation.md
│   ├── sleep-need-prediction.md
│   └── light-therapy-timing.md
├── mappings/                          # System mappings
│   ├── wia-sleep-to-fhir.json
│   ├── wia-sleep-to-hl7.json
│   └── device-mappings/
│       ├── oura-mapping.json
│       ├── whoop-mapping.json
│       ├── apple-watch-mapping.json
│       └── polysomnography-mapping.json
├── validation/
│   ├── test-cases/
│   └── conformance-criteria.md
└── docs/
    ├── implementation-guide.md
    ├── chronobiology-primer.md
    └── integration-patterns.md
```

---

## Quick Start

### Installation

```bash
npm install @wia/sleep-sdk
```

### Basic Usage

```typescript
import {
  ChronotypeService,
  SleepAnalyzer,
  OptimizationEngine,
  ChronotypeClass,
  OptimizationGoal
} from '@wia/sleep-sdk';

// Assess chronotype from MCTQ responses
const result = ChronotypeService.assessChronotype({
  subjectId: 'user-123',
  mctqResponses: {
    workdays: {
      bedtime: '23:30:00',
      wakeTime: '06:45:00',
      sleepDuration_hours: 7.0
    },
    freeDays: {
      bedtime: '00:30:00',
      wakeTime: '09:00:00',
      sleepDuration_hours: 8.5
    },
    workDaysPerWeek: 5,
    alarmUsage: true
  }
});

console.log(`Chronotype: ${result.chronotype}`);
console.log(`Optimal bedtime: ${result.optimalSleepWindow.bedtime}`);
console.log(`Social jetlag: ${result.socialJetlag_hours} hours`);

// Generate optimization plan
const plan = OptimizationEngine.generatePlan(
  {
    subjectId: 'user-123',
    goals: [
      OptimizationGoal.PHASE_ADVANCE,
      OptimizationGoal.REDUCE_SOCIAL_JETLAG
    ]
  },
  ChronotypeService.createProfile(result)
);

console.log('Light therapy:', plan.interventions.lightTherapy);
console.log('Target bedtime:', plan.targetState.targetBedtime);
```

---

## Core Concepts

### Chronotype Classification

Based on the Munich Chronotype Questionnaire (MCTQ):

| Classification | MSFsc Range | Description |
|---------------|-------------|-------------|
| Extreme Early | < 2.5 | Strong morning preference |
| Moderate Early | 2.5 - 3.5 | Morning-oriented |
| Intermediate | 3.5 - 5.0 | Neither type |
| Moderate Late | 5.0 - 6.5 | Evening-oriented |
| Extreme Late | > 6.5 | Strong evening preference |

### Circadian Phase Markers

- **DLMO** - Dim Light Melatonin Onset (gold standard phase marker)
- **CBT Nadir** - Core Body Temperature minimum (~7h after DLMO)
- **Phase Angle** - Relationship between DLMO and sleep onset

### Sleep Architecture

```
Total Sleep Time (TST)
├── N1 (Light Sleep)     ~5%
├── N2 (Intermediate)    ~50%
├── N3/SWS (Deep Sleep)  ~20%
└── REM                  ~25%
```

---

## API Overview

### Key Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/records` | POST | Submit sleep record |
| `/chronotype/assess` | POST | Assess chronotype |
| `/circadian/phase` | GET | Get current circadian phase |
| `/optimization/recommend` | POST | Get personalized recommendations |
| `/light/prescription` | POST | Generate light therapy prescription |
| `/disorders/screen` | POST | Screen for sleep disorders |

See [openapi.yaml](./api/openapi.yaml) for full specification.

---

## Device Integration

### Supported Devices

| Device | Data Quality | Mapping |
|--------|-------------|---------|
| Polysomnography (PSG) | Clinical-grade | Full architecture |
| Oura Ring | Consumer+ | Architecture, HRV |
| Apple Watch | Consumer | Architecture (watchOS 8+) |
| WHOOP | Consumer+ | Architecture, HRV, Recovery |

### Data Source Hierarchy

1. **PSG** - Gold standard for clinical assessment
2. **Clinical Wearables** - Research-grade devices
3. **Consumer Wearables** - Daily tracking
4. **Actigraphy** - Rest-activity patterns
5. **Questionnaires** - Subjective assessment

---

## Interoperability

### FHIR R4 Mapping

```json
{
  "resourceType": "Observation",
  "code": {
    "coding": [{
      "system": "http://loinc.org",
      "code": "93832-4",
      "display": "Sleep duration"
    }]
  },
  "valueQuantity": {
    "value": 432,
    "unit": "min",
    "system": "http://unitsofmeasure.org"
  }
}
```

### Related WIA Standards

- **WIA-NUTRITION** - Meal timing synchronization
- **WIA-MENTAL** - Sleep-mood correlation
- **WIA-FITNESS** - Exercise timing optimization

---

## Contributing to 33 Global Challenges

| Challenge | Contribution |
|-----------|-------------|
| #2 Mental Health | Sleep-mental health bidirectional link |
| #6 Nutrition | Chrononutrition integration |
| #8 Aging | Sleep and longevity connection |
| #19 Stress | Sleep-stress axis optimization |
| #25 Loneliness | Sleep disorder-isolation correlation |

---

## Conformance

See [conformance-criteria.md](./validation/conformance-criteria.md) for:

- Level 1: Basic Conformance
- Level 2: Standard Conformance
- Level 3: Full Conformance

---

## References

1. 선행 연구. Life between clocks. Journal of Biological Rhythms.
2. Borbély, A.A. (1982). A two process model of sleep regulation.
3. Czeisler, C.A., et al. (1989). Bright light induction of circadian resetting.
4. AASM (2020). AASM Scoring Manual Version 2.6.

---

## License

MIT License - See LICENSE file

---

## Contact

- **Standard**: WIA-SLEEP
- **Website**: https://wiastandards.com/wia-sleep
- **Repository**: https://github.com/WIA-Official/wia-standards

---

**홍익인간 (弘益人間) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
