# WIA-TRADITIONAL-MEDICINE

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **홍익인간 (弘益人間)** - 널리 인간을 이롭게 하라 (Benefit All Humanity)

Global standard for integrative traditional medicine systems, enabling interoperability between Traditional Chinese Medicine (TCM), Ayurveda, Korean Sasang Medicine, Kampo, Unani, and Tibetan Medicine.

## Overview

Traditional medicine systems represent millennia of accumulated wisdom about human health and healing. Despite their profound insights, these systems remain fragmented, lacking standardized terminology, data formats, and integration protocols. WIA-TRADITIONAL-MEDICINE bridges this gap, creating a unified framework for personalized integrative medicine.

### Key Features

- **Multi-System Constitutional Assessment**: TCM 9 constitutions, Ayurveda 3 doshas, Sasang 4 types
- **AI-Powered Diagnostics**: Tongue analysis, pulse waveform analysis, voice pattern recognition
- **Drug-Herb Interaction Database**: Evidence-based safety checking with 5000+ interaction records
- **FHIR Integration**: Seamless EHR interoperability via HL7 FHIR profiles
- **ICD-11 TM Mapping**: WHO-endorsed traditional medicine codes
- **Multi-Omics Correlation**: Genomics, metabolomics, microbiome integration

## Quick Start

### Installation

```bash
# TypeScript/JavaScript
npm install @wia/traditional-medicine-sdk

# Python
pip install wia-traditional-medicine

# Java
<dependency>
  <groupId>com.wia</groupId>
  <artifactId>traditional-medicine</artifactId>
  <version>1.0.0</version>
</dependency>
```

### Basic Usage

```typescript
import WIATraditionalMedicine from '@wia/traditional-medicine-sdk';

const client = new WIATraditionalMedicine({
  apiKey: 'your-api-key',
  baseUrl: 'https://api.wia.live/tm/v1'
});

// Assess patient constitution
const assessment = await client.constitution.assess({
  patient_id: 'PAT-2025-001',
  system: 'sasang',
  questionnaire_type: 'QSCCII',
  responses: { q1: 4, q2: 3, /* ... */ },
  include_recommendations: true
});

console.log(assessment.primary_constitution); // "taeeum"
console.log(assessment.recommendations.diet);  // ["Limit greasy foods", ...]

// Analyze tongue image
const tongueAnalysis = await client.diagnosis.analyzeTongue(
  tongueImageFile,
  'PAT-2025-001'
);

console.log(tongueAnalysis.patterns_suggested);
// [{ pattern: "Spleen Qi Deficiency", confidence: 0.89 }]

// Check drug-herb interactions
const interactions = await client.herbs.checkInteractions({
  herbs: ['Panax ginseng', 'Angelica sinensis'],
  medications: ['Warfarin', 'Metformin']
});

if (interactions.overall_risk === 'high') {
  console.warn('Significant interactions detected!');
}
```

## Directory Structure

```
traditional-medicine/
├── index.html                    # Landing page
├── simulator/
│   └── index.html                # Interactive demo (5 tabs, 99 languages)
├── ebook/
│   ├── en/                       # English documentation (8 chapters)
│   └── ko/                       # Korean documentation (한국어 문서)
├── spec/
│   ├── PHASE-1-DATA-FORMAT.md    # JSON schemas
│   ├── PHASE-2-API-INTERFACE.md  # RESTful API definitions
│   ├── PHASE-3-PROTOCOL.md       # Clinical protocols
│   └── PHASE-4-INTEGRATION.md    # Healthcare system integration
├── api/
│   └── typescript/
│       ├── src/
│       │   ├── types.ts          # Type definitions
│       │   └── index.ts          # SDK client
│       └── package.json
└── README.md
```

## Data Schemas

### Constitutional Profile

```json
{
  "constitutional_profile": {
    "profile_id": "uuid-v4",
    "patient_id": "patient-reference",
    "tcm_constitution": {
      "primary_type": "phlegm_dampness",
      "score": { "qi_deficiency": 0.35, "..." : "..." },
      "confidence": 0.85
    },
    "ayurveda_prakriti": {
      "dominant": "kapha",
      "vata": { "score": 25 },
      "pitta": { "score": 30 },
      "kapha": { "score": 45 }
    },
    "sasang_constitution": {
      "type": "taeeum",
      "confidence": 0.78
    }
  }
}
```

### Traditional Diagnosis (Four Examinations)

```json
{
  "traditional_diagnosis": {
    "four_examinations": {
      "inspection": {
        "tongue": {
          "body_color": "pale",
          "coating_color": "white",
          "coating_thickness": "thick"
        }
      },
      "inquiry": {
        "chief_complaint": "Fatigue, heavy sensation",
        "ten_questions": { "cold_heat": "cold_preference" }
      },
      "palpation": {
        "pulse": { "rate": 68, "quality": ["slippery", "soft"] }
      }
    },
    "pattern_diagnosis": {
      "icd11_tm_code": "TM1:SF32.2",
      "pattern_name": {
        "en": "Spleen Qi Deficiency with Dampness",
        "zh": "脾氣虛夾濕",
        "ko": "비기허협습"
      }
    }
  }
}
```

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/constitution/assess` | POST | Submit questionnaire, receive constitution analysis |
| `/constitution/profile/{id}` | GET | Get patient's constitutional profile |
| `/diagnosis/tongue/analyze` | POST | Upload tongue image for AI analysis |
| `/diagnosis/traditional` | POST | Submit traditional diagnosis record |
| `/herbs/search` | GET | Search herbal medicine database |
| `/herbs/formula/recommend` | POST | Get formula recommendations by pattern |
| `/herbs/interactions` | POST | Check drug-herb interactions |
| `/safety/report` | POST | Report adverse events |

## Constitution Systems

### TCM Nine Constitutions (中醫體質)

| Type | Chinese | Description |
|------|---------|-------------|
| Balanced | 平和質 | Optimal health state |
| Qi Deficiency | 氣虛質 | Low energy, fatigue |
| Yang Deficiency | 陽虛質 | Cold intolerance |
| Yin Deficiency | 陰虛質 | Heat signs, dry mouth |
| Phlegm-Dampness | 痰濕質 | Overweight tendency |
| Damp-Heat | 濕熱質 | Oily skin, irritability |
| Blood Stasis | 血瘀質 | Dark complexion, pain |
| Qi Stagnation | 氣鬱質 | Emotional sensitivity |
| Special Diathesis | 特稟質 | Allergic tendency |

### Sasang Four Constitutions (四象體質)

| Type | Korean | Organ Strength | Organ Weakness |
|------|--------|----------------|----------------|
| Taeyang | 태양인 | Lung | Liver |
| Taeeum | 태음인 | Liver | Lung |
| Soyang | 소양인 | Spleen | Kidney |
| Soeum | 소음인 | Kidney | Spleen |

### Ayurveda Three Doshas

| Dosha | Elements | Characteristics |
|-------|----------|-----------------|
| Vata | Air + Space | Movement, creativity |
| Pitta | Fire + Water | Transformation, digestion |
| Kapha | Water + Earth | Structure, stability |

## FHIR Integration

WIA-TRADITIONAL-MEDICINE provides FHIR R4 profiles for seamless EHR integration:

```typescript
// Convert pattern diagnosis to FHIR Condition
const fhirCondition = client.fhir.toCondition(
  patternDiagnosis,
  'Patient/12345'
);

// Convert constitution to FHIR Observation
const fhirObservation = client.fhir.toObservation(
  tcmConstitution,
  'Patient/12345'
);
```

### FHIR Profiles

- `TMPatternDiagnosis` - Traditional medicine pattern as Condition
- `TMConstitutionObservation` - Constitution assessment as Observation
- `HerbalMedicationRequest` - Herbal formula prescription
- `TMPractitioner` - Traditional medicine practitioner

## Safety Features

### Drug-Herb Interaction Checking

```typescript
const result = await client.herbs.checkInteractions({
  herbs: ['Ginkgo biloba', 'Panax ginseng'],
  medications: ['Warfarin', 'Aspirin']
});

// Response
{
  "interactions": [
    {
      "herb": "Ginkgo biloba",
      "drug": "Warfarin",
      "severity": "major",
      "effect": "May increase bleeding risk",
      "mechanism": "Platelet aggregation inhibition",
      "recommendation": "Avoid combination or monitor INR closely"
    }
  ],
  "overall_risk": "high",
  "alternatives": [
    { "replace": "Ginkgo biloba", "with": "Bacopa monnieri", "reason": "Similar cognitive benefits, lower interaction risk" }
  ]
}
```

### Adverse Event Reporting

```typescript
await client.safety.reportAdverseEvent({
  patient_id: 'PAT-2025-001',
  reporter_id: 'PRAC-001',
  event_date: '2025-01-15',
  products: [
    { type: 'herbal_formula', name: 'Bu Zhong Yi Qi Tang', dose: '10g', duration_days: 14 }
  ],
  reaction: {
    description: 'Gastrointestinal discomfort',
    symptoms: ['nausea', 'bloating'],
    severity: 'mild',
    onset: '2_hours',
    outcome: 'recovered'
  }
});
```

## International Standards Compliance

- **ICD-11 TM Module**: WHO Traditional Medicine classification codes
- **HL7 FHIR R4**: Healthcare interoperability
- **ISO 17218**: Vocabulary for Traditional Chinese Medicine
- **Korean Pharmacopoeia**: Herbal medicine standards
- **AYUSH Standards**: Indian traditional medicine guidelines

## Documentation

### English
- [Chapter 1: Traditional Medicine Wisdom](ebook/en/chapter-01.html)
- [Chapter 2: Systems Biology Connection](ebook/en/chapter-02.html)
- [Chapter 3: Constitutional Medicine](ebook/en/chapter-03.html)
- [Chapter 4: Data Format](ebook/en/chapter-04.html)
- [Chapter 5: API Interface](ebook/en/chapter-05.html)
- [Chapter 6: Protocols](ebook/en/chapter-06.html)
- [Chapter 7: Integration](ebook/en/chapter-07.html)
- [Chapter 8: Future Vision](ebook/en/chapter-08.html)

### 한국어
- [제1장: 전통 의학의 지혜](ebook/ko/chapter-01.html)
- [제2장: 시스템 생물학 연결](ebook/ko/chapter-02.html)
- [제3장: 체질 의학](ebook/ko/chapter-03.html)
- [제4장: 데이터 형식](ebook/ko/chapter-04.html)
- [제5장: API 인터페이스](ebook/ko/chapter-05.html)
- [제6장: 프로토콜](ebook/ko/chapter-06.html)
- [제7장: 통합](ebook/ko/chapter-07.html)
- [제8장: 미래 비전](ebook/ko/chapter-08.html)

## Contributing

We welcome contributions from traditional medicine practitioners, healthcare IT developers, and researchers worldwide.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

MIT License - see [LICENSE](LICENSE) for details.

## Acknowledgments

- World Health Organization - Traditional Medicine Strategy
- Korean Institute of Oriental Medicine (KIOM)
- China Academy of Chinese Medical Sciences
- Ministry of AYUSH, India
- Traditional medicine practitioners worldwide

---

<p align="center">
  <strong>弘益人間</strong><br>
  <em>Benefit All Humanity</em>
</p>

<p align="center">
  © 2025 WIA - World Certification Industry Association<br>
  <a href="https://wia.live/standards/traditional-medicine">wia.live/standards/traditional-medicine</a>
</p>

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
