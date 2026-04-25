# WIA Refugee Credential

**난민 자격증명 - Dignified Verification for Displaced Persons**
*국가가 무너져도, 사람의 가치는 무너지지 않습니다.*

홍익인간 (弘益人間) - Benefit All Humanity

---

## Overview

WIA Refugee Credential provides a decentralized, dignified way for displaced persons to verify their education, skills, and professional credentials when traditional documentation is lost or inaccessible.

### The Problem

```
Syrian doctor → 10 years of study → War → No documents → Janitor in Germany
Ukrainian professor → 30 years of career → War → No documents → Factory worker in Poland
```

**When nations fall, people's credentials disappear with them.**

### The Solution

```
WIA-REFUGEE-CREDENTIAL

Education/Skills → Stored with WIA-PQ-CRYPTO
    ↓
Survives national collapse
    ↓
Verifiable anywhere in the world
```

---

## Specifications

| Phase | Document | Description |
|-------|----------|-------------|
| 1 | [PHASE-1-DATA-FORMAT.md](spec/PHASE-1-DATA-FORMAT.md) | Data structures (identity, education, competencies) |
| 2 | [PHASE-2-ALGORITHMS.md](spec/PHASE-2-ALGORITHMS.md) | Confidence, peer matching, assessment algorithms |
| 3 | [PHASE-3-PROTOCOL.md](spec/PHASE-3-PROTOCOL.md) | REST API, DID-Auth, multi-language support |
| 4 | [PHASE-4-INTEGRATION.md](spec/PHASE-4-INTEGRATION.md) | UNHCR, universities, employers, governments |

---

## Quick Start

### Rust API

```bash
cd api/rust

# Build
cargo build --release

# Run server
cargo run

# Run tests
cargo test
```

### API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/health` | Health check |
| GET | `/version` | API version & verification levels |
| POST | `/api/v1/credentials` | Create credential |
| GET | `/api/v1/credentials/:id` | Get credential by ID |
| GET | `/api/v1/credentials/:id/confidence` | Calculate confidence score |
| GET | `/api/v1/credentials/:id/peers/find` | Find peer verifiers |
| POST | `/api/v1/credentials/:id/peers/verify` | Submit peer verification |
| POST | `/api/v1/credentials/:id/assessment` | Submit assessment result |
| POST | `/api/v1/verify` | Institution verification |
| WS | `/ws` | Real-time updates |

### Example: Create Credential

```bash
curl -X POST http://localhost:8080/api/v1/credentials \
  -H "Content-Type: application/json" \
  -d '{
    "holder": {
      "display_name": "أحمد محمد",
      "birth_year": 1985,
      "nationality_claimed": "SY",
      "refugee_status": "unhcr_recognized",
      "displacement_reason": "armed_conflict",
      "current_country": "DE"
    },
    "education": [
      {
        "level": "professional",
        "field_of_study": "Medicine",
        "institution_name": "Damascus University",
        "institution_country": "SY",
        "start_year": 2003,
        "end_year": 2009,
        "completed": true
      }
    ],
    "languages": [
      {"language_code": "ar", "proficiency": "native"},
      {"language_code": "en", "proficiency": "advanced", "cefr_level": "c1"},
      {"language_code": "de", "proficiency": "intermediate", "cefr_level": "b1"}
    ]
  }'
```

### Example: Find Peer Verifiers

```bash
curl http://localhost:8080/api/v1/credentials/{id}/peers/find?limit=5

# Response:
{
  "success": true,
  "data": {
    "potential_peers": [
      {
        "peer_did": "did:wia:refugee:xyz123",
        "shared_context": "Damascus University (2005-2009)",
        "relevance_score": 0.85
      }
    ],
    "match_count": 1
  }
}
```

### Example: Calculate Confidence

```bash
curl http://localhost:8080/api/v1/credentials/{id}/confidence

# Response:
{
  "success": true,
  "data": {
    "overall_confidence": 0.72,
    "verification_level": 2,
    "breakdown": {
      "base_confidence": 0.60,
      "peer_bonus": 0.10,
      "assessment_bonus": 0.02,
      "document_bonus": 0.00
    }
  }
}
```

---

## Verification Levels

| Level | Name | Confidence | Description |
|-------|------|------------|-------------|
| 1 | Self-Declaration | 30% | Holder's own claims, unverified |
| 2 | Peer Verified | 60% | Verified by peers from same institution/profession |
| 3 | Assessment Verified | 80% | Passed competency assessment |
| 4 | Document Verified | 95% | Original documents verified by institution |

### Confidence Calculation

```
Overall Confidence = Base Confidence + Peer Bonus + Assessment Bonus + Document Bonus

Where:
- Base Confidence = 0.30 (L1), 0.60 (L2), 0.80 (L3), 0.95 (L4)
- Peer Bonus = min(peer_count × 0.05, 0.15)
- Assessment Bonus = min(assessment_count × 0.03, 0.10)
- Document Bonus = min(doc_count × 0.02, 0.05)
```

---

## Data Model

```
RefugeeCredential
├── HolderIdentity
│   ├── did (decentralized identifier)
│   ├── display_name (in original script)
│   ├── birth_year (not full date for privacy)
│   ├── nationality_claimed
│   ├── refugee_status (self_declared, unhcr_recognized, etc.)
│   ├── displacement_reason (armed_conflict, persecution, etc.)
│   └── current_country
├── EducationRecord[]
│   ├── level (secondary, bachelors, masters, doctorate, professional)
│   ├── field_of_study
│   ├── institution_name (original + transliteration)
│   ├── institution_country
│   ├── years (start, end)
│   └── verification_status
├── CompetencyRecord[]
│   ├── domain (medical, engineering, education, legal, etc.)
│   ├── name
│   ├── proficiency (basic, intermediate, advanced, expert)
│   └── evidence references
├── LanguageAbility[]
│   ├── language_code (ISO 639-1)
│   ├── proficiency
│   └── cefr_level (A1-C2)
├── WorkExperience[]
│   ├── role, organization, country
│   ├── years
│   └── responsibilities
├── VerificationInfo
│   ├── level (1-4)
│   ├── overall_confidence
│   ├── peer_verifications[]
│   ├── assessment_results[]
│   └── document_verifications[]
└── Metadata
    ├── version, language
    └── privacy_level
```

---

## Supported Languages

Primary languages for refugee populations:

| Code | Language | Script |
|------|----------|--------|
| ar | العربية (Arabic) | RTL |
| uk | Українська (Ukrainian) | Cyrillic |
| fa | فارسی (Persian) | RTL |
| ps | پښتو (Pashto) | RTL |
| ti | ትግርኛ (Tigrinya) | Ge'ez |
| fr | Français | Latin |
| en | English | Latin |
| es | Español | Latin |
| de | Deutsch | Latin |
| tr | Türkçe | Latin |
| ko | 한국어 | Hangul |

---

## Key Features

| Traditional | WIA |
|-------------|-----|
| Nation guarantees | Nation-independent |
| Documents required | Competency-based verification |
| Nation collapse = end | Permanent preservation |
| Paid services | Free for refugees |

---

## Integrations

- **UNHCR**: Refugee status verification
- **Universities**: Admission with WIA credentials
- **Professional Bodies**: Medical/engineering license pathways
- **Employers**: Refugee hiring network
- **Governments**: Immigration credential recognition
- **WIA Academy**: Free bridging courses

---

## Use Cases

### 1. Syrian Doctor in Germany

```yaml
scenario: Dr. Ahmed, general physician from Syria
problem: Medical license from Damascus University, no documents
solution:
  - Create WIA credential (Level 1)
  - Find peer verifiers (2 former colleagues)
  - Pass German medical competency assessment
  - University of Damascus confirms via WIA network
result: Level 3 credential, German medical board accepts for bridging program
```

### 2. Ukrainian Professor in Poland

```yaml
scenario: Prof. Olena, Computer Science PhD from Kyiv
problem: 30 years experience, no verifiable records
solution:
  - Create WIA credential with publication list
  - 5 former students verify via peer system
  - Portfolio review by WIA assessment
result: Level 3 credential, Polish university offers visiting position
```

---

## Security

- TLS 1.3 transport encryption
- DID-based authentication
- Quantum-resistant signatures (CRYSTALS-Dilithium)
- Zero-knowledge proofs for privacy
- GDPR compliant data handling

---

## Connected Systems

```
WIA-REFUGEE-CREDENTIAL ─┬─ WIA-PQ-CRYPTO (quantum-resistant cryptography)
                        │
                        ├─ WIA Academy (free education)
                        │
                        └─ WIA Book (free book access)
```

---

## License

MIT License

Copyright (c) 2025 WIA - World Certification Industry Association

---

**Document ID**: WIA-REFUGEE-CREDENTIAL
**Version**: 1.0.0
**Date**: 2025-12-16
**Philosophy**: 이 표준은 돈을 위한 것이 아닙니다. 국가가 무너져도 사람의 가치는 무너지지 않습니다.
