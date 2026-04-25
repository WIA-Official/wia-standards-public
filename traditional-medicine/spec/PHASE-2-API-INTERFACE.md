# WIA-TRADITIONAL-MEDICINE: Phase 2 - API Interface

## 弘益人間 (Benefit All Humanity)

**Version:** 1.0.0
**Status:** Official
**Last Updated:** 2025-01

---

## 1. Overview

Phase 2 defines RESTful APIs for integrating traditional medicine data into modern healthcare systems. The API follows OpenAPI 3.0 specification and provides endpoints for constitution assessment, diagnosis, herbal medicine queries, and safety checking.

### 1.1 Base URL

```
Production: https://api.wia.live/tm/v1
Staging: https://api-staging.wia.live/tm/v1
```

### 1.2 Authentication

All API requests require OAuth 2.0 Bearer token authentication:

```http
Authorization: Bearer <access_token>
```

---

## 2. Constitution Assessment API

### 2.1 Submit Constitution Assessment

```yaml
POST /constitution/assess
Content-Type: application/json

Request:
{
  "patient_id": "PAT-2025-001",
  "system": "tcm|ayurveda|sasang",
  "questionnaire_type": "CCMQ|prakriti|QSCCII",
  "responses": {
    "q1": 4,
    "q2": 3,
    ...
  },
  "include_recommendations": true
}

Response: 200 OK
{
  "profile_id": "PROF-2025-12345",
  "timestamp": "2025-01-15T09:30:00Z",
  "primary_constitution": "phlegm_dampness",
  "secondary_constitutions": ["qi_deficiency"],
  "scores": {
    "phlegm_dampness": 0.72,
    "qi_deficiency": 0.45,
    "yin_deficiency": 0.25,
    ...
  },
  "confidence": 0.85,
  "recommendations": {
    "diet": [
      "Reduce greasy and sweet foods",
      "Increase light, bland foods",
      "Avoid cold and raw foods"
    ],
    "lifestyle": [
      "Regular aerobic exercise",
      "Avoid humid environments"
    ],
    "herbs_to_consider": ["茯苓", "白朮", "陳皮"]
  }
}
```

### 2.2 Get Constitution Profile

```yaml
GET /constitution/profile/{patient_id}

Response: 200 OK
{
  "profile_id": "PROF-2025-12345",
  "patient_id": "PAT-2025-001",
  "assessments": [
    {
      "timestamp": "2025-01-15T09:30:00Z",
      "system": "tcm",
      "primary_constitution": "phlegm_dampness",
      "confidence": 0.85
    }
  ],
  "genomic_correlates": {
    "available": true,
    "last_updated": "2024-12-01"
  }
}
```

---

## 3. Diagnosis API

### 3.1 Submit Traditional Diagnosis

```yaml
POST /diagnosis/traditional
Content-Type: application/json

Request:
{
  "patient_id": "PAT-2025-001",
  "practitioner_id": "PRAC-001",
  "four_examinations": {
    "inspection": {
      "complexion": "pale_yellow",
      "tongue": {
        "body_color": "pale",
        "coating_color": "white",
        "coating_thickness": "thick"
      }
    },
    "inquiry": {
      "chief_complaint": "Fatigue, heavy sensation",
      "ten_questions": {
        "cold_heat": "cold_preference",
        "perspiration": "spontaneous_sweating"
      }
    },
    "palpation": {
      "pulse": {
        "rate": 68,
        "quality": ["slippery", "soft"]
      }
    }
  }
}

Response: 201 Created
{
  "diagnosis_id": "DIAG-2025-789",
  "pattern_diagnosis": {
    "icd11_tm_code": "TM1:SF32.2",
    "pattern_name": {
      "en": "Spleen Qi Deficiency with Dampness",
      "zh": "脾氣虛夾濕",
      "ko": "비기허협습"
    }
  },
  "confidence": 0.87
}
```

### 3.2 Tongue Image Analysis

```yaml
POST /diagnosis/tongue/analyze
Content-Type: multipart/form-data

Request:
  image: <tongue_photo.jpg>
  patient_id: PAT-2025-001

Response: 200 OK
{
  "analysis_id": "TONG-2025-456",
  "image_quality": {
    "score": 0.92,
    "issues": []
  },
  "findings": {
    "body_color": "pale",
    "body_shape": "teeth_marked",
    "coating": {
      "color": "white",
      "thickness": "thick",
      "distribution": "root_thicker"
    },
    "moisture": "wet",
    "special_features": ["teeth_marks"]
  },
  "patterns_suggested": [
    {
      "pattern": "spleen_qi_deficiency",
      "confidence": 0.89,
      "evidence": ["pale tongue", "teeth marks", "white coating"]
    },
    {
      "pattern": "dampness",
      "confidence": 0.82,
      "evidence": ["thick coating", "wet tongue"]
    }
  ],
  "icd11_tm_codes": ["TM1:SF32.2", "TM1:SJ10"]
}
```

---

## 4. Herbal Medicine API

### 4.1 Search Herbs

```yaml
GET /herbs/search?query=人參&property=tonify_qi&limit=10

Response: 200 OK
{
  "total": 15,
  "results": [
    {
      "herb_id": "HB-RenShen-001",
      "names": {
        "scientific": "Panax ginseng C.A.Mey.",
        "chinese": "人參",
        "pinyin": "Rén Shēn",
        "korean": "인삼"
      },
      "traditional_properties": {
        "taste": ["sweet", "slightly_bitter"],
        "temperature": "warm",
        "meridians": ["spleen", "lung", "heart"]
      },
      "actions": ["Tonify Yuan Qi", "Strengthen Spleen and Lung"]
    }
  ]
}
```

### 4.2 Get Formula Recommendations

```yaml
POST /herbs/formula/recommend

Request:
{
  "pattern": "spleen_qi_deficiency_with_dampness",
  "constitution": "phlegm_dampness",
  "current_medications": ["Metformin"],
  "allergies": [],
  "pregnancy_status": false
}

Response: 200 OK
{
  "recommendations": [
    {
      "formula": {
        "name_zh": "六君子湯",
        "name_pinyin": "Liu Jun Zi Tang",
        "name_en": "Six Gentlemen Decoction"
      },
      "match_score": 0.92,
      "composition": [
        {"herb": "人參", "pinyin": "Rén Shēn", "dose_g": 9},
        {"herb": "白朮", "pinyin": "Bái Zhú", "dose_g": 9},
        {"herb": "茯苓", "pinyin": "Fú Líng", "dose_g": 9},
        {"herb": "炙甘草", "pinyin": "Zhì Gān Cǎo", "dose_g": 6},
        {"herb": "陳皮", "pinyin": "Chén Pí", "dose_g": 6},
        {"herb": "半夏", "pinyin": "Bàn Xià", "dose_g": 6}
      ],
      "modifications_suggested": [
        "Add 薏苡仁 9g for stronger dampness drainage"
      ],
      "safety_check": {
        "status": "safe",
        "interactions": [],
        "warnings": []
      }
    }
  ]
}
```

### 4.3 Check Drug-Herb Interactions

```yaml
POST /herbs/interactions

Request:
{
  "herbs": ["人參", "當歸", "川芎"],
  "medications": ["Warfarin", "Aspirin"]
}

Response: 200 OK
{
  "interactions": [
    {
      "type": "herb_drug",
      "herb": "當歸",
      "drug": "Warfarin",
      "severity": "major",
      "effect": "May enhance anticoagulant effect",
      "mechanism": "Coumarin compounds inhibit vitamin K metabolism",
      "clinical_significance": "Increased bleeding risk",
      "recommendation": "Monitor INR closely if combination unavoidable; consider alternative herbs",
      "evidence_level": "moderate",
      "references": ["PMID: 12345678", "Natural Medicines Database"]
    },
    {
      "type": "herb_drug",
      "herb": "人參",
      "drug": "Warfarin",
      "severity": "moderate",
      "effect": "May reduce anticoagulant effect",
      "mechanism": "Possible vitamin K-like activity",
      "recommendation": "Monitor INR if combining"
    }
  ],
  "herb_herb_interactions": [],
  "overall_risk": "high",
  "summary": "Combination of 當歸 with anticoagulants poses significant bleeding risk.",
  "alternatives": [
    {
      "replace": "當歸",
      "with": "黃耆",
      "reason": "Lower bleeding risk while maintaining blood-nourishing effect"
    }
  ]
}
```

---

## 5. Safety Reporting API

### 5.1 Report Adverse Event

```yaml
POST /safety/report

Request:
{
  "patient_id": "PAT-2025-001",
  "reporter_id": "PRAC-001",
  "event_date": "2025-01-10",
  "products": [
    {
      "type": "herbal_formula",
      "name": "六君子湯",
      "dose": "9g three times daily",
      "duration_days": 7
    }
  ],
  "reaction": {
    "description": "Mild gastrointestinal discomfort",
    "symptoms": ["nausea", "bloating"],
    "onset": "2 days after starting",
    "outcome": "resolved",
    "severity": "mild"
  },
  "concomitant_medications": ["Metformin 500mg twice daily"]
}

Response: 201 Created
{
  "report_id": "AE-2025-001",
  "status": "submitted",
  "causality_assessment": "possible",
  "follow_up_required": false
}
```

---

## 6. Rate Limits and Quotas

| Tier | Requests/Hour | Concurrent Connections |
|------|---------------|------------------------|
| Free | 100 | 2 |
| Standard | 1,000 | 10 |
| Professional | 10,000 | 50 |
| Enterprise | Unlimited | Unlimited |

---

## 7. Error Responses

```json
{
  "error": {
    "code": "INVALID_PATTERN",
    "message": "Unknown pattern code provided",
    "details": {
      "field": "pattern",
      "value": "invalid_pattern_code"
    }
  }
}
```

| Code | HTTP Status | Description |
|------|-------------|-------------|
| AUTH_REQUIRED | 401 | Missing or invalid authentication |
| FORBIDDEN | 403 | Insufficient permissions |
| NOT_FOUND | 404 | Resource not found |
| VALIDATION_ERROR | 422 | Invalid request data |
| RATE_LIMIT | 429 | Rate limit exceeded |

---

## 8. SDK Support

### 8.1 TypeScript/JavaScript

```typescript
import { WIATraditionalMedicine } from '@wia/traditional-medicine-sdk';

const client = new WIATraditionalMedicine({
  apiKey: 'your-api-key'
});

const result = await client.constitution.assess({
  patientId: 'PAT-001',
  system: 'sasang',
  responses: { q1: 4, q2: 3 }
});
```

### 8.2 Python

```python
from wia_traditional_medicine import WIAClient

client = WIAClient(api_key='your-api-key')
result = client.constitution.assess(
    patient_id='PAT-001',
    system='sasang',
    responses={'q1': 4, 'q2': 3}
)
```

---

**弘益人間 - Benefit All Humanity**

© 2025 WIA - World Certification Industry Association

## P.2 API Surface Cross-References

The API surface defined in this Phase consumes and emits the data formats from
Phase 1 and is transported by the protocol layer in Phase 3. Operators deploy
the surface using the integration patterns in Phase 4.

### P.2.1 Resource Naming

Resource paths follow REST conventions with snake_case segments. Identifier
segments use the canonical UUID encoding from Phase 1.

```
/v1/{collection}                        # collection
/v1/{collection}/{id}                    # member
/v1/{collection}/{id}/{sub_collection}   # nested collection
/v1/{collection}/{id}:{action}           # custom action (POST)
```

### P.2.2 Pagination

List endpoints support cursor-based pagination:

| Param | Default | Max | Description |
|-------|---------|-----|-------------|
| `page_size` | 50 | 500 | Items per page |
| `page_token` | empty | — | Opaque continuation token |

Servers MUST return `next_page_token` when the result set is truncated and an
empty string when the final page has been delivered.

### P.2.3 Idempotency

State-changing operations accept the `Idempotency-Key` header (RFC-style).
Servers MUST cache the response keyed by `(principal, key)` for at least 24 h
and replay the same response on retry.

### P.2.4 Field Masks

Partial-update operations use field masks (Google AIP-161 style) to avoid
clobbering unspecified fields. Masks are dot-paths into the canonical schema
with `*` wildcards.


---

## Appendix · Common WIA Standard Provisions

> The following provisions apply to every WIA standard and are kept in sync
> across the WIA-Standards corpus. Standard-specific deviations, where they
> exist, are listed in the standard's normative body.

### A. Conformance & Compliance

#### A.1 Conformance Levels

WIA-Standards defines four conformance levels:

| Level | Required | Description |
|-------|----------|-------------|
| L1 — Format | Phase 1 | Implementation produces and consumes the canonical data format losslessly |
| L2 — Interface | Phase 1 + 2 | Implementation exposes the API surface with required behaviour |
| L3 — Protocol | Phase 1 + 2 + 3 | Implementation interoperates over at least one normative transport binding |
| L4 — Integration | All Phases | Implementation passes the conformance test suite end-to-end in a production-shaped deployment |

Conformance claims MUST cite the level and the version of the standard
against which the claim is made (e.g. "L3 conformant against v1.0").

#### A.2 Compliance Verification

The conformance test suite is published alongside this standard at
`/cli/conformance/` and `/api/conformance/`. Implementations claiming
L2 or higher MUST publish their test report. Independent re-tests are
encouraged; the WIA Working Group accepts third-party verification reports
under the policy in §E.

### B. Security Considerations

#### B.1 Threat Model

Implementers SHOULD apply STRIDE analysis covering: spoofing of identity,
tampering with messages or stored state, repudiation of operations,
information disclosure, denial of service, and elevation of privilege.

| Threat | Default Control | Where Strengthened |
|--------|-----------------|--------------------|
| Spoofing | Mutual TLS or signed tokens | Phase 3 §P.3 |
| Tampering | TLS in transit, AEAD at rest | Phase 1 §P.1 |
| Repudiation | Append-only audit log with notarization | Phase 4 §P.4 |
| Disclosure | Field-level encryption for PII / secrets | Phase 1 §P.1 |
| DoS | Rate limit per principal & global circuit breaker | Phase 2 §P.2 |
| EoP | Least-privilege RBAC + scoped tokens | Phase 2 §P.2 |

#### B.2 Cryptographic Suites

Mandatory: TLS 1.3 with AEAD ciphers (AES-128-GCM, AES-256-GCM,
CHACHA20-POLY1305). Forbidden: TLS 1.0, TLS 1.1, RC4, MD5, SHA-1 for
signatures, RSA below 2048 bits, ECDSA on curves smaller than P-256.

Post-quantum migration: implementations SHOULD adopt hybrid key
exchange combining a classical primitive with ML-KEM (FIPS 203) once a
profile is published; signature migration to ML-DSA (FIPS 204) is
expected within the L4 conformance window of v2.0.

#### B.3 Audit Requirements

L3 and L4 implementations MUST log: (a) every authentication decision,
(b) every authorization decision, (c) every state-changing operation,
(d) every export of data outside its sovereignty boundary. Logs are
write-once for at least 1 year and 90 days indexed for incident search.

### C. Versioning & Lifecycle

Versions follow Semantic Versioning 2.0.0 (MAJOR.MINOR.PATCH).

| Phase | Duration | Conformance Status |
|-------|---------:|-------------------|
| Draft | until ratification | Non-binding |
| Active | indefinite | Binding for new deployments |
| Maintenance | 24 months from successor's Active date | Binding for existing deployments |
| Retired | indefinite | Non-binding; conformance claims rescinded |

Deprecation MUST be announced at least one minor version before a feature
is removed in a major version.

### D. Internationalization & Accessibility

Implementations SHOULD support locale negotiation via the
`Accept-Language` header (RFC 4647). Date, time, number, and currency
formatting follow CLDR. User-facing surfaces MUST satisfy WCAG 2.1 AA at
minimum and SHOULD progress towards WCAG 2.2 AA. Right-to-left scripts
(Arabic, Hebrew, Persian, Urdu) and East-Asian wide characters MUST be
laid out correctly without line-breaking heuristics that split graphemes.

### E. Governance & IP Policy

This standard is maintained by the WIA Working Group under the WIA
governance charter. Editorial changes are merged via pull request. Normative
changes require working-group consensus and a 30-day public review.
Contributions are accepted under the Apache License 2.0 with explicit
patent grant. Members participate under the WIA Patent Policy,
which requires royalty-free licensing of any essential claim necessary
to implement a normative requirement.

### F. Normative References

The following references are normative; implementations MUST satisfy
the cited clauses:

- ISO/IEC 27001:2022 — Information security management systems
- ISO/IEC 27017:2015 — Cloud-services security controls
- ISO/IEC 27701:2019 — Privacy information management
- ISO/IEC 19790:2012 — Security requirements for cryptographic modules
- ISO 8601-1:2019 — Date and time representation
- IETF RFC 8446 — TLS 1.3
- IETF RFC 7519 — JSON Web Token
- IETF RFC 6749 — OAuth 2.0
- IETF RFC 9110 — HTTP Semantics
- IETF RFC 9112 — HTTP/1.1 message syntax
- IETF RFC 9113 — HTTP/2
- IETF RFC 9114 — HTTP/3
- IETF RFC 9000 — QUIC transport
- IETF RFC 4122 — UUID URN namespace
- IETF RFC 3339 — Date and time on the Internet
- IETF RFC 6838 — Media-type specifications and registration
- W3C TraceContext — Distributed tracing context
- W3C WCAG 2.1 — Accessibility guidelines
- FIPS PUB 197 — AES
- FIPS PUB 180-4 — SHA-2 family
- FIPS PUB 203 — ML-KEM (post-quantum KEM)
- FIPS PUB 204 — ML-DSA (post-quantum signature)

### G. Glossary

| Term | Definition |
|------|------------|
| Conformance | The state of satisfying every normative requirement at a given level |
| Implementation | A software, hardware, or composite artefact that claims conformance |
| Principal | The authenticated entity bound to a security context |
| Subject | The resource or person to which an operation applies |
| Sovereignty Boundary | The legal / regulatory perimeter outside of which data export is restricted |

---

*This Appendix is authored by the WIA Standards Working Group and is kept
in lockstep across Phases 1–4 of traditional-medicine so that conformance claims at any
Phase remain unambiguous.*

