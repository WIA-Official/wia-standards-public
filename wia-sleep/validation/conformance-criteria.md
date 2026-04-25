# WIA-SLEEP Conformance Criteria

## Version 1.0.0

---

## 1. Overview

This document defines the conformance criteria for WIA-SLEEP standard compliance. Implementations must meet these requirements to claim conformance.

---

## 2. Conformance Levels

### Level 1: Basic Conformance
- Required for minimal WIA-SLEEP compatibility
- Supports core sleep architecture data
- Basic chronotype assessment

### Level 2: Standard Conformance
- Full sleep architecture support
- Circadian markers integration
- Device mapping compliance

### Level 3: Full Conformance
- Complete standard implementation
- Algorithm certification
- Interoperability validation

---

## 3. Data Schema Conformance

### 3.1 Required Fields

| Field | Level 1 | Level 2 | Level 3 |
|-------|---------|---------|---------|
| recordId | Required | Required | Required |
| subjectId | Required | Required | Required |
| timestamp | Required | Required | Required |
| sleepArchitecture.totalSleepTime_min | Required | Required | Required |
| sleepArchitecture.sleepEfficiency_pct | Required | Required | Required |
| chronotype.classification | Optional | Required | Required |
| circadianMarkers | Optional | Optional | Required |
| metabolicSync | Optional | Optional | Optional |
| geneticFactors | Optional | Optional | Optional |

### 3.2 Validation Rules

```yaml
SleepRecord:
  recordId:
    type: uuid
    required: true
    unique: true

  timestamp:
    type: datetime
    format: ISO8601
    required: true

  sleepArchitecture.totalSleepTime_min:
    type: number
    minimum: 0
    maximum: 1440
    required: true

  sleepArchitecture.sleepEfficiency_pct:
    type: number
    minimum: 0
    maximum: 100
    required: true

  sleepArchitecture.stages:
    sum_constraint: "n1_pct + n2_pct + n3_sws_pct + rem_pct should equal ~100% (±5%)"

  chronotype.mctq_score:
    type: number
    minimum: 0
    maximum: 12

  chronotype.classification:
    type: enum
    values: [extreme_early, moderate_early, intermediate, moderate_late, extreme_late]
```

---

## 4. API Conformance

### 4.1 Required Endpoints (Level 1)

| Endpoint | Method | Description |
|----------|--------|-------------|
| /records | POST | Submit sleep record |
| /records | GET | Query sleep records |
| /records/{id} | GET | Get single record |

### 4.2 Standard Endpoints (Level 2)

| Endpoint | Method | Description |
|----------|--------|-------------|
| /chronotype/assess | POST | Chronotype assessment |
| /circadian/phase | GET | Get circadian phase |
| /optimization/recommend | POST | Get recommendations |

### 4.3 Full Endpoints (Level 3)

| Endpoint | Method | Description |
|----------|--------|-------------|
| /disorders/screen | POST | Sleep disorder screening |
| /light/prescription | POST | Light therapy prescription |
| /sleep-need/predict | POST | Sleep need prediction |

### 4.4 Response Format Requirements

```json
{
  "success": true,
  "data": { },
  "meta": {
    "version": "1.0.0",
    "timestamp": "ISO8601",
    "requestId": "uuid"
  }
}
```

Error responses must follow:

```json
{
  "success": false,
  "error": {
    "code": "ERROR_CODE",
    "message": "Human readable message",
    "details": { }
  }
}
```

---

## 5. Algorithm Conformance

### 5.1 Chronotype Classification

| Test Case | Input MSFsc | Expected Output | Tolerance |
|-----------|-------------|-----------------|-----------|
| CT-001 | 2.0 | extreme_early | Exact |
| CT-002 | 3.0 | moderate_early | Exact |
| CT-003 | 4.25 | intermediate | Exact |
| CT-004 | 5.5 | moderate_late | Exact |
| CT-005 | 7.0 | extreme_late | Exact |
| CT-006 | 2.5 | moderate_early | Exact (boundary) |
| CT-007 | 3.5 | intermediate | Exact (boundary) |

### 5.2 Social Jetlag Calculation

| Test Case | MSF | MSW | Expected SJL | Tolerance |
|-----------|-----|-----|--------------|-----------|
| SJL-001 | 4.5 | 3.5 | 1.0 | ±0.1 |
| SJL-002 | 5.0 | 3.0 | 2.0 | ±0.1 |
| SJL-003 | 4.0 | 4.0 | 0.0 | Exact |

### 5.3 Phase Response Curve

| Test Case | Timing (rel CBT) | Expected Shift | Tolerance |
|-----------|------------------|----------------|-----------|
| PRC-001 | +1.5h | Advance | Direction |
| PRC-002 | -3h | Delay | Direction |
| PRC-003 | +8h | Minimal | < 0.5h |

---

## 6. Data Quality Requirements

### 6.1 Consumer Wearable Data

| Metric | Minimum Accuracy | Validation Method |
|--------|------------------|-------------------|
| Total Sleep Time | ±30 min | PSG comparison |
| Sleep Efficiency | ±10% | PSG comparison |
| Stage Detection | 70% epoch agreement | PSG comparison |
| REM Detection | 75% sensitivity | PSG comparison |
| Deep Sleep Detection | 70% sensitivity | PSG comparison |

### 6.2 Clinical Data

| Metric | Requirement | Standard |
|--------|-------------|----------|
| PSG Scoring | AASM 2018/2020 compliant | AASM Guidelines |
| Inter-scorer Agreement | >80% | AASM Standards |
| Arousal Scoring | Per AASM criteria | AASM 2020 |
| Respiratory Events | AHI per AASM | AASM 2020 |

---

## 7. Interoperability Requirements

### 7.1 FHIR Mapping

- Must correctly map to FHIR R4 Observation resources
- Must use standard LOINC codes where available
- Must support WIA-SLEEP value sets

### 7.2 Device Integration

- Must provide documented mapping for each supported device
- Must specify data quality limitations
- Must handle missing/unavailable fields gracefully

---

## 8. Security Requirements

### 8.1 Authentication

- Must support OAuth 2.0 or API key authentication
- Must use HTTPS for all endpoints
- Must implement rate limiting

### 8.2 Data Privacy

- Must support data anonymization options
- Must comply with HIPAA/GDPR where applicable
- Must provide audit logging

---

## 9. Testing Requirements

### 9.1 Unit Tests

```yaml
required_coverage:
  chronotype_service: 80%
  sleep_analyzer: 80%
  circadian_calculator: 80%
  optimization_engine: 75%
```

### 9.2 Integration Tests

- API endpoint functionality
- Data persistence
- Device mapping accuracy

### 9.3 Conformance Test Suite

Run the WIA-SLEEP conformance test suite:

```bash
npm run test:conformance
# or
wia-sleep-validator --level 2 --endpoint https://api.example.com
```

---

## 10. Certification Process

### 10.1 Self-Assessment

1. Complete conformance checklist
2. Run automated test suite
3. Document any deviations

### 10.2 WIA Certification

1. Submit implementation for review
2. Provide test results
3. Demonstrate interoperability
4. Receive certification badge

### 10.3 Certification Levels

| Level | Requirements | Badge |
|-------|--------------|-------|
| Bronze | Level 1 conformance | WIA-SLEEP Basic |
| Silver | Level 2 conformance | WIA-SLEEP Standard |
| Gold | Level 3 conformance | WIA-SLEEP Full |

---

## 11. Versioning

- Major version changes may break conformance
- Minor versions maintain backward compatibility
- Implementations must declare supported version

---

**弘益人間 (홍익인간) - Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 MIT License*
