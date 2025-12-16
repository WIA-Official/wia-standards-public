# WIA Voice-Sign: Translation Quality Assurance Specification

## 1. Overview

본 문서는 음성-수화 번역의 품질 보증(QA) 표준을 정의합니다.

**Version:** 1.0.0
**Status:** Draft

---

## 2. Confidence Thresholds

### 2.1 Score Definitions

| Level | Range | Description | Action |
|-------|-------|-------------|--------|
| **Critical** | < 0.50 | 신뢰할 수 없음 | 번역 거부, 대체 방안 제시 |
| **Low** | 0.50 - 0.70 | 낮은 신뢰도 | 경고 표시, 검증 요청 |
| **Medium** | 0.70 - 0.85 | 수용 가능 | 주의 표시 (선택적) |
| **High** | 0.85 - 0.95 | 높은 신뢰도 | 정상 출력 |
| **Excellent** | > 0.95 | 매우 높은 신뢰도 | 검증 표시 |

### 2.2 Threshold Configuration

```json
{
  "confidence_thresholds": {
    "minimum_acceptable": 0.70,
    "warning_level": 0.85,
    "high_confidence": 0.95,
    "auto_approve": 0.98
  },
  "enforcement": {
    "strict_mode": true,
    "allow_low_confidence": false,
    "require_human_review_below": 0.80
  }
}
```

---

## 3. Validation Pipeline

### 3.1 Multi-Stage Validation

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Input     │───▶│   Grammar   │───▶│   Gloss     │───▶│   Context   │
│ Validation  │    │   Check     │    │  Coverage   │    │   Check     │
└─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘
      │                  │                  │                  │
      ▼                  ▼                  ▼                  ▼
  [Encoding]        [Syntax OK]       [Mapping %]       [Coherence]
      │                  │                  │                  │
      └──────────────────┴──────────────────┴──────────────────┘
                                   │
                                   ▼
                         ┌─────────────────┐
                         │  Final Score    │
                         │  Calculation    │
                         └─────────────────┘
```

### 3.2 Input Validation

| Check | Description | Failure Action |
|-------|-------------|----------------|
| Encoding | UTF-8 유효성 | 입력 거부 |
| Length | 최대 10,000자 | 분할 처리 |
| Language | 지원 언어 확인 | 에러 반환 |
| Profanity | 유해 콘텐츠 검사 | 필터링/경고 |

### 3.3 Grammar Check

**평가 기준:**
- 문장 구조 완전성
- 주어-동사 일치
- 시제 일관성
- 어순 적합성 (수화 문법 기준)

**수화 문법 특성:**
```
음성언어 (영어): Subject + Verb + Object
수화 (ASL):      Time + Topic + Comment + Question-marker

예시:
English: "Did you eat lunch yesterday?"
ASL Gloss: YESTERDAY LUNCH YOU EAT FINISH [eyebrows-up]
```

### 3.4 Gloss Coverage

**커버리지 계산:**
```
Coverage = (매핑된 단어 수 / 총 단어 수) × 100%

Thresholds:
- Excellent: ≥ 95%
- Good: 85-94%
- Fair: 70-84%
- Poor: < 70%
```

**미매핑 단어 처리:**
1. **Fingerspelling**: 고유명사, 전문용어
2. **Classifier**: 설명적 표현으로 대체
3. **Compound**: 복합 수화로 분해
4. **Skip**: 관사, 전치사 등 (수화에서 생략)

### 3.5 Context Consistency

**검증 항목:**
- 대명사 지시 일관성 (공간 참조)
- 시간 표현 일관성
- 주제 연속성
- 감정/태도 일관성

---

## 4. Quality Metrics

### 4.1 Translation Quality Score (TQS)

```
TQS = w₁×Fluency + w₂×Adequacy + w₃×Comprehensibility

Where:
- Fluency (유창성): 수화 문법 적합도 (w₁ = 0.3)
- Adequacy (적절성): 원문 의미 보존도 (w₂ = 0.4)
- Comprehensibility (이해도): 농인 사용자 이해도 (w₃ = 0.3)
```

### 4.2 Automatic Metrics

| Metric | Description | Range |
|--------|-------------|-------|
| BLEU | N-gram 일치도 | 0-100 |
| ROUGE | 요약 품질 | 0-1 |
| WER | 단어 오류율 | 0-100% |
| GER | Gloss 오류율 | 0-100% |

### 4.3 Human Evaluation Criteria

**5점 척도 평가:**

| Score | Fluency | Adequacy | Comprehensibility |
|-------|---------|----------|-------------------|
| 5 | 완전히 자연스러움 | 완전히 정확함 | 완전히 이해됨 |
| 4 | 대부분 자연스러움 | 대부분 정확함 | 대부분 이해됨 |
| 3 | 보통 | 핵심 의미 전달 | 어느정도 이해됨 |
| 2 | 부자연스러움 | 일부만 정확함 | 이해 어려움 |
| 1 | 이해 불가 | 완전히 틀림 | 전혀 이해 안됨 |

---

## 5. Low Confidence Handling

### 5.1 Fallback Strategy

```typescript
interface LowConfidenceStrategy {
  // 단계별 폴백
  fallbackOrder: [
    'fingerspell',      // 1순위: 지문자
    'show_alternatives', // 2순위: 대안 제시
    'simplified_gloss', // 3순위: 단순화
    'text_display',     // 4순위: 텍스트 표시
    'human_request'     // 5순위: 사람 통역 요청
  ];

  // 사용자 알림
  notification: {
    visualIndicator: boolean;   // 화면 표시
    audioAlert: boolean;        // 소리 알림
    hapticFeedback: boolean;    // 진동 피드백
  };
}
```

### 5.2 Visual Indicators

| Confidence | Indicator | Color | Icon |
|------------|-----------|-------|------|
| High | ✓ Verified | Green | ✅ |
| Medium | ~ Approximate | Yellow | ⚠️ |
| Low | ? Uncertain | Orange | ❓ |
| Critical | ✗ Unreliable | Red | ❌ |

### 5.3 Logging for Improvement

```json
{
  "log_entry": {
    "timestamp": "2025-01-15T10:30:00Z",
    "input_text": "complex medical term",
    "detected_issue": "low_gloss_coverage",
    "confidence_score": 0.65,
    "fallback_used": "fingerspell",
    "user_feedback": null,
    "flagged_for_review": true
  }
}
```

---

## 6. Quality Improvement Loop

### 6.1 Feedback Collection

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   User      │───▶│   System    │───▶│   Model     │
│  Feedback   │    │   Analysis  │    │   Update    │
└─────────────┘    └─────────────┘    └─────────────┘
      ▲                                      │
      │                                      │
      └──────────────────────────────────────┘
                 Improved Output
```

### 6.2 Feedback Types

- **Explicit**: 사용자 평점, 신고
- **Implicit**: 재번역 요청, 수정 사용
- **Professional**: 통역사 검토

### 6.3 Model Improvement

| Feedback Source | Weight | Update Frequency |
|-----------------|--------|------------------|
| Deaf Users | 1.0 | Weekly |
| Interpreters | 1.2 | Weekly |
| Linguists | 1.5 | Monthly |
| Auto Metrics | 0.5 | Daily |

---

## 7. Implementation Requirements

### 7.1 API Response

```json
{
  "translation_id": "tr_123",
  "status": "success",
  "quality": {
    "overall_score": 0.87,
    "confidence": 0.89,
    "gloss_coverage": 0.92,
    "grammar_score": 0.85,
    "context_score": 0.88
  },
  "warnings": [
    {
      "type": "low_confidence_segment",
      "segment": "specialized term",
      "confidence": 0.72,
      "suggestion": "Consider fingerspelling"
    }
  ],
  "verified": false
}
```

### 7.2 Quality Configuration

```yaml
quality_config:
  strict_mode: true
  minimum_threshold: 0.70
  auto_fallback: true
  log_low_confidence: true
  require_human_review: false

  validation:
    grammar_check: true
    coverage_check: true
    context_check: true
    profanity_filter: true

  improvement:
    collect_feedback: true
    anonymize_data: true
    update_frequency: "weekly"
```

---

## 8. References

- ISO 9001:2015 Quality Management Systems
- WCAG 2.1 AAA Guidelines
- ASTM F2575-14 Standard Guide for Quality Assurance
- WIA Quality Framework v1.0
