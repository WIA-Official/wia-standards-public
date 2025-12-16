# WIA Voice-Sign: Content Safety Specification

## 1. Overview

본 문서는 음성-수화 번역 시스템의 콘텐츠 안전 표준을 정의합니다.

**Version:** 1.0.0
**Status:** Draft

---

## 2. Content Filtering

### 2.1 Filter Categories

```typescript
interface ContentFilters {
  // 필터 카테고리
  categories: {
    profanity: {
      enabled: true;
      level: 'strict' | 'moderate' | 'minimal';
      action: 'block' | 'warn' | 'censor' | 'log';
    };

    violence: {
      enabled: true;
      level: 'strict' | 'moderate' | 'minimal';
      action: 'block' | 'warn' | 'log';
    };

    discrimination: {
      enabled: true;
      level: 'strict';
      action: 'block' | 'warn';
      protectedCategories: [
        'race', 'ethnicity', 'religion', 'gender',
        'sexual_orientation', 'disability', 'age', 'nationality'
      ];
    };

    sexualContent: {
      enabled: true;
      level: 'strict';
      action: 'block';
    };

    selfHarm: {
      enabled: true;
      level: 'strict';
      action: 'block' | 'redirect_to_help';
    };
  };

  // 사용자 정의 필터
  customFilters: {
    enabled: boolean;
    keywords: string[];
    patterns: RegExp[];
    action: FilterAction;
  };
}
```

### 2.2 Filter Levels

| Level | Profanity | Violence | Discrimination |
|-------|-----------|----------|----------------|
| **Strict** | 모든 비속어 차단 | 모든 폭력 언급 차단 | 모든 차별 표현 차단 |
| **Moderate** | 심한 비속어만 | 심각한 폭력만 | 명시적 차별만 |
| **Minimal** | 극심한 것만 | 위협적인 것만 | 혐오 발언만 |

### 2.3 Action Types

```typescript
type FilterAction =
  | 'block'      // 번역 거부
  | 'warn'       // 경고 후 진행
  | 'censor'     // 수정 후 번역 (예: f***)
  | 'replace'    // 대체 표현으로 변환
  | 'log'        // 로그만 기록
  | 'redirect';  // 도움 리소스로 연결

interface FilterResult {
  original: string;
  filtered: string;
  action: FilterAction;
  category: string;
  confidence: number;
  flags: string[];
}
```

---

## 3. Sensitive Content Handling

### 3.1 Medical Terms

```typescript
interface MedicalTermHandling {
  // 의료 용어 처리
  strategy: 'translate' | 'warn' | 'verify' | 'expert_required';

  // 카테고리별 설정
  categories: {
    anatomy: { strategy: 'translate', glossAvailable: true };
    conditions: { strategy: 'translate', verifyAccuracy: true };
    procedures: { strategy: 'warn', disclaimerRequired: true };
    medications: { strategy: 'verify', showGenericName: true };
    mentalHealth: { strategy: 'translate', sensitiveHandling: true };
  };

  // 면책 조항
  disclaimer: {
    required: true;
    text: "이 번역은 의료 조언이 아닙니다. 전문가와 상담하세요.";
    display: 'before_translation' | 'with_translation';
  };

  // 정확도 요구사항
  accuracy: {
    minimumConfidence: 0.95;
    requireGlossMatch: true;
    flagUncertain: true;
  };
}
```

### 3.2 Legal Terms

```typescript
interface LegalTermHandling {
  // 법률 용어 처리
  strategy: 'translate' | 'warn' | 'professional_required';

  // 면책 조항
  disclaimer: {
    required: true;
    text: "이 번역은 법률 조언이 아닙니다.";
  };

  // 특별 처리 카테고리
  categories: {
    contracts: { strategy: 'warn', recommendInterpreter: true };
    rights: { strategy: 'translate', verifyAccuracy: true };
    courtProceedings: { strategy: 'professional_required' };
    immigration: { strategy: 'warn', sensitiveHandling: true };
  };

  // 정확도 요구
  accuracy: {
    minimumConfidence: 0.90;
    legalReview: 'recommended';
  };
}
```

### 3.3 Religious/Cultural Terms

```typescript
interface ReligiousCulturalHandling {
  // 처리 전략
  strategy: 'neutral' | 'respectful' | 'literal';

  // 중립성 원칙
  neutrality: {
    noProselytizing: true;
    respectAllBeliefs: true;
    avoidJudgment: true;
  };

  // 문화적 적응
  culturalAdaptation: {
    localizedExpressions: boolean;
    culturalContext: boolean;
    regionalVariations: boolean;
  };

  // 민감한 주제
  sensitiveTopics: {
    religiousPractices: 'respectful';
    holyTexts: 'literal';
    rituals: 'descriptive';
    symbols: 'neutral';
  };
}
```

---

## 4. Mistranslation Prevention

### 4.1 Critical Term Validation

```typescript
interface CriticalTermValidation {
  // 필수 검증 대상
  criticalCategories: [
    'numbers',           // 숫자 (금액, 날짜, 시간)
    'names',             // 이름 (사람, 장소)
    'negation',          // 부정 (예/아니오)
    'medical',           // 의료 용어
    'legal',             // 법률 용어
    'emergency',         // 긴급 용어
  ];

  // 검증 방법
  validation: {
    doubleCheck: boolean;        // 이중 검증
    backTranslation: boolean;    // 역번역 확인
    contextVerification: boolean; // 문맥 확인
    confidenceThreshold: 0.95;
  };

  // 불확실 시 처리
  uncertainHandling: {
    showAlternatives: true;
    highlightUncertain: true;
    requestClarification: boolean;
    defaultToFingerspell: boolean;
  };
}
```

### 4.2 Homonym Disambiguation

```typescript
interface HomonymDisambiguation {
  // 동음이의어 처리
  strategy: {
    useContext: true;
    showOptions: boolean;
    defaultMostCommon: boolean;
  };

  // 예시
  examples: {
    'bank': ['financial_institution', 'river_bank'];
    'right': ['correct', 'direction', 'entitlement'];
    'light': ['illumination', 'weight', 'color'];
    '배': ['fruit_pear', 'stomach', 'ship', 'double'];
  };

  // 문맥 단서
  contextClues: {
    precedingWords: 3;
    followingWords: 3;
    topicModel: boolean;
    userHistory: boolean;
  };
}
```

### 4.3 Negation Handling

```typescript
interface NegationHandling {
  // 부정 표현 특별 처리
  importance: 'critical';

  // 검증
  verification: {
    doubleCheckNegation: true;
    highlightNegative: true;
    confirmUnderstanding: boolean;
  };

  // 수화 부정 표현
  signNegation: {
    headShake: true;           // 머리 흔들기
    negativeFacialMarker: true; // 부정 표정
    negativeSign: boolean;      // 부정 수화
  };

  // 오류 방지
  prevention: {
    // "I am not sick" vs "I am sick" 구분 중요
    detectNegationWords: ['not', 'no', 'never', 'without', '안', '못', '없'];
    verifyNegationPreserved: true;
    alertOnNegationUnclear: true;
  };
}
```

---

## 5. Context Verification

### 5.1 Semantic Consistency

```typescript
interface SemanticConsistency {
  // 의미 일관성 검증
  checks: {
    subjectConsistency: boolean;   // 주어 일관성
    timeframeConsistency: boolean; // 시제 일관성
    locationConsistency: boolean;  // 장소 일관성
    pronounReference: boolean;     // 대명사 참조
  };

  // 불일치 처리
  onInconsistency: {
    autoCorrect: boolean;
    showWarning: boolean;
    requestClarification: boolean;
  };

  // 문맥 창
  contextWindow: {
    previousSentences: 3;
    topicTracking: true;
    entityTracking: true;
  };
}
```

### 5.2 Tone & Register

```typescript
interface ToneRegister {
  // 어조/격식 수준
  detection: {
    formal: boolean;
    informal: boolean;
    professional: boolean;
    casual: boolean;
  };

  // 수화 적응
  adaptation: {
    matchSourceTone: boolean;
    adjustFormality: boolean;
    preserveEmotion: boolean;
  };

  // 설정
  settings: {
    defaultRegister: 'neutral';
    allowUserOverride: true;
    contextBasedAdjustment: true;
  };
}
```

---

## 6. User Reporting

### 6.1 Report System

```typescript
interface ReportSystem {
  // 신고 카테고리
  categories: [
    'mistranslation',
    'offensive_content',
    'cultural_insensitivity',
    'technical_error',
    'other'
  ];

  // 신고 양식
  reportForm: {
    category: string;
    description: string;
    originalText?: string;
    translatedGloss?: string;
    screenshot?: boolean;
    timestamp: Date;
    userId?: string;
  };

  // 처리 절차
  handling: {
    autoAcknowledge: true;
    reviewTimeTarget: '24h';
    escalationPath: ['AI_review', 'human_review', 'expert_review'];
    feedbackToUser: true;
  };
}
```

### 6.2 Content Review Process

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Report    │───▶│  AI Triage  │───▶│   Human     │───▶│   Action    │
│  Submitted  │    │  Analysis   │    │   Review    │    │   Taken     │
└─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘
                          │                  │                  │
                          ▼                  ▼                  ▼
                   [Auto-resolve]      [Escalate/      [Fix/Block/
                   [if obvious]        Investigate]     Feedback]
```

### 6.3 Appeal Process

```typescript
interface AppealProcess {
  // 이의 제기
  appeal: {
    allowedFor: ['content_blocked', 'account_action'];
    timeLimit: '14_days';
    humanReviewGuaranteed: true;
  };

  // 절차
  steps: [
    'submit_appeal',
    'provide_context',
    'human_review',
    'decision',
    'notification'
  ];

  // 결과
  outcomes: ['upheld', 'reversed', 'modified'];
}
```

---

## 7. Audit & Logging

### 7.1 Content Audit Log

```typescript
interface ContentAuditLog {
  // 기록 항목
  logEntry: {
    timestamp: Date;
    sessionId: string;
    inputHash: string;          // 원본 해시 (개인정보 보호)
    filterTriggered: boolean;
    filterCategory?: string;
    actionTaken: FilterAction;
    confidence: number;
    reviewStatus: 'pending' | 'reviewed' | 'cleared';
  };

  // 보존 정책
  retention: {
    flaggedContent: '90_days';
    clearedContent: '30_days';
    aggregatedStats: 'indefinite';
  };

  // 접근 제어
  access: {
    safetyTeam: 'full';
    developers: 'anonymized';
    auditors: 'full_with_approval';
  };
}
```

### 7.2 Safety Metrics

```typescript
interface SafetyMetrics {
  // 추적 지표
  metrics: {
    filterTriggerRate: number;      // 필터 발동 비율
    falsePositiveRate: number;      // 오탐 비율
    falseNegativeRate: number;      // 미탐 비율
    userReportRate: number;         // 사용자 신고 비율
    appealRate: number;             // 이의 제기 비율
    overturnRate: number;           // 결정 번복 비율
  };

  // 목표
  targets: {
    falsePositiveRate: '< 1%';
    falseNegativeRate: '< 0.1%';
    reviewResponseTime: '< 24h';
  };

  // 보고
  reporting: {
    frequency: 'weekly';
    stakeholders: ['safety_team', 'product', 'legal'];
  };
}
```

---

## 8. Implementation

### 8.1 API Response with Safety Info

```json
{
  "translation_id": "tr_abc123",
  "status": "success",
  "content_safety": {
    "passed": true,
    "filters_checked": ["profanity", "violence", "discrimination"],
    "warnings": [],
    "modified": false,
    "confidence": 0.99
  },
  "sensitive_content": {
    "detected": true,
    "categories": ["medical"],
    "disclaimer_shown": true
  }
}
```

### 8.2 Configuration

```yaml
content_safety:
  enabled: true

  filters:
    profanity:
      enabled: true
      level: moderate
      action: censor
    violence:
      enabled: true
      level: moderate
      action: warn
    discrimination:
      enabled: true
      level: strict
      action: block

  sensitive_content:
    medical:
      strategy: warn
      disclaimer: true
    legal:
      strategy: warn
      recommend_professional: true

  mistranslation_prevention:
    critical_term_validation: true
    homonym_disambiguation: true
    negation_verification: true
    minimum_confidence: 0.90

  reporting:
    enabled: true
    anonymous_allowed: true
    review_sla: 24h
```

---

## 9. References

- Trust & Safety Best Practices (Tech Coalition)
- Content Moderation Guidelines (Stanford Internet Observatory)
- AI Ethics Guidelines (IEEE, EU)
- Digital Services Act (EU DSA)
