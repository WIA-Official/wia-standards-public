# WIA Cognitive AAC - Assessment Metrics Specification
# AAC 효과 평가 메트릭 표준 명세서

**Version**: 1.0.0
**Date**: 2025-01-14
**Status**: Draft

---

## 철학
**홍익인간 (弘益人間)** - 널리 인간을 이롭게 하라

---

## 1. 개요

### 1.1 목적
AAC 개입의 효과를 객관적으로 측정하고 추적하기 위한 표준화된 메트릭을 정의합니다.

### 1.2 필요성
- 서로 다른 AAC 시스템 간 효과 비교
- 개인의 진행 상황 추적
- 연구 및 임상에서의 일관된 측정
- 근거 기반 의사결정 지원

### 1.3 메트릭 범주

```
AAC 효과 메트릭
├── 의사소통 효율성 (Communication Efficiency)
├── 의사소통 효과성 (Communication Effectiveness)
├── 언어 발달 (Language Development)
├── 사회적 참여 (Social Participation)
├── 독립성 (Independence)
├── 인지 부하 (Cognitive Load)
└── 사용자 만족도 (User Satisfaction)
```

---

## 2. 메트릭 데이터 구조

### 2.1 기본 구조

```typescript
interface AACMetrics {
  // 메타데이터
  id: string;
  profileId: string;              // 연결된 CognitiveProfile ID
  sessionId?: string;             // 세션 ID (있는 경우)
  timestamp: string;              // ISO 8601
  measurementPeriod: {
    start: string;
    end: string;
    durationDays: number;
  };

  // 메트릭 범주
  efficiency: EfficiencyMetrics;
  effectiveness: EffectivenessMetrics;
  languageDevelopment: LanguageDevelopmentMetrics;
  socialParticipation: SocialParticipationMetrics;
  independence: IndependenceMetrics;
  cognitiveLoad: CognitiveLoadMetrics;
  satisfaction: SatisfactionMetrics;

  // 컨텍스트
  context: MeasurementContext;
}

interface MeasurementContext {
  aacSystem: string;              // 사용된 AAC 시스템
  setting: 'home' | 'school' | 'clinic' | 'community' | 'work';
  communicationPartners: string[]; // 대화 상대
  assessor: string;               // 평가자
  method: 'automated' | 'manual' | 'mixed';
}
```

---

## 3. 의사소통 효율성 메트릭

### 3.1 EfficiencyMetrics

```typescript
interface EfficiencyMetrics {
  // 의사소통 속도
  communicationRate: {
    messagesPerMinute: number;    // 분당 메시지 수
    symbolsPerMinute: number;     // 분당 상징 선택 수
    wordsPerMinute: number;       // 분당 단어 수 (TTS 기준)
    trend: 'increasing' | 'stable' | 'decreasing';
  };

  // 선택 속도
  selectionSpeed: {
    averageTimeMs: number;        // 평균 선택 시간 (ms)
    medianTimeMs: number;         // 중앙값 선택 시간
    variability: number;          // 변동성 (표준편차)
  };

  // 탐색 효율성
  navigationEfficiency: {
    averageStepsToTarget: number; // 목표까지 평균 단계 수
    backtrackRate: number;        // 되돌아가기 비율 (%)
    searchSuccessRate: number;    // 검색 성공률 (%)
  };

  // 오류율
  errorMetrics: {
    errorRate: number;            // 전체 오류율 (%)
    correctionRate: number;       // 오류 수정 비율 (%)
    abandonmentRate: number;      // 포기 비율 (%)
  };
}
```

### 3.2 효율성 벤치마크

| 메트릭 | 초보 | 중급 | 고급 | 전문 |
|--------|:----:|:----:|:----:|:----:|
| 분당 메시지 | <2 | 2-4 | 4-8 | >8 |
| 분당 단어 | <5 | 5-15 | 15-30 | >30 |
| 평균 선택 시간 | >3s | 1-3s | 0.5-1s | <0.5s |
| 오류율 | >20% | 10-20% | 5-10% | <5% |

---

## 4. 의사소통 효과성 메트릭

### 4.1 EffectivenessMetrics

```typescript
interface EffectivenessMetrics {
  // 메시지 이해도
  messageClarity: {
    understoodByPartner: number;  // 상대가 이해한 비율 (%)
    requiresClarification: number; // 명확화 필요 비율 (%)
    misunderstandingRate: number; // 오해 비율 (%)
  };

  // 의사소통 성공률
  successRate: {
    overallSuccess: number;       // 전체 성공률 (%)
    byFunction: {
      requesting: number;         // 요청 성공률
      commenting: number;         // 언급 성공률
      questioning: number;        // 질문 성공률
      responding: number;         // 응답 성공률
      socialInteraction: number;  // 사회적 상호작용 성공률
    };
  };

  // 의사소통 범위
  communicativeRange: {
    functionsUsed: number;        // 사용된 의사소통 기능 수
    topicsDiscussed: number;      // 논의된 주제 수
    partnersEngaged: number;      // 참여한 대화 상대 수
  };

  // 대화 참여
  conversationalParticipation: {
    initiationRate: number;       // 시작 비율 (%)
    responseRate: number;         // 응답 비율 (%)
    turnTakingAccuracy: number;   // 차례 주고받기 정확도 (%)
    topicMaintenance: number;     // 주제 유지율 (%)
  };
}
```

### 4.2 의사소통 기능 분류

```typescript
enum CommunicativeFunction {
  // 행동 조절
  REQUEST_OBJECT = 'request_object',
  REQUEST_ACTION = 'request_action',
  REQUEST_INFORMATION = 'request_information',
  PROTEST = 'protest',
  REJECT = 'reject',

  // 사회적 상호작용
  GREET = 'greet',
  CALL_ATTENTION = 'call_attention',
  REQUEST_SOCIAL_ROUTINE = 'request_social_routine',
  SHOW_OFF = 'show_off',
  ACKNOWLEDGE = 'acknowledge',

  // 공동 주의
  COMMENT = 'comment',
  REQUEST_JOINT_ATTENTION = 'request_joint_attention',
  CLARIFY = 'clarify',
  ANSWER = 'answer'
}
```

---

## 5. 언어 발달 메트릭

### 5.1 LanguageDevelopmentMetrics

```typescript
interface LanguageDevelopmentMetrics {
  // 어휘
  vocabulary: {
    totalSymbolsUsed: number;     // 사용된 총 상징 수
    uniqueSymbolsUsed: number;    // 고유 상징 수
    newSymbolsThisPeriod: number; // 이번 기간 새 상징
    vocabularyGrowthRate: number; // 어휘 성장률 (%/week)
    symbolCategories: {
      nouns: number;
      verbs: number;
      adjectives: number;
      socialWords: number;
      coreWords: number;
      fringeWords: number;
    };
  };

  // 문법/구문
  syntax: {
    averageUtteranceLength: number; // 평균 발화 길이 (MLU)
    maxUtteranceLength: number;   // 최대 발화 길이
    multiWordCombinations: number; // 다어 조합 빈도
    grammaticalStructures: string[]; // 사용된 문법 구조
  };

  // 의미론
  semantics: {
    semanticRelationsUsed: string[]; // 사용된 의미 관계
    conceptCategories: string[];    // 사용된 개념 범주
  };

  // Communication Matrix 레벨
  communicationMatrixLevel: {
    currentLevel: 1 | 2 | 3 | 4 | 5 | 6 | 7;
    previousLevel?: number;
    dateOfChange?: string;
  };
}
```

### 5.2 어휘 성장 추적

```typescript
interface VocabularyGrowthTracker {
  // 시간별 추적
  timeline: VocabularySnapshot[];

  // 분석
  analysis: {
    growthTrend: 'accelerating' | 'linear' | 'plateauing' | 'declining';
    predictedVocabIn30Days: number;
    categoryBalance: 'balanced' | 'noun_heavy' | 'verb_heavy' | 'social_heavy';
  };
}

interface VocabularySnapshot {
  date: string;
  totalActive: number;           // 활성 어휘
  totalPassive: number;          // 수동 어휘 (인식하지만 사용 안함)
  newThisWeek: number;
  mostUsed: string[];            // 가장 많이 사용된 단어
  leastUsed: string[];           // 가장 적게 사용된 단어
}
```

---

## 6. 사회적 참여 메트릭

### 6.1 SocialParticipationMetrics

```typescript
interface SocialParticipationMetrics {
  // 상호작용 빈도
  interactionFrequency: {
    totalInteractions: number;    // 총 상호작용 수
    interactionsPerDay: number;   // 일일 평균 상호작용
    uniquePartners: number;       // 고유 대화 상대 수
    partnerTypes: {
      family: number;
      peers: number;
      professionals: number;
      strangers: number;
    };
  };

  // 상호작용 품질
  interactionQuality: {
    reciprocity: number;          // 상호성 점수 (1-10)
    engagementDuration: number;   // 평균 참여 지속 시간 (초)
    positiveInteractions: number; // 긍정적 상호작용 비율 (%)
  };

  // 사회적 맥락
  socialContexts: {
    contexts: string[];           // 참여한 사회적 맥락
    preferredContexts: string[];  // 선호하는 맥락
    challengingContexts: string[]; // 어려운 맥락
  };

  // 관계 발전
  relationshipDevelopment: {
    newRelationships: number;     // 새로운 관계 수
    strengthenedRelationships: number; // 강화된 관계 수
    socialNetworkSize: number;    // 사회적 네트워크 크기
  };
}
```

---

## 7. 독립성 메트릭

### 7.1 IndependenceMetrics

```typescript
interface IndependenceMetrics {
  // 독립적 사용
  independentUse: {
    unpromotedInitiations: number; // 촉진 없는 시작 (%)
    independentNavigation: number; // 독립적 탐색 (%)
    selfCorrection: number;       // 자기 수정 비율 (%)
  };

  // 촉진 수준
  promptingLevel: {
    noPrompt: number;             // 촉진 없음 (%)
    indirectVerbal: number;       // 간접 언어적 촉진 (%)
    directVerbal: number;         // 직접 언어적 촉진 (%)
    gestural: number;             // 제스처 촉진 (%)
    physical: number;             // 신체적 촉진 (%)
    trend: 'decreasing' | 'stable' | 'increasing';
  };

  // 일반화
  generalization: {
    acrossSettings: number;       // 환경 간 일반화 (%)
    acrossPartners: number;       // 대화 상대 간 일반화 (%)
    acrossTopics: number;         // 주제 간 일반화 (%)
    maintenance: number;          // 유지율 (%)
  };

  // 자기 옹호
  selfAdvocacy: {
    expressesNeeds: boolean;      // 필요 표현
    makesChoices: boolean;        // 선택하기
    expressesFeelings: boolean;   // 감정 표현
    requestsHelp: boolean;        // 도움 요청
  };
}
```

### 7.2 촉진 수준 정의

| 수준 | 설명 | 독립성 점수 |
|------|------|:----------:|
| **독립적** | 촉진 없이 수행 | 100 |
| **간접 언어** | "뭔가 말하고 싶니?" | 80 |
| **직접 언어** | "기기 사용해볼래?" | 60 |
| **제스처** | 기기를 가리킴 | 40 |
| **부분 신체** | 손을 기기 쪽으로 안내 | 20 |
| **완전 신체** | 손 위에 손 (hand-over-hand) | 0 |

---

## 8. 인지 부하 메트릭

### 8.1 CognitiveLoadMetrics

```typescript
interface CognitiveLoadMetrics {
  // 주관적 측정
  subjective: {
    perceivedDifficulty: number;  // 지각된 어려움 (1-10)
    mentalEffort: number;         // 정신적 노력 (1-10)
    frustrationLevel: number;     // 좌절 수준 (1-10)
    confidenceLevel: number;      // 자신감 수준 (1-10)
  };

  // 행동 지표
  behavioral: {
    taskCompletionTime: number;   // 과제 완료 시간 (ms)
    hesitationFrequency: number;  // 망설임 빈도
    errorPattern: ErrorPattern;   // 오류 패턴
    abandonmentRate: number;      // 포기율 (%)
  };

  // 생리적 지표 (가능한 경우)
  physiological?: {
    eyeTrackingMetrics?: {
      fixationDuration: number;   // 고정 지속 시간
      saccadeFrequency: number;   // 급속 안구운동 빈도
      pupilDilation: number;      // 동공 확장
    };
  };

  // 부하 최적화
  optimization: {
    optimalItemsPerScreen: number; // 최적 화면당 항목 수
    optimalSessionDuration: number; // 최적 세션 길이 (분)
    breakFrequency: number;       // 권장 휴식 빈도 (분당)
  };
}

interface ErrorPattern {
  type: 'random' | 'fatigue' | 'complexity' | 'confusion';
  timeOfOccurrence: 'early' | 'middle' | 'late';
  categoryMostAffected?: string;
}
```

### 8.2 인지 부하 권장 수준

| 인지 수준 | 권장 화면 항목 | 세션 길이 | 휴식 빈도 |
|----------|:-------------:|:---------:|:---------:|
| PROFOUND (1) | 2-4 | 5분 | 5분마다 |
| SEVERE (2) | 4-6 | 10분 | 10분마다 |
| MODERATE (3) | 6-9 | 15분 | 15분마다 |
| MILD (4) | 9-16 | 20분 | 20분마다 |
| TYPICAL (5) | 16+ | 30분+ | 필요시 |

---

## 9. 사용자 만족도 메트릭

### 9.1 SatisfactionMetrics

```typescript
interface SatisfactionMetrics {
  // 전반적 만족도
  overall: {
    satisfactionScore: number;    // 1-10
    netPromoterScore?: number;    // -100 to 100 (NPS)
    wouldRecommend: boolean;
  };

  // 사용성
  usability: {
    easeOfUse: number;            // 사용 용이성 (1-10)
    learnability: number;         // 학습 용이성 (1-10)
    efficiency: number;           // 효율성 (1-10)
    errorRecovery: number;        // 오류 복구 (1-10)
  };

  // 기능 만족도
  featureSatisfaction: {
    vocabularyCoverage: number;   // 어휘 범위 (1-10)
    symbolQuality: number;        // 상징 품질 (1-10)
    voiceQuality: number;         // 음성 품질 (1-10)
    customization: number;        // 맞춤화 (1-10)
  };

  // 정서적 반응
  emotionalResponse: {
    confidence: number;           // 자신감 (1-10)
    empowerment: number;          // 역량 강화 (1-10)
    frustration: number;          // 좌절감 (1-10, 낮을수록 좋음)
    enjoyment: number;            // 즐거움 (1-10)
  };

  // 삶의 질 영향
  qualityOfLifeImpact: {
    communicationImprovement: number; // 의사소통 향상 (1-10)
    socialConnection: number;     // 사회적 연결 (1-10)
    independence: number;         // 독립성 (1-10)
    overallWellbeing: number;     // 전반적 웰빙 (1-10)
  };

  // 돌봄자 관점 (해당 시)
  caregiverPerspective?: {
    easeOfSetup: number;          // 설정 용이성 (1-10)
    maintenanceEffort: number;    // 유지 노력 (1-10, 낮을수록 좋음)
    perceivedBenefit: number;     // 인지된 효과 (1-10)
    stressReduction: number;      // 스트레스 감소 (1-10)
  };
}
```

---

## 10. 메트릭 수집 방법

### 10.1 자동 수집

```typescript
interface AutomaticDataCollection {
  // 기기 로그
  deviceLogs: {
    selectionTimestamps: number[];
    navigationPaths: string[];
    errorLogs: ErrorLog[];
    sessionDurations: number[];
  };

  // 사용 패턴
  usagePatterns: {
    peakUsageTimes: string[];
    frequentlyUsedSymbols: string[];
    categoryUsageDistribution: Record<string, number>;
  };

  // 성능 메트릭
  performance: {
    responseLatency: number[];
    successfulSelections: number;
    failedSelections: number;
  };
}
```

### 10.2 수동 평가

```typescript
interface ManualAssessment {
  // 평가 도구
  assessmentTool: string;         // 사용된 평가 도구
  assessor: string;               // 평가자
  date: string;                   // 평가 일자

  // 관찰 기록
  observations: {
    communicativeAttempts: number;
    successfulCommunications: number;
    communicativePartners: string[];
    contexts: string[];
    notes: string;
  };

  // 체크리스트
  checklist: {
    item: string;
    achieved: boolean;
    notes?: string;
  }[];
}
```

### 10.3 혼합 방법

```typescript
interface MixedMethodAssessment {
  automatic: AutomaticDataCollection;
  manual: ManualAssessment;

  // 통합 분석
  integratedAnalysis: {
    concordance: number;          // 자동/수동 일치도 (%)
    discrepancies: string[];      // 불일치 영역
    recommendations: string[];    // 권장사항
  };
}
```

---

## 11. 진행 보고서

### 11.1 ProgressReport 구조

```typescript
interface ProgressReport {
  // 메타데이터
  reportId: string;
  profileId: string;
  reportPeriod: {
    start: string;
    end: string;
  };
  generatedAt: string;

  // 요약
  summary: {
    overallProgress: 'significant' | 'moderate' | 'minimal' | 'regression';
    keyAchievements: string[];
    areasOfConcern: string[];
    recommendations: string[];
  };

  // 메트릭 비교
  comparison: {
    previous: AACMetrics;
    current: AACMetrics;
    changes: MetricChanges;
  };

  // 목표 추적
  goalTracking: {
    goals: Goal[];
    overallGoalAttainment: number; // %
  };

  // 시각화 데이터
  visualizations: {
    progressChart: ChartData;
    vocabularyGrowth: ChartData;
    communicationFrequency: ChartData;
  };
}

interface Goal {
  id: string;
  description: string;
  targetValue: number;
  currentValue: number;
  attainmentPercentage: number;
  status: 'achieved' | 'in_progress' | 'not_started' | 'discontinued';
}

interface MetricChanges {
  improved: string[];
  declined: string[];
  stable: string[];
  significantChanges: {
    metric: string;
    previousValue: number;
    currentValue: number;
    percentChange: number;
  }[];
}
```

### 11.2 진행 보고서 예시 형식

```markdown
# AAC 진행 보고서

## 기간: 2024-10-01 ~ 2024-12-31

### 요약
- **전반적 진행**: 상당한 향상
- **주요 성취**:
  - 어휘 50개 → 120개 (140% 증가)
  - 독립적 시작 30% → 55% 향상
  - 의사소통 성공률 60% → 78%

### 세부 메트릭

| 메트릭 | 이전 | 현재 | 변화 |
|--------|:----:|:----:|:----:|
| 활성 어휘 | 50 | 120 | +140% |
| 분당 메시지 | 2.1 | 3.4 | +62% |
| 오류율 | 18% | 12% | -33% |
| 독립적 사용 | 30% | 55% | +83% |

### 권장사항
1. 어휘 범주 확장 (동사 추가)
2. 다어 조합 목표 설정
3. 새로운 환경으로 일반화 연습
```

---

## 12. 벤치마킹 및 비교

### 12.1 인구 기준 비교

```typescript
interface BenchmarkComparison {
  // 대상자 메트릭
  individualMetrics: AACMetrics;

  // 비교 그룹
  comparisonGroup: {
    description: string;          // 예: "자폐 스펙트럼, 초기 상징 사용자"
    sampleSize: number;
    demographicMatch: string[];
  };

  // 백분위
  percentileRanks: {
    efficiency: number;           // 효율성 백분위
    effectiveness: number;        // 효과성 백분위
    vocabulary: number;           // 어휘 백분위
    independence: number;         // 독립성 백분위
  };

  // 해석
  interpretation: {
    strengths: string[];
    areasForGrowth: string[];
    comparedToTypical: string;
  };
}
```

---

## 13. API 예시

### 13.1 메트릭 기록

```typescript
// POST /metrics
async function recordMetrics(
  profileId: string,
  metrics: Partial<AACMetrics>
): Promise<AACMetrics> {
  const fullMetrics: AACMetrics = {
    id: generateUUID(),
    profileId,
    timestamp: new Date().toISOString(),
    ...metrics
  };

  await saveToDatabase(fullMetrics);
  return fullMetrics;
}
```

### 13.2 진행 보고서 생성

```typescript
// GET /profiles/{id}/progress-report
async function generateProgressReport(
  profileId: string,
  startDate: string,
  endDate: string
): Promise<ProgressReport> {
  const metrics = await getMetricsInRange(profileId, startDate, endDate);
  const previousMetrics = await getPreviousPeriodMetrics(profileId, startDate);

  return {
    reportId: generateUUID(),
    profileId,
    reportPeriod: { start: startDate, end: endDate },
    generatedAt: new Date().toISOString(),
    summary: analyzeTrends(metrics, previousMetrics),
    comparison: compareMetrics(previousMetrics, metrics),
    goalTracking: evaluateGoals(profileId, metrics),
    visualizations: generateCharts(metrics)
  };
}
```

---

## 14. 참조

### 14.1 관련 측정 도구
- Communication Matrix (Rowland & Schweigert)
- ASHA's Functional Communication Measures
- AAC Profile (Hill & Romich)
- COMPASS (Communication Participation Item Bank)

### 14.2 연구 문헌
- Light, J., & McNaughton, D. (2014). Communicative competence for AAC users
- Beukelman, D. R., & Light, J. C. (2020). Augmentative and Alternative Communication

---

## 15. 변경 이력

| 버전 | 날짜 | 변경 내용 |
|------|------|----------|
| 1.0.0 | 2025-01-14 | 초기 버전 |

---

<div align="center">

**WIA Cognitive AAC - Assessment Metrics Standard**

*"측정할 수 없으면 개선할 수 없다"*

**홍익인간** - 널리 인간을 이롭게 하라

</div>
