# WIA Cognitive AAC - Phase 3: Prediction & Learning

## 전제조건
- Phase 1-2 완료

## Phase 3 목표: 사용 패턴 기반 개인화 예측

### 3.1 사용 패턴 학습

```typescript
interface UsagePattern {
  // 시간대별 패턴
  temporal: {
    hourlyFrequency: Map<number, SymbolFrequency[]>;  // 0-23시
    dayOfWeek: Map<number, SymbolFrequency[]>;        // 0-6
    contextual: Map<string, SymbolFrequency[]>;       // 식사, 취침 등
  };

  // 시퀀스 패턴
  sequences: {
    commonPhrases: Phrase[];           // 자주 쓰는 문장
    symbolChains: SymbolChain[];       // 심볼 연쇄
    conversationPatterns: ConversationPattern[];
  };

  // 상황별 패턴
  contextual: {
    locationBased: Map<string, SymbolFrequency[]>;
    personBased: Map<string, SymbolFrequency[]>;      // 대화 상대별
    activityBased: Map<string, SymbolFrequency[]>;
  };
}

interface SymbolChain {
  symbols: string[];           // 심볼 ID 시퀀스
  frequency: number;
  avgInterval: number;         // 심볼 간 평균 시간 (ms)
  probability: number;         // 다음 심볼 예측 확률
}
```

### 3.2 예측 엔진

```typescript
interface PredictionEngine {
  // 다음 심볼 예측
  predictNext(
    currentContext: Context,
    recentSymbols: string[]
  ): PredictedSymbol[];

  // 문장 완성 예측
  predictCompletion(
    partialPhrase: string[],
    context: Context
  ): Phrase[];

  // 상황 기반 추천
  recommendForContext(
    context: Context
  ): RecommendedBoard;
}

interface PredictedSymbol {
  symbolId: string;
  probability: number;
  source: 'frequency' | 'sequence' | 'context' | 'time';
  explanation?: string;        // "이 시간에 자주 사용해요"
}

interface Context {
  time: Date;
  location?: string;
  conversationPartner?: string;
  recentActivity?: string;
  mood?: string;               // 감정 인식 결과
  environmentalCues?: string[];
}
```

### 3.3 인지장애 특화 예측

```typescript
// 자폐: 루틴 기반 예측
interface RoutineBasedPrediction {
  // 일과 시간표 학습
  dailyRoutine: RoutineStep[];

  // 루틴 이탈 시 지원
  routineDeviation: {
    detectDeviation(): boolean;
    suggestReturnToRoutine(): RoutineStep[];
    offerFlexibleAlternative(): SymbolBoard;
  };

  // 전환 예측
  transitionPrediction: {
    predictNextActivity(): Activity;
    prepareTransitionSupport(): TransitionAid;
  };
}

// 치매: 회상 기반 예측
interface ReminiscenceBasedPrediction {
  // 장기 기억 활용
  longTermMemoryTriggers: {
    familiarPhotos: Photo[];
    significantDates: Date[];
    musicMemories: Song[];
  };

  // 반복 질문 패턴
  repetitiveQuestions: {
    detect(): boolean;
    gentlyRedirect(): Response;
    provideReassurance(): Message;
  };

  // 시간/장소 지남력 지원
  orientationSupport: {
    currentTimeAwareness(): TemporalInfo;
    locationReminder(): LocationInfo;
    familiarFaceIdentification(): Person[];
  };
}
```

### 3.4 프라이버시 보호 학습

```typescript
interface PrivacyPreservingLearning {
  // 온디바이스 학습
  localLearning: {
    enabled: boolean;
    modelStorage: 'local_only';
    noCloudUpload: boolean;
  };

  // 연합 학습 (선택적)
  federatedLearning: {
    enabled: boolean;
    consentRequired: boolean;
    anonymization: 'full' | 'partial';
    aggregationOnly: boolean;    // 개별 데이터 전송 안함
  };

  // 데이터 보존
  retention: {
    maxDays: number;
    autoDelete: boolean;
    exportFormat: 'encrypted_backup';
  };
}
```

---

## 산출물

```
cognitive-aac/
├── api/
│   ├── typescript/
│   │   └── src/prediction/
│   │       ├── PredictionEngine.ts
│   │       ├── PatternLearner.ts
│   │       ├── ContextDetector.ts
│   │       └── PrivacyManager.ts
│   └── python/
│       └── wia_cognitive_aac/
│           ├── ml/
│           │   ├── sequence_model.py
│           │   └── context_model.py
│           └── privacy/
│               └── federated.py
└── spec/
    ├── PREDICTION-SPEC.md
    └── PRIVACY-SPEC.md
```

---

## 다음 단계

Phase 3 완료 후:
- `prompts/PHASE-4-PROMPT.md` 읽고 Phase 4 (케어기버 연동) 시작
