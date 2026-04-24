# WIA Cognitive AAC - Prediction & Learning Specification

## Phase 3: 예측/학습 알고리즘 명세

**홍익인간 (弘益人間) - 널리 인간을 이롭게 하라**

---

## 1. 개요

### 1.1 목적

인지 장애 사용자의 AAC 의사소통 효율성을 높이기 위한 개인화된 예측 시스템.
사용자의 패턴을 학습하여 다음에 사용할 가능성이 높은 심볼을 예측합니다.

### 1.2 핵심 원칙

1. **온디바이스 학습**: 모든 학습은 사용자 기기에서 수행
2. **프라이버시 우선**: 개인 데이터는 기기를 떠나지 않음
3. **적응적 예측**: 사용자 특성에 맞는 맞춤 예측
4. **투명한 설명**: 예측 이유를 이해하기 쉽게 제공

---

## 2. 예측 모델

### 2.1 다중 소스 예측

예측은 여러 소스의 정보를 종합하여 생성됩니다:

| 소스 | 설명 | 가중치 |
|------|------|--------|
| **frequency** | 전체 사용 빈도 기반 | 0.25 |
| **sequence** | 심볼 연쇄 패턴 | 0.30 |
| **context** | 상황(위치, 활동, 대화상대) | 0.25 |
| **time** | 시간대/요일 패턴 | 0.20 |

### 2.2 예측 결과 구조

```typescript
interface PredictedSymbol {
  symbolId: string;           // 심볼 식별자
  probability: number;        // 예측 확률 (0-1)
  source: PredictionSource;   // 예측 근거
  explanation?: string;       // 사용자 친화적 설명
  rank: number;               // 순위
}

type PredictionSource =
  | 'frequency'     // "자주 사용하는 심볼"
  | 'sequence'      // "이전에 이어서 사용했던 심볼"
  | 'context'       // "식사 시간에 자주 사용하는 심볼"
  | 'time'          // "아침에 자주 사용하는 심볼"
  | 'routine'       // "일과 시간에 필요한 심볼" (자폐)
  | 'reminiscence'; // "익숙한 기억과 연결된 심볼" (치매)
```

---

## 3. 학습 알고리즘

### 3.1 N-gram 시퀀스 모델

경량화된 N-gram 모델로 심볼 시퀀스 패턴 학습:

```
P(symbol_n | symbol_1, ..., symbol_{n-1})
```

#### 특징:
- 최대 4-gram까지 지원
- Laplace smoothing 적용
- 시간 기반 감쇠 (decay factor: 0.95/일)
- 온디바이스 추론 가능한 경량 설계

#### 예시:
```
사용자가 "아침" → "먹다" 시퀀스를 자주 사용하면:
P("먹다" | "아침") = 높음
→ "아침" 선택 후 "먹다" 예측 확률 상승
```

### 3.2 컨텍스트 모델

다중 컨텍스트 신호를 통합한 예측:

| 컨텍스트 | 감지 방법 | 예시 |
|----------|-----------|------|
| **시간** | 시스템 시계 | 7-9시 → 아침 식사 |
| **위치** | GPS/실내 위치 | 부엌 → 식사 관련 |
| **활동** | 일과표/감지 | 수업 시간 → 학습 관련 |
| **대화상대** | 앱 설정 | 엄마 → 가족 대화 |
| **기분** | 이전 선택 | 통증 표현 → 불편함 |

### 3.3 인지장애 특화 모델

#### 자폐 스펙트럼 (Autism)

```typescript
interface AutismPredictionModel {
  // 루틴 기반 예측 강화
  routineWeight: 1.5;  // 루틴 심볼 가중치 50% 상승

  // 전환 예측
  transitionPrediction: {
    warnBeforeMinutes: 5;  // 전환 5분 전 알림
    suggestTransitionSymbols: true;
  };

  // 예측 가능성 최대화
  consistentPredictions: true;  // 동일 상황 → 동일 예측
}
```

#### 치매 (Dementia)

```typescript
interface DementiaPredictionModel {
  // 장기 기억 우선
  longTermMemoryWeight: 1.3;  // 익숙한 심볼 가중치 상승

  // 느린 학습
  decayFactor: 0.99;  // 천천히 잊기

  // 반복 허용
  allowRepetition: true;  // 같은 질문 반복 허용

  // 지남력 지원
  orientationSupport: {
    showDateTime: true;
    showLocation: true;
  };
}
```

---

## 4. 사용 패턴 데이터 구조

### 4.1 시간별 패턴

```typescript
interface TemporalPattern {
  hourlyFrequency: Map<number, SymbolFrequency[]>;  // 0-23시
  dayOfWeek: Map<number, SymbolFrequency[]>;        // 0-6 (일-토)
  contextual: Map<string, SymbolFrequency[]>;       // 식사, 수업 등
}
```

### 4.2 시퀀스 패턴

```typescript
interface SequencePattern {
  commonPhrases: Phrase[];      // 자주 사용하는 문장
  symbolChains: SymbolChain[];  // 심볼 연쇄
}

interface SymbolChain {
  symbols: string[];     // ["인사", "감사"]
  frequency: number;     // 50회
  avgInterval: number;   // 평균 500ms
  probability: number;   // 0.75
}
```

### 4.3 컨텍스트별 패턴

```typescript
interface ContextualPattern {
  locationBased: Map<string, SymbolFrequency[]>;   // 위치별
  personBased: Map<string, SymbolFrequency[]>;     // 대화상대별
  activityBased: Map<string, SymbolFrequency[]>;   // 활동별
}
```

---

## 5. API 명세

### 5.1 TypeScript API

```typescript
// 예측 엔진 생성
const engine = PredictionEngine.forAutism(config);

// 다음 심볼 예측
const predictions = engine.predictNext(context, recentSymbols);

// 문장 완성 예측
const completions = engine.predictCompletion(partialPhrase, context);

// 상황 기반 보드 추천
const board = engine.recommendForContext(context);

// 학습 이벤트 기록
engine.recordSymbolSelection(symbolId, context, responseTime);
```

### 5.2 Python API

```python
# 시퀀스 모델
model = SequenceModel()
model.train(sequences)
predictions = model.predict(context, top_k=8)

# 컨텍스트 모델
ctx_model = ContextModel()
ctx_model.update(symbol_id, context)
ctx_predictions = ctx_model.predict(context)
```

---

## 6. 성능 요구사항

### 6.1 응답 시간

| 작업 | 목표 | 최대 |
|------|------|------|
| 다음 심볼 예측 | 50ms | 100ms |
| 문장 완성 | 100ms | 200ms |
| 학습 업데이트 | 10ms | 50ms |

### 6.2 정확도 목표

| 지표 | 목표 |
|------|------|
| Top-1 정확도 | 30% |
| Top-5 정확도 | 60% |
| Top-8 정확도 | 75% |

### 6.3 리소스 제한

| 리소스 | 제한 |
|--------|------|
| 메모리 | < 50MB |
| 모델 크기 | < 10MB |
| 배터리 영향 | < 5% |

---

## 7. 적응 전략

### 7.1 신규 사용자

- 첫 주: 인구 통계 기반 기본 예측
- 2-4주: 빈도 기반 학습 시작
- 1개월+: 전체 패턴 학습 활성화

### 7.2 점진적 학습

```
신뢰도 = min(1.0, 사용_횟수 / 100)
예측_가중치 = 기본_가중치 × (0.5 + 0.5 × 신뢰도)
```

### 7.3 감쇠 및 갱신

- **일간 감쇠**: factor = 0.95
- **최소 임계값**: 2회 미만 패턴 제거
- **최대 저장**: 최근 90일 데이터

---

## 8. 통합 가이드

### 8.1 React 컴포넌트 통합

```tsx
function AACBoard() {
  const { config } = useAdaptiveUI({ profile });
  const [predictions, setPredictions] = useState([]);

  const engine = useRef(PredictionEngine.forAutism());

  const handleSymbolSelect = (symbolId: string) => {
    engine.current.recordSymbolSelection(symbolId, currentContext);
    const newPredictions = engine.current.predictNext(context, [symbolId]);
    setPredictions(newPredictions);
  };

  return (
    <AdaptiveGrid>
      {predictions.map(pred => (
        <AdaptiveSymbol
          key={pred.symbolId}
          highlighted={pred.rank <= 3}
        />
      ))}
    </AdaptiveGrid>
  );
}
```

### 8.2 Python 백엔드 통합

```python
from wia_cognitive_aac import SequenceModel, ContextModel

# 모델 초기화
seq_model = SequenceModel()
ctx_model = ContextModel()

# 예측 서비스
def get_predictions(context, recent_symbols):
    seq_preds = seq_model.predict(recent_symbols)
    ctx_preds = ctx_model.predict(context)
    return merge_predictions(seq_preds, ctx_preds)
```

---

## 9. 다음 단계

Phase 3 완료 후 Phase 4 (케어기버 연동) 진행:
- 케어기버 앱 인터페이스
- 원격 설정 관리
- 사용 통계 대시보드

---

*WIA Cognitive AAC Prediction Specification v1.0*
*Last Updated: 2024*
