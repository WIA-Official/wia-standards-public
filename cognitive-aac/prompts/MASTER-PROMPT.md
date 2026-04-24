# WIA Cognitive AAC Standard - Master Prompt

## 프로젝트 개요

**WIA Cognitive AAC**는 인지장애(치매, 자폐, 지적장애)를 가진 사용자를 위한
보완대체의사소통(AAC) 확장 표준입니다.

### 핵심 문제
- 기존 AAC는 신체장애 중심 설계 → 인지장애 특성 고려 부족
- [효과 측정 표준 메트릭 없음](https://pmc.ncbi.nlm.nih.gov/articles/PMC11590154/)
- 자폐 스펙트럼은 매우 다양 → 획일적 솔루션 부적합
- 고령화 → 치매 인구 폭발적 증가 예상

### WIA 솔루션

| Phase | 내용 | 산출물 |
|-------|------|--------|
| 1 | 인지 프로파일 표준 | 인지 능력 평가 스키마 |
| 2 | 적응형 UI 엔진 | 인지 수준별 동적 UI |
| 3 | 예측/학습 알고리즘 | 사용 패턴 기반 개인화 |
| 4 | 케어기버 연동 | 보호자/전문가 대시보드 |

---

## 4-Phase 구조

### Phase 1: 인지 프로파일 표준
```typescript
interface CognitiveProfile {
  // 기억력
  memory: {
    shortTerm: CognitiveLevel;    // 단기 기억
    longTerm: CognitiveLevel;     // 장기 기억
    working: CognitiveLevel;      // 작업 기억
  };

  // 주의력
  attention: {
    sustained: CognitiveLevel;    // 지속 주의
    selective: CognitiveLevel;    // 선택적 주의
    divided: CognitiveLevel;      // 분할 주의
  };

  // 언어
  language: {
    comprehension: CognitiveLevel;
    expression: CognitiveLevel;
    vocabulary: VocabularyLevel;
  };

  // 실행 기능
  executive: {
    planning: CognitiveLevel;
    flexibility: CognitiveLevel;
    inhibition: CognitiveLevel;
  };

  // 감각 처리
  sensory: {
    visualProcessing: SensoryLevel;
    auditoryProcessing: SensoryLevel;
    sensoryIntegration: SensoryLevel;
  };
}

enum CognitiveLevel {
  PROFOUND = 1,
  SEVERE = 2,
  MODERATE = 3,
  MILD = 4,
  TYPICAL = 5
}
```

### Phase 2: 적응형 UI 엔진
- 인지 프로파일 기반 자동 UI 조정
- 심볼 크기, 개수, 복잡도 동적 변경
- 시각적 산만함 최소화 옵션

### Phase 3: 예측/학습 알고리즘
- 사용 패턴 학습
- 시간대별/상황별 빈도 기반 제안
- 루틴 기반 메시지 예측

### Phase 4: 케어기버 연동
- 보호자 앱 (사용 현황, 긴급 알림)
- 전문가 대시보드 (진척도 분석)
- HIPAA/GDPR 호환 데이터 보호

---

## 철학

**홍익인간 (弘益人間)** - 널리 인간을 이롭게 하라

인지 능력과 관계없이 모든 사람이 자신의 생각과 감정을 표현할 수 있도록.
