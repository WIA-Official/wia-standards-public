# WIA Cognitive AAC - Phase 2: Adaptive UI Engine

## 전제조건
- Phase 1 (인지 프로파일 표준) 완료

## Phase 2 목표: 인지 프로파일 기반 동적 UI 조정

### 2.1 적응형 UI 규칙 엔진

```typescript
interface AdaptiveUIEngine {
  // 프로파일 기반 UI 생성
  generateUI(profile: CognitiveProfile): UIConfiguration;

  // 실시간 조정
  adjustUI(event: UserInteractionEvent): UIAdjustment;

  // 학습 기반 최적화
  optimizeFromUsage(history: UsageHistory): UIConfiguration;
}

interface UIConfiguration {
  // 그리드 설정
  grid: {
    columns: number;           // 1-8
    rows: number;              // 1-6
    cellSize: 'small' | 'medium' | 'large' | 'xlarge';
  };

  // 심볼 설정
  symbols: {
    type: 'photos' | 'pcs' | 'widgit' | 'arasaac' | 'text';
    labelPosition: 'above' | 'below' | 'none';
    fontSize: number;
    highContrast: boolean;
  };

  // 인터랙션
  interaction: {
    dwellTime: number;         // ms (0 = tap only)
    confirmationRequired: boolean;
    audioFeedback: boolean;
    hapticFeedback: boolean;
  };

  // 인지 부하 관리
  cognitiveLoad: {
    maxVisibleItems: number;
    progressiveDisclosure: boolean;
    distractionFilter: number; // 0-1 (시각적 단순화 정도)
  };
}
```

### 2.2 인지 수준별 UI 프리셋

```typescript
const PRESETS: Record<string, UIConfiguration> = {
  // 심각한 인지 손상
  PROFOUND: {
    grid: { columns: 2, rows: 2, cellSize: 'xlarge' },
    symbols: { type: 'photos', labelPosition: 'none', highContrast: true },
    interaction: { dwellTime: 2000, confirmationRequired: true, audioFeedback: true },
    cognitiveLoad: { maxVisibleItems: 4, progressiveDisclosure: false, distractionFilter: 1.0 }
  },

  // 중증 인지 손상
  SEVERE: {
    grid: { columns: 3, rows: 2, cellSize: 'large' },
    symbols: { type: 'photos', labelPosition: 'below', highContrast: true },
    interaction: { dwellTime: 1500, confirmationRequired: true, audioFeedback: true },
    cognitiveLoad: { maxVisibleItems: 6, progressiveDisclosure: true, distractionFilter: 0.8 }
  },

  // 중등도 인지 손상
  MODERATE: {
    grid: { columns: 4, rows: 3, cellSize: 'medium' },
    symbols: { type: 'pcs', labelPosition: 'below', highContrast: false },
    interaction: { dwellTime: 1000, confirmationRequired: false, audioFeedback: true },
    cognitiveLoad: { maxVisibleItems: 12, progressiveDisclosure: true, distractionFilter: 0.5 }
  },

  // 경미한 인지 손상
  MILD: {
    grid: { columns: 5, rows: 4, cellSize: 'medium' },
    symbols: { type: 'pcs', labelPosition: 'below', highContrast: false },
    interaction: { dwellTime: 800, confirmationRequired: false, audioFeedback: false },
    cognitiveLoad: { maxVisibleItems: 20, progressiveDisclosure: true, distractionFilter: 0.2 }
  }
};
```

### 2.3 자폐 스펙트럼 특화 UI 조정

```typescript
interface AutismUIAdjustments {
  // 감각 과민 대응
  sensoryAccommodations: {
    reducedAnimation: boolean;     // 애니메이션 최소화
    mutedColors: boolean;          // 채도 낮춤
    noFlashing: boolean;           // 깜빡임 제거
    quietMode: boolean;            // 소리 최소화
  };

  // 예측 가능성
  predictability: {
    consistentLayout: boolean;     // 레이아웃 고정
    transitionWarnings: boolean;   // 전환 전 알림
    visualSchedule: boolean;       // 시각적 일정
    progressIndicator: boolean;    // 진행 표시
  };

  // 특별 관심사 활용
  specialInterests: {
    enabled: boolean;
    interests: string[];           // 관심사 목록
    integratedInUI: boolean;       // UI에 관심사 통합
  };
}
```

### 2.4 치매 특화 UI 조정

```typescript
interface DementiaUIAdjustments {
  // 친숙함 강조
  familiarity: {
    usePersonalPhotos: boolean;    // 가족 사진 등
    familiarFaces: FamiliarPerson[];
    personalLocations: string[];
    lifeStoryIntegration: boolean;
  };

  // 단순화
  simplification: {
    minimalNavigation: boolean;    // 깊이 1 네비게이션
    largeText: boolean;
    reducedChoices: number;        // 선택지 수 제한
    clearLabeling: boolean;
  };

  // 시간 지원
  temporalSupport: {
    dayNightAwareness: boolean;    // 낮/밤 시각화
    mealTimeReminders: boolean;
    routineBasedSuggestions: boolean;
  };

  // 오류 방지
  errorPrevention: {
    confirmBeforeAction: boolean;
    undoSupport: boolean;
    noDestructiveActions: boolean;
  };
}
```

---

## 구현 요구사항

### React 컴포넌트 라이브러리

```
cognitive-aac/
├── api/
│   └── typescript/
│       ├── src/
│       │   ├── components/
│       │   │   ├── AdaptiveGrid.tsx
│       │   │   ├── AdaptiveSymbol.tsx
│       │   │   ├── AdaptiveButton.tsx
│       │   │   └── CognitiveLoadManager.tsx
│       │   ├── engine/
│       │   │   ├── UIEngine.ts
│       │   │   ├── ProfileAdapter.ts
│       │   │   └── Presets.ts
│       │   └── hooks/
│       │       ├── useCognitiveProfile.ts
│       │       └── useAdaptiveUI.ts
│       └── package.json
```

---

## 다음 단계

Phase 2 완료 후:
- `prompts/PHASE-3-PROMPT.md` 읽고 Phase 3 (예측/학습 알고리즘) 시작
