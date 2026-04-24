# WIA Cognitive AAC - Phase 2: Adaptive UI Engine
# 적응형 UI 엔진 명세서

**Version**: 1.0.0
**Date**: 2025-01-14
**Status**: Complete

---

## 철학
**홍익인간 (弘益人間)** - 널리 인간을 이롭게 하라

---

## 1. 개요

### 1.1 목적
Phase 1에서 정의한 인지 프로파일을 기반으로 AAC 인터페이스를 동적으로 적응시키는 UI 엔진을 구현합니다.

### 1.2 핵심 기능
- 인지 프로파일 → UI 설정 자동 변환
- 실시간 인지 부하 모니터링 및 조정
- 자폐/치매 특화 UI 조정
- 사용 이력 기반 최적화

### 1.3 기술 스택
- **언어**: TypeScript 5.x
- **프레임워크**: React 18+
- **빌드**: tsup
- **패키지**: @wia/cognitive-aac

---

## 2. 아키텍처

### 2.1 모듈 구조

```
@wia/cognitive-aac
├── types/              # 타입 정의
│   └── index.ts
├── engine/             # 핵심 엔진 로직
│   ├── UIEngine.ts     # 메인 적응 엔진
│   ├── ProfileAdapter.ts # 프로파일 → UI 변환
│   └── Presets.ts      # 인지 수준별 프리셋
├── components/         # React 컴포넌트
│   ├── AdaptiveGrid.tsx
│   ├── AdaptiveSymbol.tsx
│   ├── AdaptiveButton.tsx
│   └── CognitiveLoadManager.tsx
└── hooks/              # React Hooks
    ├── useCognitiveProfile.ts
    └── useAdaptiveUI.ts
```

### 2.2 데이터 흐름

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│ CognitiveProfile │────▶│  ProfileAdapter  │────▶│ UIConfiguration │
└─────────────────┘     └──────────────────┘     └─────────────────┘
                                                          │
                                                          ▼
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│ User Interaction│────▶│    UIEngine      │────▶│  UIAdjustment   │
└─────────────────┘     └──────────────────┘     └─────────────────┘
                                │
                                ▼
                        ┌──────────────────┐
                        │  React Components │
                        └──────────────────┘
```

---

## 3. 핵심 엔진

### 3.1 UIEngine

```typescript
interface AdaptiveUIEngine {
  // 프로파일 기반 UI 생성
  generateUI(profile: CognitiveProfile): UIConfiguration;

  // 실시간 조정
  adjustUI(event: UserInteractionEvent): UIAdjustment | null;

  // 사용 이력 기반 최적화
  optimizeFromUsage(history: UsageHistory): UIConfiguration;
}
```

### 3.2 EngineConfig

```typescript
interface EngineConfig {
  enableRealTimeAdjustment: boolean;  // 실시간 조정 활성화
  enableLearning: boolean;            // 학습 기반 최적화
  adjustmentThreshold: number;        // 조정 임계값 (0-1)
  maxAdjustmentsPerSession: number;   // 세션당 최대 조정 횟수
  minEventsForLearning: number;       // 학습에 필요한 최소 이벤트 수
}
```

### 3.3 프로파일 유형별 엔진 생성

```typescript
// 자폐 스펙트럼용 (민감한 조정)
const engine = UIEngine.forAutism(profile, {
  adjustmentThreshold: 0.25,
});

// 치매용 (안정성 우선)
const engine = UIEngine.forDementia(profile, {
  enableLearning: false,
  maxAdjustmentsPerSession: 5,
});

// 일반용
const engine = UIEngine.forGeneral(profile);
```

---

## 4. 인지 수준별 프리셋

### 4.1 UI 설정 매핑

| 인지 수준 | 그리드 | 셀 크기 | 상징 | 피드백 | 인지 부하 |
|:--------:|:------:|:------:|:----:|:------:|:---------:|
| PROFOUND (1) | 2×2 | xlarge | 사진, 레이블 없음 | 모두 활성 | 4항목, 100% 필터 |
| SEVERE (2) | 3×2 | large | 사진, 하단 레이블 | 모두 활성 | 6항목, 80% 필터 |
| MODERATE (3) | 4×3 | medium | PCS, 하단 레이블 | 오디오만 | 12항목, 50% 필터 |
| MILD (4) | 5×4 | medium | PCS, 하단 레이블 | 없음 | 20항목, 20% 필터 |
| TYPICAL (5) | 6×5 | small | PCS, 하단 레이블 | 없음 | 30항목, 필터 없음 |

### 4.2 Dwell Time 설정

| 인지 수준 | Dwell Time | 확인 필요 |
|:--------:|:----------:|:---------:|
| PROFOUND | 2000ms | Yes |
| SEVERE | 1500ms | Yes |
| MODERATE | 1000ms | No |
| MILD | 800ms | No |
| TYPICAL | 0ms (탭) | No |

---

## 5. 자폐 스펙트럼 특화 조정

### 5.1 AutismUIAdjustments

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
    interests: string[];
    integratedInUI: boolean;       // UI에 관심사 통합
  };
}
```

### 5.2 감각 반응성 기반 조정

| 감각 | 과반응 (Hyper) | 저반응 (Hypo) |
|:----:|:--------------|:-------------|
| **시각** | 밝기↓, 채도↓, 애니메이션 비활성 | 고대비, 눈에 띄는 색상 |
| **청각** | 소리 비활성, 볼륨↓ | 오디오 피드백 활성 |
| **촉각** | 햅틱 비활성 | 햅틱 활성 |

### 5.3 DSM-5 지원 수준별 프리셋

| 지원 수준 | 특징 | UI 조정 |
|:--------:|------|--------|
| Level 1 | 지원 필요 | 전환 경고, 관심사 통합 |
| Level 2 | 상당한 지원 | + 애니메이션↓, 채도↓, 시각적 일정 |
| Level 3 | 매우 상당한 지원 | + 조용한 모드, 최대 단순화 |

---

## 6. 치매 특화 조정

### 6.1 DementiaUIAdjustments

```typescript
interface DementiaUIAdjustments {
  // 친숙함 강조
  familiarity: {
    usePersonalPhotos: boolean;    // 가족 사진 등
    familiarFaces: FamiliarPerson[];
    lifeStoryIntegration: boolean;
  };

  // 단순화
  simplification: {
    minimalNavigation: boolean;    // 깊이 1 네비게이션
    largeText: boolean;
    reducedChoices: number;
    clearLabeling: boolean;
  };

  // 시간 지원
  temporalSupport: {
    dayNightAwareness: boolean;
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

### 6.2 치매 단계별 UI 복잡도

| 단계 | 화면 항목 | 메뉴 깊이 | 셀 크기 | 특징 |
|:----:|:--------:|:--------:|:------:|------|
| MCI | 16 | 2 | medium | 단어 찾기 지원 |
| 초기 | 12 | 1-2 | large | 회상 요소 |
| 중기 | 6 | 1 | large | 친숙한 얼굴, 기본 요구 |
| 말기 | 4 | 1 | xlarge | 예/아니오, 최소 요구 |

### 6.3 회상 요소

```typescript
interface ReminiscenceElements {
  familyPhotos: { name: string; relationship: string }[];
  preferredMusic: string[];
  accessibleMemoryPeriods: string[];
  suggestedActivities: string[];
}
```

---

## 7. React 컴포넌트

### 7.1 AdaptiveGrid

```tsx
<AdaptiveGrid
  page={symbolPage}
  config={uiConfig}
  selectedItems={['item-1']}
  onSelectItem={(item) => handleSelect(item)}
  onNavigate={(pageId) => handleNavigate(pageId)}
/>
```

### 7.2 AdaptiveSymbol

```tsx
<AdaptiveSymbol
  item={symbolItem}
  symbolConfig={config.symbols}
  interactionConfig={config.interaction}
  cellSize={config.grid.cellSize}
  onSelect={handleSelect}
  onDwellComplete={handleDwellComplete}
/>
```

### 7.3 CognitiveLoadManager

```tsx
<CognitiveLoadManagerProvider
  profile={profile}
  onConfigChange={handleConfigChange}
  onOverload={handleOverload}
>
  <CognitiveLoadIndicator position="top-right" />
  <AdaptiveGrid ... />
</CognitiveLoadManagerProvider>
```

---

## 8. React Hooks

### 8.1 useCognitiveProfile

```typescript
const {
  profile,
  isAutism,
  isDementia,
  cognitiveLevel,
  setProfile,
  updateProfile,
  getStrengths,
  getChallenges,
} = useCognitiveProfile({
  initialProfile,
  persistKey: 'user-profile',
});
```

### 8.2 useAdaptiveUI

```typescript
const {
  config,
  autismAdjustments,
  dementiaAdjustments,
  specialInterestElements,
  reminiscenceElements,
  loadLevel,
  recordInteraction,
  simplify,
  resetConfig,
} = useAdaptiveUI({
  profile,
  onConfigChange: handleConfigChange,
});
```

---

## 9. 인지 부하 모니터링

### 9.1 부하 상태

```typescript
interface CognitiveLoadState {
  currentLoad: number;           // 0-1
  status: 'optimal' | 'elevated' | 'high' | 'overloaded';
  errorRate: number;
  averageResponseTime: number;
  sessionDuration: number;
  adjustmentsMade: number;
}
```

### 9.2 부하 상태 임계값

| 상태 | 부하 수준 | 권장 조치 |
|:----:|:--------:|----------|
| optimal | < 0.3 | 유지 |
| elevated | 0.3-0.5 | 모니터링 |
| high | 0.5-0.7 | 단순화 고려 |
| overloaded | > 0.7 | 즉시 단순화 |

### 9.3 자동 조정 트리거

- 인지 부하 임계값 초과
- 연속 오류 3회 이상
- 연속 타임아웃 2회 이상
- 평균 응답 시간 급증

---

## 10. API 사용 예시

### 10.1 기본 사용

```typescript
import {
  UIEngine,
  useCognitiveProfile,
  useAdaptiveUI,
  AdaptiveGrid,
  CognitiveLoadManagerProvider,
} from '@wia/cognitive-aac';

function AACApp() {
  const { profile } = useCognitiveProfile({
    initialProfile: loadedProfile,
  });

  const { config, recordInteraction } = useAdaptiveUI({
    profile: profile!,
  });

  return (
    <CognitiveLoadManagerProvider profile={profile!}>
      <AdaptiveGrid
        page={currentPage}
        config={config}
        onSelectItem={(item) => {
          recordInteraction({
            type: 'selection',
            targetId: item.id,
            successful: true,
          });
        }}
      />
    </CognitiveLoadManagerProvider>
  );
}
```

### 10.2 자폐 스펙트럼 특화

```typescript
const { config, autismAdjustments, specialInterestElements } = useAdaptiveUI({
  profile: autismProfile,
});

// 특별 관심사 활용
if (specialInterestElements?.canUseForMotivation) {
  const topics = specialInterestElements.topics;
  // 관심사 기반 어휘/테마 적용
}

// 감각 조정 적용
if (autismAdjustments?.sensoryAccommodations.reducedAnimation) {
  // 애니메이션 비활성화
}
```

### 10.3 치매 특화

```typescript
const { config, dementiaAdjustments, reminiscenceElements } = useAdaptiveUI({
  profile: dementiaProfile,
});

// 회상 요소 활용
if (reminiscenceElements) {
  const familyPhotos = reminiscenceElements.familyPhotos;
  const preferredMusic = reminiscenceElements.preferredMusic;
  // 친숙한 요소 통합
}

// 시간 지원
if (dementiaAdjustments?.temporalSupport.mealTimeReminders) {
  // 식사 시간 알림 활성화
}
```

---

## 11. 접근성 고려사항

### 11.1 WCAG 2.2 준수

- **1.4.3** 명암비 최소 4.5:1 (고대비 모드 7:1)
- **2.1.1** 키보드 접근성
- **2.3.1** 깜빡임 없음 (자폐 고려)
- **2.4.7** 포커스 표시
- **2.5.5** 터치 영역 최소 44×44px

### 11.2 인지 접근성 (ISO 21801-1)

- 단순하고 일관된 레이아웃
- 명확한 시각적 피드백
- 오류 방지 및 복구
- 적응형 복잡도

---

## 12. 파일 목록

```
cognitive-aac/api/typescript/
├── package.json
├── tsconfig.json
└── src/
    ├── index.ts
    ├── types/
    │   └── index.ts
    ├── engine/
    │   ├── index.ts
    │   ├── UIEngine.ts
    │   ├── ProfileAdapter.ts
    │   └── Presets.ts
    ├── components/
    │   ├── index.ts
    │   ├── AdaptiveGrid.tsx
    │   ├── AdaptiveSymbol.tsx
    │   ├── AdaptiveButton.tsx
    │   └── CognitiveLoadManager.tsx
    └── hooks/
        ├── index.ts
        ├── useCognitiveProfile.ts
        └── useAdaptiveUI.ts
```

---

## 13. 다음 단계: Phase 3

Phase 3에서는 **예측/학습 알고리즘**을 구현합니다:
- 사용 패턴 분석
- 어휘 예측
- 상황 인식 기반 제안
- 개인화 학습

```bash
cd cognitive-aac && cat prompts/PHASE-3-PROMPT.md
```

---

## 14. 참조

### 14.1 접근성 표준
- [WCAG 2.2](https://www.w3.org/TR/WCAG22/)
- [ISO 21801-1:2020](https://www.iso.org/standard/71711.html)

### 14.2 AAC 리소스
- [ASHA AAC Practice Portal](https://www.asha.org/practice-portal/professional-issues/augmentative-and-alternative-communication/)
- [ISAAC](https://www.isaac-online.org/)

---

<div align="center">

**WIA Cognitive AAC - Phase 2 Complete**

*"인지를 이해하고, 인터페이스를 적응시키고, 소통을 연결하다"*

**홍익인간** - 널리 인간을 이롭게 하라

**Phase 3으로 계속 →**

</div>
