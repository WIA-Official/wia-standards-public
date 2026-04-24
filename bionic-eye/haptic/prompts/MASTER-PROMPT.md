# WIA Haptic Standard - Master Prompt

## 프로젝트 개요

**WIA Haptic Protocol**은 시각장애인을 위한 촉각/햅틱 피드백 표준입니다.

### 핵심 문제
- [개발/표준화/구현 모두 초기 단계](https://arxiv.org/html/2412.19105v1)
- 진동 패턴 → 의미 매핑 표준 없음
- 제조사마다 다른 햅틱 패턴 → 학습 비용
- 공통 인터페이스 포인트(HapticGL) 없음

### WIA 솔루션

| Phase | 내용 | 산출물 |
|-------|------|--------|
| 1 | 햅틱 언어 표준 | 진동 패턴 → 의미 매핑 |
| 2 | 디바이스 추상화 API | 하드웨어 독립적 SDK |
| 3 | 공간 인코딩 | 방향/거리 햅틱 표현 |
| 4 | 에코시스템 통합 | 내비게이션, 스마트홈 |

---

## 핵심 기술 컨셉

### 햅틱 패턴 언어

```typescript
interface HapticPattern {
  id: string;
  semanticMeaning: string;      // "obstacle_left", "path_clear", etc.
  waveform: HapticWaveform;
  duration: number;             // ms
  intensity: number;            // 0.0 - 1.0
  frequency: number;            // Hz
  position?: BodyLocation;      // 촉각 위치 (웨어러블용)
}

interface HapticWaveform {
  type: 'sine' | 'square' | 'sawtooth' | 'custom';
  envelope: {
    attack: number;   // ms
    decay: number;
    sustain: number;  // 0.0 - 1.0
    release: number;  // ms
  };
  modulation?: {
    type: 'am' | 'fm';
    frequency: number;
    depth: number;
  };
}
```

### 의미론적 햅틱 매핑

```typescript
const HAPTIC_VOCABULARY: Record<string, HapticPattern> = {
  // 내비게이션
  'nav.forward': { ... },          // 전진 안전
  'nav.stop': { ... },             // 정지
  'nav.left': { ... },             // 좌회전
  'nav.right': { ... },            // 우회전
  'nav.obstacle': { ... },         // 장애물

  // 거리 인코딩 (진동 빈도로)
  'distance.near': { frequency: 10, ... },    // 가까움
  'distance.medium': { frequency: 5, ... },   // 중간
  'distance.far': { frequency: 2, ... },      // 멀리

  // 알림
  'alert.info': { ... },
  'alert.warning': { ... },
  'alert.danger': { ... },

  // 확인
  'confirm.success': { ... },
  'confirm.error': { ... },
};
```

---

## 4-Phase 구조

### Phase 1: 햅틱 언어 표준 정의
- 기본 파형 정의 (ADSR envelope)
- 의미론적 매핑 사전
- 국제 표준 조사 (ISO, IEEE)

### Phase 2: 디바이스 추상화 API
- 하드웨어 독립적 인터페이스
- 다양한 액추에이터 지원 (ERM, LRA, piezo)
- 웨어러블 위치 매핑 (손목, 팔, 조끼 등)

### Phase 3: 공간 인코딩
- 방향 인코딩 (360도)
- 거리 인코딩 (연속적)
- 높이 인코딩 (상/중/하)
- 복합 공간 정보

### Phase 4: 에코시스템 통합
- 내비게이션 앱 연동 (Google Maps, Apple Maps)
- 스마트홈 연동 (Matter/Thread)
- 게이밍 연동
- WIA 에코시스템 (AAC, Eye Gaze 등)

---

## 참조 리소스

- [dotLumen Haptic Glasses (CES 2024)](https://www.letsenvision.com/blog/ces-2024-dotlumens-haptic-navigation-smart-glasses)
- [Shape-changing haptic interface (Nature 2024)](https://www.nature.com/articles/s41598-024-79845-7)
- 표준: IEEE 1918.1 (Tactile Internet)

---

## 철학

**홍익인간 (弘益人間)** - 널리 인간을 이롭게 하라

촉각으로 세상을 인식하는 분들이 어떤 기기를 사용하든 동일한 언어로 소통할 수 있도록.
