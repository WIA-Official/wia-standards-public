# WIA Haptic Standard - Phase 1: Haptic Language Definition

## 목표
촉각 피드백의 표준 "언어"를 정의합니다.

## 1.1 기본 파형 표준

```typescript
interface HapticPrimitive {
  // 기본 파형
  waveform: 'sine' | 'square' | 'triangle' | 'sawtooth' | 'noise';

  // ADSR 엔벨로프
  envelope: {
    attack: number;      // 0-500ms
    decay: number;       // 0-500ms
    sustain: number;     // 0.0-1.0
    release: number;     // 0-500ms
  };

  // 주파수 (진동 속도)
  frequency: number;     // 1-300Hz (인체 감지 범위)

  // 강도
  intensity: number;     // 0.0-1.0
}
```

## 1.2 의미론적 카테고리

```typescript
enum HapticCategory {
  NAVIGATION = 'navigation',      // 방향, 거리, 장애물
  NOTIFICATION = 'notification',  // 알림, 경고
  CONFIRMATION = 'confirmation',  // 성공, 실패
  SPATIAL = 'spatial',            // 공간 정보
  TEMPORAL = 'temporal',          // 시간 정보
  SOCIAL = 'social',              // 사회적 신호
}

// 내비게이션 패턴
const NAV_PATTERNS = {
  CLEAR_PATH: '▁▁▁',              // 낮은 연속음
  OBSTACLE_NEAR: '▓▓▓',           // 강한 진동
  TURN_LEFT: '◀◀◀',              // 왼쪽 강조 (웨어러블 위치)
  TURN_RIGHT: '▶▶▶',             // 오른쪽 강조
  DESTINATION: '♪',               // 멜로디 패턴
};
```

## 1.3 신체 위치 매핑

```typescript
enum BodyLocation {
  // 손/손목
  LEFT_WRIST = 'left_wrist',
  RIGHT_WRIST = 'right_wrist',
  LEFT_PALM = 'left_palm',
  RIGHT_PALM = 'right_palm',

  // 팔
  LEFT_FOREARM = 'left_forearm',
  RIGHT_FOREARM = 'right_forearm',

  // 몸통 (조끼형)
  CHEST_LEFT = 'chest_left',
  CHEST_CENTER = 'chest_center',
  CHEST_RIGHT = 'chest_right',
  BACK_LEFT = 'back_left',
  BACK_CENTER = 'back_center',
  BACK_RIGHT = 'back_right',

  // 머리 (헤드셋/안경)
  FOREHEAD_LEFT = 'forehead_left',
  FOREHEAD_CENTER = 'forehead_center',
  FOREHEAD_RIGHT = 'forehead_right',
}
```

---

## 연구 과제

1. 기존 햅틱 표준 조사 (Apple Haptics, Android Vibration API)
2. 인체 진동 인식 연구 (주파수별 민감도)
3. 학습 용이성 연구 (패턴 구분 능력)

---

## 산출물

```
haptic/
├── spec/
│   ├── HAPTIC-PRIMITIVES.md
│   ├── SEMANTIC-MAPPING.md
│   └── BODY-LOCATION-MAP.md
├── schemas/
│   ├── haptic-pattern.schema.json
│   └── body-location.schema.json
```

---

## 다음: Phase 2 (디바이스 추상화 API)
