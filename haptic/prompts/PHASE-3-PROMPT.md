# WIA Haptic Standard - Phase 3: Spatial Encoding

## 목표
방향, 거리, 높이 등 공간 정보를 햅틱으로 인코딩합니다.

## 3.1 방향 인코딩

```typescript
interface DirectionalHaptic {
  // 360도 방향 표현
  encodeDirection(
    azimuth: number,    // 0-360도 (0=전방)
    elevation?: number  // -90~90도 (위/아래)
  ): HapticPattern[];

  // 다중 액추에이터 활용
  multiActuatorDirection: {
    // 4개 액추에이터 (전/후/좌/우)
    quad: QuadDirectionEncoder;
    // 8개 액추에이터 (8방향)
    octal: OctalDirectionEncoder;
    // 연속 위치 (조끼형)
    continuous: ContinuousDirectionEncoder;
  };
}

// 4방향 인코더
class QuadDirectionEncoder {
  encode(azimuth: number): ActuatorActivation[] {
    // 45도 구간별 활성화
    // 0도: 전방 강함
    // 90도: 우측 강함
    // 180도: 후방 강함
    // 270도: 좌측 강함
    // 중간: 보간
  }
}
```

## 3.2 거리 인코딩

```typescript
interface DistanceHaptic {
  // 거리 → 진동 주파수 매핑
  encodeDistance(
    distance: number,    // 미터
    maxRange: number     // 최대 감지 거리
  ): HapticPattern;

  // 인코딩 방식
  encodingMethod:
    | 'frequency'        // 가까울수록 빠른 진동
    | 'intensity'        // 가까울수록 강한 진동
    | 'rhythm'           // 가까울수록 짧은 간격
    | 'combined';        // 복합
}

// 예: 주파수 기반 거리 인코딩
function encodeDistanceByFrequency(distance: number): number {
  // 0m = 20Hz (빠른 진동, 위험!)
  // 5m = 2Hz (느린 진동, 안전)
  const normalizedDistance = Math.min(distance / 5, 1);
  return 20 - (normalizedDistance * 18);  // 20Hz → 2Hz
}
```

## 3.3 복합 공간 정보

```typescript
interface SpatialHapticScene {
  // 장면 전체 인코딩
  encodeScene(obstacles: Obstacle[]): HapticSequence;

  // 다중 객체 표현
  multiObjectEncoding: {
    // 시분할 다중화 (순차적 표시)
    timeDivision: (objects: Obstacle[]) => HapticSequence;
    // 공간 분할 (다른 위치 액추에이터)
    spatialDivision: (objects: Obstacle[]) => ActuatorMap;
    // 주파수 분할 (다른 주파수)
    frequencyDivision: (objects: Obstacle[]) => HapticLayers;
  };
}

interface Obstacle {
  type: 'wall' | 'person' | 'vehicle' | 'stairs' | 'drop';
  direction: number;  // 방향
  distance: number;   // 거리
  height: number;     // 높이
  priority: number;   // 위험도
}
```

---

## 산출물

```
haptic/
├── spec/
│   ├── DIRECTIONAL-ENCODING.md
│   ├── DISTANCE-ENCODING.md
│   └── SPATIAL-SCENE.md
├── api/
│   └── rust/src/
│       ├── spatial/
│       │   ├── direction.rs
│       │   ├── distance.rs
│       │   └── scene.rs
```

---

## 다음: Phase 4 (에코시스템 통합)
