# 8장: 구현 가이드

## 개요

이 장은 센서 통합, 정확도 표준, 배터리 최적화, 개인정보 보호 규정 준수 및 인증 요구사항을 다루는 WIA-IND-012 규격 피트니스 트래킹 시스템 구현에 대한 포괄적인 지침을 제공합니다.

---

## 8.1 센서 통합 요구사항

### 8.1.1 가속도계

**샘플링 속도:**
```
최소: 50 Hz
권장: 100 Hz
최대: 200 Hz (이상에서는 수익 감소)
```

**목적:**
- 걸음 감지
- 활동 분류
- 움직임 패턴
- 낙상 감지

**구성:**
```typescript
interface AccelerometerConfig {
  samplingRate: number;        // Hz
  sensitivity: 'low' | 'medium' | 'high';
  filterType: 'low_pass' | 'high_pass' | 'band_pass';
  cutoffFrequency: number;     // Hz
}

const config: AccelerometerConfig = {
  samplingRate: 100,
  sensitivity: 'medium',
  filterType: 'high_pass',
  cutoffFrequency: 0.5
};
```

**걸음 감지 알고리즘:**
```typescript
class StepDetector {
  private threshold: number = 1.2;  // g-force 임계값
  private minPeakDistance: number = 200;  // ms
  private stepCount: number = 0;

  processAccelData(x: number, y: number, z: number, timestamp: number): boolean {
    // 크기 계산
    const magnitude = Math.sqrt(x * x + y * y + z * z);

    // 걸음 확인
    if (magnitude > this.threshold &&
        timestamp - this.lastStepTime > this.minPeakDistance) {

      this.stepCount++;
      this.lastStepTime = timestamp;
      return true;  // 걸음 감지됨
    }

    return false;
  }
}
```

### 8.1.2 GPS

**샘플링 속도:**
```
활동 유형       속도        이유
걷기/하이킹:    1 Hz        느린 움직임, 배터리 효율적
달리기:         1-5 Hz      정확도와 배터리 균형
사이클링:       5 Hz        더 높은 속도로 더 많은 포인트 필요
레이싱:         10 Hz       최대 정확도
```

**정확도 요구사항:**
```typescript
interface GPSAccuracy {
  horizontal: number;      // 미터 (±)
  vertical: number;        // 미터 (±)
  confidence: number;      // 0-100%
}

const minimumAccuracy: GPSAccuracy = {
  horizontal: 10,
  vertical: 15,
  confidence: 95
};
```

### 8.1.3 심박수 모니터

**샘플링 속도:**
```
최소: 1 Hz (초당 1회 판독)
권장: 5 Hz (초당 5회 판독)
HRV 측정: 250-500 Hz (R-R 간격 감지용)
```

**광학 심박수 (PPG):**
```typescript
class OpticalHRMonitor {
  private ppgSignal: number[] = [];
  private heartRates: number[] = [];

  processSignal(ppgValue: number, timestamp: number): void {
    this.ppgSignal.push(ppgValue);

    // 10초 윈도우 유지
    if (this.ppgSignal.length > 50) {  // 5 Hz * 10초
      this.ppgSignal.shift();

      // 피크에서 심박수 계산
      const hr = this.detectHeartRate();
      if (hr) {
        this.heartRates.push(hr);
      }
    }
  }

  private detectHeartRate(): number | null {
    // PPG 신호에서 피크 찾기
    const peaks = this.findPeaks(this.ppgSignal);

    if (peaks.length < 2) return null;

    // 피크 간 평균 간격 계산
    const intervals: number[] = [];
    for (let i = 1; i < peaks.length; i++) {
      intervals.push(peaks[i] - peaks[i - 1]);
    }

    const avgInterval = intervals.reduce((a, b) => a + b) / intervals.length;

    // BPM으로 변환 (5 Hz 샘플링 가정)
    const samplesPerSecond = 5;
    const secondsPerBeat = avgInterval / samplesPerSecond;
    const bpm = 60 / secondsPerBeat;

    return Math.round(bpm);
  }
}
```

---

## 8.2 정확도 표준

### 8.2.1 필요한 정확도 임계값

```typescript
interface AccuracyStandards {
  stepCount: {
    error: number;             // ±5%
    minimumCount: number;      // 검증을 위한 100걸음
  };

  distance: {
    gpsError: number;          // ±2% 또는 50m, 더 큰 값
    estimatedError: number;    // ±10%
  };

  heartRate: {
    error: number;             // ±5 BPM 또는 ±5%, 더 큰 값
    restingRange: [number, number];   // 40-100 BPM 유효
    activeRange: [number, number];    // 60-220 BPM 유효
  };

  calories: {
    error: number;             // ±15%
  };

  elevation: {
    error: number;             // ±10m
  };
}

const accuracyStandards: AccuracyStandards = {
  stepCount: {
    error: 0.05,
    minimumCount: 100
  },
  distance: {
    gpsError: 0.02,
    estimatedError: 0.10
  },
  heartRate: {
    error: 5,
    restingRange: [40, 100],
    activeRange: [60, 220]
  },
  calories: {
    error: 0.15
  },
  elevation: {
    error: 10
  }
};
```

---

## 8.3 배터리 최적화

### 8.3.1 적응형 샘플링 전략

```typescript
enum ActivityState {
  STATIONARY,
  WALKING,
  RUNNING,
  CYCLING,
  HIGH_INTENSITY
}

interface SamplingConfig {
  gpsRate: number;          // Hz
  accelerometerRate: number; // Hz
  heartRateRate: number;    // Hz
}

function getOptimalSamplingConfig(state: ActivityState): SamplingConfig {
  switch (state) {
    case ActivityState.STATIONARY:
      return {
        gpsRate: 0,           // GPS 끔
        accelerometerRate: 10, // 움직임 감지를 위한 낮은 속도
        heartRateRate: 0.2    // 5초마다
      };

    case ActivityState.WALKING:
      return {
        gpsRate: 1,
        accelerometerRate: 50,
        heartRateRate: 1
      };

    case ActivityState.RUNNING:
      return {
        gpsRate: 1,
        accelerometerRate: 100,
        heartRateRate: 1
      };

    case ActivityState.CYCLING:
      return {
        gpsRate: 5,            // 속도를 위해 더 높음
        accelerometerRate: 50,
        heartRateRate: 1
      };
  }
}
```

### 8.3.2 배터리 소비 추정

```
구성요소           전력 소비      배터리 영향 (1시간)
GPS (연속):        50-150 mW      ~5-10% 배터리
GPS (1 Hz):        20-50 mW       ~2-4% 배터리
가속도계:          0.5-2 mW       ~0.1% 배터리
광학 심박수:       5-15 mW        ~0.5-1% 배터리
Bluetooth:        10-30 mW       ~1-2% 배터리
화면 (활성):       200-500 mW     ~15-25% 배터리

총합 (화면 끔 활성 추적):
~30-80 mW = 3-6시간의 연속 추적
```

---

## 8.4 개인정보 보호 규정 준수

### 8.4.1 GDPR 준수 체크리스트

**데이터 수집:**
```typescript
interface ConsentRecord {
  userId: string;
  purpose: string;
  granted: boolean;
  timestamp: Date;
}

class GDPRCompliance {
  // 명시적 동의 획득
  async requestConsent(userId: string, purpose: string): Promise<boolean> {
    const consent = await this.showConsentDialog(purpose);

    const record: ConsentRecord = {
      userId,
      purpose,
      granted: consent,
      timestamp: new Date()
    };

    await this.storeConsentRecord(record);
    return consent;
  }

  // 액세스 권한
  async exportUserData(userId: string): Promise<Blob> {
    const data = await this.collectAllUserData(userId);
    const json = JSON.stringify(data, null, 2);
    return new Blob([json], { type: 'application/json' });
  }

  // 삭제 권한 (잊혀질 권리)
  async deleteUserData(userId: string): Promise<void> {
    await this.deleteActivities(userId);
    await this.deleteWorkouts(userId);
    await this.deleteHealthMetrics(userId);
    await this.anonymizeAnalytics(userId);
  }
}
```

### 8.4.2 HIPAA 준수 (의료 통합용)

**요구사항:**
```typescript
class HIPAASecureStorage {
  async storeData(data: any, userId: string): Promise<void> {
    // 데이터 암호화
    const encrypted = await this.encrypt(data);

    // 액세스 로그와 함께 저장
    await this.saveEncrypted(encrypted, userId);
    await this.logAccess(userId, 'write', new Date());
  }

  async retrieveData(userId: string, accessorId: string): Promise<any> {
    // 권한 확인
    if (!await this.isAuthorized(accessorId, userId)) {
      throw new Error('무단 액세스 시도');
    }

    // 액세스 로그
    await this.logAccess(userId, 'read', new Date(), accessorId);

    // 검색 및 복호화
    const encrypted = await this.loadEncrypted(userId);
    return await this.decrypt(encrypted);
  }
}
```

---

## 8.5 인증 요구사항

### 8.5.1 인증 수준

**레벨 1: 기본**
- ✓ 걸음 수 (±5% 정확도)
- ✓ 기본 활동 로깅
- ✓ 수동 칼로리 입력
- ✓ 간단한 목표 추적
- ✓ 데이터 내보내기 (JSON)

**레벨 2: 표준**
- ✓ 모든 레벨 1 기능
- ✓ GPS 추적 (±2% 거리 정확도)
- ✓ 심박수 모니터링 (±5 BPM 정확도)
- ✓ 자동 칼로리 계산 (MET 기반)
- ✓ 운동 분석

**레벨 3: 고급**
- ✓ 모든 레벨 2 기능
- ✓ 다중 스포츠 지원 (10개 이상 활동 유형)
- ✓ HRV 측정 (RMSSD, SDNN)
- ✓ VO2 Max 추정
- ✓ 트레이닝 부하 계산 (TRIMP/TSS)
- ✓ 크로스 플랫폼 동기화

**레벨 4: 전문가**
- ✓ 모든 레벨 3 기능
- ✓ 의료급 정확도 (±3%)
- ✓ FHIR 의료 통합
- ✓ 연구급 데이터 내보내기
- ✓ HIPAA/GDPR 준수
- ✓ 감사 로깅

### 8.5.2 인증 프로세스

**1단계: 자체 평가**
- 온라인 체크리스트 완료
- 기능 확인

**2단계: 신청 제출**
- 온라인 신청서 작성
- 문서 제출
- 인증 수수료 지불

**3단계: 테스트**
- 정확도 테스트 (통제 환경)
- 상호운용성 테스트 (데이터 교환)
- 보안 테스트 (침투 테스트)

**4단계: 검토 및 인증**
- 테스트 결과 검토
- 격차 해결
- 인증 수령
- 인증 배지 표시

---

## 8.6 참조 구현

### 8.6.1 완전한 예시

**TypeScript SDK:**
```typescript
import { WIAFitnessTracker } from '@wia/fitness-tracking';

// 초기화
const tracker = new WIAFitnessTracker({
  apiKey: 'your_api_key',
  userId: 'user_12345',
  certificationLevel: 2
});

// 추적 시작
await tracker.startActivity({
  type: 'running',
  enableGPS: true,
  enableHeartRate: true
});

// 실시간 업데이트
tracker.on('heartRate', (hr: number) => {
  console.log(`심박수: ${hr} BPM`);
});

tracker.on('distance', (distance: number) => {
  console.log(`거리: ${(distance / 1000).toFixed(2)} km`);
});

// 중지 및 저장
const activity = await tracker.stopActivity();
console.log(`활동 저장됨: ${activity.id}`);
console.log(`칼로리: ${activity.calories} kcal`);
console.log(`거리: ${(activity.distance / 1000).toFixed(2)} km`);
```

---

## 핵심 요점

✓ 정확한 걸음 감지를 위해 50-100 Hz의 가속도계

✓ GPS 샘플링 속도는 활동 유형에 따라 달라짐 (1-10 Hz)

✓ 심박수 모니터링은 1-5 Hz 샘플링 필요

✓ 정확도 표준: ±5% 걸음, ±2% GPS 거리, ±5 BPM 심박수

✓ 적응형 샘플링 및 배치 처리를 통한 배터리 최적화

✓ GDPR 준수는 명시적 동의 및 데이터 이동성 필요

✓ HIPAA 준수는 암호화, 감사 로그, BAA 필요

✓ 기본에서 전문가까지 4개 인증 수준

✓ 인증 프로세스에는 정확도 테스트 및 보안 감사 포함

✓ 빠른 개발을 위한 참조 구현 가능

---

## 결론

WIA-IND-012는 정확하고 상호운용 가능하며 개인정보를 보호하는 피트니스 트래킹 시스템을 구축하기 위한 포괄적인 프레임워크를 제공합니다. 이 가이드북의 지침을 따라 개발자는 생태계 전반에서 원활하게 작동하면서 사용자 신뢰와 규정 준수를 유지하는 인증된 구현을 만들 수 있습니다.

피트니스 트래킹의 미래는 개방적이고 표준화되어 있으며 사용자 중심적입니다. 이를 구축하는 데 동참하십시오.

**弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

---

**이전:** [← 7장: 시스템 통합](07-system-integration.md)
**시작:** [← 목차로 돌아가기](00-index.md)

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
