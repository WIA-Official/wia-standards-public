# 제2장: 물리적 센서 (온도, 압력, 염분)

## 해양의 기본 특성 측정

물리적 센서는 해수의 기본 상태 변수인 온도, 압력(깊이) 및 염분을 측정합니다. 이 세 가지 매개변수는 해수 밀도를 정의하며, 이는 해양 순환, 성층화 및 혼합을 구동합니다. 이들은 가장 널리 측정되는 해양 특성으로, 수세기 전으로 거슬러 올라가는 관측과 전 지구적 범위를 제공하는 현대 네트워크가 있습니다.

### 온도 센서

해양 온도는 극지 해역의 -2°C에서 열대 표면 해역의 30°C 이상, 심해의 0-2°C에서 열수 분출구의 400°C까지 다양합니다. 온도는 생물학적 대사, 화학 반응 속도, 음향 전파 및 해수 밀도에 영향을 미칩니다.

#### 서미스터 기반 센서

현대 해양 온도 센서는 주로 **서미스터**를 사용합니다 - 전기 저항이 온도에 따라 예측 가능하게 변하는 반도체 장치입니다.

**원리:** 저항은 온도가 증가함에 따라 감소합니다 (음의 온도 계수)

**장점:**
- 높은 정밀도: ±0.001°C 달성 가능
- 빠른 응답 시간: < 0.1초
- 작은 크기: 밀리미터 단위 가능
- 낮은 전력 소비
- 안정적인 보정

**구현:**
```typescript
interface ThermistorSensor {
  type: "thermistor";
  serialNumber: string;

  // Steinhart-Hart 방정식 계수
  calibration: {
    A: number;
    B: number;
    C: number;
    D?: number;              // 선택적 4차 항
  };

  // 측정 사양
  range: {
    min: number;             // °C
    max: number;             // °C
  };
  resolution: number;        // °C
  accuracy: number;          // °C
  responseTime: number;      // 초

  // 현재 판독값
  resistance: number;        // Ohms
}

function calculateTemperature(sensor: ThermistorSensor): number {
  const R = sensor.resistance;
  const lnR = Math.log(R);

  // Steinhart-Hart 방정식: 1/T = A + B*ln(R) + C*(ln(R))^3
  const { A, B, C, D } = sensor.calibration;

  const invT = A + B * lnR + C * Math.pow(lnR, 3) + (D || 0) * Math.pow(lnR, 4);

  const kelvin = 1 / invT;
  const celsius = kelvin - 273.15;

  return celsius;
}
```

#### 백금 저항 온도계 (PRT)

최고 정확도 응용 프로그램의 경우 **백금 저항 온도계**가 사용됩니다:

**원리:** 순수 백금 와이어의 저항이 온도에 따라 선형적으로 증가

**장점:**
- 우수한 안정성: 드리프트 < 0.001°C/년
- 매우 높은 정확도: ±0.0001°C 가능
- 국제 표준 (ITS-90 온도 스케일)
- 재현 가능한 제조

**단점:**
- 느린 응답 시간
- 높은 비용
- 더 큰 크기

#### KIOST 온도 센서 개발

한국해양과학기술원은 독자적인 온도 센서 기술을 개발했습니다:

**심해 온도 센서:**
- 측정 범위: -5°C ~ 40°C
- 정확도: ±0.002°C
- 내압: 6,000m 수심
- 장기 안정성: ±0.001°C/년

**고속 응답 온도 센서:**
- 응답 시간: 0.02초
- 난류 측정용
- 미세 구조 관측

### 압력 센서

압력 센서는 깊이(1 dbar ≈ 1미터)를 측정하고 조수, 쓰나미 및 내부파로 인한 압력 변화를 감지합니다.

#### 스트레인 게이지 압력 센서

대부분의 해양 압력 센서는 다이어프램에 접합된 스트레인 게이지를 사용합니다:

**원리:** 압력이 다이어프램을 변형시켜 접합된 저항기를 변형시키고 저항을 변경합니다

**유형:**
- **압저항 실리콘:** 높은 감도, 온도 의존적
- **금속 포일 스트레인 게이지:** 견고하고 안정적이지만 감도가 낮음
- **공진 수정:** 최고 정확도, 주파수 출력, 비싸다

```typescript
interface PressureSensor {
  type: "strain_gauge" | "quartz_resonator";
  serialNumber: string;

  calibration: {
    // 압력 = (주파수^2 * C) + D, 수정의 경우
    // 또는 압력 = A + B*전압, 스트레인 게이지의 경우
    coefficients: Record<string, number>;
    temperatureCompensation: {
      enabled: boolean;
      coefficients?: number[];
    };
  };

  range: {
    min: number;              // dbar (데시바)
    max: number;              // dbar
  };
  resolution: number;         // dbar
  accuracy: number;           // dbar

  // 드리프트 특성
  drift: {
    rate: number;             // dbar/년
    lastCalibration: Date;
  };
}

function calculatePressure(
  sensor: PressureSensor,
  rawFrequency: number,
  temperature: number
): number {
  const { coefficients, temperatureCompensation } = sensor.calibration;

  let pressure: number;

  if (sensor.type === "quartz_resonator") {
    // Digiquartz 공식: P = C*(T^2) + D
    const T = rawFrequency;
    pressure = coefficients.C * Math.pow(T, 2) + coefficients.D;

    // 온도 보상
    if (temperatureCompensation.enabled && temperatureCompensation.coefficients) {
      const [U0, Y1, Y2, Y3] = temperatureCompensation.coefficients;
      const tempCorrection = Y1 * temperature + Y2 * Math.pow(temperature, 2) +
                            Y3 * Math.pow(temperature, 3);
      pressure += tempCorrection;
    }
  } else {
    // 선형 스트레인 게이지
    pressure = coefficients.A + coefficients.B * rawFrequency;
  }

  // 드리프트 보정 적용
  const daysSinceCalibration =
    (Date.now() - sensor.drift.lastCalibration.getTime()) / (1000 * 60 * 60 * 24);
  const driftCorrection = (sensor.drift.rate / 365) * daysSinceCalibration;
  pressure -= driftCorrection;

  return pressure;
}

// 압력을 깊이로 변환
function pressureToDepth(pressure: number, latitude: number): number {
  // 압력에서 깊이에 대한 UNESCO 공식
  const g = 9.780318 * (1 + (5.2788e-3 + 2.36e-5 * latitude) *
            Math.pow(Math.sin(latitude * Math.PI / 180), 2));

  const c1 = 9.72659;
  const c2 = -2.2512e-5;
  const c3 = 2.279e-10;
  const c4 = -1.82e-15;

  const depth = c1 * pressure + c2 * Math.pow(pressure, 2) +
                c3 * Math.pow(pressure, 3) + c4 * Math.pow(pressure, 4);

  return depth;
}
```

#### 쓰나미 탐지 시스템

해저 압력 센서는 통과하는 쓰나미를 감지합니다:

**KIOST 쓰나미 조기경보 시스템:**
- 동해안 해저 압력계 네트워크
- 실시간 데이터 전송
- 조기 경보 발령 시스템
- 정확도: ±0.01 dbar

### 염분 센서 (전도도)

염분은 해수의 용해된 염, 주로 염화나트륨을 정량화합니다. 이는 밀도, 어는점 및 음속에 영향을 미칩니다. 염분은 전기 전도도 측정에서 파생됩니다.

#### 전도도 셀

**원리:** 해수는 전기를 전도합니다; 전도도는 염분과 온도에 따라 증가합니다

**셀 유형:**
- **유도형 (전극 없음):** 두 개의 토로이달 변압기, 해수가 회로를 완성, 오염에 강함
- **전극형 (덕트형):** 정밀한 기하학, 전극이 컨덕턴스 측정, 더 높은 정확도이지만 오염되기 쉬움

```typescript
interface ConductivitySensor {
  type: "inductive" | "electrode";
  serialNumber: string;

  // 셀 특성
  cellConstant: number;           // 기하학적 인수

  calibration: {
    coefficients: {
      g: number;                  // 셀 계수
      h: number;                  // 온도 계수
      i: number;                  // 압력 계수
      j: number;                  // 2차 압력 계수
    };
    date: Date;
  };

  range: {
    min: number;                  // S/m (Siemens/미터)
    max: number;
  };
  resolution: number;             // S/m
  accuracy: number;               // S/m
}

function calculateSalinity(
  conductivity: number,           // S/m
  temperature: number,            // °C
  pressure: number                // dbar
): number {
  // PSS-78 (실용 염분 척도 1978)
  // 염분은 무차원, 대략 천분율

  // 전도도 비율: 샘플 / 표준 해수 (S=35, T=15°C, P=0)
  const C_standard = 4.2914;      // S/m, S=35, T=15, P=0에서
  const R = conductivity / C_standard;

  // 온도 보정
  const rt = temperatureCorrection(temperature);
  const Rt = R / rt;

  // 압력 보정
  const rp = pressureCorrection(pressure, temperature);
  const Rp = Rt / rp;

  // 염분 계산
  const salinity = salinitySeries(Rp, temperature);

  return salinity;
}

function temperatureCorrection(t: number): number {
  // rt = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4
  const c = [0.6766097, 2.00564e-2, 1.104259e-4, -6.9698e-7, 1.0031e-9];

  return c[0] + c[1]*t + c[2]*Math.pow(t,2) + c[3]*Math.pow(t,3) + c[4]*Math.pow(t,4);
}

function pressureCorrection(p: number, t: number): number {
  // 복잡한 다항식 - 단순화된 버전
  const e1 = 2.070e-5;
  const e2 = -6.370e-10;
  const e3 = 3.989e-15;

  return 1 + (e1 + e2*p + e3*Math.pow(p,2)) * p;
}

function salinitySeries(Rp: number, t: number): number {
  // PSS-78의 다항식 계수
  const a = [0.0080, -0.1692, 25.3851, 14.0941, -7.0261, 2.7081];
  const b = [0.0005, -0.0056, -0.0066, -0.0375, 0.0636, -0.0144];

  const k = 0.0162;
  const dt = t - 15;

  let salinity = 0;
  for (let i = 0; i < a.length; i++) {
    salinity += a[i] * Math.pow(Rp, i/2);
  }

  // 염분에 대한 온도 보정
  let tempCorrection = 0;
  for (let i = 0; i < b.length; i++) {
    tempCorrection += b[i] * Math.pow(Rp, i/2);
  }
  salinity += tempCorrection * dt / (1 + k*dt);

  return salinity;
}
```

#### CTD 프로파일러

**전도도-온도-깊이** (CTD) 프로파일러는 물리적 해양학의 주력입니다:

**KIOST CTD 시스템:**
- 모델: Sea-Bird SBE 911plus (개량형)
- 깊이 등급: 6,800m
- 정확도: 온도 ±0.001°C, 염분 ±0.002 PSU
- 샘플링 속도: 24Hz
- 추가 센서: 용존 산소, 형광, 탁도

```typescript
interface CTDProfile {
  cast: {
    castId: string;
    date: Date;
    location: GeographicPosition;
    ship: string;
    maxDepth: number;
    operator: string;
  };

  // 센서
  sensors: {
    temperature: ThermistorSensor[];   // 기본 + 보조
    conductivity: ConductivitySensor[]; // 기본 + 보조
    pressure: PressureSensor;
  };

  // 하강 중 측정
  measurements: {
    pressure: number;              // dbar
    depth: number;                 // 미터
    temperature1: number;          // °C, 기본
    temperature2: number;          // °C, 보조
    conductivity1: number;         // S/m, 기본
    conductivity2: number;         // S/m, 보조
    salinity1: number;             // PSU
    salinity2: number;             // PSU
    soundVelocity: number;         // m/s
    density: number;               // kg/m³
    oxygen?: number;               // μmol/kg
    fluorescence?: number;         // mg/m³ 엽록소
    turbidity?: number;            // NTU

    qualityFlags: Record<string, QualityFlag>;
  }[];

  // 파생 제품
  derived: {
    mixedLayerDepth: number;       // 미터
    thermoclineDepth: number;      // 미터
    maxBuoyancyFrequency: number;  // rad/s
    temperatureInversion: boolean;
  };
}
```

### 해류 센서

해류는 열, 염, 영양염, 유생 및 오염 물질을 운반합니다. 해류 측정은 해양 순환을 이해하는 데 필수적입니다.

#### 음향 도플러 유속 프로파일러 (ADCP)

ADCP는 물속 입자에서 산란된 도플러 이동 반향을 감지하여 수속을 측정합니다:

**KIOST ADCP 운영:**
- 동해 종합 해양 과학 기지 설치
- 연속 해류 모니터링
- 데이터: 실시간 전송 및 공개
- 응용: 순환 패턴, 난류 연구

### 통합 다중 센서 시스템

현대 플랫폼은 여러 물리적 센서를 결합합니다:

**동해 종합 해양 과학 기지 센서:**
- CTD (온도, 염분, 압력)
- ADCP (해류)
- 파고계 (파도)
- 기상 센서 (바람, 기압)
- 생지화학 센서 (산소, 형광)
- 실시간 데이터 공개: www.khoa.go.kr

### 철학: 弘益人間 (홍익인간)

물리적 해양 센서는 모든 사람에게 이익이 되는 기본 지식을 제공함으로써 弘익人間을 구현합니다:

**기후 이해:** 온도 및 염분 측정은 모든 국가의 기후 예측을 알려주는 해양 열 흡수 및 순환 변화를 문서화합니다

**해양 안전:** 해류, 온도 및 염분의 정확한 측정은 전 세계적으로 안전한 항해 및 수색 구조 작업을 지원합니다

**식량 안보:** 물리 해양학 데이터는 어류 서식지 예측을 돕고 지속 가능한 어업 관리를 지원합니다

**재해 예방:** 쓰나미 센서 및 폭풍 해일 모니터링은 전 세계 연안 지역 사회를 보호합니다

**공유 지식:** 물리적 해양 데이터는 국제 데이터베이스를 통해 자유롭게 흐르며 모든 국가의 연구 및 운영 서비스를 가능하게 합니다

해양의 온도, 염분 및 해류는 국경을 알지 못합니다. 이를 정확하게 측정하고 그 지식을 공개적으로 공유하는 것은 모든 인류에게 도움이 됩니다.

---

**다음 장:** pH, 산소, 영양염 및 해양의 탄소 시스템을 측정하는 화학 센서를 탐구합니다 - 해양 건강과 기후 변화를 이해하는 데 중요합니다.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · 널리 인간을 이롭게 하라
