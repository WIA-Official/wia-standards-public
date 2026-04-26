# WIA-CITY-010: 냉난방 시스템 표준 ❄️

> **홍익인간 (弘益人間) (홍익인간) - 널리 인간을 이롭게 하라**

건물 냉난방 시스템(HVAC)의 모니터링, 제어, 최적화를 위한 포괄적 표준입니다.

## 🌟 개요

WIA-CITY-010 냉난방 시스템 표준은 다양한 HVAC 장비, 제어 프로토콜, 에너지 관리를 통합하여 효율적이고 쾌적한 실내 환경을 제공합니다.

### 철학

**홍익인간 (弘益人間) (홍익인간)** - "널리 인간을 이롭게 하라"의 철학을 바탕으로, 이 표준은 모든 건물 사용자에게 쾌적한 환경을 제공하면서 에너지 효율과 지속가능성을 추구합니다.

## 📊 HVAC의 중요성

### 현황 (2024)
- **에너지 소비**: 건물 에너지의 40-60% 차지
- **시장 규모**: 글로벌 HVAC 시장 $2,000억+
- **효율성 격차**: 30-50% 에너지 절감 가능
- **쾌적성**: 실내 온습도가 생산성에 직접 영향

### 주요 과제
1. ⚡ **높은 에너지 소비**: 전체 건물 에너지의 절반
2. 🌡️ **쾌적성 불균형**: 존별 온도 편차
3. 🔧 **유지보수 비용**: 예방 정비 부족으로 고장 발생
4. 📊 **가시성 부족**: 시스템 성능 모니터링 미흡
5. 🔌 **통합 어려움**: 다양한 프로토콜 혼재

## 🎯 표준의 가치

### 핵심 목표
- ✅ **통합 관리**: 다양한 HVAC 시스템 단일 플랫폼 관리
- ✅ **에너지 절감**: 20-40% 에너지 비용 절감
- ✅ **쾌적성 향상**: 정밀한 온습도 제어
- ✅ **예지 정비**: 고장 사전 예측 및 예방
- ✅ **프로토콜 통합**: BACnet, Modbus 등 지원

### 기대 효과
- ⚡ **에너지**: 20-40% 에너지 소비 감소
- 💰 **비용**: 연간 운영 비용 30% 절감
- 🏢 **생산성**: 쾌적한 환경으로 8-11% 향상
- 🔧 **수명**: 장비 수명 20-30% 연장

## 📁 저장소 구조

```
hvac-system/
├── README.md              # 본 문서
├── install.sh             # 설치 스크립트
├── spec/
│   └── WIA-CITY-010-v1.0.md  # 상세 기술 명세
├── api/
│   └── typescript/
│       ├── src/
│       │   ├── types.ts   # TypeScript 타입 정의
│       │   └── index.ts   # SDK 구현
│       └── package.json   # 패키지 정보
└── cli/
    └── hvac-system.sh     # CLI 도구
```

## 🔧 지원 HVAC 시스템

### 1. 분리형 에어컨 (Split AC)
```
특징:
  - 실내기 + 실외기
  - 단일 공간 냉난방
  - 주거/소규모 상업용

제어:
  - 온도: 16-30°C
  - 모드: COOLING, HEATING, AUTO, DRY, FAN_ONLY
  - 팬 속도: LOW, MEDIUM, HIGH, TURBO
```

### 2. VRF 시스템
```
특징:
  - 다중 실내기 (최대 50대+)
  - 개별 존 제어
  - 동시 냉난방 가능
  - 고효율 (COP 4.0+)

장점:
  - 에너지 효율 30-40% 향상
  - 공간 절약 (단일 실외기)
  - 유연한 설계
```

### 3. 칠러 시스템
```
특징:
  - 중대형 건물용
  - 냉각수 순환 방식
  - FCU, AHU 연계

구성:
  - 칠러 (공랭/수냉)
  - 냉각탑
  - 펌프 (1차/2차 루프)
  - 분배 시스템
```

### 4. 히트펌프
```
특징:
  - 냉난방 겸용
  - 높은 COP (4.0-5.0)

타입:
  - 공기열 히트펌프 (ASHP)
  - 지열 히트펌프 (GSHP)
  - 수열 히트펌프 (WSHP)
```

### 5. 공조기 (AHU)
```
기능:
  - 외기 처리
  - 냉각/가열
  - 가습/제습
  - 필터링

제어:
  - VAV (가변 풍량)
  - CAV (정풍량)
  - 공급 온도 리셋
  - CO₂ 기반 환기
```

## 🌡️ 온도 및 습도 제어

### 온도 제어
```yaml
냉방:
  설정 범위: 18-28°C
  권장: 24-26°C (여름)
  데드밴드: 2°C

난방:
  설정 범위: 16-26°C
  권장: 20-22°C (겨울)
  데드밴드: 2°C

제어 방식:
  - PID 제어
  - 비례 제어
  - ON/OFF 제어
```

### 습도 제어
```yaml
설정 범위: 40-60% RH
최적: 50% RH

제어 설비:
  - 가습기 (증기식, 초음파식)
  - 제습기 (코일 제습 + 재열)

건강 영향:
  - 낮음 (<30%): 피부 건조, 정전기
  - 높음 (>60%): 곰팡이, 진드기
```

## 🌬️ 공기질 통합

HVAC 시스템은 WIA-ENE-027 실내 공기질 표준과 통합됩니다.

### 모니터링 파라미터
```yaml
CO₂:
  기준: < 1000 ppm
  대응: 환기량 증가

PM2.5:
  기준: < 35 μg/m³
  대응: 필터 효율 향상

VOC:
  기준: < 500 ppb
  대응: 외기 도입 증가

습도:
  기준: 40-60%
  대응: 가습/제습 작동
```

### 환기 제어
```yaml
모드:
  - 최소 환기: 20% 외기
  - 요구 제어: CO₂ 기반
  - 이코노마이저: 외기 온도 활용
  - 최대 환기: 100% 외기

전략:
  - Night Purge (야간 퍼지)
  - Pre-Occupancy Flush (재실 전 환기)
  - Demand Controlled Ventilation (DCV)
```

## ⚡ 에너지 효율

### 성능 지표

#### COP (Coefficient of Performance)
```
정의: 출력 / 입력

냉방 COP:
  일반: 2.5-3.5
  고효율: 4.0+

난방 COP:
  일반: 3.0-4.0
  고효율: 4.5+

지열 히트펌프:
  냉방 COP: 4.5-5.5
  난방 COP: 4.0-5.0
```

#### SEER (Seasonal Energy Efficiency Ratio)
```
정의: 계절별 냉방 총량 / 소비 전력 총량
단위: BTU/Wh

등급:
  기본: 13-14
  표준: 14-16
  고효율: 17-20
  초고효율: 20+
```

### 에너지 절감 전략

| 전략 | 설명 | 절감율 |
|------|------|--------|
| **Optimal Start/Stop** | 건물 열 특성 학습 | 10-20% |
| **Demand Response** | 피크 부하 감소 | 15-25% |
| **Free Cooling** | 외기 냉방 | 20-40% |
| **VFD 제어** | 팬/펌프 인버터 | 30-50% |
| **Heat Recovery** | 배기 열 회수 | 15-30% |
| **Occupancy Control** | 재실 감지 기반 | 20-35% |

## 🔌 프로토콜 통합

### BACnet
```yaml
버전: 1.24
프로파일: B-ASC (Application Specific Controller)

주요 객체:
  - ANALOG_INPUT: 온도, 습도 센서
  - ANALOG_OUTPUT: 밸브, 댐퍼 제어
  - BINARY_INPUT: 재실, 상태 감지
  - BINARY_OUTPUT: 팬, 펌프 ON/OFF

서비스:
  - ReadProperty
  - WriteProperty
  - SubscribeCOV (Change of Value)
  - GetAlarmSummary
```

### Modbus
```yaml
프로토콜: Modbus TCP/RTU
포트: 502 (TCP)

레지스터 맵:
  - Holding Registers (40001+): 설정값 (읽기/쓰기)
  - Input Registers (30001+): 측정값 (읽기 전용)
  - Coils (00001+): 디지털 출력 (읽기/쓰기)
  - Discrete Inputs (10001+): 디지털 입력 (읽기 전용)
```

### KNX
```yaml
프로토콜: KNX/IP

주요 데이터포인트:
  - DPT_Switch: ON/OFF 제어
  - DPT_Value_Temp: 온도 값
  - DPT_Scaling: 0-100% 제어
```

## 🔮 예지 정비

### 상태 모니터링
```yaml
진동 모니터링:
  - 압축기 진동 (mm/s)
  - 기준: < 7.1 mm/s
  - 경보: CRITICAL

온도 모니터링:
  - 베어링 온도
  - 권선 온도
  - 기준: < 75°C

압력 모니터링:
  - 흡입/토출 압력
  - 냉매 압력 범위
  - 이상 압력 경보

전류 모니터링:
  - 모터 전류
  - 베이스라인 대비 편차
  - 허용 범위: ±20%
```

### 고장 예측
```yaml
알고리즘:
  - 머신러닝 (LSTM)
  - 통계적 분석
  - 패턴 인식

고장 모드:
  - 압축기 고장
  - 팬 모터 베어링
  - 냉매 누설
  - 필터 막힘

예측 지표:
  - 고장 확률 (%)
  - 예상 고장 일수
  - 권장 조치
  - 긴급도
```

## 📡 API 인터페이스

### 시스템 등록
```http
POST /api/v1/hvac/systems/register
Content-Type: application/json

{
  "system": {
    "system_id": "HVAC-BLDG-A-01",
    "name": "본관 VRF 시스템",
    "system_type": "VRF",
    "building": "본관",
    "total_cooling_capacity_kw": 50.0,
    "total_heating_capacity_kw": 55.0
  }
}

Response: 201 Created
{
  "success": true,
  "data": {
    "system_id": "HVAC-BLDG-A-01",
    "dashboard_url": "https://hvac.wia.org/systems/HVAC-BLDG-A-01"
  }
}
```

### 온도 제어
```http
PUT /api/v1/hvac/systems/{system_id}/zones/{zone_id}/setpoint
Content-Type: application/json

{
  "temperature_c": 22.0,
  "mode": "AUTO",
  "fan_speed": "AUTO"
}

Response: 200 OK
{
  "success": true,
  "data": {
    "zone_id": "ZONE-301",
    "setpoint_updated": "2025-12-25T10:05:00Z"
  }
}
```

### 상태 조회
```http
GET /api/v1/hvac/systems/{system_id}/status

Response: 200 OK
{
  "success": true,
  "data": {
    "overall_status": "RUNNING",
    "zones": [
      {
        "zone_id": "ZONE-301",
        "temperature_c": 22.5,
        "setpoint_c": 22.0,
        "comfort_level": "OPTIMAL"
      }
    ],
    "energy": {
      "current_power_kw": 35.2,
      "daily_energy_kwh": 456.8,
      "cop": 4.2
    }
  }
}
```

## 🚀 빠른 시작

### 1. 설치
```bash
# 저장소 복제
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/hvac-system

# 설치
./install.sh
```

### 2. CLI 사용
```bash
# 시스템 등록
./cli/hvac-system.sh register \
  --system-id HVAC-BLDG-A-01 \
  --name "본관 VRF 시스템" \
  --type VRF \
  --cooling-kw 50 \
  --heating-kw 55

# 현재 상태 조회
./cli/hvac-system.sh status HVAC-BLDG-A-01

# 온도 설정
./cli/hvac-system.sh set-temp \
  --system-id HVAC-BLDG-A-01 \
  --zone-id ZONE-301 \
  --temp 22 \
  --mode AUTO

# 에너지 사용량
./cli/hvac-system.sh energy HVAC-BLDG-A-01

# 활성 경보
./cli/hvac-system.sh alarms HVAC-BLDG-A-01

# 필터 상태
./cli/hvac-system.sh filters HVAC-BLDG-A-01
```

### 3. TypeScript SDK
```typescript
import { HVACClient } from '@wia/city-010';

const client = new HVACClient({
  apiKey: 'your-api-key',
  endpoint: 'https://api.wia.org/city-010/v1'
});

// 시스템 등록
const system = await client.registerSystem({
  system: {
    system_id: 'HVAC-BLDG-A-01',
    name: '본관 VRF 시스템',
    system_type: 'VRF',
    building: '본관',
    location: { lat: 37.5665, lon: 126.9780 },
    total_cooling_capacity_kw: 50.0,
    total_heating_capacity_kw: 55.0,
    equipment: [],
    zones: [],
    status: 'RUNNING'
  }
});

// 현재 상태 조회
const status = await client.getStatus('HVAC-BLDG-A-01');
console.log('전체 상태:', status.overall_status);
console.log('현재 전력:', status.energy.current_power_kw, 'kW');
console.log('COP:', status.energy.cop);

// 온도 설정
await client.setTemperature('HVAC-BLDG-A-01', 'ZONE-301', {
  zone_id: 'ZONE-301',
  temperature_c: 22.0,
  mode: 'AUTO',
  fan_speed: 'AUTO'
});

// 실시간 모니터링
const ws = client.subscribeToStatus('HVAC-BLDG-A-01', (status) => {
  console.log('상태 업데이트:', status);
});

// 에너지 데이터
const energy = await client.getEnergyData(
  'HVAC-BLDG-A-01',
  '2025-12-18T00:00:00Z',
  '2025-12-25T23:59:59Z'
);
console.log('에너지 소비:', energy.energy_kwh, 'kWh');
console.log('평균 COP:', energy.cop);
```

## 📊 대시보드 예시

### 시스템 개요
```
┌─────────────────────────────────────────┐
│  HVAC 시스템: HVAC-BLDG-A-01            │
│  상태: 가동 중 ✓                        │
├─────────────────────────────────────────┤
│  전체 냉방 용량: 50.0 kW                │
│  전체 난방 용량: 55.0 kW                │
│  현재 전력: 35.2 kW                     │
│  금일 에너지: 456.8 kWh                 │
│  평균 COP: 4.2                          │
├─────────────────────────────────────────┤
│  존 상태:                               │
│  ✓ 3F 회의실 A: 22.5°C (설정 22.0°C)   │
│  ✓ 3F 회의실 B: 23.0°C (설정 23.0°C)   │
│  ✓ 3F 사무실: 22.8°C (설정 22.5°C)     │
└─────────────────────────────────────────┘
```

### 에너지 분석
```
┌─────────────────────────────────────────┐
│  에너지 분석 (주간)                     │
├─────────────────────────────────────────┤
│  총 소비: 3,200 kWh                     │
│  냉방: 2,100 kWh (65%)                  │
│  난방: 800 kWh (25%)                    │
│  팬: 300 kWh (10%)                      │
├─────────────────────────────────────────┤
│  평균 COP: 4.1                          │
│  피크 전력: 48.5 kW                     │
│  전력 비용: ₩520,000                    │
├─────────────────────────────────────────┤
│  절감 기회:                             │
│  • 야간 세트백 적용: 15% 절감 가능      │
│  • 이코노마이저 활용: 20% 절감 가능     │
│  • 재실 기반 제어: 10% 절감 가능        │
└─────────────────────────────────────────┘
```

## 💡 사용 사례

### 오피스 빌딩
- **과제**: 층별 온도 편차, 높은 에너지 비용
- **솔루션**: VRF 시스템 + 존 제어 + 스케줄링
- **결과**: 에너지 30% 절감, 쾌적성 40% 향상

### 병원
- **과제**: 24시간 운영, 엄격한 온습도 기준
- **솔루션**: 칠러 + AHU + 정밀 제어
- **결과**: 기준 100% 준수, 유지보수 비용 25% 감소

### 데이터센터
- **과제**: 고밀도 냉각, 에너지 효율
- **솔루션**: 프리 쿨링 + 열통로 관리 + 실시간 모니터링
- **결과**: PUE 1.8 → 1.3, 냉방 비용 40% 절감

### 쇼핑몰
- **과제**: 넓은 공간, 가변 부하
- **솔루션**: 멀티 칠러 + VAV + CO₂ 기반 환기
- **결과**: 에너지 35% 절감, 재실자 만족도 증가

## 📈 성과 지표

### 에너지 효율
- **COP**: 목표 > 4.0
- **에너지 절감**: 20-40%
- **피크 전력**: 15-25% 감소
- **전력 비용**: 연간 30% 절감

### 쾌적성
- **온도 만족도**: > 90%
- **온도 편차**: ±1°C 이내
- **습도 유지율**: 40-60% RH 유지 > 95%

### 유지보수
- **예방 정비**: 계획대로 100% 수행
- **고장 예측 정확도**: > 80%
- **장비 수명**: 20-30% 연장
- **긴급 수리**: 60% 감소

## 🤝 기여

이 표준은 HVAC 산업 커뮤니티의 기여를 환영합니다:

- 기술 피드백 및 개선 제안
- 사례 연구 및 구현 사례
- 추가 언어 번역
- 교육 자료 및 튜토리얼
- 프로토콜 통합 지원

## 📜 라이선스

© 2025 SmileStory Inc. / WIA - World Certification Industry Association

이 표준은 MIT 라이선스 하에 배포됩니다.

## 🔗 관련 표준

- **WIA-ENE-027**: Indoor Air Quality (실내 공기질)
- **WIA-ENE-001**: Energy Monitoring (에너지 모니터링)
- **WIA-CITY-001**: Smart Building Platform (스마트 빌딩)
- **WIA-INTENT**: Intent Expression Standard

## 📞 문의 및 지원

### 공식 채널
- **웹사이트**: https://wia-official.org
- **이메일**: standards@wia-official.org
- **GitHub**: https://github.com/WIA-Official/wia-standards

### 커뮤니티
- 기술 포럼: 구현 관련 질문
- 정기 웨비나 및 교육 세션
- 연례 WIA 표준 컨퍼런스
- 지역별 워킹 그룹

---

## 홍익인간 (弘益人間) (홍익인간)

**널리 인간을 이롭게 하라**

WIA-CITY-010 표준은 홍익인간 (弘益人間)의 철학을 통해 모든 건물 사용자에게 쾌적한 환경을 제공하면서, 에너지 효율과 지속가능성을 동시에 추구합니다. 스마트한 HVAC 제어를 통해 더 나은 미래를 만들어갑니다.

**효율적인 냉난방, 쾌적한 환경, 지속가능한 미래**

---

*문서 버전: 1.0.0*
*최종 업데이트: 2025-12-25*
*상태: Active*
