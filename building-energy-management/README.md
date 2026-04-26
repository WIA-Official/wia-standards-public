# WIA-CITY-011: 빌딩 에너지 관리 표준 ⚡

> **홍익인간 (弘益人間) (홍익인간) - 널리 인간을 이롭게 하라**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/Version-1.0-blue.svg)](https://github.com/WIA-Official/wia-standards)
[![Standard](https://img.shields.io/badge/Standard-WIA--CITY--011-sky.svg)](https://wia.org/standards/city-011)

## 개요

WIA-CITY-011 빌딩 에너지 관리 표준은 건물의 에너지 소비를 체계적으로 모니터링, 분석, 최적화하기 위한 통합 표준입니다. BEMS (Building Energy Management System)의 데이터 형식, 통신 프로토콜, 제어 로직, 성능 지표를 정의하여 에너지 효율 향상과 탄소 배출 감축을 실현합니다.

### 주요 기능

- 🔋 **실시간 모니터링**: 전력, 가스, 수도, 냉난방 에너지 사용량 15분 단위 계측
- 🤖 **AI 기반 최적화**: 머신러닝을 활용한 에너지 사용 패턴 예측 및 최적화
- 📉 **피크 절삭**: ESS와 부하 제어를 통한 최대 전력 수요 15-30% 감축
- ⚡ **수요 반응**: 전력망 상황에 따른 자동 부하 조절 및 보상 수익
- ☀️ **재생 에너지 통합**: 태양광 PV, ESS, 지열 히트펌프 통합 관리
- 🌍 **탄소 발자국**: 실시간 CO₂ 배출량 산정 및 넷제로 달성 경로 제시
- 📊 **에너지 벤치마킹**: 유사 건물 대비 에너지 효율 비교 및 개선 권고
- 🏆 **인증 지원**: LEED, BREEAM, G-SEED 등 녹색 건축 인증 데이터 자동 생성

## 빠른 시작

### 1. 설치

```bash
# 저장소 클론
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/building-energy-management

# 의존성 설치
./install.sh
```

### 2. TypeScript SDK 사용

```typescript
import { BuildingEnergySDK } from '@wia/city-011';

const sdk = new BuildingEnergySDK({
  apiKey: 'your-api-key',
  endpoint: 'https://api.wia.org/city-011/v1'
});

// 실시간 대시보드 조회
const dashboard = await sdk.getRealtimeDashboard('BLD-SEOUL-001');
console.log('현재 전력 수요:', dashboard.data.energy.currentDemand, 'kW');
console.log('금일 소비량:', dashboard.data.energy.todayConsumption, 'kWh');
console.log('금일 CO₂:', dashboard.data.carbon.todayEmissions, 'kg');

// HVAC 설정 온도 제어
await sdk.setHVACSetpoint({
  buildingId: 'BLD-SEOUL-001',
  systemId: 'HVAC-AHU-001',
  mode: 'cooling',
  setpoint: 26.0,
  duration: 3600  // 1시간
});

// 태양광 발전 현황
const solarData = await sdk.getSolarPVData('PV-ROOF-001');
console.log('현재 발전량:', solarData.data.generation.current, 'kW');
console.log('금일 발전량:', solarData.data.generation.daily, 'kWh');
console.log('성능비 (PR):', solarData.data.performanceRatio, '%');

// ESS 제어
await sdk.controlESS({
  buildingId: 'BLD-SEOUL-001',
  essId: 'ESS-001',
  mode: 'discharge',
  power: 200  // kW
});

// 탄소 배출 리포트
const carbon = await sdk.getCarbonReport('BLD-SEOUL-001', 'monthly', '2025-12-01');
console.log('월간 총 배출:', carbon.data.emissions.total, 'kg CO₂');
console.log('재생에너지 상쇄:', carbon.data.offsetByRenewable.total, 'kg CO₂');
console.log('순 배출:', carbon.data.netEmissions, 'kg CO₂');
```

### 3. CLI 도구 사용

```bash
# 실시간 대시보드
./cli/building-energy-management.sh dashboard \
  --building-id BLD-SEOUL-001

# 에너지 소비 현황
./cli/building-energy-management.sh energy \
  --building-id BLD-SEOUL-001 \
  --type electricity \
  --period today

# HVAC 시스템 상태
./cli/building-energy-management.sh hvac-status \
  --system-id HVAC-AHU-001

# HVAC 설정 온도 변경
./cli/building-energy-management.sh hvac-control \
  --building-id BLD-SEOUL-001 \
  --system-id HVAC-AHU-001 \
  --setpoint 24.0

# 태양광 발전 현황
./cli/building-energy-management.sh solar \
  --pv-id PV-ROOF-001

# ESS 상태
./cli/building-energy-management.sh ess-status \
  --ess-id ESS-001

# 탄소 배출
./cli/building-energy-management.sh carbon \
  --building-id BLD-SEOUL-001 \
  --period monthly

# 에너지 벤치마킹
./cli/building-energy-management.sh benchmark \
  --building-id BLD-SEOUL-001

# 활성 알람
./cli/building-energy-management.sh alerts \
  --building-id BLD-SEOUL-001
```

### 4. 상세 사양 확인

- **스펙 문서**: [`spec/WIA-CITY-011-v1.0.md`](spec/WIA-CITY-011-v1.0.md)

## 저장소 구조

```
building-energy-management/
├── README.md              # 본 문서
├── install.sh             # 설치 스크립트
├── spec/
│   └── WIA-CITY-011-v1.0.md  # 상세 스펙 (한국어)
├── api/
│   └── typescript/
│       ├── src/
│       │   ├── types.ts    # 타입 정의
│       │   └── index.ts    # SDK 구현
│       └── package.json    # npm 패키지 설정
└── cli/
    └── building-energy-management.sh  # CLI 도구
```

## 기술 범위

### BEMS 구성 요소

- **데이터 수집**: 센서 데이터 실시간 수집 (BACnet, Modbus, MQTT)
- **에너지 분석**: 사용 패턴 분석, 이상 탐지, 예측 모델
- **최적화 제어**: 스케줄 제어, 최적 기동/정지, 외기 냉방
- **사용자 인터페이스**: 대시보드, 알람, 리포트

### 에너지 계측

| 에너지원 | 계측 항목 | 정확도 | 주기 |
|---------|----------|--------|------|
| 전력 | 유효전력, 무효전력, 역률 | ±1% | 15분 |
| 가스 | 순간 유량, 누적 사용량 | ±2% | 15분 |
| 지역난방 | 공급/환수 온도, 유량 | ±3% | 15분 |
| 수도 | 순간 유량, 누적 사용량 | ±2% | 15분 |

### 부하 관리

**피크 절삭 (Peak Shaving)**:
- 목표: 최대 전력 수요 15-30% 감축
- 방법: ESS 방전, 비필수 부하 차단, 냉난방 설정 온도 조정
- 효과: 전력 요금 절감, 전력망 안정화 기여

**수요 반응 (Demand Response)**:
- OpenADR 2.0b 프로토콜 지원
- 한전 DR 시장 연동
- 대응 수준: Level 1 (5-10%), Level 2 (10-20%), Level 3 (20-30%)
- 보상 메커니즘: 감축량 × 시간 × 단가

### 재생 에너지 통합

**태양광 발전 (Solar PV)**:
```typescript
interface SolarPVData {
  generation: {
    current: number;          // kW
    daily: number;            // kWh
    monthly: number;          // kWh
    annual: number;           // kWh
  };
  irradiance: number;         // W/m²
  performanceRatio: number;   // % (PR)
}
```

**에너지 저장 장치 (ESS)**:
```typescript
interface ESSData {
  mode: 'idle' | 'charging' | 'discharging';
  stateOfCharge: number;      // % (0-100)
  power: number;              // kW
  health: {
    stateOfHealth: number;    // % (capacity retention)
    remainingCycleLife: number;
  };
}
```

**지열 히트펌프 (Geothermal)**:
- COP 난방: 4.5 이상
- COP 냉방: 5.5 이상
- 지중 온도 모니터링

### 탄소 배출 관리

**배출량 산정**:
```
전력: kWh × 0.4594 kg CO₂/kWh (한국 배출 계수)
가스: m³ × 2.23 kg CO₂/m³
지역난방: MJ × 0.067 kg CO₂/MJ
```

**넷제로 달성 경로**:
1. Phase 1 (2025-2027): 에너지 효율 개선 → 30% 감축
2. Phase 2 (2027-2030): 재생 에너지 확대 → 60% 감축
3. Phase 3 (2030-2035): 100% 재생 에너지 → 넷제로 달성

### 에너지 벤치마킹

**비교 지표**:

| 건물 유형 | EUI 중간값 | 상위 25% | 하위 25% |
|----------|-----------|---------|---------|
| 오피스 | 150 kWh/m²/년 | 100 | 200 |
| 쇼핑몰 | 250 kWh/m²/년 | 180 | 350 |
| 호텔 | 300 kWh/m²/년 | 220 | 400 |
| 병원 | 400 kWh/m²/년 | 300 | 550 |
| 학교 | 120 kWh/m²/년 | 80 | 180 |

**벤치마킹 결과**:
- 귀사 EUI 순위
- 개선 잠재력
- 에너지 절감 권고사항
- 투자 회수 기간

### 녹색 건축 인증

**LEED (Leadership in Energy and Environmental Design)**:
- EA Credit 1: Optimize Energy Performance (최대 18점)
- EA Credit 3: Advanced Energy Metering (1점)
- EA Credit 6: Renewable Energy Production (최대 3점)

**BREEAM (Building Research Establishment Environmental Assessment Method)**:
- Ene 01: Energy efficiency (15%)
- Ene 02: Energy monitoring (2%)
- Ene 04: Low carbon design (3%)

**데이터 제공**:
- 12개월 연속 에너지 소비 데이터
- 15분 간격 미터 데이터
- 에너지 모델링 결과
- 재생 에너지 발전량

## 데이터 포맷 표준

### 빌딩 정보

```json
{
  "buildingId": "BLD-SEOUL-001",
  "name": "그린타워",
  "type": "commercial-office",
  "location": {
    "address": "서울특별시 강남구 테헤란로 123",
    "coordinates": {
      "latitude": 37.5665,
      "longitude": 126.9780
    },
    "timezone": "Asia/Seoul",
    "climateZone": "4A"
  },
  "specifications": {
    "totalFloorArea": 50000,
    "floors": {
      "aboveGround": 25,
      "underground": 5
    },
    "occupancy": 2000,
    "certifications": ["LEED Gold", "G-SEED 1등급"]
  }
}
```

### 에너지 계측 데이터

```json
{
  "meterId": "MTR-ELEC-001",
  "buildingId": "BLD-SEOUL-001",
  "meterType": "electricity",
  "timestamp": "2025-12-25T14:30:00+09:00",
  "interval": "15min",
  "readings": {
    "activePower": 850.5,
    "energy": {
      "cumulative": 1250000.5,
      "interval": 212.5
    }
  },
  "breakdown": {
    "hvac": 450.2,
    "lighting": 180.5,
    "plugLoad": 150.8
  }
}
```

## API 명세

### 인증

```bash
# API Key 발급
POST /api/v1/auth/register
{
  "organizationName": "그린빌딩 주식회사",
  "email": "admin@greenbuilding.com",
  "buildingId": "BLD-SEOUL-001"
}

# 헤더
Authorization: Bearer {apiKey}
Content-Type: application/json
X-WIA-Standard: CITY-011
X-WIA-Version: 1.0.0
```

### 주요 엔드포인트

| 엔드포인트 | 메서드 | 설명 |
|-----------|--------|------|
| `/api/v1/buildings/{id}` | GET | 빌딩 정보 조회 |
| `/api/v1/meters/{id}/latest` | GET | 최신 계측 데이터 |
| `/api/v1/energy/consumption` | GET | 에너지 소비 조회 |
| `/api/v1/hvac/{id}/status` | GET | HVAC 상태 조회 |
| `/api/v1/hvac/setpoint` | POST | HVAC 설정 온도 변경 |
| `/api/v1/renewable/solar/{id}` | GET | 태양광 데이터 |
| `/api/v1/renewable/ess/{id}` | GET | ESS 데이터 |
| `/api/v1/renewable/ess/control` | POST | ESS 제어 |
| `/api/v1/carbon/{id}/report` | GET | 탄소 배출 리포트 |
| `/api/v1/benchmarking/{id}/compare` | GET | 벤치마킹 비교 |
| `/api/v1/dashboard/{id}/realtime` | GET | 실시간 대시보드 |

## 적용 효과

### 경제적 효과

- **에너지 절감**: 기존 대비 20-40% 에너지 소비 감축
- **비용 절감**: 연간 운영 비용 15-30% 절감
- **자산 가치**: 건물 가치 5-15% 상승
- **DR 보상**: 연간 500만-2,000만 원 추가 수익

### 환경적 효과

- **탄소 감축**: 건물 운영 탄소 배출 30-50% 감축
- **재생 에너지**: 태양광 발전으로 10-30% 자가 소비
- **대기 질**: 실내 공기질 개선 (CO₂, PM2.5 감소)

### 사회적 효과

- **쾌적성**: 실내 온도, 습도, 조도 최적화
- **생산성**: 쾌적한 환경으로 근무자 생산성 5-10% 향상
- **전력망**: 피크 절삭으로 전력망 안정화 기여

## 용례 (Use Cases)

### 오피스 빌딩

**건물 정보**:
- 연면적: 50,000m²
- 층수: 지상 25층, 지하 5층
- 재실자: 2,000명
- 연간 에너지 비용: 10억 원

**적용 효과**:
- 에너지 소비 25% 감축
- 연간 2.5억 원 절감
- 탄소 배출 500톤 감축
- LEED Gold 인증 취득

### 쇼핑몰

**건물 정보**:
- 연면적: 100,000m²
- 영업 시간: 10:00-22:00
- 연간 방문객: 1,000만 명

**적용 효과**:
- 냉난방 에너지 30% 감축
- 조명 에너지 40% 감축 (LED + 주광 센서)
- 피크 수요 20% 감축 (ESS 활용)
- 연간 5억 원 절감

## 성능 기준

| 지표 | 목표 |
|------|------|
| 데이터 수집 주기 | ≤ 15분 |
| 데이터 정확도 | ≥ 98% |
| 시스템 가용성 | ≥ 99.5% |
| 제어 응답 시간 | ≤ 5초 |
| 에너지 절감 | ≥ 20% (기존 대비) |
| 피크 수요 감축 | ≥ 15% |

## 보안 요구사항

- **네트워크 보안**: OT/IT 네트워크 분리, 방화벽, VPN
- **데이터 보안**: TLS 1.3 암호화, AES-256 저장
- **인증/권한**: API Key + OAuth 2.0, RBAC
- **감사 로그**: 모든 제어 명령 및 접근 기록
- **백업/복구**: 일일 자동 백업, RPO 24시간, RTO 4시간

## 참조 표준

- ISO 50001: 에너지 경영 시스템
- ISO 52000 시리즈: 건물 에너지 성능
- ASHRAE 90.1: 에너지 표준
- ASHRAE 135: BACnet 통신 프로토콜
- IEC 61850: 전력 시스템 통신
- OpenADR 2.0b: 수요 반응 통신
- IEEE 1547: 분산 에너지 자원 연계

## 기여 방법

WIA-CITY-011 표준 개선에 기여하실 수 있습니다:

1. **기술 피드백**: 표준 개선 제안
2. **사례 연구**: 빌딩 에너지 관리 성공 사례 공유
3. **번역**: 다양한 언어로 문서 번역
4. **도구 개발**: 오픈소스 분석 도구 개발
5. **교육**: 교육 자료 및 튜토리얼 제작

## 라이선스

WIA-CITY-011 빌딩 에너지 관리 표준은 MIT License로 공개됩니다.

자유롭게:
- **공유** - 복사 및 재배포
- **수정** - 리믹스, 변형, 2차 저작물 제작

조건:
- **저작자 표시** - WIA에 적절한 크레딧 제공

## 문의

### 표준 개발

- **기관**: WIA (World Certification Industry Association)
- **발행**: SmileStory Inc.
- **이메일**: standards@wia.org
- **홈페이지**: https://wia.org/standards/city-011

### 기술 지원

- **이메일**: support@wia.org
- **문서**: [기술 스펙](spec/WIA-CITY-011-v1.0.md)
- **커뮤니티**: https://forum.wia.org/building-energy

### 인증 문의

- **이메일**: certification@wia.org
- **비용**: [인증 수수료 안내](https://wia.org/certification/fees)
- **신청**: https://wia.org/certification/apply

## 버전 이력

| 버전 | 날짜 | 변경 내용 |
|------|------|----------|
| 1.0.0 | 2025-12-25 | 초판 발행 |

---

**발행 기관**: WIA (World Certification Industry Association)
**라이선스**: MIT License
**문의**: standards@wia.org
**홈페이지**: https://wia.org/standards/city-011

**홍익인간 (弘益人間) (홍익인간) - 널리 인간을 이롭게 하라**
