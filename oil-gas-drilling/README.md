# WIA-ENE-036: 석유/가스 시추 표준 🛢️

> **홍익인간 (弘益人間) (홍익인간)** - 널리 인간을 이롭게 하라

## 개요

WIA-ENE-036 석유/가스 시추 표준은 전 세계 석유 및 천연가스 시추 작업의 안전하고 환경 친화적인 운영을 위한 포괄적인 프레임워크를 제공합니다. 본 표준은 시추 운영, 저류층 관리, 생산율 모니터링, 환경 영향 완화, 메탄 배출 관리, 유출 방지, 폐쇄 절차에 대한 통합 솔루션을 제공합니다.

## 📁 저장소 구조

```
oil-gas-drilling/
├── spec/
│   └── WIA-ENE-036-v1.0.md      # 상세 표준 문서 (한글)
├── api/
│   └── typescript/
│       ├── src/
│       │   ├── types.ts          # TypeScript 타입 정의
│       │   └── index.ts          # SDK 구현
│       └── package.json
├── cli/
│   └── oil-gas-drilling.sh       # CLI 도구
├── README.md                     # 본 문서
└── install.sh                    # 설치 스크립트
```

## 🎯 표준 적용 범위

### 핵심 영역

1. **유정 분류**
   - 탐사정 (Exploration Wells)
   - 생산정 (Production Wells - 수직, 경사, 수평, 다중분지)
   - 주입정 (Injection Wells)

2. **시추 작업**
   - 전통적 시추 (Rotary Drilling)
   - 방향성 시추 (Directional Drilling)
   - 수압 파쇄 (Hydraulic Fracturing)

3. **저류층 특성**
   - 사암, 탄산염암, 셰일, 타이트 샌드
   - 원유 및 천연가스 특성
   - 저류층 압력 및 온도

4. **생산 모니터링**
   - 일일/월간/연간 생산율
   - 정두 조건 (Wellhead Conditions)
   - 누적 생산 및 예측

5. **환경 관리**
   - 메탄 배출 감지 및 저감
   - 플레어링 관리
   - 유출 방지 및 대응
   - 폐수 처리

6. **안전 프로토콜**
   - Well Control (유정 제어)
   - Blowout Preventer (BOP) 시험
   - H₂S 안전 관리

7. **폐쇄 및 복원**
   - Plug & Abandonment
   - 부지 복원
   - 사후 모니터링

## 🚀 빠른 시작

### TypeScript SDK 설치

```bash
npm install @wia/ene-036
```

### 기본 사용 예시

```typescript
import { OilGasDrillingClient } from '@wia/ene-036';

const client = new OilGasDrillingClient({
  apiKey: process.env.WIA_API_KEY,
  operatorId: 'OP-EXXON-US-001'
});

// 새 유정 생성
const response = await client.createWell({
  wellType: 'PROD-3',
  wellName: 'Permian Basin #42H',
  operatorId: 'OP-EXXON-US-001',
  location: {
    address: {
      state: 'Texas',
      country: 'US'
    },
    coordinates: {
      latitude: 31.8457,
      longitude: -102.0779,
      datum: 'WGS84'
    },
    surfaceElevation: {
      value: 870,
      unit: 'meters'
    }
  },
  wellboreGeometry: {
    totalDepth: {
      measuredDepth: 3850,
      trueVerticalDepth: 2950,
      unit: 'meters'
    },
    casingProgram: []
  }
});

console.log('Well ID:', response.wellId);

// 생산 데이터 제출
await client.submitProduction({
  wellId: response.wellId,
  date: '2025-12-25',
  production: {
    oil: {
      net: {
        value: 450,
        unit: 'bbl/day'
      }
    },
    gas: {
      gross: {
        value: 850000,
        unit: 'scf/day'
      }
    },
    water: {
      produced: {
        value: 120,
        unit: 'bbl/day'
      }
    }
  }
});

// 배출 데이터 제출
await client.submitEnvironmentalData({
  wellId: response.wellId,
  reportingPeriod: {
    startDate: '2025-12-01',
    endDate: '2025-12-31'
  },
  emissionsData: {
    methaneEmissions: {
      venting: {
        value: 2.5,
        unit: 'tonnes_CO2e/year'
      },
      flaring: {
        value: 8.2,
        unit: 'tonnes_CO2e/year'
      },
      fugitiveEmissions: {
        value: 1.8,
        unit: 'tonnes_CO2e/year'
      }
    },
    flaringData: {
      volumeFlared: {
        value: 45000,
        unit: 'scf/day'
      },
      flaringIntensity: {
        value: 5.3,
        unit: 'percent_of_gas_production'
      }
    }
  }
});

// 생산 예측
const forecast = await client.getProductionForecast({
  wellId: response.wellId,
  forecastPeriod: 12,
  method: 'decline_curve'
});

console.log('EUR:', forecast.estimatedUltimateRecovery);
```

### CLI 도구 사용

```bash
# 환경 변수 설정
export WIA_ENE036_API_KEY="your-api-key"
export WIA_ENE036_OPERATOR_ID="OP-EXXON-US-001"

# 새 유정 생성 (대화형)
./cli/oil-gas-drilling.sh create-well

# 유정 조회
./cli/oil-gas-drilling.sh get-well WIA-OG-2025-TX-001234

# 유정 목록
./cli/oil-gas-drilling.sh list-wells

# 생산 데이터 제출
./cli/oil-gas-drilling.sh submit-production WIA-OG-2025-TX-001234

# 생산 기록 조회 (최근 30일)
./cli/oil-gas-drilling.sh get-production WIA-OG-2025-TX-001234 30

# 생산 예측
./cli/oil-gas-drilling.sh forecast WIA-OG-2025-TX-001234 12

# 배출 데이터 보고
./cli/oil-gas-drilling.sh report-emissions WIA-OG-2025-TX-001234

# 유출 사고 보고
./cli/oil-gas-drilling.sh report-spill WIA-OG-2025-TX-001234

# BOP 시험 제출
./cli/oil-gas-drilling.sh submit-bop-test WIA-OG-2025-TX-001234

# 운영자 대시보드
./cli/oil-gas-drilling.sh dashboard
```

## 📊 데이터 구조

### 유정 정보

```typescript
interface Well {
  wellId: string;
  wellType: 'PROD-1' | 'PROD-2' | 'PROD-3' | 'EXP-1' | 'INJ-1';
  wellName: string;
  status: 'drilling' | 'producing' | 'shut_in' | 'abandoned';
  operator: Operator;
  location: WellLocation;
  wellboreGeometry: WellboreGeometry;
  reservoir?: Reservoir;
  production?: ProductionData;
  environmental?: EnvironmentalMonitoring;
  safety?: SafetySystems;
  regulatory?: RegulatoryInfo;
}
```

### 생산 데이터

```typescript
interface DailyProduction {
  oil?: {
    gross?: Volume;
    net: Volume;
    bsw?: { value: number; unit: 'percent' };
  };
  gas?: {
    gross: Volume;
    sales?: Volume;
    flared?: Volume;
    vented?: Volume;
  };
  water?: {
    produced: Volume;
    disposed?: Volume;
  };
}
```

### 환경 모니터링

```typescript
interface EnvironmentalMonitoring {
  waterUsage: WaterUsage;
  emissionsMonitoring: {
    methaneEmissions: MethaneEmissions;
    flaringData?: FlaringData;
    co2Emissions?: Measurement;
  };
  spillPrevention: SpillPrevention;
  wastewaterManagement: WastewaterManagement;
}
```

## 🔬 주요 기능

### 1. 유정 관리

- 탐사정, 생산정, 주입정 등록 및 추적
- 실시간 상태 업데이트
- 시추 작업 기록
- 완결 설계 저장

### 2. 생산 모니터링

- 일일/월간/연간 생산 데이터
- 원유, 가스, 물 생산율
- 정두 조건 모니터링
- 누적 생산 추적

### 3. 예측 분석

- 감퇴 곡선 분석 (Decline Curve Analysis)
- 생산 예측 (ML 기반)
- 궁극 회수량 (EUR) 추정
- 저류층 압력 트렌드

### 4. 환경 관리

- **메탄 배출 감지**: OGI, 레이저 흡수, LDAR
- **플레어링 관리**: 연속 모니터링, 감축 목표
- **유출 방지**: 2차 격납, 비상 대응
- **폐수 관리**: 주입정 처분, 재사용

### 5. 안전 시스템

- BOP 시험 기록 및 추적
- H₂S 검출 및 알람
- 안전 사고 보고
- 비상 종료 시스템 (ESD)

### 6. 규제 준수

- 허가증 관리 및 만료 알림
- 생산/배출 보고 자동화
- 규제 기관 검사 기록
- 위반 사항 추적

## 🛡️ 환경 및 안전 기준

### 메탄 배출 목표

| 연도 | 목표 |
|------|------|
| 2025 | 광학 가스 이미징 (OGI) 필수화 |
| 2027 | 연속 모니터링 시스템 도입 |
| 2030 | 플레어링 90% 감축 |
| 2035 | 플레어링 제로 달성 |
| 2040 | 순배출 제로 (Net-Zero Emissions) |

### 안전 프로토콜

**Well Control (유정 제어)**
- Kick Detection: 순환 유량, 피트 용적, 압력 모니터링
- BOP 활성화 절차
- Well Kill 절차

**H₂S 안전 관리**
- 안전 (Safe): < 10 ppm
- 경계 (Caution): 10-20 ppm
- 위험 (Danger): 20-100 ppm
- 즉시 대피 (IDLH): > 100 ppm

## 🌍 국제 규제 준수

| 국가/지역 | 규제 기관 | 주요 규제 |
|----------|----------|----------|
| 🇺🇸 미국 | EPA, State Commissions | Clean Air Act, Clean Water Act |
| 🇺🇸 텍사스 | Texas Railroad Commission | 생산 보고, 우물 폐쇄 |
| 🇨🇦 캐나다 | Alberta Energy Regulator | 시추 및 생산 규제 |
| 🇳🇴 노르웨이 | Norwegian Petroleum Directorate | 해양 안전 및 환경 |

## 📖 참조 문서

### 국제 표준

- **API Standards**
  - API RP 500: Classification of Locations
  - API Spec 6A: Wellhead Equipment
  - API RP 53: Blowout Prevention

- **ISO Standards**
  - ISO 13628: Subsea Production Systems
  - ISO 16530: Well Integrity

### 환경 프로토콜

- **OGCI**: 메탄 배출 감축 지침
- **World Bank Zero Routine Flaring**: 플레어링 제로 이니셔티브
- **EPA Methane Challenge**: 메탄 배출 저감 프로그램

## 🔧 설치 및 설정

### 자동 설치

```bash
chmod +x install.sh
./install.sh
```

### 수동 설치

```bash
# TypeScript SDK
cd api/typescript
npm install
npm run build

# CLI 도구
chmod +x cli/oil-gas-drilling.sh
sudo ln -s $(pwd)/cli/oil-gas-drilling.sh /usr/local/bin/oil-gas-drilling

# 환경 변수 설정
echo 'export WIA_ENE036_API_KEY="your-api-key"' >> ~/.bashrc
echo 'export WIA_ENE036_OPERATOR_ID="your-operator-id"' >> ~/.bashrc
source ~/.bashrc
```

## 📞 연락처 및 지원

### WIA 표준 사무국

- **웹사이트**: https://wia.org/standards/ene-036
- **이메일**: oil-gas-drilling@wia.org
- **GitHub**: https://github.com/WIA-Official/wia-standards

### 기술 지원

- **이메일**: tech-support@wia.org
- **포럼**: https://forum.wia.org/ene-036

### 긴급 연락

- **유출 대응**: +1-XXX-SPILL-RESPONSE
- **안전 비상**: +1-XXX-SAFETY-EMERGENCY

## 📜 라이선스 및 저작권

© 2025 WIA (World Certification Industry Association)

본 표준은 **홍익인간 (弘益人間) (홍익인간)** — 널리 인간을 이롭게 하라는 철학 아래 배포됩니다.

**라이선스**: Creative Commons BY 4.0
- ✅ 자유로운 사용 및 적용
- ✅ 출처 표시 필수
- ✅ 상업적 사용 허용
- ✅ 2차 저작물 작성 허용

## 🙏 감사의 말

본 표준은 다음의 협력을 통해 개발되었습니다:
- 국제 석유 및 가스 산업 전문가
- 환경 보호 단체
- 규제 기관
- 연구 기관
- 운영 회사

모든 기여자들이 **홍익인간 (弘益人間) (홍익인간)**의 정신으로 책임 있는 에너지 개발과 환경 보호를 위해 노력해 주신 것에 감사드립니다.

---

## 버전 이력

| 버전 | 날짜 | 변경 내용 |
|------|------|-----------|
| 1.0.0 | 2025-12-25 | 초기 버전 발행 |

---

**홍익인간 (弘益人間) (홍익인간) - 널리 인간을 이롭게 하라**

*책임 있는 자원 개발을 통해 에너지 안보와 환경 보호를 동시에 실현합니다* 🛢️
