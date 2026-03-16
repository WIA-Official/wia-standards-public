# WIA-ENE-031: 생태계 모니터링 표준 v1.0 🌲

> **弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

**표준 ID:** WIA-ENE-031
**버전:** 1.0.0
**발행일:** 2025-12-25
**상태:** 활성
**카테고리:** 에너지/환경 (ENE)

---

## 목차

1. [개요](#1-개요)
2. [적용 범위](#2-적용-범위)
3. [용어 정의](#3-용어-정의)
4. [생태계 분류 체계](#4-생태계-분류-체계)
5. [데이터 모델](#5-데이터-모델)
6. [모니터링 프로토콜](#6-모니터링-프로토콜)
7. [건강도 지표](#7-건강도-지표)
8. [탄소 흡수 및 저장](#8-탄소-흡수-및-저장)
9. [물 순환 모니터링](#9-물-순환-모니터링)
10. [영양분 순환](#10-영양분-순환)
11. [위성 영상 통합](#11-위성-영상-통합)
12. [IoT 센서 네트워크](#12-iot-센서-네트워크)
13. [복원 프로젝트 관리](#13-복원-프로젝트-관리)
14. [보호구역 관리](#14-보호구역-관리)
15. [성과 지표 (KPI)](#15-성과-지표-kpi)
16. [통합 및 상호운용성](#16-통합-및-상호운용성)
17. [보안 및 개인정보보호](#17-보안-및-개인정보보호)
18. [인증 요구사항](#18-인증-요구사항)

---

## 1. 개요

### 1.1 목적

WIA-ENE-031 생태계 모니터링 표준은 다양한 생태계의 건강 상태, 생물다양성, 생태계 서비스를 실시간으로 모니터링하고 평가하기 위한 국제 표준입니다. 본 표준은 생태계 보전, 기후 변화 대응, 지속가능한 자연 관리를 목표로 합니다.

### 1.2 핵심 원칙

- **통합 모니터링 (Integrated Monitoring)**: 생태계 구성요소 전체 관찰
- **과학적 정확성 (Scientific Rigor)**: 검증된 방법론 기반 데이터 수집
- **실시간성 (Real-time)**: IoT 센서 및 위성을 통한 즉각적 데이터 수집
- **개방성 (Openness)**: 데이터 공개 및 투명한 공유
- **예방 원칙 (Precautionary Principle)**: 생태계 훼손 예방 우선
- **참여 (Participation)**: 지역 공동체 및 이해관계자 참여
- **지속가능성 (Sustainability)**: 생태계 서비스의 장기적 유지

### 1.3 적용 대상

- 환경부 및 산림청 등 정부 기관
- 국립공원 및 자연보호구역 관리기관
- 생태 복원 프로젝트 운영 기관
- 환경 연구소 및 대학
- 탄소배출권 거래소 및 탄소 크레딧 검증기관
- 스마트 농업 및 임업 기업
- 환경 NGO 및 시민과학 단체

---

## 2. 적용 범위

### 2.1 생태계 유형

본 표준은 다음 생태계 유형에 적용됩니다:

- **산림 생태계 (Forest)**: 온대림, 열대림, 한대림, 맹그로브
- **습지 생태계 (Wetland)**: 늪, 습원, 갯벌, 강 하구
- **해양 생태계 (Marine)**: 연안, 심해, 산호초, 해초대
- **초원 생태계 (Grassland)**: 사바나, 프레리, 스텝, 고산 초원
- **도시 생태계 (Urban)**: 도시 공원, 녹지대, 옥상 정원, 도시 숲
- **농업 생태계 (Agricultural)**: 유기농장, 혼농임업, 논밭 생태계
- **습윤 생태계 (Riparian)**: 하천변, 호수변 식생 지대
- **극지 생태계 (Polar)**: 툰드라, 빙하 주변 생태계

### 2.2 모니터링 대상

- 생물 다양성 (종 구성, 개체 수, 멸종위기종)
- 식생 건강도 (NDVI, LAI, 광합성 활성)
- 토양 상태 (유기물 함량, pH, 수분, 미생물 활성)
- 수질 및 수량 (영양염류, 용존산소, 유량)
- 대기질 (CO2 농도, 에어로졸, VOCs)
- 탄소 저장 및 흡수 (바이오매스, 토양 탄소)
- 생태계 서비스 (수원 함양, 홍수 조절, 기후 조절)

---

## 3. 용어 정의

### 3.1 기본 용어

- **생태계 (Ecosystem)**: 생물 군집과 무생물 환경이 상호작용하는 기능 단위
- **생물다양성 (Biodiversity)**: 유전자, 종, 생태계 수준의 다양성
- **생태계 서비스 (Ecosystem Services)**: 인간이 생태계로부터 얻는 혜택
- **건강도 지표 (Health Indicator)**: 생태계 상태를 나타내는 정량적 지표
- **탄소 흡수원 (Carbon Sink)**: CO2를 흡수하여 저장하는 생태계
- **기준선 (Baseline)**: 모니터링 시작 시점의 생태계 상태
- **복원력 (Resilience)**: 교란 후 회복하는 생태계 능력

### 3.2 기술 용어

- **NDVI (Normalized Difference Vegetation Index)**: 정규식생지수
- **LAI (Leaf Area Index)**: 잎면적지수
- **GPP (Gross Primary Productivity)**: 총 일차 생산량
- **NPP (Net Primary Productivity)**: 순 일차 생산량
- **NEP (Net Ecosystem Productivity)**: 순 생태계 생산량
- **SOC (Soil Organic Carbon)**: 토양 유기탄소
- **BOD (Biological Oxygen Demand)**: 생화학적 산소 요구량
- **EDI (Environmental DNA Index)**: 환경 DNA 지수

---

## 4. 생태계 분류 체계

### 4.1 1차 분류 (생태계 유형)

```
생태계
├── 육상 생태계
│   ├── 산림 (Forest)
│   │   ├── 온대림 (Temperate Forest)
│   │   ├── 열대림 (Tropical Forest)
│   │   ├── 한대림 (Boreal Forest)
│   │   └── 맹그로브 (Mangrove)
│   ├── 초원 (Grassland)
│   │   ├── 사바나 (Savanna)
│   │   ├── 스텝 (Steppe)
│   │   └── 고산 초원 (Alpine Grassland)
│   ├── 사막 (Desert)
│   └── 툰드라 (Tundra)
├── 담수 생태계
│   ├── 하천 (River)
│   ├── 호수 (Lake)
│   ├── 습지 (Wetland)
│   └── 지하수 (Groundwater)
├── 해양 생태계
│   ├── 연안 (Coastal)
│   ├── 산호초 (Coral Reef)
│   ├── 해초대 (Seagrass Bed)
│   └── 심해 (Deep Sea)
└── 인공 생태계
    ├── 도시 (Urban)
    ├── 농업 (Agricultural)
    └── 조성 숲 (Plantation)
```

### 4.2 2차 분류 (생태계 코드)

| 분류 코드 | 생태계 유형 | 주요 특징 | 주요 서비스 |
|----------|------------|---------|-----------|
| ECO-01 | 온대 활엽수림 | 낙엽성, 사계절 | 탄소저장, 수원함양 |
| ECO-02 | 열대우림 | 고생물다양성, 상록성 | 산소생산, 기후조절 |
| ECO-03 | 맹그로브 | 해안 식생, 내염성 | 연안보호, 수산자원 |
| ECO-04 | 갯벌 | 조간대, 고영양 | 수질정화, 탄소저장 |
| ECO-05 | 산호초 | 해양 생물다양성 핫스팟 | 어류 서식지, 관광 |
| ECO-06 | 고산 초원 | 고지대, 낮은 생산성 | 생물다양성, 수원 |
| ECO-07 | 도시 숲 | 인공 조성, 관리형 | 대기정화, 열섬완화 |
| ECO-08 | 논 생태계 | 계절적 침수, 경작지 | 식량생산, 생물서식 |
| ECO-09 | 하천 습지 | 범람원, 동적 시스템 | 홍수조절, 수질정화 |
| ECO-10 | 복원 생태계 | 인위적 복원, 천이 중 | 생태복원, 탄소흡수 |

### 4.3 건강도 등급

- **Grade A (우수)**: 자연 상태, 최소 교란, 높은 생물다양성
- **Grade B (양호)**: 양호한 상태, 일부 교란, 안정적 기능
- **Grade C (보통)**: 중간 상태, 교란 가시화, 기능 유지
- **Grade D (불량)**: 훼손 상태, 복원 필요, 기능 저하
- **Grade E (매우 불량)**: 심각한 훼손, 긴급 개입 필요

---

## 5. 데이터 모델

### 5.1 생태계 기본 정보

```typescript
interface EcosystemBasicInfo {
  // 식별 정보
  ecosystemId: string;                // 고유 식별자
  ecosystemCode: string;              // 생태계 코드 (ECO-01 ~ ECO-10)
  name: string;                       // 생태계 명칭
  type: EcosystemType;                // 생태계 유형

  // 위치 정보
  location: {
    coordinates: {
      latitude: number;               // 위도
      longitude: number;              // 경도
      elevation: number;              // 고도 (m)
    };
    area: number;                     // 면적 (ha)
    boundary: GeoJSON;                // 경계 (GeoJSON Polygon)
    region: string;                   // 행정구역
    country: string;                  // 국가
  };

  // 기후 정보
  climate: {
    zone: string;                     // 기후대 (tropical, temperate, etc.)
    meanAnnualTemp: number;           // 연평균 기온 (°C)
    meanAnnualPrecip: number;         // 연평균 강수량 (mm)
    growingSeason: number;            // 생장 기간 (days)
  };

  // 관리 정보
  management: {
    protectionStatus: string;         // 보호 상태 (국립공원, 보호구역 등)
    manager: string;                  // 관리 기관
    establishedDate: string;          // 지정일
    managementPlan?: string;          // 관리 계획 ID
  };

  // 메타데이터
  metadata: {
    createdAt: string;                // 등록일시
    updatedAt: string;                // 수정일시
    dataQuality: number;              // 데이터 품질 (0-100)
    verified: boolean;                // 검증 여부
  };
}
```

### 5.2 생물다양성 모니터링 데이터

```typescript
interface BiodiversityData {
  monitoringId: string;               // 모니터링 ID
  ecosystemId: string;                // 생태계 ID
  timestamp: string;                  // 조사 일시 (ISO 8601)

  // 종 다양성
  species: {
    flora: {
      totalSpecies: number;           // 총 식물 종 수
      endemicSpecies: number;         // 고유종 수
      endangeredSpecies: number;      // 멸종위기종 수
      invasiveSpecies: number;        // 침입외래종 수
      dominantSpecies: {
        scientificName: string;       // 학명
        abundance: number;            // 개체수 또는 피도 (%)
        biomassDensity: number;       // 바이오매스 밀도 (kg/ha)
      }[];
    };
    fauna: {
      totalSpecies: number;           // 총 동물 종 수
      endemicSpecies: number;         // 고유종 수
      endangeredSpecies: number;      // 멸종위기종 수
      keySpecies: {
        scientificName: string;       // 학명
        commonName: string;           // 일반명
        populationSize: number;       // 개체군 크기
        populationTrend: 'increasing' | 'stable' | 'decreasing';
      }[];
    };
    microorganisms?: {
      bacterialDiversity: number;     // 세균 다양성 지수
      fungalDiversity: number;        // 균류 다양성 지수
      soilMicrobiome?: {
        shannon: number;              // Shannon 다양성 지수
        simpson: number;              // Simpson 다양성 지수
      };
    };
  };

  // 다양성 지수
  indices: {
    shannonIndex: number;             // Shannon-Wiener 다양성 지수
    simpsonIndex: number;             // Simpson 다양성 지수
    evenness: number;                 // 균등도
    richness: number;                 // 종 풍부도
  };

  // 조사 방법
  method: {
    surveyType: string;               // 조사 유형 (현장조사, eDNA, 카메라트랩 등)
    samplingArea: number;             // 샘플링 면적 (m²)
    samplingEffort: number;           // 조사 노력 (인시, person-hours)
    observers: string[];              // 조사자
  };
}
```

### 5.3 식생 건강도 데이터

```typescript
interface VegetationHealthData {
  monitoringId: string;
  ecosystemId: string;
  timestamp: string;

  // 원격탐사 지수
  remoteSensingIndices: {
    ndvi: {                           // 정규식생지수
      mean: number;                   // 평균값 (-1 to 1)
      stdDev: number;                 // 표준편차
      min: number;
      max: number;
    };
    evi: {                            // 향상식생지수
      mean: number;
      stdDev: number;
    };
    lai: {                            // 잎면적지수
      mean: number;                   // m²/m²
      stdDev: number;
    };
    fpar: number;                     // 광합성유효복사흡수율 (0-1)
    gpp: number;                      // 총 일차 생산량 (gC/m²/day)
    npp: number;                      // 순 일차 생산량 (gC/m²/day)
  };

  // 현장 측정
  fieldMeasurements: {
    canopyCover: number;              // 수관 피복률 (%)
    canopyHeight: number;             // 수관 높이 (m)
    basalArea: number;                // 흉고단면적 (m²/ha)
    stemDensity: number;              // 줄기 밀도 (stems/ha)
    dbh: {                            // 흉고직경 분포
      mean: number;                   // 평균 (cm)
      distribution: {
        class: string;                // 직경급 (예: "10-20cm")
        count: number;
      }[];
    };
  };

  // 생리적 지표
  physiologicalIndicators?: {
    chlorophyllContent: number;       // 엽록소 함량 (mg/g)
    leafWaterPotential: number;       // 잎 수분 퍼텐셜 (MPa)
    photosynthesisRate: number;       // 광합성 속도 (μmol CO2/m²/s)
    stomatalConductance: number;      // 기공전도도 (mmol H2O/m²/s)
  };

  // 스트레스 지표
  stress: {
    droughtStress: number;            // 가뭄 스트레스 지수 (0-100)
    heatStress: number;               // 열 스트레스 지수 (0-100)
    pestInfestation: number;          // 병해충 피해율 (%)
    diseaseIncidence: number;         // 질병 발생률 (%)
  };
}
```

### 5.4 토양 건강도 데이터

```typescript
interface SoilHealthData {
  monitoringId: string;
  ecosystemId: string;
  timestamp: string;
  sampleLocation: {
    latitude: number;
    longitude: number;
    depth: number;                    // 샘플링 깊이 (cm)
  };

  // 물리적 특성
  physical: {
    texture: string;                  // 토성 (clay, loam, sand)
    bulkDensity: number;              // 용적밀도 (g/cm³)
    porosity: number;                 // 공극률 (%)
    waterContent: number;             // 수분 함량 (%)
    infiltrationRate: number;         // 침투율 (cm/hr)
    aggregateStability: number;       // 입단 안정성 (%)
  };

  // 화학적 특성
  chemical: {
    ph: number;                       // pH
    organicCarbon: number;            // 유기탄소 (%)
    organicMatter: number;            // 유기물 함량 (%)
    totalNitrogen: number;            // 총 질소 (%)
    availablePhosphorus: number;      // 유효인산 (mg/kg)
    cec: number;                      // 양이온교환용량 (cmol/kg)
    nutrients: {
      potassium: number;              // 칼륨 (mg/kg)
      calcium: number;                // 칼슘 (mg/kg)
      magnesium: number;              // 마그네슘 (mg/kg)
      sulfur: number;                 // 황 (mg/kg)
    };
    heavyMetals?: {
      lead: number;                   // 납 (mg/kg)
      cadmium: number;                // 카드뮴 (mg/kg)
      arsenic: number;                // 비소 (mg/kg)
      mercury: number;                // 수은 (mg/kg)
    };
  };

  // 생물학적 특성
  biological: {
    microbialBiomass: number;         // 미생물 바이오매스 (mg C/kg)
    enzymeActivity: {
      dehydrogenase: number;          // 탈수소효소 (μg TPF/g/day)
      urease: number;                 // 우레아제 (μg NH4-N/g/hr)
      phosphatase: number;            // 인산가수분해효소 (μg PNP/g/hr)
    };
    respiration: number;              // 토양 호흡 (mg CO2/kg/day)
    nematodeDiversity: number;        // 선충 다양성 지수
    earthwormDensity: number;         // 지렁이 밀도 (individuals/m²)
  };

  // 탄소 저장
  carbonStorage: {
    organicCarbon: number;            // 유기탄소 (Mg C/ha)
    inorganicCarbon: number;          // 무기탄소 (Mg C/ha)
    totalCarbon: number;              // 총 탄소 (Mg C/ha)
    sequestrationRate: number;        // 탄소 격리 속도 (Mg C/ha/year)
  };
}
```

### 5.5 수질 및 수문 데이터

```typescript
interface WaterQualityData {
  monitoringId: string;
  ecosystemId: string;
  timestamp: string;
  sampleLocation: {
    waterBodyType: 'stream' | 'river' | 'lake' | 'wetland' | 'groundwater';
    latitude: number;
    longitude: number;
    depth?: number;                   // 수심 (m)
  };

  // 물리적 특성
  physical: {
    temperature: number;              // 수온 (°C)
    transparency: number;             // 투명도 (cm, Secchi depth)
    turbidity: number;                // 탁도 (NTU)
    color: number;                    // 색도 (Pt-Co)
    conductivity: number;             // 전기전도도 (μS/cm)
  };

  // 화학적 특성
  chemical: {
    ph: number;                       // pH
    dissolvedOxygen: number;          // 용존산소 (mg/L)
    bod: number;                      // BOD (mg/L)
    cod: number;                      // COD (mg/L)
    toc: number;                      // 총유기탄소 (mg/L)
    nutrients: {
      totalNitrogen: number;          // 총 질소 (mg/L)
      ammonium: number;               // 암모늄 (mg/L)
      nitrate: number;                // 질산염 (mg/L)
      totalPhosphorus: number;        // 총 인 (mg/L)
      phosphate: number;              // 인산염 (mg/L)
    };
    ions: {
      chloride: number;               // 염화물 (mg/L)
      sulfate: number;                // 황산염 (mg/L)
      calcium: number;                // 칼슘 (mg/L)
      magnesium: number;              // 마그네슘 (mg/L)
    };
  };

  // 생물학적 특성
  biological: {
    chlorophyllA: number;             // 엽록소 a (μg/L)
    phytoplanktonDensity: number;     // 식물플랑크톤 밀도 (cells/mL)
    zooplanktonDensity: number;       // 동물플랑크톤 밀도 (individuals/L)
    bacterialCount: number;           // 총 세균수 (CFU/mL)
    coliformBacteria: number;         // 대장균군 (MPN/100mL)
    benthicIndex?: number;            // 저서생물 지수
  };

  // 수문 특성
  hydrology?: {
    streamflow: number;               // 유량 (m³/s)
    waterLevel: number;               // 수위 (m)
    precipitation: number;            // 강수량 (mm)
    evapotranspiration: number;       // 증발산량 (mm)
  };

  // 수질 등급
  qualityGrade: 'excellent' | 'good' | 'moderate' | 'poor' | 'very_poor';
}
```

### 5.6 탄소 흡수 및 저장 데이터

```typescript
interface CarbonSequestrationData {
  monitoringId: string;
  ecosystemId: string;
  timestamp: string;

  // 바이오매스 탄소
  biomassCarbon: {
    abovegroundBiomass: number;       // 지상부 바이오매스 (Mg/ha)
    belowgroundBiomass: number;       // 지하부 바이오매스 (Mg/ha)
    totalBiomass: number;             // 총 바이오매스 (Mg/ha)
    carbonContent: number;            // 탄소 함량 (%)
    carbonStock: number;              // 탄소 저장량 (Mg C/ha)
  };

  // 토양 탄소
  soilCarbon: {
    depth0to30cm: number;             // 0-30cm 탄소 (Mg C/ha)
    depth30to100cm: number;           // 30-100cm 탄소 (Mg C/ha)
    totalSoilCarbon: number;          // 총 토양 탄소 (Mg C/ha)
  };

  // 낙엽층 탄소
  litterCarbon: number;               // 낙엽층 탄소 (Mg C/ha)

  // 총 탄소 저장
  totalCarbonStock: number;           // 총 탄소 저장량 (Mg C/ha)

  // 탄소 흡수 속도
  sequestrationRate: {
    annual: number;                   // 연간 흡수율 (Mg C/ha/year)
    perHectare: number;               // 헥타르당 (Mg C/ha/year)
    totalAnnual: number;              // 전체 연간 흡수 (Mg C/year)
  };

  // 탄소 플럭스
  carbonFlux: {
    gpp: number;                      // 총 일차 생산 (gC/m²/day)
    npp: number;                      // 순 일차 생산 (gC/m²/day)
    nep: number;                      // 순 생태계 생산 (gC/m²/day)
    nee: number;                      // 순 생태계 교환 (gC/m²/day)
    soilRespiration: number;          // 토양 호흡 (gC/m²/day)
  };

  // CO2 등가
  co2Equivalent: {
    sequestered: number;              // CO2 흡수량 (Mg CO2e/year)
    avoided: number;                  // CO2 회피량 (Mg CO2e/year)
    total: number;                    // 총 CO2 감축 효과 (Mg CO2e/year)
  };

  // 측정 방법
  methodology: {
    biomassMethod: string;            // 바이오매스 측정법 (allometric, destructive)
    soilSamplingMethod: string;       // 토양 샘플링법
    fluxMeasurementMethod: string;    // 플럭스 측정법 (eddy covariance, chamber)
    uncertaintyRange: number;         // 불확실성 범위 (%)
  };
}
```

---

## 6. 모니터링 프로토콜

### 6.1 모니터링 주기

#### 6.1.1 표준 모니터링 주기

| 항목 | 주기 | 방법 | 비고 |
|------|------|------|------|
| 위성 영상 (NDVI) | 주 1회 | Sentinel-2, Landsat | 계절별 변화 추적 |
| 기상 데이터 | 실시간 | IoT 기상센서 | 온도, 습도, 강수 |
| 토양 수분 | 일 1회 | IoT 토양센서 | 뿌리층 수분 |
| 수질 모니터링 | 주 1회 | 자동 수질센서 | 주요 수질 지표 |
| 식생 현장 조사 | 계절별 (연 4회) | 현장 조사 | 종 구성, 피복 |
| 생물다양성 조사 | 연 2회 | 현장 조사, 카메라트랩 | 봄, 가을 |
| 토양 샘플링 | 연 1회 | 실험실 분석 | 화학, 물리, 생물학적 |
| 탄소 플럭스 | 연속 (30분 간격) | 에디 공분산 타워 | 주요 생태계만 |
| 바이오매스 측정 | 연 1회 | 현장 측정 | 성장 시즌 종료 후 |

#### 6.1.2 긴급 모니터링

다음 상황 발생 시 긴급 모니터링 실시:
- 자연재해 (산불, 홍수, 태풍, 가뭄)
- 병해충 대발생
- 대규모 동물 집단 폐사
- 수질 급격한 악화
- 불법 벌목 또는 개발

### 6.2 현장 조사 프로토콜

#### 6.2.1 식생 조사

```typescript
interface VegetationSurveyProtocol {
  plotSetup: {
    plotSize: {                       // 방형구 크기
      trees: string;                  // "20m × 20m"
      shrubs: string;                 // "5m × 5m"
      herbs: string;                  // "1m × 1m"
    };
    numberOfPlots: number;            // 방형구 수 (최소 3개)
    plotLayout: 'systematic' | 'random' | 'stratified';
  };

  measurements: {
    trees: {
      species: boolean;               // 수종 식별
      dbh: boolean;                   // 흉고직경 측정
      height: boolean;                // 수고 측정
      canopyCover: boolean;           // 수관 피복 측정
      healthStatus: boolean;          // 건강 상태 평가
    };
    understory: {
      speciesComposition: boolean;    // 종 구성
      coverPercentage: boolean;       // 피복률
      heightClass: boolean;           // 높이급
    };
  };

  dataRecording: {
    gpsLocation: boolean;             // GPS 좌표 기록
    photos: boolean;                  // 사진 촬영
    fieldNotes: boolean;              // 현장 메모
    datasheet: string;                // 조사 양식 ID
  };
}
```

#### 6.2.2 토양 샘플링

- **샘플링 깊이**: 0-10cm, 10-30cm, 30-100cm (각 깊이별 복합 샘플)
- **샘플 수**: 생태계 면적 1ha당 최소 3개 복합 샘플
- **샘플링 도구**: 토양 오거 (soil auger) 또는 코어 샘플러
- **보관**: 냉장 보관 (4°C), 72시간 이내 분석
- **분석 항목**: pH, 유기물, 질소, 인, 탄소, 수분, 미생물

#### 6.2.3 수질 샘플링

- **샘플링 위치**: 유입부, 중심부, 유출부 (하천/호수)
- **샘플링 깊이**: 표층 (0-0.5m), 중층, 저층 (층화된 수역)
- **샘플 용기**: 멸균 폴리에틸렌 또는 유리병
- **현장 측정**: pH, DO, 수온, 전기전도도
- **실험실 분석**: 영양염류, BOD, COD, 중금속

---

## 7. 건강도 지표

### 7.1 종합 건강도 지수 (Ecosystem Health Index, EHI)

```typescript
interface EcosystemHealthIndex {
  ecosystemId: string;
  timestamp: string;

  // 하위 지수
  subIndices: {
    biodiversityIndex: number;        // 생물다양성 지수 (0-100)
    vegetationIndex: number;          // 식생 건강도 지수 (0-100)
    soilIndex: number;                // 토양 건강도 지수 (0-100)
    waterIndex: number;               // 수질 건강도 지수 (0-100)
    carbonIndex: number;              // 탄소 기능 지수 (0-100)
  };

  // 종합 지수
  overallEHI: number;                 // 종합 EHI (0-100)
  healthGrade: 'A' | 'B' | 'C' | 'D' | 'E';

  // 추세
  trend: {
    direction: 'improving' | 'stable' | 'declining';
    changeRate: number;               // 연간 변화율 (%)
    comparedTo: string;               // 비교 기준 (baseline, previous_year)
  };

  // 위협 요소
  threats: {
    type: string;                     // 위협 유형
    severity: 'low' | 'medium' | 'high' | 'critical';
    description: string;
  }[];

  // 권고사항
  recommendations: string[];
}
```

### 7.2 생물다양성 지수 계산

```
생물다양성 지수 = (Shannon 지수 × 40%) +
                  (종 풍부도 × 30%) +
                  (고유종/멸종위기종 보전 × 30%)

여기서:
- Shannon 지수는 0-5 범위를 0-100으로 정규화
- 종 풍부도는 생태계 유형별 기준값 대비 백분율
- 고유종/멸종위기종은 서식 종 수 기반 점수
```

### 7.3 식생 건강도 지수 계산

```
식생 건강도 지수 = (NDVI 점수 × 35%) +
                   (바이오매스 점수 × 25%) +
                   (생산성 점수 × 25%) +
                   (스트레스 점수 × 15%)

여기서:
- NDVI 점수: 0.8 이상 = 100점, 0.5 이하 = 0점 (선형)
- 바이오매스: 생태계 유형별 기준값 대비
- 생산성: NPP 기반
- 스트레스: 병해충, 가뭄 등 역산 점수
```

---

## 8. 탄소 흡수 및 저장

### 8.1 탄소 저장량 추정 방법

#### 8.1.1 산림 바이오매스 탄소

**상대생장식 (Allometric Equation) 방법:**

```
지상부 바이오매스 (AGB) = a × DBH^b

여기서:
- DBH = 흉고직경 (cm)
- a, b = 수종별 계수 (IPCC 또는 국가 계수)

탄소량 (C) = AGB × 탄소 함량 계수 (일반적으로 0.47~0.50)
```

**주요 수종별 계수 (예시):**

| 수종 | a | b | 탄소 함량 |
|------|---|---|----------|
| 소나무 | 0.0509 | 2.4257 | 0.48 |
| 참나무 | 0.0678 | 2.3584 | 0.49 |
| 낙엽송 | 0.0481 | 2.4564 | 0.47 |
| 열대림 | 0.0673 | 2.295 | 0.47 |

#### 8.1.2 토양 유기탄소 (SOC)

```
SOC (Mg C/ha) = BD × Depth × OC% × 100

여기서:
- BD = 용적밀도 (g/cm³)
- Depth = 토층 깊이 (cm)
- OC% = 유기탄소 함량 (%)
```

#### 8.1.3 습지 탄소

- **갯벌**: 20-200 Mg C/ha (깊이 1m)
- **맹그로브**: 500-1,000 Mg C/ha (총 탄소)
- **이탄지**: 1,000-3,000 Mg C/ha (깊이 의존)

### 8.2 탄소 흡수 속도 측정

#### 8.2.1 에디 공분산 (Eddy Covariance) 방법

```typescript
interface EddyCovarianceTower {
  towerId: string;
  ecosystemId: string;
  location: { latitude: number; longitude: number; };
  height: number;                     // 타워 높이 (m)

  instruments: {
    sonicAnemometer: {                // 초음파 풍속계
      model: string;
      frequency: number;              // Hz (일반적으로 10-20Hz)
    };
    irgaAnalyzer: {                   // CO2/H2O 적외선 가스분석기
      model: string;
      frequency: number;
    };
  };

  measurements: {
    timestamp: string;                // 30분 평균
    nee: number;                      // 순 생태계 교환 (μmol CO2/m²/s)
    le: number;                       // 잠열 플럭스 (W/m²)
    h: number;                        // 현열 플럭스 (W/m²)
    ustar: number;                    // 마찰속도 (m/s)
    windSpeed: number;                // 풍속 (m/s)
    windDirection: number;            // 풍향 (°)
  };
}
```

#### 8.2.2 체임버 (Chamber) 방법

- **자동 체임버**: 토양 호흡 및 NEE 측정
- **측정 빈도**: 30분-1시간 간격
- **체임버 크기**: 일반적으로 50cm × 50cm
- **측정 항목**: CO2, CH4, N2O 플럭스

### 8.3 탄소 크레딧 산정

```typescript
interface CarbonCreditCalculation {
  projectId: string;
  ecosystemId: string;
  methodology: string;                // VCS, Gold Standard, CDM 등

  // 기준선
  baseline: {
    carbonStock: number;              // 기준 탄소 저장량 (Mg C)
    year: number;                     // 기준 연도
  };

  // 프로젝트 시나리오
  project: {
    carbonStock: number;              // 프로젝트 탄소 저장량 (Mg C)
    sequestrationRate: number;        // 연간 흡수율 (Mg C/year)
    monitoringPeriod: number;         // 모니터링 기간 (years)
  };

  // 탄소 크레딧
  credits: {
    totalReductions: number;          // 총 감축량 (Mg CO2e)
    annualReductions: number;         // 연간 감축량 (Mg CO2e/year)
    verifiedCredits: number;          // 검증된 크레딧 (tCO2e)
    vintage: number;                  // 발행 연도
  };

  // 누출 및 영속성
  adjustments: {
    leakage: number;                  // 누출 (%)
    permanenceDiscount: number;       // 영속성 할인 (%)
    uncertaintyDeduction: number;     // 불확실성 차감 (%)
  };

  // 최종 크레딧
  netCredits: number;                 // 순 크레딧 (tCO2e)

  verification: {
    verifier: string;                 // 검증기관
    verificationDate: string;
    certificationBody: string;        // 인증 기관 (Verra, Gold Standard)
    certificationNumber: string;
  };
}
```

---

## 9. 물 순환 모니터링

### 9.1 물 수지 (Water Balance)

```
물 수지: P = ET + Q + ΔS

여기서:
- P = 강수량 (Precipitation)
- ET = 증발산량 (Evapotranspiration)
- Q = 유출량 (Runoff)
- ΔS = 저장 변화 (Change in Storage)
```

### 9.2 수문 데이터 모델

```typescript
interface HydrologicalData {
  ecosystemId: string;
  timestamp: string;

  // 강수
  precipitation: {
    rainfall: number;                 // 강우량 (mm)
    snowfall: number;                 // 강설량 (mm water equivalent)
    intensity: number;                // 강우강도 (mm/hr)
    duration: number;                 // 지속시간 (hr)
  };

  // 증발산
  evapotranspiration: {
    actual: number;                   // 실제 증발산 (mm)
    potential: number;                // 잠재 증발산 (mm)
    evaporation: number;              // 증발 (mm)
    transpiration: number;            // 증산 (mm)
  };

  // 지표수
  surfaceWater: {
    streamflow: number;               // 하천 유량 (m³/s)
    waterLevel: number;               // 수위 (m)
    inundatedArea: number;            // 침수 면적 (ha)
    waterStorage: number;             // 저수량 (m³)
  };

  // 지하수
  groundwater: {
    waterTableDepth: number;          // 지하수위 깊이 (m)
    rechargeRate: number;             // 함양률 (mm/year)
    dischargeRate: number;            // 배출률 (m³/day)
  };

  // 토양 수분
  soilMoisture: {
    depth0to10cm: number;             // 0-10cm 수분 (%)
    depth10to30cm: number;            // 10-30cm 수분 (%)
    depth30to100cm: number;           // 30-100cm 수분 (%)
    fieldCapacity: number;            // 포장용수량 (%)
    wiltingPoint: number;             // 위조점 (%)
  };

  // 물 순환 서비스
  ecosystemServices: {
    waterRetention: number;           // 수원 함양량 (m³/year)
    floodMitigation: number;          // 홍수 조절 용량 (m³)
    waterPurification: number;        // 수질 정화 능력 (등급)
  };
}
```

### 9.3 수원 함양 서비스 평가

```typescript
interface WaterRetentionService {
  ecosystemId: string;

  // 수원 함양 용량
  retentionCapacity: {
    annualRetention: number;          // 연간 함양량 (m³/year)
    perHectare: number;               // 헥타르당 (m³/ha/year)
    totalVolume: number;              // 총 저장 용량 (m³)
  };

  // 홍수 조절
  floodRegulation: {
    peakFlowReduction: number;        // 첨두 유량 감소율 (%)
    detentionTime: number;            // 체류 시간 (hours)
    floodStorageCapacity: number;     // 홍수 저장 용량 (m³)
  };

  // 가뭄 완화
  droughtMitigation: {
    baseflowContribution: number;     // 기저유량 기여도 (%)
    drySeasonWaterSupply: number;     // 건기 물 공급량 (m³)
  };

  // 경제적 가치
  economicValue: {
    replacementCost: number;          // 대체 비용 (원/year)
    waterSupplyValue: number;         // 물 공급 가치 (원/m³)
    floodDamageAvoided: number;       // 홍수 피해 회피액 (원/year)
  };
}
```

---

## 10. 영양분 순환

### 10.1 질소 순환

```typescript
interface NitrogenCycleData {
  ecosystemId: string;
  timestamp: string;

  // 질소 저장고
  pools: {
    atmosphericN2: number;            // 대기 N2 (kg N/ha)
    soilOrganicN: number;             // 토양 유기질소 (kg N/ha)
    soilInorganicN: number;           // 토양 무기질소 (kg N/ha)
    plantBiomassN: number;            // 식물체 질소 (kg N/ha)
    microbialBiomassN: number;        // 미생물 질소 (kg N/ha)
  };

  // 질소 플럭스
  fluxes: {
    atmosphericDeposition: number;    // 대기 침적 (kg N/ha/year)
    biologicalFixation: number;       // 생물학적 고정 (kg N/ha/year)
    mineralization: number;           // 무기화 (kg N/ha/year)
    immobilization: number;           // 부동화 (kg N/ha/year)
    nitrification: number;            // 질산화 (kg N/ha/year)
    denitrification: number;          // 탈질 (kg N/ha/year)
    plantUptake: number;              // 식물 흡수 (kg N/ha/year)
    leaching: number;                 // 용탈 (kg N/ha/year)
    volatilization: number;           // 휘발 (kg N/ha/year)
  };

  // 질소 수지
  budget: {
    inputs: number;                   // 총 유입 (kg N/ha/year)
    outputs: number;                  // 총 유출 (kg N/ha/year)
    netBalance: number;               // 순 수지 (kg N/ha/year)
  };

  // 질소 이용 효율
  efficiency: {
    nue: number;                      // 질소 이용 효율 (%)
    retentionRate: number;            // 생태계 질소 보유율 (%)
  };
}
```

### 10.2 인 순환

```typescript
interface PhosphorusCycleData {
  ecosystemId: string;
  timestamp: string;

  // 인 저장고
  pools: {
    soilOrganicP: number;             // 토양 유기인 (kg P/ha)
    soilInorganicP: number;           // 토양 무기인 (kg P/ha)
    availableP: number;               // 유효 인 (kg P/ha)
    plantBiomassP: number;            // 식물체 인 (kg P/ha)
    litterP: number;                  // 낙엽층 인 (kg P/ha)
  };

  // 인 플럭스
  fluxes: {
    weathering: number;               // 풍화 (kg P/ha/year)
    mineralization: number;           // 무기화 (kg P/ha/year)
    immobilization: number;           // 부동화 (kg P/ha/year)
    plantUptake: number;              // 식물 흡수 (kg P/ha/year)
    litterfall: number;               // 낙엽 귀환 (kg P/ha/year)
    leaching: number;                 // 용탈 (kg P/ha/year)
    runoff: number;                   // 지표 유출 (kg P/ha/year)
  };

  // 인 제한 여부
  limitation: {
    isLimiting: boolean;              // 인 제한 생태계 여부
    npRatio: number;                  // N:P 비율
    criticalNPRatio: number;          // 임계 N:P 비율
  };
}
```

---

## 11. 위성 영상 통합

### 11.1 지원 위성 플랫폼

| 위성 | 센서 | 해상도 | 재방문 주기 | 주요 용도 |
|------|------|--------|------------|----------|
| Sentinel-2 | MSI | 10-60m | 5일 | NDVI, LAI, 식생 모니터링 |
| Landsat 8/9 | OLI | 30m | 16일 | 장기 변화 추적 |
| MODIS | Terra/Aqua | 250-1000m | 1-2일 | 일별 NDVI, GPP |
| Planet | Dove | 3m | 일별 | 고해상도 변화 탐지 |
| Sentinel-1 | SAR | 10m | 6-12일 | 바이오매스, 토양 수분 |
| GEDI | Lidar | 25m footprint | - | 수목 높이, 바이오매스 |

### 11.2 위성 데이터 모델

```typescript
interface SatelliteImageryData {
  imageId: string;
  ecosystemId: string;

  // 이미지 메타데이터
  metadata: {
    satellite: string;                // 위성명 (Sentinel-2, Landsat-8)
    sensor: string;                   // 센서명 (MSI, OLI)
    acquisitionDate: string;          // 촬영 일시
    cloudCover: number;               // 구름 피복률 (%)
    sunElevation: number;             // 태양 고도각 (°)
    processingLevel: string;          // 처리 수준 (L1C, L2A)
  };

  // 공간 정보
  spatial: {
    resolution: number;               // 공간 해상도 (m)
    extent: {                         // 범위
      minLat: number;
      maxLat: number;
      minLon: number;
      maxLon: number;
    };
    crs: string;                      // 좌표계 (EPSG:4326)
  };

  // 스펙트럼 밴드
  bands: {
    blue: number[];                   // B2 (490nm)
    green: number[];                  // B3 (560nm)
    red: number[];                    // B4 (665nm)
    redEdge: number[];                // B5-B7 (705-783nm)
    nir: number[];                    // B8 (842nm)
    swir1: number[];                  // B11 (1610nm)
    swir2: number[];                  // B12 (2190nm)
  };

  // 파생 지수
  indices: {
    ndvi: {                           // 정규식생지수
      mean: number;
      min: number;
      max: number;
      stdDev: number;
    };
    evi: number;                      // 향상식생지수
    savi: number;                     // 토양조정식생지수
    ndwi: number;                     // 정규수분지수
    ndmi: number;                     // 정규습윤지수
    nbr: number;                      // 정규연소비율
  };

  // 분류 결과
  classification?: {
    landCoverType: string;            // 토지 피복 유형
    confidence: number;               // 분류 신뢰도 (%)
    changeDetection?: {
      changeType: string;             // 변화 유형
      changeMagnitude: number;        // 변화 크기
      previousDate: string;           // 비교 기준일
    };
  };

  // 바이오매스 추정 (SAR 또는 Lidar)
  biomassEstimate?: {
    abovegroundBiomass: number;       // 지상부 바이오매스 (Mg/ha)
    canopyHeight: number;             // 수관 높이 (m)
    uncertainty: number;              // 불확실성 (%)
  };
}
```

### 11.3 영상 분석 워크플로우

```typescript
interface ImageryAnalysisWorkflow {
  workflowId: string;
  ecosystemId: string;

  steps: [
    {
      stepName: 'Image Acquisition';
      description: '위성 영상 다운로드 (Sentinel-2 L2A)';
      automated: true;
      frequency: 'weekly';
    },
    {
      stepName: 'Cloud Masking';
      description: '구름 마스킹 및 품질 평가';
      algorithm: 'Sen2Cor QA band';
      threshold: { maxCloudCover: 20 };
    },
    {
      stepName: 'Index Calculation';
      description: 'NDVI, EVI, LAI 계산';
      indices: ['NDVI', 'EVI', 'LAI'];
    },
    {
      stepName: 'Zonal Statistics';
      description: '생태계 경계 내 통계 추출';
      statistics: ['mean', 'min', 'max', 'stdDev'];
    },
    {
      stepName: 'Change Detection';
      description: '시계열 변화 탐지';
      method: 'time-series-analysis';
      baselinePeriod: '2020-2022';
    },
    {
      stepName: 'Anomaly Detection';
      description: '이상 징후 탐지 (산불, 병해충)';
      algorithm: 'threshold-based';
      alertThreshold: -0.2;
    },
    {
      stepName: 'Report Generation';
      description: '모니터링 보고서 자동 생성';
      format: 'PDF + GeoJSON';
      recipients: ['manager@example.com'];
    }
  ];

  outputs: {
    timeSeries: string;               // 시계열 데이터 파일 경로
    changeMap: string;                // 변화 지도 파일
    report: string;                   // 보고서 파일
  };
}
```

---

## 12. IoT 센서 네트워크

### 12.1 센서 유형 및 사양

#### 12.1.1 기상 센서

```typescript
interface WeatherSensorStation {
  stationId: string;
  ecosystemId: string;
  location: { latitude: number; longitude: number; elevation: number; };

  sensors: {
    temperature: {
      model: string;                  // 예: Vaisala HMP155
      range: string;                  // "-40 to +60°C"
      accuracy: string;               // "±0.2°C"
      interval: number;               // 측정 간격 (분)
    };
    humidity: {
      model: string;
      range: string;                  // "0 to 100%"
      accuracy: string;               // "±2%"
    };
    precipitation: {
      model: string;                  // 예: TE525 Tipping Bucket
      resolution: string;             // "0.1mm"
      accuracy: string;               // "±1%"
    };
    solarRadiation: {
      model: string;
      range: string;                  // "0 to 2000 W/m²"
      accuracy: string;               // "±5%"
    };
    windSpeed: {
      model: string;
      range: string;                  // "0 to 50 m/s"
      accuracy: string;               // "±0.3 m/s"
    };
    windDirection: {
      model: string;
      range: string;                  // "0 to 360°"
      accuracy: string;               // "±3°"
    };
    barometricPressure: {
      model: string;
      range: string;                  // "500 to 1100 hPa"
      accuracy: string;               // "±0.5 hPa"
    };
  };

  dataTransmission: {
    protocol: 'LoRaWAN' | '4G' | 'Satellite';
    frequency: number;                // 전송 빈도 (분)
    powerSource: 'solar' | 'battery' | 'grid';
    batteryLevel?: number;            // 배터리 잔량 (%)
  };

  measurements: {
    timestamp: string;
    temperature: number;              // °C
    humidity: number;                 // %
    precipitation: number;            // mm
    solarRadiation: number;           // W/m²
    windSpeed: number;                // m/s
    windDirection: number;            // °
    pressure: number;                 // hPa
  };
}
```

#### 12.1.2 토양 센서

```typescript
interface SoilSensorArray {
  arrayId: string;
  ecosystemId: string;
  location: { latitude: number; longitude: number; };

  sensors: {
    moisture: {
      depths: number[];               // [10, 30, 60, 100] cm
      model: string;                  // 예: Decagon 5TE
      type: 'capacitance' | 'TDR' | 'tensiometer';
      accuracy: string;               // "±3%"
    };
    temperature: {
      depths: number[];               // [10, 30, 60] cm
      accuracy: string;               // "±0.5°C"
    };
    conductivity: {
      depths: number[];               // [10, 30] cm
      unit: 'dS/m';
    };
  };

  measurements: {
    timestamp: string;
    soilMoisture: {
      depth10cm: number;              // % VWC
      depth30cm: number;
      depth60cm: number;
      depth100cm: number;
    };
    soilTemperature: {
      depth10cm: number;              // °C
      depth30cm: number;
      depth60cm: number;
    };
    soilConductivity: {
      depth10cm: number;              // dS/m
      depth30cm: number;
    };
  };
}
```

#### 12.1.3 수질 센서

```typescript
interface WaterQualitySensor {
  sensorId: string;
  ecosystemId: string;
  waterBodyType: 'stream' | 'lake' | 'wetland';
  location: { latitude: number; longitude: number; depth: number; };

  sensors: {
    multiparameter: {
      model: string;                  // 예: YSI EXO2
      parameters: string[];           // ['DO', 'pH', 'Temp', 'Conductivity', 'Turbidity']
    };
  };

  measurements: {
    timestamp: string;
    dissolvedOxygen: number;          // mg/L
    oxygenSaturation: number;         // %
    ph: number;                       // pH units
    temperature: number;              // °C
    conductivity: number;             // μS/cm
    turbidity: number;                // NTU
    chlorophyllA?: number;            // μg/L
    blueGreenAlgae?: number;          // cells/mL
  };

  maintenance: {
    lastCalibration: string;
    nextCalibration: string;
    lastCleaning: string;
    sensorHealth: 'good' | 'fair' | 'poor';
  };
}
```

#### 12.1.4 CO2 플럭스 센서 (자동 체임버)

```typescript
interface AutomatedCO2Chamber {
  chamberId: string;
  ecosystemId: string;
  location: { latitude: number; longitude: number; };

  configuration: {
    chamberType: 'soil' | 'ecosystem';
    area: number;                     // m²
    volume: number;                   // L
    closureTime: number;              // seconds
    measurementFrequency: number;     // per day
  };

  sensors: {
    co2Analyzer: {
      model: string;                  // 예: LI-COR LI-8100A
      range: string;                  // "0-20000 ppm"
      accuracy: string;               // "1.5%"
    };
    temperature: {
      airTemp: boolean;
      soilTemp: boolean;
    };
    soilMoisture: boolean;
    par: boolean;                     // Photosynthetically Active Radiation
  };

  measurements: {
    timestamp: string;
    co2Flux: number;                  // μmol CO2/m²/s
    ch4Flux?: number;                 // nmol CH4/m²/s
    n2oFlux?: number;                 // nmol N2O/m²/s
    airTemp: number;                  // °C
    soilTemp: number;                 // °C
    soilMoisture: number;             // %
    par: number;                      // μmol/m²/s
  };
}
```

### 12.2 센서 네트워크 아키텍처

```
센서 레이어 (Field Layer)
    ├── 기상 스테이션 (10-50개/생태계)
    ├── 토양 센서 어레이 (20-100개/생태계)
    ├── 수질 센서 (5-20개/생태계)
    └── CO2 체임버 (1-10개/생태계)
         ↓ (LoRaWAN / 4G / Satellite)
게이트웨이 레이어 (Gateway Layer)
    ├── LoRaWAN 게이트웨이 (1-5개/생태계)
    ├── 데이터 집계 및 전처리
    └── 로컬 저장 (엣지 컴퓨팅)
         ↓ (인터넷)
클라우드 레이어 (Cloud Layer)
    ├── 데이터 수집 서버
    ├── 데이터베이스 (시계열 DB)
    ├── 분석 엔진 (AI/ML)
    └── API 서버
         ↓
응용 레이어 (Application Layer)
    ├── 실시간 대시보드
    ├── 모바일 앱
    ├── 알림 시스템
    └── 보고서 생성
```

---

## 13. 복원 프로젝트 관리

### 13.1 복원 프로젝트 데이터 모델

```typescript
interface RestorationProject {
  projectId: string;
  ecosystemId: string;

  // 기본 정보
  basicInfo: {
    name: string;                     // 프로젝트명
    location: {
      address: string;
      coordinates: { latitude: number; longitude: number; };
      area: number;                   // 복원 면적 (ha)
    };
    type: string;                     // 복원 유형 (재조림, 습지복원, 하천복원 등)
    objective: string;                // 복원 목표
    startDate: string;
    plannedEndDate: string;
    actualEndDate?: string;
  };

  // 초기 상태 (Baseline)
  baseline: {
    assessmentDate: string;
    degradationType: string;          // 훼손 유형 (산림 파괴, 토양 침식 등)
    degradationSeverity: 'low' | 'medium' | 'high' | 'severe';

    initialConditions: {
      vegetationCover: number;        // 식생 피복률 (%)
      speciesRichness: number;        // 종 풍부도
      soilHealth: number;             // 토양 건강도 점수 (0-100)
      waterQuality: number;           // 수질 점수 (0-100)
      carbonStock: number;            // 탄소 저장량 (Mg C/ha)
    };

    threats: string[];                // 주요 위협 요소
    photos: string[];                 // 초기 사진
  };

  // 복원 계획
  plan: {
    targets: {
      vegetationCoverTarget: number;  // 목표 식생 피복률 (%)
      speciesRichnessTarget: number;  // 목표 종 풍부도
      carbonStockTarget: number;      // 목표 탄소 저장량 (Mg C/ha)
      timeframe: number;              // 목표 달성 기간 (years)
    };

    activities: {
      activityId: string;
      activityType: string;           // 활동 유형 (식재, 토양 개량, 외래종 제거 등)
      description: string;
      scheduledDate: string;
      completedDate?: string;
      status: 'planned' | 'ongoing' | 'completed' | 'cancelled';
      budget: number;                 // 예산 (원)
      actualCost?: number;            // 실제 비용 (원)
    }[];

    species: {
      scientificName: string;         // 식재 종
      commonName: string;
      quantity: number;               // 식재 개체 수
      survivorRate?: number;          // 생존율 (%)
      spacing: string;                // 식재 간격 (예: "2m × 2m")
      source: string;                 // 묘목 출처
    }[];
  };

  // 모니터링 데이터 (시계열)
  monitoringRecords: {
    recordId: string;
    date: string;

    vegetation: {
      cover: number;                  // 식생 피복률 (%)
      height: number;                 // 평균 수고 (m)
      survivorRate: number;           // 생존율 (%)
      speciesRichness: number;        // 종 풍부도
      invasiveSpeciesCover: number;   // 침입외래종 피복률 (%)
    };

    soil: {
      organicMatter: number;          // 유기물 함량 (%)
      ph: number;
      bulkDensity: number;            // 용적밀도 (g/cm³)
    };

    carbonStock: number;              // 탄소 저장량 (Mg C/ha)

    biodiversity: {
      birdSpecies: number;            // 조류 종 수
      mammalSpecies: number;          // 포유류 종 수
      insectFamilies: number;         // 곤충 과 수
    };

    threats: {
      erosion: 'none' | 'low' | 'medium' | 'high';
      invasiveSpecies: 'none' | 'low' | 'medium' | 'high';
      disease: 'none' | 'low' | 'medium' | 'high';
      browsing: 'none' | 'low' | 'medium' | 'high';
    };

    photos: string[];
    notes: string;
  }[];

  // 성과 평가
  performance: {
    currentStatus: 'on_track' | 'needs_attention' | 'off_track' | 'success';
    achievementRate: number;          // 목표 달성률 (%)

    successCriteria: {
      criterionName: string;
      targetValue: number;
      currentValue: number;
      achieved: boolean;
    }[];

    lessonsLearned: string[];
    recommendations: string[];
  };

  // 이해관계자
  stakeholders: {
    organization: string;
    role: string;                     // 역할 (주관기관, 협력기관, 지역사회 등)
    contactPerson: string;
    email: string;
    phone: string;
  }[];
}
```

### 13.2 복원 성공 기준

| 기준 | 측정 방법 | 목표값 (5년 후) | 평가 주기 |
|------|----------|----------------|----------|
| 식생 피복률 | 현장 조사, 위성 영상 | ≥ 70% | 연 2회 |
| 종 다양성 | 현장 조사 | 초기 대비 50% 증가 | 연 1회 |
| 외래 침입종 | 현장 조사 | < 10% 피복 | 연 2회 |
| 식재목 생존율 | 현장 조사 | ≥ 80% | 연 1회 |
| 토양 유기물 | 실험실 분석 | ≥ 5% | 연 1회 |
| 탄소 저장량 | 현장 측정, 모델링 | 초기 대비 100% 증가 | 연 1회 |
| 생태계 서비스 | 모델링, 평가 | 기준선 대비 유의미한 증가 | 3년차, 5년차 |

### 13.3 적응적 관리 (Adaptive Management)

```typescript
interface AdaptiveManagementCycle {
  cycleId: string;
  projectId: string;
  cycleNumber: number;              // 사이클 번호

  // 1단계: 계획 (Plan)
  plan: {
    objectives: string[];
    hypotheses: string[];            // 관리 가설
    actions: string[];               // 계획된 조치
    monitoringPlan: string;          // 모니터링 계획
  };

  // 2단계: 실행 (Do)
  implementation: {
    startDate: string;
    endDate: string;
    actualActions: string[];         // 실제 수행 조치
    challenges: string[];            // 발생한 문제
  };

  // 3단계: 점검 (Check)
  monitoring: {
    monitoringDate: string;
    results: {
      indicator: string;
      measuredValue: number;
      expectedValue: number;
      deviation: number;             // 편차 (%)
    }[];
    findings: string[];              // 주요 발견사항
  };

  // 4단계: 조치 (Act)
  adjustment: {
    decision: 'continue' | 'modify' | 'stop';
    modifications: string[];         // 계획 수정 사항
    reasoning: string;               // 의사결정 근거
    nextSteps: string[];             // 다음 단계
  };
}
```

---

## 14. 보호구역 관리

### 14.1 보호구역 데이터 모델

```typescript
interface ProtectedArea {
  protectedAreaId: string;
  ecosystemId: string;

  // 기본 정보
  designation: {
    name: string;                     // 보호구역명
    category: string;                 // IUCN 카테고리 (Ia, Ib, II, III, IV, V, VI)
    designation: string;              // 지정 유형 (국립공원, 천연보호구역 등)
    internationalDesignation?: string; // 국제 지정 (UNESCO, Ramsar 등)
    establishedDate: string;
    legalBasis: string;               // 법적 근거
  };

  // 공간 정보
  spatial: {
    totalArea: number;                // 총 면적 (ha)
    coreZoneArea: number;             // 핵심구역 면적 (ha)
    bufferZoneArea: number;           // 완충구역 면적 (ha)
    transitionZoneArea: number;       // 전이구역 면적 (ha)
    boundary: GeoJSON;                // 경계 (GeoJSON Polygon)
    zonation: {
      zoneId: string;
      zoneName: string;               // 구역명
      zoneType: 'core' | 'buffer' | 'transition' | 'restoration';
      area: number;                   // 면적 (ha)
      boundary: GeoJSON;
      allowedActivities: string[];    // 허용 활동
      restrictions: string[];         // 제한 사항
    }[];
  };

  // 생물다양성 가치
  biodiversityValue: {
    endemicSpecies: {
      scientificName: string;
      commonName: string;
      conservationStatus: string;     // IUCN Red List 등급
    }[];
    endangeredSpecies: {
      scientificName: string;
      commonName: string;
      conservationStatus: string;
      populationSize: number;
      populationTrend: 'increasing' | 'stable' | 'decreasing';
    }[];
    keyBiodiversityArea: boolean;     // KBA 여부
    importanceLevel: 'global' | 'regional' | 'national' | 'local';
  };

  // 관리 계획
  managementPlan: {
    planId: string;
    planPeriod: {
      startYear: number;
      endYear: number;
    };
    vision: string;                   // 비전
    objectives: string[];             // 목표
    strategies: string[];             // 전략
    zonationStrategy: string;         // 구역화 전략

    programs: {
      programId: string;
      programName: string;            // 프로그램명 (서식지 관리, 종 보전 등)
      activities: {
        activityName: string;
        schedule: string;             // 일정
        budget: number;               // 예산 (원)
        responsible: string;          // 담당자
      }[];
    }[];
  };

  // 위협 평가
  threats: {
    threatType: string;               // 위협 유형 (밀렵, 서식지 파괴, 오염 등)
    severity: 'low' | 'medium' | 'high' | 'critical';
    scope: 'localized' | 'widespread';
    trend: 'increasing' | 'stable' | 'decreasing';
    description: string;
    mitigationMeasures: string[];     // 완화 조치
  }[];

  // 방문자 관리
  visitorManagement: {
    annualVisitors: number;           // 연간 방문객 수
    maxCapacity: number;              // 최대 수용 인원
    permitRequired: boolean;          // 허가 필요 여부
    entryFee: number;                 // 입장료 (원)

    facilities: {
      facilityType: string;           // 시설 유형 (탐방로, 대피소 등)
      location: { latitude: number; longitude: number; };
      capacity: number;
      condition: 'excellent' | 'good' | 'fair' | 'poor';
    }[];

    interpretationPrograms: {
      programName: string;
      targetAudience: string;         // 대상 (학생, 일반인 등)
      frequency: string;              // 빈도
      participants: number;           // 참가자 수/년
    }[];
  };

  // 집행 및 순찰
  enforcement: {
    patrolRecords: {
      patrolId: string;
      date: string;
      route: GeoJSON;                 // 순찰 경로
      duration: number;               // 순찰 시간 (hours)
      rangers: string[];              // 순찰원
      observations: string[];         // 관찰 사항
      violations?: {
        type: string;                 // 위반 유형
        location: { latitude: number; longitude: number; };
        action: string;               // 조치 사항
      }[];
    }[];

    violations: {
      violationType: string;
      count: number;                  // 발생 건수
      trend: 'increasing' | 'stable' | 'decreasing';
    }[];
  };

  // 관리 효과성 평가
  managementEffectiveness: {
    assessmentFramework: string;      // 평가 체계 (METT, RAPPAM 등)
    assessmentDate: string;

    scores: {
      context: number;                // 맥락 (0-100)
      planning: number;               // 계획 (0-100)
      inputs: number;                 // 투입 (0-100)
      processes: number;              // 과정 (0-100)
      outputs: number;                // 산출 (0-100)
      outcomes: number;               // 성과 (0-100)
    };

    overallScore: number;             // 종합 점수 (0-100)
    overallRating: 'poor' | 'fair' | 'good' | 'very_good' | 'excellent';

    strengths: string[];              // 강점
    weaknesses: string[];             // 약점
    recommendations: string[];        // 개선 권고사항
  };
}
```

### 14.2 IUCN 보호구역 카테고리

| 카테고리 | 명칭 | 주요 목적 | 관리 수준 |
|---------|------|----------|----------|
| Ia | 엄정보호구역 | 과학 연구 | 최소 개입 |
| Ib | 원시자연보호구역 | 원시성 보전 | 최소 개입 |
| II | 국립공원 | 생태계 보전 및 여가 | 중간 |
| III | 천연기념물 | 특정 자연 특성 보전 | 중간 |
| IV | 서식지/종 관리구역 | 서식지 및 종 관리 | 높음 |
| V | 경관보호구역 | 경관 보전 및 여가 | 높음 |
| VI | 자원관리보호구역 | 지속가능한 자연자원 이용 | 높음 |

---

## 15. 성과 지표 (KPI)

### 15.1 생태계 건강도 KPI

| KPI | 목표값 | 측정 단위 | 산정 방식 |
|-----|--------|----------|----------|
| 생태계 건강도 지수 (EHI) | 80 이상 | 점수 (0-100) | 하위 지수 가중평균 |
| 생물다양성 지수 | 75 이상 | 점수 (0-100) | Shannon 지수 기반 |
| 식생 건강도 | NDVI 0.6 이상 | NDVI | 위성 영상 분석 |
| 토양 건강도 | 70 이상 | 점수 (0-100) | 물리화학생물학적 지표 종합 |
| 수질 등급 | 1-2등급 | 등급 | 환경부 수질 기준 |

### 15.2 탄소 KPI

- **탄소 저장량**: 생태계 유형별 기준값 달성
  - 온대림: 150 Mg C/ha 이상
  - 열대림: 300 Mg C/ha 이상
  - 맹그로브: 500 Mg C/ha 이상
  - 갯벌: 100 Mg C/ha 이상

- **탄소 흡수율**: 연간 2 Mg C/ha/year 이상
- **탄소 크레딧**: 연간 1,000 tCO2e 이상 (대규모 프로젝트)

### 15.3 생태계 서비스 KPI

- **수원 함양**: 10,000 m³/ha/year 이상
- **홍수 조절**: 첨두 유량 30% 이상 감소
- **수질 정화**: BOD 50% 이상 저감
- **생물다양성 지원**: 멸종위기종 5종 이상 서식

---

## 16. 통합 및 상호운용성

### 16.1 WIA 생태계 통합

#### 16.1.1 연계 표준

- **WIA-ENE-017 (Climate)**: 기후 데이터 및 탄소 배출량 연계
- **WIA-ENE-003 (Carbon Capture)**: 탄소 포집 기술과 자연 탄소 흡수원 통합
- **WIA-ENE-022 (Waste Management)**: 생태계 기반 폐기물 처리 (퇴비화 등)
- **WIA-ENE-010 (Water Quality)**: 수질 모니터링 데이터 공유
- **WIA-BLOCKCHAIN**: 탄소 크레딧 및 생태계 서비스 거래 추적
- **WIA-SOCIAL**: 시민과학 및 지역 사회 참여 프로그램

#### 16.1.2 API 엔드포인트

```
GET    /api/v1/ecosystems                    # 생태계 목록 조회
GET    /api/v1/ecosystems/{id}                # 생태계 상세 정보
POST   /api/v1/ecosystems                     # 생태계 등록
PUT    /api/v1/ecosystems/{id}                # 생태계 정보 수정

GET    /api/v1/ecosystems/{id}/health         # 건강도 지표
GET    /api/v1/ecosystems/{id}/biodiversity   # 생물다양성 데이터
GET    /api/v1/ecosystems/{id}/carbon         # 탄소 저장/흡수 데이터
GET    /api/v1/ecosystems/{id}/water          # 수문 데이터

POST   /api/v1/monitoring/biodiversity        # 생물다양성 데이터 제출
POST   /api/v1/monitoring/vegetation          # 식생 데이터 제출
POST   /api/v1/monitoring/soil                # 토양 데이터 제출
POST   /api/v1/monitoring/water               # 수질 데이터 제출

POST   /api/v1/satellite/imagery              # 위성 영상 데이터 등록
GET    /api/v1/satellite/timeseries/{id}      # 시계열 위성 데이터

POST   /api/v1/iot/sensor                     # IoT 센서 등록
POST   /api/v1/iot/data                       # 센서 데이터 전송
GET    /api/v1/iot/sensors/{id}/data          # 센서 데이터 조회

POST   /api/v1/restoration/projects           # 복원 프로젝트 등록
GET    /api/v1/restoration/projects/{id}      # 프로젝트 정보
PUT    /api/v1/restoration/projects/{id}      # 프로젝트 업데이트
POST   /api/v1/restoration/monitoring         # 복원 모니터링 데이터

GET    /api/v1/protected-areas                # 보호구역 목록
GET    /api/v1/protected-areas/{id}           # 보호구역 상세 정보
POST   /api/v1/protected-areas/patrols        # 순찰 기록 제출

GET    /api/v1/analytics/kpi/{id}             # KPI 대시보드
GET    /api/v1/analytics/trends/{id}          # 추세 분석
POST   /api/v1/analytics/report               # 보고서 생성
```

### 16.2 데이터 교환 포맷

- **기본 포맷**: JSON (UTF-8), GeoJSON (공간 데이터)
- **대용량 데이터**: NetCDF (기상 데이터), GeoTIFF (위성 영상)
- **실시간 스트림**: MQTT, WebSocket
- **시계열 데이터**: InfluxDB Line Protocol
- **생물다양성 데이터**: Darwin Core, EML (Ecological Metadata Language)

### 16.3 상호운용성 요구사항

- **공간 데이터**: OGC WMS, WFS, WCS 지원
- **메타데이터**: ISO 19115, EML 준수
- **시간 표기**: ISO 8601
- **좌표계**: WGS84 (EPSG:4326) 기본, 국가 좌표계 선택 지원
- **분류체계**: 국제 표준 (IUCN, GBIF) 연계

---

## 17. 보안 및 개인정보보호

### 17.1 데이터 보안

#### 17.1.1 보안 등급

| 데이터 유형 | 보안 등급 | 암호화 | 접근 통제 |
|------------|----------|--------|----------|
| 일반 모니터링 데이터 | Public | 불필요 | 공개 |
| 생태계 공간 정보 | Public | 불필요 | 공개 |
| 멸종위기종 위치 정보 | Highly Confidential | End-to-End | 엄격 제한 |
| 보호구역 순찰 경로 | Confidential | AES-256 | 권한자만 |
| 불법 행위 증거 | Highly Confidential | End-to-End | 사법기관 |

#### 17.1.2 멸종위기종 정보 보호

```typescript
interface EndangeredSpeciesProtection {
  speciesId: string;
  scientificName: string;
  conservationStatus: string;

  // 위치 정보 보호
  locationData: {
    preciseLocation: {                // 정확한 위치 (보호)
      latitude: number;
      longitude: number;
      accessLevel: 'researcher' | 'manager' | 'authority';
      encrypted: boolean;
    };
    generalizedLocation: {            // 일반화된 위치 (공개)
      gridCell: string;               // 10km × 10km 격자
      region: string;                 // 행정구역
      accessLevel: 'public';
    };
  };

  // 데이터 접근 로그
  accessLog: {
    timestamp: string;
    userId: string;
    accessType: 'view' | 'download' | 'export';
    justification: string;
  }[];
}
```

---

## 18. 인증 요구사항

### 18.1 생태계 관리 인증 등급

- **Bronze**: 기본 모니터링 실시 (연 1회 이상)
- **Silver**: 체계적 모니터링 (연 4회 이상), 건강도 B 등급 이상
- **Gold**: 고빈도 모니터링 (월 1회 이상), IoT 센서 활용, 건강도 A 등급
- **Platinum**: 실시간 모니터링, AI 분석, 탄소 크레딧 인증, 국제 기준 충족

### 18.2 인증 프로세스

1. **신청 및 서류 제출** (1주)
2. **기초 자료 검토** (2주)
3. **현장 실사** (3-5일)
4. **데이터 검증 및 분석** (3주)
5. **전문가 심사 위원회** (1주)
6. **인증서 발급**
7. **연간 사후 관리** (연 1회 현장 방문)

### 18.3 모니터링 인력 교육

- **기본 과정**: 생태계 모니터링 기초 (16시간)
- **전문 과정**: 종 식별, 식생 조사, 토양 샘플링 (40시간)
- **기술 과정**: 위성 영상 분석, GIS, 통계 (40시간)
- **관리자 과정**: 생태계 관리, 복원 기획, 보호구역 관리 (40시간)
- **재교육**: 2년마다 갱신 교육 (8시간)

---

## 부록

### A. 생태계 유형별 표준 모니터링 프로토콜

[각 생태계 유형별 세부 조사 방법 및 체크리스트]

### B. 종 식별 가이드

[주요 지표 종 및 멸종위기종 식별 방법]

### C. 위성 영상 분석 튜토리얼

[Google Earth Engine 활용 NDVI 분석 코드]

### D. IoT 센서 설치 가이드

[센서 설치 위치 선정, 캘리브레이션, 유지보수]

### E. 탄소 크레딧 방법론

[VCS, Gold Standard, CDM 방법론 적용 가이드]

### F. 샘플 데이터셋

[실제 활용 가능한 JSON 예시]

### G. API 레퍼런스

[전체 API 엔드포인트 상세 문서]

### H. 용어 사전

[300개 이상의 생태학 용어 정의]

---

## 개정 이력

| 버전 | 날짜 | 주요 변경사항 |
|------|------|--------------|
| 1.0.0 | 2025-12-25 | 초판 발행 |

---

## 라이선스

© 2025 WIA (World Certification Industry Association)
이 표준은 Creative Commons Attribution 4.0 International License 하에 배포됩니다.

---

## 연락처

**WIA 표준 사무국**
웹사이트: https://wiastandards.com
이메일: standards@wia-official.org
GitHub: https://github.com/WIA-Official/wia-standards

---

**弘益人間 (홍익인간) · 널리 인간을 이롭게 하라**

생태계 모니터링은 지구의 건강을 지키고, 생물다양성을 보전하며, 기후 변화에 대응하는 핵심 활동입니다. WIA-ENE-031 표준은 과학적 방법론과 첨단 기술을 결합하여 전 세계 생태계의 체계적 관리를 지원하고, 인류와 자연이 조화롭게 공존하는 미래를 만듭니다.

**Together, we protect nature for the benefit of all humanity.**

---
