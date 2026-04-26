# WIA-ENE-030: 생물다양성 지수 표준 v1.0 🦋

> **弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

**표준 ID:** WIA-ENE-030
**버전:** 1.0.0
**발행일:** 2025-12-25
**상태:** 활성
**카테고리:** 에너지/환경 (ENE)

---

## 목차

1. [개요](#1-개요)
2. [적용 범위](#2-적용-범위)
3. [용어 정의](#3-용어-정의)
4. [생물다양성 측정 체계](#4-생물다양성-측정-체계)
5. [데이터 모델](#5-데이터-모델)
6. [조사 방법론](#6-조사-방법론)
7. [다양성 지수 계산](#7-다양성-지수-계산)
8. [서식지 평가](#8-서식지-평가)
9. [모니터링 및 보고](#9-모니터링-및-보고)
10. [성과 지표 (KPI)](#10-성과-지표-kpi)
11. [통합 및 상호운용성](#11-통합-및-상호운용성)
12. [보안 및 개인정보보호](#12-보안-및-개인정보보호)
13. [인증 요구사항](#13-인증-요구사항)

---

## 1. 개요

### 1.1 목적

WIA-ENE-030 생물다양성 지수 표준은 생태계의 종 다양성, 서식지 건강도, 생물학적 풍부도를 체계적으로 측정하고 모니터링하기 위한 국제 표준입니다. 본 표준은 생물다양성 보전, 생태계 복원, 지속가능한 환경 관리를 목표로 합니다.

### 1.2 핵심 원칙

- **과학적 정확성 (Scientific Accuracy)**: 검증된 생태학적 방법론 적용
- **포괄적 조사 (Comprehensive Survey)**: 동식물, 미생물 포함 전체 생태계 관찰
- **장기 모니터링 (Long-term Monitoring)**: 시간에 따른 변화 추적
- **시민 과학 (Citizen Science)**: 일반 시민 참여를 통한 데이터 수집
- **기술 혁신 (Technology Innovation)**: DNA 바코딩, eDNA, AI 기반 종 식별
- **투명성 (Transparency)**: 모든 데이터 공개 및 공유
- **보전 중심 (Conservation Focus)**: 멸종위기종 및 고유종 보호

### 1.3 적용 대상

- 국립공원 및 자연보호구역
- 해양보호구역 및 습지
- 산림청 및 환경 관리 기관
- 생태 연구소 및 대학
- 환경영향평가 기관
- NGO 및 보전단체
- 스마트시티 환경 모니터링 플랫폼
- 국제 생물다양성 협약 당사국

---

## 2. 적용 범위

### 2.1 생태계 유형

본 표준은 다음 생태계 유형에 적용됩니다:

- **육상 생태계**: 산림, 초원, 사막, 툰드라
- **담수 생태계**: 하천, 호수, 습지, 지하수
- **해양 생태계**: 연안, 대양, 산호초, 심해
- **도시 생태계**: 공원, 가로수, 옥상 정원
- **농업 생태계**: 논, 밭, 과수원, 목초지

### 2.2 생물 분류군

- **식물 (Flora)**: 관속식물, 이끼, 조류
- **동물 (Fauna)**: 포유류, 조류, 파충류, 양서류, 어류, 무척추동물
- **미생물 (Microorganisms)**: 박테리아, 고세균, 진균, 원생생물
- **곤충 (Insects)**: 나비, 벌, 딱정벌레 등 수분매개자 및 지표종

### 2.3 측정 범위

- 종 풍부도 (Species Richness)
- 종 균등도 (Species Evenness)
- 기능적 다양성 (Functional Diversity)
- 유전적 다양성 (Genetic Diversity)
- 생태계 다양성 (Ecosystem Diversity)

---

## 3. 용어 정의

### 3.1 기본 용어

- **생물다양성 (Biodiversity)**: 모든 생명체의 다양성 (종, 유전자, 생태계)
- **종 풍부도 (Species Richness)**: 특정 지역에 서식하는 종의 수
- **고유종 (Endemic Species)**: 특정 지역에만 서식하는 종
- **멸종위기종 (Endangered Species)**: IUCN 적색목록에 등재된 위협받는 종
- **지표종 (Indicator Species)**: 생태계 건강 상태를 나타내는 종
- **기능군 (Functional Group)**: 생태계에서 유사한 역할을 하는 종의 집합

### 3.2 기술 용어

- **Shannon 지수 (Shannon Index)**: 종 다양성을 나타내는 엔트로피 기반 지수
- **Simpson 지수 (Simpson Index)**: 두 개체가 같은 종일 확률
- **DNA 바코딩 (DNA Barcoding)**: 유전자 서열을 이용한 종 식별
- **eDNA (environmental DNA)**: 환경 샘플에서 추출한 DNA
- **Transect**: 생태 조사를 위한 일정한 경로
- **Quadrat**: 정량 조사를 위한 구획

---

## 4. 생물다양성 측정 체계

### 4.1 다양성 수준 분류

```
생물다양성
├── 유전적 다양성 (Genetic Diversity)
│   ├── 개체군 내 변이
│   └── 개체군 간 변이
├── 종 다양성 (Species Diversity)
│   ├── 종 풍부도 (Richness)
│   ├── 종 균등도 (Evenness)
│   └── 종 고유성 (Endemism)
└── 생태계 다양성 (Ecosystem Diversity)
    ├── 서식지 유형
    ├── 군집 구조
    └── 생태적 과정
```

### 4.2 IUCN 적색목록 범주

| 코드 | 범주 | 설명 | 보전 우선순위 |
|------|------|------|--------------|
| EX | 절멸 (Extinct) | 마지막 개체 사망 | N/A |
| EW | 야생절멸 (Extinct in the Wild) | 사육 개체만 존재 | 매우 높음 |
| CR | 위급 (Critically Endangered) | 극도로 높은 절멸 위험 | 매우 높음 |
| EN | 위기 (Endangered) | 높은 절멸 위험 | 높음 |
| VU | 취약 (Vulnerable) | 절멸 위험 증가 | 중간 |
| NT | 준위협 (Near Threatened) | 가까운 미래 위협 | 중간 |
| LC | 관심대상 (Least Concern) | 낮은 절멸 위험 | 낮음 |
| DD | 정보부족 (Data Deficient) | 평가 자료 불충분 | 조사 필요 |

### 4.3 서식지 품질 등급

- **Grade A - 우수**: 원시림, 미개발 생태계
- **Grade B - 양호**: 일부 교란, 복원 가능
- **Grade C - 보통**: 중간 교란, 관리 필요
- **Grade D - 불량**: 심각한 훼손, 복원 시급
- **Grade E - 파괴**: 생태계 기능 상실

---

## 5. 데이터 모델

### 5.1 종 관찰 정보

```typescript
interface SpeciesObservation {
  // 기본 정보
  observationId: string;              // 고유 식별자
  timestamp: string;                  // ISO 8601 형식
  observerId: string;                 // 관찰자 ID

  // 위치 정보
  location: {
    siteName: string;                 // 조사 지점명
    coordinates: {
      latitude: number;               // 위도
      longitude: number;              // 경도
      altitude?: number;              // 고도 (m)
    };
    habitat: HabitatType;             // 서식지 유형
    ecosystem: EcosystemType;         // 생태계 유형
  };

  // 종 정보
  species: {
    scientificName: string;           // 학명
    commonName: string;               // 일반명
    taxonomicRank: TaxonomicRank;     // 분류 계급
    taxonomy: {
      kingdom: string;                // 계
      phylum: string;                 // 문
      class: string;                  // 강
      order: string;                  // 목
      family: string;                 // 과
      genus: string;                  // 속
      species: string;                // 종
    };
    iucnStatus?: IUCNCategory;        // IUCN 적색목록 범주
    isEndemic: boolean;               // 고유종 여부
  };

  // 개체 정보
  individual: {
    count: number;                    // 개체수
    lifeStage?: string;               // 생활사 단계
    sex?: 'male' | 'female' | 'unknown';
    behavior?: string;                // 행동
    reproductiveStatus?: string;      // 번식 상태
  };

  // 관찰 방법
  method: {
    surveyType: SurveyType;           // 조사 방법
    samplingEffort: number;           // 조사 노력 (시간/인원)
    detectionMethod: string;          // 탐지 방법
    identificationMethod: string;     // 식별 방법
  };

  // 증거 자료
  evidence: {
    photos?: string[];                // 사진 URL
    audio?: string[];                 // 음성 기록 URL
    dnaSequence?: string;             // DNA 서열 (바코드)
    voucherSpecimen?: string;         // 표본 ID
  };

  // 메타데이터
  metadata: {
    dataQuality: number;              // 데이터 품질 점수 (0-100)
    verified: boolean;                // 전문가 검증 여부
    verifiedBy?: string;              // 검증자
    verificationDate?: string;        // 검증 일자
  };
}
```

### 5.2 생물다양성 조사 정보

```typescript
interface BiodiversitySurvey {
  surveyId: string;                   // 조사 ID
  projectName: string;                // 프로젝트명

  // 조사 기간
  period: {
    startDate: string;                // 시작일
    endDate: string;                  // 종료일
    season: string;                   // 계절
  };

  // 조사 지역
  area: {
    siteName: string;                 // 조사지명
    boundary: {                       // 경계 (GeoJSON Polygon)
      type: 'Polygon';
      coordinates: number[][][];
    };
    areaSize: number;                 // 면적 (ha)
    protectionStatus: string;         // 보호구역 여부
  };

  // 조사팀
  team: {
    leader: string;                   // 조사 책임자
    members: string[];                // 조사원
    volunteers?: string[];            // 자원봉사자
    organization: string;             // 소속 기관
  };

  // 조사 대상
  targets: {
    taxonomicGroups: string[];        // 분류군
    focusSpecies?: string[];          // 중점 조사 종
    habitatTypes: HabitatType[];      // 서식지 유형
  };

  // 조사 결과
  results: {
    totalSpecies: number;             // 총 종수
    totalIndividuals: number;         // 총 개체수
    endemicSpecies: number;           // 고유종 수
    endangeredSpecies: number;        // 멸종위기종 수
    observations: SpeciesObservation[];  // 관찰 기록
  };
}
```

### 5.3 다양성 지수 정보

```typescript
interface DiversityIndices {
  indexId: string;                    // 지수 ID
  surveyId: string;                   // 조사 ID 참조
  calculationDate: string;            // 계산 일자

  // 기본 지수
  speciesRichness: number;            // 종 풍부도 (S)
  totalIndividuals: number;           // 총 개체수 (N)

  // 다양성 지수
  shannonIndex: number;               // Shannon 지수 (H')
  shannonEquitability: number;        // Shannon 균등도 (J')
  simpsonIndex: number;               // Simpson 지수 (D)
  simpsonDiversity: number;           // Simpson 다양성 (1-D)
  bergerParkerIndex: number;          // Berger-Parker 우점도

  // 추가 지수
  margalefRichness?: number;          // Margalef 풍부도
  pielouEquitability?: number;        // Pielou 균등도
  fisherAlpha?: number;               // Fisher's Alpha

  // 종 구성
  speciesComposition: {
    scientificName: string;
    count: number;                    // 개체수
    frequency: number;                // 출현 빈도
    relativeAbundance: number;        // 상대 풍부도 (%)
    dominance: number;                // 우점도
  }[];

  // 통계
  statistics: {
    rareSpecies: number;              // 희귀종 수 (n=1)
    commonSpecies: number;            // 일반종 수
    dominantSpecies: string[];        // 우점종
    rarefactionCurve?: {              // Rarefaction 곡선
      sampleSize: number[];
      expectedSpecies: number[];
    };
  };
}
```

### 5.4 서식지 평가 정보

```typescript
interface HabitatAssessment {
  assessmentId: string;               // 평가 ID
  siteName: string;                   // 조사지명
  assessmentDate: string;             // 평가 일자

  // 물리적 특성
  physical: {
    habitatType: HabitatType;         // 서식지 유형
    areaSize: number;                 // 면적 (ha)
    elevation: number;                // 고도 (m)
    slope: number;                    // 경사 (도)
    aspect: string;                   // 사면 방향
    soilType?: string;                // 토양 유형
  };

  // 환경 조건
  environmental: {
    temperature: {
      mean: number;                   // 평균 기온 (°C)
      min: number;                    // 최저 기온
      max: number;                    // 최고 기온
    };
    precipitation: number;            // 강수량 (mm/년)
    humidity: number;                 // 습도 (%)
    lightIntensity?: number;          // 광도 (lux)
    waterQuality?: {                  // 수질 (습지/하천)
      pH: number;
      dissolvedOxygen: number;        // 용존산소 (mg/L)
      turbidity: number;              // 탁도 (NTU)
    };
  };

  // 식생 구조
  vegetation: {
    canopyCover: number;              // 수관 피복도 (%)
    treeHeight: number;               // 평균 수고 (m)
    understory: string;               // 하층 식생
    invasiveSpecies: string[];        // 침입 외래종
  };

  // 교란 요인
  disturbances: {
    type: string;                     // 교란 유형
    severity: 'low' | 'medium' | 'high';
    frequency: string;                // 빈도
    lastOccurrence?: string;          // 최근 발생일
  }[];

  // 위협 요인
  threats: {
    threat: string;                   // 위협 요인
    impact: 'low' | 'medium' | 'high' | 'critical';
    scope: number;                    // 영향 범위 (%)
    timeframe: string;                // 시간대
    mitigation?: string;              // 완화 조치
  }[];

  // 품질 등급
  quality: {
    overallGrade: 'A' | 'B' | 'C' | 'D' | 'E';
    integrityScore: number;           // 생태적 건전성 (0-100)
    conditionScore: number;           // 상태 점수 (0-100)
    managementNeeds: string[];        // 관리 필요사항
  };
}
```

---

## 6. 조사 방법론

### 6.1 표준 조사 방법

#### 6.1.1 Line Transect Method (선조사법)

- 일정한 경로를 따라 이동하며 관찰
- 적용: 조류, 포유류, 식물
- 표준 길이: 500m ~ 2km
- 관찰 폭: 양쪽 각 25m

#### 6.1.2 Quadrat Sampling (방형구법)

- 정해진 크기의 구획 내 모든 종 조사
- 적용: 식물, 저서생물
- 표준 크기: 1m × 1m (초본), 10m × 10m (교목)
- 반복: 최소 30개 방형구

#### 6.1.3 Point Count (정점 조사법)

- 고정된 지점에서 일정 시간 관찰
- 적용: 조류
- 관찰 시간: 5-10분/지점
- 관찰 반경: 50m

#### 6.1.4 Camera Trapping (무인카메라)

- 자동 촬영 장치를 이용한 관찰
- 적용: 포유류, 중대형 동물
- 설치 기간: 최소 30일
- 카메라 밀도: 1대/km²

#### 6.1.5 Acoustic Monitoring (음향 모니터링)

- 소리를 이용한 종 탐지
- 적용: 조류, 양서류, 박쥐, 고래
- 녹음 시간: 24시간 연속 또는 정시 샘플링
- 분석: 음성 패턴 인식 AI

### 6.2 분자생물학적 방법

#### 6.2.1 DNA 바코딩

```typescript
interface DNABarcoding {
  sampleId: string;
  geneRegion: 'COI' | 'rbcL' | 'ITS' | '16S' | '18S';
  sequence: string;                   // DNA 서열
  sequenceLength: number;             // 염기쌍 수
  quality: number;                    // 시퀀싱 품질
  matchedSpecies: {
    scientificName: string;
    similarity: number;               // 유사도 (%)
    database: string;                 // BOLD, GenBank 등
  }[];
}
```

#### 6.2.2 eDNA (Environmental DNA) 샘플링

- **수질 샘플**: 물 1-2L 채수 후 여과
- **토양 샘플**: 표토 10cm, 10g 채취
- **대기 샘플**: 공기 여과
- **분석**: 메타바코딩 (Metabarcoding)
- **장점**: 비침습적, 광범위 종 탐지

### 6.3 시민 과학 (Citizen Science)

#### 6.3.1 관찰 플랫폼

- iNaturalist
- eBird
- 국가 생물종 정보 시스템
- WIA Biodiversity App

#### 6.3.2 데이터 품질 관리

- 사진 증거 필수
- 전문가 검증 시스템
- 위치 정확도 확인
- 시간 타임스탬프 검증

---

## 7. 다양성 지수 계산

### 7.1 Shannon 다양성 지수 (H')

**공식:**
```
H' = -Σ(pi × ln(pi))
```

여기서:
- pi = 종 i의 상대 풍부도 (ni/N)
- ni = 종 i의 개체수
- N = 전체 개체수

**해석:**
- H' = 0: 단일 종만 존재 (최소 다양성)
- H' > 3: 매우 높은 다양성
- 일반적 범위: 1.5 ~ 3.5

### 7.2 Shannon 균등도 (J')

**공식:**
```
J' = H' / H'max = H' / ln(S)
```

여기서:
- S = 종 풍부도 (총 종수)
- H'max = 최대 다양성 (모든 종이 균등할 때)

**해석:**
- J' = 1: 완벽한 균등 분포
- J' < 0.5: 일부 종이 우점
- J' > 0.8: 높은 균등도

### 7.3 Simpson 지수 (D)

**공식:**
```
D = Σ(pi²)
```

**Simpson 다양성 (1-D):**
```
1 - D = 1 - Σ(pi²)
```

**해석:**
- D = 1: 완전 우점 (단일 종)
- D → 0: 높은 다양성
- 1-D: 값이 클수록 다양성 높음

### 7.4 Margalef 풍부도 (DMg)

**공식:**
```
DMg = (S - 1) / ln(N)
```

**해석:**
- 종수를 개체수로 표준화
- 값이 클수록 풍부도 높음

### 7.5 실제 계산 예시

```typescript
// 조사 데이터
const observations = [
  { species: 'A', count: 50 },
  { species: 'B', count: 30 },
  { species: 'C', count: 15 },
  { species: 'D', count: 5 }
];

const N = 100;  // 총 개체수
const S = 4;    // 총 종수

// Shannon Index
const H = -1 * observations.reduce((sum, obs) => {
  const pi = obs.count / N;
  return sum + (pi * Math.log(pi));
}, 0);
// H' ≈ 1.18

// Shannon Equitability
const J = H / Math.log(S);
// J' ≈ 0.85

// Simpson Index
const D = observations.reduce((sum, obs) => {
  const pi = obs.count / N;
  return sum + (pi * pi);
}, 0);
// D ≈ 0.36
// 1-D ≈ 0.64
```

---

## 8. 서식지 평가

### 8.1 서식지 적합성 지수 (HSI)

#### 8.1.1 구성 요소

| 요소 | 가중치 | 평가 항목 |
|------|--------|----------|
| 식생 구조 | 30% | 수관 피복, 하층 식생, 종 다양성 |
| 수자원 | 20% | 수질, 수량, 접근성 |
| 먹이 자원 | 20% | 먹이 식물, 먹이 동물, 가용성 |
| 은신처 | 15% | 은폐 장소, 번식지, 안전성 |
| 연결성 | 15% | 서식지 연결, 이동 통로, 단편화 |

#### 8.1.2 HSI 계산

```
HSI = Σ(요소 점수 × 가중치)
```

**등급:**
- HSI > 0.8: 최적 서식지
- 0.6 < HSI ≤ 0.8: 양호한 서식지
- 0.4 < HSI ≤ 0.6: 보통 서식지
- 0.2 < HSI ≤ 0.4: 불량한 서식지
- HSI ≤ 0.2: 부적합 서식지

### 8.2 생태계 서비스 평가

- **공급 서비스**: 식량, 약용 식물, 목재
- **조절 서비스**: 수분, 해충 조절, 탄소 저장
- **문화 서비스**: 관광, 교육, 정신적 가치
- **지지 서비스**: 토양 형성, 영양 순환, 1차 생산

---

## 9. 모니터링 및 보고

### 9.1 장기 모니터링 프로그램

#### 9.1.1 모니터링 주기

| 분류군 | 조사 주기 | 조사 계절 | 반복 횟수 |
|--------|----------|----------|----------|
| 포유류 | 연 1-2회 | 봄, 가을 | 2회 이상 |
| 조류 | 연 4회 | 4계절 | 계절별 3회 |
| 양서파충류 | 연 2회 | 봄, 여름 | 2회 이상 |
| 곤충 | 연 3회 | 봄, 여름, 가을 | 3회 이상 |
| 식물 | 연 2회 | 봄, 여름 | 2회 이상 |
| eDNA | 연 4회 | 4계절 | 계절별 1회 |

#### 9.1.2 고정 조사구 설치

- 영구 조사구 설정
- GPS 좌표 기록
- 사진 기록점 표시
- 최소 5년 이상 장기 모니터링

### 9.2 데이터 표준화

#### 9.2.1 Darwin Core 준수

- occurrenceID: 관찰 고유 ID
- basisOfRecord: 관찰 근거
- scientificName: 학명
- eventDate: 관찰 일시
- decimalLatitude, decimalLongitude: 좌표
- coordinateUncertaintyInMeters: 위치 정확도

#### 9.2.2 메타데이터

```json
{
  "datasetId": "BIO-2025-001",
  "datasetName": "한라산 생물다양성 조사",
  "license": "CC BY 4.0",
  "rightsHolder": "한라산국립공원관리소",
  "contact": {
    "name": "김생태",
    "email": "ecology@hallasan.go.kr"
  },
  "geographicCoverage": {
    "boundingBox": {
      "north": 33.400,
      "south": 33.300,
      "east": 126.600,
      "west": 126.400
    }
  },
  "taxonomicCoverage": ["Animalia", "Plantae", "Fungi"],
  "temporalCoverage": "2025-01-01/2025-12-31"
}
```

### 9.3 보고 체계

#### 9.3.1 정기 보고

- **분기 보고**: 조사 진행 현황, 주요 발견
- **연간 보고**: 종 목록, 다양성 지수, 변화 추세
- **5개년 종합**: 장기 트렌드, 보전 상태 평가

#### 9.3.2 특별 보고

- 희귀종 발견
- 외래종 침입
- 대량 폐사 사건
- 서식지 급격한 변화

---

## 10. 성과 지표 (KPI)

### 10.1 생물다양성 KPI

| KPI | 목표값 | 측정 단위 | 산정 방식 |
|-----|--------|----------|----------|
| 종 풍부도 | 기준선 대비 유지 | 종수 | 조사 지역 총 종수 |
| Shannon 지수 | H' > 2.5 | 무차원 | -Σ(pi × ln(pi)) |
| 멸종위기종 개체수 | 전년 대비 10% 증가 | 개체 | 개체군 조사 |
| 고유종 비율 | 20% 이상 | % | (고유종 수 / 총 종수) × 100 |
| 서식지 품질 | Grade B 이상 | 등급 | HSI 평가 |
| 외래종 침입률 | 5% 미만 | % | (외래종 수 / 총 종수) × 100 |

### 10.2 모니터링 성과 지표

- **조사 완료율**: 95% 이상 (계획 대비)
- **데이터 품질**: 평균 80점 이상
- **전문가 검증률**: 80% 이상
- **시민 과학 참여**: 연간 1,000건 이상

### 10.3 보전 성과 지표

- **서식지 복원 면적**: 연간 10ha 이상
- **보호구역 확대**: 전년 대비 5% 증가
- **멸종위기종 복원**: 3종 이상 개체군 증가
- **생태 통로 조성**: 연간 5개소 이상

---

## 11. 통합 및 상호운용성

### 11.1 WIA 생태계 통합

#### 11.1.1 연계 표준

- **WIA-ENE-001 (Climate)**: 기후 변화가 생물다양성에 미치는 영향 분석
- **WIA-ENE-004 (Ocean Plastic)**: 해양 생물다양성 위협 요인 연계
- **WIA-ENE-029 (Water Quality)**: 수생 생물다양성 서식지 평가
- **WIA-SOCIAL**: 생물다양성 교육 및 시민 참여
- **WIA-BLOCKCHAIN**: 생물다양성 데이터 무결성 보장

#### 11.1.2 API 엔드포인트

```
POST   /api/v1/observation/create       # 종 관찰 등록
GET    /api/v1/observation/{id}         # 관찰 정보 조회
POST   /api/v1/survey/create            # 조사 프로젝트 생성
GET    /api/v1/survey/{id}/results      # 조사 결과 조회
POST   /api/v1/diversity/calculate      # 다양성 지수 계산
GET    /api/v1/habitat/assess           # 서식지 평가
GET    /api/v1/species/endangered       # 멸종위기종 목록
GET    /api/v1/species/endemic          # 고유종 목록
POST   /api/v1/edna/analyze             # eDNA 분석 요청
GET    /api/v1/analytics/trends         # 장기 트렌드 분석
```

### 11.2 국제 표준 및 데이터베이스 연동

#### 11.2.1 분류학 데이터베이스

- **GBIF (Global Biodiversity Information Facility)**: 전 세계 생물다양성 데이터
- **IUCN Red List**: 멸종위기종 정보
- **BOLD Systems**: DNA 바코드 데이터베이스
- **Catalogue of Life**: 세계 종 목록
- **국가생물종목록**: 한국 생물 종 정보

#### 11.2.2 데이터 교환 포맷

- **Darwin Core**: 생물다양성 데이터 표준
- **EML (Ecological Metadata Language)**: 생태 메타데이터
- **GeoJSON**: 공간 데이터
- **FASTA**: DNA 서열 데이터
- **JSON-LD**: 연결 데이터

### 11.3 상호운용성 요구사항

- Darwin Core 표준 준수
- ISO 19115 (지리정보 메타데이터) 준수
- ISO 8601 (날짜 및 시간) 준수
- OAuth 2.0 (인증) 지원
- RESTful API 설계 원칙 준수
- OpenAPI 3.0 스펙 문서화

---

## 12. 보안 및 개인정보보호

### 12.1 민감 종 정보 보호

#### 12.1.1 보안 등급

| 종 범주 | 공개 수준 | 위치 정밀도 | 접근 권한 |
|---------|----------|------------|----------|
| 일반종 | 완전 공개 | 10m 이내 | 전체 공개 |
| 희귀종 | 일부 공개 | 1km 격자 | 등록 사용자 |
| 멸종위기종 | 제한 공개 | 10km 격자 | 연구자만 |
| 밀렵 대상종 | 비공개 | 비공개 | 관리 기관만 |

#### 12.1.2 위치 정보 흐림 처리

- 멸종위기종: 좌표를 10km × 10km 격자 중심점으로 변환
- 희귀 식물: 좌표를 1km × 1km 격자 중심점으로 변환
- 맹금류 둥지: 위치 정보 완전 삭제

### 12.2 데이터 보안

- **전송 중 암호화**: TLS 1.3 이상
- **저장 시 암호화**: AES-256-GCM
- **접근 로그**: 모든 API 호출 기록
- **백업**: 일일 백업 및 90일 보관
- **감사 추적**: 데이터 변경 이력 추적

### 12.3 연구 윤리

- **동물 복지**: 최소 침습 조사 방법 사용
- **표본 채집**: 필요 최소한으로 제한
- **서식지 보호**: 조사로 인한 서식지 훼손 금지
- **지역 사회 동의**: 원주민 지역 조사 시 사전 동의

---

## 13. 인증 요구사항

### 13.1 조사자 인증

#### 13.1.1 자격 요건

- **전문가 (Expert)**: 생물학 석사 이상 또는 10년 경력
- **조사원 (Surveyor)**: 생물학 학사 또는 5년 경력
- **자원봉사자 (Volunteer)**: 기본 교육 이수

#### 13.1.2 교육 과정

- **기본 과정**: 생물다양성 개론, 조사 방법 (16시간)
- **분류군별 전문**: 식물, 조류, 곤충 등 (24시간)
- **기술 교육**: DNA 바코딩, eDNA, AI 식별 (16시간)
- **재교육**: 2년마다 갱신 (8시간)

### 13.2 데이터 품질 인증

#### 13.2.1 품질 등급

- **Gold**: 전문가 검증 + 표본/사진 증거
- **Silver**: 전문가 검증 또는 사진 증거
- **Bronze**: 관찰 기록만 (검증 필요)

#### 13.2.2 검증 프로세스

1. 자동 검증: 위치, 날짜, 종명 유효성 확인
2. AI 검증: 사진 자동 식별
3. 전문가 검증: 희귀종 및 의심 기록
4. 커뮤니티 검증: 시민 과학자 상호 검토

### 13.3 생태 복원 프로젝트 인증

- **Bronze**: 생물다양성 기초 조사 완료
- **Silver**: 5년 모니터링 + 종 다양성 10% 증가
- **Gold**: 10년 모니터링 + 멸종위기종 복원 성공
- **Platinum**: 완전 생태계 복원 + 자립 가능

---

## 부록

### A. 주요 생물 분류군 조사 가이드

[50개 이상 분류군별 세부 조사 방법]

### B. DNA 바코딩 프로토콜

[실험실 프로토콜 및 분석 가이드]

### C. eDNA 샘플링 표준 절차

[샘플 채취부터 분석까지 전 과정]

### D. 다양성 지수 계산 코드

[R, Python, TypeScript 예제 코드]

### E. IUCN 적색목록 평가 기준

[범주별 상세 기준 및 평가 방법]

### F. 서식지 유형 분류 체계

[200개 이상 서식지 유형 정의]

### G. 용어 사전

[500개 이상 생태학 전문 용어 정의]

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

생물다양성은 지구 생명의 근간입니다. WIA-ENE-030 표준은 과학적 방법론과 최신 기술을 통해 생물다양성을 체계적으로 모니터링하고, 멸종위기에 처한 종들을 보호하며, 미래 세대를 위해 건강한 생태계를 보전하는 데 기여합니다.

**Together, we protect the web of life for all humanity.**

---
