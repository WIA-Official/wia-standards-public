# WIA-ENE-030: 생물다양성 지수 표준 🦋

> **홍익인간 (弘益人間) (홍익인간) - 널리 인간을 이롭게 하라**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/Version-1.0-blue.svg)](https://github.com/WIA-Official/wia-standards)
[![Standard](https://img.shields.io/badge/Standard-WIA--ENE--030-red.svg)](https://wia.org/standards/ene-030)

## 개요

WIA-ENE-030 생물다양성 지수 표준은 생태계의 종 다양성, 서식지 건강도, 생물학적 풍부도를 체계적으로 측정하고 모니터링하기 위한 국제 표준입니다. 본 표준은 생물다양성 보전, 생태계 복원, 지속가능한 환경 관리를 목표로 합니다.

### 주요 기능

- 🌿 **종 다양성 측정**: Shannon, Simpson 등 표준 다양성 지수 계산
- 🔬 **분자생물학 지원**: DNA 바코딩, eDNA 샘플링
- 📊 **장기 모니터링**: 시계열 데이터 분석 및 트렌드 추적
- 🌐 **시민 과학**: 일반 시민 참여를 통한 대규모 데이터 수집
- 🦅 **멸종위기종 관리**: IUCN 적색목록 기반 보전 상태 평가
- 🏞️ **서식지 평가**: 생태계 건강도 및 품질 평가
- 📱 **개방형 표준**: 전 세계 생물다양성 데이터베이스 상호운용성 보장

## 빠른 시작

### 1. 설치

```bash
# 저장소 클론
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/biodiversity-index

# 의존성 설치
./install.sh
```

### 2. TypeScript SDK 사용

```typescript
import { BiodiversitySDK } from '@wia/ene-030';

const sdk = new BiodiversitySDK({
  apiKey: 'your-api-key',
  endpoint: 'https://api.wia.org/ene-030/v1'
});

// 종 관찰 기록
const observation = await sdk.createObservation({
  location: {
    siteName: '한라산 국립공원',
    coordinates: { latitude: 33.3616, longitude: 126.5292, altitude: 1200 },
    habitat: HabitatType.MONTANE_FOREST,
    ecosystem: EcosystemType.TERRESTRIAL
  },
  species: {
    scientificName: 'Parus major',
    commonName: '박새',
    taxonomicRank: TaxonomicRank.SPECIES,
    taxonomy: {
      kingdom: 'Animalia',
      phylum: 'Chordata',
      class: 'Aves',
      order: 'Passeriformes',
      family: 'Paridae',
      genus: 'Parus',
      species: 'major'
    },
    iucnStatus: IUCNCategory.LC,
    isEndemic: false
  },
  individual: {
    count: 3,
    lifeStage: 'adult',
    sex: 'unknown',
    behavior: '먹이 활동'
  },
  method: {
    surveyType: SurveyType.POINT_COUNT,
    samplingEffort: 10,
    detectionMethod: '시각 및 청각',
    identificationMethod: '야외 관찰'
  }
});

console.log('관찰 ID:', observation.observationId);

// 다양성 지수 계산
const diversity = await sdk.calculateDiversity({
  surveyId: 'SURVEY-2025-001',
  observations: [
    { species: 'Parus major', count: 50 },
    { species: 'Cyanistes caeruleus', count: 30 },
    { species: 'Dendrocopos major', count: 15 },
    { species: 'Sitta europaea', count: 5 }
  ]
});

console.log('Shannon 지수:', diversity.shannonIndex);
console.log('Simpson 다양성:', diversity.simpsonDiversity);
console.log('종 풍부도:', diversity.speciesRichness);
```

### 3. CLI 도구 사용

```bash
# 종 관찰 등록
./cli/biodiversity-index.sh create-observation \
  --species "Parus major" \
  --common-name "박새" \
  --count 3 \
  --location "한라산 국립공원"

# 조사 프로젝트 생성
./cli/biodiversity-index.sh create-survey \
  --name "한라산 생물다양성 조사 2025" \
  --site "한라산 국립공원" \
  --start-date 2025-01-01 \
  --end-date 2025-12-31

# 다양성 지수 계산
./cli/biodiversity-index.sh calculate-diversity \
  --survey-id SURVEY-2025-001

# 멸종위기종 조회
./cli/biodiversity-index.sh list-endangered \
  --region "제주도"

# 서식지 평가
./cli/biodiversity-index.sh assess-habitat \
  --site "한라산 국립공원" \
  --habitat-type MONTANE_FOREST
```

### 4. 상세 사양 확인

- **스펙 문서**: [`spec/WIA-ENE-030-v1.0.md`](spec/WIA-ENE-030-v1.0.md)

## 저장소 구조

```
biodiversity-index/
├── README.md                  # 본 문서
├── install.sh                 # 설치 스크립트
├── spec/
│   └── WIA-ENE-030-v1.0.md   # 상세 스펙
├── api/
│   └── typescript/
│       ├── src/
│       │   ├── types.ts       # 타입 정의
│       │   └── index.ts       # SDK 구현
│       └── package.json       # npm 패키지 설정
└── cli/
    └── biodiversity-index.sh  # CLI 도구
```

## 기술 범위

### 생물 분류군

| 분류군 | 조사 방법 | 주요 지표 |
|--------|----------|----------|
| 포유류 | 무인카메라, 흔적 조사 | 개체수, 활동 패턴 |
| 조류 | 정점 조사, 선조사 | 종 다양성, 번식 성공률 |
| 양서·파충류 | 야간 조사, 시각/청각 | 개체군 크기, 번식지 |
| 곤충 | 채집, 말레이즈 트랩 | 종 풍부도, 수분매개자 |
| 식물 | 방형구법, 식생 조사 | 피도, 종 조성 |
| 어류 | 투망, eDNA | 종 다양성, 생체량 |
| 미생물 | eDNA, 메타게노믹스 | 종 풍부도, 기능 다양성 |

### 다양성 지수

```typescript
interface DiversityIndices {
  // 기본 지수
  speciesRichness: number;      // 종 풍부도 (S)
  totalIndividuals: number;     // 총 개체수 (N)

  // 다양성 지수
  shannonIndex: number;         // Shannon H' = -Σ(pi × ln(pi))
  shannonEquitability: number;  // Shannon J' = H' / ln(S)
  simpsonIndex: number;         // Simpson D = Σ(pi²)
  simpsonDiversity: number;     // Simpson 1-D
  bergerParkerIndex: number;    // 최우점종 비율

  // 풍부도 지수
  margalefRichness: number;     // (S-1) / ln(N)
  fisherAlpha: number;          // Fisher's Alpha

  // 균등도 지수
  pielouEquitability: number;   // Pielou's J
}
```

### 조사 방법론

#### 1. 전통적 방법
- **Line Transect**: 일정 경로 따라 관찰
- **Quadrat Sampling**: 방형구 내 모든 종 조사
- **Point Count**: 고정 지점에서 시간 기반 관찰
- **Camera Trapping**: 무인카메라 자동 촬영

#### 2. 분자생물학적 방법
- **DNA Barcoding**: 유전자 서열 기반 종 식별
- **eDNA**: 환경 샘플에서 DNA 추출 및 분석
- **Metabarcoding**: 혼합 샘플의 다종 동시 식별

#### 3. 첨단 기술
- **AI 이미지 인식**: 사진/영상 자동 종 식별
- **음향 모니터링**: 소리 패턴 인식
- **드론 조사**: 항공 영상 서식지 분석
- **위성 원격탐사**: 대규모 생태계 모니터링

## 데이터 포맷 표준

### 종 관찰 기록

```json
{
  "observationId": "OBS-2025-001234",
  "timestamp": "2025-12-25T10:30:00Z",
  "observerId": "eco-researcher-001",
  "location": {
    "siteName": "한라산 국립공원",
    "coordinates": {
      "latitude": 33.3616,
      "longitude": 126.5292,
      "altitude": 1200
    },
    "habitat": "MONTANE_FOREST",
    "ecosystem": "TERRESTRIAL"
  },
  "species": {
    "scientificName": "Parus major",
    "commonName": "박새",
    "taxonomicRank": "SPECIES",
    "taxonomy": {
      "kingdom": "Animalia",
      "phylum": "Chordata",
      "class": "Aves",
      "order": "Passeriformes",
      "family": "Paridae",
      "genus": "Parus",
      "species": "major"
    },
    "iucnStatus": "LC",
    "isEndemic": false
  },
  "individual": {
    "count": 3,
    "lifeStage": "adult",
    "sex": "unknown",
    "behavior": "먹이 활동"
  },
  "method": {
    "surveyType": "POINT_COUNT",
    "samplingEffort": 10,
    "detectionMethod": "시각 및 청각",
    "identificationMethod": "야외 관찰"
  },
  "evidence": {
    "photos": ["https://storage.wia.org/obs/001234-1.jpg"],
    "audio": [],
    "dnaSequence": null,
    "voucherSpecimen": null
  },
  "metadata": {
    "dataQuality": 95,
    "verified": true,
    "verifiedBy": "expert-ornithologist-01",
    "verificationDate": "2025-12-26T09:00:00Z"
  }
}
```

### Darwin Core 표준 준수

```json
{
  "occurrenceID": "urn:uuid:550e8400-e29b-41d4-a716-446655440000",
  "basisOfRecord": "HumanObservation",
  "scientificName": "Parus major",
  "kingdom": "Animalia",
  "eventDate": "2025-12-25T10:30:00Z",
  "decimalLatitude": 33.3616,
  "decimalLongitude": 126.5292,
  "coordinateUncertaintyInMeters": 10,
  "individualCount": 3,
  "lifeStage": "adult",
  "establishmentMeans": "native",
  "occurrenceStatus": "present"
}
```

## API 엔드포인트

WIA-ENE-030 표준은 RESTful API 엔드포인트를 정의합니다:

### 종 관찰 관리

- `POST /api/v1/observation/create` - 종 관찰 등록
- `GET /api/v1/observation/{id}` - 관찰 정보 조회
- `PUT /api/v1/observation/{id}` - 관찰 정보 수정
- `DELETE /api/v1/observation/{id}` - 관찰 정보 삭제
- `GET /api/v1/observations` - 관찰 목록 조회 (필터링)

### 조사 프로젝트 관리

- `POST /api/v1/survey/create` - 조사 프로젝트 생성
- `GET /api/v1/survey/{id}` - 조사 정보 조회
- `GET /api/v1/survey/{id}/results` - 조사 결과 조회
- `POST /api/v1/survey/{id}/observation` - 조사에 관찰 추가

### 다양성 분석

- `POST /api/v1/diversity/calculate` - 다양성 지수 계산
- `GET /api/v1/diversity/{surveyId}` - 다양성 지수 조회
- `GET /api/v1/diversity/trends` - 시계열 트렌드 분석

### 종 정보

- `GET /api/v1/species/search` - 종 검색
- `GET /api/v1/species/endangered` - 멸종위기종 목록
- `GET /api/v1/species/endemic` - 고유종 목록
- `GET /api/v1/species/{scientificName}` - 종 상세 정보

### 서식지 평가

- `POST /api/v1/habitat/assess` - 서식지 평가
- `GET /api/v1/habitat/{siteId}` - 서식지 정보 조회
- `GET /api/v1/habitat/quality` - 서식지 품질 분석

### eDNA 분석

- `POST /api/v1/edna/submit` - eDNA 샘플 제출
- `GET /api/v1/edna/{sampleId}/results` - 분석 결과 조회
- `POST /api/v1/edna/metabarcoding` - 메타바코딩 분석 요청

## 구현 가이드

### 1단계: 조사 계획 수립 (1-2개월)

1. 조사 목표 설정
2. 조사 지역 선정
3. 대상 분류군 결정
4. 조사 방법 선택
5. 인력 및 장비 확보

### 2단계: 현장 조사 (3-12개월)

1. 조사구 설치
2. 정기적 조사 수행 (계절별)
3. 데이터 수집 및 기록
4. 표본 채집 (필요시)
5. 사진/영상 기록

### 3단계: 데이터 분석 (1-2개월)

1. 데이터 정리 및 검증
2. 종 동정 및 확인
3. 다양성 지수 계산
4. 통계 분석
5. 트렌드 분석

### 4단계: 보고 및 보전 (지속)

1. 조사 보고서 작성
2. 데이터베이스 등록
3. 보전 계획 수립
4. 모니터링 지속
5. 적응적 관리

## 주요 성과 지표 (KPI)

### 생물다양성 지표

- **종 풍부도**: 기준선 대비 유지
- **Shannon 지수**: H' > 2.5
- **멸종위기종 개체수**: 전년 대비 10% 증가
- **고유종 비율**: 20% 이상
- **서식지 품질**: Grade B 이상
- **외래종 침입률**: 5% 미만

### 조사 성과

- **조사 완료율**: 95% 이상
- **데이터 품질**: 평균 80점 이상
- **전문가 검증률**: 80% 이상
- **시민 과학 참여**: 연간 1,000건 이상

### 보전 성과

- **서식지 복원**: 연간 10ha 이상
- **보호구역 확대**: 전년 대비 5% 증가
- **종 복원**: 3종 이상 개체군 증가
- **생태 통로**: 연간 5개소 조성

## MRV 프로토콜

### Tier 1: 공개 대시보드 (실시간)

- 총 관찰 종수
- 멸종위기종 개체수
- 조사 진행 현황
- 시민 과학 참여 통계

### Tier 2: 규제 보고 (분기)

- 상세 종 목록
- 다양성 지수 변화
- 서식지 품질 평가
- 위협 요인 분석

### Tier 3: 학술 검증 (연간)

- 완전한 조사 데이터
- 통계 분석 결과
- 장기 트렌드 분석
- 전문가 peer review

## 통합 예제

### Python

```python
from wia_ene030 import BiodiversityClient

client = BiodiversityClient(
    api_key='your-api-key',
    endpoint='https://api.wia.org/ene-030/v1'
)

# 종 관찰 등록
observation = client.create_observation(
    scientific_name='Parus major',
    common_name='박새',
    count=3,
    latitude=33.3616,
    longitude=126.5292,
    habitat='MONTANE_FOREST'
)

print(f'관찰 ID: {observation.observation_id}')

# 다양성 지수 계산
diversity = client.calculate_diversity(
    survey_id='SURVEY-2025-001'
)

print(f'Shannon 지수: {diversity.shannon_index:.2f}')
print(f'종 풍부도: {diversity.species_richness}')
```

### R

```r
library(wiaene030)

# 연결 설정
client <- BiodiversityClient$new(
  api_key = "your-api-key",
  endpoint = "https://api.wia.org/ene-030/v1"
)

# 종 관찰 등록
observation <- client$create_observation(
  scientific_name = "Parus major",
  common_name = "박새",
  count = 3,
  latitude = 33.3616,
  longitude = 126.5292
)

# 다양성 지수 계산
diversity <- client$calculate_diversity(
  survey_id = "SURVEY-2025-001"
)

print(paste("Shannon 지수:", round(diversity$shannon_index, 2)))
print(paste("종 풍부도:", diversity$species_richness))
```

### REST API (cURL)

```bash
curl -X POST https://api.wia.org/ene-030/v1/observation/create \
  -H "Authorization: Bearer YOUR_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "location": {
      "siteName": "한라산 국립공원",
      "coordinates": {
        "latitude": 33.3616,
        "longitude": 126.5292,
        "altitude": 1200
      },
      "habitat": "MONTANE_FOREST",
      "ecosystem": "TERRESTRIAL"
    },
    "species": {
      "scientificName": "Parus major",
      "commonName": "박새",
      "taxonomicRank": "SPECIES",
      "iucnStatus": "LC",
      "isEndemic": false
    },
    "individual": {
      "count": 3,
      "lifeStage": "adult"
    }
  }'
```

## 시민 과학 참여

### 관찰 앱 사용법

1. **앱 다운로드**: WIA Biodiversity App (iOS/Android)
2. **계정 생성**: 무료 가입
3. **종 관찰**: 사진 촬영 + 위치 자동 기록
4. **AI 식별**: 자동 종 식별 제안
5. **기록 제출**: 전문가 검증 대기
6. **배지 획득**: 기여도에 따른 레벨업

### 데이터 품질 보장

- **사진 필수**: 종 식별 가능한 고화질 사진
- **위치 정확도**: GPS 오차 10m 이내
- **시간 기록**: 자동 타임스탬프
- **전문가 검증**: 희귀종 및 의심 기록 검토
- **커뮤니티 확인**: 다수 관찰자 교차 검증

## 글로벌 현황

### 현재 상황 (2024)

- **조사 지역**: 전 세계 10,000+ 곳
- **등록 종수**: 200만+ 종
- **멸종위기종**: 44,000+ 종
- **시민 과학자**: 100만+ 명
- **관찰 기록**: 10억+ 건

### 2030 목표

- **보호구역**: 육지 30%, 해양 30%
- **멸종위기종 복원**: 1,000+ 종
- **서식지 복원**: 100만 ha
- **시민 참여**: 1,000만+ 명

### 2050 비전

- **생물다양성 손실 중단**: 순증가 전환
- **생태계 완전 복원**: 주요 생태계 건강 회복
- **인간-자연 공존**: 조화로운 공존 모델 확립

## 기여하기

WIA-ENE-030 표준 개선에 기여해 주세요:

1. **관찰 데이터 제출**: 야외 관찰 기록 공유
2. **전문가 검증**: 종 동정 검토
3. **방법론 개선**: 조사 기법 제안
4. **번역**: 다른 언어로 문서 번역
5. **코드 기여**: SDK 기능 추가

자세한 내용은 [CONTRIBUTING.md](../CONTRIBUTING.md)를 참조하세요.

## 커뮤니티 및 지원

- **웹사이트**: [wia.org/standards/ene-030](https://wia.org/standards/ene-030)
- **문서**: [docs.wia.org/ene-030](https://docs.wia.org/ene-030)
- **GitHub**: [github.com/WIA-Official/wia-standards](https://github.com/WIA-Official/wia-standards)
- **이메일**: standards@wia.org
- **커뮤니티**: [community.wia.org](https://community.wia.org)

## 라이선스

이 표준은 [MIT License](https://opensource.org/licenses/MIT) 하에 배포됩니다.

다음과 같은 자유가 있습니다:
- **공유**: 자료를 복사하고 재배포
- **수정**: 리믹스, 변환 및 빌드
- **상업적 사용**: 상업적 목적으로 사용

단, 다음 조건을 준수해야 합니다:
- **저작자 표시**: WIA에 적절한 크레딧 제공

## 인용

```bibtex
@standard{wia-ene-030,
  title = {WIA-ENE-030: Biodiversity Index Standard},
  author = {{World Certification Industry Association}},
  year = {2025},
  version = {1.0},
  url = {https://github.com/WIA-Official/wia-standards/biodiversity-index}
}
```

## 감사의 말

이 표준은 다음 분들의 기여로 개발되었습니다:
- 생태학자 및 분류학 전문가
- 국립공원 및 보호구역 관리자
- 생물다양성 연구 기관
- 시민 과학자 및 자원봉사자
- 환경 NGO 및 보전 단체
- 국제 생물다양성 협약 당사국

## 관련 표준

- **WIA-ENE-001**: 기후 변화 관리
- **WIA-ENE-004**: 해양 플라스틱 추적
- **WIA-ENE-028**: 대기 질 모니터링
- **WIA-ENE-029**: 수질 관리
- **WIA-SOCIAL**: 환경 교육 및 시민 참여

## 변경 이력

### Version 1.0.0 (2025-12-25)

- 초판 발행
- 완전한 문서 패키지
- TypeScript SDK
- CLI 도구
- API 스펙
- Darwin Core 준수

---

## 홍익인간 (弘益人間) (홍익인간) · 널리 인간을 이롭게 하라

WIA-ENE-030 표준은 홍익인간 (弘益人間)(홍익인간)의 정신을 구현합니다. 생물다양성 보전을 통해 모든 생명체의 공존을 보장하고, 과학적 방법론과 시민 참여를 결합하여 지구의 생명의 그물을 지키며, 미래 세대를 위한 건강한 생태계를 보전합니다.

개방형 표준, 투명한 데이터, 협력적 모니터링을 통해 생물다양성 보전이 인류 전체와 지구의 공동선에 기여하도록 보장합니다.

**함께, 우리는 모든 생명을 위한 지구를 지킵니다.**

---

© 2025 SmileStory Inc. / WIA
**홍익인간 (弘益人間) (홍익인간) · Benefit All Humanity**
