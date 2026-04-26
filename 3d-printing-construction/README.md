# WIA-CITY-008: 3D 프린팅 건설 표준 🖨️

> **홍익인간 (弘益人間) (홍익인간) - 널리 인간을 이롭게 하라**

3D 프린팅 건설(Construction 3D Printing)의 표준화된 프린터, 재료, 공정, 품질 관리를 위한 포괄적 표준입니다.

## 🌟 개요

WIA-CITY-008 3D 프린팅 건설 표준은 적층 제조(Additive Manufacturing) 기술을 건축 분야에 적용하여 더 빠르고, 저렴하고, 친환경적인 건설을 실현하기 위한 완전한 프레임워크를 제공합니다.

### 철학

**홍익인간 (弘益人間) (홍익인간)** - "널리 인간을 이롭게 하라"의 철학을 바탕으로, 이 표준은 3D 프린팅 기술로 더 많은 사람에게 안전하고 저렴한 주거를 제공하는 것을 목표로 합니다.

## 📊 현황

### 글로벌 3D 프린팅 건설 현황 (2024)

- **시장 규모**: 산업 보고서 기준 빠른 성장 단계 (수억 USD → 수십억 USD 전망)
- **성장세**: 연 두 자릿수 후반 성장률로 알려짐 (출처별 편차 큼, 본 표준은 점추정 미사용)
- **완공된 건물**: 전 세계 다수 사례 (북미·유럽·중동 중심)
- **주요 시장**: 미국, 중국, 유럽, 중동

### 주요 제조사

| 제조사 | 국가 | 대표 모델 | 특징 |
|--------|------|-----------|------|
| **ICON** | 미국 🇺🇸 | Vulcan II | NASA 달 기지 프린터 |
| **COBOD** | 덴마크 🇩🇰 | BOD2 | 세계 최대 건설 프린터 |
| **WASP** | 이탈리아 🇮🇹 | Crane WASP | 친환경 점토 프린팅 |
| **Apis Cor** | 러시아 🇷🇺 | Mobile Printer | 24시간 주택 건설 |
| **Mighty Buildings** | 미국 🇺🇸 | MDU | 복합 재료 프린팅 |

### 주요 문제점

1. 🔧 **표준 부재**: 프린터, 재료, 품질 기준 표준화 미흡
2. 📜 **건축 법규**: 기존 법규와의 충돌, 인증 절차 불명확
3. 🏗️ **구조 안전**: 층간 접착력 검증 부족
4. ⏳ **내구성**: 장기 내구성 데이터 부족 (>50년)
5. 🔄 **상호운용성**: 제조사별 독자 규격
6. 👷 **인력**: 전문 인력 부족

## 🎯 표준의 가치

### 핵심 목표

- ✅ **통일된 표준**: 프린터, 재료, 공정의 표준화
- ✅ **품질 보증**: 구조 안전성 검증 및 인증 체계
- ✅ **법규 준수**: 국제 건축 법규와의 호환성
- ✅ **추적성**: 건설 전 과정의 투명한 기록
- ✅ **상호운용성**: 제조사 간 호환 가능한 데이터 형식
- ✅ **지속가능성**: 친환경 재료 및 저탄소 건설

### 기대 효과

**경제적 효과 (현장 보고 기반 — 점추정 아님):**
- 💰 **건설 비용**: 전통 공법 대비 유의한 절감이 다수 사례에서 보고됨
- 💰 **인건비**: 자동화로 큰 폭의 절감 가능 (현장 조건에 따라 편차)
- 💰 **재료 낭비**: 적층 방식 특성상 절단·잔재가 적어 폐기물 감소

**시간적 효과 (대표 사례 보고치):**
- ⚡ **소형 주택**: 일 단위 (사례별 24~48시간 보고)
- ⚡ **중형 주택**: 며칠
- ⚡ **대형 건물**: 수 주
- ⚡ **전통 대비**: 공기 단축 효과가 일관되게 보고됨

**환경적 효과 (정성적 기조):**
- 🌍 **CO2 배출**: 저탄소 시멘트·지오폴리머 적용 시 감소
- 🌍 **재료 낭비**: 적층 방식으로 현저히 감소
- 🌍 **재활용 재료**: 점토·재활용 골재 등 활용 가능
- 🌍 **에너지 소비**: 공정 자동화·재료 최적화로 감소 가능

**사회적 효과:**
- 🏘️ **저가 주택**: 개발도상국, 재난 지역 신속 공급
- 🏘️ **맞춤형 주택**: 개인 맞춤 설계
- 🏘️ **특화 주택**: 고령자, 장애인 맞춤형
- 🏘️ **긴급 대피소**: 재난 지역 24시간 내 건설

## 📁 저장소 구조

```
3d-printing-construction/
├── README.md                  # 본 문서
├── install.sh                 # 설치 스크립트
├── spec/
│   └── WIA-CITY-008-v1.0.md  # 상세 기술 명세
├── api/
│   └── typescript/
│       ├── src/
│       │   ├── types.ts       # TypeScript 타입 정의
│       │   └── index.ts       # SDK 구현
│       └── package.json       # 패키지 정보
└── cli/
    └── 3d-printing-construction.sh  # CLI 도구
```

## 🖨️ 프린터 유형

### 1. Gantry (갠트리)

```
특징:
  - 고정 레일 시스템
  - 프린트 영역: 최대 15m × 15m × 10m
  - 정밀도: ±2-5mm
  - 속도: 50-200 mm/s

대표 모델:
  - COBOD BOD2: 100m² 주택 48시간
  - PERI 3D Printer

용도: 주거용, 상업용 건물
```

### 2. Robotic Arm (로봇 암)

```
특징:
  - 6축 산업용 로봇
  - 작업 반경: 2-4m
  - 정밀도: ±1-3mm
  - 유연성: 복잡한 곡선 형상 가능

대표 모델:
  - Apis Cor: 24시간 주택 건설
  - CyBe Construction

용도: 맞춤형 건축, 예술 건축
```

### 3. WASP (델타 프린터)

```
특징:
  - 삼각 케이블 시스템
  - 프린트 지름: 최대 12m
  - 재료: 주로 점토, 흙
  - 친환경: 100% 재활용 가능

대표 모델:
  - Crane WASP: 친환경 주택

용도: 친환경 주택, 저가 주택
```

### 4. Crane (크레인)

```
특징:
  - 타워 크레인 기반
  - 프린트 영역: 무제한 (크레인 이동)
  - 대형 건물 가능

용도: 초대형 건물, 공공 시설
```

## 🏗️ 재료 종류

### 1. Concrete (콘크리트)

```
조성:
  시멘트: 25-35%
  골재: 40-50%
  물: 10-15%
  첨가제: 3-8%

물성:
  압축 강도: 20-60 MPa
  경화 시간: 28일

특징:
  - 전통 건축과 동일한 성능
  - 검증된 내구성
  - 비교적 저렴

용도: 주거용, 상업용 건물
```

### 2. Geopolymer (지오폴리머)

```
조성:
  플라이애시: 60-80%
  슬래그: 10-20%
  알칼리 활성제: 5-10%

환경 장점:
  CO2 배출: 전통 시멘트 대비 80% 감소
  에너지 소비: 70% 감소

물성:
  압축 강도: 30-80 MPa (매우 높음)
  내화성: 1,000°C까지 안정

용도: 친환경 주택, 내화 구조물
```

### 3. Clay (점토)

```
조성:
  점토: 70-90%
  모래: 5-15%
  섬유: 2-5%

환경 장점:
  100% 천연 재료
  100% 재활용 가능
  CO2 배출: 거의 없음

물성:
  압축 강도: 2-10 MPa
  건조 시간: 2-4주

용도: 저가 주택, 친환경 주택
```

### 4. Fiber-Reinforced (섬유 보강)

```
첨가 섬유:
  - 유리 섬유: 인장 강도 +50%
  - 탄소 섬유: 인장 강도 +100%
  - 강철 섬유: 인성 향상

효과:
  - 인장 강도: 2-3배 향상
  - 굽힘 강도: 1.5-2배 향상
  - 균열 저항: 크게 향상

용도: 고강도 구조물
```

## 📡 API 인터페이스

### 프로젝트 등록

```http
POST /api/v1/projects/register
Content-Type: application/json

{
  "project": {
    "name": "에코 하우스",
    "type": "RESIDENTIAL",
    "location": {
      "lat": 37.5665,
      "lon": 126.9780,
      "address": "서울특별시 중구",
      "city": "서울",
      "country": "Korea"
    },
    "design": {
      "dimensions": {
        "length_m": 10.0,
        "width_m": 8.0,
        "height_m": 3.0
      }
    }
  }
}

Response:
{
  "success": true,
  "data": {
    "projectId": "CITY3DP-2025-KOR-001",
    "trackingUrl": "https://track.wia.org/city3dp-2025-kor-001"
  }
}
```

### 프린트 작업 제출

```http
POST /api/v1/print-jobs/submit

{
  "projectId": "CITY3DP-2025-KOR-001",
  "gcode_url": "https://storage.wia.org/gcode/eco-house.gcode",
  "parameters": {
    "layer_height_mm": 15,
    "print_speed_mm_s": 100,
    "extrusion_rate_kg_h": 200
  }
}

Response:
{
  "jobId": "PRINTJOB-2025-KOR-001",
  "status": "QUEUED",
  "estimated_completion": "2025-06-05T18:00:00Z"
}
```

### 실시간 모니터링

```http
GET /api/v1/print-jobs/{jobId}

Response:
{
  "status": "PRINTING",
  "progress_percent": 45,
  "current_layer": 90,
  "total_layers": 200,
  "remaining_time_hours": 15.2,
  "material_used_kg": 2250
}
```

## 🚀 빠른 시작

### 1. 설치

```bash
# 저장소 복제
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/3d-printing-construction

# 설치
./install.sh
```

### 2. CLI 사용

```bash
# 프로젝트 등록
./cli/3d-printing-construction.sh register-project \
  --name "Eco House" \
  --type RESIDENTIAL \
  --address "123 Main St" \
  --city "Seoul" \
  --country "Korea" \
  --lat 37.5665 \
  --lon 126.9780 \
  --length 10 \
  --width 8 \
  --height 3

# 프린트 작업 제출
./cli/3d-printing-construction.sh submit-print \
  --project-id CITY3DP-2025-KOR-001 \
  --gcode https://storage.wia.org/gcode/house.gcode \
  --layer-height 15

# 작업 상태 조회
./cli/3d-printing-construction.sh status PRINTJOB-2025-001

# 프로젝트 목록
./cli/3d-printing-construction.sh list-projects

# 프린터 목록
./cli/3d-printing-construction.sh list-printers

# 비용 추정 (length, width, height in meters)
./cli/3d-printing-construction.sh estimate-cost 10 8 3
```

### 3. TypeScript SDK

```typescript
import { City3DPrintClient } from '@wia/city-008';

const client = new City3DPrintClient({
  apiKey: 'your-api-key',
  endpoint: 'https://api.wia.org/city-008/v1'
});

// 프로젝트 등록
const project = await client.registerProject({
  project: {
    name: 'Eco House',
    design: {
      dimensions: {
        length_m: 10.0,
        width_m: 8.0,
        height_m: 3.0,
        total_area_m2: 80.0
      },
      walls: {
        thickness_mm: 200,
        material: 'CONCRETE'
      }
    },
    location: {
      lat: 37.5665,
      lon: 126.9780,
      address: 'Seoul, Korea',
      city: 'Seoul',
      country: 'Korea'
    }
  }
});

console.log('Project ID:', project.data?.projectId);

// 비용 추정
const estimate = await client.estimateProjectCost({
  dimensions: {
    length_m: 10,
    width_m: 8,
    height_m: 3
  }
});

console.log('Estimated cost:', estimate.data?.total_cost_USD);

// 프린트 작업 제출
const job = await client.submitPrintJob({
  projectId: 'CITY3DP-2025-KOR-001',
  gcode_url: 'https://storage.wia.org/gcode/house.gcode',
  parameters: {
    layer_height_mm: 15,
    print_speed_mm_s: 100
  }
});

// 실시간 모니터링
import { PrintJobMonitor } from '@wia/city-008';

const monitor = new PrintJobMonitor({ apiKey: 'your-api-key' });
await monitor.connect();

monitor.subscribe('PRINTJOB-2025-001', (update) => {
  console.log(`Progress: ${update.progress_percent}%`);
  console.log(`Layer: ${update.current_layer}/${update.total_layers}`);
});
```

## 🏅 성공 사례

### ICON - Community First! Village (미국, 텍사스)

```
프로젝트: 저가 주택 단지
규모: 50채
면적: 60m² / 채
비용: $10,000 / 채 (전통 대비 70% 절감)
공기: 24시간 / 채
특징: 노숙자 주거 지원
```

### COBOD - 유럽 최초 3층 건물 (독일)

```
프로젝트: 3층 주거용 건물
면적: 380m²
공기: 3주 (프린팅), 12주 (총 공사)
특징: 복잡한 곡선 형상
```

### WASP - Gaia (이탈리아)

```
프로젝트: 친환경 주택
재료: 100% 천연 점토
면적: 30m²
비용: $900 (재료비만)
특징: 100% 재활용 가능
```

### Apis Cor - 24시간 주택 (러시아)

```
프로젝트: 단독 주택
면적: 38m²
공기: 24시간
비용: $10,000
특징: 이동식 프린터, 원형 주택
```

## 🏗️ 건축 법규 준수

### 국제 건축 법규

- **IBC** (International Building Code): 국제 표준
- **KBC** (Korean Building Code): 한국 표준
- **Eurocode**: 유럽 표준
- **NBC**: 인도, 캐나다 표준

### 3D 프린팅 특화 요구사항

1. **층간 접착력 검증**:
   - 시험 방법: ASTM C1583
   - 최소 요구: 3.0 MPa
   - 샘플링: 100m² 당 1개소

2. **경화 시간 준수**:
   - 최소 경화: 28일
   - 하중 재하: 56일 이후

3. **설계 도서**:
   - 3D 모델 (STL, OBJ)
   - G-code
   - 재료 명세서
   - 구조 계산서

4. **시공 기록**:
   - 전 과정 비디오 기록
   - 온도/습도 로그
   - 재료 배치 추적
   - 품질 검사 기록

## 🌍 글로벌 영향

### 2030 지향점

- 저가 주택의 대규모 보급 (개발도상국·재난지역 우선)
- 저탄소 자재 채택 확대로 누적 CO2 저감
- 전통 공법 대비 의미 있는 비용·공기 절감 정착
- 재난 대응을 위한 긴급 대피소 신속 공급 체계

### 2050 비전

- 일정 규모 이상의 시장 점유 확보
- 완전 자동화: AI 기반 무인 건설
- 우주 건설: 달/화성 기지 건설
- 순환경제: 재활용 재료 활용 극대화

## 🤝 기여

이 표준은 3D 프린팅 건설 커뮤니티의 기여를 환영합니다:

- 기술 피드백 및 개선 제안
- 프린터 및 재료 사례 연구
- 건설 프로젝트 사례 공유
- 품질 기준 개선
- 추가 언어 번역

## 📜 라이선스

© 2025 SmileStory Inc. / WIA - World Certification Industry Association

이 표준은 MIT 라이선스 하에 배포됩니다.

## 🔗 관련 표준

- **WIA-CITY-001**: 스마트 시티 표준
- **WIA-ENE-022**: 폐기물 관리 표준
- **WIA-BLOCKCHAIN**: 블록체인 기반 추적
- **WIA-INTENT**: 의도 표현 표준

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

WIA-CITY-008 표준은 弘익人間의 철학을 통해 3D 프린팅 기술로 더 많은 사람에게 안전하고 저렴한 주거를 제공합니다.

3D 프린팅 건설은 전통 공법 대비 공기·비용·환경 부담의 의미 있는 절감을 다수 현장에서 보여 왔습니다. 이를 통해 개발도상국, 재난 지역, 저소득층에게 신속하고 저렴한 주거를 제공할 수 있습니다.

**함께 만드는 지속가능한 건설, 함께 지키는 아름다운 지구**

---

*문서 버전: 1.0.0*
*최종 업데이트: 2025-12-25*
*상태: Active*
