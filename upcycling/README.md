# WIA-ENE-024: 업사이클링 표준 🔄

> **弘익人間 (홍익인간)** - 널리 인간을 이롭게 하라

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Version](https://img.shields.io/badge/Version-1.0-blue.svg)](https://github.com/WIA-Official/wia-standards)
[![Standard](https://img.shields.io/badge/Standard-WIA--ENE--024-green.svg)](https://wia.org/standards/ene-024)

## 개요

WIA-ENE-024 업사이클링 표준은 폐기물을 더 높은 가치의 제품으로 전환하는 프로세스를 표준화하여, 순환경제 실현과 지속가능한 자원 활용을 촉진합니다.

### 핵심 특징

- 🔄 **창의적 가치 창출**: 폐기물을 원재료보다 높은 가치의 제품으로 전환
- 📊 **표준화된 프로세스**: 수집, 분류, 디자인, 제작, 품질검증의 체계적 워크플로우
- 🌱 **환경 영향 측정**: LCA 기반 CO₂ 저감, 물 절약, 에너지 절약 정량화
- 💡 **비즈니스 모델**: 아티산 스튜디오부터 대량 생산까지 다양한 수익 모델
- 🔒 **품질 및 안전**: 엄격한 품질 기준과 화학물질 안전성 검증
- 🎓 **교육 및 인증**: 체계적인 교육 프로그램과 4단계 자격 인증 시스템

## 빠른 시작

### 1. 설치

```bash
# 저장소 클론
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/upcycling

# 설치 스크립트 실행
chmod +x install.sh
./install.sh
```

### 2. TypeScript SDK 사용

```typescript
import { UpcyclingClient } from '@wia/ene-024';

const client = new UpcyclingClient({
  apiKey: 'your-api-key',
  endpoint: 'https://api.wia.org/ene-024/v1'
});

// 새 업사이클링 프로젝트 생성
const project = await client.createProject({
  projectName: '청바지 토트백 제작',
  sourceMaterial: {
    materialType: 'textile',
    subType: 'denim',
    quantity: 3,
    weight: 1.5,
    weightUnit: 'kg',
    condition: 'B',
    sourceChannel: 'donation_center',
    acquisitionCost: 5000,
    currency: 'KRW'
  },
  outputProduct: {
    productType: 'bag',
    subType: 'tote_bag',
    productName: 'Vintage Denim Tote',
    quantity: 2
  }
});

console.log('프로젝트 ID:', project.data.projectId);
```

### 3. CLI 사용

```bash
# 프로젝트 생성
upcycling create --name "폐타이어 화분 제작" \
  --material-type rubber \
  --quantity 10

# 프로젝트 목록 조회
upcycling list --status completed

# 환경 영향 계산
upcycling calculate-impact --project-id UP-2025-001234

# 프로젝트 상세 조회
upcycling get --id UP-2025-001234
```

## 저장소 구조

```
upcycling/
├── README.md               # 이 파일 (한국어)
├── install.sh              # 설치 스크립트
├── spec/
│   └── WIA-ENE-024-v1.0.md # 상세 기술 명세
├── api/
│   └── typescript/
│       ├── src/
│       │   ├── types.ts    # TypeScript 타입 정의
│       │   └── index.ts    # SDK 구현
│       └── package.json    # NPM 패키지 설정
└── cli/
    └── upcycling.sh        # CLI 도구
```

## 업사이클링 프로세스

### 7단계 워크플로우

```
1. 원료 수집 (Collection)
   └─> 재활용센터, 기업, 개인으로부터 폐기물 확보

2. 분류/선별 (Sorting)
   └─> A급 (95-100%) ~ F급 (0-24%) 상태 등급 분류

3. 세척/준비 (Preparation)
   └─> 재료별 최적 세척 및 표면 처리

4. 디자인 (Design)
   └─> 창의적 가치 부여 및 제품 기획

5. 제작 (Production)
   └─> 수작업, 세미오토, 자동화 방식 제작

6. 품질검증 (QA)
   └─> 구조, 안전성, 내구성, 친환경성 검증

7. 판매/유통 (Distribution)
   └─> 온라인, 팝업, 갤러리, 기업 제휴
```

## 적용 분야

| 분야 | 원료 | 제품 예시 |
|------|------|----------|
| **섬유/패션** | 의류, 가방 | 리디자인 청바지, 토트백, 액세서리 |
| **가구/인테리어** | 폐가구, 팔레트 | 업사이클 가구, 조명, 선반 |
| **플라스틱** | 폐플라스틱 | 화분, 수납함, 장난감 |
| **금속** | 폐금속 | 조형물, 램프, 가구 부품 |
| **전자제품** | E-waste | 부품 재활용, 예술작품 |
| **건축자재** | 건설 폐기물 | 벽돌, 타일, 단열재 |
| **유리/도자기** | 깨진 유리 | 모자이크, 장식품 |
| **종이/골판지** | 폐지 | 디자인 소품, 패키징 |

## 데이터 표준

### 재료 카테고리 코드

| 코드 | 카테고리 | 세부 분류 |
|------|----------|----------|
| **TEX** | 섬유 (Textile) | cotton, denim, wool, synthetic, leather |
| **PLT** | 플라스틱 (Plastic) | PET, HDPE, PVC, LDPE, PP, PS |
| **MET** | 금속 (Metal) | steel, aluminum, copper, brass, iron |
| **WOD** | 목재 (Wood) | pine, oak, plywood, pallet, furniture |
| **GLS** | 유리 (Glass) | bottle, window, ceramic, porcelain |
| **PAP** | 종이 (Paper) | cardboard, newspaper, magazine, book |
| **EWS** | 전자폐기물 | phone, computer, appliance, circuit_board |
| **CON** | 건축자재 | brick, tile, concrete, insulation |

### 상태 등급

| 등급 | 설명 | 업사이클링 적합성 |
|------|------|------------------|
| **A** | Excellent - 거의 손상 없음 | 매우 높음 (95-100%) |
| **B** | Good - 경미한 손상 | 높음 (75-94%) |
| **C** | Fair - 중간 손상 | 보통 (50-74%) |
| **D** | Poor - 심각한 손상 | 낮음 (25-49%) |
| **F** | Failed - 업사이클 불가 | 매우 낮음 (0-24%) |

## API 엔드포인트

### 프로젝트 관리

```
POST   /api/v1/upcycling/projects          # 새 프로젝트 생성
GET    /api/v1/upcycling/projects          # 프로젝트 목록 조회
GET    /api/v1/upcycling/projects/{id}     # 프로젝트 상세 조회
PATCH  /api/v1/upcycling/projects/{id}     # 프로젝트 업데이트
DELETE /api/v1/upcycling/projects/{id}     # 프로젝트 삭제
```

### 재료 관리

```
POST   /api/v1/materials                    # 재료 등록
GET    /api/v1/materials                    # 재료 목록
GET    /api/v1/materials/{id}               # 재료 상세
PATCH  /api/v1/materials/{id}/status        # 재료 상태 변경
```

### 환경 영향 분석

```
GET    /api/v1/impact/calculate             # 환경 영향 계산
GET    /api/v1/impact/report/{projectId}    # 영향 보고서
```

## 환경 영향 평가

### 계산 방법론 (LCA 기반)

#### CO₂ 배출 저감량

```
CO₂ 저감량 (kg) = [신제품 생산 CO₂ - 업사이클링 CO₂]
```

**재료별 신제품 생산 배출 계수**:

| 재료 | CO₂ 배출량 (kg CO₂/kg 재료) |
|------|---------------------------|
| Cotton (면) | 8.0 |
| Denim (데님) | 10.0 |
| Polyester | 6.4 |
| Plastic | 6.0 |
| Steel | 2.0 |
| Aluminum | 9.0 |
| Wood | 0.5 |
| Glass | 0.85 |

#### 물 절약량

| 재료 | 신제품 물 사용량 (L/kg) |
|------|---------------------|
| Cotton | 10,000 |
| Denim | 11,000 |
| Polyester | 80 |
| Paper | 300 |

## 품질 및 안전 기준

### 화학물질 안전 기준

| 항목 | 기준 | 측정 방법 |
|------|------|----------|
| 납 (Pb) | ≤ 90 ppm | XRF 분석 |
| 카드뮴 (Cd) | ≤ 75 ppm | XRF 분석 |
| 수은 (Hg) | ≤ 60 ppm | XRF 분석 |
| VOC 방출 | ≤ 500 μg/m³ | 실내공기질 측정 |
| 폼알데히드 | ≤ 75 ppm | 방출 테스트 |

### 내구성 테스트

**가구류**:
- 좌면 하중: ≥ 150 kg (500회 반복)
- 등받이 하중: ≥ 80 kg (100회 반복)
- 안정성: 전도 각도 ≥ 15°

**가방/섬유류**:
- 인장 강도: ≥ 200 N
- 스트랩 하중: ≥ 20 kg (1000회 반복)
- 마모 저항: ≥ 10,000회 (Martindale)

## 경제성 평가

### 가치 배율 (Value Multiplier)

```
가치 배율 = 최종 판매가 / 원료 획득 비용
```

**벤치마크**:
- 우수: ≥ 10배
- 양호: 5-10배
- 보통: 3-5배
- 저조: < 3배

### 가격 책정 전략

```
권장 판매가 = (총 원가 × 2.5) + 예술적 프리미엄
```

**예술적 프리미엄 요소**:
- 독창적 디자인: +20-50%
- 유명 디자이너: +30-100%
- 스토리텔링: +10-30%
- 한정판: +50-200%
- 사회적 가치: +10-20%

## WIA-ENE-024 인증

### 인증 레벨

#### 레벨 1: 기본 인증 (Basic)

**요구사항**:
- 원료 출처 문서화
- 제작 프로세스 기록
- 품질 검사 통과
- 환경 영향 계산

**혜택**:
- WIA-ENE-024 인증 마크 사용
- WIA 레지스트리 등록
- 기본 마케팅 자료 제공

#### 레벨 2: 프리미엄 인증 (Premium)

**요구사항** (레벨 1 +):
- 제3자 품질 감사
- 탄소중립 제작 인증
- 사회적 가치 입증
- 지속가능성 보고서

**혜택**:
- 프리미엄 인증 마크
- WIA 공식 파트너 지정
- 국제 전시회 참가 지원
- B2B 매칭 지원

#### 레벨 3: 엑셀런스 인증 (Excellence)

**요구사항** (레벨 2 +):
- 혁신적 디자인 수상
- 국제 인증 (GRS, C2C 등)
- 사회적 임팩트 검증
- 산업 표준 기여

**혜택**:
- 엑셀런스 인증 마크
- WIA 앰버서더 지정
- 국제 협력 프로젝트 참여
- 정책 자문 기회

## 블록체인 추적성

모든 WIA-ENE-024 인증 제품은 블록체인에 기록되어 전체 이력을 투명하게 추적할 수 있습니다.

```
원료 수집 → 분류/선별 → 제작 → 품질검증 → 판매
    ↓          ↓         ↓         ↓         ↓
[블록체인 이벤트 기록 - 변조 불가능한 타임라인]
```

## 교육 및 자격

### 교육 프로그램

| 과정 | 시간 | 내용 |
|------|------|------|
| **입문 과정** | 4시간 | 업사이클링 개념, 재료 선별, 간단한 실습 |
| **실무 과정** | 16시간 | 디자인 씽킹, 제작 기법, 품질 관리, 실전 프로젝트 |
| **비즈니스 과정** | 12시간 | 사업 계획, 마케팅, 유통, 법률/세무, 투자 유치 |

### 자격 인증

| 레벨 | 명칭 | 요구사항 | 역량 |
|------|------|----------|------|
| **L1** | 업사이클러 | 입문 과정 수료 | 기초 제작 |
| **L2** | 전문 업사이클러 | 실무 과정 수료 + 실적 5건 | 독립 제작 |
| **L3** | 마스터 업사이클러 | L2 + 2년 경력 + 수상 | 디자인/교육 |
| **L4** | 업사이클 컨설턴트 | L3 + 비즈니스 과정 | 사업 지원 |

## 성공 사례

### 프라이탁 (FREITAG) - 스위스

- **원료**: 중고 트럭 방수포, 자전거 튜브, 자동차 안전벨트
- **제품**: 메신저백, 백팩, 지갑
- **성과**: 연매출 6천만 유로 (약 800억 원)
- **전략**: 각 제품의 고유성, 강력한 브랜드, 내구성 (보증 2년)

### 터치포굿 (Touch4Good) - 한국

- **원료**: 폐현수막, 폐가죽, 폐자동차 시트
- **제품**: 가방, 파우치, 소품
- **성과**: 누적 판매 10만 개 이상
- **전략**: 기업 CSR 협력, 사회적 기업 가치, 장애인 고용 40%

## 시장 전망

### 글로벌 업사이클링 시장

| 연도 | 시장 규모 | 성장률 |
|------|----------|--------|
| 2024 | $480억 | - |
| 2025 | $540억 | 12.5% |
| 2027 | $680억 | 12.0% |
| 2030 | $950억 | 11.5% |

### 주요 성장 동인

1. **환경 의식 증가**: Z세대, 밀레니얼의 지속가능 소비
2. **순환경제 정책**: 각국 정부의 규제 및 지원
3. **기술 발전**: 3D 프린팅, AI 디자인
4. **기업 ESG**: 대기업의 업사이클 제품 구매
5. **온라인 플랫폼**: 글로벌 유통 확대

## 2030 비전

**WIA-ENE-024가 그리는 미래**:

1. **보편화**: 업사이클링이 리사이클링만큼 일상화
2. **산업화**: 대규모 업사이클링 공장 운영
3. **글로벌 표준**: WIA-ENE-024가 국제 표준으로 채택
4. **교육 통합**: 초중고 교육과정에 업사이클링 포함
5. **문화 정착**: "새 제품보다 업사이클 제품"이 자랑

## 커뮤니티 및 지원

- **웹사이트**: [wia.org/standards/ene-024](https://wia.org/standards/ene-024)
- **문서**: [docs.wia.org/ene-024](https://docs.wia.org/ene-024)
- **GitHub**: [github.com/WIA-Official/wia-standards](https://github.com/WIA-Official/wia-standards)
- **이메일**: ene024@wia-official.org

## 라이선스

본 표준은 **MIT License** 하에 배포됩니다.

```
MIT License

Copyright (c) 2025 SmileStory Inc. / WIA

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
```

## 인용

```bibtex
@standard{wia-ene-024,
  title = {WIA-ENE-024: Upcycling Standard},
  author = {{World Certification Industry Association}},
  year = {2025},
  version = {1.0},
  url = {https://github.com/WIA-Official/wia-standards/upcycling}
}
```

## 관련 표준

- **WIA-ENE-003**: Carbon Capture & Storage
- **WIA-ENE-025**: Waste Management
- **WIA-ENE-026**: Recycling
- **WIA-ENE-027**: E-Waste Management
- **WIA-BLOCKCHAIN**: Immutable Product Tracking

## 감사의 글

이 표준은 다음 분들의 기여로 개발되었습니다:
- 업사이클링 아티스트 및 디자이너
- 순환경제 연구자 및 학자
- 환경 정책 전문가
- 사회적 기업 운영자
- 소비자 및 커뮤니티

---

## 홍익인간 (弘益人間) (홍익인간) · 널리 인간을 이롭게 하라

업사이클링은 단순한 재활용을 넘어서, 창의성과 환경 보호, 경제적 가치를 동시에 실현하는 지속가능한 미래의 핵심입니다.

버려진 것에서 새로운 가능성을 발견하고, 폐기물을 예술과 기능으로 승화시키는 업사이클링은 우리 모두가 참여할 수 있는 홍익인간의 실천입니다.

**WIA-ENE-024와 함께, 지속가능한 미래를 만들어갑니다.**

---

© 2025 SmileStory Inc. / WIA
**홍익인간 (弘益人間) (홍익인간) · Benefit All Humanity**
