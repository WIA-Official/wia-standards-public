# 제2장: GS1 표준 및 식별 시스템

**WIA-AGRI-016 전자책 시리즈**

---

## GS1 소개

GS1은 비즈니스 커뮤니케이션을 위한 표준을 개발하고 유지하는 글로벌 조직입니다. 그들의 식별 표준은 현대 이력 추적 시스템의 기초이며, 150개국의 200만 개 이상의 회사에서 사용됩니다.

### GS1 표준이 중요한 이유

**보편적 언어:** GS1 표준은 회사, 산업 또는 국가에 관계없이 전체 공급망에서 제품, 위치 및 자산을 식별하는 공통 언어를 만듭니다.

**상호운용성:** GS1 표준을 사용하는 시스템은 맞춤 통합 없이 자동으로 데이터를 교환할 수 있습니다.

**대규모 검증:** 40년 이상 사용되어 매일 수십억 건의 거래를 처리합니다.

---

## 핵심 GS1 식별자

### 1. GTIN (글로벌 무역 품목 번호)

**목적:** 제품을 고유하게 식별

**형식:** 8, 12, 13 또는 14자리

**구조:**
```
GTIN-14: 01234567890128
         ││└─ 제품 참조 (7자리)
         │└── 회사 접두사 (6자리)
         └─── 지시자 숫자 (1자리)
         └─── 체크 디지트 (1자리)
```

**예시:**
```
GTIN: 00012345678905
- 회사 접두사: 0001234 (GS1이 할당)
- 품목 참조: 56789 (회사가 할당)
- 체크 디지트: 5 (계산됨)
```

**사용 시기:**
- 다른 제품 유형 (사과 vs. 오렌지)
- 다른 브랜드
- 다른 패키지 크기
- 다른 제형

**사용하지 않을 때:**
- 동일 제품의 다른 배치 (로트 번호 사용)
- 개별 품목 (일련번호 사용)

### 2. GLN (글로벌 위치 번호)

**목적:** 물리적 위치 및 법인을 고유하게 식별

**형식:** 13자리

**예시:**
```
농장 위치:          1234567890123
포장장:            1234567890124
물류센터:          1234567890125
소매점 #456:       1234567890126
```

**세분화 수준:**
- 법인: ABC 유기농 농장 주식회사
- 기능: ABC 유기농 농장 - 품질관리부
- 물리적 위치: ABC 유기농 농장 - 창고 건물 2
- 하위 위치: ABC 유기농 농장 - 냉장 보관실 3

---

## 바코드

### 1. EAN/UPC 바코드

**EAN-13:** 유럽/아시아 소매 제품에 가장 일반적
**UPC-A:** 북미 소매 제품에 가장 일반적

**특성:**
- 1D 선형 바코드
- GTIN 인코딩
- 전방향 스캔
- 인쇄 크기: 최소 37.29mm × 25.93mm

**사용 시기:**
- 소매 판매 시점 스캔
- 소비자 대면 제품
- 대량 스캔 환경

### 2. GS1-128 바코드

**목적:** GTIN과 추가 데이터(배치, 날짜, 일련번호) 인코딩

**응용 식별자 (AIs):**
```
(01) GTIN: 01234567890128
(10) 배치/로트: LOT2025001
(17) 유효기간: 260630 (2026년 6월 30일)
(21) 일련번호: 123456

전체 바코드 데이터:
(01)01234567890128(10)LOT2025001(17)260630(21)123456
```

**식품 이력 추적을 위한 일반 AI:**

| AI | 데이터 | 형식 | 예시 |
|----|------|------|------|
| (01) | GTIN | N14 | 01234567890128 |
| (10) | 배치/로트 | X..20 | LOT2025001 |
| (11) | 생산일 | N6 (YYMMDD) | 251201 |
| (13) | 포장일 | N6 (YYMMDD) | 251202 |
| (15) | 권장 소비 기한 | N6 (YYMMDD) | 260601 |
| (17) | 유효기간 | N6 (YYMMDD) | 260630 |
| (21) | 일련번호 | X..20 | 123456 |
| (3103) | 순중량 (kg) | N6 | 001000 (100.0 kg) |

### 3. GS1 QR 코드

**목적:** 소비자 참여를 위한 모바일 친화적 2D 바코드

**데이터 구조:**
```
https://id.gs1.org/01/01234567890128/10/LOT2025001/21/123456

디코딩:
- GTIN: 01234567890128
- 배치: LOT2025001
- 일련번호: 123456
```

**혜택:**
- 스마트폰 스캔 (특별한 장비 불필요)
- 디지털 제품 정보 링크
- 소비자 이력 추적 접근
- 마케팅 및 참여

---

## GS1 식별자 구현

### 1단계: GS1 가입

1. 지역 GS1 회원 조직에 문의
2. 회사 접두사 구매
3. 고유 접두사 수령 (예: 0123456)

**비용:**
- 초기 비용: 30만원-1,000만원 (국가 및 접두사 크기에 따라 다름)
- 연간 갱신: 20만원-200만원

### 2단계: GTIN 할당

```javascript
// GTIN 할당 시스템 예시
class GTINManager {
  constructor(companyPrefix) {
    this.companyPrefix = companyPrefix; // 예: "0123456"
    this.itemCounter = 0;
  }

  assignGTIN(productName) {
    this.itemCounter++;

    const indicatorDigit = "0";
    const itemReference = this.itemCounter.toString().padStart(5, '0');

    const baseGTIN = indicatorDigit + this.companyPrefix + itemReference;
    const checkDigit = this.calculateCheckDigit(baseGTIN);

    const gtin = baseGTIN + checkDigit;

    // 데이터베이스에 저장
    this.saveGTIN({
      gtin: gtin,
      productName: productName,
      assignedDate: new Date().toISOString()
    });

    return gtin;
  }

  calculateCheckDigit(gtin13) {
    let sum = 0;
    for (let i = 0; i < 13; i++) {
      const digit = parseInt(gtin13[i]);
      sum += (i % 2 === 0) ? digit * 1 : digit * 3;
    }
    return ((10 - (sum % 10)) % 10).toString();
  }
}

// 사용법
const gtinManager = new GTINManager("0123456");
const appleGTIN = gtinManager.assignGTIN("유기농 사과 1kg");
// 반환값: "00123456000018"
```

---

## 이력 추적과의 통합

### 배치 식별 형식

GTIN과 로트 번호 결합:
```
형식: GTIN + 로트 번호
예시: 01234567890128.LOT2025001

데이터베이스 저장:
{
  "batchId": "01234567890128.LOT2025001",
  "gtin": "01234567890128",
  "lot": "LOT2025001"
}
```

### 이벤트 위치 식별

모든 위치 참조에 GLN 사용:
```json
{
  "eventId": "evt_001",
  "eventType": "shipping",
  "readPoint": {
    "id": "urn:epc:id:sgln:0123456.00001.0",
    "gln": "0123456000010",
    "name": "창고 도크 3"
  },
  "bizLocation": {
    "id": "urn:epc:id:sgln:0123456.00001.0",
    "gln": "0123456000010",
    "name": "본관 물류센터"
  }
}
```

---

## 모범 사례

### 1. 명명 표준화

**좋음:**
```
제품: 유기농 사과 1kg
GTIN: 00123456000018
```

**나쁨:**
```
제품: 사과-유기-1000g
GTIN: (수동 생성, 표준 없음)
```

### 2. GTIN 문서화

마스터 데이터베이스 유지:
```sql
CREATE TABLE product_catalog (
  gtin VARCHAR(14) PRIMARY KEY,
  product_name VARCHAR(255) NOT NULL,
  brand VARCHAR(100),
  category VARCHAR(100),
  net_content_value DECIMAL(10,3),
  net_content_unit VARCHAR(10),
  assigned_date DATE,
  status VARCHAR(20),
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

---

## 장 요약

GS1 표준은 식품 이력 추적을 위한 보편적 언어를 제공합니다:

**주요 식별자:**
- **GTIN:** 제품
- **GLN:** 위치
- **SSCC:** 물류 단위
- **GIAI:** 자산

**구현 단계:**
1. GS1 가입 및 회사 접두사 획득
2. 체계적으로 GTIN 할당
3. 모든 위치에 GLN 할당
4. 준수 바코드 생성
5. 모든 식별자 검증

**혜택:**
- 글로벌 상호운용성
- 자동화된 데이터 수집
- 규제 준수
- 공급망 효율성

---

## 다음 장

**제3장: EPCIS 이벤트 아키텍처**

공급망 전반에 걸쳐 이력 추적 이벤트를 캡처하고 공유하기 위한 GS1 EPCIS 구현 방법을 배웁니다.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
