# 제3장: EPCIS 이벤트 아키텍처

**WIA-AGRI-016 전자책 시리즈**

---

## EPCIS 소개

EPCIS(전자 제품 코드 정보 서비스)는 공급망 전반에 걸쳐 가시성 이벤트 데이터를 캡처하고 공유하기 위한 GS1 표준입니다. 표준화되고 상호운용 가능한 형식으로 공급망 활동을 설명하는 공통 언어를 제공합니다.

---

## EPCIS의 5W

모든 EPCIS 이벤트는 다음에 답합니다:

1. **무엇 (WHAT):** 어떤 물체/제품이 관련되는가?
2. **언제 (WHEN):** 언제 발생했는가?
3. **어디서 (WHERE):** 어디서 발생했는가?
4. **왜 (WHY):** 어떤 비즈니스 단계가 수행되었는가?
5. **어떻게 (HOW):** 어떤 조건에서?

---

## EPCIS 이벤트 유형

### 1. ObjectEvent (객체 이벤트)

**목적:** 부모-자식 관계 없이 객체의 가시성 추적

**사용 사례:**
- 농산물 수확
- 선적 수령
- 품질 검사
- 소매 판매

**예시: 수확 이벤트 (JSON)**

```json
{
  "type": "ObjectEvent",
  "eventTime": "2025-12-01T08:30:00Z",
  "eventTimeZoneOffset": "+09:00",
  "eventID": "urn:uuid:550e8400-e29b-41d4-a716-446655440000",

  "epcList": [
    "urn:epc:id:sgtin:0123456.789012.LOT2025001"
  ],

  "action": "ADD",
  "bizStep": "commissioning",
  "disposition": "active",

  "readPoint": {
    "id": "urn:epc:id:sgln:0123456.00001.FIELD3"
  },

  "bizLocation": {
    "id": "urn:epc:id:sgln:0123456.00001.0"
  },

  "quantityList": [
    {
      "epcClass": "urn:epc:class:lgtin:0123456.789012.LOT2025001",
      "quantity": 1000,
      "uom": "KGM"
    }
  ],

  "ilmd": {
    "harvestDate": "2025-12-01",
    "variety": "부사",
    "field": "북쪽 3번 밭",
    "temperature": 18,
    "temperatureUnit": "CEL"
  }
}
```

### 2. AggregationEvent (집계 이벤트)

**목적:** 부모-자식 관계 기록 (포장 계층)

**사용 사례:**
- 개별 품목을 케이스에 포장
- 케이스 팔레타이징
- 컨테이너 적재
- 팔레트 해체/포장 해제

### 3. TransformationEvent (변환 이벤트)

**목적:** 입력 재료가 출력 제품으로 변환되는 추적

**사용 사례:**
- 주스 압착 (사과 → 주스)
- 조리/베이킹
- 성분 혼합
- 모든 가공 작업

**예시: 사과 주스 생산**

```json
{
  "type": "TransformationEvent",
  "eventTime": "2025-12-05T10:30:00Z",
  "eventTimeZoneOffset": "+09:00",
  "transformationID": "urn:uuid:transform-20251205-001",

  "inputQuantityList": [
    {
      "epcClass": "urn:epc:class:lgtin:0123456.789012.LOT2025001",
      "quantity": 500,
      "uom": "KGM",
      "productName": "유기농 사과"
    },
    {
      "epcClass": "urn:epc:class:lgtin:0123456.789013.LOT2025020",
      "quantity": 200,
      "uom": "KGM",
      "productName": "유기농 배"
    }
  ],

  "outputQuantityList": [
    {
      "epcClass": "urn:epc:class:lgtin:0123456.789020.LOT2025100",
      "quantity": 650,
      "uom": "LTR",
      "productName": "유기농 사과-배 주스"
    }
  ],

  "bizStep": "commissioning",
  "disposition": "in_progress",

  "bizLocation": {
    "id": "urn:epc:id:sgln:0123456.00002.PROCESSING"
  },

  "processParameters": {
    "temperature": 72,
    "temperatureUnit": "CEL",
    "duration": 15,
    "durationUnit": "SEC",
    "process": "살균"
  }
}
```

### 4. TransactionEvent (거래 이벤트)

**목적:** 비즈니스 거래(구매 주문서, 송장)를 물리적 상품과 연결

**사용 사례:**
- 구매 주문서에 대한 출하
- 송장에 대한 수령
- 반품/크레딧
- 소유권 이전

### 5. AssociationEvent (연관 이벤트, EPCIS 2.0)

**목적:** 객체 간의 연관 관리 (예: 제품과 RTI)

**사용 사례:**
- 제품을 재사용 가능한 용기에 연결
- 센서를 선적과 연관
- 반환 가능한 운송 품목(RTI) 추적

---

## 비즈니스 단계 및 상태

### 일반적인 비즈니스 단계 (bizStep)

| 비즈니스 단계 | 설명 | 일반적인 이벤트 |
|---------------|------|----------------|
| commissioning | 새 객체 생성 | 수확, 생산 |
| receiving | 상품 수용 | 창고 수령 |
| shipping | 상품 발송 | 출고 선적 |
| packing | 포장 계층 생성 | 팔레타이징 |
| retail_selling | 판매 시점 | 소비자 구매 |
| storing | 보관 배치 | 창고 보관 |
| transporting | 위치 간 이동 | 운송 중 |

### 일반적인 상태 (disposition)

| 상태 | 설명 |
|------|------|
| active | 객체가 활성/사용 가능 |
| in_progress | 처리 진행 중 |
| in_transit | 운송 중 |
| container_closed | 포장 및 밀봉됨 |
| damaged | 제품 손상 |
| expired | 유효기간 경과 |
| recalled | 리콜 대상 |
| destroyed | 파기/폐기 |

---

## EPCIS 구현

### 아키텍처

```
┌──────────────────┐
│  애플리케이션     │  (ERP, WMS, QC 시스템)
└────────┬─────────┘
         │
┌────────▼─────────┐
│  EPCIS 캡처      │  (이벤트 생성)
│  인터페이스      │
└────────┬─────────┘
         │
┌────────▼─────────┐
│  EPCIS 저장소    │  (이벤트 저장 및 쿼리)
└────────┬─────────┘
         │
┌────────▼─────────┐
│  EPCIS 쿼리      │  (이벤트 검색)
│  인터페이스      │
└────────┬─────────┘
         │
┌────────▼─────────┐
│  외부 시스템     │  (거래 파트너)
└──────────────────┘
```

### 캡처 인터페이스

```javascript
class EPCISCapture {
  constructor(repositoryUrl) {
    this.repositoryUrl = repositoryUrl;
  }

  async captureObjectEvent(event) {
    const epcisEvent = {
      "@context": [
        "https://ref.gs1.org/standards/epcis/2.0.0/epcis-context.jsonld"
      ],
      "type": "EPCISDocument",
      "schemaVersion": "2.0",
      "creationDate": new Date().toISOString(),
      "epcisBody": {
        "eventList": [event]
      }
    };

    const response = await fetch(`${this.repositoryUrl}/capture`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(epcisEvent)
    });

    if (!response.ok) {
      throw new Error(`EPCIS 캡처 실패: ${response.statusText}`);
    }

    return await response.json();
  }
}

// 사용법
const capture = new EPCISCapture('https://epcis.example.com');

await capture.captureObjectEvent({
  type: "ObjectEvent",
  eventTime: "2025-12-01T08:30:00Z",
  eventTimeZoneOffset: "+09:00",
  epcList: ["urn:epc:id:sgtin:0123456.789012.LOT2025001"],
  action: "ADD",
  bizStep: "commissioning",
  disposition: "active",
  readPoint: {
    id: "urn:epc:id:sgln:0123456.00001.FIELD3"
  }
});
```

---

## 모범 사례

### 1. 이벤트 세분화

**좋음:** 의미 있는 비즈니스 이벤트 캡처
```
- 수확 완료
- 고객 출하
- 품질 검사 통과
```

**나쁨:** 과도하게 세분화된 이벤트
```
- 작업자 수확 시작
- 작업자 휴식
- 작업자 수확 재개
- 작업자 수확 완료
```

### 2. 일관된 타임스탬프

- 항상 ISO 8601 형식 사용
- 시간대 오프셋 포함
- 국제 운영에는 UTC 사용
- 시설 전반에 걸쳐 시계 동기화

---

## 장 요약

EPCIS는 이력 추적 이벤트를 캡처하고 공유하는 표준 프레임워크를 제공합니다:

**5가지 이벤트 유형:**
1. ObjectEvent - 객체 가시성
2. AggregationEvent - 포장 계층
3. TransformationEvent - 가공
4. TransactionEvent - 비즈니스 문서
5. AssociationEvent - 객체 연관

**주요 구성 요소:**
- 무엇, 언제, 어디서, 왜, 어떻게
- 비즈니스 단계 및 상태
- 제품 속성을 위한 ILMD
- 센서 데이터 통합

---

## 다음 장

**제4장: 불변 기록을 위한 블록체인 통합**

EPCIS를 블록체인 기술과 결합하여 변조 방지 이력 추적 기록을 만드는 방법을 배웁니다.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
