# 제5장: API 인터페이스 사양

## 학습 목표

이 장을 완료하면 다음을 수행할 수 있습니다:

1. WIA 전자폐기물 API 인증 및 권한 부여 구현
2. 기기 등록 및 추적 엔드포인트 설계
3. 수거 및 처리 이벤트 API 구축
4. 재료 회수 보고 인터페이스 생성
5. 규정 준수 및 보고 API 통합 개발

---

## 5.1 API 아키텍처 개요

### 5.1.1 RESTful API 설계

```
WIA 전자폐기물 API 아키텍처:
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│  클라이언트 애플리케이션                                             │
│  ├─ 생산자 시스템                                                  │
│  ├─ 수거 지점 앱                                                   │
│  ├─ 처리 시설 ERP                                                  │
│  ├─ 규제 포털                                                      │
│  └─ 소비자 모바일 앱                                               │
│                                                                     │
│         │                                                           │
│         ▼                                                           │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                    API 게이트웨이                            │   │
│  │  ├─ 속도 제한                                               │   │
│  │  ├─ 인증                                                    │   │
│  │  ├─ 요청 라우팅                                             │   │
│  │  └─ 로깅                                                    │   │
│  └─────────────────────────────────────────────────────────────┘   │
│         │                                                           │
│         ▼                                                           │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                    API 서비스                                │   │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐       │   │
│  │  │ 기기     │ │ 수거     │ │ 처리     │ │ 규정준수 │       │   │
│  │  │ 레지스트리│ │ 서비스  │ │ 서비스   │ │ 서비스   │       │   │
│  │  └──────────┘ └──────────┘ └──────────┘ └──────────┘       │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 5.1.2 기본 URL 및 버전 관리

```typescript
// API 기본 구성
const apiConfig = {
  production: "https://api.wia-ewaste.org/v1",
  sandbox: "https://sandbox.wia-ewaste.org/v1",
  korea: "https://api.wia-ewaste.kr/v1",  // 한국 지역 엔드포인트

  versioning: {
    current: "v1",
    supported: ["v1"],
    deprecated: [],
    sunset: []
  },

  contentType: "application/json",
  acceptedFormats: ["application/json", "application/xml"],

  rateLimit: {
    basic: { requests: 100, period: "minute" },
    standard: { requests: 1000, period: "minute" },
    premium: { requests: 10000, period: "minute" }
  }
};
```

### 5.1.3 공통 응답 형식

```typescript
// 표준 API 응답 래퍼
interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    messageKo?: string;  // 한국어 메시지
    details?: any;
  };
  meta: {
    requestId: string;
    timestamp: string;
    version: string;
  };
  pagination?: {
    page: number;
    pageSize: number;
    totalItems: number;
    totalPages: number;
    hasNext: boolean;
    hasPrevious: boolean;
  };
}

// 성공 응답 예시
{
  "success": true,
  "data": {
    "deviceId": "WIA-SAMSUNG-SM-2024-A1B2C3D4-7X",
    "status": "registered"
  },
  "meta": {
    "requestId": "req-abc123",
    "timestamp": "2025-01-15T10:30:00Z",
    "version": "v1"
  }
}

// 오류 응답 예시
{
  "success": false,
  "error": {
    "code": "DEVICE_NOT_FOUND",
    "message": "Device with specified ID does not exist",
    "messageKo": "지정된 ID의 기기가 존재하지 않습니다",
    "details": {
      "deviceId": "WIA-INVALID-ID"
    }
  },
  "meta": {
    "requestId": "req-xyz789",
    "timestamp": "2025-01-15T10:30:00Z",
    "version": "v1"
  }
}
```

---

## 5.2 인증 및 권한 부여

### 5.2.1 OAuth 2.0 구현

```typescript
// OAuth 2.0 구성
interface AuthConfig {
  authorizationEndpoint: "https://auth.wia-ewaste.org/oauth/authorize";
  tokenEndpoint: "https://auth.wia-ewaste.org/oauth/token";

  grantTypes: ["client_credentials", "authorization_code", "refresh_token"];

  scopes: {
    "device:read": "기기 정보 읽기";
    "device:write": "기기 등록 및 업데이트";
    "collection:read": "수거 이벤트 읽기";
    "collection:write": "수거 이벤트 제출";
    "processing:read": "처리 데이터 읽기";
    "processing:write": "처리 이벤트 제출";
    "material:read": "재료 회수 데이터 읽기";
    "material:write": "재료 회수 데이터 제출";
    "compliance:read": "규정 준수 보고서 읽기";
    "compliance:write": "규정 준수 보고서 제출";
    "admin": "관리 접근";
  };
}

// 토큰 요청 예시
async function getAccessToken(clientId: string, clientSecret: string): Promise<TokenResponse> {
  const response = await fetch("https://auth.wia-ewaste.org/oauth/token", {
    method: "POST",
    headers: {
      "Content-Type": "application/x-www-form-urlencoded",
      "Authorization": `Basic ${btoa(`${clientId}:${clientSecret}`)}`
    },
    body: new URLSearchParams({
      grant_type: "client_credentials",
      scope: "device:read device:write collection:write"
    })
  });

  return response.json();
}
```

### 5.2.2 역할 기반 접근 제어

| 역할 | 스코프 | 설명 |
|-----|-------|-----|
| 생산자 | device:write, device:read, compliance:read | 기기 등록, 규정 준수 조회 |
| 수거업체 | collection:write, device:read | 수거 이벤트 제출 |
| 처리업체 | processing:write, material:write, device:read | 처리 데이터 제출 |
| 규제기관 | *:read, compliance:write | 전체 읽기, 규정 준수 검증 |
| 소비자 | device:read (본인 것) | 재활용 인증서 조회 |
| 감사자 | *:read | 읽기 전용 감사 접근 |

---

## 5.3 기기 레지스트리 API

### 5.3.1 기기 등록 엔드포인트

```typescript
// POST /devices - 새 기기 등록
interface RegisterDeviceRequest {
  producer: {
    id: string;
    name: string;
    model: string;
    modelNumber: string;
  };
  category: {
    wiaCategory: string;
    koreaEPRCategory?: string;  // 한국 EPR 품목 분류
  };
  production: {
    date: string;
    facility: string;
    batchNumber?: string;
    serialNumber?: string;
  };
  physical?: {
    weightKg: number;
    dimensions?: object;
  };
  materials: MaterialComposition;
  hazardous?: HazardousSubstances;
}

// 응답
interface RegisterDeviceResponse {
  deviceId: string;              // 할당된 WDID
  registrationDate: string;
  status: "registered";
  qrCode: string;                // QR 코드 이미지 URL
  verificationUrl: string;       // 소비자 검증 URL
}
```

### 5.3.2 기기 조회 예시

```bash
# 새 기기 등록
curl -X POST https://api.wia-ewaste.kr/v1/devices \
  -H "Authorization: Bearer {token}" \
  -H "Content-Type: application/json" \
  -d '{
    "producer": {
      "id": "SAMSUNG-KR",
      "name": "삼성전자",
      "model": "Galaxy S24",
      "modelNumber": "SM-S921N"
    },
    "category": {
      "wiaCategory": "SM",
      "koreaEPRCategory": "휴대폰"
    },
    "production": {
      "date": "2024-01-15",
      "facility": "삼성전자 구미공장",
      "batchNumber": "2024-01-001"
    },
    "physical": {
      "weightKg": 0.187
    },
    "materials": {
      "totalWeightKg": 0.187,
      "aggregates": {
        "metals": {"ferrous": 0.025, "nonFerrousBase": 0.042}
      }
    }
  }'

# ID로 기기 조회
curl -X GET https://api.wia-ewaste.kr/v1/devices/WIA-SAMSUNG-SM-2024-A1B2C3D4-7X \
  -H "Authorization: Bearer {token}"

# 생산자별 기기 검색
curl -X GET "https://api.wia-ewaste.kr/v1/devices?producer=SAMSUNG-KR&from=2024-01-01&to=2024-12-31" \
  -H "Authorization: Bearer {token}"
```

---

## 5.4 수거 API

### 5.4.1 수거 이벤트 엔드포인트

```typescript
// POST /collections - 수거 이벤트 제출
interface CollectionEventRequest {
  collectionPoint: {
    id: string;
    type: "retail" | "municipal" | "producer_program" | "private";
  };

  // 개별 기기 추적용
  device?: {
    deviceId?: string;           // WDID (알려진 경우)
    serialNumber?: string;       // 대체 조회
    imei?: string;               // 휴대폰용
  };

  // 배치 수거용
  batch?: {
    category: string;
    quantity?: number;
    weightKg: number;
    description?: string;
  };

  condition: {
    functional: boolean;
    physicalCondition: "excellent" | "good" | "fair" | "poor" | "damaged";
    completeness: "complete" | "missing_accessories" | "missing_parts";
    batteryPresent?: boolean;
    dataWiped?: boolean;
  };

  consumer?: {
    anonymousId?: string;
    email?: string;              // 인증서 전달용
    certificateRequested: boolean;
  };

  routing: {
    recommendation: "reuse" | "refurbishment" | "recycling";
    destinationFacility?: string;
  };

  verification: {
    method: "visual" | "functional_test" | "automated";
    operatorId: string;
    photos?: string[];           // 증거 사진 URL
  };
}

// 응답
interface CollectionEventResponse {
  eventId: string;
  collectionId: string;          // 공개 참조 번호
  deviceId?: string;
  timestamp: string;

  status: "accepted" | "pending_verification";

  certificate?: {
    id: string;
    url: string;
    qrCode: string;
  };

  nextSteps: {
    routing: string;
    estimatedProcessingDate?: string;
  };
}
```

### 5.4.2 수거 지점 관리

```typescript
// GET /collection-points - 수거 지점 목록
// GET /collection-points/{id} - 수거 지점 상세
// POST /collection-points - 새 수거 지점 등록

interface CollectionPoint {
  id: string;
  name: string;
  type: "retail" | "municipal" | "producer_program" | "private" | "mobile";

  location: {
    address: string;
    city: string;
    postalCode: string;
    country: string;
    coordinates: {
      latitude: number;
      longitude: number;
    };
  };

  operator: {
    entityId: string;
    name: string;
    contact: string;
  };

  capabilities: {
    acceptedCategories: string[];
    maxWeightKg?: number;
    hasScanning: boolean;
    hasFunctionalTesting: boolean;
    hasSecureDataWipe: boolean;
  };

  operatingHours: {
    [day: string]: { open: string; close: string } | "closed";
  };

  certifications: string[];
  status: "active" | "inactive" | "suspended";
}

// 인근 수거 지점 검색
// GET /collection-points/nearby?lat={lat}&lng={lng}&radius={km}
```

---

## 5.5 처리 API

### 5.5.1 처리 이벤트 엔드포인트

```typescript
// POST /processing/events - 처리 이벤트 제출
interface ProcessingEventRequest {
  facilityId: string;
  processType: "receiving" | "sorting" | "disassembly" | "shredding" | "separation" | "refining";

  input: {
    batchIds?: string[];
    deviceIds?: string[];
    fromEventId?: string;        // 이전 이벤트 링크
    totalWeightKg: number;
    categoryBreakdown?: { category: string; weightKg: number }[];
  };

  output: {
    fractions: OutputFraction[];
    waste?: { type: string; weightKg: number; disposalMethod: string }[];
  };

  processMetrics?: {
    startTime: string;
    endTime: string;
    energyKwh?: number;
    waterLiters?: number;
  };

  qualityControl?: {
    samplingDone: boolean;
    contaminationPercent?: number;
    notes?: string;
  };
}

// POST /processing/material-recovery - 재료 회수 데이터 제출
interface MaterialRecoveryRequest {
  facilityId: string;
  reportingPeriod: { from: string; to: string };

  input: {
    totalWeightKg: number;
    byCategory: { category: string; weightKg: number }[];
    sourceBatches: string[];
  };

  recovery: {
    materials: {
      materialCode: string;
      materialName: string;
      weightKg: number;
      purity: number;
      qualityCertification?: string;
      buyerId?: string;
      revenue?: number;
    }[];

    reusedComponents: {
      type: string;
      quantity: number;
      estimatedValue?: number;
    }[];
  };

  residuals: {
    hazardous: { type: string; weightKg: number; treatment: string }[];
    nonHazardous: { type: string; weightKg: number; disposal: string }[];
  };

  metrics: {
    overallRecoveryRate: number;
    recyclingRate: number;
    reuseRate: number;
    disposalRate: number;
    co2Avoided?: number;
  };
}
```

---

## 5.6 규정 준수 API

### 5.6.1 한국 EPR 보고서 제출

```typescript
// POST /compliance/reports/korea-epr - 한국 EPR 보고서 제출
interface KoreaEPRReportSubmission {
  reportType: "annual" | "quarterly";
  reportingPeriod: { from: string; to: string };

  producerId: string;

  출고량: {
    byCategory: {
      koreaEPRCode: string;    // 한국 EPR 품목 코드
      category: string;
      quantity: number;
      weightKg: number;
    }[];
  };

  수거실적: {
    byCategory: {
      koreaEPRCode: string;
      collectedKg: number;
      method: "직접" | "위탁" | "공제조합";
    }[];
  };

  재활용실적: {
    byCategory: {
      koreaEPRCode: string;
      recycledKg: number;
      recoveryRate: number;
    }[];
  };

  certification: {
    certifiedBy: string;
    title: string;
    date: string;
    digitalSignature?: string;
  };
}

// 응답
interface ComplianceReportResponse {
  reportId: string;
  status: "submitted" | "under_review" | "approved" | "rejected";
  submissionDate: string;
  acknowledgementNumber: string;

  validation: {
    valid: boolean;
    warnings?: string[];
    errors?: string[];
  };

  complianceStatus: {
    overall: "compliant" | "partial" | "non_compliant";
    details: {
      category: string;
      obligationKg: number;
      achievedKg: number;
      rate: number;
      status: string;
    }[];
  };
}

// GET /compliance/verify/{entityId} - 엔티티 규정 준수 상태 확인
```

---

## 5.7 웹훅 통합

### 5.7.1 이벤트 알림

```typescript
// 웹훅 구성
interface WebhookConfig {
  url: string;
  events: WebhookEvent[];
  secret: string;              // 서명 검증용
  active: boolean;
  retryPolicy: {
    maxRetries: 3;
    backoffSeconds: [10, 60, 300];
  };
}

type WebhookEvent =
  | "device.registered"
  | "device.collected"
  | "device.processed"
  | "device.lifecycle_closed"
  | "batch.created"
  | "batch.transferred"
  | "material.recovered"
  | "compliance.due"
  | "compliance.submitted"
  | "certification.expiring"
  | "alert.triggered";

// 웹훅 페이로드
interface WebhookPayload {
  id: string;
  event: WebhookEvent;
  timestamp: string;
  data: object;
  signature: string;           // HMAC-SHA256
}

// POST /webhooks - 웹훅 등록
// GET /webhooks - 웹훅 목록
// PUT /webhooks/{id} - 웹훅 업데이트
// DELETE /webhooks/{id} - 웹훅 제거
// POST /webhooks/{id}/test - 테스트 이벤트 전송
```

---

## 5.8 SDK 예시

### 5.8.1 TypeScript SDK

```typescript
// WIA 전자폐기물 SDK 사용
import { WiaEwasteClient } from "@wia/ewaste-sdk";

const client = new WiaEwasteClient({
  apiKey: process.env.WIA_API_KEY,
  environment: "production",
  region: "korea"
});

// 기기 등록
async function registerDevice() {
  const device = await client.devices.create({
    producer: {
      id: "SAMSUNG-KR",
      name: "삼성전자",
      model: "Galaxy S24"
    },
    category: {
      wiaCategory: "SM",
      koreaEPRCategory: "휴대폰"
    },
    production: {
      date: "2024-01-15",
      facility: "삼성전자 구미"
    },
    materials: {
      totalWeightKg: 0.187
    }
  });

  console.log(`등록된 기기: ${device.deviceId}`);
  return device;
}

// 수거 이벤트 제출
async function submitCollection(deviceId: string) {
  const collection = await client.collections.create({
    collectionPoint: { id: "CP-001", type: "retail" },
    device: { deviceId },
    condition: {
      functional: false,
      physicalCondition: "fair"
    },
    routing: { recommendation: "recycling" }
  });

  return collection;
}
```

---

## 5.9 복습 문제

### 문제 1
수거업체에서 기기를 수령하고 재료 회수 보고서를 제출해야 하는 재활용 시설에 대한 OAuth 2.0 스코프 세트를 설계하시오. 어떤 스코프가 필요하고 그 이유는?

### 문제 2
동일한 생산 런에서 1,000개의 동일한 노트북 컴퓨터 배치를 등록하는 API 요청을 작성하시오. 모든 필수 필드를 포함하시오.

### 문제 3
수거 지점이 기기가 자신의 시설로 라우팅될 때 실시간 알림을 받고자 합니다. 웹훅 구성 및 핸들러를 설계하시오.

### 문제 4
수거부터 최종 재료 회수까지 기기를 추적하기 위한 API 호출 시퀀스를 생성하시오. 모든 상태 확인을 포함하시오.

### 문제 5
민감한 비즈니스 데이터를 노출하지 않고 재활용 인증서 정보를 반환하는 공개 기기 검증용 API 엔드포인트를 설계하시오.

---

## 5.10 핵심 요약

| API 범주 | 주요 엔드포인트 | 주요 사용자 |
|---------|--------------|----------|
| 기기 레지스트리 | /devices, /devices/{id} | 생산자 |
| 수거 | /collections, /collection-points | 수거업체 |
| 처리 | /processing/events, /material-recovery | 처리업체 |
| 규정 준수 | /compliance/reports, /verify | 전체, 규제기관 |
| 웹훅 | /webhooks | 통합자 |

### API 모범 사례
- 대량 작업에는 **배치 엔드포인트** 사용
- 실시간 통합에는 **웹훅** 구현
- 적절한 경우 **응답 캐시**
- 지수 백오프로 **속도 제한 처리**
- 모든 웹훅 페이로드에서 **서명 검증**

### 다음 장 미리보기

제6장에서는 관리 연속성, 재료 취급, 유해물질 관리 및 규정 준수 검증 절차를 위한 운영 프로토콜을 다룹니다.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - 널리 인간을 이롭게 하라
