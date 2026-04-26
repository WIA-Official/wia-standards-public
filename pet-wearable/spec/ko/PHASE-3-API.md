# WIA-PET-007 PHASE 3: API 표준

**버전:** 1.0.0  
**날짜:** 2025-12-25  
**상태:** 활성 표준

---

## 1. API 개요

PHASE 3는 WIA-PET-007 반려동물 웨어러블 생태계를 위한 RESTful API 표준, BLE 통신 프로토콜 및 데이터 교환 메커니즘을 정의합니다.

---

## 2. RESTful API 엔드포인트

### 2.1 기본 URL 구조

```
https://api.{provider}.com/v1/
```

### 2.2 인증

**OAuth 2.0 with PKCE**

```
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code&
code={authorization_code}&
redirect_uri={redirect_uri}&
client_id={client_id}&
code_verifier={code_verifier}
```

### 2.3 핵심 API 엔드포인트

| 메서드 | 엔드포인트 | 설명 |
|--------|-----------|-----|
| GET | /devices | 사용자 기기 목록 |
| GET | /devices/{id} | 기기 정보 가져오기 |
| GET | /devices/{id}/health | 건강 데이터 가져오기 |
| GET | /devices/{id}/activity | 활동 데이터 가져오기 |
| GET | /devices/{id}/location | 위치 데이터 가져오기 |
| POST | /devices/{id}/geofence | 지오펜스 생성 |
| PUT | /devices/{id}/settings | 설정 업데이트 |

---

## 3. 요청/응답 형식

### 3.1 성공 응답

```json
{
  "status": "success",
  "timestamp": "2025-12-25T15:30:00.000Z",
  "data": {
    "deviceId": "PW-DOG-12345",
    "healthMetrics": [ ... ]
  },
  "pagination": {
    "page": 1,
    "pageSize": 100,
    "totalPages": 3
  }
}
```

### 3.2 오류 응답

```json
{
  "status": "error",
  "timestamp": "2025-12-25T15:30:00.000Z",
  "error": {
    "code": "INVALID_GEOFENCE_RADIUS",
    "message": "지오펜스 반경은 50~5000미터 사이여야 합니다",
    "details": {
      "field": "radius",
      "providedValue": 25,
      "allowedRange": "50-5000"
    }
  }
}
```

---

## 4. HTTP 상태 코드

| 상태 코드 | 의미 | 사용 사례 |
|----------|-----|---------|
| 200 | OK | 성공적인 요청 |
| 201 | Created | 리소스 생성됨 |
| 400 | Bad Request | 잘못된 JSON, 누락된 필드 |
| 401 | Unauthorized | 잘못되거나 누락된 인증 토큰 |
| 404 | Not Found | 기기/리소스가 존재하지 않음 |
| 429 | Too Many Requests | 속도 제한 초과 |
| 500 | Internal Server Error | 서버 측 오류 |

---

## 5. 속도 제한

### 5.1 속도 제한 계층

| 엔드포인트 유형 | 무료 계층 | 프리미엄 계층 |
|---------------|---------|-------------|
| 기기 데이터 | 시간당 100회 | 시간당 1000회 |
| 이력 쿼리 | 시간당 20회 | 시간당 200회 |
| 위치 업데이트 | 일일 500회 | 일일 5000회 |

---

## 6. BLE 통신 프로토콜

### 6.1 GATT 서비스 UUID

| 서비스 | UUID |
|--------|------|
| 기기 정보 | 0x180A |
| 배터리 서비스 | 0x180F |
| 건강 데이터 서비스 | WIA-PET-001 |
| 활동 서비스 | WIA-PET-002 |
| 위치 서비스 | WIA-PET-003 |

**WIA 사용자 지정 UUID:**
- WIA-PET-001: `a0b1c2d3-e4f5-6789-abcd-ef0123456789`
- WIA-PET-002: `a0b1c2d3-e4f5-6789-abcd-ef0123456790`
- WIA-PET-003: `a0b1c2d3-e4f5-6789-abcd-ef0123456791`

### 6.2 건강 데이터 서비스 (WIA-PET-001)

**특성:**

| 특성 | UUID | 속성 | 설명 |
|-----|------|------|-----|
| 심박수 | 0x2A37 | Notify | bpm 단위 심박수 |
| 온도 | 0x2A6E | Read, Notify | 체온 |
| 호흡수 | WIA-custom | Read, Notify | 분당 호흡수 |

---

## 7. WebSocket 실시간 API

### 7.1 연결

```javascript
const ws = new WebSocket('wss://realtime.api.provider.com/v1/stream');
ws.send(JSON.stringify({
  action: 'subscribe',
  deviceId: 'PW-DOG-12345',
  channels: ['health', 'location', 'alerts']
}));
```

### 7.2 실시간 메시지 형식

```json
{
  "channel": "health",
  "deviceId": "PW-DOG-12345",
  "timestamp": "2025-12-25T15:30:00.000Z",
  "data": {
    "heartRate": {"value": 95, "unit": "bpm"}
  }
}
```

---

## 8. 데이터 내보내기 API

### 8.1 내보내기 요청

```
POST /exports
{
  "deviceId": "PW-DOG-12345",
  "dataTypes": ["health", "activity", "location"],
  "startDate": "2025-01-01",
  "endDate": "2025-12-31",
  "format": "json|csv|pdf"
}
```

---

## 9. FHIR 수의학 통합

### 9.1 FHIR 환자 리소스 (반려동물용 적용)

```json
{
  "resourceType": "Patient",
  "id": "PET-ABC-789",
  "extension": [
    {
      "url": "http://wia.org/fhir/StructureDefinition/pet-species",
      "valueString": "dog"
    }
  ],
  "name": [{"text": "맥스"}],
  "gender": "male",
  "birthDate": "2020-03-15"
}
```

---

**弘益人間 (홍익인간) · 널리 인간을 이롭게 하라**

© 2025 SmileStory Inc. / WIA  
WIA-PET-007 PHASE 3 사양
