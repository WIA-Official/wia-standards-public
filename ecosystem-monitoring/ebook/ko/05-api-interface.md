# 제5장: API 인터페이스 사양

## 학습 목표

이 장을 마친 후 다음을 할 수 있게 됩니다:

1. RESTful API를 통해 생태계 모니터링 데이터 쿼리
2. 프로그래밍 방식으로 관찰 제출
3. WebSocket 및 MQTT를 사용하여 실시간 센서 데이터 스트리밍
4. 인증 구현 및 속도 제한 처리
5. 대량 데이터셋에 비동기적으로 액세스

---

## 5.1 RESTful API 사양

### 5.1.1 기본 URL 구조

```
https://api.{domain}/v1/
```

**예제:**
- `https://api.ecosystem-monitoring.org/v1/`
- `https://api.neon.org/wia/v1/`
- `https://api.myorganization.org/ecosystem/v1/`

**버전 관리:** URL 경로의 API 버전은 이전 버전과의 호환성을 유지하면서 향후 버전에서 주요 변경을 허용합니다.

### 5.1.2 핵심 엔드포인트

#### GET /observations

필터링을 통한 종 관찰 쿼리.

**요청 매개변수:**
```typescript
interface ObservationQueryParams {
  taxon?: string;              // 학명 (예: "Ursus arctos")
  start_date?: string;         // ISO 8601 (예: "2025-01-01")
  end_date?: string;           // ISO 8601 (예: "2025-12-31")
  bbox?: string;               // "minLon,minLat,maxLon,maxLat"
  limit?: number;              // 최대 레코드 (기본값: 100, 최대: 1000)
  offset?: number;             // 페이지네이션 오프셋 (기본값: 0)
  format?: 'json'|'csv'|'geojson'; // 응답 형식 (기본값: json)
  validation_status?: string;  // QC 상태로 필터링
  detection_method?: string;   // 방법으로 필터링
}
```

**예제 요청:**
```bash
curl "https://api.example.org/v1/observations?\
  taxon=Haliaeetus%20leucocephalus&\
  start_date=2025-01-01&\
  end_date=2025-12-31&\
  bbox=-125,40,-110,50&\
  limit=100&\
  format=json" \
  -H "Authorization: Bearer YOUR_API_KEY"
```

**응답 형식:**
```json
{
  "status": "success",
  "api_version": "1.0",
  "request_id": "req-7f8d9a2b-3c4e-5f6a-8b9c-0d1e2f3a4b5c",
  "timestamp": "2025-06-15T14:30:00Z",

  "query": {
    "taxon": "Haliaeetus leucocephalus",
    "start_date": "2025-01-01",
    "end_date": "2025-12-31",
    "bbox": "-125,40,-110,50",
    "limit": 100
  },

  "pagination": {
    "total_records": 1247,
    "returned_records": 100,
    "page": 1,
    "total_pages": 13,
    "next_page": "https://api.example.org/v1/observations?...&offset=100",
    "prev_page": null
  },

  "data": [
    {
      "wia_version": "1.0",
      "schema_type": "species-observation",
      "record_id": "550e8400-e29b-41d4-a716-446655440000",
      // ... 완전한 관찰 레코드
    },
    // ... 99개 더 많은 레코드
  ]
}
```

#### POST /observations

새 관찰 제출.

**요청 본문 (단일 관찰):**
```json
{
  "wia_version": "1.0",
  "schema_type": "species-observation",
  "record_id": "uuid-generated-by-client",
  "timestamp": "2025-06-15T14:30:00-07:00",
  // ... 완전한 관찰 필드
}
```

**요청 본문 (다중 관찰):**
```json
[
  { /* 관찰 1 */ },
  { /* 관찰 2 */ },
  { /* 관찰 3 */ }
]
```

**응답:**
```json
{
  "status": "success",
  "message": "3개 관찰 제출됨",
  "results": [
    {
      "record_id": "550e8400-e29b-41d4-a716-446655440000",
      "status": "accepted",
      "validation_status": "validated",
      "validation_warnings": []
    },
    {
      "record_id": "650e8400-e29b-41d4-a716-446655440111",
      "status": "accepted",
      "validation_status": "questionable",
      "validation_warnings": [
        "고도 250m가 이 위치에서 예상되는 185m와 다릅니다"
      ]
    },
    {
      "record_id": "750e8400-e29b-41d4-a716-446655440222",
      "status": "rejected",
      "validation_status": "invalid",
      "validation_errors": [
        "위도 95.5가 범위(-90 ~ 90)를 벗어났습니다"
      ]
    }
  ]
}
```

#### GET /observations/{id}

ID로 특정 관찰 검색.

**예제:**
```bash
curl "https://api.example.org/v1/observations/550e8400-e29b-41d4-a716-446655440000" \
  -H "Authorization: Bearer YOUR_API_KEY"
```

**응답:** 단일 관찰 객체 (`/observations` 배열과 동일한 형식).

#### GET /sensors

메타데이터와 함께 사용 가능한 센서 나열.

**응답:**
```json
{
  "status": "success",
  "data": [
    {
      "sensor_id": "WEATHER-STATION-042",
      "sensor_type": "기상 관측소",
      "location": {
        "latitude": 47.6815,
        "longitude": -121.7453,
        "elevation": 850
      },
      "deployment_date": "2025-01-20",
      "status": "active",
      "parameters": ["temperature", "humidity", "precipitation", "wind"],
      "data_url": "/v1/sensors/WEATHER-STATION-042/data"
    },
    // ... 더 많은 센서
  ]
}
```

#### GET /sensors/{id}/data

센서 시계열 데이터 검색.

**매개변수:**
```typescript
interface SensorDataParams {
  start_time: string;          // ISO 8601 타임스탬프
  end_time: string;            // ISO 8601 타임스탬프
  aggregation?: 'raw'|'hourly'|'daily'|'monthly'; // 기본값: raw
  format?: 'json'|'csv'|'netcdf'; // 기본값: json
  qc_filter?: 'good'|'good,questionable'|'all'; // 기본값: good
}
```

**예제:**
```bash
curl "https://api.example.org/v1/sensors/WEATHER-STATION-042/data?\
  start_time=2025-06-01T00:00:00Z&\
  end_time=2025-06-07T23:59:59Z&\
  aggregation=daily&\
  format=json" \
  -H "Authorization: Bearer YOUR_API_KEY"
```

---

## 5.2 인증

### 5.2.1 API 키 인증

서버 간 또는 개인 사용을 위한 가장 간단한 방법:

```bash
curl "https://api.example.org/v1/observations" \
  -H "Authorization: Bearer YOUR_API_KEY"
```

**API 키 얻기:**
1. https://api.example.org/register 에서 등록
2. 계정 대시보드에서 키 생성
3. 안전하게 저장 (Git에 커밋하지 않음)

**환경 변수:**
```bash
export WIA_API_KEY="your-api-key-here"
curl "https://api.example.org/v1/observations" \
  -H "Authorization: Bearer $WIA_API_KEY"
```

### 5.2.2 OAuth 2.0

사용자를 대신하여 데이터에 액세스하는 애플리케이션용:

**인증 코드 플로우:**
```typescript
// 1. 사용자를 인증 엔드포인트로 리디렉션
const authUrl = `https://api.example.org/oauth/authorize?
  client_id=${CLIENT_ID}&
  redirect_uri=${REDIRECT_URI}&
  response_type=code&
  scope=observations:read observations:write`;

window.location.href = authUrl;

// 2. 사용자 승인, 코드와 함께 리디렉션됨
// https://yourapp.com/callback?code=AUTH_CODE

// 3. 코드를 액세스 토큰으로 교환
const tokenResponse = await fetch('https://api.example.org/oauth/token', {
  method: 'POST',
  body: JSON.stringify({
    grant_type: 'authorization_code',
    code: AUTH_CODE,
    client_id: CLIENT_ID,
    client_secret: CLIENT_SECRET,
    redirect_uri: REDIRECT_URI
  })
});

const { access_token, refresh_token } = await tokenResponse.json();

// 4. API 요청에 액세스 토큰 사용
const observations = await fetch('https://api.example.org/v1/observations', {
  headers: { Authorization: `Bearer ${access_token}` }
});
```

### 5.2.3 JWT 토큰

서비스 간 인증용:

```typescript
import jwt from 'jsonwebtoken';

const token = jwt.sign(
  {
    sub: 'service-account-id',
    iss: 'your-organization',
    aud: 'ecosystem-monitoring-api',
    exp: Math.floor(Date.now() / 1000) + (60 * 60) // 1시간
  },
  PRIVATE_KEY,
  { algorithm: 'RS256' }
);

const response = await fetch('https://api.example.org/v1/observations', {
  headers: { Authorization: `Bearer ${token}` }
});
```

---

## 5.3 속도 제한

### 5.3.1 속도 제한 계층

| 계층 | 요청/시간 | 버스트 제한 | 사용 사례 |
|------|----------|-----------|---------|
| 익명 | 100 | 10/분 | 공개 쿼리, 테스트 |
| 인증됨 | 1,000 | 50/분 | 표준 사용자 |
| 프리미엄 | 10,000 | 200/분 | 대량 통합 |
| 엔터프라이즈 | 무제한 | 사용자 정의 | 대규모 조직 |

### 5.3.2 속도 제한 헤더

모든 응답에는 속도 제한 정보가 포함됩니다:

```http
HTTP/1.1 200 OK
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1735228800
X-RateLimit-Reset-Date: Thu, 26 Dec 2025 14:00:00 GMT
```

### 5.3.3 속도 제한 처리

```typescript
async function fetchWithRetry(url: string, maxRetries = 3) {
  for (let i = 0; i < maxRetries; i++) {
    const response = await fetch(url, {
      headers: { Authorization: `Bearer ${API_KEY}` }
    });

    if (response.status === 429) {
      // 속도 제한 초과
      const resetTime = parseInt(response.headers.get('X-RateLimit-Reset'));
      const waitSeconds = resetTime - Math.floor(Date.now() / 1000);

      console.log(`속도 제한 초과. ${waitSeconds}초 대기 중...`);
      await new Promise(resolve => setTimeout(resolve, waitSeconds * 1000));
      continue; // 재시도
    }

    if (response.ok) {
      return await response.json();
    }

    throw new Error(`HTTP ${response.status}: ${response.statusText}`);
  }

  throw new Error('최대 재시도 횟수 초과');
}
```

---

## 5.4 실시간 스트리밍 API

### 5.4.1 WebSocket 프로토콜

**스트림에 연결:**
```javascript
const ws = new WebSocket('wss://api.example.org/stream');

ws.addEventListener('open', () => {
  console.log('실시간 스트림에 연결됨');

  // 센서 구독
  ws.send(JSON.stringify({
    action: 'subscribe',
    sensors: ['WEATHER-001', 'WATER-QUALITY-042'],
    filters: {
      quality_min: 0.8,
      parameters: ['temperature', 'dissolved_oxygen']
    }
  }));
});

ws.addEventListener('message', (event) => {
  const data = JSON.parse(event.data);

  switch (data.type) {
    case 'sensor_reading':
      console.log(`${data.sensor_id}: ${data.value} ${data.unit}`);
      updateDashboard(data);
      break;

    case 'sensor_alert':
      console.warn(`경고: ${data.sensor_id} - ${data.message}`);
      sendNotification(data);
      break;

    case 'subscription_confirmed':
      console.log(`${data.sensors.length}개 센서 구독됨`);
      break;
  }
});

ws.addEventListener('error', (error) => {
  console.error('WebSocket 오류:', error);
});

ws.addEventListener('close', () => {
  console.log('스트림 연결 해제됨');
  // 재연결 로직 구현
});
```

**메시지 유형:**

```typescript
// 센서 판독값
{
  "type": "sensor_reading",
  "sensor_id": "WEATHER-001",
  "timestamp": "2025-06-15T14:30:00Z",
  "parameter": "temperature",
  "value": 18.5,
  "unit": "celsius",
  "qc_flag": "good"
}

// 경고
{
  "type": "sensor_alert",
  "sensor_id": "WATER-QUALITY-042",
  "timestamp": "2025-06-15T14:30:00Z",
  "alert_type": "threshold_exceeded",
  "parameter": "dissolved_oxygen",
  "value": 3.2,
  "threshold": 5.0,
  "message": "용존 산소가 임계값 이하입니다"
}

// 종 관찰 (카메라 트랩에서 실시간)
{
  "type": "observation",
  "record_id": "uuid",
  "schema_type": "species-observation",
  // ... 관찰 필드
}
```

### 5.4.2 MQTT 프로토콜

IoT 장치 및 저대역폭 시나리오용:

**토픽:**
```
sensors/{sensor_id}/data       # 센서 판독값
sensors/{sensor_id}/status     # 온라인/오프라인, 배터리 수준
sensors/{sensor_id}/alerts     # 임계값 초과
observations/{site_id}/species # 종 감지
```

**Python 예제:**
```python
import paho.mqtt.client as mqtt
import json

def on_connect(client, userdata, flags, rc):
    print(f"연결됨, 결과 코드 {rc}")
    # 사이트의 모든 센서 구독
    client.subscribe("sensors/SITE-042/+/data")

def on_message(client, userdata, msg):
    data = json.loads(msg.payload)
    print(f"{msg.topic}: {data['value']} {data['unit']}")

client = mqtt.Client()
client.username_pw_set("your_username", "your_password")
client.on_connect = on_connect
client.on_message = on_message

client.connect("mqtt.example.org", 1883, 60)
client.loop_forever()
```

**QoS 레벨:**
- **QoS 0** (최대 한 번): 빠르지만 메시지를 잃을 수 있음
- **QoS 1** (최소 한 번): 신뢰할 수 있는 전달, 중복 가능
- **QoS 2** (정확히 한 번): 보장된 전달, 느림

**권장 사항:**
- 센서 데이터: QoS 1 (신뢰할 수 있음, 중복은 필터링 가능)
- 경고: QoS 2 (중요, 중복되어서는 안 됨)
- 상태: QoS 0 (빈번한 업데이트, 하나 놓쳐도 괜찮음)

---

## 5.5 대량 데이터 액세스

### 5.5.1 비동기 쿼리 패턴

대용량 데이터셋(수백만 레코드)의 경우 비동기 쿼리 사용:

**1단계: 쿼리 제출**
```bash
curl -X POST "https://api.example.org/v1/bulk-query" \
  -H "Authorization: Bearer $API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "type": "observations",
    "filters": {
      "taxon": "Ursus arctos",
      "start_date": "2020-01-01",
      "end_date": "2024-12-31",
      "bbox": [-125, 40, -110, 50]
    },
    "format": "csv",
    "email_when_complete": true
  }'
```

**응답:**
```json
{
  "status": "accepted",
  "job_id": "job-7f8d9a2b-3c4e-5f6a-8b9c-0d1e2f3a4b5c",
  "estimated_records": 1234567,
  "estimated_completion": "2025-06-15T15:00:00Z",
  "status_url": "/v1/jobs/job-7f8d9a2b-3c4e-5f6a-8b9c-0d1e2f3a4b5c"
}
```

**2단계: 완료 확인**
```bash
curl "https://api.example.org/v1/jobs/job-7f8d9a2b-3c4e-5f6a-8b9c-0d1e2f3a4b5c" \
  -H "Authorization: Bearer $API_KEY"
```

**응답 (처리 중):**
```json
{
  "job_id": "job-7f8d9a2b-3c4e-5f6a-8b9c-0d1e2f3a4b5c",
  "status": "processing",
  "progress": 0.45,
  "records_processed": 555555,
  "estimated_completion": "2025-06-15T15:00:00Z"
}
```

**응답 (완료):**
```json
{
  "job_id": "job-7f8d9a2b-3c4e-5f6a-8b9c-0d1e2f3a4b5c",
  "status": "complete",
  "total_records": 1234567,
  "file_size_bytes": 125000000,
  "download_url": "https://downloads.example.org/job-7f8d9a2b.csv.gz",
  "expires_at": "2025-06-22T15:00:00Z",
  "checksum_sha256": "abc123..."
}
```

**3단계: 다운로드**
```bash
curl "https://downloads.example.org/job-7f8d9a2b.csv.gz" \
  -o grizzly-observations-2020-2024.csv.gz
```

### 5.5.2 미리 생성된 데이터 덤프

자주 요청되는 데이터셋의 경우 미리 생성된 덤프를 사용할 수 있습니다:

**사용 가능한 덤프 나열:**
```bash
curl "https://api.example.org/v1/dumps" \
  -H "Authorization: Bearer $API_KEY"
```

**응답:**
```json
{
  "dumps": [
    {
      "name": "all-observations-current-year",
      "description": "현재 연도의 모든 종 관찰",
      "format": "ndjson.gz",
      "size_bytes": 5000000000,
      "last_updated": "2025-06-15T00:00:00Z",
      "update_frequency": "daily",
      "download_url": "/v1/dumps/all-observations-2025.ndjson.gz"
    },
    {
      "name": "sensor-data-hourly",
      "description": "시간별로 집계된 모든 센서 데이터",
      "format": "csv.gz",
      "size_bytes": 2000000000,
      "last_updated": "2025-06-15T01:00:00Z",
      "update_frequency": "hourly",
      "download_url": "/v1/dumps/sensor-hourly-latest.csv.gz"
    }
  ]
}
```

---

## 5.6 오류 처리

### 5.6.1 오류 응답 형식

모든 오류는 일관된 JSON 구조를 반환합니다:

```json
{
  "status": "error",
  "error_code": "INVALID_PARAMETER",
  "message": "'bbox' 매개변수가 잘못되었습니다",
  "details": {
    "parameter": "bbox",
    "provided": "invalid-format",
    "expected": "minLon,minLat,maxLon,maxLat"
  },
  "request_id": "req-7f8d9a2b-3c4e-5f6a-8b9c-0d1e2f3a4b5c",
  "timestamp": "2025-06-15T14:30:00Z",
  "documentation": "https://docs.example.org/api/errors/invalid-parameter"
}
```

### 5.6.2 HTTP 상태 코드

| 상태 | 의미 | 일반적인 원인 |
|-----|-----|-------------|
| 200 | 성공 | 요청이 성공적으로 완료됨 |
| 201 | 생성됨 | 리소스 생성됨 (POST /observations) |
| 400 | 잘못된 요청 | 잘못된 매개변수, 잘못된 JSON |
| 401 | 미인증 | API 키 누락 또는 잘못됨 |
| 403 | 금지됨 | 유효한 자격 증명이지만 권한 불충분 |
| 404 | 찾을 수 없음 | 리소스가 존재하지 않음 |
| 429 | 요청 과다 | 속도 제한 초과 |
| 500 | 내부 서버 오류 | 서버 측 문제 |
| 503 | 서비스 사용 불가 | 유지보수 또는 과부하 |

### 5.6.3 오류 코드

```typescript
enum ErrorCode {
  INVALID_PARAMETER = "매개변수 값이 잘못되었거나 형식이 잘못됨",
  MISSING_REQUIRED = "필수 매개변수 누락",
  AUTHENTICATION_FAILED = "잘못된 API 키 또는 토큰",
  PERMISSION_DENIED = "작업에 대한 권한 불충분",
  RATE_LIMIT_EXCEEDED = "요청이 너무 많음",
  RESOURCE_NOT_FOUND = "요청된 리소스가 존재하지 않음",
  VALIDATION_FAILED = "데이터 검증 검사 실패",
  INTERNAL_ERROR = "예기치 않은 서버 오류",
  SERVICE_UNAVAILABLE = "서비스 일시적으로 사용 불가"
}
```

---

## 5.7 클라이언트 라이브러리

### 5.7.1 Python 클라이언트

```python
from wia import EcosystemMonitoring

# 클라이언트 초기화
client = EcosystemMonitoring(api_key='YOUR_API_KEY')

# 관찰 쿼리
observations = client.get_observations(
    taxon='Haliaeetus leucocephalus',
    start_date='2025-01-01',
    end_date='2025-12-31',
    bbox=(-125, 40, -110, 50),
    limit=1000
)

# pandas DataFrame으로 변환
df = client.to_dataframe(observations)

# 공간 분석을 위해 GeoDataFrame으로 변환
gdf = client.to_geodataframe(observations)

# 새 관찰 제출
client.submit_observation({
    'wia_version': '1.0',
    'schema_type': 'species-observation',
    # ... 관찰 필드
})

# 실시간 센서 데이터 스트리밍
def on_reading(data):
    print(f"{data['sensor_id']}: {data['value']}")

client.stream_sensors(
    sensors=['WEATHER-001'],
    callback=on_reading
)
```

### 5.7.2 R 클라이언트

```r
library(wiaR)

# 클라이언트 초기화
client <- wia_connect(api_key = "YOUR_API_KEY")

# 관찰 쿼리
obs <- get_observations(
  client,
  taxon = "Haliaeetus leucocephalus",
  start_date = "2025-01-01",
  end_date = "2025-12-31",
  bbox = c(-125, 40, -110, 50)
)

# 공간 분석을 위해 sf 객체로 변환
obs_sf <- wia_to_sf(obs)

# 관찰 플롯
library(ggplot2)
ggplot(obs_sf) +
  geom_sf(aes(color = life_stage)) +
  theme_minimal()

# 센서 데이터 가져오기
sensor_data <- get_sensor_data(
  client,
  sensor_id = "WEATHER-001",
  start_time = "2025-06-01T00:00:00Z",
  end_time = "2025-06-07T23:59:59Z",
  aggregation = "daily"
)
```

---

## 5.8 복습 질문

### 질문 1
국립공원 경계 내에서 지난 5년간 멸종위기종 (종 목록이 있음)의 모든 관찰을 찾는 쿼리를 설계하시오. 10,000개 이상의 결과에 대해 페이지네이션을 어떻게 처리하겠는가?

### 질문 2
실시간 데이터 스트리밍에 WebSocket과 MQTT를 언제 사용할지 설명하시오. 절충점은 무엇인가?

### 질문 3
애플리케이션이 속도 제한에 도달하고 있습니다. 필요한 모든 데이터에 계속 액세스하면서 제한 내에 머물기 위해 어떤 전략을 사용할 수 있는가?

### 질문 4
10,000개의 관찰을 효율적으로 제출하는 Python 코드를 작성하시오. 하나의 API 호출을 사용해야 하는가 아니면 여러 개를 사용해야 하는가? 부분 실패를 어떻게 처리하겠는가?

### 질문 5
대량 쿼리 작업이 레코드의 90%를 처리한 후 실패했습니다. 처음부터 시작하는 대신 작업을 재개할 수 있도록 API를 어떻게 설계하겠는가?

---

## 5.9 주요 요점

| 구성요소 | 핵심 사항 |
|---------|---------|
| **REST API** | 관찰 및 센서 데이터 쿼리, 제출, 검색 |
| **인증** | API 키(간단), OAuth(사용자 위임), JWT(서비스) |
| **속도 제한** | 시간당 100-10,000 요청, X-RateLimit 헤더 존중 |
| **실시간** | WebSocket(대시보드), MQTT(IoT 장치) |
| **대량 액세스** | 수백만 레코드에 대한 비동기 쿼리 |
| **오류 처리** | 일관된 형식, HTTP 상태 코드, 오류 코드 |
| **클라이언트 라이브러리** | 손쉬운 통합을 위한 Python, R 패키지 |

### 모범 사례
- 응답에서 항상 `validation_status` 확인
- 큰 결과 집합에 페이지네이션 사용
- 재시도를 위해 지수 백오프 구현
- API 키를 안전하게 저장 (환경 변수, 코드 아님)
- 속도 제한을 우아하게 처리
- 실시간에서 필요한 센서/토픽만 구독

### 다음 장 미리보기

6장은 Phase 3(프로토콜 사양)을 다루며, 데이터 품질 및 과학적 엄격성을 보장하는 QA/QC 절차, 보정 표준 및 현장 샘플링 프로토콜을 자세히 설명합니다.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
