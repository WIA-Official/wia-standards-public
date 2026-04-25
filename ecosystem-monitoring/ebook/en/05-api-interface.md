# Chapter 5: API Interface Specification

## Learning Objectives

After completing this chapter, you will be able to:

1. Query ecosystem monitoring data via RESTful APIs
2. Submit observations programmatically
3. Stream real-time sensor data using WebSocket and MQTT
4. Implement authentication and handle rate limiting
5. Access bulk datasets asynchronously

---

## 5.1 RESTful API Specification

### 5.1.1 Base URL Structure

```
https://api.{domain}/v1/
```

**Examples:**
- `https://api.ecosystem-monitoring.org/v1/`
- `https://api.neon.org/wia/v1/`
- `https://api.myorganization.org/ecosystem/v1/`

**Versioning:** API version in URL path allows breaking changes in future versions while maintaining backward compatibility.

### 5.1.2 Core Endpoints

#### GET /observations

Query species observations with filtering.

**Request Parameters:**
```typescript
interface ObservationQueryParams {
  taxon?: string;              // Scientific name (e.g., "Ursus arctos")
  start_date?: string;         // ISO 8601 (e.g., "2025-01-01")
  end_date?: string;           // ISO 8601 (e.g., "2025-12-31")
  bbox?: string;               // "minLon,minLat,maxLon,maxLat"
  limit?: number;              // Max records (default: 100, max: 1000)
  offset?: number;             // Pagination offset (default: 0)
  format?: 'json'|'csv'|'geojson'; // Response format (default: json)
  validation_status?: string;  // Filter by QC status
  detection_method?: string;   // Filter by method
}
```

**Example Request:**
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

**Response Format:**
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
      // ... complete observation record
    },
    // ... 99 more records
  ]
}
```

#### POST /observations

Submit new observation(s).

**Request Body (Single Observation):**
```json
{
  "wia_version": "1.0",
  "schema_type": "species-observation",
  "record_id": "uuid-generated-by-client",
  "timestamp": "2025-06-15T14:30:00-07:00",
  // ... complete observation fields
}
```

**Request Body (Multiple Observations):**
```json
[
  { /* observation 1 */ },
  { /* observation 2 */ },
  { /* observation 3 */ }
]
```

**Response:**
```json
{
  "status": "success",
  "message": "3 observations submitted",
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
        "Elevation 250m differs from expected 185m at this location"
      ]
    },
    {
      "record_id": "750e8400-e29b-41d4-a716-446655440222",
      "status": "rejected",
      "validation_status": "invalid",
      "validation_errors": [
        "Latitude 95.5 out of range (-90 to 90)"
      ]
    }
  ]
}
```

#### GET /observations/{id}

Retrieve specific observation by ID.

**Example:**
```bash
curl "https://api.example.org/v1/observations/550e8400-e29b-41d4-a716-446655440000" \
  -H "Authorization: Bearer YOUR_API_KEY"
```

**Response:** Single observation object (same format as in `/observations` array).

#### GET /sensors

List available sensors with metadata.

**Response:**
```json
{
  "status": "success",
  "data": [
    {
      "sensor_id": "WEATHER-STATION-042",
      "sensor_type": "Meteorological station",
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
    // ... more sensors
  ]
}
```

#### GET /sensors/{id}/data

Retrieve sensor time series data.

**Parameters:**
```typescript
interface SensorDataParams {
  start_time: string;          // ISO 8601 timestamp
  end_time: string;            // ISO 8601 timestamp
  aggregation?: 'raw'|'hourly'|'daily'|'monthly'; // default: raw
  format?: 'json'|'csv'|'netcdf'; // default: json
  qc_filter?: 'good'|'good,questionable'|'all'; // default: good
}
```

**Example:**
```bash
curl "https://api.example.org/v1/sensors/WEATHER-STATION-042/data?\
  start_time=2025-06-01T00:00:00Z&\
  end_time=2025-06-07T23:59:59Z&\
  aggregation=daily&\
  format=json" \
  -H "Authorization: Bearer YOUR_API_KEY"
```

---

## 5.2 Authentication

### 5.2.1 API Key Authentication

Simplest method for server-to-server or personal use:

```bash
curl "https://api.example.org/v1/observations" \
  -H "Authorization: Bearer YOUR_API_KEY"
```

**Obtaining an API Key:**
1. Register at https://api.example.org/register
2. Generate key in account dashboard
3. Store securely (never commit to Git)

**Environment Variable:**
```bash
export WIA_API_KEY="your-api-key-here"
curl "https://api.example.org/v1/observations" \
  -H "Authorization: Bearer $WIA_API_KEY"
```

### 5.2.2 OAuth 2.0

For applications accessing data on behalf of users:

**Authorization Code Flow:**
```typescript
// 1. Redirect user to authorization endpoint
const authUrl = `https://api.example.org/oauth/authorize?
  client_id=${CLIENT_ID}&
  redirect_uri=${REDIRECT_URI}&
  response_type=code&
  scope=observations:read observations:write`;

window.location.href = authUrl;

// 2. User approves, redirected back with code
// https://yourapp.com/callback?code=AUTH_CODE

// 3. Exchange code for access token
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

// 4. Use access token for API requests
const observations = await fetch('https://api.example.org/v1/observations', {
  headers: { Authorization: `Bearer ${access_token}` }
});
```

### 5.2.3 JWT Tokens

For service-to-service authentication:

```typescript
import jwt from 'jsonwebtoken';

const token = jwt.sign(
  {
    sub: 'service-account-id',
    iss: 'your-organization',
    aud: 'ecosystem-monitoring-api',
    exp: Math.floor(Date.now() / 1000) + (60 * 60) // 1 hour
  },
  PRIVATE_KEY,
  { algorithm: 'RS256' }
);

const response = await fetch('https://api.example.org/v1/observations', {
  headers: { Authorization: `Bearer ${token}` }
});
```

---

## 5.3 Rate Limiting

### 5.3.1 Rate Limit Tiers

| Tier | Requests/Hour | Burst Limit | Use Case |
|------|---------------|-------------|----------|
| Anonymous | 100 | 10/minute | Public queries, testing |
| Authenticated | 1,000 | 50/minute | Standard users |
| Premium | 10,000 | 200/minute | High-volume integrations |
| Enterprise | Unlimited | Custom | Large organizations |

### 5.3.2 Rate Limit Headers

Every response includes rate limit information:

```http
HTTP/1.1 200 OK
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1735228800
X-RateLimit-Reset-Date: Thu, 26 Dec 2025 14:00:00 GMT
```

### 5.3.3 Handling Rate Limits

```typescript
async function fetchWithRetry(url: string, maxRetries = 3) {
  for (let i = 0; i < maxRetries; i++) {
    const response = await fetch(url, {
      headers: { Authorization: `Bearer ${API_KEY}` }
    });

    if (response.status === 429) {
      // Rate limit exceeded
      const resetTime = parseInt(response.headers.get('X-RateLimit-Reset'));
      const waitSeconds = resetTime - Math.floor(Date.now() / 1000);

      console.log(`Rate limit exceeded. Waiting ${waitSeconds} seconds...`);
      await new Promise(resolve => setTimeout(resolve, waitSeconds * 1000));
      continue; // Retry
    }

    if (response.ok) {
      return await response.json();
    }

    throw new Error(`HTTP ${response.status}: ${response.statusText}`);
  }

  throw new Error('Max retries exceeded');
}
```

---

## 5.4 Real-Time Streaming API

### 5.4.1 WebSocket Protocol

**Connect to Stream:**
```javascript
const ws = new WebSocket('wss://api.example.org/stream');

ws.addEventListener('open', () => {
  console.log('Connected to real-time stream');

  // Subscribe to sensors
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
      console.warn(`Alert: ${data.sensor_id} - ${data.message}`);
      sendNotification(data);
      break;

    case 'subscription_confirmed':
      console.log(`Subscribed to ${data.sensors.length} sensors`);
      break;
  }
});

ws.addEventListener('error', (error) => {
  console.error('WebSocket error:', error);
});

ws.addEventListener('close', () => {
  console.log('Disconnected from stream');
  // Implement reconnection logic
});
```

**Message Types:**

```typescript
// Sensor reading
{
  "type": "sensor_reading",
  "sensor_id": "WEATHER-001",
  "timestamp": "2025-06-15T14:30:00Z",
  "parameter": "temperature",
  "value": 18.5,
  "unit": "celsius",
  "qc_flag": "good"
}

// Alert
{
  "type": "sensor_alert",
  "sensor_id": "WATER-QUALITY-042",
  "timestamp": "2025-06-15T14:30:00Z",
  "alert_type": "threshold_exceeded",
  "parameter": "dissolved_oxygen",
  "value": 3.2,
  "threshold": 5.0,
  "message": "Dissolved oxygen below critical threshold"
}

// Species observation (real-time from camera traps)
{
  "type": "observation",
  "record_id": "uuid",
  "schema_type": "species-observation",
  // ... observation fields
}
```

### 5.4.2 MQTT Protocol

For IoT devices and low-bandwidth scenarios:

**Topics:**
```
sensors/{sensor_id}/data       # Sensor readings
sensors/{sensor_id}/status     # Online/offline, battery level
sensors/{sensor_id}/alerts     # Threshold exceedances
observations/{site_id}/species # Species detections
```

**Python Example:**
```python
import paho.mqtt.client as mqtt
import json

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    # Subscribe to all sensors at a site
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

**QoS Levels:**
- **QoS 0** (At most once): Fast but may lose messages
- **QoS 1** (At least once): Reliable delivery, possible duplicates
- **QoS 2** (Exactly once): Guaranteed delivery, slower

**Recommendation:**
- Sensor data: QoS 1 (reliable, duplicates can be filtered)
- Alerts: QoS 2 (critical, must not duplicate)
- Status: QoS 0 (frequent updates, okay to miss one)

---

## 5.5 Bulk Data Access

### 5.5.1 Asynchronous Query Pattern

For large datasets (millions of records), use asynchronous queries:

**Step 1: Submit Query**
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

**Response:**
```json
{
  "status": "accepted",
  "job_id": "job-7f8d9a2b-3c4e-5f6a-8b9c-0d1e2f3a4b5c",
  "estimated_records": 1234567,
  "estimated_completion": "2025-06-15T15:00:00Z",
  "status_url": "/v1/jobs/job-7f8d9a2b-3c4e-5f6a-8b9c-0d1e2f3a4b5c"
}
```

**Step 2: Poll for Completion**
```bash
curl "https://api.example.org/v1/jobs/job-7f8d9a2b-3c4e-5f6a-8b9c-0d1e2f3a4b5c" \
  -H "Authorization: Bearer $API_KEY"
```

**Response (Processing):**
```json
{
  "job_id": "job-7f8d9a2b-3c4e-5f6a-8b9c-0d1e2f3a4b5c",
  "status": "processing",
  "progress": 0.45,
  "records_processed": 555555,
  "estimated_completion": "2025-06-15T15:00:00Z"
}
```

**Response (Complete):**
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

**Step 3: Download**
```bash
curl "https://downloads.example.org/job-7f8d9a2b.csv.gz" \
  -o grizzly-observations-2020-2024.csv.gz
```

### 5.5.2 Pre-Generated Data Dumps

For frequently requested datasets, pre-generated dumps are available:

**Listing Available Dumps:**
```bash
curl "https://api.example.org/v1/dumps" \
  -H "Authorization: Bearer $API_KEY"
```

**Response:**
```json
{
  "dumps": [
    {
      "name": "all-observations-current-year",
      "description": "All species observations for current year",
      "format": "ndjson.gz",
      "size_bytes": 5000000000,
      "last_updated": "2025-06-15T00:00:00Z",
      "update_frequency": "daily",
      "download_url": "/v1/dumps/all-observations-2025.ndjson.gz"
    },
    {
      "name": "sensor-data-hourly",
      "description": "All sensor data aggregated to hourly",
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

## 5.6 Error Handling

### 5.6.1 Error Response Format

All errors return consistent JSON structure:

```json
{
  "status": "error",
  "error_code": "INVALID_PARAMETER",
  "message": "The 'bbox' parameter is malformed",
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

### 5.6.2 HTTP Status Codes

| Status | Meaning | Common Causes |
|--------|---------|---------------|
| 200 | Success | Request completed successfully |
| 201 | Created | Resource created (POST /observations) |
| 400 | Bad Request | Invalid parameters, malformed JSON |
| 401 | Unauthorized | Missing or invalid API key |
| 403 | Forbidden | Valid credentials but insufficient permissions |
| 404 | Not Found | Resource doesn't exist |
| 429 | Too Many Requests | Rate limit exceeded |
| 500 | Internal Server Error | Server-side problem |
| 503 | Service Unavailable | Maintenance or overload |

### 5.6.3 Error Codes

```typescript
enum ErrorCode {
  INVALID_PARAMETER = "Parameter value invalid or malformed",
  MISSING_REQUIRED = "Required parameter missing",
  AUTHENTICATION_FAILED = "Invalid API key or token",
  PERMISSION_DENIED = "Insufficient permissions for operation",
  RATE_LIMIT_EXCEEDED = "Too many requests",
  RESOURCE_NOT_FOUND = "Requested resource doesn't exist",
  VALIDATION_FAILED = "Data failed validation checks",
  INTERNAL_ERROR = "Unexpected server error",
  SERVICE_UNAVAILABLE = "Service temporarily unavailable"
}
```

---

## 5.7 Client Libraries

### 5.7.1 Python Client

```python
from wia import EcosystemMonitoring

# Initialize client
client = EcosystemMonitoring(api_key='YOUR_API_KEY')

# Query observations
observations = client.get_observations(
    taxon='Haliaeetus leucocephalus',
    start_date='2025-01-01',
    end_date='2025-12-31',
    bbox=(-125, 40, -110, 50),
    limit=1000
)

# Convert to pandas DataFrame
df = client.to_dataframe(observations)

# Convert to GeoDataFrame for spatial analysis
gdf = client.to_geodataframe(observations)

# Submit new observation
client.submit_observation({
    'wia_version': '1.0',
    'schema_type': 'species-observation',
    # ... observation fields
})

# Stream real-time sensor data
def on_reading(data):
    print(f"{data['sensor_id']}: {data['value']}")

client.stream_sensors(
    sensors=['WEATHER-001'],
    callback=on_reading
)
```

### 5.7.2 R Client

```r
library(wiaR)

# Initialize client
client <- wia_connect(api_key = "YOUR_API_KEY")

# Query observations
obs <- get_observations(
  client,
  taxon = "Haliaeetus leucocephalus",
  start_date = "2025-01-01",
  end_date = "2025-12-31",
  bbox = c(-125, 40, -110, 50)
)

# Convert to sf object for spatial analysis
obs_sf <- wia_to_sf(obs)

# Plot observations
library(ggplot2)
ggplot(obs_sf) +
  geom_sf(aes(color = life_stage)) +
  theme_minimal()

# Get sensor data
sensor_data <- get_sensor_data(
  client,
  sensor_id = "WEATHER-001",
  start_time = "2025-06-01T00:00:00Z",
  end_time = "2025-06-07T23:59:59Z",
  aggregation = "daily"
)
```

---

## 5.8 Review Questions

### Question 1
Design a query to find all observations of endangered species (you have a list of species) within a national park boundary over the last 5 years. How would you handle pagination for 10,000+ results?

### Question 2
Explain when you would use WebSocket vs. MQTT for real-time data streaming. What are the trade-offs?

### Question 3
Your application is hitting rate limits. What strategies can you use to stay within limits while still accessing all needed data?

### Question 4
Write Python code to submit 10,000 observations efficiently. Should you use one API call or multiple? How do you handle partial failures?

### Question 5
A bulk query job failed after processing 90% of records. How would you design the API to allow resuming the job instead of starting over?

---

## 5.9 Key Takeaways

| Component | Key Points |
|-----------|------------|
| **REST API** | Query, submit, retrieve observations and sensor data |
| **Authentication** | API keys (simple), OAuth (user-delegated), JWT (service) |
| **Rate Limiting** | 100-10,000 requests/hour, respect X-RateLimit headers |
| **Real-Time** | WebSocket (dashboards), MQTT (IoT devices) |
| **Bulk Access** | Asynchronous queries for millions of records |
| **Error Handling** | Consistent format, HTTP status codes, error codes |
| **Client Libraries** | Python, R packages for easy integration |

### Best Practices
- Always check `validation_status` in responses
- Use pagination for large result sets
- Implement exponential backoff for retries
- Store API keys securely (environment variables, not code)
- Handle rate limits gracefully
- Subscribe only to needed sensors/topics in real-time

### Next Chapter Preview

Chapter 6 covers Phase 3 (Protocol Specification), detailing QA/QC procedures, calibration standards, and field sampling protocols that ensure data quality and scientific rigor.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity
