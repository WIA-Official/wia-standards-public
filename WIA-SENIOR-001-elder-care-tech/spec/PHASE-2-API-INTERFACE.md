# WIA-SENIOR-001: Elder Care Technology Standard
## PHASE 2: API INTERFACE SPECIFICATION

> 弘益人間 (Benefit All Humanity)

**Version:** 1.0.0
**Base URL:** `https://api.wia.org/senior-001`

---

## 1. Authentication

### 1.1 API Key
```http
Authorization: Bearer YOUR_API_KEY
X-WIA-Standard: SENIOR-001
```

### 1.2 OAuth 2.0
Supported flows: Authorization Code, Client Credentials

---

## 2. Elder Profile Management

### 2.1 Create Elder Profile
```http
POST /elders
Content-Type: application/json

{
  "personalInfo": {...},
  "medicalInfo": {...},
  "carePreferences": {...}
}
```

### 2.2 Get Elder Profile
```http
GET /elders/{elderId}
```

### 2.3 Update Elder Profile
```http
PATCH /elders/{elderId}
```

### 2.4 Delete Elder Profile
```http
DELETE /elders/{elderId}
```

---

## 3. Vital Signs Monitoring

### 3.1 Record Vital Signs
```http
POST /vitals
Content-Type: application/json

{
  "elderId": "elder-123",
  "timestamp": "2025-01-15T08:00:00Z",
  "heartRate": {"bpm": 72, "rhythm": "regular"},
  "bloodPressure": {"systolic": 120, "diastolic": 80}
}
```

### 3.2 Get Vital Signs History
```http
GET /elders/{elderId}/vitals?startDate=2025-01-01&endDate=2025-01-15
```

### 3.3 Get Latest Vital Signs
```http
GET /elders/{elderId}/vitals/latest
```

---

## 4. Alert Management

### 4.1 Create Alert
```http
POST /alerts
```

### 4.2 Get Alerts
```http
GET /elders/{elderId}/alerts?resolved=false&severity=HIGH
```

### 4.3 Resolve Alert
```http
PATCH /alerts/{alertId}/resolve
```

---

## 5. Medication Management

### 5.1 Add Medication
```http
POST /medications
```

### 5.2 Get Medications
```http
GET /elders/{elderId}/medications
```

### 5.3 Record Medication Taken
```http
POST /medications/{medicationId}/taken
```

---

## 6. WebSocket Real-Time Events

### 6.1 Connection
```javascript
wss://api.wia.org/senior-001/ws?apiKey=YOUR_KEY
```

### 6.2 Event Types
- `vitals`: Vital signs update
- `alert`: New alert created
- `activity`: Activity recorded

---

## 7. Response Format

### Success
```json
{
  "success": true,
  "data": {...},
  "timestamp": "2025-01-15T08:00:00Z"
}
```

### Error
```json
{
  "success": false,
  "error": {
    "code": "INVALID_REQUEST",
    "message": "Elder ID is required"
  },
  "timestamp": "2025-01-15T08:00:00Z"
}
```

---

## 8. Rate Limiting

- Standard: 1000 requests/hour
- Premium: 10000 requests/hour
- Headers: `X-RateLimit-Limit`, `X-RateLimit-Remaining`

---

**Copyright:** © 2025 SmileStory Inc. / WIA
