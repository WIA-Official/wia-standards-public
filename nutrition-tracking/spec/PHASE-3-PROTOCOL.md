# WIA-MED-022 Nutrition Tracking Standard
## Phase 3: Protocol Specification

### Version: 1.0.0
### Status: Complete
### Last Updated: 2025-01-15

---

## 1. Communication Protocols

### 1.1 Transport Layer
- **Primary**: HTTPS (TLS 1.3)
- **WebSocket**: WSS for real-time updates
- **Fallback**: HTTPS long-polling

### 1.2 Data Serialization
- **Format**: JSON (UTF-8)
- **Alternative**: Protocol Buffers (for mobile)
- **Compression**: gzip, brotli

---

## 2. Authentication & Authorization

### 2.1 JWT Token Structure
```json
{
  "header": {
    "alg": "RS256",
    "typ": "JWT"
  },
  "payload": {
    "sub": "user_id",
    "email": "user@example.com",
    "role": "user",
    "iat": 1642248000,
    "exp": 1642251600
  }
}
```

### 2.2 OAuth 2.0 Flow
```
1. Authorization Request
   GET /oauth/authorize?client_id=...&redirect_uri=...

2. User Consent

3. Authorization Code
   Redirect to: callback?code=AUTH_CODE

4. Token Exchange
   POST /oauth/token
   grant_type=authorization_code&code=AUTH_CODE

5. Access Token Response
   {
     "access_token": "...",
     "refresh_token": "...",
     "expires_in": 3600
   }
```

---

## 3. Data Synchronization

### 3.1 Offline-First Strategy
```
1. Local Changes
   - Store in local DB
   - Mark as "pending_sync"

2. Online Recovery
   - Detect connectivity
   - Upload pending changes
   - Download server updates

3. Conflict Resolution
   - Last-write-wins (default)
   - Custom resolution for critical data
```

### 3.2 Sync Protocol
```
POST /sync
Authorization: Bearer {token}

Request:
{
  "since": "2025-01-15T10:00:00Z",
  "changes": [
    {
      "type": "meal_create",
      "data": { ... },
      "client_timestamp": "2025-01-15T12:30:00Z"
    }
  ]
}

Response:
{
  "sync_timestamp": "2025-01-15T14:00:00Z",
  "updates": [
    {
      "type": "goal_update",
      "data": { ... }
    }
  ],
  "conflicts": []
}
```

---

## 4. Real-Time Updates

### 4.1 WebSocket Connection
```
wss://api.nutrition-tracker.example.com/ws?token={jwt}

Client → Server:
{
  "type": "subscribe",
  "channels": ["nutrition_updates", "goals"]
}

Server → Client:
{
  "type": "nutrition_update",
  "data": {
    "meal_id": "uuid",
    "nutrition_summary": { ... }
  }
}
```

### 4.2 Event Types
- `meal_logged`: New meal entry
- `goal_updated`: Nutrition goal changed
- `daily_summary`: End of day summary
- `alert`: Important notification

---

## 5. Data Privacy & Security

### 5.1 Encryption
```
- In-Transit: TLS 1.3
- At-Rest: AES-256
- Field-Level: Sensitive data (passwords, medical info)
```

### 5.2 Data Access Control
```
User Permissions:
- READ_OWN: Access own data
- WRITE_OWN: Modify own data
- SHARE: Share with nutritionist/coach
- DELETE: Request data deletion (GDPR)

Nutritionist Permissions:
- READ_CLIENT: View assigned client data
- WRITE_PLAN: Create meal plans
- READ_ANALYTICS: Aggregate analytics
```

---

## 6. Integration Protocols

### 6.1 HealthKit (iOS)
```swift
import HealthKit

// Request Authorization
HKHealthStore().requestAuthorization(
  toShare: [
    HKQuantityType.quantityType(forIdentifier: .dietaryEnergyConsumed)!
  ],
  read: [
    HKQuantityType.quantityType(forIdentifier: .activeEnergyBurned)!
  ]
) { success, error in ... }

// Save Nutrition Data
let calorieType = HKQuantityType.quantityType(
  forIdentifier: .dietaryEnergyConsumed)!
let calorieQuantity = HKQuantity(
  unit: .kilocalorie(),
  doubleValue: 450)
let sample = HKQuantitySample(
  type: calorieType,
  quantity: calorieQuantity,
  start: Date(),
  end: Date())
healthStore.save(sample) { ... }
```

### 6.2 Google Fit (Android)
```kotlin
import com.google.android.gms.fitness.Fitness

// Connect
Fitness.getRecordingClient(context, account)
  .subscribe(DataType.TYPE_NUTRITION)

// Save Data
val dataSource = DataSource.Builder()
  .setAppPackageName(context)
  .setDataType(DataType.TYPE_NUTRITION)
  .setType(DataSource.TYPE_RAW)
  .build()

val dataPoint = DataPoint.builder(dataSource)
  .setField(Field.FIELD_NUTRIENTS, calories)
  .build()
```

### 6.3 FHIR (Healthcare Systems)
```json
{
  "resourceType": "NutritionOrder",
  "status": "active",
  "patient": {
    "reference": "Patient/123"
  },
  "dateTime": "2025-01-15T10:00:00Z",
  "oralDiet": {
    "nutrient": [
      {
        "modifier": {
          "coding": [{
            "system": "http://snomed.info/sct",
            "code": "2331003",
            "display": "Carbohydrate"
          }]
        },
        "amount": {
          "value": 200,
          "unit": "g"
        }
      }
    ]
  }
}
```

---

## 7. Quality of Service

### 7.1 Performance Targets
- API Response Time: p95 < 200ms
- Photo Analysis: < 2 seconds
- Sync Operation: < 1 second
- Uptime: 99.9%

### 7.2 Retry Policy
```
Exponential Backoff:
- Attempt 1: Immediate
- Attempt 2: +1 second
- Attempt 3: +2 seconds
- Attempt 4: +4 seconds
- Attempt 5: +8 seconds
- Max Attempts: 5
```

---

© 2025 WIA Standards - Benefit All Humanity (弘益人間)
