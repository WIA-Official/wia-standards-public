# WIA-MED-022 Nutrition Tracking Standard
## Phase 4: Integration Specification

### Version: 1.0.0
### Status: Complete
### Last Updated: 2025-01-15

---

## 1. System Integration Overview

This phase defines how WIA-MED-022 integrates with external systems including healthcare platforms, wearable devices, and third-party services.

---

## 2. Healthcare System Integration

### 2.1 Electronic Medical Records (EMR)
```
HL7 v2 Message Format:

MSH|^~\&|NutritionApp|Hospital|EMR|Hospital|20250115100000||ORM^O01|MSG001|P|2.5
PID|1||123456||Doe^John||19900101|M
ORC|NW|ORD001|||||||20250115100000
OBR|1|ORD001||DIET^Diabetic Diet

Integration Points:
- Patient demographics sync
- Nutrition orders from dietitian
- Lab results for nutritional assessment
- Medication interactions check
```

### 2.2 FHIR R4 Compliance
```
Resource Mapping:
- Patient → user_profile
- Observation → nutrition_metrics
- NutritionOrder → meal_plan
- AllergyIntolerance → user_allergies
- Condition → medical_conditions
```

---

## 3. Wearable Device Integration

### 3.1 Apple Watch
```
Features:
- Quick meal logging via voice
- Water intake tracking
- Calorie burn sync (ActivityKit)
- Goal progress complications

WatchKit Implementation:
- Meal log interface
- Nutrition summary view
- Water tracking buttons
- Daily goal rings
```

### 3.2 Fitbit
```
API Integration:
- Endpoint: https://api.fitbit.com/1/user/-/foods/log.json
- Permissions: nutrition, weight

Data Exchange:
Fitbit → App:
  - Calories burned
  - Weight measurements
  - Sleep data
  - Activity levels

App → Fitbit:
  - Calories consumed
  - Water intake
  - Macro nutrients
```

### 3.3 Garmin
```
Garmin Health API:
- Activity data
- Heart rate
- Stress levels
- Sleep metrics

Integration:
- Adjust TDEE based on Garmin activity
- Update meal recommendations
- Correlate stress with eating patterns
```

---

## 4. Food Database Integration

### 4.1 USDA FoodData Central
```
API Configuration:
- Base URL: https://api.nal.usda.gov/fdc/v1/
- Auth: API Key in header
- Rate Limit: 1000 requests/hour

Endpoints Used:
- /foods/search - Food lookup
- /food/{fdcId} - Detailed nutrition
- /foods/list - Bulk retrieval

Data Sync:
- Initial: Full database import
- Incremental: Weekly updates
- Cache: 7-day TTL
```

### 4.2 Open Food Facts
```
API Configuration:
- Base URL: https://world.openfoodfacts.org/api/v2/
- No auth required (open source)
- Rate Limit: Unlimited (be reasonable)

Barcode Lookup:
GET /product/{barcode}.json

Data Mapping:
- product_name → food.name
- nutriments → food.nutrition
- allergens → food.allergens
- image_url → food.photo_url
```

### 4.3 Korean Food Database
```
Source: 식품의약품안전처
- Food composition database
- ~6,000 Korean food items
- Korean traditional dishes

Integration Method:
- CSV import
- Monthly updates
- Manual verification for quality
```

---

## 5. AI Service Integration

### 5.1 Image Recognition API
```
Service: TensorFlow Serving / Custom API

Request:
POST /v1/models/food-recognition:predict
Content-Type: application/json

{
  "instances": [
    {
      "image_bytes": "base64_encoded_image"
    }
  ]
}

Response:
{
  "predictions": [
    {
      "class": "grilled_chicken",
      "confidence": 0.92,
      "bounding_box": [x, y, w, h],
      "portion_grams": 180
    }
  ]
}
```

### 5.2 Barcode Scanning
```
Service: Google ML Kit / ZXing

Implementation:
- Real-time camera feed processing
- Barcode detection (EAN-13, UPC-A, QR)
- Product lookup in databases
- Fallback to manual entry
```

---

## 6. Third-Party Service Integration

### 6.1 Payment Processing
```
Stripe Integration:
- Subscription management
- Premium feature access
- One-time purchases (meal plans)

Endpoints:
- /api/billing/subscribe
- /api/billing/manage
- /api/billing/cancel
```

### 6.2 Cloud Storage
```
AWS S3 Configuration:
- Meal photos
- User profile pictures
- Export files (PDF reports)

Settings:
- Encryption: AES-256
- Lifecycle: 90 days for unused photos
- CDN: CloudFront for global access
```

### 6.3 Analytics
```
Google Analytics 4:
- User engagement metrics
- Feature usage tracking
- Goal achievement rates
- Retention analysis

Custom Events:
- meal_logged
- goal_set
- photo_scanned
- barcode_scanned
```

---

## 7. Export & Interoperability

### 7.1 Data Export Formats
```
CSV Export:
date,meal_type,food_name,quantity,calories,protein,carbs,fat
2025-01-15,breakfast,Oatmeal,50g,180,6,30,3

JSON Export:
{
  "export_date": "2025-01-15",
  "date_range": {
    "start": "2025-01-01",
    "end": "2025-01-15"
  },
  "meals": [ ... ],
  "summary": { ... }
}

PDF Report:
- Formatted nutrition summary
- Charts and graphs
- Recommendations
- Goal progress
```

### 7.2 Import Support
```
Supported Formats:
- MyFitnessPal CSV
- Lose It! JSON
- Cronometer CSV
- Apple Health XML

Import Process:
1. Upload file
2. Validate format
3. Map fields
4. Preview import
5. Confirm & import
6. Conflict resolution
```

---

## 8. Webhook Integration

### 8.1 Webhook Configuration
```
POST /api/webhooks
Authorization: Bearer {token}

{
  "url": "https://your-server.com/webhook",
  "events": ["meal.created", "goal.updated"],
  "secret": "webhook_secret_key"
}
```

### 8.2 Webhook Payload
```
POST {webhook_url}
X-WIA-Signature: sha256=hash
Content-Type: application/json

{
  "event": "meal.created",
  "timestamp": "2025-01-15T12:30:00Z",
  "data": {
    "meal_id": "uuid",
    "user_id": "uuid",
    "nutrition_summary": { ... }
  }
}
```

---

## 9. Migration Tools

### 9.1 Database Migration
```
Supported Sources:
- MyFitnessPal
- Lose It!
- Cronometer
- Noom

Migration Steps:
1. Export from source app
2. Upload to migration tool
3. Automatic field mapping
4. Data validation
5. Preview & confirm
6. Import into WIA-MED-022
```

---

## 10. Compliance & Standards

### 10.1 Supported Standards
- FHIR R4 (Healthcare)
- HL7 v2 (Legacy EMR)
- OAuth 2.0 (Authentication)
- OpenID Connect (Identity)
- GDPR (Data Privacy)
- HIPAA (Health Data, US)
- PIPA (Personal Info, Korea)

### 10.2 Certification
- WIA-MED-022 Certified
- SOC 2 Type II
- ISO 27001
- HITRUST

---

© 2025 WIA Standards - Benefit All Humanity (弘益人間)
