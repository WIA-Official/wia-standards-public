# WIA-IND-010 Phase 4: Integration Specification
## Personalized Nutrition Standard
### 弘益人間 · Benefit All Humanity

---

## Overview

Phase 4 of WIA-IND-010 defines ecosystem integration guidelines for connecting personalized nutrition platforms with health platforms, wearable devices, genetic testing services, food delivery apps, and other ecosystem partners. These integrations create a comprehensive, seamless personalized nutrition experience.

## 1. Health Platform Integration

### 1.1 Apple Health (HealthKit) Integration

**Setup Requirements:**
- iOS app with HealthKit capability enabled
- Privacy usage description in Info.plist
- User authorization request

**Data Types to Read:**
```swift
let typesToRead: Set<HKObjectType> = [
    HKQuantityType.quantityType(forIdentifier: .activeEnergyBurned)!,
    HKQuantityType.quantityType(forIdentifier: .stepCount)!,
    HKQuantityType.quantityType(forIdentifier: .heartRate)!,
    HKQuantityType.quantityType(forIdentifier: .bodyMass)!,
    HKQuantityType.quantityType(forIdentifier: .bodyFatPercentage)!,
    HKQuantityType.quantityType(forIdentifier: .height)!,
    HKCategoryType.categoryType(forIdentifier: .sleepAnalysis)!
]
```

**Data Types to Write:**
```swift
let typesToWrite: Set<HKSampleType> = [
    HKQuantityType.quantityType(forIdentifier: .dietaryEnergyConsumed)!,
    HKQuantityType.quantityType(forIdentifier: .dietaryProtein)!,
    HKQuantityType.quantityType(forIdentifier: .dietaryCarbohydrates)!,
    HKQuantityType.quantityType(forIdentifier: .dietaryFatTotal)!,
    HKQuantityType.quantityType(forIdentifier: .dietaryFiber)!,
    HKQuantityType.quantityType(forIdentifier: .dietaryWater)!
]
```

**Writing Meal Data:**
```swift
let calorieQuantity = HKQuantity(unit: HKUnit.kilocalorie(),
                                   doubleValue: 450.0)
let calorieSample = HKQuantitySample(
    type: HKQuantityType.quantityType(forIdentifier: .dietaryEnergyConsumed)!,
    quantity: calorieQuantity,
    start: mealDate,
    end: mealDate
)

healthStore.save(calorieSample) { success, error in
    // Handle result
}
```

### 1.2 Google Fit Integration

**API Setup:**
- Enable Google Fit API in Google Cloud Console
- OAuth 2.0 client ID configuration
- Scopes: `FITNESS_ACTIVITY_READ`, `FITNESS_NUTRITION_WRITE`

**Reading Activity Data:**
```http
GET https://www.googleapis.com/fitness/v1/users/me/dataset:aggregate
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "aggregateBy": [{
    "dataTypeName": "com.google.calories.expended",
    "dataSourceId": "derived:com.google.calories.expended:com.google.android.gms:merge_calories_expended"
  }],
  "bucketByTime": { "durationMillis": 86400000 },
  "startTimeMillis": 1640592000000,
  "endTimeMillis": 1640678400000
}
```

**Writing Nutrition Data:**
```http
PATCH https://www.googleapis.com/fitness/v1/users/me/dataSources/{dataSourceId}/datasets/{datasetId}
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "dataSourceId": "raw:com.google.nutrition:com.wia.nutrition:meal_log",
  "point": [{
    "startTimeNanos": 1640620800000000000,
    "endTimeNanos": 1640620800000000000,
    "dataTypeName": "com.google.nutrition",
    "value": [
      { "fpVal": 450.0, "mapVal": [{"key": "meal_type", "value": {"stringVal": "lunch"}}] },
      { "fpVal": 35.0, "mapVal": [{"key": "nutrient", "value": {"stringVal": "protein"}}] }
    ]
  }]
}
```

### 1.3 Samsung Health Integration

**SDK Setup:**
```java
HealthDataStore mStore = new HealthDataStore(context, mConnectionListener);
mStore.connectService();
```

**Reading Step Count:**
```java
HealthDataResolver resolver = new HealthDataResolver(mStore, null);
Filter filter = Filter.greaterThanEquals(HealthConstants.StepCount.START_TIME, startTime);

HealthDataResolver.ReadRequest request = new ReadRequest.Builder()
    .setDataType(HealthConstants.StepCount.HEALTH_DATA_TYPE)
    .setFilter(filter)
    .build();

resolver.read(request).setResultListener(result -> {
    // Process step count data
});
```

## 2. Wearable Device Integration

### 2.1 Continuous Glucose Monitor (CGM)

**Dexcom G7 API:**
```http
POST https://api.dexcom.com/v2/oauth2/token
Content-Type: application/x-www-form-urlencoded

client_id={client_id}&
client_secret={client_secret}&
grant_type=authorization_code&
code={authorization_code}&
redirect_uri={redirect_uri}
```

**Getting Glucose Data:**
```http
GET https://api.dexcom.com/v2/users/self/egvs
Authorization: Bearer {access_token}
startDate=2025-12-27T00:00:00&
endDate=2025-12-27T23:59:59
```

**Response:**
```json
{
  "records": [
    {
      "systemTime": "2025-12-27T12:00:00",
      "displayTime": "2025-12-27T12:00:00",
      "value": 105,
      "trend": "flat",
      "trendRate": 0.5
    }
  ]
}
```

### 2.2 Fitness Trackers

**Fitbit API:**
```http
GET https://api.fitbit.com/1/user/-/activities/date/2025-12-27.json
Authorization: Bearer {access_token}
```

**Oura Ring API:**
```http
GET https://api.ouraring.com/v2/usercollection/daily_activity
Authorization: Bearer {access_token}
start_date=2025-12-27&
end_date=2025-12-27
```

**WHOOP API:**
```http
GET https://api.prod.whoop.com/developer/v1/activity/sleep/{sleep_id}
Authorization: Bearer {access_token}
```

## 3. Genetic Testing Service Integration

### 3.1 23andMe API

**Authorization Flow:**
```http
GET https://api.23andme.com/authorize/
?redirect_uri=https://yourapp.com/callback&
response_type=code&
client_id={client_id}&
scope=basic genomes
```

**Getting Genotype Data:**
```http
GET https://api.23andme.com/3/genotype/{profile_id}/
Authorization: Bearer {access_token}
locations=rs1801133,rs9939609,rs429358,rs7412,rs4988235
```

**Response:**
```json
{
  "id": "profile-123456",
  "genotype": {
    "rs1801133": "CT",
    "rs9939609": "AT",
    "rs429358": "CC",
    "rs7412": "CT",
    "rs4988235": "CC"
  }
}
```

### 3.2 AncestryDNA / MyHeritage

**File Upload Approach:**
- User downloads raw data file from provider
- User uploads file to WIA platform
- Platform parses 23andMe v3/v4/v5 format
- Extract nutrition-relevant SNPs
- Generate personalized recommendations

**Parser Example:**
```python
def parse_23andme_file(file_path):
    snps = {}
    with open(file_path, 'r') as f:
        for line in f:
            if line.startswith('#'):
                continue
            rsid, chromosome, position, genotype = line.strip().split('\t')
            snps[rsid] = genotype
    return snps
```

## 4. Microbiome Testing Integration

### 4.1 Viome API

**Submitting Sample:**
```http
POST https://api.viome.com/v1/samples
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "userId": "user-123456",
  "sampleType": "stool",
  "collectionDate": "2025-12-20",
  "kitId": "kit-789012"
}
```

**Getting Results:**
```http
GET https://api.viome.com/v1/samples/{sampleId}/results
Authorization: Bearer {access_token}
```

**Response:**
```json
{
  "sampleId": "sample-345678",
  "status": "completed",
  "microbiomeScore": 8.2,
  "diversity": {
    "shannon": 3.8,
    "richness": 167
  },
  "recommendations": {
    "superfoods": ["avocado", "blueberries", "walnuts"],
    "avoid": ["red meat", "processed sugar"],
    "supplements": ["prebiotic fiber", "omega-3"]
  }
}
```

## 5. Food Delivery Integration

### 5.1 Uber Eats / DoorDash Integration

**Menu Search with Nutrition Filter:**
```http
POST /api/v1/delivery/search
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "userId": "user-123456",
  "location": {
    "latitude": 37.7749,
    "longitude": -122.4194
  },
  "nutritionCriteria": {
    "maxCalories": 600,
    "minProtein_g": 30,
    "maxSodium_mg": 800
  },
  "cuisine": ["Mediterranean", "Asian"],
  "delivery": "uber_eats"
}
```

**Response:**
```json
{
  "restaurants": [
    {
      "name": "Mediterranean Kitchen",
      "delivery_service": "uber_eats",
      "menu_items": [
        {
          "name": "Grilled Salmon Bowl",
          "calories": 580,
          "protein_g": 42,
          "sodium_mg": 650,
          "matchScore": 0.95,
          "deepLink": "ubereats://menu/item/12345"
        }
      ]
    }
  ]
}
```

### 5.2 Meal Kit Services (HelloFresh, Blue Apron)

**Weekly Meal Kit Recommendation:**
```http
POST /api/v1/mealkit/recommend
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "userId": "user-123456",
  "weekStartDate": "2025-12-30",
  "servings": 2,
  "mealsPerWeek": 4,
  "dietaryPreferences": ["vegetarian"],
  "provider": "hellofresh"
}
```

**Auto-Ordering:**
Integration with meal kit provider APIs to automatically order recommended meals based on weekly meal plan.

## 6. Grocery Shopping Integration

### 6.1 Instacart API

**Creating Shopping List:**
```http
POST https://api.instacart.com/v1/lists
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "name": "WIA Nutrition Weekly Shopping",
  "items": [
    {"product_id": "12345", "quantity": 2, "name": "Organic Spinach"},
    {"product_id": "67890", "quantity": 1, "name": "Wild Caught Salmon"}
  ]
}
```

**Product Nutrition Lookup:**
```http
GET https://api.instacart.com/v1/products/{product_id}/nutrition
Authorization: Bearer {access_token}
```

### 6.2 Amazon Fresh Integration

**Barcode Scanning:**
```http
POST /api/v1/products/lookup
Content-Type: application/json

{
  "barcode": "012345678905",
  "marketplace": "amazon_fresh"
}
```

**Response with Alternatives:**
```json
{
  "product": {
    "name": "Brand X Whole Milk",
    "nutrition": {...},
    "healthScore": 6.5
  },
  "healthierAlternatives": [
    {
      "name": "Organic 2% Milk",
      "healthScore": 7.8,
      "reason": "Lower saturated fat, organic",
      "priceComparison": "+$1.50"
    }
  ]
}
```

## 7. Restaurant Menu Integration

### 7.1 Menu QR Code Scanning

**QR Code Format:**
```json
{
  "type": "wia-nutrition-menu",
  "restaurant": "Mediterranean Bistro",
  "menuId": "menu-123456",
  "dataUrl": "https://api.restaurant.com/menu/nutrition"
}
```

**Fetching Menu Nutrition:**
```http
GET https://api.restaurant.com/menu/nutrition
Accept: application/json
```

**Response with Personalized Recommendations:**
```json
{
  "menuItems": [
    {
      "name": "Grilled Chicken Salad",
      "nutrition": {...},
      "personalizedScore": 9.2,
      "alignmentReason": "High protein, aligns with weight loss goal, APOE e4 friendly (low saturated fat)"
    }
  ]
}
```

## 8. Clinical Lab Integration

### 8.1 Quest Diagnostics / LabCorp

**Order Test:**
```http
POST /api/v1/labs/order
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "userId": "user-123456",
  "provider": "quest",
  "testPanels": [
    "lipid_panel",
    "vitamin_d",
    "hba1c",
    "crp"
  ],
  "physicianNPI": "1234567890"
}
```

**Results Webhook:**
```http
POST https://yourapp.com/webhooks/lab-results
Content-Type: application/json

{
  "orderId": "order-789012",
  "userId": "user-123456",
  "status": "completed",
  "results": {
    "totalCholesterol_mgdL": 195,
    "ldlCholesterol_mgdL": 115,
    "hdlCholesterol_mgdL": 58,
    "triglycerides_mgdL": 110
  }
}
```

## 9. AI Assistant Integration

### 9.1 Voice Assistant (Alexa / Google Assistant)

**Alexa Skill:**
```json
{
  "intents": [
    {
      "name": "LogMealIntent",
      "slots": [
        {"name": "foodItem", "type": "AMAZON.Food"},
        {"name": "quantity", "type": "AMAZON.NUMBER"}
      ],
      "samples": [
        "I ate {quantity} {foodItem}",
        "Log {foodItem}",
        "I had {quantity} servings of {foodItem}"
      ]
    },
    {
      "name": "GetRecommendationIntent",
      "samples": [
        "What should I eat for dinner",
        "Recommend a meal",
        "What's good for lunch"
      ]
    }
  ]
}
```

**Google Assistant Action:**
```javascript
app.intent('Log Meal', (conv, {foodItem, quantity}) => {
  const mealLog = {
    userId: conv.user.id,
    foodItem: foodItem,
    quantity: quantity,
    timestamp: new Date().toISOString()
  };

  // Call WIA API to log meal
  await logMeal(mealLog);

  conv.ask(`I've logged ${quantity} ${foodItem}. Anything else?`);
});
```

## 10. Developer SDKs

### 10.1 JavaScript/TypeScript SDK

```typescript
import { WIANutritionClient } from '@wia/nutrition-sdk';

const client = new WIANutritionClient({
  apiKey: 'your-api-key',
  environment: 'production'
});

// Get personalized recommendations
const recommendations = await client.recommendations.getMeals({
  userId: 'user-123456',
  mealType: 'dinner',
  preferences: {
    cuisine: ['Mediterranean'],
    maxPrepTime: 30
  }
});

// Log meal
await client.meals.log({
  userId: 'user-123456',
  meal: {
    name: 'Grilled Salmon',
    calories: 450,
    protein_g: 42
  }
});
```

### 10.2 Python SDK

```python
from wia_nutrition import WIAClient

client = WIAClient(api_key='your-api-key')

# Analyze genetic data
genetic_recs = client.genetics.get_recommendations(
    user_id='user-123456'
)

# Generate meal plan
meal_plan = client.meal_plans.generate(
    user_id='user-123456',
    start_date='2025-12-30',
    duration_days=7
)
```

## 11. Webhooks and Event Notifications

### 11.1 Webhook Registration

```http
POST /api/v1/webhooks
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "url": "https://yourapp.com/webhooks/wia-nutrition",
  "events": [
    "meal.logged",
    "biomarker.updated",
    "recommendation.generated"
  ],
  "secret": "webhook-secret-key"
}
```

### 11.2 Event Payload

```json
{
  "eventId": "event-123456",
  "eventType": "meal.logged",
  "timestamp": "2025-12-27T12:30:00Z",
  "data": {
    "userId": "user-123456",
    "mealId": "meal-789012",
    "calories": 450,
    "mealType": "lunch"
  },
  "signature": "sha256=abcdef123456..."
}
```

## Conclusion

WIA-IND-010 Phase 4 integration specifications enable comprehensive ecosystem connectivity, creating seamless experiences across health platforms, wearables, genetic services, food delivery, grocery shopping, clinical labs, and AI assistants. Through standardized APIs, SDKs, and webhooks, developers can build integrated personalized nutrition solutions embodying the principle of 弘益人間 (Benefit All Humanity).

---

© 2025 SmileStory Inc. / WIA
弘益人間 · Benefit All Humanity
WIA-IND-010 v1.0
