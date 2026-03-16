# WIA-IND-010 Phase 2: API Interface Specification
## Personalized Nutrition Standard
### 弘益人間 · Benefit All Humanity

---

## Overview

Phase 2 of WIA-IND-010 defines RESTful API interfaces for nutrition analysis, personalized recommendation generation, meal planning, and data exchange. These APIs enable developers to integrate personalized nutrition capabilities into applications, platforms, and services.

## 1. Authentication and Authorization

### 1.1 OAuth 2.0 Implementation

**Authorization Endpoint:**
```
POST /oauth/authorize
```

**Token Endpoint:**
```
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code&
code={authorization_code}&
redirect_uri={redirect_uri}&
client_id={client_id}&
client_secret={client_secret}
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "refresh_token": "def50200...",
  "scope": "read_profile write_meals read_genetics"
}
```

### 1.2 API Key Authentication (Alternative)

```
GET /api/v1/resource
Authorization: ApiKey YOUR_API_KEY
```

### 1.3 Scopes

- `read_profile`: Access user health profile and demographics
- `write_profile`: Update user profile information
- `read_genetics`: Access genetic data
- `read_microbiome`: Access microbiome analysis
- `read_meals`: Access meal logs
- `write_meals`: Create and update meal logs
- `read_biomarkers`: Access laboratory results
- `write_biomarkers`: Submit laboratory results
- `read_recommendations`: Access personalized nutrition recommendations
- `admin`: Full system access (restricted)

## 2. Health Profile APIs

### 2.1 Create/Update Health Profile

```
POST /api/v1/users/{userId}/profile
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "demographics": {
    "age": 35,
    "biologicalSex": "female",
    "ethnicity": "Asian"
  },
  "anthropometrics": {
    "weight_kg": 65.5,
    "height_cm": 165,
    "bodyFat_percent": 25.5
  },
  "healthGoals": {
    "primary": "weight_loss",
    "targetWeight_kg": 60.0,
    "timeframe_weeks": 12
  },
  "dietaryRestrictions": {
    "preferences": "vegetarian",
    "allergies": ["peanuts", "shellfish"]
  }
}
```

**Response (201 Created):**
```json
{
  "userId": "550e8400-e29b-41d4-a716-446655440000",
  "profileId": "profile-123456",
  "status": "active",
  "createdAt": "2025-12-27T10:00:00Z",
  "message": "Health profile created successfully"
}
```

### 2.2 Get Health Profile

```
GET /api/v1/users/{userId}/profile
Authorization: Bearer {access_token}
```

**Response (200 OK):**
Returns complete health profile in Phase 1 format.

## 3. Nutrition Analysis APIs

### 3.1 Analyze Meal

```
POST /api/v1/nutrition/analyze
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "meal": {
    "name": "Grilled Chicken Salad",
    "ingredients": [
      {"name": "chicken breast", "weight_g": 150},
      {"name": "mixed greens", "weight_g": 100},
      {"name": "olive oil", "weight_g": 10}
    ]
  },
  "userId": "550e8400-e29b-41d4-a716-446655440000"
}
```

**Response (200 OK):**
```json
{
  "analysisId": "analysis-789012",
  "nutrition": {
    "totalCalories": 350,
    "macronutrients": {
      "protein_g": 42,
      "carbohydrates_g": 8,
      "fat_g": 18
    },
    "micronutrients": {
      "vitaminA_mcg": 250,
      "iron_mg": 3.2,
      "calcium_mg": 80
    }
  },
  "personalizedInsights": {
    "glycemicImpact": "low",
    "alignmentWithGoals": 95,
    "recommendations": [
      "Excellent protein source for muscle maintenance",
      "Add quinoa for complex carbohydrates"
    ]
  },
  "predictedGlucoseResponse": {
    "peak_mgdL": 110,
    "excursion_mgdL": 25,
    "confidence": 0.87
  }
}
```

### 3.2 Food Image Recognition

```
POST /api/v1/nutrition/analyze/image
Authorization: Bearer {access_token}
Content-Type: multipart/form-data

image: [binary image data]
userId: 550e8400-e29b-41d4-a716-446655440000
```

**Response (200 OK):**
```json
{
  "recognizedFoods": [
    {
      "name": "grilled salmon",
      "confidence": 0.94,
      "portion_g": 180
    },
    {
      "name": "brown rice",
      "confidence": 0.89,
      "portion_g": 150
    },
    {
      "name": "steamed broccoli",
      "confidence": 0.92,
      "portion_g": 100
    }
  ],
  "nutrition": {
    "totalCalories": 520,
    "macronutrients": {...}
  }
}
```

## 4. Personalized Recommendation APIs

### 4.1 Generate Meal Recommendations

```
POST /api/v1/recommendations/meals
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "userId": "550e8400-e29b-41d4-a716-446655440000",
  "mealType": "dinner",
  "preferences": {
    "cuisine": ["Mediterranean", "Asian"],
    "maxPrepTime_minutes": 30,
    "budget": "moderate"
  },
  "nutritionalTargets": {
    "calories": 600,
    "protein_g": 40,
    "carbs_g": 50
  }
}
```

**Response (200 OK):**
```json
{
  "recommendations": [
    {
      "mealId": "meal-rec-001",
      "name": "Mediterranean Baked Cod with Quinoa",
      "matchScore": 0.94,
      "nutrition": {
        "totalCalories": 585,
        "macronutrients": {
          "protein_g": 42,
          "carbohydrates_g": 48,
          "fat_g": 20
        }
      },
      "recipe": {
        "ingredients": [...],
        "instructions": [...],
        "prepTime_minutes": 25,
        "difficulty": "easy"
      },
      "personalizedRationale": [
        "Omega-3 rich for APOE e4 genotype",
        "High protein aligns with FTO AA weight loss goal",
        "Low glycemic index for stable blood sugar"
      ],
      "estimatedGlucoseResponse": {
        "peak_mgdL": 115,
        "prediction_confidence": 0.88
      }
    },
    {
      "mealId": "meal-rec-002",
      "name": "Asian Tofu Stir-Fry",
      "matchScore": 0.89,
      ...
    }
  ],
  "algorithmVersion": "2.1.0",
  "timestamp": "2025-12-27T15:30:00Z"
}
```

### 4.2 Generate Weekly Meal Plan

```
POST /api/v1/recommendations/meal-plan
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "userId": "550e8400-e29b-41d4-a716-446655440000",
  "startDate": "2025-12-30",
  "duration_days": 7,
  "preferences": {
    "variety": "high",
    "repeatFavorites": true
  }
}
```

**Response (200 OK):**
```json
{
  "mealPlanId": "plan-456789",
  "startDate": "2025-12-30",
  "endDate": "2026-01-05",
  "dailyPlans": [
    {
      "date": "2025-12-30",
      "breakfast": {...},
      "lunch": {...},
      "dinner": {...},
      "snacks": [{...}],
      "dailyTotals": {
        "calories": 1850,
        "macronutrients": {...}
      }
    }
  ],
  "shoppingList": {
    "produce": [...],
    "proteins": [...],
    "grains": [...],
    "dairy": [...],
    "other": [...]
  },
  "weeklyNutritionSummary": {
    "averageCalories": 1875,
    "macroRatios": {
      "protein_percent": 30,
      "carbs_percent": 40,
      "fat_percent": 30
    },
    "micronutrientAdequacy": {
      "vitaminD": 95,
      "iron": 87,
      "calcium": 102
    }
  }
}
```

## 5. Genetic Data APIs

### 5.1 Submit Genetic Data

```
POST /api/v1/genetics/{userId}/snps
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "testProvider": "23andMe",
  "testDate": "2025-06-15",
  "testType": "genotyping_array",
  "snps": [
    {
      "rsid": "rs1801133",
      "gene": "MTHFR",
      "genotype": "CT"
    },
    ...
  ]
}
```

**Response (201 Created):**
```json
{
  "geneticProfileId": "genetic-123456",
  "processedSnps": 127,
  "nutritionRelevantVariants": 15,
  "status": "processed",
  "summary": {
    "folateMeta bolism": "intermediate_efficiency",
    "obesityRisk": "moderate",
    "caffeineMetabolism": "slow"
  }
}
```

### 5.2 Get Genetic-Based Recommendations

```
GET /api/v1/genetics/{userId}/recommendations
Authorization: Bearer {access_token}
```

**Response (200 OK):**
```json
{
  "geneticRecommendations": [
    {
      "gene": "MTHFR",
      "variant": "C677T (CT)",
      "impact": "Reduced folate metabolism efficiency",
      "recommendations": [
        "Increase folate intake to 600mcg/day",
        "Choose methylfolate over folic acid",
        "Emphasize leafy greens and legumes"
      ],
      "supplementSuggestions": [
        {
          "name": "Methylfolate",
          "dosage": "400-600mcg daily",
          "timing": "with meals"
        }
      ]
    }
  ]
}
```

## 6. Microbiome APIs

### 6.1 Submit Microbiome Data

```
POST /api/v1/microbiome/{userId}/analysis
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "testProvider": "Viome",
  "testDate": "2025-11-20",
  "sequencingMethod": "metatranscriptomic",
  "diversityMetrics": {
    "shannonIndex": 3.8,
    "speciesRichness": 167
  },
  "keySpecies": [...]
}
```

**Response (201 Created):**
```json
{
  "microbiomeAnalysisId": "microbiome-789012",
  "gutHealthScore": 8.2,
  "recommendations": {
    "prebiotics": [
      "Increase resistant starch (cooked/cooled potatoes)",
      "Add inulin-rich foods (garlic, onions, leeks)"
    ],
    "probiotics": [
      "Consider Bifidobacterium longum supplementation",
      "Include fermented foods daily (kefir, sauerkraut)"
    ],
    "dietaryPatterns": [
      "Diversify plant foods (target 30+ different types weekly)",
      "Emphasize fiber-rich meals (25-40g daily)"
    ]
  }
}
```

## 7. Biomarker APIs

### 7.1 Submit Lab Results

```
POST /api/v1/biomarkers/{userId}/results
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "testDate": "2025-12-15",
  "fasting": true,
  "biomarkers": {
    "lipidPanel": {
      "totalCholesterol_mgdL": 195,
      "ldlCholesterol_mgdL": 115,
      "hdlCholesterol_mgdL": 58,
      "triglycerides_mgdL": 110
    },
    "glucoseMetabolism": {
      "fastingGlucose_mgdL": 92,
      "hba1c_percent": 5.4
    }
  }
}
```

**Response (201 Created):**
```json
{
  "biomarkerResultId": "bio-345678",
  "interpretations": [
    {
      "biomarker": "LDL Cholesterol",
      "value": 115,
      "status": "borderline",
      "nutritionalActions": [
        "Increase soluble fiber to 10-15g daily",
        "Limit saturated fat to <7% of calories",
        "Add plant sterols (2g daily)"
      ]
    }
  ],
  "overallRisk": {
    "cardiovascular": "low-moderate",
    "metabolic": "low"
  }
}
```

## 8. Real-Time Feedback APIs

### 8.1 Submit CGM Data

```
POST /api/v1/wearables/cgm/{userId}/readings
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "deviceType": "dexcom_g7",
  "measurements": [
    {
      "timestamp": "2025-12-27T12:00:00Z",
      "glucose_mgdL": 105,
      "trend": "stable"
    },
    {
      "timestamp": "2025-12-27T12:05:00Z",
      "glucose_mgdL": 108,
      "trend": "rising"
    }
  ]
}
```

**Response (200 OK):**
```json
{
  "processed": 2,
  "currentGlucose_mgdL": 108,
  "trend": "rising",
  "insights": {
    "status": "within_normal_range",
    "action": null
  }
}
```

### 8.2 Correlate Meal with Glucose Response

```
POST /api/v1/analysis/meal-glucose-correlation
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "userId": "550e8400-e29b-41d4-a716-446655440000",
  "mealId": "meal-123456",
  "glucoseReadings": [...]
}
```

**Response (200 OK):**
```json
{
  "correlationId": "corr-789012",
  "mealImpact": {
    "preMealGlucose_mgdL": 95,
    "peakGlucose_mgdL": 135,
    "glucoseExcursion_mgdL": 40,
    "timeToPeak_minutes": 45,
    "interpretation": "moderate_spike",
    "futureRecommendations": [
      "Consider reducing portion size by 20%",
      "Add more fiber or protein to slow absorption",
      "Try meal timing 30 minutes later relative to activity"
    ]
  }
}
```

## 9. Error Handling

### Standard Error Response Format

```json
{
  "error": {
    "code": "INVALID_REQUEST",
    "message": "Missing required field: userId",
    "details": {
      "field": "userId",
      "location": "body"
    },
    "timestamp": "2025-12-27T15:45:00Z",
    "requestId": "req-abc123"
  }
}
```

### Error Codes

- `400 Bad Request`: Invalid request format or parameters
- `401 Unauthorized`: Missing or invalid authentication
- `403 Forbidden`: Insufficient permissions for resource
- `404 Not Found`: Resource does not exist
- `409 Conflict`: Resource conflict (e.g., duplicate entry)
- `422 Unprocessable Entity`: Validation errors
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Server error
- `503 Service Unavailable`: Temporary service outage

## 10. Rate Limiting

- **Free Tier**: 100 requests/hour
- **Standard Tier**: 1,000 requests/hour
- **Premium Tier**: 10,000 requests/hour
- **Enterprise**: Custom limits

**Rate Limit Headers:**
```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 847
X-RateLimit-Reset: 1640623200
```

## 11. Versioning

API versions are specified in the URL path:
- Current: `/api/v1/`
- Deprecated versions supported for 12 months after new release
- Breaking changes require new major version

## Conclusion

WIA-IND-010 Phase 2 API interfaces enable seamless integration of personalized nutrition capabilities into diverse applications and platforms. Through standardized RESTful endpoints, OAuth 2.0 authentication, comprehensive error handling, and rate limiting, developers can build robust nutrition solutions embodying the principle of 弘益人間 (Benefit All Humanity).

---

© 2025 SmileStory Inc. / WIA
弘益人間 · Benefit All Humanity
WIA-IND-010 v1.0
