# WIA-MED-022 Nutrition Tracking Standard
## Phase 2: API Interface Specification

### Version: 1.0.0
### Status: Complete
### Last Updated: 2025-01-15

---

## 1. API Overview

Base URL: `https://api.nutrition-tracker.example.com/v1`

All endpoints require HTTPS and JWT authentication unless specified otherwise.

---

## 2. Authentication

### 2.1 Register User
```
POST /auth/register
Content-Type: application/json

Request:
{
  "email": "user@example.com",
  "password": "securePassword123",
  "name": "John Doe"
}

Response: 201 Created
{
  "user_id": "uuid",
  "email": "user@example.com",
  "access_token": "eyJhbGc...",
  "refresh_token": "eyJhbGc...",
  "expires_in": 3600
}
```

### 2.2 Login
```
POST /auth/login
Content-Type: application/json

Request:
{
  "email": "user@example.com",
  "password": "securePassword123"
}

Response: 200 OK
{
  "access_token": "eyJhbGc...",
  "refresh_token": "eyJhbGc...",
  "expires_in": 3600,
  "user": {
    "user_id": "uuid",
    "email": "user@example.com",
    "name": "John Doe"
  }
}
```

---

## 3. Food Database

### 3.1 Search Foods
```
GET /foods/search?q={query}&limit={limit}&offset={offset}
Authorization: Bearer {token}

Parameters:
- q: Search query (required)
- limit: Results per page (default: 20, max: 100)
- offset: Pagination offset (default: 0)

Response: 200 OK
{
  "total": 150,
  "limit": 20,
  "offset": 0,
  "results": [
    {
      "food_id": "uuid",
      "name": "Chicken Breast, Grilled",
      "brand": null,
      "serving_size": 100,
      "calories": 165,
      "protein": 31,
      "carbs": 0,
      "fat": 3.6,
      "verified": true
    }
  ]
}
```

### 3.2 Get Food Details
```
GET /foods/{food_id}
Authorization: Bearer {token}

Response: 200 OK
{
  "food_id": "uuid",
  "name": "Chicken Breast, Grilled",
  "serving_size": 100,
  "nutrition": { ... },
  "allergens": ["poultry"],
  "verified": true,
  "data_source": "USDA"
}
```

---

## 4. Meal Logging

### 4.1 Create Meal
```
POST /meals
Authorization: Bearer {token}
Content-Type: application/json

Request:
{
  "datetime": "2025-01-15T12:30:00Z",
  "meal_type": "lunch",
  "items": [
    {
      "food_id": "uuid",
      "quantity": 200,
      "unit": "g"
    }
  ],
  "notes": "At work cafeteria"
}

Response: 201 Created
{
  "meal_id": "uuid",
  "user_id": "uuid",
  "datetime": "2025-01-15T12:30:00Z",
  "meal_type": "lunch",
  "nutrition_summary": {
    "calories": 330,
    "protein": 62,
    "carbs": 0,
    "fat": 7.2
  }
}
```

### 4.2 Get Daily Meals
```
GET /meals?date={YYYY-MM-DD}
Authorization: Bearer {token}

Response: 200 OK
{
  "date": "2025-01-15",
  "meals": [
    {
      "meal_id": "uuid",
      "meal_type": "breakfast",
      "datetime": "2025-01-15T07:30:00Z",
      "nutrition_summary": { ... }
    }
  ],
  "daily_totals": {
    "calories": 1850,
    "protein": 142,
    "carbs": 185,
    "fat": 62
  }
}
```

---

## 5. AI Analysis

### 5.1 Analyze Food Photo
```
POST /ai/analyze-image
Authorization: Bearer {token}
Content-Type: multipart/form-data

Request:
- image: (binary file)
- meal_type: "lunch"

Response: 200 OK
{
  "analysis_id": "uuid",
  "detected_foods": [
    {
      "food_id": "uuid",
      "name": "Grilled Chicken",
      "confidence": 0.92,
      "quantity": 180,
      "unit": "g",
      "bounding_box": {
        "x": 120,
        "y": 80,
        "width": 200,
        "height": 150
      }
    }
  ],
  "total_nutrition": {
    "calories": 480,
    "protein": 58,
    "carbs": 52,
    "fat": 7
  }
}
```

### 5.2 Scan Barcode
```
POST /ai/barcode-scan
Authorization: Bearer {token}
Content-Type: application/json

Request:
{
  "barcode": "8801234567890"
}

Response: 200 OK
{
  "food_id": "uuid",
  "name": "Chocolate Chip Cookies",
  "brand": "Brand Name",
  "barcode": "8801234567890",
  "nutrition": { ... },
  "allergens": ["gluten", "milk"]
}
```

---

## 6. Nutrition Summary

### 6.1 Get Daily Summary
```
GET /nutrition/summary?date={YYYY-MM-DD}
Authorization: Bearer {token}

Response: 200 OK
{
  "date": "2025-01-15",
  "goals": {
    "calories": 2100,
    "protein": 150,
    "carbs": 200,
    "fat": 70
  },
  "consumed": {
    "calories": 1850,
    "protein": 142,
    "carbs": 185,
    "fat": 62
  },
  "remaining": {
    "calories": 250,
    "protein": 8,
    "carbs": 15,
    "fat": 8
  },
  "percentage_complete": {
    "calories": 88,
    "protein": 95,
    "carbs": 93,
    "fat": 89
  }
}
```

---

## 7. Error Handling

### 7.1 Error Response Format
```json
{
  "error": {
    "code": "INVALID_INPUT",
    "message": "Quantity must be a positive number",
    "details": {
      "field": "quantity",
      "value": -10
    }
  }
}
```

### 7.2 HTTP Status Codes
- 200 OK: Successful request
- 201 Created: Resource created
- 400 Bad Request: Invalid input
- 401 Unauthorized: Authentication required
- 403 Forbidden: Insufficient permissions
- 404 Not Found: Resource not found
- 429 Too Many Requests: Rate limit exceeded
- 500 Internal Server Error: Server error

---

## 8. Rate Limiting

### 8.1 Limits
- Free Tier: 1,000 requests/hour
- Premium: 10,000 requests/hour
- AI Endpoints: 100 requests/hour

### 8.2 Headers
```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 850
X-RateLimit-Reset: 1642248000
```

---

© 2025 WIA Standards - Benefit All Humanity (弘益人間)
