# WIA-IND-008: Smart Kitchen Standard
# Phase 2: API Interface Specification

**Version:** 1.0
**Category:** Industrial (IND)
**Philosophy:** 弘益人間 (Benefit All Humanity)
**Last Updated:** 2025-01-15

## Overview

Phase 2 defines RESTful API interfaces for smart kitchen systems, enabling device control, data synchronization, and service integration. All APIs follow standard HTTP conventions and return JSON responses using Phase 1 data formats.

## Core Principles

1. **RESTful Design:** Standard HTTP methods (GET, POST, PUT, DELETE, PATCH)
2. **Stateless:** Each request contains all necessary information
3. **Versioned:** APIs include version in URL path (`/v1/`)
4. **Secure:** HTTPS required, OAuth 2.0 or API key authentication
5. **Rate Limited:** Protection against abuse with clear limit headers
6. **Paginated:** Large datasets use cursor-based pagination

## Base URL Structure

```
https://api.{domain}/wia-ind-008/v1/{resource}
```

Example: `https://api.smart-kitchen.example.com/wia-ind-008/v1/devices`

## 1. Device Management APIs

### 1.1 List All Devices

**Endpoint:** `GET /devices`

**Query Parameters:**
- `type` (optional): Filter by device type
- `status` (optional): Filter by status (on|off|error)
- `location` (optional): Filter by location
- `limit` (optional): Number of results (default: 50, max: 100)
- `cursor` (optional): Pagination cursor

**Response:**
```json
{
  "devices": [
    {
      /* SmartKitchenAppliance schema */
    }
  ],
  "pagination": {
    "cursor": "string",
    "hasMore": "boolean",
    "total": "number"
  }
}
```

### 1.2 Get Device Details

**Endpoint:** `GET /devices/{deviceId}`

**Response:** SmartKitchenAppliance object (see Phase 1)

### 1.3 Update Device

**Endpoint:** `PATCH /devices/{deviceId}`

**Request Body:**
```json
{
  "name": "string (optional)",
  "location": "object (optional)",
  "settings": "object (device-specific, optional)"
}
```

**Response:** Updated SmartKitchenAppliance object

### 1.4 Control Device

**Endpoint:** `POST /devices/{deviceId}/control`

**Request Body:**
```json
{
  "command": "string (required)",
  "parameters": "object (optional)"
}
```

**Common Commands:**
- `power`: `{state: "on"|"off"}`
- `setTemperature`: `{temperature: number, unit: "celsius"|"fahrenheit"}`
- `startCycle`: `{mode: string, duration: number}`
- `cancelOperation`: `{}`

**Response:**
```json
{
  "success": "boolean",
  "commandId": "string",
  "status": "object (current device status)"
}
```

### 1.5 Get Device Telemetry

**Endpoint:** `GET /devices/{deviceId}/telemetry`

**Query Parameters:**
- `start`: ISO 8601 datetime
- `end`: ISO 8601 datetime
- `interval`: Time bucket size (1m, 5m, 1h, 1d)

**Response:**
```json
{
  "deviceId": "string",
  "period": {
    "start": "datetime",
    "end": "datetime"
  },
  "dataPoints": [
    {
      "timestamp": "datetime",
      "telemetry": "object"
    }
  ]
}
```

## 2. Recipe APIs

### 2.1 Search Recipes

**Endpoint:** `GET /recipes`

**Query Parameters:**
- `q`: Search query
- `ingredients`: Comma-separated ingredient list
- `cuisine`: Cuisine filter
- `difficulty`: Difficulty level
- `maxTime`: Maximum total time in minutes
- `dietary`: Dietary filters (comma-separated)
- `sort`: Sort by (popularity|rating|time|recent)
- `limit`: Results per page
- `cursor`: Pagination cursor

**Response:**
```json
{
  "recipes": [
    {
      /* Recipe schema */
    }
  ],
  "facets": {
    "cuisines": ["array of available cuisines with counts"],
    "difficulty": "object with counts",
    "averageTime": "number"
  },
  "pagination": "pagination object"
}
```

### 2.2 Get Recipe

**Endpoint:** `GET /recipes/{recipeId}`

**Response:** Recipe object (see Phase 1)

### 2.3 Create Recipe

**Endpoint:** `POST /recipes`

**Authentication:** Required

**Request Body:** Recipe object

**Response:**
```json
{
  "id": "string",
  "recipe": "Recipe object",
  "status": "enum: pending|approved|published"
}
```

### 2.4 Start Cooking

**Endpoint:** `POST /recipes/{recipeId}/cook`

**Request Body:**
```json
{
  "servings": "number (optional)",
  "devices": ["array of device IDs to use"],
  "scheduledStart": "datetime (optional)"
}
```

**Response:**
```json
{
  "cookingSessionId": "string",
  "steps": "array of steps with device commands",
  "estimatedCompletion": "datetime"
}
```

### 2.5 Cooking Session Status

**Endpoint:** `GET /cooking-sessions/{sessionId}`

**Response:**
```json
{
  "id": "string",
  "recipeId": "string",
  "status": "enum: preparing|cooking|completed|paused|cancelled",
  "currentStep": "number",
  "completedSteps": "array of step numbers",
  "activeDevices": "array of device IDs",
  "estimatedCompletion": "datetime"
}
```

## 3. Inventory APIs

### 3.1 Get Inventory

**Endpoint:** `GET /inventory`

**Query Parameters:**
- `location`: Filter by storage location
- `category`: Filter by category
- `expiring`: Get items expiring within N days
- `low`: Get items below threshold quantity

**Response:**
```json
{
  "items": [
    {
      /* FoodInventoryItem schema */
    }
  ],
  "summary": {
    "totalItems": "number",
    "expiringItems": "number",
    "lowStockItems": "number",
    "totalValue": "number"
  }
}
```

### 3.2 Add Inventory Item

**Endpoint:** `POST /inventory`

**Request Body:** FoodInventoryItem object

**Response:** Created FoodInventoryItem with generated ID

### 3.3 Update Inventory Item

**Endpoint:** `PATCH /inventory/{itemId}`

**Request Body:** Partial FoodInventoryItem object

**Response:** Updated FoodInventoryItem

### 3.4 Remove/Consume Item

**Endpoint:** `POST /inventory/{itemId}/consume`

**Request Body:**
```json
{
  "amount": "number",
  "unit": "string",
  "reason": "enum: consumed|discarded|expired"
}
```

**Response:** Updated FoodInventoryItem or deleted if fully consumed

### 3.5 Scan Barcode

**Endpoint:** `POST /inventory/scan`

**Request Body:**
```json
{
  "barcode": "string (UPC/EAN)",
  "quantity": "number (optional)"
}
```

**Response:**
```json
{
  "recognized": "boolean",
  "product": {
    "name": "string",
    "brand": "string",
    "nutritional": "object"
  },
  "suggestedItem": "FoodInventoryItem object with pre-filled data"
}
```

## 4. Meal Planning APIs

### 4.1 Get Meal Plan

**Endpoint:** `GET /meal-plans/current`

**Query Parameters:**
- `start`: Start date
- `end`: End date

**Response:** MealPlan object (see Phase 1)

### 4.2 Create Meal Plan

**Endpoint:** `POST /meal-plans`

**Request Body:**
```json
{
  "startDate": "date",
  "endDate": "date",
  "preferences": {
    "calorieTarget": "number",
    "mealsPerDay": "number",
    "avoidIngredients": "array",
    "prioritizeInventory": "boolean"
  }
}
```

**Response:** Generated MealPlan object

### 4.3 Update Meal

**Endpoint:** `PATCH /meal-plans/{planId}/meals/{mealId}`

**Request Body:**
```json
{
  "recipeId": "string (optional)",
  "servings": "number (optional)",
  "scheduledTime": "time (optional)"
}
```

**Response:** Updated Meal object

### 4.4 Generate Shopping List

**Endpoint:** `POST /meal-plans/{planId}/shopping-list`

**Response:** ShoppingList object with items needed for meal plan

## 5. Shopping APIs

### 5.1 Get Shopping List

**Endpoint:** `GET /shopping-lists/current`

**Response:** ShoppingList object

### 5.2 Add Shopping Item

**Endpoint:** `POST /shopping-lists/current/items`

**Request Body:**
```json
{
  "name": "string",
  "amount": "number",
  "unit": "string",
  "category": "string",
  "priority": "enum"
}
```

**Response:** Updated ShoppingList

### 5.3 Mark Item Acquired

**Endpoint:** `POST /shopping-lists/current/items/{itemId}/acquire`

**Response:** Updated ShoppingList

### 5.4 Compare Prices

**Endpoint:** `GET /shopping/prices`

**Query Parameters:**
- `items`: Comma-separated item names
- `stores`: Comma-separated store IDs
- `location`: User location for local pricing

**Response:**
```json
{
  "items": [
    {
      "name": "string",
      "prices": [
        {
          "store": "string",
          "price": "number",
          "unit": "string",
          "inStock": "boolean"
        }
      ],
      "bestPrice": "object (store and price)"
    }
  ]
}
```

## 6. Energy APIs

### 6.1 Get Energy Usage

**Endpoint:** `GET /energy/usage`

**Query Parameters:**
- `deviceId`: Specific device (optional, otherwise all devices)
- `start`: Start datetime
- `end`: End datetime
- `groupBy`: hour|day|week|month

**Response:** EnergyUsageReport object (see Phase 1)

### 6.2 Get Energy Recommendations

**Endpoint:** `GET /energy/recommendations`

**Response:**
```json
{
  "recommendations": [
    {
      "type": "enum: schedule-shift|mode-change|maintenance|replacement",
      "deviceId": "string",
      "description": "string",
      "estimatedSavings": {
        "kWh": "number",
        "cost": "number",
        "percentage": "number"
      },
      "difficulty": "enum: easy|medium|hard",
      "automated": "boolean"
    }
  ],
  "totalPotentialSavings": "number"
}
```

## 7. User Preferences APIs

### 7.1 Get User Profile

**Endpoint:** `GET /users/me/profile`

**Response:** UserProfile object (see Phase 1)

### 7.2 Update User Profile

**Endpoint:** `PATCH /users/me/profile`

**Request Body:** Partial UserProfile object

**Response:** Updated UserProfile

## 8. Webhook APIs

### 8.1 Register Webhook

**Endpoint:** `POST /webhooks`

**Request Body:**
```json
{
  "url": "string (HTTPS URL)",
  "events": ["array of event types"],
  "secret": "string (for signature verification)"
}
```

**Event Types:**
- `device.status.changed`
- `inventory.item.expiring`
- `inventory.item.low`
- `cooking.session.completed`
- `energy.threshold.exceeded`
- `recipe.recommended`

**Response:**
```json
{
  "id": "string",
  "url": "string",
  "events": "array",
  "created": "datetime",
  "status": "enum: active|paused|failed"
}
```

### 8.2 Webhook Payload Format

```json
{
  "eventId": "string (unique)",
  "eventType": "string",
  "timestamp": "datetime",
  "data": "object (event-specific)",
  "signature": "HMAC-SHA256 of payload"
}
```

## Authentication & Authorization

### OAuth 2.0 Flow

1. **Authorization Request:** `GET /oauth/authorize`
2. **Token Exchange:** `POST /oauth/token`
3. **Token Refresh:** `POST /oauth/token` with `grant_type=refresh_token`

### API Key Authentication

Alternative for server-to-server:
- Header: `Authorization: Bearer {api_key}`
- Query: `?api_key={key}` (not recommended)

## Error Responses

Standard error format:
```json
{
  "error": {
    "code": "string (ERROR_CODE)",
    "message": "string (human-readable)",
    "details": "object (optional)",
    "requestId": "string"
  }
}
```

**Common Error Codes:**
- `INVALID_REQUEST`: Malformed request
- `UNAUTHORIZED`: Authentication required
- `FORBIDDEN`: Insufficient permissions
- `NOT_FOUND`: Resource not found
- `RATE_LIMITED`: Too many requests
- `SERVER_ERROR`: Internal server error

## Rate Limiting

**Headers:**
- `X-RateLimit-Limit`: Requests per time window
- `X-RateLimit-Remaining`: Remaining requests
- `X-RateLimit-Reset`: Timestamp when limit resets

**Default Limits:**
- Authenticated: 1000 requests/hour
- Unauthenticated: 100 requests/hour
- Burst: 20 requests/second

## 弘益人間 Considerations

- APIs are designed for accessibility (simple, well-documented)
- Rate limits balanced to serve hobbyists and enterprises
- Open specification enables community implementations
- Webhooks reduce polling, saving energy
- Privacy-focused: minimal data collection, user control

---

**Previous Phase:** [Phase 1: Data Format](PHASE-1-DATA-FORMAT.md)
**Next Phase:** [Phase 3: Protocol](PHASE-3-PROTOCOL.md)

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity
