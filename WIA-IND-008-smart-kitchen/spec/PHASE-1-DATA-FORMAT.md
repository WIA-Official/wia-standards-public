# WIA-IND-008: Smart Kitchen Standard
# Phase 1: Data Format Specification

**Version:** 1.0
**Category:** Industrial (IND)
**Philosophy:** 弘益人間 (Benefit All Humanity)
**Last Updated:** 2025-01-15

## Overview

Phase 1 defines standardized data formats for all smart kitchen entities including appliances, recipes, inventory items, and nutritional information. These schemas ensure interoperability across different manufacturers and platforms.

## Core Principles

1. **JSON-based:** All data formats use JSON for human readability and machine processing
2. **Extensible:** Schemas support additional properties for vendor-specific features
3. **Versioned:** Each schema includes version information for compatibility
4. **Localized:** Support for internationalization (i18n) built into core schemas
5. **Semantic:** Uses JSON-LD for linked data and semantic web compatibility

## 1. Appliance Data Schema

### 1.1 Base Appliance Schema

```json
{
  "@context": "https://wia-official.org/schemas/kitchen/v1",
  "@type": "SmartKitchenAppliance",
  "id": "string (required)",
  "name": "string (required)",
  "manufacturer": "string (required)",
  "model": "string (required)",
  "deviceType": "enum (required)",
  "firmwareVersion": "string",
  "protocol": ["array of strings"],
  "status": {
    "power": "enum: on|off|standby",
    "operational": "enum: idle|active|error",
    "lastUpdate": "ISO 8601 datetime"
  },
  "capabilities": ["array of capability strings"],
  "location": {
    "room": "string",
    "zone": "string",
    "coordinates": {"x": "number", "y": "number"}
  },
  "telemetry": "object (device-specific)",
  "metadata": "object (optional)"
}
```

### 1.2 Device Type Enumeration

- `refrigerator`
- `oven`
- `microwave`
- `dishwasher`
- `stove`
- `range`
- `coffeeMaker`
- `blender`
- `foodProcessor`
- `toaster`
- `airFryer`
- `slowCooker`
- `pressureCooker`
- `mixer`

### 1.3 Refrigerator-Specific Schema

```json
{
  "@type": "Refrigerator",
  "telemetry": {
    "temperature": {
      "freezer": "number (celsius)",
      "refrigerator": "number (celsius)",
      "flexZone": "number (celsius, optional)"
    },
    "humidity": {
      "crisper": "number (percentage)"
    },
    "doors": {
      "main": "enum: open|closed",
      "freezer": "enum: open|closed",
      "dispenser": "enum: open|closed"
    },
    "cameras": [
      {
        "location": "string",
        "lastCapture": "ISO 8601 datetime",
        "imageUrl": "string (URL)"
      }
    ],
    "icemaker": {
      "status": "enum: idle|producing|full",
      "level": "number (percentage)"
    },
    "waterFilter": {
      "lifeRemaining": "number (percentage)",
      "lastChanged": "ISO 8601 datetime"
    }
  },
  "inventory": ["array of inventory item IDs"]
}
```

### 1.4 Oven-Specific Schema

```json
{
  "@type": "Oven",
  "telemetry": {
    "temperature": {
      "current": "number (celsius)",
      "target": "number (celsius)"
    },
    "mode": "enum: off|bake|roast|broil|convection|proof|clean",
    "remainingTime": "number (seconds)",
    "preheating": "boolean",
    "door": "enum: open|closed|locked",
    "racks": [
      {
        "position": "number (1-5)",
        "occupied": "boolean"
      }
    ],
    "selfClean": {
      "active": "boolean",
      "progress": "number (percentage)"
    }
  },
  "activeRecipe": "string (recipe ID, optional)"
}
```

## 2. Recipe Data Schema

### 2.1 Standard Recipe Format

```json
{
  "@context": "https://wia-official.org/schemas/recipe/v1",
  "@type": "Recipe",
  "id": "string (required)",
  "name": "string (required)",
  "description": "string",
  "author": {
    "name": "string",
    "url": "string (optional)",
    "verified": "boolean"
  },
  "cuisine": "string",
  "category": "enum: breakfast|lunch|dinner|snack|dessert|appetizer",
  "difficulty": "enum: easy|medium|hard",
  "prepTime": "number (minutes)",
  "cookTime": "number (minutes)",
  "totalTime": "number (minutes)",
  "servings": "number",
  "yield": "string (descriptive)",
  "ingredients": [
    {
      "name": "string (required)",
      "amount": "number",
      "unit": "string",
      "preparation": "string (optional)",
      "optional": "boolean",
      "substitutes": [
        {
          "name": "string",
          "amount": "number",
          "unit": "string",
          "notes": "string"
        }
      ]
    }
  ],
  "steps": [
    {
      "stepNumber": "number (required)",
      "instruction": "string (required)",
      "duration": "number (minutes, optional)",
      "temperature": "object (optional)",
      "appliance": "string (deviceType)",
      "timerName": "string (optional)",
      "image": "string (URL, optional)",
      "video": "string (URL, optional)"
    }
  ],
  "nutrition": {
    "servingSize": "string",
    "calories": "number",
    "protein": "number (grams)",
    "carbohydrates": "number (grams)",
    "fat": "number (grams)",
    "saturatedFat": "number (grams)",
    "fiber": "number (grams)",
    "sugar": "number (grams)",
    "sodium": "number (milligrams)",
    "cholesterol": "number (milligrams)",
    "vitamins": "object (optional)",
    "minerals": "object (optional)"
  },
  "tags": ["array of strings"],
  "appliances": ["array of deviceType strings"],
  "allergens": ["array of allergen strings"],
  "dietaryInfo": {
    "vegetarian": "boolean",
    "vegan": "boolean",
    "glutenFree": "boolean",
    "dairyFree": "boolean",
    "nutFree": "boolean",
    "lowCarb": "boolean",
    "keto": "boolean",
    "paleo": "boolean"
  },
  "ratings": {
    "average": "number (0-5)",
    "count": "number"
  },
  "images": ["array of image URLs"],
  "sourceUrl": "string (URL, optional)",
  "published": "ISO 8601 datetime",
  "modified": "ISO 8601 datetime"
}
```

## 3. Inventory Data Schema

### 3.1 Food Inventory Item

```json
{
  "@context": "https://wia-official.org/schemas/inventory/v1",
  "@type": "FoodInventoryItem",
  "id": "string (required)",
  "product": {
    "name": "string (required)",
    "brand": "string",
    "barcode": "string (UPC/EAN)",
    "category": "string",
    "subcategory": "string"
  },
  "quantity": {
    "amount": "number (required)",
    "unit": "string (required)",
    "remaining": "number (percentage, 0-100)"
  },
  "dates": {
    "purchased": "ISO 8601 date",
    "opened": "ISO 8601 date (optional)",
    "expires": "ISO 8601 date",
    "bestBefore": "ISO 8601 date (optional)"
  },
  "storage": {
    "location": "enum: refrigerator|freezer|pantry|cabinet",
    "zone": "string",
    "temperature": "number (celsius, optional)",
    "deviceId": "string (appliance ID)"
  },
  "nutritional": {
    "servingSize": "string",
    "servingsPerContainer": "number",
    "calories": "number",
    "macros": {
      "protein": "number (grams)",
      "carbs": "number (grams)",
      "fat": "number (grams)"
    }
  },
  "price": {
    "amount": "number",
    "currency": "string (ISO 4217)"
  },
  "tracking": {
    "addedBy": "string (user ID)",
    "addedAt": "ISO 8601 datetime",
    "modifiedAt": "ISO 8601 datetime"
  },
  "alerts": [
    {
      "type": "enum: expiring|expired|low|out",
      "triggered": "boolean",
      "date": "ISO 8601 datetime"
    }
  ]
}
```

## 4. Meal Plan Schema

```json
{
  "@context": "https://wia-official.org/schemas/mealplan/v1",
  "@type": "MealPlan",
  "id": "string (required)",
  "userId": "string (required)",
  "startDate": "ISO 8601 date",
  "endDate": "ISO 8601 date",
  "meals": [
    {
      "date": "ISO 8601 date",
      "mealType": "enum: breakfast|lunch|dinner|snack",
      "recipeId": "string",
      "servings": "number",
      "scheduledTime": "HH:MM",
      "prepTime": "number (minutes)",
      "notes": "string"
    }
  ],
  "shoppingList": {
    "generated": "ISO 8601 datetime",
    "items": [
      {
        "ingredientName": "string",
        "amount": "number",
        "unit": "string",
        "acquired": "boolean",
        "recipes": ["array of recipe IDs using this ingredient"]
      }
    ]
  },
  "nutritionalSummary": {
    "dailyAverages": {
      "calories": "number",
      "protein": "number",
      "carbs": "number",
      "fat": "number"
    },
    "goals": {
      "calorieTarget": "number",
      "macroTargets": "object"
    }
  }
}
```

## 5. Energy Usage Schema

```json
{
  "@context": "https://wia-official.org/schemas/energy/v1",
  "@type": "EnergyUsageReport",
  "deviceId": "string (required)",
  "period": {
    "start": "ISO 8601 datetime",
    "end": "ISO 8601 datetime"
  },
  "consumption": {
    "total": "number (kWh)",
    "peak": "number (kW)",
    "average": "number (kW)"
  },
  "cost": {
    "amount": "number",
    "currency": "string (ISO 4217)",
    "rate": "number (per kWh)"
  },
  "breakdown": [
    {
      "mode": "string",
      "duration": "number (minutes)",
      "consumption": "number (kWh)",
      "percentage": "number (0-100)"
    }
  ],
  "comparison": {
    "previousPeriod": "number (kWh)",
    "percentageChange": "number",
    "averageForDeviceType": "number (kWh)"
  }
}
```

## 6. User Profile Schema

```json
{
  "@context": "https://wia-official.org/schemas/user/v1",
  "@type": "UserProfile",
  "id": "string (required)",
  "preferences": {
    "cuisines": ["array of preferred cuisines"],
    "avoidances": ["array of disliked ingredients"],
    "spiceLevel": "enum: none|mild|medium|hot|extreme",
    "cookingSkill": "enum: beginner|intermediate|advanced|expert"
  },
  "dietary": {
    "restrictions": ["array of dietary restrictions"],
    "allergens": ["array of allergens"],
    "goals": {
      "type": "enum: weight-loss|muscle-gain|maintenance|health",
      "calorieTarget": "number",
      "macroTargets": {
        "protein": "number (grams or percentage)",
        "carbs": "number",
        "fat": "number"
      }
    }
  },
  "household": {
    "size": "number",
    "members": [
      {
        "name": "string",
        "age": "number",
        "dietary": "object (same structure as user dietary)"
      }
    ]
  },
  "language": "string (ISO 639-1)",
  "measurementSystem": "enum: metric|imperial",
  "timezone": "string (IANA timezone)"
}
```

## 7. Shopping List Schema

```json
{
  "@context": "https://wia-official.org/schemas/shopping/v1",
  "@type": "ShoppingList",
  "id": "string (required)",
  "userId": "string (required)",
  "created": "ISO 8601 datetime",
  "modified": "ISO 8601 datetime",
  "items": [
    {
      "id": "string",
      "name": "string (required)",
      "amount": "number",
      "unit": "string",
      "category": "string",
      "priority": "enum: low|medium|high|urgent",
      "acquired": "boolean",
      "estimatedPrice": "number",
      "notes": "string",
      "source": "enum: manual|recipe|inventory|predicted"
    }
  ],
  "stores": [
    {
      "name": "string",
      "items": ["array of item IDs available at this store"],
      "totalEstimatedCost": "number"
    }
  ]
}
```

## Implementation Notes

1. **Validation:** All schemas should be validated using JSON Schema Draft 7 or later
2. **Extensibility:** Implementers may add custom fields prefixed with `x-` or `custom-`
3. **Backwards Compatibility:** New versions must maintain compatibility with previous versions
4. **Localization:** Use JSON-LD for multilingual support
5. **Privacy:** Personal data fields should be encrypted at rest and in transit

## 弘益人間 Considerations

- Schemas support accessibility features (screen readers, voice interfaces)
- Nutritional data enables health-conscious choices for all users
- Open format prevents vendor lock-in
- Supports diverse cuisines and cultural preferences
- Energy tracking promotes environmental responsibility

---

**Next Phase:** [Phase 2: API Interface](PHASE-2-API-INTERFACE.md)

© 2025 SmileStory Inc. / WIA
弘益人間 (홍익인간) · Benefit All Humanity

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-IND-008-smart-kitchen is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/WIA-IND-008-smart-kitchen/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-IND-008-smart-kitchen/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-IND-008-smart-kitchen/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---
