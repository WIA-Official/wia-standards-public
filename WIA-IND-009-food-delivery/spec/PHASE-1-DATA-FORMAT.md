# WIA-IND-009: PHASE 1 - DATA FORMAT SPECIFICATION
## Food Delivery Platform Standard
### Version 1.0 | 弘益人間 (Benefit All Humanity)

---

## Table of Contents
1. [Overview](#overview)
2. [Order Data Schema](#order-data-schema)
3. [Menu Data Structure](#menu-data-structure)
4. [Driver Data Model](#driver-data-model)
5. [Location Format](#location-format)
6. [Restaurant Profile](#restaurant-profile)
7. [Customer Profile](#customer-profile)
8. [Timestamp Standards](#timestamp-standards)
9. [Validation Rules](#validation-rules)

---

## Overview

Phase 1 of WIA-IND-009 defines the fundamental data formats and structures used throughout the food delivery ecosystem. All data MUST be represented in JSON format with UTF-8 encoding. These schemas ensure interoperability between platforms, restaurants, drivers, and third-party integrations.

**Design Principles:**
- **Simplicity**: Easy to understand and implement
- **Extensibility**: Support for custom fields without breaking compatibility
- **Validation**: Clear rules for data integrity
- **Internationalization**: Support for multiple languages and currencies

---

## Order Data Schema

### Core Order Object

```json
{
  "orderId": "string (required)",
  "version": "string (required, semantic versioning)",
  "status": "enum (required)",
  "customerId": "string (required)",
  "restaurantId": "string (required)",
  "driverId": "string (optional)",
  "orderType": "enum (delivery|pickup|dine-in)",
  "scheduledTime": "ISO8601 timestamp (optional)",
  "items": "array of OrderItem (required)",
  "pricing": "PricingDetails object (required)",
  "deliveryAddress": "Address object (required for delivery)",
  "pickupAddress": "Address object (required)",
  "contactInfo": "ContactInfo object (required)",
  "specialInstructions": "string (optional, max 500 chars)",
  "timestamps": "TimestampCollection object (required)",
  "metadata": "object (optional, custom fields)"
}
```

### Order Status Enum

Valid order states follow a defined state machine:

- `PENDING`: Order placed, awaiting restaurant confirmation
- `CONFIRMED`: Restaurant accepted the order
- `PREPARING`: Food is being prepared
- `READY`: Food ready for pickup
- `ASSIGNED`: Driver assigned to the order
- `PICKED_UP`: Driver collected the order
- `IN_TRANSIT`: Order is being delivered
- `DELIVERED`: Order successfully delivered
- `COMPLETED`: Order fully completed and paid
- `CANCELLED`: Order was cancelled
- `FAILED`: Delivery failed

### OrderItem Schema

```json
{
  "itemId": "string (required)",
  "name": "string (required)",
  "description": "string (optional)",
  "quantity": "number (required, positive integer)",
  "unitPrice": "number (required, decimal with 2 precision)",
  "currency": "string (required, ISO 4217 code)",
  "customizations": "array of Customization (optional)",
  "specialInstructions": "string (optional, max 200 chars)",
  "imageUrl": "string (optional, valid URL)",
  "categoryId": "string (optional)",
  "allergens": "array of string (optional)",
  "nutritionInfo": "NutritionInfo object (optional)"
}
```

### Customization Schema

```json
{
  "customizationId": "string (required)",
  "name": "string (required)",
  "value": "string or array (required)",
  "priceAdjustment": "number (optional, can be negative)",
  "required": "boolean (default: false)"
}
```

### PricingDetails Schema

```json
{
  "subtotal": "number (required, sum of all items)",
  "tax": "number (required)",
  "taxRate": "number (optional, percentage)",
  "deliveryFee": "number (required, 0 for pickup)",
  "serviceFee": "number (optional)",
  "tip": "number (optional)",
  "discount": "number (optional, positive value)",
  "discountCode": "string (optional)",
  "total": "number (required, final amount)",
  "currency": "string (required, ISO 4217 code)"
}
```

**Validation Rules:**
- `total` MUST equal `subtotal + tax + deliveryFee + serviceFee + tip - discount`
- All monetary values MUST be non-negative except `priceAdjustment`
- Prices MUST have exactly 2 decimal places
- Currency code MUST be consistent across all pricing fields

---

## Menu Data Structure

### Restaurant Menu Schema

```json
{
  "menuId": "string (required)",
  "restaurantId": "string (required)",
  "restaurantName": "string (required)",
  "version": "string (required)",
  "lastUpdated": "ISO8601 timestamp (required)",
  "currency": "string (required, ISO 4217)",
  "categories": "array of MenuCategory (required)",
  "availability": "AvailabilitySchedule object (required)",
  "metadata": "object (optional)"
}
```

### MenuCategory Schema

```json
{
  "categoryId": "string (required)",
  "name": "string (required)",
  "description": "string (optional)",
  "displayOrder": "number (required, for sorting)",
  "imageUrl": "string (optional)",
  "items": "array of MenuItem (required)",
  "available": "boolean (default: true)",
  "availabilitySchedule": "AvailabilitySchedule (optional)"
}
```

### MenuItem Schema

```json
{
  "itemId": "string (required, unique within restaurant)",
  "name": "string (required)",
  "description": "string (optional)",
  "price": "number (required)",
  "currency": "string (required, ISO 4217)",
  "imageUrl": "string (optional, valid URL)",
  "images": "array of ImageObject (optional, multiple views)",
  "available": "boolean (required)",
  "preparationTime": "number (minutes, optional)",
  "customizations": "array of CustomizationGroup (optional)",
  "allergens": "array of string (optional)",
  "dietaryInfo": "array of string (optional, vegetarian, vegan, etc)",
  "nutritionInfo": "NutritionInfo object (optional)",
  "tags": "array of string (optional, searchable tags)",
  "displayOrder": "number (optional)"
}
```

### CustomizationGroup Schema

```json
{
  "groupId": "string (required)",
  "name": "string (required, e.g., 'Size', 'Toppings')",
  "required": "boolean (default: false)",
  "minSelections": "number (default: 0)",
  "maxSelections": "number (optional, null for unlimited)",
  "options": "array of CustomizationOption (required)"
}
```

### CustomizationOption Schema

```json
{
  "optionId": "string (required)",
  "name": "string (required)",
  "priceAdjustment": "number (default: 0)",
  "default": "boolean (default: false)",
  "available": "boolean (default: true)"
}
```

### NutritionInfo Schema

```json
{
  "servingSize": "string (optional, e.g., '1 pizza')",
  "calories": "number (optional)",
  "protein": "number (optional, grams)",
  "carbohydrates": "number (optional, grams)",
  "fat": "number (optional, grams)",
  "saturatedFat": "number (optional, grams)",
  "sodium": "number (optional, milligrams)",
  "sugar": "number (optional, grams)",
  "fiber": "number (optional, grams)"
}
```

---

## Driver Data Model

### Driver Profile Schema

```json
{
  "driverId": "string (required, unique identifier)",
  "name": "string (required)",
  "phone": "string (required, E.164 format)",
  "email": "string (optional, valid email)",
  "profileImageUrl": "string (optional)",
  "vehicle": "VehicleInfo object (required)",
  "currentLocation": "Location object (optional)",
  "status": "enum (required)",
  "rating": "number (optional, 0.0 to 5.0)",
  "totalDeliveries": "number (optional)",
  "onlineHours": "number (optional, total hours)",
  "currentOrders": "array of string (order IDs)",
  "maxCapacity": "number (required, max concurrent orders)",
  "certifications": "array of string (optional)",
  "metadata": "object (optional)"
}
```

### Driver Status Enum

- `OFFLINE`: Driver not available
- `AVAILABLE`: Driver online and available for orders
- `ASSIGNED`: Driver assigned to order(s) but not picked up yet
- `PICKING_UP`: Driver at restaurant picking up
- `DELIVERING`: Driver en route to customer
- `ON_BREAK`: Driver temporarily unavailable

### VehicleInfo Schema

```json
{
  "type": "enum (required: bicycle, motorcycle, car, scooter, robot)",
  "make": "string (optional)",
  "model": "string (optional)",
  "licensePlate": "string (optional)",
  "color": "string (optional)",
  "year": "number (optional)",
  "insuranceVerified": "boolean (default: false)",
  "capacity": "object (optional, volume/weight limits)"
}
```

---

## Location Format

### Location Schema

All location data MUST use WGS84 coordinate system (GPS standard).

```json
{
  "latitude": "number (required, -90 to 90)",
  "longitude": "number (required, -180 to 180)",
  "accuracy": "number (optional, meters)",
  "altitude": "number (optional, meters)",
  "heading": "number (optional, 0-360 degrees)",
  "speed": "number (optional, km/h)",
  "timestamp": "ISO8601 (optional, when location was captured)"
}
```

### Address Schema

```json
{
  "street": "string (required)",
  "apartment": "string (optional, unit/apt/suite number)",
  "city": "string (required)",
  "state": "string (optional, state/province)",
  "zipCode": "string (required)",
  "country": "string (required, ISO 3166-1 alpha-2)",
  "latitude": "number (optional but recommended)",
  "longitude": "number (optional but recommended)",
  "deliveryInstructions": "string (optional, max 500 chars)",
  "placeId": "string (optional, Google Places ID or equivalent)"
}
```

---

## Restaurant Profile

### Restaurant Schema

```json
{
  "restaurantId": "string (required, unique identifier)",
  "name": "string (required)",
  "description": "string (optional)",
  "cuisineTypes": "array of string (required)",
  "logoUrl": "string (optional)",
  "bannerUrl": "string (optional)",
  "address": "Address object (required)",
  "phone": "string (required)",
  "email": "string (optional)",
  "website": "string (optional, valid URL)",
  "operatingHours": "OperatingHours object (required)",
  "rating": "number (optional, 0.0 to 5.0)",
  "reviewCount": "number (optional)",
  "priceRange": "number (optional, 1-4 scale)",
  "deliveryZone": "GeoJSON Polygon (optional)",
  "minimumOrder": "number (optional)",
  "estimatedPrepTime": "number (optional, minutes)",
  "features": "array of string (optional, e.g., 'outdoor seating')",
  "licenses": "array of LicenseInfo (optional)",
  "metadata": "object (optional)"
}
```

### OperatingHours Schema

```json
{
  "monday": "DaySchedule (optional)",
  "tuesday": "DaySchedule (optional)",
  "wednesday": "DaySchedule (optional)",
  "thursday": "DaySchedule (optional)",
  "friday": "DaySchedule (optional)",
  "saturday": "DaySchedule (optional)",
  "sunday": "DaySchedule (optional)",
  "specialHours": "array of SpecialHour (optional, holidays)",
  "timezone": "string (required, IANA timezone)"
}
```

### DaySchedule Schema

```json
{
  "open": "time string (required, HH:MM format)",
  "close": "time string (required, HH:MM format)",
  "breaks": "array of TimeRange (optional, closed periods)"
}
```

---

## Customer Profile

### Customer Schema

```json
{
  "customerId": "string (required, unique identifier)",
  "name": "string (required)",
  "email": "string (optional, valid email)",
  "phone": "string (required, E.164 format)",
  "profileImageUrl": "string (optional)",
  "savedAddresses": "array of Address (optional)",
  "savedPaymentMethods": "array of PaymentMethod (optional)",
  "preferences": "CustomerPreferences object (optional)",
  "dietaryRestrictions": "array of string (optional)",
  "allergens": "array of string (optional)",
  "joinDate": "ISO8601 timestamp (optional)",
  "loyaltyPoints": "number (optional)",
  "metadata": "object (optional)"
}
```

### CustomerPreferences Schema

```json
{
  "defaultAddress": "string (address ID)",
  "defaultPayment": "string (payment method ID)",
  "notifications": "NotificationPreferences object",
  "language": "string (ISO 639-1 code)",
  "currency": "string (ISO 4217 code)",
  "dietaryFilters": "array of string"
}
```

---

## Timestamp Standards

All timestamps MUST use ISO 8601 format with timezone information:

**Format:** `YYYY-MM-DDTHH:mm:ss.sssZ`
**Example:** `2025-12-27T12:35:20.123Z`

### TimestampCollection Schema

```json
{
  "placed": "ISO8601 (required, when order was placed)",
  "confirmed": "ISO8601 (optional, restaurant confirmation)",
  "preparing": "ISO8601 (optional, preparation started)",
  "ready": "ISO8601 (optional, ready for pickup)",
  "pickedUp": "ISO8601 (optional, driver picked up)",
  "delivered": "ISO8601 (optional, delivered to customer)",
  "completed": "ISO8601 (optional, fully completed)",
  "estimatedPreparation": "ISO8601 (optional, expected ready time)",
  "estimatedDelivery": "ISO8601 (optional, expected delivery time)"
}
```

---

## Validation Rules

### General Rules

1. **Required Fields**: All fields marked as `required` MUST be present
2. **Data Types**: Values MUST match specified data types
3. **Enums**: Enum values MUST be from specified lists (case-sensitive)
4. **String Lengths**: Respect maximum length constraints where specified
5. **Number Ranges**: Numeric values MUST fall within specified ranges
6. **URL Validation**: URL fields MUST be valid, absolute URLs
7. **Email Validation**: Email fields MUST be RFC 5322 compliant
8. **Phone Validation**: Phone numbers SHOULD use E.164 format

### Specific Validation Rules

**Order Validation:**
- `orderId` MUST be unique within the platform
- `items` array MUST contain at least one item
- `total` MUST equal calculated sum of all pricing components
- `deliveryAddress` is REQUIRED for order type `delivery`

**Menu Validation:**
- `itemId` MUST be unique within restaurant menu
- `price` MUST be positive
- If `customizations.required` is true, `minSelections` MUST be at least 1

**Location Validation:**
- `latitude` MUST be between -90 and 90
- `longitude` MUST be between -180 and 180
- `accuracy` if provided MUST be positive

**Driver Validation:**
- `rating` if provided MUST be between 0.0 and 5.0
- `currentOrders` length MUST NOT exceed `maxCapacity`

---

## Version History

- **v1.0** (2025-12-27): Initial release
- Future versions will maintain backward compatibility where possible

---

## License

This specification is part of the WIA-IND-009 standard.

**Copyright © 2025 SmileStory Inc. / WIA**

Licensed under MIT License. Free to use, modify, and distribute.

**弘益人間 (Benefit All Humanity)**

---

## See Also

- [PHASE-2-API-INTERFACE.md](PHASE-2-API-INTERFACE.md) - API Specifications
- [PHASE-3-PROTOCOL.md](PHASE-3-PROTOCOL.md) - Communication Protocols
- [PHASE-4-INTEGRATION.md](PHASE-4-INTEGRATION.md) - Integration Patterns

---

## Annex A — Conformance Tier Matrix

WIA conformance for WIA-IND-009-food-delivery is evaluated across three tiers:

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

- `wia-standards/standards/WIA-IND-009-food-delivery/api/` — TypeScript SDK skeleton
- `wia-standards/standards/WIA-IND-009-food-delivery/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/WIA-IND-009-food-delivery/simulator/` — interactive browser-based simulator for the PHASE protocol

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
