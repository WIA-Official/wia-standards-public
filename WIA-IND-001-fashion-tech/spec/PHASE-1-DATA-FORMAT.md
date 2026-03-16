# WIA-IND-001: Phase 1 - Data Format Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document specifies the standardized data formats for representing body measurements, garment specifications, and fit data in the WIA-IND-001 Fashion Technology Standard. All implementations MUST support these data formats to ensure interoperability across platforms, devices, and vendors.

## 2. Body Measurement Schema

### 2.1 Primary Data Structure

Body measurements are represented as JSON objects conforming to the following schema:

```json
{
  "@context": "https://wia.org/standards/IND-001/v1",
  "@type": "BodyMeasurement",
  "id": "string (UUID or URI)",
  "version": "1.0",
  "timestamp": "ISO 8601 datetime",
  "subject": {
    "id": "string (user identifier)",
    "birthDate": "ISO 8601 date",
    "gender": "enum: male|female|nonbinary|other|prefer-not-to-say",
    "ethnicity": "string (optional)",
    "customAttributes": {}
  },
  "measurements": {
    "height": {
      "value": "number",
      "unit": "enum: cm|inch",
      "accuracy": "number (±cm or ±inch)",
      "method": "enum: smartphone-scan|professional-scan|manual|estimated",
      "timestamp": "ISO 8601 datetime"
    },
    "weight": {
      "value": "number",
      "unit": "enum: kg|lb",
      "accuracy": "number",
      "method": "string",
      "timestamp": "ISO 8601 datetime"
    },
    "chest": { "...": "same structure as height" },
    "waist": { "...": "same structure as height" },
    "hip": { "...": "same structure as height" },
    "shoulderWidth": { "...": "same structure as height" },
    "sleeveLength": { "...": "same structure as height" },
    "inseam": { "...": "same structure as height" },
    "outseam": { "...": "same structure as height" },
    "armLength": { "...": "same structure as height" },
    "backLength": { "...": "same structure as height" },
    "frontLength": { "...": "same structure as height" },
    "neckCircumference": { "...": "same structure as height" },
    "bicepCircumference": { "...": "same structure as height" },
    "thighCircumference": { "...": "same structure as height" }
  },
  "derived": {
    "bmi": "number (calculated)",
    "bodyType": "enum: hourglass|pear|apple|rectangle|inverted-triangle|oval|trapezoid",
    "fitProfile": "string",
    "customDerivedFields": {}
  },
  "privacy": {
    "sharing": "enum: private|encrypted|pseudonymized|public",
    "retention": "string (ISO 8601 duration)",
    "purpose": "string",
    "consent": {
      "granted": "boolean",
      "timestamp": "ISO 8601 datetime",
      "scope": "array of strings"
    }
  },
  "metadata": {
    "device": "string (device identifier)",
    "software": "string (software version)",
    "environment": "object (capture environment details)",
    "operator": "string (optional)"
  },
  "philosophy": "弘益人間"
}
```

### 2.2 Measurement Definitions

#### 2.2.1 Height
- **Definition:** Vertical distance from floor to crown of head
- **Position:** Standing straight, bare feet together, looking forward
- **Accuracy:** ±0.5cm (smartphone), ±0.3cm (professional)
- **Range:** 100-250 cm

#### 2.2.2 Weight
- **Definition:** Body mass in kilograms or pounds
- **Measurement:** Calibrated scale, minimal clothing
- **Accuracy:** ±0.5 kg or ±1 lb
- **Range:** 30-200 kg

#### 2.2.3 Chest/Bust Circumference
- **Definition:** Circumference at fullest part of chest
- **Position:** Tape parallel to floor, arms relaxed at sides
- **Measurement Point:** Nipple level (male), fullest bust point (female)
- **Accuracy:** ±0.5 cm
- **Range:** 60-150 cm

#### 2.2.4 Waist Circumference
- **Definition:** Circumference at natural waistline
- **Position:** Narrowest part of torso, approximately at navel level
- **Measurement:** Tape parallel to floor, normal breathing
- **Accuracy:** ±0.5 cm
- **Range:** 50-150 cm

#### 2.2.5 Hip Circumference
- **Definition:** Circumference at fullest part of hips and buttocks
- **Position:** Tape parallel to floor around fullest part
- **Accuracy:** ±0.5 cm
- **Range:** 60-160 cm

#### 2.2.6 Shoulder Width
- **Definition:** Distance between shoulder points
- **Measurement:** Across back from shoulder corner to shoulder corner
- **Accuracy:** ±0.5 cm
- **Range:** 30-60 cm

#### 2.2.7 Sleeve Length
- **Definition:** Shoulder point to wrist bone
- **Position:** Arm relaxed, slightly bent
- **Accuracy:** ±0.5 cm
- **Range:** 50-90 cm

#### 2.2.8 Inseam
- **Definition:** Crotch to ankle along inside leg
- **Position:** Legs slightly apart, standing straight
- **Accuracy:** ±0.5 cm
- **Range:** 60-100 cm

## 3. Garment Specification Schema

### 3.1 Primary Data Structure

```json
{
  "@context": "https://wia.org/standards/IND-001/v1",
  "@type": "GarmentSpecification",
  "id": "string (UUID or SKU)",
  "version": "1.0",
  "product": {
    "name": "string",
    "brand": "string",
    "category": "enum: shirt|pants|dress|jacket|shoes|...",
    "subcategory": "string",
    "style": "string",
    "season": "string"
  },
  "sizing": {
    "system": "enum: US|EU|UK|Asian|Universal",
    "sizes": [
      {
        "label": "string (XS|S|M|L|XL|2XL|numeric)",
        "measurements": {
          "chest": { "min": "number", "max": "number", "ideal": "number", "unit": "cm" },
          "waist": { "..." },
          "hip": { "..." },
          "length": { "..." },
          "custom": {}
        },
        "fit": "enum: tight|slim|regular|relaxed|oversized",
        "availability": "boolean",
        "stock": "number (optional)"
      }
    ],
    "grading": {
      "method": "string",
      "increments": {}
    }
  },
  "materials": {
    "primary": {
      "type": "string (cotton|polyester|wool|silk|...)",
      "percentage": "number",
      "origin": "string",
      "sustainability": {
        "organic": "boolean",
        "recycled": "boolean",
        "certifications": "array of strings"
      }
    },
    "secondary": [ "array of material objects" ],
    "fabric": {
      "weight": { "value": "number", "unit": "gsm" },
      "stretch": { "horizontal": "percentage", "vertical": "percentage" },
      "texture": "string",
      "care": "array of strings"
    }
  },
  "3dModel": {
    "available": "boolean",
    "format": "enum: OBJ|FBX|GLTF|glb",
    "url": "string (URL to model file)",
    "lod": "array of level-of-detail URLs",
    "textures": {
      "diffuse": "URL",
      "normal": "URL",
      "roughness": "URL",
      "metalness": "URL"
    }
  },
  "sustainability": {
    "carbonFootprint": { "value": "number", "unit": "kg CO2e" },
    "waterUsage": { "value": "number", "unit": "liters" },
    "recyclability": "percentage",
    "lifespan": { "estimated": "number", "unit": "years" },
    "certifications": "array of strings"
  },
  "philosophy": "弘益人間"
}
```

## 4. Size Chart Format

Size charts provide mapping between body measurements and garment sizes:

```json
{
  "@context": "https://wia.org/standards/IND-001/v1",
  "@type": "SizeChart",
  "brand": "string",
  "category": "string",
  "gender": "string",
  "sizingSystem": "string",
  "units": "enum: metric|imperial",
  "chart": [
    {
      "size": "string",
      "bodyMeasurements": {
        "height": { "min": 160, "max": 170 },
        "chest": { "min": 86, "max": 92 },
        "waist": { "min": 70, "max": 76 },
        "hip": { "min": 92, "max": 98 }
      },
      "garmentMeasurements": {
        "length": 70,
        "shoulders": 42,
        "sleeves": 60,
        "chest": 95
      }
    }
  ],
  "notes": "array of strings with fit guidance",
  "philosophy": "弘益人間"
}
```

## 5. Fit Preference Profile

User fit preferences personalize recommendations:

```json
{
  "@context": "https://wia.org/standards/IND-001/v1",
  "@type": "FitPreferenceProfile",
  "userId": "string",
  "preferences": {
    "global": {
      "fit": "enum: tight|slim|regular|relaxed|oversized",
      "style": "array of strings",
      "brands": [
        {
          "name": "string",
          "preferredSize": "string",
          "notes": "string"
        }
      ]
    },
    "byCategory": {
      "shirts": { "fit": "string", "details": {} },
      "pants": { "fit": "string", "details": {} },
      "dresses": { "fit": "string", "details": {} }
    }
  },
  "history": [
    {
      "productId": "string",
      "size": "string",
      "purchased": "ISO 8601 datetime",
      "returned": "boolean",
      "feedback": {
        "fit": "enum: too-small|slightly-small|perfect|slightly-large|too-large",
        "quality": "number (1-5)",
        "comfort": "number (1-5)",
        "notes": "string"
      }
    }
  ],
  "philosophy": "弘益人間"
}
```

## 6. Validation Rules

### 6.1 Required Fields
- `@context`, `@type`, `id`, `version`, `timestamp` are REQUIRED
- `measurements` object MUST contain at least: `height`, `chest`, `waist`, `hip`
- Each measurement MUST include: `value`, `unit`, `accuracy`, `method`

### 6.2 Data Type Validation
- Numeric fields MUST be positive numbers
- Enum fields MUST match defined values exactly
- ISO 8601 timestamps MUST be valid and include timezone
- Units MUST be consistent within a document or explicitly converted

### 6.3 Range Validation
- Height: 100-250 cm or 39-98 inches
- Weight: 30-200 kg or 66-440 lbs
- Circumferences: 50-160 cm or 20-63 inches
- Length measurements: 30-120 cm or 12-47 inches

### 6.4 Logical Consistency
- Hip ≥ Waist (typically 0.9-1.3x)
- Chest and Hip proportional to Height (0.5-0.7x)
- BMI = weight/(height²) should be 12-50
- Outseam ≥ Inseam

## 7. Units and Conversions

### 7.1 Standard Units
- **Length:** Centimeters (cm) is preferred; inches supported
- **Weight:** Kilograms (kg) is preferred; pounds supported
- **Conversion:** 1 inch = 2.54 cm, 1 lb = 0.453592 kg

### 7.2 Precision
- Length measurements: round to 0.5 cm or 0.25 inch
- Weight measurements: round to 0.5 kg or 0.5 lb
- Percentages: round to 1 decimal place
- Calculations: maintain precision until final rounding

## 8. Extension Mechanism

Implementations MAY extend schemas with custom fields:

```json
{
  "measurements": {
    "...standard fields...",
    "custom:brandSpecificMeasurement": {
      "value": 42,
      "unit": "cm",
      "schema": "https://example.com/schemas/custom-measurement"
    }
  }
}
```

Custom fields MUST:
- Use namespace prefix (e.g., `custom:`, `brand:`)
- Include schema reference when possible
- Not conflict with standard fields
- Be documented for interoperability

## 9. Privacy and Security

### 9.1 Data Protection
- Body measurement data MUST be encrypted at rest (AES-256)
- Transmission MUST use TLS 1.3 or higher
- Access logs MUST be maintained
- User consent MUST be recorded

### 9.2 Anonymization
For aggregate analytics, measurements MAY be anonymized by:
- Removing `subject.id` and other identifiers
- Adding statistical noise (±2 cm)
- Aggregating into ranges rather than exact values
- Removing temporal information

## 10. Examples

### 10.1 Complete Body Measurement

```json
{
  "@context": "https://wia.org/standards/IND-001/v1",
  "@type": "BodyMeasurement",
  "id": "urn:uuid:550e8400-e29b-41d4-a716-446655440000",
  "version": "1.0",
  "timestamp": "2025-01-15T10:30:00Z",
  "subject": {
    "id": "user-12345",
    "birthDate": "1990-05-15",
    "gender": "female"
  },
  "measurements": {
    "height": {
      "value": 165.5,
      "unit": "cm",
      "accuracy": 0.5,
      "method": "smartphone-scan",
      "timestamp": "2025-01-15T10:30:00Z"
    },
    "weight": {
      "value": 58.2,
      "unit": "kg",
      "accuracy": 0.5,
      "method": "digital-scale",
      "timestamp": "2025-01-15T10:25:00Z"
    },
    "chest": {
      "value": 88.0,
      "unit": "cm",
      "accuracy": 0.5,
      "method": "smartphone-scan",
      "timestamp": "2025-01-15T10:30:00Z"
    },
    "waist": {
      "value": 68.5,
      "unit": "cm",
      "accuracy": 0.5,
      "method": "smartphone-scan",
      "timestamp": "2025-01-15T10:30:00Z"
    },
    "hip": {
      "value": 94.0,
      "unit": "cm",
      "accuracy": 0.5,
      "method": "smartphone-scan",
      "timestamp": "2025-01-15T10:30:00Z"
    }
  },
  "derived": {
    "bmi": 21.3,
    "bodyType": "hourglass",
    "fitProfile": "athletic"
  },
  "privacy": {
    "sharing": "encrypted",
    "retention": "P365D",
    "purpose": "size-recommendation",
    "consent": {
      "granted": true,
      "timestamp": "2025-01-15T10:28:00Z",
      "scope": ["measurement", "recommendation", "analytics-aggregate"]
    }
  },
  "metadata": {
    "device": "iPhone 15 Pro",
    "software": "WIA-Scanner v1.2.0",
    "environment": {
      "lighting": "good",
      "temperature": 22.5
    }
  },
  "philosophy": "弘益人間"
}
```

---

**Copyright © 2025 SmileStory Inc. / WIA**
**License:** CC BY 4.0
**弘益人間 - Benefit All Humanity**
