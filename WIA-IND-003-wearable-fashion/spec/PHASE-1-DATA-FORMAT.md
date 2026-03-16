# WIA-IND-003: Phase 1 - Data Format Specification
## Wearable Fashion Data Standards

**Version:** 1.0
**Status:** Final
**Date:** 2025-01-15
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

This specification defines data formats for wearable fashion devices, ensuring interoperability across manufacturers, platforms, and applications. Phase 1 establishes the foundation for device identification, sensor data representation, and fashion metadata.

## 2. Device Data Schema

### 2.1 Device Information Format

```json
{
  "standard": "WIA-IND-003",
  "version": "1.0",
  "device": {
    "id": "string (unique identifier)",
    "type": "enum [smartwatch, smartring, smartbracelet, smartnecklace, smartglasses, smartbelt, smartshoes, smartbag]",
    "brand": "string",
    "model": "string",
    "firmware": "string (semver format)",
    "manufactured": "ISO 8601 date",
    "serial": "string (unique serial number)"
  },
  "capabilities": {
    "sensors": ["array of sensor types"],
    "connectivity": ["array of connectivity protocols"],
    "display": "boolean",
    "haptic": "boolean",
    "waterproof": "string (IP rating)"
  },
  "fashion": {
    "material": "string or array",
    "colors": ["array of available colors"],
    "sizes": ["array of available sizes"],
    "style": "string (design category)",
    "weight": "number (grams)",
    "dimensions": {
      "length": "number (mm)",
      "width": "number (mm)",
      "height": "number (mm)"
    }
  }
}
```

### 2.2 Required Fields

- `standard`: Must be "WIA-IND-003"
- `version`: Specification version (currently "1.0")
- `device.id`: Globally unique identifier (GUID format recommended)
- `device.type`: Must be one of the enumerated types
- `device.brand`: Manufacturer brand name
- `device.model`: Model designation

### 2.3 Optional Fields

- `firmware`: Current firmware version
- `serial`: Manufacturing serial number
- `capabilities.*`: Device capabilities (defaults to empty arrays/false)
- `fashion.*`: Fashion-specific metadata

## 3. Sensor Data Formats

### 3.1 Optical Sensors (Heart Rate, SpO2)

```json
{
  "type": "optical",
  "timestamp": "ISO 8601 datetime",
  "deviceId": "string",
  "measurements": {
    "heartRate": {
      "value": "number (bpm)",
      "unit": "bpm",
      "quality": "enum [high, medium, low]",
      "confidence": "number (0.0-1.0)"
    },
    "spo2": {
      "value": "number (percentage)",
      "unit": "percent",
      "quality": "enum [high, medium, low]"
    }
  }
}
```

### 3.2 Motion Sensors (Accelerometer, Gyroscope)

```json
{
  "type": "motion",
  "timestamp": "ISO 8601 datetime",
  "deviceId": "string",
  "accelerometer": {
    "x": "number (m/s²)",
    "y": "number (m/s²)",
    "z": "number (m/s²)",
    "unit": "m/s²"
  },
  "gyroscope": {
    "x": "number (rad/s)",
    "y": "number (rad/s)",
    "z": "number (rad/s)",
    "unit": "rad/s"
  },
  "magnetometer": {
    "x": "number (µT)",
    "y": "number (µT)",
    "z": "number (µT)",
    "unit": "µT"
  }
}
```

### 3.3 Environmental Sensors

```json
{
  "type": "environmental",
  "timestamp": "ISO 8601 datetime",
  "deviceId": "string",
  "temperature": {
    "value": "number",
    "unit": "enum [celsius, fahrenheit, kelvin]"
  },
  "humidity": {
    "value": "number (0-100)",
    "unit": "percent"
  },
  "pressure": {
    "value": "number",
    "unit": "enum [hPa, mmHg, psi]"
  },
  "ambientLight": {
    "value": "number",
    "unit": "lux"
  }
}
```

### 3.4 Battery and Power Status

```json
{
  "type": "power",
  "timestamp": "ISO 8601 datetime",
  "deviceId": "string",
  "battery": {
    "level": "number (0-100)",
    "unit": "percent",
    "charging": "boolean",
    "voltage": "number (volts, optional)",
    "temperature": "number (celsius, optional)",
    "cycleCount": "integer (optional)",
    "health": "number (0-100, optional)"
  }
}
```

## 4. Activity Data Format

### 4.1 Step Count and Activity

```json
{
  "type": "activity",
  "timestamp": "ISO 8601 datetime",
  "deviceId": "string",
  "period": {
    "start": "ISO 8601 datetime",
    "end": "ISO 8601 datetime"
  },
  "steps": {
    "count": "integer",
    "distance": "number (meters)",
    "calories": "number (kcal)"
  },
  "activity": {
    "type": "enum [walking, running, cycling, swimming, other]",
    "duration": "number (seconds)",
    "intensity": "enum [light, moderate, vigorous]"
  }
}
```

## 5. Fashion Metadata Standard

### 5.1 Product Information

```json
{
  "product": {
    "sku": "string",
    "name": "string",
    "category": "enum [watch, ring, bracelet, necklace, glasses, belt, shoes, bag]",
    "collection": "string (optional)",
    "season": "string (e.g., 'Spring/Summer 2025', optional)",
    "releaseDate": "ISO 8601 date"
  },
  "design": {
    "designer": "string (optional)",
    "style": "string (e.g., 'modern minimalist', 'classic luxury')",
    "targetGender": "enum [unisex, mens, womens, other]",
    "ageGroup": "enum [adult, youth, child, all]"
  },
  "materials": [
    {
      "component": "enum [case, band, display, accent]",
      "material": "string (e.g., 'titanium', '316L stainless steel')",
      "finish": "string (e.g., 'brushed', 'polished')",
      "color": "string",
      "sustainable": "boolean (optional)"
    }
  ],
  "certifications": [
    {
      "type": "enum [biocompatibility, waterproof, safety, environmental]",
      "standard": "string (e.g., 'ISO 10993', 'IP68')",
      "certifiedBy": "string",
      "date": "ISO 8601 date"
    }
  ]
}
```

### 5.2 Customization Options

```json
{
  "customization": {
    "colors": ["array of hex colors"],
    "sizes": ["array of size codes"],
    "engraving": {
      "available": "boolean",
      "maxCharacters": "integer",
      "fonts": ["array of font names"]
    },
    "interchangeable": {
      "bands": "boolean",
      "faces": "boolean",
      "decorations": "boolean"
    }
  }
}
```

## 6. Data Validation Rules

### 6.1 Timestamp Requirements

- All timestamps MUST use ISO 8601 format with timezone
- Example: `2025-01-15T14:30:00Z` or `2025-01-15T14:30:00+09:00`
- Past timestamps are valid for historical data
- Future timestamps (beyond 1 minute) SHOULD be rejected

### 6.2 Numeric Ranges

- Heart rate: 40-200 bpm (values outside trigger warnings)
- SpO2: 0-100 percent
- Battery level: 0-100 percent
- Temperature: -40 to 85 °C (device operating range)
- Accelerometer: ±8g typical, ±16g maximum

### 6.3 String Validation

- Device IDs: Alphanumeric with hyphens, 8-64 characters
- Brand/Model: 1-100 characters, UTF-8 supported
- Colors: Hex format (#RRGGBB) or standard color names

## 7. Data Encoding and Transport

### 7.1 JSON Encoding

- UTF-8 encoding required
- Minified JSON acceptable for bandwidth efficiency
- Pretty-printed JSON recommended for human readability

### 7.2 Binary Encoding (Optional)

- Protocol Buffers (protobuf) supported for efficient transmission
- CBOR (Concise Binary Object Representation) supported
- Schema files provided separately

### 7.3 Compression

- gzip compression recommended for data transfer
- Compression MUST NOT be applied to real-time streaming data

## 8. Privacy and Security

### 8.1 Personal Data Handling

- All sensor data containing health information is considered personal data
- Devices MUST support data encryption at rest and in transit
- Users MUST have ability to export and delete all personal data

### 8.2 Anonymization

When sharing data for research or analytics:
- Remove or hash device IDs
- Remove timestamps or round to hour precision
- Remove location data if not essential
- Aggregate data when possible

## 9. Versioning and Compatibility

### 9.1 Specification Versioning

- Major version changes indicate breaking changes
- Minor version changes add optional fields
- Patch version changes fix specification errors

### 9.2 Backward Compatibility

- Parsers MUST ignore unknown fields
- Required fields MUST NOT be changed or removed
- New optional fields MAY be added in minor versions

## 10. Example Complete Data Packet

```json
{
  "standard": "WIA-IND-003",
  "version": "1.0",
  "timestamp": "2025-01-15T14:30:00Z",
  "device": {
    "id": "WF-SW-12345-ABCDEF",
    "type": "smartwatch",
    "brand": "FashionTech",
    "model": "Elegance Pro X",
    "firmware": "2.1.0"
  },
  "sensors": {
    "heartRate": {
      "value": 72,
      "unit": "bpm",
      "quality": "high"
    },
    "steps": {
      "count": 5420,
      "distance": 3852
    },
    "battery": {
      "level": 85,
      "charging": false
    }
  },
  "philosophy": "弘益人間 - Data serving humanity"
}
```

---

**Philosophy Note:** 弘益人間 (Benefit All Humanity)

This data format specification serves humanity by:
- Ensuring interoperability across devices and platforms
- Protecting user privacy and data ownership
- Supporting health and wellness through standardized monitoring
- Enabling innovation through clear, documented standards

---

**© 2025 SmileStory Inc. / WIA**
**WIA-IND-003 Phase 1 Specification v1.0**
