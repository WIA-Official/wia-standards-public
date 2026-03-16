# WIA-AUTO-022 PHASE 1: Data Format

> **Version:** 1.0.0
> **Status:** Active
> **Last Updated:** 2025-12-27
> **Focus:** JSON Schemas, EDR formats, telemetry structures

---

## Overview

Phase 1 of the WIA-AUTO-022 standard focuses on Data Format implementation, providing comprehensive specifications for standardized data structures across all vehicle safety systems. This phase establishes the foundational data contracts that enable interoperability and forms the basis for all subsequent phases.

**弘益人間 (Benefit All Humanity)** - Standardized data formats enable fleet-wide learning, accelerating safety improvements that save lives globally.

---

## Objectives

### Primary Goals

1. **Data Standardization**: Establish unified data format standards across the automotive industry
2. **Interoperability**: Enable seamless data exchange between different manufacturers, systems, and stakeholders
3. **Completeness**: Ensure all safety-critical data is captured with sufficient detail for analysis
4. **Accessibility**: Make safety data machine-readable and human-understandable
5. **Versionability**: Support schema evolution while maintaining backward compatibility

### Success Criteria

- [x] All data schemas documented using JSON Schema Draft 2020-12
- [x] Validation tools available in major programming languages
- [x] Real-world crash data successfully parsed with 100% schema compliance
- [x] EDR format adopted by at least 5 major OEMs
- [x] Data format reduces integration costs by minimum 60%

---

## Core Data Structures

### 1. Crash Event Data

**Purpose:** Comprehensive crash event recording for post-incident analysis and continuous improvement.

**Schema:** `crash-event-data.schema.json`

**Required Fields:**
- `timestamp` (ISO 8601 date-time): Exact time of crash detection
- `vin` (string, 17 characters): Vehicle Identification Number
- `crash_type` (enum): frontal | side | rear | rollover | oblique
- `delta_v` (object): Change in velocity (longitudinal, lateral, vertical in km/h)
- `peak_acceleration` (object): Maximum acceleration in each axis (g-forces)
- `airbag_deployment` (array): Timing and stage for each airbag position
- `belt_pretensioner` (object): Activation status and timing for each seat
- `edr_status` (string): EDR recording status (complete | partial | failed)

**Optional Fields:**
- `pre_crash_data` (array): 5 seconds of data before crash detection
- `crash_pulse` (array): Millisecond-resolution acceleration time series
- `active_safety_interventions` (array): AEB, ESC activations during event
- `post_crash_data` (object): Vehicle state immediately after crash
- `environmental_conditions` (object): Weather, road surface, lighting

**Example:**
```json
{
  "timestamp": "2025-12-27T14:23:45.123Z",
  "vin": "WDB1234567890ABCD",
  "crash_type": "frontal",
  "delta_v": {
    "longitudinal": -45.3,
    "lateral": 12.1,
    "vertical": -2.3
  },
  "peak_acceleration": {
    "x": -38.5,
    "y": 8.2,
    "z": -2.1
  },
  "airbag_deployment": [
    {"position": "driver_frontal", "time_ms": 28, "stage": 2},
    {"position": "passenger_frontal", "time_ms": 32, "stage": 2},
    {"position": "curtain_left", "time_ms": 18, "stage": 1}
  ],
  "belt_pretensioner": {
    "driver": {"activated": true, "time_ms": 12},
    "passenger": {"activated": true, "time_ms": 12}
  },
  "edr_status": "complete"
}
```

---

### 2. Safety System Status

**Purpose:** Real-time monitoring of all safety system health and operational status.

**Schema:** `safety-system-status.schema.json`

**Structure:**
```json
{
  "timestamp": "2025-12-27T14:23:40Z",
  "vehicle_id": "WDB1234567890ABCD",
  "systems": {
    "abs": {
      "status": "operational",
      "last_test": "2025-12-20",
      "fault_codes": []
    },
    "esc": {
      "status": "operational",
      "calibration_date": "2024-12-15"
    },
    "airbags": {
      "driver": {"status": "armed", "deployment_count": 0},
      "passenger": {"status": "suppressed", "reason": "child_detected"}
    },
    "aeb": {
      "status": "active",
      "sensitivity": "normal",
      "range": 145.3
    },
    "ldw": {
      "status": "active",
      "last_calibration": "2024-11-20"
    }
  }
}
```

**Update Frequency:** Minimum 10 Hz for real-time systems, 1 Hz for diagnostic monitoring

---

### 3. NCAP Rating Data

**Purpose:** Standardized representation of crash test results and safety ratings.

**Schema:** `ncap-rating.schema.json`

**Key Components:**
- Overall star rating (1-5)
- Category scores (adult occupant, child occupant, VRU, safety assist)
- Individual test results with injury metrics
- Testing authority metadata
- Test date and vehicle variant information

**Example:**
```json
{
  "program": "Euro NCAP",
  "year": 2025,
  "vehicle": {
    "make": "Example",
    "model": "SafetyCar",
    "variant": "Premium"
  },
  "overall_rating": 5,
  "scores": {
    "adult_occupant": {
      "percentage": 92,
      "points": 35.0,
      "max_points": 38.0
    },
    "child_occupant": {
      "percentage": 87,
      "points": 42.6,
      "max_points": 49.0
    }
  }
}
```

---

### 4. Injury Biomechanics Data

**Purpose:** Capture comprehensive injury metrics from crash testing.

**Schema:** `injury-biomechanics.schema.json`

**Injury Criteria Covered:**
- HIC-15 (Head Injury Criterion, 15ms window)
- HIC-36 (36ms window for severe impacts)
- Chest compression (mm and percentage)
- Chest acceleration (3ms clip, g-forces)
- Femur load (kN, each leg)
- Tibia index (dimensionless)
- Neck forces and moments (tension, compression, shear)

**Data Resolution:**
- Sampling rate: 10,000 Hz (10 kHz) minimum
- Timestamp precision: Microsecond
- Multi-dummy support: Multiple test dummies per test
- Position mapping: Maps sensor readings to anatomical locations

---

## Implementation Guide

### Step 1: Schema Integration (Week 1)

**Tasks:**
1. Download official WIA-AUTO-022 JSON schemas from GitHub repository
2. Integrate validation library:
   - JavaScript/Node.js: `npm install ajv ajv-formats`
   - Python: `pip install jsonschema`
   - Go: `go get github.com/xeipuuv/gojsonschema`
   - Java: Add `json-schema-validator` to pom.xml
3. Write basic validation unit tests
4. Establish CI/CD pipeline to validate against schemas

**Example (JavaScript):**
```javascript
const Ajv = require('ajv');
const addFormats = require('ajv-formats');
const crashEventSchema = require('./schemas/crash-event-data.schema.json');

const ajv = new Ajv();
addFormats(ajv);

const validate = ajv.compile(crashEventSchema);

function validateCrashData(data) {
  const valid = validate(data);
  if (!valid) {
    console.error('Validation errors:', validate.errors);
    return false;
  }
  return true;
}
```

### Step 2: EDR Update (Weeks 2-4)

**Tasks:**
1. Modify EDR firmware to output WIA-AUTO-022 compliant JSON
2. Implement buffering for pre-crash data (5 seconds minimum)
3. Ensure non-volatile storage survives crash conditions
4. Add checksums for data integrity verification
5. Test with simulated crash scenarios

**Technical Considerations:**
- Storage: Minimum 128 MB non-volatile memory for EDR
- Write speed: Must complete within 500ms of crash detection
- Durability: Survive 250g impact, -40°C to +85°C temperature range
- Power: Independent power backup (capacitor or battery) for minimum 30 seconds

### Step 3: Data Pipeline (Weeks 5-6)

**Tasks:**
1. Implement data ingestion pipeline for crash data
2. Set up validation at ingestion point
3. Establish data lake/warehouse for long-term storage
4. Create ETL processes for analytics
5. Implement access controls and audit logging

**Architecture:**
```
Vehicle EDR → Wireless Upload → API Gateway → Validation → Data Lake
                                     ↓
                              Compliance Check
                                     ↓
                            Analytics Pipeline
```

---

## Validation and Compliance

### Automated Validation

**Tools:**
- Schema validators (Ajv, jsonschema, etc.)
- Custom business rule validators
- Continuous compliance monitoring

**Validation Levels:**
1. **Syntactic**: JSON structure and types conform to schema
2. **Semantic**: Values within acceptable ranges (e.g., velocity 0-300 km/h)
3. **Temporal**: Event sequences are logically consistent
4. **Cross-field**: Relationships between fields are valid

### Compliance Testing

**Test Cases:**
- 100 synthetic crash scenarios covering all crash types
- Edge cases: Maximum/minimum values, missing optional fields
- Error conditions: Invalid formats, out-of-range values
- Performance: Validation completes in < 10ms for typical crash event

**Passing Criteria:**
- 100% schema validation pass rate
- All edge cases handled gracefully
- Error messages are clear and actionable

---

## Best Practices

### Data Quality

1. **Completeness**: Capture all required fields, include optional fields when available
2. **Accuracy**: Calibrate sensors regularly, validate data ranges
3. **Timeliness**: Minimize latency from event occurrence to data recording
4. **Consistency**: Use consistent units, timestamp formats, field names

### Schema Evolution

1. **Versioning**: Include schema version in all data instances
2. **Backward Compatibility**: New versions must parse old data
3. **Deprecation Policy**: 18-month notice before field removal
4. **Migration Tools**: Provide automated converters for version upgrades

### Security

1. **Encryption at Rest**: AES-256 for stored crash data
2. **Encryption in Transit**: TLS 1.3 for data transmission
3. **Access Control**: RBAC with principle of least privilege
4. **Audit Logging**: Log all data access and modifications

---

## Common Challenges

### Challenge 1: Legacy System Integration

**Problem:** Existing EDRs output proprietary binary formats

**Solution:**
- Develop adapter layer to convert binary to JSON
- Use intermediate parsing library for legacy formats
- Implement dual-mode EDR (legacy + WIA-AUTO-022) during transition
- Provide migration tools for historical data

### Challenge 2: Data Volume Management

**Problem:** High-frequency sensor data generates massive volumes

**Solution:**
- Implement intelligent sampling (higher frequency during events)
- Use data compression (gzip, Protocol Buffers for telemetry)
- Establish data retention policies (hot/warm/cold storage tiers)
- Aggregate historical data for analytics

### Challenge 3: Schema Validation Performance

**Problem:** Real-time validation adds latency

**Solution:**
- Pre-compile schemas for faster validation
- Use streaming parsers for large datasets
- Implement caching for repeated validations
- Batch validation for non-critical paths

---

## References

- JSON Schema Specification: https://json-schema.org/draft/2020-12/schema
- ISO 18571:2014 - Road transport and traffic telematics
- SAE J1698 - Event Data Recorder (EDR)
- NHTSA 49 CFR Part 563 - Event Data Recorders
- WIA-AUTO-022 Full Specification: https://wia-standards.org/auto-022

---

**Document Version:** 1.0.0
**Last Modified:** 2025-12-27
**Status:** Active

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA
MIT License
