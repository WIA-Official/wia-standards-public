# PHASE 1: DATA FORMAT SPECIFICATION
## WIA-CONTACT-001: First Contact Protocol

> 弘益人間 (홍익인간) · Benefit All Humanity

---

## 1. Introduction

This document defines the data format specifications for storing, transmitting, and processing extraterrestrial signal data within the First Contact Protocol framework.

## 2. Signal Data Format

### 2.1 Base Signal Structure

```json
{
  "version": "1.0",
  "signalId": "SIG-YYYY-NNNN",
  "detectionTimestamp": "ISO8601",
  "observatory": {
    "id": "string",
    "name": "string",
    "location": {
      "latitude": number,
      "longitude": number,
      "altitude": number
    }
  },
  "frequency": {
    "center": number,
    "bandwidth": number,
    "unit": "MHz"
  },
  "source": {
    "rightAscension": number,
    "declination": number,
    "galacticLatitude": number,
    "galacticLongitude": number,
    "epoch": "J2000"
  },
  "strength": {
    "value": number,
    "unit": "dBm"
  },
  "modulation": {
    "type": "am|fm|pm|pulse|binary|complex|unknown",
    "parameters": {}
  },
  "duration": {
    "value": number,
    "unit": "seconds"
  },
  "rawData": "base64|uri",
  "metadata": {}
}
```

### 2.2 Field Specifications

#### 2.2.1 Signal ID Format
- Pattern: `SIG-YYYY-NNNN`
- YYYY: Year of detection
- NNNN: Sequential number (0001-9999)
- Example: `SIG-2025-0042`

#### 2.2.2 Timestamp Format
- Standard: ISO 8601
- Format: `YYYY-MM-DDTHH:mm:ss.sssZ`
- Timezone: UTC
- Example: `2025-12-27T14:23:45.123Z`

#### 2.2.3 Frequency Specification
- Range: 1 MHz - 100 GHz
- Precision: 0.001 MHz (1 kHz)
- Notable frequencies:
  - 1420.4 MHz: Hydrogen line
  - 1666.7 MHz: Hydroxyl line
  - "Water Hole": 1420.4 - 1666.7 MHz

#### 2.2.4 Coordinate System
- Right Ascension: 0° - 360°
- Declination: -90° - +90°
- Epoch: J2000.0 (default)
- Precision: 0.001 degrees

## 3. Verification Data Format

### 3.1 Verification Record

```json
{
  "verificationId": "VER-YYYY-NNNN",
  "signalId": "SIG-YYYY-NNNN",
  "initiatedAt": "ISO8601",
  "completedAt": "ISO8601",
  "observatories": [
    {
      "observatoryId": "string",
      "observatoryName": "string",
      "confirmed": boolean,
      "confidence": number,
      "timestamp": "ISO8601",
      "notes": "string"
    }
  ],
  "consensus": number,
  "authentic": boolean,
  "status": "pending|in-progress|completed|failed"
}
```

### 3.2 Multi-Site Verification Requirements
- Minimum observatories: 3
- Minimum consensus: 0.95 (95%)
- Maximum time window: 48 hours
- Geographic diversity: 3+ continents

## 4. Pattern Analysis Format

### 4.1 Pattern Data Structure

```json
{
  "analysisId": "ANA-YYYY-NNNN",
  "signalId": "SIG-YYYY-NNNN",
  "timestamp": "ISO8601",
  "patternType": "prime-sequence|fibonacci|mathematical-constants|binary-encoding|pictorial|audio-signature|unknown",
  "patterns": [
    {
      "sequence": [],
      "interpretation": "string",
      "confidence": number
    }
  ],
  "mathematicalSignificance": "low|medium|high",
  "artificialProbability": number,
  "entropy": number,
  "decodedContent": "string",
  "recommendations": []
}
```

### 4.2 Pattern Recognition Algorithms
- Prime number detection
- Fibonacci sequence identification
- Mathematical constant recognition
- Binary encoding analysis
- Pictorial data reconstruction
- Audio signature extraction

## 5. Threat Assessment Format

### 5.1 Threat Data Structure

```json
{
  "assessmentId": "THR-YYYY-NNNN",
  "signalId": "SIG-YYYY-NNNN",
  "timestamp": "ISO8601",
  "threatLevel": "benign|caution|elevated|high|critical",
  "indicators": [
    {
      "type": "string",
      "description": "string",
      "severity": "low|medium|high"
    }
  ],
  "confidence": number,
  "recommendation": "string",
  "mitigationStrategies": []
}
```

### 5.2 Threat Levels
- **Benign**: No hostile indicators, peaceful intent likely
- **Caution**: Unknown intent, monitoring required
- **Elevated**: Potentially concerning patterns detected
- **High**: Hostile indicators present
- **Critical**: Immediate threat identified

## 6. Response Message Format

### 6.1 Response Structure

```json
{
  "responseId": "RES-YYYY-NNNN",
  "signalId": "SIG-YYYY-NNNN",
  "createdAt": "ISO8601",
  "type": "mathematical-acknowledgment|pictorial-message|audio-signal|binary-code|custom",
  "content": {
    "encoding": "text|binary|mathematical|pictorial|audio",
    "data": "base64|string",
    "checksum": "sha256"
  },
  "transmission": {
    "frequency": number,
    "power": number,
    "duration": number,
    "targetCoordinates": {}
  },
  "approvals": [
    {
      "authority": "string",
      "approved": boolean,
      "timestamp": "ISO8601",
      "votes": {
        "for": number,
        "against": number,
        "abstain": number
      }
    }
  ],
  "status": "draft|under-review|approved|rejected|transmitted"
}
```

## 7. Data Storage Requirements

### 7.1 Storage Specifications
- **Format**: JSON, HDF5, FITS
- **Compression**: GZIP, BZIP2
- **Encryption**: AES-256
- **Backup**: 3-2-1 strategy (3 copies, 2 media types, 1 offsite)
- **Retention**: Permanent (minimum 100 years)

### 7.2 Access Control
- Classification levels: Public, Restricted, Classified
- Role-based access control (RBAC)
- Audit logging required
- Multi-factor authentication for classified data

## 8. Data Exchange Format

### 8.1 Standard Exchange Format
- Primary: JSON-LD with schema.org ontology
- Secondary: XML with custom XSD schema
- Binary: Protocol Buffers for high-performance scenarios

### 8.2 API Data Format
- RESTful JSON API
- GraphQL support
- gRPC for real-time streaming
- WebSocket for live monitoring

## 9. Validation Rules

### 9.1 Required Fields
- signalId
- detectionTimestamp
- observatory
- frequency
- source
- strength

### 9.2 Data Quality Checks
- Frequency range validation
- Coordinate bounds checking
- Timestamp validity
- Signal strength reasonableness
- Data completeness verification

## 10. Version Control

### 10.1 Versioning Scheme
- Format: MAJOR.MINOR.PATCH
- Current version: 1.0.0
- Backward compatibility guaranteed within major versions

### 10.2 Migration Procedures
- Automated migration tools provided
- 6-month deprecation notice for breaking changes
- Legacy format support: 3 years minimum

---

**Document Version**: 1.0.0
**Last Updated**: 2025-12-27
**Status**: Active
**Maintainer**: WIA Technical Committee

© 2025 SmileStory Inc. / WIA · CC BY-SA 4.0
