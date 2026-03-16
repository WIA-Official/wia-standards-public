# WIA-UNI-015: Peace Monitoring - Phase 1: Data Format

> **Version:** 1.0.0
> **Status:** Complete
> **Last Updated:** 2025-12-25

## 1. Overview

This specification defines the data formats for peace monitoring activities on the Korean Peninsula, including arms control verification, DMZ monitoring, troop movement tracking, and confidence-building measures. The format ensures interoperability between North Korea, South Korea, and international observers (UN, IAEA, NNSC).

## 2. Core Data Schemas

### 2.1 Monitoring Event Schema

```typescript
interface MonitoringEvent {
  monitoringId: string;           // Format: MON-{timestamp}
  timestamp: string;              // ISO 8601 format
  type: MonitoringType;
  zone: GeographicZone;
  coordinates: GeoCoordinates;
  description: string;
  status: EventStatus;
  verificationRequired: boolean;
  reportedBy: string;            // Organization/system identifier
  confidenceLevel: number;       // 0.0 - 1.0
  attachments?: Attachment[];
}

enum MonitoringType {
  ARMS_INVENTORY = "ARMS_INVENTORY",
  TROOP_MOVEMENT = "TROOP_MOVEMENT",
  DMZ_SENSOR = "DMZ_SENSOR",
  CONFIDENCE_BUILDING = "CONFIDENCE_BUILDING",
  VERIFICATION_REQUEST = "VERIFICATION_REQUEST",
  INCIDENT_REPORT = "INCIDENT_REPORT"
}

enum GeographicZone {
  DMZ = "DMZ",                   // Demilitarized Zone
  NLL = "NLL",                   // Northern Limit Line (sea)
  NORTH_KOREA = "NORTH_KOREA",
  SOUTH_KOREA = "SOUTH_KOREA",
  JSA = "JSA",                   // Joint Security Area
  INTERNATIONAL = "INTERNATIONAL"
}

enum EventStatus {
  ACTIVE = "ACTIVE",
  RESOLVED = "RESOLVED",
  UNDER_INVESTIGATION = "UNDER_INVESTIGATION",
  VERIFIED = "VERIFIED",
  ESCALATED = "ESCALATED"
}
```

### 2.2 Arms Inventory Schema

```typescript
interface ArmsInventory {
  inventoryId: string;
  declaringParty: "NORTH_KOREA" | "SOUTH_KOREA";
  timestamp: string;
  location: Location;
  categories: WeaponCategory[];
  totalItems: number;
  verificationStatus: VerificationStatus;
  verifiedBy?: string[];         // Observer organizations
  nextInspectionDue?: string;
}

interface WeaponCategory {
  category: string;              // e.g., "ARTILLERY", "TANKS", "AIRCRAFT"
  subcategory?: string;          // e.g., "HEAVY_ARTILLERY", "MAIN_BATTLE_TANK"
  quantity: number;
  serialNumbers?: string[];
  specifications?: {
    model: string;
    caliber?: string;
    range?: number;              // in kilometers
    operational: boolean;
  };
}

enum VerificationStatus {
  DECLARED = "DECLARED",
  INSPECTION_SCHEDULED = "INSPECTION_SCHEDULED",
  VERIFIED = "VERIFIED",
  DISCREPANCY_FOUND = "DISCREPANCY_FOUND",
  PENDING_CLARIFICATION = "PENDING_CLARIFICATION"
}
```

### 2.3 DMZ Sensor Data Schema

```typescript
interface DMZSensorData {
  sensorId: string;
  sensorType: SensorType;
  location: GeoCoordinates;
  timestamp: string;
  readings: SensorReading[];
  alertLevel: AlertLevel;
  processingStatus: "RAW" | "PROCESSED" | "ANALYZED";
  metadata: {
    installationDate: string;
    lastMaintenance: string;
    operationalStatus: "ACTIVE" | "DEGRADED" | "OFFLINE";
    calibrationStatus: "CALIBRATED" | "NEEDS_CALIBRATION";
  };
}

enum SensorType {
  SEISMIC = "SEISMIC",           // Detects ground movement, explosions
  ACOUSTIC = "ACOUSTIC",         // Sound detection
  RADAR = "RADAR",               // Movement tracking
  INFRARED = "INFRARED",         // Heat signatures
  RADIATION = "RADIATION",       // Nuclear monitoring
  CHEMICAL = "CHEMICAL",         // Chemical weapons detection
  OPTICAL = "OPTICAL"            // Visual/camera
}

interface SensorReading {
  timestamp: string;
  value: number;
  unit: string;
  threshold: number;
  thresholdExceeded: boolean;
  confidence: number;
}

enum AlertLevel {
  NONE = "NONE",
  LOW = "LOW",
  MEDIUM = "MEDIUM",
  HIGH = "HIGH",
  CRITICAL = "CRITICAL"
}
```

### 2.4 Troop Movement Schema

```typescript
interface TroopMovement {
  movementId: string;
  timestamp: string;
  party: "NORTH_KOREA" | "SOUTH_KOREA";
  movementType: "DEPLOYMENT" | "WITHDRAWAL" | "ROTATION" | "EXERCISE";
  origin: Location;
  destination: Location;
  units: MilitaryUnit[];
  estimatedDuration: number;     // in hours
  purpose?: string;
  notificationGiven: boolean;
  observersInvited: string[];
  complianceStatus: ComplianceStatus;
}

interface MilitaryUnit {
  unitId: string;
  unitType: "INFANTRY" | "ARMOR" | "ARTILLERY" | "AIR_DEFENSE" | "SUPPORT";
  personnelCount: number;
  equipmentCount: number;
  commandLevel: "BATTALION" | "REGIMENT" | "BRIGADE" | "DIVISION" | "CORPS";
}

enum ComplianceStatus {
  COMPLIANT = "COMPLIANT",
  NON_COMPLIANT = "NON_COMPLIANT",
  EXEMPTED = "EXEMPTED",
  UNDER_REVIEW = "UNDER_REVIEW"
}
```

### 2.5 Confidence Building Measure Schema

```typescript
interface ConfidenceBuildingMeasure {
  cbmId: string;
  type: CBMType;
  initiatingParty: "NORTH_KOREA" | "SOUTH_KOREA" | "JOINT";
  timestamp: string;
  status: "PROPOSED" | "SCHEDULED" | "IN_PROGRESS" | "COMPLETED" | "CANCELLED";
  participants: Participant[];
  objectives: string[];
  outcomes?: Outcome[];
}

enum CBMType {
  MILITARY_HOTLINE_TEST = "MILITARY_HOTLINE_TEST",
  JOINT_INSPECTION = "JOINT_INSPECTION",
  INFORMATION_EXCHANGE = "INFORMATION_EXCHANGE",
  NOTIFICATION_OF_EXERCISES = "NOTIFICATION_OF_EXERCISES",
  JOINT_SEARCH_AND_RESCUE = "JOINT_SEARCH_AND_RESCUE",
  DMZ_DEMINING = "DMZ_DEMINING",
  CULTURAL_EXCHANGE = "CULTURAL_EXCHANGE"
}

interface Participant {
  party: string;
  role: "LEAD" | "PARTICIPANT" | "OBSERVER";
  representatives: string[];
}

interface Outcome {
  timestamp: string;
  description: string;
  agreementsReached: string[];
  followUpRequired: boolean;
}
```

## 3. Geolocation Data

```typescript
interface GeoCoordinates {
  latitude: number;              // Decimal degrees
  longitude: number;
  altitude?: number;             // meters above sea level
  accuracy?: number;             // meters
  datum?: "WGS84" | "Korea2000";
}

interface Location {
  name: string;
  coordinates: GeoCoordinates;
  zone: GeographicZone;
  administrativeArea?: string;   // Province/region
  militaryGrid?: string;         // MGRS coordinate
}
```

## 4. Verification and Audit

```typescript
interface VerificationRecord {
  verificationId: string;
  targetEventId: string;
  verifierOrganization: string;  // e.g., "UNC", "IAEA", "NNSC"
  verificationDate: string;
  methodology: string[];         // e.g., ["ON_SITE_INSPECTION", "SATELLITE_IMAGERY"]
  findings: Finding[];
  conclusion: "VERIFIED" | "PARTIALLY_VERIFIED" | "NOT_VERIFIED" | "INCONCLUSIVE";
  digitalSignature: string;
}

interface Finding {
  category: string;
  observation: string;
  evidenceType: "VISUAL" | "SENSOR_DATA" | "DOCUMENTATION" | "TESTIMONY";
  evidenceId?: string;
  conformity: "CONFORMING" | "NON_CONFORMING" | "UNCLEAR";
}
```

## 5. Attachment Schema

```typescript
interface Attachment {
  attachmentId: string;
  type: "PHOTO" | "VIDEO" | "DOCUMENT" | "SENSOR_DATA" | "REPORT";
  filename: string;
  mimeType: string;
  sizeBytes: number;
  hash: string;                  // SHA-256 hash for integrity
  uploadedBy: string;
  uploadTimestamp: string;
  classification: "PUBLIC" | "RESTRICTED" | "CONFIDENTIAL" | "SECRET";
  encryptionUsed: boolean;
}
```

## 6. Validation Rules

### 6.1 Required Fields
- All records MUST have: `id`, `timestamp`, `type`
- Coordinates MUST be within Korean Peninsula bounds (33°-43°N, 124°-132°E)
- Timestamps MUST be in ISO 8601 format with timezone
- All IDs MUST follow format: `{PREFIX}-{timestamp}` or UUID v4

### 6.2 Data Constraints
- Confidence levels: 0.0 ≤ value ≤ 1.0
- Alert levels: Must be one of defined enum values
- Geographic coordinates: Latitude ±90°, Longitude ±180°
- Weapon quantities: Must be non-negative integers

### 6.3 Business Rules
- Arms inventory declarations MUST be submitted quarterly
- Troop movements >1000 personnel REQUIRE 72-hour advance notice
- DMZ sensor alerts at CRITICAL level MUST trigger automatic notification
- All verification records MUST include digital signature

## 7. Example Data

### 7.1 DMZ Sensor Alert Example

```json
{
  "sensorId": "DMZ-SENSOR-1234",
  "sensorType": "SEISMIC",
  "location": {
    "latitude": 38.123456,
    "longitude": 127.234567,
    "altitude": 45,
    "datum": "WGS84"
  },
  "timestamp": "2025-12-25T14:30:00Z",
  "readings": [
    {
      "timestamp": "2025-12-25T14:30:00Z",
      "value": 2.4,
      "unit": "RICHTER",
      "threshold": 2.0,
      "thresholdExceeded": true,
      "confidence": 0.95
    }
  ],
  "alertLevel": "MEDIUM",
  "processingStatus": "ANALYZED",
  "metadata": {
    "installationDate": "2024-01-15",
    "lastMaintenance": "2025-11-01",
    "operationalStatus": "ACTIVE",
    "calibrationStatus": "CALIBRATED"
  }
}
```

### 7.2 Verification Request Example

```json
{
  "monitoringId": "MON-1735140600",
  "timestamp": "2025-12-25T15:00:00Z",
  "type": "VERIFICATION_REQUEST",
  "zone": "NORTH_KOREA",
  "coordinates": {
    "latitude": 39.456789,
    "longitude": 126.789012
  },
  "description": "Request for inspection of reported arms facility at Yongbyon",
  "status": "ACTIVE",
  "verificationRequired": true,
  "reportedBy": "SOUTH_KOREA_DEFENSE_MINISTRY",
  "confidenceLevel": 0.87
}
```

## 8. Integration Points

- **WIA-UNI-003**: Family reunion data during peace process
- **WIA-UNI-001**: Unified ID system for personnel credentials
- **WIA-SEC-xxx**: Encryption standards for sensitive data
- **UN Systems**: Peace monitoring data exchange
- **IAEA Safeguards**: Nuclear monitoring integration

## 9. Compliance & Certification

Organizations implementing WIA-UNI-015 Phase 1 must:
1. Support all core schemas (Sections 2.1-2.5)
2. Validate data according to Section 6
3. Maintain data integrity with cryptographic hashing
4. Provide audit trail for all data modifications
5. Support JSON and Protocol Buffer serialization

---

**弘益人間 (Benefit All Humanity)**
© 2025 WIA - World Certification Industry Association
Licensed under MIT License
