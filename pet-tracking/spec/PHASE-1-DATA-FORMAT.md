# Phase 1: Pet Tracking Data Format Specification

## WIA-PET-TRACKING Data Format Standard

**Version**: 1.0.0  
**Date**: 2025-12-25  
**Status**: Active  
**Standard ID**: WIA-PET-008-PHASE1  
**Primary Color**: #F59E0B (Amber)

---

## 1. Overview

### 1.1 Purpose

WIA-PET-TRACKING is a comprehensive standard for real-time pet location tracking, geofencing, and lost pet recovery systems. This Phase 1 specification defines the core data formats and structures that all implementations MUST support for interoperability.

**Core Objectives**:
- Standardize location data representation across all implementations
- Enable seamless data exchange between devices, apps, and services
- Support multiple GNSS systems and positioning methods
- Provide flexible yet consistent data structures
- Ensure privacy and security in data handling

### 1.2 Scope

This phase covers:

| Domain | Description |
|--------|-------------|
| **Location Data** | GPS coordinates, accuracy, timestamps |
| **Geofence Definitions** | Circular, polygonal, and corridor geofences |
| **Device Metadata** | Tracker information, battery, network status |
| **Track History** | Time-series location data and analytics |
| **Error Handling** | Standard error codes and status messages |

### 1.3 Philosophy

**弘益人間 (홍익인간)** - Benefit All Humanity

Standardized data formats enable the pet tracking industry to work together, ensuring no pet is ever truly lost.

---

## 2. Coordinate System Standards

### 2.1 WGS84 Datum

ALL location data MUST use the WGS84 (World Geodetic System 1984) coordinate reference system.

**Requirements**:
- Latitude: Decimal degrees, -90.0 to +90.0
- Longitude: Decimal degrees, -180.0 to +180.0
- Precision: Minimum 6 decimal places (~0.11m resolution)
- Format: Decimal degrees (NOT degrees-minutes-seconds)

```json
{
  "latitude": 37.774929,
  "longitude": -122.419418
}
```

### 2.2 Altitude Reference

Altitude values MUST be referenced to the WGS84 ellipsoid (NOT mean sea level).

```json
{
  "altitude": 52.3,
  "altitudeAccuracy": 15.0
}
```

---

## 3. Core Location Data Structure

### 3.1 LocationUpdate Schema

```typescript
interface LocationUpdate {
  // Required Fields
  trackerId: string;           // Unique tracker device ID
  timestamp: string;           // ISO 8601 UTC timestamp
  location: Location;          // Position data
  
  // Optional Fields
  petId?: string;              // Associated pet ID
  positioning?: PositioningInfo;
  device?: DeviceStatus;
  metadata?: Metadata;
}

interface Location {
  // Required
  latitude: number;            // WGS84 decimal degrees
  longitude: number;           // WGS84 decimal degrees
  accuracy: number;            // Horizontal accuracy in meters (68% confidence)
  
  // Optional
  altitude?: number;           // Meters above WGS84 ellipsoid
  altitudeAccuracy?: number;   // Vertical accuracy in meters
  heading?: number;            // Direction of travel (0-360 degrees, 0=North)
  speed?: number;              // Ground speed in m/s
}

interface PositioningInfo {
  method: PositioningMethod;   // How location was determined
  gnss?: GNSSInfo;             // GNSS-specific data
  wifi?: WiFiPositioning;      // Wi-Fi positioning data
  cellular?: CellularPositioning;
  confidence: number;          // Confidence score 0.0-1.0
}

enum PositioningMethod {
  GNSS = "gnss",
  MULTI_GNSS = "multi-gnss",
  WIFI = "wifi",
  CELLULAR = "cellular",
  HYBRID = "hybrid",
  UNKNOWN = "unknown"
}

interface GNSSInfo {
  satellites: number;          // Number of satellites used
  systems: GNSSSystem[];       // Which satellite systems
  hdop: number;                // Horizontal dilution of precision
  vdop?: number;               // Vertical dilution of precision
  pdop?: number;               // Position dilution of precision
}

enum GNSSSystem {
  GPS = "GPS",
  GLONASS = "GLONASS",
  GALILEO = "Galileo",
  BEIDOU = "BeiDou",
  QZSS = "QZSS"
}

interface DeviceStatus {
  battery: BatteryInfo;
  network?: NetworkInfo;
  firmware?: string;
}

interface BatteryInfo {
  level: number;               // Battery percentage 0-100
  charging: boolean;           // Is device charging
  voltage?: number;            // Battery voltage
  estimatedHours?: number;     // Estimated hours remaining
}

interface NetworkInfo {
  type: string;                // "4G-LTE", "LTE-M", "NB-IoT", "LoRaWAN"
  signal: number;              // Signal strength in dBm
  carrier?: string;            // Network carrier name
}

interface Metadata {
  sequenceNumber?: number;     // Incremental update counter
  updateReason?: UpdateReason; // Why this update was sent
  environment?: "indoor" | "outdoor";
}

enum UpdateReason {
  SCHEDULED = "scheduled",
  GEOFENCE = "geofence",
  MOTION = "motion",
  MANUAL = "manual",
  ALERT = "alert"
}
```

### 3.2 Complete Example

```json
{
  "trackerId": "TRK-ABC123",
  "petId": "PET-789XYZ",
  "timestamp": "2025-12-25T10:30:45.123Z",
  "location": {
    "latitude": 37.774929,
    "longitude": -122.419418,
    "accuracy": 8.5,
    "altitude": 52.3,
    "altitudeAccuracy": 15.0,
    "heading": 275.5,
    "speed": 1.2
  },
  "positioning": {
    "method": "multi-gnss",
    "gnss": {
      "satellites": 12,
      "systems": ["GPS", "GLONASS", "Galileo"],
      "hdop": 0.9,
      "vdop": 1.2,
      "pdop": 1.5
    },
    "confidence": 0.95
  },
  "device": {
    "battery": {
      "level": 78,
      "charging": false,
      "voltage": 3.87,
      "estimatedHours": 36
    },
    "network": {
      "type": "4G-LTE",
      "signal": -68,
      "carrier": "Global Carrier"
    },
    "firmware": "2.4.1"
  },
  "metadata": {
    "sequenceNumber": 12847,
    "updateReason": "scheduled",
    "environment": "outdoor"
  }
}
```

---

## 4. Geofence Data Structures

### 4.1 Circular Geofence

```typescript
interface CircularGeofence {
  geofenceId: string;
  name: string;
  type: "circular";
  active: boolean;
  center: {
    latitude: number;
    longitude: number;
  };
  radius: number;              // Meters
  triggers: GeofenceTriggers;
  actions?: GeofenceActions;
  schedule?: GeofenceSchedule;
  metadata?: GeofenceMetadata;
}
```

Example:
```json
{
  "geofenceId": "GEO-HOME-001",
  "name": "Home",
  "type": "circular",
  "active": true,
  "center": {
    "latitude": 37.774929,
    "longitude": -122.419418
  },
  "radius": 100,
  "triggers": {
    "entry": true,
    "exit": true,
    "dwell": {
      "enabled": true,
      "duration": 300
    }
  }
}
```

### 4.2 Polygonal Geofence

```typescript
interface PolygonalGeofence {
  geofenceId: string;
  name: string;
  type: "polygonal";
  active: boolean;
  vertices: Array<{
    latitude: number;
    longitude: number;
  }>;
  triggers: GeofenceTriggers;
  actions?: GeofenceActions;
}
```

### 4.3 Geofence Event Format

```typescript
interface GeofenceEvent {
  eventId: string;
  eventType: "entry" | "exit" | "dwell";
  timestamp: string;
  trackerId: string;
  petId?: string;
  geofence: {
    id: string;
    name: string;
  };
  location: Location;
  context: {
    previousState: "inside" | "outside";
    newState: "inside" | "outside";
    crossingDirection?: "inbound" | "outbound";
    timeInsideZone?: number;  // seconds
  };
  severity: "low" | "medium" | "high" | "critical";
}
```

---

## 5. Track History Format

### 5.1 Location Track

```typescript
interface LocationTrack {
  trackId: string;
  trackerId: string;
  petId?: string;
  startTime: string;
  endTime: string;
  points: LocationPoint[];
  statistics: TrackStatistics;
}

interface LocationPoint {
  timestamp: string;
  latitude: number;
  longitude: number;
  accuracy: number;
  altitude?: number;
  speed?: number;
}

interface TrackStatistics {
  totalDistance: number;       // meters
  duration: number;             // seconds
  averageSpeed: number;         // m/s
  maxSpeed: number;             // m/s
  startLocation: Location;
  endLocation: Location;
  pointCount: number;
}
```

---

## 6. Timestamp Standards

### 6.1 ISO 8601 Format

ALL timestamps MUST use ISO 8601 format in UTC timezone:

**Format**: `YYYY-MM-DDTHH:MM:SS.sssZ`

**Requirements**:
- UTC timezone (Z suffix)
- 24-hour format
- Millisecond precision recommended
- NO local timezones

```json
{
  "timestamp": "2025-12-25T10:30:45.123Z"
}
```

---

## 7. Error and Status Codes

### 7.1 Standard Status Codes

| Code | Status | Description |
|------|--------|-------------|
| 1000 | OK | Operation successful |
| 1001 | LOCATION_UPDATED | Location successfully updated |
| 1002 | GEOFENCE_CREATED | Geofence successfully created |
| 2000 | WARNING | Non-critical warning |
| 2001 | LOW_BATTERY | Battery below threshold |
| 2002 | POOR_GPS_SIGNAL | GPS accuracy degraded |
| 3000 | ERROR | General error |
| 3001 | INVALID_DATA | Data validation failed |
| 3002 | GPS_UNAVAILABLE | No GPS signal |
| 4000 | CRITICAL | Critical system error |
| 4001 | DEVICE_OFFLINE | Device not responding |

---

## 8. Data Validation Rules

### 8.1 Location Validation

```typescript
function validateLocation(loc: Location): boolean {
  if (loc.latitude < -90 || loc.latitude > 90) return false;
  if (loc.longitude < -180 || loc.longitude > 180) return false;
  if (loc.accuracy < 0) return false;
  if (loc.heading !== undefined && (loc.heading < 0 || loc.heading >= 360)) return false;
  if (loc.speed !== undefined && loc.speed < 0) return false;
  return true;
}
```

### 8.2 Timestamp Validation

```typescript
function validateTimestamp(ts: string): boolean {
  const iso8601Regex = /^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(\.\d{3})?Z$/;
  if (!iso8601Regex.test(ts)) return false;
  
  const date = new Date(ts);
  if (isNaN(date.getTime())) return false;
  
  // Timestamp must not be in the future (allow 1 minute clock skew)
  if (date.getTime() > Date.now() + 60000) return false;
  
  return true;
}
```

---

## 9. Conformance Requirements

### 9.1 Mandatory Fields

Implementations MUST support all required fields in LocationUpdate.

### 9.2 Optional Field Handling

Implementations MUST gracefully handle optional fields when not present.

### 9.3 Backward Compatibility

New versions MUST maintain backward compatibility with existing data structures.

---

**弘益人間 · Benefit All Humanity**  
© 2025 WIA - World Certification Industry Association | MIT License
