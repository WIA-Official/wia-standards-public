# WIA Autonomous Vehicle Accessibility Standard
## Phase 3: Communication Protocol Specification

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-01-15

---

## 1. Overview

This specification defines the communication protocols for the WIA Autonomous Vehicle Accessibility Standard. It establishes how different components (passenger apps, fleet management, vehicles, support centers) communicate to provide accessible transportation.

### 1.1 Design Principles

1. **Accessibility First**: All protocols must support accessibility features
2. **Real-time**: Low-latency communication for safety-critical operations
3. **Security**: End-to-end encryption and authentication
4. **Reliability**: Graceful degradation and recovery mechanisms
5. **Interoperability**: Open standards and well-documented APIs

### 1.2 System Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                        Cloud Infrastructure                        │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐   │
│  │  Fleet Server   │  │  Profile Store  │  │  Trip Manager   │   │
│  └────────┬────────┘  └────────┬────────┘  └────────┬────────┘   │
│           │                    │                    │              │
│           └────────────────────┼────────────────────┘              │
│                                │                                   │
│                         ┌──────▼──────┐                           │
│                         │   API GW    │                           │
│                         └──────┬──────┘                           │
└────────────────────────────────┼───────────────────────────────────┘
                                 │
            ┌────────────────────┼────────────────────┐
            │                    │                    │
     ┌──────▼──────┐     ┌──────▼──────┐     ┌──────▼──────┐
     │  Passenger  │     │   Vehicle   │     │   Support   │
     │     App     │     │   System    │     │   Center    │
     └─────────────┘     └─────────────┘     └─────────────┘
```

---

## 2. Transport Protocols

### 2.1 REST API (HTTPS)

Used for: CRUD operations, configuration, non-real-time queries

**Base URL**: `https://api.wia-auto.org/v1`

#### Authentication
- OAuth 2.0 with PKCE for mobile apps
- API keys for server-to-server communication
- JWT tokens with 15-minute expiration

#### Headers
```http
Authorization: Bearer <jwt_token>
Content-Type: application/json
Accept: application/json
X-WIA-Version: 1.0.0
X-Request-ID: <uuid>
X-Accessibility-Preferences: <encoded_preferences>
```

### 2.2 WebSocket (WSS)

Used for: Real-time updates, bidirectional communication

**Endpoint**: `wss://ws.wia-auto.org/v1/stream`

#### Connection Parameters
```
?token=<jwt_token>
&client_type=<passenger|vehicle|support>
&client_id=<uuid>
```

#### Message Format
```json
{
  "type": "event_type",
  "id": "message_uuid",
  "timestamp": "2025-01-15T10:30:00Z",
  "payload": { ... },
  "correlation_id": "original_request_uuid"
}
```

#### Heartbeat
- Client sends `ping` every 30 seconds
- Server responds with `pong`
- Disconnect after 3 missed heartbeats

### 2.3 gRPC (HTTP/2)

Used for: Vehicle control, low-latency operations, streaming

**Endpoint**: `grpc.wia-auto.org:443`

#### TLS Requirements
- TLS 1.3 required
- Client certificate authentication for vehicles
- Server certificate pinning recommended

---

## 3. REST API Specification

### 3.1 Passenger Profile Management

#### Create Profile
```http
POST /profiles
Content-Type: application/json

{
  "passenger": {
    "name": "Kim Minji",
    "disabilities": ["mobility_wheelchair_power"],
    "service_animal": {
      "has_animal": true,
      "animal_type": "guide_dog",
      "animal_size": "large"
    }
  },
  "mobility_aid": {
    "type": "power_wheelchair",
    "dimensions": {
      "width_cm": 68,
      "length_cm": 120,
      "height_cm": 100,
      "weight_kg": 150
    },
    "requires_ramp": true,
    "requires_securement": true
  },
  "communication": {
    "preferred_language": "ko",
    "preferred_modalities": ["audio_tts", "haptic_vibration"]
  }
}
```

**Response**: `201 Created`
```json
{
  "profile_id": "550e8400-e29b-41d4-a716-446655440000",
  "version": "1.0.0",
  "created_at": "2025-01-15T10:30:00Z"
}
```

#### Get Profile
```http
GET /profiles/{profile_id}
```

#### Update Profile
```http
PUT /profiles/{profile_id}
```

#### Delete Profile
```http
DELETE /profiles/{profile_id}
```

### 3.2 Trip Management

#### Request Trip
```http
POST /trips
Content-Type: application/json

{
  "passenger_profile_id": "550e8400-e29b-41d4-a716-446655440000",
  "trip": {
    "pickup": {
      "location": {
        "latitude": 37.5665,
        "longitude": 126.9780,
        "address": "Seoul City Hall"
      },
      "notes": "Wheelchair boarding area",
      "pickup_side": "same_side",
      "curb_to_curb": true
    },
    "dropoff": {
      "location": {
        "latitude": 37.5796,
        "longitude": 126.9770,
        "address": "Gyeongbokgung Palace"
      }
    },
    "scheduled_time": "2025-01-15T14:00:00Z",
    "flexibility_minutes": 15
  },
  "accessibility_requirements": {
    "wheelchair_accessible": true,
    "ramp_required": true,
    "service_animal_space": true
  },
  "preferences": {
    "minimize_walking": true,
    "audio_guidance": true,
    "quiet_ride": true
  }
}
```

**Response**: `202 Accepted`
```json
{
  "response_id": "660e8400-e29b-41d4-a716-446655440001",
  "request_id": "770e8400-e29b-41d4-a716-446655440002",
  "status": "searching",
  "estimated_wait_minutes": 5
}
```

#### Get Trip Status
```http
GET /trips/{request_id}
```

**Response**: `200 OK`
```json
{
  "response_id": "660e8400-e29b-41d4-a716-446655440001",
  "request_id": "770e8400-e29b-41d4-a716-446655440002",
  "status": "confirmed",
  "vehicle": {
    "vehicle_id": "880e8400-e29b-41d4-a716-446655440003",
    "eta_minutes": 8,
    "distance_km": 2.5
  },
  "accessibility_match": {
    "wheelchair_accessible": true,
    "ramp_available": true,
    "service_animal_ok": true,
    "match_score": 95
  },
  "wayfinding": {
    "pickup_instructions": "Vehicle will arrive at the wheelchair boarding zone.",
    "audio_guidance_available": true,
    "find_vehicle_features": ["horn", "lights", "melody"],
    "braille_identifier": "WIA-001"
  }
}
```

#### Cancel Trip
```http
DELETE /trips/{request_id}
```

### 3.3 Vehicle Management

#### List Available Vehicles
```http
GET /vehicles?lat=37.5665&lng=126.9780&radius_km=5&wheelchair=true
```

#### Get Vehicle Details
```http
GET /vehicles/{vehicle_id}
```

### 3.4 Emergency Reporting

#### Report Emergency
```http
POST /emergency
Content-Type: application/json

{
  "vehicle_id": "880e8400-e29b-41d4-a716-446655440003",
  "trip_id": "770e8400-e29b-41d4-a716-446655440002",
  "event_type": "panic_button",
  "severity": "high",
  "location": {
    "latitude": 37.5730,
    "longitude": 126.9775,
    "address": "Sejong-ro, Seoul"
  },
  "passenger": {
    "profile_id": "550e8400-e29b-41d4-a716-446655440000",
    "disabilities": ["mobility_wheelchair_power"]
  }
}
```

**Response**: `201 Created`
```json
{
  "event_id": "990e8400-e29b-41d4-a716-446655440004",
  "response": {
    "auto_pulled_over": true,
    "support_contacted": true,
    "emergency_services_called": false,
    "eta_support_minutes": 3
  }
}
```

---

## 4. WebSocket Events

### 4.1 Event Types

| Event | Direction | Description |
|-------|-----------|-------------|
| `trip.status.updated` | Server→Client | Trip status changed |
| `vehicle.location.updated` | Server→Client | Vehicle location update |
| `vehicle.eta.updated` | Server→Client | ETA changed |
| `securement.status.changed` | Server→Client | Wheelchair securement status |
| `emergency.alert` | Server→Client | Emergency notification |
| `hmi.command` | Client→Server | HMI command request |
| `hmi.response` | Server→Client | HMI command response |
| `support.message` | Bidirectional | Support chat message |

### 4.2 Event Payloads

#### trip.status.updated
```json
{
  "type": "trip.status.updated",
  "id": "msg-001",
  "timestamp": "2025-01-15T10:35:00Z",
  "payload": {
    "trip_id": "770e8400-e29b-41d4-a716-446655440002",
    "previous_status": "searching",
    "new_status": "confirmed",
    "vehicle_id": "880e8400-e29b-41d4-a716-446655440003",
    "eta_minutes": 8
  }
}
```

#### vehicle.location.updated
```json
{
  "type": "vehicle.location.updated",
  "id": "msg-002",
  "timestamp": "2025-01-15T10:35:30Z",
  "payload": {
    "vehicle_id": "880e8400-e29b-41d4-a716-446655440003",
    "location": {
      "latitude": 37.5680,
      "longitude": 126.9785
    },
    "heading_degrees": 45,
    "speed_kmh": 30,
    "eta_minutes": 6
  }
}
```

#### securement.status.changed
```json
{
  "type": "securement.status.changed",
  "id": "msg-003",
  "timestamp": "2025-01-15T10:45:00Z",
  "payload": {
    "vehicle_id": "880e8400-e29b-41d4-a716-446655440003",
    "trip_id": "770e8400-e29b-41d4-a716-446655440002",
    "overall_status": "secured",
    "wheelchair_detected": true,
    "securement_points": [
      {"position": "front_left", "status": "engaged", "locked": true},
      {"position": "front_right", "status": "engaged", "locked": true},
      {"position": "rear_left", "status": "engaged", "locked": true},
      {"position": "rear_right", "status": "engaged", "locked": true}
    ],
    "safety_check_passed": true,
    "ready_to_move": true
  }
}
```

#### emergency.alert
```json
{
  "type": "emergency.alert",
  "id": "msg-004",
  "timestamp": "2025-01-15T10:50:00Z",
  "payload": {
    "event_id": "990e8400-e29b-41d4-a716-446655440004",
    "event_type": "panic_button",
    "severity": "high",
    "vehicle_id": "880e8400-e29b-41d4-a716-446655440003",
    "location": {
      "latitude": 37.5730,
      "longitude": 126.9775
    },
    "response": {
      "auto_pulled_over": true,
      "support_contacted": true,
      "eta_support_minutes": 3
    },
    "actions_required": ["acknowledge", "contact_passenger"]
  }
}
```

### 4.3 Client Commands

#### Subscribe to Events
```json
{
  "type": "subscribe",
  "id": "cmd-001",
  "payload": {
    "events": ["trip.status.updated", "vehicle.location.updated"],
    "trip_id": "770e8400-e29b-41d4-a716-446655440002"
  }
}
```

#### Unsubscribe
```json
{
  "type": "unsubscribe",
  "id": "cmd-002",
  "payload": {
    "events": ["vehicle.location.updated"]
  }
}
```

#### Send HMI Command
```json
{
  "type": "hmi.command",
  "id": "cmd-003",
  "payload": {
    "vehicle_id": "880e8400-e29b-41d4-a716-446655440003",
    "command": "announce",
    "parameters": {
      "message": "Arriving at destination in 2 minutes",
      "modalities": ["audio_tts", "haptic_vibration"]
    }
  }
}
```

---

## 5. gRPC Service Definition

### 5.1 Vehicle Control Service

```protobuf
syntax = "proto3";

package wia.auto.v1;

import "google/protobuf/timestamp.proto";
import "google/protobuf/empty.proto";

// Vehicle control service for real-time operations
service VehicleControl {
  // Stream vehicle location updates
  rpc StreamLocation(StreamLocationRequest) returns (stream Location);

  // Stream securement status updates
  rpc StreamSecurement(StreamSecurementRequest) returns (stream SecurementStatus);

  // Send HMI command to vehicle
  rpc SendHmiCommand(HmiCommand) returns (HmiResponse);

  // Trigger emergency stop
  rpc EmergencyStop(EmergencyStopRequest) returns (EmergencyStopResponse);

  // Configure HMI settings
  rpc ConfigureHmi(HmiConfig) returns (HmiConfigResponse);

  // Start wheelchair securement
  rpc StartSecurement(SecurementRequest) returns (SecurementResponse);

  // Release wheelchair securement
  rpc ReleaseSecurement(SecurementRequest) returns (SecurementResponse);
}

message StreamLocationRequest {
  string vehicle_id = 1;
  int32 update_interval_ms = 2;  // Default: 1000ms
}

message Location {
  double latitude = 1;
  double longitude = 2;
  float heading_degrees = 3;
  float speed_kmh = 4;
  google.protobuf.Timestamp timestamp = 5;
}

message StreamSecurementRequest {
  string vehicle_id = 1;
}

message SecurementStatus {
  string vehicle_id = 1;
  OverallStatus overall_status = 2;
  bool wheelchair_detected = 3;
  repeated SecurementPoint points = 4;
  bool safety_check_passed = 5;
  bool ready_to_move = 6;
  google.protobuf.Timestamp timestamp = 7;

  enum OverallStatus {
    UNSECURED = 0;
    SECURING = 1;
    SECURED = 2;
    ERROR = 3;
  }
}

message SecurementPoint {
  string point_id = 1;
  Position position = 2;
  PointStatus status = 3;
  float force_newtons = 4;
  bool locked = 5;

  enum Position {
    FRONT_LEFT = 0;
    FRONT_RIGHT = 1;
    REAR_LEFT = 2;
    REAR_RIGHT = 3;
  }

  enum PointStatus {
    DISENGAGED = 0;
    ENGAGING = 1;
    ENGAGED = 2;
    ERROR = 3;
  }
}

message HmiCommand {
  string vehicle_id = 1;
  CommandType command = 2;
  map<string, string> parameters = 3;

  enum CommandType {
    ANNOUNCE = 0;
    DISPLAY = 1;
    HAPTIC = 2;
    CHIME = 3;
    FIND_VEHICLE = 4;
  }
}

message HmiResponse {
  bool success = 1;
  string message = 2;
}

message HmiConfig {
  string vehicle_id = 1;
  VisualConfig visual = 2;
  AudioConfig audio = 3;
  HapticConfig haptic = 4;
}

message VisualConfig {
  bool enabled = 1;
  int32 brightness = 2;
  ContrastMode contrast = 3;
  TextSize text_size = 4;

  enum ContrastMode {
    NORMAL = 0;
    HIGH = 1;
  }

  enum TextSize {
    SMALL = 0;
    MEDIUM = 1;
    LARGE = 2;
    EXTRA_LARGE = 3;
  }
}

message AudioConfig {
  bool enabled = 1;
  int32 volume = 2;
  bool tts_enabled = 3;
  TtsSpeed tts_speed = 4;
  string language = 5;

  enum TtsSpeed {
    SLOW = 0;
    NORMAL = 1;
    FAST = 2;
  }
}

message HapticConfig {
  bool enabled = 1;
  int32 intensity = 2;
}

message HmiConfigResponse {
  bool success = 1;
  string config_id = 2;
}

message EmergencyStopRequest {
  string vehicle_id = 1;
  string trip_id = 2;
  EmergencyReason reason = 3;

  enum EmergencyReason {
    PANIC_BUTTON = 0;
    SECUREMENT_FAILURE = 1;
    OBSTACLE_DETECTED = 2;
    MEDICAL = 3;
    OTHER = 4;
  }
}

message EmergencyStopResponse {
  bool success = 1;
  bool vehicle_stopped = 2;
  bool support_notified = 3;
  int32 eta_support_minutes = 4;
}

message SecurementRequest {
  string vehicle_id = 1;
  string trip_id = 2;
}

message SecurementResponse {
  bool success = 1;
  SecurementStatus status = 2;
}
```

### 5.2 Fleet Management Service

```protobuf
// Fleet management service
service FleetManagement {
  // Find available vehicles matching requirements
  rpc FindVehicles(FindVehiclesRequest) returns (FindVehiclesResponse);

  // Dispatch vehicle to pickup
  rpc DispatchVehicle(DispatchRequest) returns (DispatchResponse);

  // Get vehicle status
  rpc GetVehicleStatus(VehicleStatusRequest) returns (VehicleStatus);

  // Stream fleet status updates
  rpc StreamFleetStatus(google.protobuf.Empty) returns (stream FleetStatusUpdate);
}

message FindVehiclesRequest {
  Location location = 1;
  float radius_km = 2;
  AccessibilityRequirements requirements = 3;
}

message AccessibilityRequirements {
  bool wheelchair_accessible = 1;
  bool ramp_required = 2;
  bool lift_required = 3;
  bool service_animal_space = 4;
  int32 companion_space = 5;
  repeated string preferred_modalities = 6;
}

message FindVehiclesResponse {
  repeated VehicleMatch vehicles = 1;
}

message VehicleMatch {
  string vehicle_id = 1;
  VehicleInfo info = 2;
  Location location = 3;
  float distance_km = 4;
  float eta_minutes = 5;
  int32 match_score = 6;
  AccessibilityMatch accessibility = 7;
}

message VehicleInfo {
  string make = 1;
  string model = 2;
  int32 year = 3;
  string license_plate = 4;
  SaeLevel sae_level = 5;

  enum SaeLevel {
    LEVEL_0 = 0;
    LEVEL_1 = 1;
    LEVEL_2 = 2;
    LEVEL_3 = 3;
    LEVEL_4 = 4;
    LEVEL_5 = 5;
  }
}

message AccessibilityMatch {
  bool wheelchair_accessible = 1;
  bool ramp_available = 2;
  bool lift_available = 3;
  bool service_animal_ok = 4;
  repeated string modalities_supported = 5;
  int32 match_score = 6;
}

message DispatchRequest {
  string vehicle_id = 1;
  string trip_id = 2;
  Location pickup = 3;
  Location dropoff = 4;
}

message DispatchResponse {
  bool success = 1;
  string assignment_id = 2;
  float eta_minutes = 3;
}

message VehicleStatusRequest {
  string vehicle_id = 1;
}

message VehicleStatus {
  string vehicle_id = 1;
  VehicleState state = 2;
  Location location = 3;
  string current_trip_id = 4;

  enum VehicleState {
    OFFLINE = 0;
    AVAILABLE = 1;
    DISPATCHED = 2;
    EN_ROUTE = 3;
    ARRIVED = 4;
    IN_SERVICE = 5;
    RETURNING = 6;
  }
}

message FleetStatusUpdate {
  string vehicle_id = 1;
  VehicleStatus status = 2;
  google.protobuf.Timestamp timestamp = 3;
}
```

---

## 6. Security Specification

### 6.1 Transport Security

- **TLS Version**: 1.3 required, 1.2 minimum
- **Cipher Suites**: TLS_AES_256_GCM_SHA384, TLS_CHACHA20_POLY1305_SHA256
- **Certificate**: X.509 with RSA-2048 or ECDSA P-256
- **Certificate Pinning**: Required for mobile apps

### 6.2 Authentication

#### OAuth 2.0 Flow (Passenger Apps)
```
1. Authorization Code + PKCE
2. Token endpoint: POST /oauth/token
3. Access token: JWT, 15 min expiry
4. Refresh token: Opaque, 7 day expiry
```

#### API Key (Server-to-Server)
```
X-API-Key: <api_key>
X-API-Secret: <hmac_signature>
X-Timestamp: <unix_timestamp>
```

#### Vehicle Authentication
```
1. Client certificate (mTLS)
2. Vehicle ID in certificate CN
3. Hardware security module (HSM) for key storage
```

### 6.3 Message Signing

All sensitive messages signed with HMAC-SHA256:

```
Signature = HMAC-SHA256(
  key = shared_secret,
  message = timestamp + method + path + body_hash
)

X-WIA-Signature: <base64_signature>
X-WIA-Timestamp: <unix_timestamp>
```

### 6.4 Data Encryption

#### Personal Data (PII)
- Algorithm: AES-256-GCM
- Key derivation: HKDF-SHA256
- Key rotation: Every 30 days

#### Fields Encrypted at Rest
- Passenger name
- Emergency contact information
- Location history
- Disability details

### 6.5 Rate Limiting

| Endpoint Type | Rate Limit |
|---------------|------------|
| Public | 100 req/min |
| Authenticated | 1000 req/min |
| Vehicle | 5000 req/min |
| WebSocket | 100 msg/sec |

### 6.6 Audit Logging

All API calls logged with:
- Timestamp
- Request ID
- User/Vehicle ID
- Endpoint
- Response status
- Latency

Logs retained for 90 days minimum.

---

## 7. Error Handling

### 7.1 HTTP Status Codes

| Code | Meaning |
|------|---------|
| 200 | Success |
| 201 | Created |
| 202 | Accepted (async) |
| 400 | Bad Request |
| 401 | Unauthorized |
| 403 | Forbidden |
| 404 | Not Found |
| 409 | Conflict |
| 422 | Validation Error |
| 429 | Rate Limited |
| 500 | Server Error |
| 503 | Service Unavailable |

### 7.2 Error Response Format

```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid accessibility requirements",
    "details": [
      {
        "field": "wheelchair_accessible",
        "issue": "Cannot be false when ramp_required is true"
      }
    ],
    "request_id": "req-123",
    "documentation_url": "https://docs.wia-auto.org/errors/VALIDATION_ERROR"
  }
}
```

### 7.3 Error Codes

| Code | Description |
|------|-------------|
| `AUTH_INVALID_TOKEN` | Invalid or expired token |
| `AUTH_INSUFFICIENT_SCOPE` | Missing required permissions |
| `VALIDATION_ERROR` | Request validation failed |
| `TRIP_NOT_FOUND` | Trip does not exist |
| `TRIP_ALREADY_CANCELLED` | Trip was already cancelled |
| `VEHICLE_NOT_AVAILABLE` | No matching vehicles |
| `VEHICLE_NOT_FOUND` | Vehicle does not exist |
| `PROFILE_NOT_FOUND` | Profile does not exist |
| `RATE_LIMIT_EXCEEDED` | Too many requests |
| `SERVICE_UNAVAILABLE` | Service temporarily down |

---

## 8. Accessibility Considerations

### 8.1 Response Times

- Trip status updates: < 1 second
- Location updates: < 500ms
- Emergency alerts: < 100ms
- HMI commands: < 200ms

### 8.2 Alternative Formats

All text content available in:
- Plain text
- SSML for TTS
- Braille-ready format

### 8.3 Localization

- All messages localizable
- Support for: en, ko, ja, zh, es, ar
- RTL language support

### 8.4 Fallback Mechanisms

1. WebSocket → Long polling
2. gRPC → REST
3. TTS → Text display
4. Visual → Audio description

---

## 9. Implementation Notes

### 9.1 Client SDK Requirements

- Automatic retry with exponential backoff
- Connection pooling
- Offline queue for critical operations
- Automatic reconnection for WebSocket/gRPC

### 9.2 Testing

- Mock server provided at `https://mock.wia-auto.org`
- Test credentials available for sandbox
- Load testing endpoints available

---

## Appendix A: Message Envelope

All messages wrapped in standard envelope:

```json
{
  "wia_auto": {
    "version": "1.0.0",
    "message_id": "uuid",
    "timestamp": "ISO8601",
    "source": "passenger_app|vehicle|fleet_mgmt|support",
    "destination": "passenger_app|vehicle|fleet_mgmt|support",
    "message_type": "profile|capabilities|trip_request|trip_response|hmi_config|securement|emergency",
    "correlation_id": "uuid",
    "priority": "low|normal|high|critical",
    "payload": { ... }
  }
}
```

---

弘益人間 - Benefit All Humanity
