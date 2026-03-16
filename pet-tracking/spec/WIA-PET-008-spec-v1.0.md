# WIA-PET-008 Pet Tracking Standard - Specification v1.0

**Status:** Published  
**Version:** 1.0.0  
**Date:** 2025-12-25  
**Authors:** WIA Technical Committee on Pet Safety  

## 1. Introduction

### 1.1 Purpose
The WIA-PET-008 specification defines a comprehensive standard for pet tracking systems using GPS/GNSS technology, ensuring interoperability, performance, and security across devices, applications, and services.

### 1.2 Scope
This standard covers:
- Location data formats and schemas
- GPS/GNSS positioning requirements
- Geofencing algorithms and alert systems
- Battery optimization and power management
- Communication protocols and APIs
- Security and privacy requirements
- Integration with third-party systems

### 1.3 Philosophy
Built on the principle of 弘益人間 (Benefit All Humanity), this standard prioritizes:
- Pet safety and well-being
- Owner peace of mind
- Accessibility for all users
- Environmental responsibility
- Data privacy and security

## 2. Location Data Format

### 2.1 Coordinate System
- **Standard:** WGS84 (World Geodetic System 1984)
- **Latitude:** -90.0 to 90.0 decimal degrees
- **Longitude:** -180.0 to 180.0 decimal degrees
- **Altitude:** Meters above sea level
- **Precision:** Minimum 6 decimal places (±0.11m accuracy)

### 2.2 Location Event Schema
```json
{
  "location": {
    "latitude": 37.7749295,
    "longitude": -122.4194155,
    "altitude": 52.3,
    "accuracy": 5.2,
    "heading": 245.5,
    "speed": 1.2
  },
  "timestamp": "2025-12-25T14:30:00.000Z",
  "deviceId": "PET-TRACK-12345",
  "petId": "pet_abc123",
  "quality": {
    "method": "GNSS",
    "satellites": 12,
    "hdop": 1.2,
    "constellations": ["GPS", "GLONASS"]
  }
}
```

## 3. GNSS Requirements

### 3.1 Multi-Constellation Support
**Required:** GPS + at least one additional constellation  
**Recommended:** GPS + GLONASS + Galileo + BeiDou  

### 3.2 Accuracy Standards
- **Horizontal Accuracy:** ≤ 5m (95% confidence)
- **Vertical Accuracy:** ≤ 10m (95% confidence)
- **Time-to-First-Fix:** ≤ 30 seconds (cold start with A-GPS)
- **Update Rate:** Minimum 1Hz (1 position per second) when active

### 3.3 Assisted GPS (A-GPS)
- **Requirement:** Mandatory for certified devices
- **Assistance Data:** Ephemeris (4-hour validity), Almanac (7-day validity)
- **Delivery:** Via cellular data or Wi-Fi
- **Performance:** TTFF ≤ 10 seconds with valid assistance data

## 4. Geofencing

### 4.1 Geofence Types
- **Circular:** Center point + radius
- **Polygon:** Array of coordinate points
- **Corridor:** Path + width tolerance

### 4.2 Detection Requirements
- **Accuracy:** ±10 meters at geofence boundary
- **Hysteresis:** Minimum 10-meter buffer zone
- **Dwell Time:** Configurable 0-300 seconds
- **Alert Latency:** ≤ 30 seconds from boundary crossing

### 4.3 Alert Channels
- Push Notification (required)
- SMS (recommended)
- Email (recommended)
- Phone Call (optional for critical alerts)

## 5. Battery and Power Management

### 5.1 Battery Life Targets
- **Minimum:** 7 days normal use
- **Target:** 30 days with optimization
- **Emergency Mode:** 48 hours at critical battery level

### 5.2 Power States
- **Active Tracking:** GPS ON, updates every 10-60 seconds
- **Normal Mode:** GPS periodic (60-300 seconds)
- **Sleep Mode:** GPS OFF, accelerometer wake-up
- **Emergency:** Position broadcast every 15 minutes

### 5.3 Battery Reporting
- **Frequency:** Included in every position update
- **Format:** Percentage (0-100%)
- **Alerts:** Low battery ≤ 20%, Critical ≤ 10%

## 6. Communication Protocols

### 6.1 Supported Protocols
- **Primary:** HTTPS/REST API
- **Real-time:** WebSocket, MQTT
- **Constrained Devices:** CoAP

### 6.2 Data Format
- **Encoding:** JSON (required), Protocol Buffers (optional)
- **Compression:** Gzip for HTTP, optional for real-time protocols
- **Batching:** Support for multiple location points per transmission

### 6.3 Quality of Service
- **Delivery Guarantee:** At-least-once
- **Retry Policy:** Exponential backoff, max 5 attempts
- **Offline Buffering:** Minimum 24 hours of data

## 7. Security Requirements

### 7.1 Encryption
- **Transport:** TLS 1.3 minimum
- **Data at Rest:** AES-256
- **End-to-End:** Required for location data

### 7.2 Authentication
- **Device:** Certificate-based or JWT tokens
- **User:** OAuth 2.0 / OpenID Connect
- **Token Expiry:** Maximum 24 hours

### 7.3 Privacy
- **Data Retention:** Configurable 30-365 days
- **User Rights:** GDPR compliance (access, delete, portability)
- **Location Sharing:** Explicit consent required

## 8. Performance Standards

### 8.1 API Response Times
- **Location Query:** ≤ 200ms (p95)
- **History Query:** ≤ 500ms (p95)
- **Alert Delivery:** ≤ 30 seconds end-to-end

### 8.2 Scalability
- **Concurrent Devices:** Support 10,000+ devices per instance
- **Update Rate:** Handle 100+ updates/second
- **Data Retention:** Minimum 90 days location history

## 9. Compliance and Certification

### 9.1 Certification Levels
- **Level 1:** Basic location tracking
- **Level 2:** Advanced features (geofencing, alerts)
- **Level 3:** Full compliance with all requirements

### 9.2 Testing Requirements
- Accuracy testing in urban, suburban, rural environments
- Battery life verification under standard use profile
- Security audit and penetration testing
- Interoperability testing with reference implementation

---

**© 2025 World Certification Industry Association (WIA)**  
弘益人間 (홍익인간) · Benefit All Humanity
