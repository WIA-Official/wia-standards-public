# WIA-UNI-008 - Phase 1: Data Format

**Version:** 1.0.0
**Status:** Active
**Last Updated:** 2025-12-25

## 1. Overview

Phase 1 of the WIA-UNI-008 Transportation Network Standard defines comprehensive data formats for representing transportation routes, schedules, bookings, cargo tracking, and logistics operations across the Korean Peninsula. Using JSON-LD for semantic interoperability, these formats enable consistent data exchange between diverse transportation systems.

### 1.1 Objectives

- Establish common data schemas for all transportation modes
- Enable semantic interoperability through linked data
- Support real-time tracking and status updates
- Provide extensibility for multi-modal transportation
- Ensure data quality through validation rules

### 1.2 Scope

This specification covers:

- Transportation route metadata
- Schedule and timetable formats
- Booking and ticketing data
- Cargo and freight tracking
- Vehicle/vessel registration
- Real-time location and status data

## 2. Core Data Schema

### 2.1 Transportation Route Format

```json
{
  "@context": "https://wiastandards.com/contexts/uni-008/v1",
  "@type": "TransportationRoute",
  "id": "wia:uni:route:{unique-id}",
  "routeType": "high-speed-rail|conventional-rail|highway|air-route|maritime",
  "name": "string",
  "description": "string",
  "operator": {
    "name": "string",
    "operatorCode": "string",
    "country": "ISO 3166-1 alpha-2 code",
    "license": "string (optional)"
  },
  "origin": {
    "name": "string",
    "code": "IATA/station code",
    "location": {
      "latitude": "number",
      "longitude": "number",
      "elevation": "number (optional)"
    }
  },
  "destination": {
    "name": "string",
    "code": "IATA/station code",
    "location": {
      "latitude": "number",
      "longitude": "number",
      "elevation": "number (optional)"
    }
  },
  "waypoints": [
    {
      "name": "string",
      "code": "string",
      "location": { "latitude": "number", "longitude": "number" },
      "stopDuration": "number (minutes)"
    }
  ],
  "distance": {
    "value": "number",
    "unit": "km|miles"
  },
  "estimatedDuration": {
    "value": "number",
    "unit": "hours|minutes"
  },
  "schedule": {
    "frequency": "daily|weekly|on-demand",
    "departureTimes": ["array of ISO 8601 times"],
    "validFrom": "ISO 8601 date",
    "validUntil": "ISO 8601 date (optional)"
  },
  "specifications": {
    "maxSpeed": "number (km/h)",
    "capacity": {
      "passengers": "number",
      "cargo": "number (tons)"
    },
    "gauge": "number (mm, for rail)",
    "electrification": "string (for rail)",
    "signaling": "string (for rail)"
  },
  "standards": ["array of applicable standards"],
  "certificationStatus": "pending|in-progress|certified|expired",
  "metadata": {
    "createdBy": "string",
    "createdAt": "ISO 8601 datetime",
    "modifiedBy": "string",
    "modifiedAt": "ISO 8601 datetime",
    "version": "string"
  }
}
```

### 2.2 Booking & Ticketing Format

```json
{
  "@context": "https://wiastandards.com/contexts/uni-008/v1",
  "@type": "TransportationBooking",
  "id": "wia:uni:booking:{unique-id}",
  "bookingReference": "string",
  "status": "pending|confirmed|cancelled|completed",
  "bookingDate": "ISO 8601 datetime",
  "passenger": {
    "name": "string",
    "passportNumber": "string",
    "nationality": "ISO 3166-1 alpha-2",
    "dateOfBirth": "ISO 8601 date",
    "contactInfo": {
      "email": "string",
      "phone": "string"
    }
  },
  "journey": {
    "routeId": "string (reference to route)",
    "origin": "station/airport code",
    "destination": "station/airport code",
    "departureDate": "ISO 8601 date",
    "departureTime": "ISO 8601 time",
    "arrivalTime": "ISO 8601 time (estimated)",
    "class": "economy|business|first",
    "seatNumber": "string (optional)"
  },
  "pricing": {
    "basePrice": "number",
    "taxes": "number",
    "fees": "number",
    "totalAmount": "number",
    "currency": "ISO 4217 currency code",
    "discounts": ["array of discount objects"]
  },
  "payment": {
    "method": "credit-card|bank-transfer|digital-wallet",
    "status": "pending|completed|refunded",
    "transactionId": "string"
  },
  "additionalServices": [
    {
      "type": "meal|baggage|lounge-access",
      "description": "string",
      "price": "number"
    }
  ],
  "specialRequirements": ["array of strings"],
  "verifiableCredential": {
    "id": "string (VC identifier)",
    "issued": "boolean"
  }
}
```

### 2.3 Vehicle/Vessel Registration Format

```json
{
  "@context": "https://wiastandards.com/contexts/uni-008/v1",
  "@type": "TransportVehicle",
  "id": "wia:uni:vehicle:{unique-id}",
  "vehicleType": "train|bus|truck|aircraft|ship",
  "registrationNumber": "string",
  "operator": {
    "name": "string",
    "operatorCode": "string"
  },
  "specifications": {
    "manufacturer": "string",
    "model": "string",
    "yearBuilt": "number",
    "capacity": {
      "passengers": "number",
      "cargo": "number (tons)"
    },
    "fuelType": "electric|diesel|hybrid|hydrogen",
    "maxSpeed": "number (km/h)"
  },
  "certifications": [
    {
      "type": "safety|emissions|operations",
      "issuedBy": "string",
      "validUntil": "ISO 8601 date"
    }
  ],
  "maintenance": {
    "lastService": "ISO 8601 date",
    "nextService": "ISO 8601 date",
    "serviceHistory": ["array of service records"]
  },
  "currentStatus": "active|maintenance|retired",
  "iotDevices": [
    {
      "deviceId": "string",
      "type": "gps|sensor|camera",
      "status": "active|inactive"
    }
  ]
}
```

### 2.4 Cargo Tracking Format

```json
{
  "@context": "https://wiastandards.com/contexts/uni-008/v1",
  "@type": "CargoShipment",
  "id": "wia:uni:cargo:{unique-id}",
  "trackingNumber": "string",
  "status": "booked|in-transit|customs|delivered",
  "shipper": {
    "name": "string",
    "address": "object",
    "contact": "object"
  },
  "consignee": {
    "name": "string",
    "address": "object",
    "contact": "object"
  },
  "cargo": {
    "description": "string",
    "type": "container|bulk|perishable|hazardous",
    "weight": {
      "value": "number",
      "unit": "kg|tons"
    },
    "volume": {
      "value": "number",
      "unit": "m3"
    },
    "containerNumber": "string (optional)",
    "hsCode": "string (Harmonized System code)"
  },
  "route": {
    "origin": "string",
    "destination": "string",
    "currentLocation": {
      "name": "string",
      "coordinates": { "latitude": "number", "longitude": "number" },
      "timestamp": "ISO 8601 datetime"
    },
    "segments": [
      {
        "mode": "rail|road|air|sea",
        "from": "string",
        "to": "string",
        "vehicleId": "string",
        "estimatedDeparture": "ISO 8601 datetime",
        "estimatedArrival": "ISO 8601 datetime"
      }
    ]
  },
  "customs": {
    "status": "pending|cleared|inspection",
    "declarationNumber": "string",
    "estimatedClearance": "ISO 8601 datetime"
  },
  "conditions": {
    "temperature": {
      "current": "number (°C)",
      "min": "number",
      "max": "number"
    },
    "humidity": "number (%)",
    "alerts": ["array of alert strings"]
  }
}
```

### 2.5 Real-time Tracking Format

```json
{
  "@context": "https://wiastandards.com/contexts/uni-008/v1",
  "@type": "VehicleTracking",
  "vehicleId": "string",
  "trackingId": "string",
  "timestamp": "ISO 8601 datetime",
  "location": {
    "latitude": "number",
    "longitude": "number",
    "altitude": "number (meters, optional)",
    "heading": "number (degrees, 0-360)",
    "accuracy": "number (meters)"
  },
  "speed": {
    "value": "number",
    "unit": "km/h"
  },
  "status": "in-transit|stopped|delayed|emergency",
  "nextStop": {
    "name": "string",
    "code": "string",
    "estimatedArrival": "ISO 8601 datetime",
    "distance": "number (km)"
  },
  "occupancy": {
    "passengerCount": "number",
    "cargoLoad": "number (tons)"
  },
  "onTimePerformance": "percentage",
  "alerts": [
    {
      "type": "delay|emergency|maintenance",
      "severity": "info|warning|critical",
      "message": "string",
      "timestamp": "ISO 8601 datetime"
    }
  ]
}
```

## 3. Validation Rules

### 3.1 Required Fields

**TransportationRoute:**
- `@context`, `@type`, `id`, `routeType`, `name`, `operator`, `origin`, `destination`

**TransportationBooking:**
- `@context`, `@type`, `id`, `bookingReference`, `status`, `passenger`, `journey`, `pricing`

**CargoShipment:**
- `@context`, `@type`, `id`, `trackingNumber`, `status`, `shipper`, `consignee`, `cargo`, `route`

### 3.2 Data Constraints

- Coordinates: Latitude [-90, 90], Longitude [-180, 180]
- Dates: ISO 8601 format (YYYY-MM-DD)
- Times: ISO 8601 format (HH:MM:SS or HH:MM)
- Datetime: ISO 8601 format with timezone (e.g., 2025-12-25T09:00:00Z)
- Currency codes: ISO 4217 (e.g., USD, KRW, CNY)
- Country codes: ISO 3166-1 alpha-2 (e.g., KR, KP, CN)

### 3.3 Semantic Validation

All data must be valid JSON-LD and resolve against the WIA-UNI-008 context:
`https://wiastandards.com/contexts/uni-008/v1`

## 4. Geospatial Standards

### 4.1 Coordinate Reference System

- **CRS:** WGS84 (EPSG:4326)
- **Format:** Decimal degrees
- **Precision:** Minimum 6 decimal places

### 4.2 Elevation Data

- **Datum:** EGM96 (Earth Gravitational Model 1996)
- **Unit:** Meters above mean sea level
- **Optional** for most use cases

## 5. Extensions

The data format supports extensions for:

- **Emergency protocols** - Additional fields for emergency situations
- **Environmental data** - Carbon emissions, fuel consumption tracking
- **Accessibility** - Wheelchair access, special assistance requirements
- **Integration with customs** - Trade facilitation data
- **Tourism** - Tourist information, multi-language support

## 6. Implementation Examples

### 6.1 Trans-Korean Railway Route

```json
{
  "@context": "https://wiastandards.com/contexts/uni-008/v1",
  "@type": "TransportationRoute",
  "id": "wia:uni:route:tkr-2025-001",
  "routeType": "high-speed-rail",
  "name": "Trans-Korean Express",
  "description": "High-speed rail service connecting Busan to Sinuiju via Seoul and Pyongyang",
  "operator": {
    "name": "Korea Railway Corporation",
    "operatorCode": "KORAIL",
    "country": "KR"
  },
  "origin": {
    "name": "Busan Station",
    "code": "BUS",
    "location": {
      "latitude": 35.1156,
      "longitude": 129.0403
    }
  },
  "destination": {
    "name": "Sinuiju Station",
    "code": "SIN",
    "location": {
      "latitude": 40.1021,
      "longitude": 124.3976
    }
  },
  "waypoints": [
    {
      "name": "Seoul Station",
      "code": "SEL",
      "location": { "latitude": 37.5547, "longitude": 126.9707 },
      "stopDuration": 15
    },
    {
      "name": "Kaesong Station",
      "code": "KAE",
      "location": { "latitude": 37.9704, "longitude": 126.5550 },
      "stopDuration": 10
    },
    {
      "name": "Pyongyang Station",
      "code": "PYO",
      "location": { "latitude": 39.0194, "longitude": 125.7541 },
      "stopDuration": 20
    }
  ],
  "distance": {
    "value": 745,
    "unit": "km"
  },
  "estimatedDuration": {
    "value": 5.5,
    "unit": "hours"
  },
  "schedule": {
    "frequency": "daily",
    "departureTimes": ["06:00", "09:00", "13:00", "17:00"],
    "validFrom": "2026-01-01",
    "validUntil": "2026-12-31"
  },
  "specifications": {
    "maxSpeed": 350,
    "capacity": {
      "passengers": 400,
      "cargo": 0
    },
    "gauge": 1435,
    "electrification": "AC 25kV 60Hz",
    "signaling": "ERTMS-Level-2"
  },
  "standards": ["WIA-UNI-008", "ERTMS-Level-2", "UIC-RIC"],
  "certificationStatus": "certified",
  "metadata": {
    "createdBy": "system-admin",
    "createdAt": "2025-12-25T00:00:00Z",
    "version": "1.0.0"
  }
}
```

## 7. References

- **JSON-LD:** https://www.w3.org/TR/json-ld11/
- **ISO 8601:** Date and time format
- **ISO 4217:** Currency codes
- **ISO 3166:** Country codes
- **WGS84:** Geographic coordinate system
- **UIC RIC:** Union Internationale des Chemins de fer Railway Interoperability Codes

---

**弘益人間 (홍익인간) · Benefit All Humanity**

© 2025 SmileStory Inc. / WIA
