# WIA-SOC-007 Phase 1: Data Format Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 1 defines the standardized data formats for public transportation systems, including transit schedules, real-time vehicle positions, route information, and fare structures. All data MUST use GTFS and GTFS-Realtime formats with WIA-SOC-007 extensions.

## 2. Core Data Types

### 2.1 Transit Agency

```json
{
  "@context": "https://wiastandards.com/soc-007/v1",
  "@type": "TransitAgency",
  "agencyId": "agency-2025-XXXX",
  "agencyName": "Metropolitan Transit Authority",
  "agencyUrl": "https://example-transit.com",
  "agencyTimezone": "America/New_York",
  "agencyLang": "en",
  "agencyPhone": "+1-555-123-4567",
  "agencyFareUrl": "https://example-transit.com/fares",
  "agencyEmail": "info@example-transit.com"
}
```

### 2.2 Route Information

```json
{
  "@type": "TransitRoute",
  "routeId": "route-101",
  "agencyId": "agency-2025-XXXX",
  "routeShortName": "101",
  "routeLongName": "Downtown Express",
  "routeDesc": "Express service between downtown and airport",
  "routeType": 3,
  "routeUrl": "https://example-transit.com/routes/101",
  "routeColor": "FF6319",
  "routeTextColor": "FFFFFF",
  "routeSortOrder": 101,
  "continuousPickup": 0,
  "continuousDropOff": 0
}
```

Route Type Codes:
- 0: Tram, Streetcar, Light rail
- 1: Subway, Metro
- 2: Rail (intercity or long-distance)
- 3: Bus
- 4: Ferry
- 5: Cable tram
- 6: Aerial lift, suspended cable car
- 7: Funicular
- 11: Trolleybus
- 12: Monorail

### 2.3 Stop/Station Data

```json
{
  "@type": "TransitStop",
  "stopId": "stop-12345",
  "stopCode": "12345",
  "stopName": "Main Street Station",
  "stopDesc": "Major transfer hub with connections to all lines",
  "stopLat": 40.748817,
  "stopLon": -73.985428,
  "zoneId": "zone-1",
  "stopUrl": "https://example-transit.com/stops/12345",
  "locationType": 1,
  "parentStation": null,
  "stopTimezone": "America/New_York",
  "wheelchairBoarding": 1,
  "levelId": "level-ground",
  "platformCode": "A"
}
```

Location Type Values:
- 0: Stop or platform
- 1: Station
- 2: Entrance/Exit
- 3: Generic node
- 4: Boarding area

Wheelchair Boarding:
- 0: No information
- 1: Accessible
- 2: Not accessible

### 2.4 Trip Schedule

```json
{
  "@type": "TransitTrip",
  "tripId": "trip-101-001",
  "routeId": "route-101",
  "serviceId": "weekday-service",
  "tripHeadsign": "Airport Terminal",
  "tripShortName": "101A",
  "directionId": 0,
  "blockId": "block-101",
  "shapeId": "shape-101-outbound",
  "wheelchairAccessible": 1,
  "bikesAllowed": 1
}
```

### 2.5 Stop Times

```json
{
  "@type": "StopTime",
  "tripId": "trip-101-001",
  "arrivalTime": "08:30:00",
  "departureTime": "08:30:30",
  "stopId": "stop-12345",
  "stopSequence": 5,
  "stopHeadsign": "Airport via Downtown",
  "pickupType": 0,
  "dropOffType": 0,
  "continuousPickup": 1,
  "continuousDropOff": 1,
  "shapeDistTraveled": 2.5,
  "timepoint": 1
}
```

## 3. Real-time Data Formats

### 3.1 Vehicle Position

```json
{
  "@type": "VehiclePosition",
  "vehicleId": "bus-1234",
  "tripId": "trip-101-001",
  "routeId": "route-101",
  "position": {
    "latitude": 40.748817,
    "longitude": -73.985428,
    "bearing": 135.5,
    "speed": 12.5
  },
  "timestamp": "2025-12-26T14:32:15Z",
  "currentStopSequence": 5,
  "currentStatus": "IN_TRANSIT_TO",
  "congestionLevel": "RUNNING_SMOOTHLY",
  "occupancyStatus": "MANY_SEATS_AVAILABLE"
}
```

Current Status Values:
- INCOMING_AT: Vehicle is approaching stop
- STOPPED_AT: Vehicle is stopped at stop
- IN_TRANSIT_TO: Vehicle is between stops

Congestion Levels:
- UNKNOWN_CONGESTION_LEVEL
- RUNNING_SMOOTHLY
- STOP_AND_GO
- CONGESTION
- SEVERE_CONGESTION

Occupancy Status:
- EMPTY
- MANY_SEATS_AVAILABLE
- FEW_SEATS_AVAILABLE
- STANDING_ROOM_ONLY
- CRUSHED_STANDING_ROOM_ONLY
- FULL
- NOT_ACCEPTING_PASSENGERS

### 3.2 Trip Update

```json
{
  "@type": "TripUpdate",
  "tripId": "trip-101-001",
  "routeId": "route-101",
  "vehicleId": "bus-1234",
  "timestamp": "2025-12-26T14:32:15Z",
  "delay": 120,
  "stopTimeUpdates": [
    {
      "stopSequence": 6,
      "stopId": "stop-12346",
      "arrival": {
        "delay": 120,
        "time": "2025-12-26T08:35:00Z"
      },
      "departure": {
        "delay": 120,
        "time": "2025-12-26T08:35:30Z"
      },
      "scheduleRelationship": "SCHEDULED"
    }
  ]
}
```

### 3.3 Service Alert

```json
{
  "@type": "ServiceAlert",
  "alertId": "alert-2025-001",
  "cause": "CONSTRUCTION",
  "effect": "DETOUR",
  "url": "https://example-transit.com/alerts/2025-001",
  "headerText": {
    "en": "Route 101 Detour",
    "es": "Desvío de Ruta 101"
  },
  "descriptionText": {
    "en": "Due to construction on Main Street, Route 101 will follow a detour.",
    "es": "Debido a construcción en Main Street, la Ruta 101 seguirá un desvío."
  },
  "activePeriod": [
    {
      "start": "2025-12-26T06:00:00Z",
      "end": "2025-12-26T18:00:00Z"
    }
  ],
  "informedEntity": [
    {
      "agencyId": "agency-2025-XXXX",
      "routeId": "route-101",
      "routeType": 3
    }
  ]
}
```

## 4. Fare Data Structures

### 4.1 Fare Attributes

```json
{
  "@type": "FareAttributes",
  "fareId": "fare-adult-single",
  "price": 2.75,
  "currencyType": "USD",
  "paymentMethod": 0,
  "transfers": 1,
  "agencyId": "agency-2025-XXXX",
  "transferDuration": 7200
}
```

Payment Method:
- 0: Paid on board
- 1: Paid before boarding

### 4.2 Fare Rules

```json
{
  "@type": "FareRule",
  "fareId": "fare-adult-single",
  "routeId": "route-101",
  "originId": "zone-1",
  "destinationId": "zone-2",
  "containsId": null
}
```

## 5. Accessibility Data

### 5.1 Pathway Information

```json
{
  "@type": "Pathway",
  "pathwayId": "pathway-001",
  "fromStopId": "stop-entrance-1",
  "toStopId": "stop-platform-1",
  "pathwayMode": 1,
  "isBidirectional": 1,
  "length": 50.0,
  "traversalTime": 60,
  "stairCount": 0,
  "maxSlope": 0.083,
  "minWidth": 1.5,
  "signpostedAs": "Platform 1",
  "reversedSignpostedAs": "Exit"
}
```

Pathway Modes:
- 1: Walkway
- 2: Stairs
- 3: Moving sidewalk/travelator
- 4: Escalator
- 5: Elevator
- 6: Fare gate
- 7: Exit gate

## 6. WIA-SOC-007 Extensions

### 6.1 Carbon Footprint

```json
{
  "@type": "CarbonFootprint",
  "tripId": "trip-101-001",
  "emissionsPerPassengerKm": 0.041,
  "totalEmissions": 2.5,
  "comparisonToPrivateCar": {
    "savings": 85.2,
    "percentage": 77
  }
}
```

### 6.2 Crowding Prediction

```json
{
  "@type": "CrowdingPrediction",
  "tripId": "trip-101-001",
  "stopId": "stop-12345",
  "predictedOccupancy": "FEW_SEATS_AVAILABLE",
  "confidence": 0.87,
  "historicalAverage": "MANY_SEATS_AVAILABLE",
  "peakHourIndicator": true
}
```

## 7. Data Quality Requirements

### 7.1 Location Accuracy
- Stop coordinates MUST be accurate within 5 meters
- Vehicle positions MUST be updated every 15-30 seconds
- Route shapes MUST follow actual vehicle paths

### 7.2 Schedule Accuracy
- Static schedules MUST be updated within 24 hours of changes
- Real-time predictions MUST be within 2 minutes accuracy (95th percentile)
- Service alerts MUST be published within 5 minutes of incident

### 7.3 Data Freshness
- GTFS feeds MUST be regenerated at least daily
- GTFS-RT feeds MUST update every 30 seconds or less
- Historical data MUST be retained for minimum 90 days

## 8. Validation Rules

### 8.1 Required Fields
All entities MUST include:
- Unique identifiers
- Timestamp (for real-time data)
- Geographic coordinates (for location-based entities)

### 8.2 Data Consistency
- Trip IDs MUST reference valid routes
- Stop IDs in stop_times MUST reference valid stops
- Service IDs MUST have corresponding calendar entries
- Fare IDs MUST have corresponding fare attributes

### 8.3 Logical Constraints
- Arrival time <= Departure time
- Stop sequence numbers MUST be sequential
- Shape distances MUST be non-decreasing
- Transfer durations MUST be positive

## 9. Example Complete Feed

```json
{
  "@context": "https://wiastandards.com/soc-007/v1",
  "@type": "TransitFeed",
  "feedPublisher": "Metropolitan Transit Authority",
  "feedVersion": "20251226",
  "feedStartDate": "2025-12-26",
  "feedEndDate": "2026-03-26",
  "feedLang": "en",
  "agencies": [...],
  "routes": [...],
  "trips": [...],
  "stops": [...],
  "stopTimes": [...],
  "calendar": [...],
  "fareAttributes": [...],
  "fareRules": [...]
}
```

---

**Next**: [Phase 2 - API Interface](PHASE-2-API.md)

弘益人間 - Benefit All Humanity

© 2025 WIA / SmileStory Inc.
