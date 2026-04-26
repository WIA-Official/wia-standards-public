# WIA Deep Sea Exploration - Phase 1: Data Format Specification
**Version:** 1.0
**Status:** Active
**Last Updated:** 2025-01-15
**Philosophy:** 弘益人間 (Benefit All Humanity)

---

## 1. Overview

Phase 1 of the WIA Deep Sea Exploration Standard defines standardized data formats for oceanographic research, underwater vehicle telemetry, sensor data collection, and sample metadata. This specification ensures interoperability between different research institutions, vehicle manufacturers, and data repositories worldwide.

### 1.1 Scope

This specification covers:
- Bathymetric mapping data formats
- Sensor data structures (temperature, pressure, salinity, oxygen, pH)
- Sample collection metadata
- Vehicle telemetry formats
- Mission planning and execution logs
- Environmental monitoring data
- Acoustic survey data
- Video and image metadata

### 1.2 Design Principles

1. **Extensibility**: Support for future sensor types and data sources
2. **Interoperability**: Compatible with existing oceanographic standards (NOAA, OOI, MBARI)
3. **Efficiency**: Optimized for low-bandwidth underwater communication
4. **Reliability**: Built-in validation and error detection
5. **Accessibility**: Human-readable JSON with binary alternatives for bandwidth-constrained scenarios

---

## 2. Core Data Structures

### 2.1 Base Message Format

All WIA Deep Sea Exploration messages follow this base structure:

```json
{
  "wiaVersion": "1.0",
  "messageType": "DATA_PACKET",
  "timestamp": "2025-01-15T14:30:00.000Z",
  "sequenceNumber": 12345,
  "sourceId": "ROV-ATLANTIS-001",
  "priority": "NORMAL",
  "payload": {},
  "metadata": {},
  "checksum": "SHA256:abc123..."
}
```

**Field Descriptions:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| wiaVersion | string | Yes | WIA standard version (semantic versioning) |
| messageType | enum | Yes | Type of message (DATA_PACKET, TELEMETRY, COMMAND, etc.) |
| timestamp | ISO8601 | Yes | UTC timestamp with millisecond precision |
| sequenceNumber | integer | Yes | Monotonically increasing sequence number |
| sourceId | string | Yes | Unique identifier for the data source |
| priority | enum | Yes | Message priority (LOW, NORMAL, HIGH, CRITICAL) |
| payload | object | Yes | Message-specific data |
| metadata | object | No | Optional metadata for routing, filtering |
| checksum | string | Yes | SHA256 hash for integrity verification |

### 2.2 Oceanographic Data Packet

Standard format for environmental measurements:

```json
{
  "wiaVersion": "1.0",
  "messageType": "OCEANOGRAPHIC_DATA",
  "timestamp": "2025-01-15T14:30:00.000Z",
  "sequenceNumber": 12345,
  "sourceId": "ROV-ATLANTIS-001",
  "priority": "NORMAL",
  "payload": {
    "location": {
      "latitude": 36.7977,
      "longitude": -121.8472,
      "depth": 3547.2,
      "altitude": 12.5,
      "coordinateSystem": "WGS84",
      "accuracy": {
        "horizontal": 2.3,
        "vertical": 0.8,
        "unit": "meters"
      }
    },
    "environment": {
      "temperature": {
        "value": 2.47,
        "unit": "celsius",
        "sensorId": "TEMP-01",
        "calibrationDate": "2024-12-01"
      },
      "pressure": {
        "value": 354.72,
        "unit": "bar",
        "sensorId": "PRESS-01",
        "calibrationDate": "2024-12-01"
      },
      "salinity": {
        "value": 34.89,
        "unit": "PSU",
        "sensorId": "SAL-01",
        "calibrationDate": "2024-12-01"
      },
      "dissolvedOxygen": {
        "value": 6.23,
        "unit": "mg/L",
        "sensorId": "DO-01",
        "calibrationDate": "2024-12-01"
      },
      "pH": {
        "value": 7.82,
        "unit": "pH",
        "sensorId": "PH-01",
        "calibrationDate": "2024-12-01"
      },
      "turbidity": {
        "value": 0.45,
        "unit": "NTU",
        "sensorId": "TURB-01",
        "calibrationDate": "2024-12-01"
      },
      "soundVelocity": {
        "value": 1489.3,
        "unit": "m/s",
        "sensorId": "SV-01",
        "calibrationDate": "2024-12-01"
      }
    },
    "waterCurrent": {
      "speed": 0.23,
      "direction": 147.5,
      "unit": "m/s",
      "directionReference": "true-north"
    }
  },
  "metadata": {
    "mission": "HYDROTHERMAL-SURVEY-2025-01",
    "institution": "MBARI",
    "researchVessel": "R/V Western Flyer",
    "chiefScientist": "Dr. Jane Smith",
    "dataQuality": "VALIDATED"
  },
  "checksum": "SHA256:e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
}
```

### 2.3 Bathymetric Data Format

For seafloor mapping and terrain modeling:

```json
{
  "wiaVersion": "1.0",
  "messageType": "BATHYMETRIC_DATA",
  "timestamp": "2025-01-15T14:30:00.000Z",
  "sequenceNumber": 12346,
  "sourceId": "AUV-SENTRY-042",
  "priority": "NORMAL",
  "payload": {
    "surveyArea": {
      "boundingBox": {
        "northWest": {"lat": 36.8, "lon": -121.85},
        "southEast": {"lat": 36.79, "lon": -121.84}
      },
      "gridResolution": 1.0,
      "gridUnit": "meters",
      "coordinateSystem": "WGS84"
    },
    "sonarConfiguration": {
      "type": "MULTIBEAM",
      "model": "Reson SeaBat 7125",
      "frequency": 400,
      "frequencyUnit": "kHz",
      "beamWidth": 1.0,
      "beamWidthUnit": "degrees",
      "swathWidth": 165,
      "swathWidthUnit": "degrees"
    },
    "soundings": [
      {
        "latitude": 36.7977,
        "longitude": -121.8472,
        "depth": 3547.2,
        "uncertainty": 0.3,
        "intensity": 142,
        "beamAngle": 45.2,
        "quality": "VERIFIED"
      }
    ],
    "processingParameters": {
      "soundVelocityProfile": "SVP-2025-01-15-001",
      "tidalCorrection": true,
      "rayTracing": "SNELL",
      "artifactRemoval": true
    }
  },
  "checksum": "SHA256:..."
}
```

### 2.4 Sample Collection Metadata

For biological, geological, and chemical samples:

```json
{
  "wiaVersion": "1.0",
  "messageType": "SAMPLE_METADATA",
  "timestamp": "2025-01-15T14:30:00.000Z",
  "sequenceNumber": 12347,
  "sourceId": "ROV-JASON-007",
  "priority": "HIGH",
  "payload": {
    "sampleId": "SAMPLE-2025-01-15-001",
    "sampleType": "BIOLOGICAL",
    "collectionMethod": "MANIPULATOR_ARM",
    "location": {
      "latitude": 36.7977,
      "longitude": -121.8472,
      "depth": 3547.2,
      "habitat": "HYDROTHERMAL_VENT",
      "substrate": "BASALT"
    },
    "environment": {
      "temperature": 2.47,
      "pressure": 354.72,
      "salinity": 34.89,
      "pH": 7.82
    },
    "specimen": {
      "taxonCandidate": "Riftia pachyptila",
      "description": "Tube worm colony, approximately 1.2m length",
      "photographs": [
        "IMG-2025-01-15-001.jpg",
        "IMG-2025-01-15-002.jpg"
      ],
      "videoClip": "VID-2025-01-15-001.mp4"
    },
    "container": {
      "type": "BIO_BOX",
      "containerId": "BB-07",
      "preservative": "SEAWATER",
      "temperature": "AMBIENT"
    },
    "collector": {
      "name": "Dr. Robert Chen",
      "institution": "Woods Hole Oceanographic Institution",
      "orcid": "0000-0002-1234-5678"
    },
    "permits": [
      "NOAA-PERMIT-2025-001",
      "STATE-CALIFORNIA-MARINE-001"
    ]
  },
  "checksum": "SHA256:..."
}
```

### 2.5 Vehicle Telemetry Format

Real-time status and health monitoring:

```json
{
  "wiaVersion": "1.0",
  "messageType": "VEHICLE_TELEMETRY",
  "timestamp": "2025-01-15T14:30:00.000Z",
  "sequenceNumber": 12348,
  "sourceId": "ROV-ATLANTIS-001",
  "priority": "HIGH",
  "payload": {
    "vehicle": {
      "type": "ROV",
      "model": "Remotely Operated Vehicle - Work Class",
      "manufacturer": "Oceaneering",
      "serialNumber": "ROV-2018-042",
      "maxDepthRating": 6000,
      "maxDepthRatingUnit": "meters"
    },
    "position": {
      "latitude": 36.7977,
      "longitude": -121.8472,
      "depth": 3547.2,
      "heading": 187.5,
      "pitch": -2.3,
      "roll": 1.7,
      "altitude": 12.5
    },
    "propulsion": {
      "thrusterStatus": {
        "forward": {"power": 45, "rpm": 1200, "status": "OPERATIONAL"},
        "aft": {"power": 45, "rpm": 1200, "status": "OPERATIONAL"},
        "port": {"power": 30, "rpm": 800, "status": "OPERATIONAL"},
        "starboard": {"power": 30, "rpm": 800, "status": "OPERATIONAL"},
        "vertical_fore": {"power": 60, "rpm": 1500, "status": "OPERATIONAL"},
        "vertical_aft": {"power": 60, "rpm": 1500, "status": "OPERATIONAL"}
      }
    },
    "power": {
      "batteryVoltage": 48.2,
      "batteryCurrent": 23.5,
      "batteryCapacityRemaining": 68,
      "estimatedTimeRemaining": 242,
      "estimatedTimeRemainingUnit": "minutes",
      "powerConsumption": 1134,
      "powerConsumptionUnit": "watts"
    },
    "systems": {
      "hydraulics": {"pressure": 3000, "temperature": 35, "status": "NOMINAL"},
      "cameras": {"hd_main": "ACTIVE", "hd_zoom": "ACTIVE", "sd_pilot": "ACTIVE"},
      "lights": {"intensity": 75, "status": "ON"},
      "manipulators": {"left": "STOWED", "right": "DEPLOYED"},
      "sonar": {"status": "ACTIVE", "range": 100, "rangeUnit": "meters"},
      "communications": {
        "fiber": "CONNECTED",
        "acoustic": "STANDBY",
        "bandwidth": 10,
        "bandwidthUnit": "Mbps"
      }
    },
    "alerts": []
  },
  "checksum": "SHA256:..."
}
```

---

## 3. Data Validation Rules

### 3.1 Required Field Validation

All messages MUST include:
- Valid `wiaVersion` matching this specification
- ISO8601 `timestamp` in UTC
- Unique `sequenceNumber` (monotonically increasing per source)
- Valid `sourceId` (alphanumeric, hyphens, underscores only)
- Valid `messageType` from enumerated list
- Valid `checksum` matching payload

### 3.2 Range Validation

Oceanographic parameters MUST fall within these ranges:

| Parameter | Minimum | Maximum | Unit |
|-----------|---------|---------|------|
| Depth | 0 | 11,000 | meters |
| Temperature | -2 | 400 | celsius |
| Pressure | 0 | 1,100 | bar |
| Salinity | 0 | 50 | PSU |
| pH | 0 | 14 | pH |
| Dissolved Oxygen | 0 | 20 | mg/L |
| Latitude | -90 | 90 | degrees |
| Longitude | -180 | 180 | degrees |

Values outside these ranges MUST be flagged with data quality indicator "OUT_OF_RANGE".

### 3.3 Timestamp Synchronization

All timestamps MUST be synchronized to UTC within ±1 second. Systems SHOULD use NTP or GPS time sources. For underwater vehicles without real-time surface communication, time drift MUST be documented in metadata.

---

## 4. Binary Format (Optional)

For bandwidth-constrained underwater acoustic communication, a binary format is provided:

### 4.1 Binary Header (32 bytes)

```
Offset  Size  Field
0       4     Magic Number (0x57494145 = "WIAE")
4       2     Version (Major.Minor)
6       1     Message Type (enum)
7       1     Priority (0-3)
8       8     Timestamp (Unix nanoseconds)
16      4     Sequence Number
20      8     Source ID (hash)
28      4     Payload Length
```

### 4.2 Binary Payload

Payload follows Protocol Buffers v3 encoding for efficiency. Schema definitions available in `/schemas/protobuf/`.

---

## 5. File Storage Formats

### 5.1 Mission Data Archive

Mission data SHOULD be archived in the following structure:

```
mission-<id>/
├── metadata.json
├── telemetry/
│   ├── vehicle-telemetry-001.jsonl
│   ├── vehicle-telemetry-002.jsonl
│   └── ...
├── oceanographic/
│   ├── ctd-data-001.jsonl
│   ├── ctd-data-002.jsonl
│   └── ...
├── bathymetry/
│   ├── multibeam-raw/
│   ├── multibeam-processed/
│   └── bathymetry-grid.tiff
├── samples/
│   ├── biological/
│   ├── geological/
│   └── chemical/
├── imagery/
│   ├── photos/
│   ├── video/
│   └── imagery-index.json
└── logs/
    ├── mission-log.txt
    └── event-log.jsonl
```

### 5.2 JSONL Format for Time Series

Time-series data SHOULD use JSON Lines format (.jsonl) with one complete JSON object per line:

```
{"timestamp":"2025-01-15T14:30:00.000Z","depth":3547.2,"temp":2.47}
{"timestamp":"2025-01-15T14:30:01.000Z","depth":3547.3,"temp":2.46}
{"timestamp":"2025-01-15T14:30:02.000Z","depth":3547.4,"temp":2.45}
```

---

## 6. Compliance and Certification

### 6.1 Compliance Levels

- **Level 1 (Basic)**: Implements core message format with required fields
- **Level 2 (Standard)**: Implements all Phase 1 data types
- **Level 3 (Advanced)**: Implements binary format and full validation

### 6.2 Certification Process

Systems claiming WIA Deep Sea Exploration compliance MUST:
1. Pass the official validation test suite
2. Provide sample data demonstrating all implemented message types
3. Document any extensions or deviations from the standard
4. Undergo peer review by WIA certification committee

---

## 7. References

- NOAA National Oceanographic Data Center (NODC) Standards
- Ocean Observatories Initiative (OOI) Data Standards
- MBARI Deep Sea Guide
- International Oceanographic Data and Information Exchange (IODE)
- CF Conventions for NetCDF

---

**Document Control:**
- Author: WIA Standards Committee
- Contributors: NOAA, MBARI, WHOI, Scripps Institution of Oceanography
- License: CC BY 4.0
- Repository: https://github.com/WIA-Official/wia-standards

弘益人間 · Benefit All Humanity
