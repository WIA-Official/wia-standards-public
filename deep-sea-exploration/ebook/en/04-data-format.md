# Chapter 4: Data Format Specification (Phase 1)

## Standardized Formats for Oceanographic Data and Vehicle Telemetry

---

## 4.1 Base Message Format

### Universal Message Structure

Every WIA Deep Sea Exploration message follows a consistent base structure that ensures traceability, validation, and interoperability. This design draws inspiration from established data standards while addressing the unique requirements of underwater operations.

**Complete Base Message Schema**:

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "required": ["wiaVersion", "messageType", "timestamp", "sequenceNumber", "sourceId", "priority", "payload", "checksum"],
  "properties": {
    "wiaVersion": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+$",
      "description": "WIA standard version (e.g., '1.0')"
    },
    "messageType": {
      "type": "string",
      "enum": ["VEHICLE_TELEMETRY", "OCEANOGRAPHIC_DATA", "BATHYMETRIC_DATA", "SAMPLE_METADATA", "NAVIGATION_DATA", "VIDEO_METADATA", "MISSION_LOG", "COMMAND", "RESPONSE", "CUSTOM_DATA"]
    },
    "timestamp": {
      "type": "string",
      "format": "date-time",
      "description": "ISO8601 UTC timestamp with millisecond precision"
    },
    "sequenceNumber": {
      "type": "integer",
      "minimum": 0,
      "description": "Monotonically increasing per source"
    },
    "sourceId": {
      "type": "string",
      "pattern": "^[A-Za-z0-9_-]+$",
      "maxLength": 64,
      "description": "Unique identifier for data source"
    },
    "priority": {
      "type": "string",
      "enum": ["LOW", "NORMAL", "HIGH", "CRITICAL"]
    },
    "payload": {
      "type": "object",
      "description": "Message-specific data"
    },
    "metadata": {
      "type": "object",
      "description": "Optional routing and context metadata"
    },
    "checksum": {
      "type": "string",
      "pattern": "^SHA256:[a-f0-9]{64}$",
      "description": "SHA256 hash for integrity verification"
    }
  }
}
```

### Field Specifications

| Field | Type | Required | Description | Example |
|-------|------|----------|-------------|---------|
| wiaVersion | string | Yes | Standard version | "1.0" |
| messageType | enum | Yes | Type of message | "OCEANOGRAPHIC_DATA" |
| timestamp | ISO8601 | Yes | UTC with milliseconds | "2025-01-15T14:30:00.123Z" |
| sequenceNumber | integer | Yes | Monotonic counter | 12345 |
| sourceId | string | Yes | Unique source identifier | "ROV-ATLANTIS-001" |
| priority | enum | Yes | Transmission priority | "NORMAL" |
| payload | object | Yes | Message-specific content | {...} |
| metadata | object | No | Optional context | {...} |
| checksum | string | Yes | SHA256 hash | "SHA256:abc123..." |

### Source ID Conventions

The `sourceId` field uses a hierarchical naming convention:

```
[TYPE]-[PLATFORM]-[INSTANCE]

Examples:
ROV-JASON-002          (ROV named Jason, unit 2)
AUV-SENTRY-001         (AUV named Sentry, unit 1)
SENSOR-CTD-003         (CTD sensor, unit 3)
SHIP-FALKOR-001        (Research vessel Falkor)
MOORING-OOI-CE01-001   (OOI coastal endurance mooring)
```

### Timestamp Requirements

All timestamps MUST be:
- In UTC timezone (no local time)
- ISO8601 format with 'Z' suffix
- Minimum millisecond precision
- Synchronized to NTP or GPS time source

**Time Synchronization Quality Metadata**:

```json
{
  "metadata": {
    "timeSync": {
      "source": "GPS",
      "accuracy": 0.001,
      "accuracyUnit": "seconds",
      "lastSync": "2025-01-15T14:00:00.000Z",
      "drift": 0.0002
    }
  }
}
```

---

## 4.2 Oceanographic Data Packets

### CTD and Environmental Data

The oceanographic data packet captures water column properties measured by CTD (Conductivity, Temperature, Depth) and related sensors.

**Complete Oceanographic Data Message**:

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
        "calibrationDate": "2024-12-01",
        "accuracy": 0.001,
        "qualityFlag": "GOOD"
      },
      "pressure": {
        "value": 354.72,
        "unit": "bar",
        "sensorId": "PRESS-01",
        "calibrationDate": "2024-12-01",
        "accuracy": 0.01,
        "qualityFlag": "GOOD"
      },
      "salinity": {
        "value": 34.89,
        "unit": "PSU",
        "sensorId": "SAL-01",
        "calibrationDate": "2024-12-01",
        "accuracy": 0.001,
        "qualityFlag": "GOOD"
      },
      "dissolvedOxygen": {
        "value": 6.23,
        "unit": "mg/L",
        "sensorId": "DO-01",
        "calibrationDate": "2024-12-01",
        "accuracy": 0.01,
        "qualityFlag": "GOOD"
      },
      "pH": {
        "value": 7.82,
        "unit": "pH",
        "sensorId": "PH-01",
        "calibrationDate": "2024-12-01",
        "accuracy": 0.01,
        "qualityFlag": "GOOD"
      },
      "turbidity": {
        "value": 0.45,
        "unit": "NTU",
        "sensorId": "TURB-01",
        "qualityFlag": "GOOD"
      },
      "chlorophyll": {
        "value": 0.12,
        "unit": "ug/L",
        "sensorId": "CHL-01",
        "qualityFlag": "GOOD"
      },
      "soundVelocity": {
        "value": 1489.3,
        "unit": "m/s",
        "sensorId": "SV-01",
        "qualityFlag": "GOOD"
      }
    },
    "waterCurrent": {
      "speed": 0.23,
      "direction": 147.5,
      "unit": "m/s",
      "directionReference": "true-north",
      "sensorId": "ADCP-01"
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

### Quality Flags

The WIA standard uses quality flags aligned with international conventions:

| Flag Value | Description | Action |
|------------|-------------|--------|
| GOOD | Data passed all QC | Use without reservation |
| PROBABLY_GOOD | Minor QC issues | Use with awareness |
| PROBABLY_BAD | QC concerns | Use with caution |
| BAD | Failed QC | Do not use |
| NOT_APPLIED | QC not performed | Apply QC before use |
| INTERPOLATED | Gap-filled data | Use with awareness |
| MISSING | No data available | N/A |

### Parameter Validation Ranges

| Parameter | Minimum | Maximum | Unit | Notes |
|-----------|---------|---------|------|-------|
| Depth | 0 | 11,000 | meters | Full ocean depth |
| Temperature | -2 | 400 | °C | Includes vent fluids |
| Pressure | 0 | 1,100 | bar | Full ocean depth |
| Salinity | 0 | 50 | PSU | Includes hypersaline |
| pH | 0 | 14 | pH | Full range |
| Dissolved Oxygen | 0 | 20 | mg/L | Supersaturation |
| Latitude | -90 | 90 | degrees | WGS84 |
| Longitude | -180 | 180 | degrees | WGS84 |

---

## 4.3 Bathymetric Survey Data

### Multibeam Sonar Data

Bathymetric data from multibeam echosounders requires detailed metadata about the sonar configuration and processing parameters.

**Bathymetric Data Message**:

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
        "northWest": {"lat": 36.8000, "lon": -121.8500},
        "southEast": {"lat": 36.7900, "lon": -121.8400}
      },
      "gridResolution": 1.0,
      "gridUnit": "meters",
      "coordinateSystem": "WGS84",
      "verticalDatum": "MSL"
    },
    "sonarConfiguration": {
      "type": "MULTIBEAM",
      "manufacturer": "Kongsberg",
      "model": "EM304",
      "serialNumber": "EM304-2021-042",
      "frequency": {
        "center": 30,
        "range": [26, 34],
        "unit": "kHz"
      },
      "beamWidth": {
        "alongTrack": 1.0,
        "acrossTrack": 1.0,
        "unit": "degrees"
      },
      "swathWidth": {
        "value": 140,
        "unit": "degrees"
      },
      "maxDepthRating": 7000,
      "maxDepthRatingUnit": "meters"
    },
    "soundVelocityProfile": {
      "profileId": "SVP-2025-01-15-001",
      "timestamp": "2025-01-15T12:00:00.000Z",
      "profiles": [
        {"depth": 0, "velocity": 1520.3},
        {"depth": 100, "velocity": 1510.2},
        {"depth": 500, "velocity": 1495.1},
        {"depth": 1000, "velocity": 1485.0},
        {"depth": 2000, "velocity": 1489.5},
        {"depth": 3000, "velocity": 1502.3}
      ]
    },
    "processingParameters": {
      "tidalCorrection": {
        "applied": true,
        "source": "NOAA-COOPS",
        "station": "9413450"
      },
      "rayTracing": "SNELL",
      "artifactRemoval": {
        "applied": true,
        "algorithm": "MEDIAN_FILTER",
        "windowSize": 5
      }
    },
    "soundings": [
      {
        "latitude": 36.7977,
        "longitude": -121.8472,
        "depth": 3547.2,
        "uncertainty": 0.3,
        "intensity": 142,
        "beamAngle": 45.2,
        "quality": "VERIFIED",
        "timestamp": "2025-01-15T14:30:00.123Z"
      }
    ],
    "statistics": {
      "totalSoundings": 125000,
      "validSoundings": 123500,
      "rejectedSoundings": 1500,
      "minDepth": 3480.2,
      "maxDepth": 3612.8,
      "meanDepth": 3545.6
    }
  },
  "checksum": "SHA256:..."
}
```

### Side-Scan Sonar Data

```json
{
  "wiaVersion": "1.0",
  "messageType": "BATHYMETRIC_DATA",
  "payload": {
    "sonarConfiguration": {
      "type": "SIDE_SCAN",
      "manufacturer": "EdgeTech",
      "model": "2205",
      "frequency": {
        "low": 120,
        "high": 410,
        "unit": "kHz"
      },
      "range": {
        "value": 150,
        "unit": "meters",
        "perSide": true
      },
      "resolution": {
        "alongTrack": 0.1,
        "acrossTrack": 0.05,
        "unit": "meters"
      }
    },
    "imagery": {
      "format": "GEOTIFF",
      "compression": "LZW",
      "resolution": 0.1,
      "resolutionUnit": "meters",
      "files": [
        {
          "filename": "sss-2025-01-15-001.tif",
          "size": 524288000,
          "checksum": "SHA256:...",
          "boundingBox": {...}
        }
      ]
    }
  }
}
```

---

## 4.4 Sample Collection Metadata

### Biological Samples

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
    "category": "MACROFAUNA",
    "collectionMethod": {
      "type": "MANIPULATOR_ARM",
      "tool": "SUCTION_SAMPLER",
      "duration": 45,
      "durationUnit": "seconds"
    },
    "location": {
      "latitude": 36.7977,
      "longitude": -121.8472,
      "depth": 3547.2,
      "altitude": 0.5,
      "coordinateSystem": "WGS84",
      "accuracy": {"horizontal": 1.0, "vertical": 0.5, "unit": "meters"}
    },
    "habitat": {
      "type": "HYDROTHERMAL_VENT",
      "zone": "DIFFUSE_FLOW",
      "substrate": "BASALT",
      "associatedFeature": "CHIMNEY_BASE"
    },
    "environmentAtCollection": {
      "temperature": 12.5,
      "temperatureUnit": "celsius",
      "pH": 6.2,
      "dissolvedOxygen": 0.8,
      "dissolvedOxygenUnit": "mg/L"
    },
    "specimen": {
      "taxonCandidate": "Riftia pachyptila",
      "commonName": "Giant tube worm",
      "lifeStage": "ADULT",
      "count": 3,
      "measurements": {
        "length": {"value": 1.2, "unit": "meters"},
        "weight": {"value": 450, "unit": "grams", "estimated": true}
      },
      "condition": "LIVE",
      "description": "Tube worm colony, approximately 1.2m length, healthy coloration"
    },
    "preservation": {
      "method": "ETHANOL_95",
      "container": {
        "type": "BIO_BOX",
        "containerId": "BB-07",
        "capacity": 5,
        "capacityUnit": "liters"
      },
      "temperature": "AMBIENT",
      "preservative": "ETHANOL",
      "preservativeConcentration": 95
    },
    "documentation": {
      "photographs": [
        {"id": "IMG-2025-01-15-001", "type": "IN_SITU", "format": "JPEG"},
        {"id": "IMG-2025-01-15-002", "type": "COLLECTION", "format": "JPEG"}
      ],
      "videoClips": [
        {"id": "VID-2025-01-15-001", "duration": 120, "format": "MP4"}
      ]
    },
    "collector": {
      "name": "Dr. Robert Chen",
      "institution": "Woods Hole Oceanographic Institution",
      "orcid": "0000-0002-1234-5678"
    },
    "permits": [
      {"type": "NOAA_RESEARCH", "number": "NOAA-PERMIT-2025-001"},
      {"type": "STATE", "number": "CA-MARINE-2025-042"}
    ],
    "chainOfCustody": [
      {
        "action": "COLLECTED",
        "timestamp": "2025-01-15T14:30:00.000Z",
        "personnel": "Dr. Robert Chen",
        "location": "IN_SITU"
      },
      {
        "action": "TRANSFERRED",
        "timestamp": "2025-01-15T18:00:00.000Z",
        "personnel": "Ship Lab Tech",
        "location": "R/V WESTERN_FLYER"
      }
    ]
  },
  "checksum": "SHA256:..."
}
```

### Geological Samples

```json
{
  "sampleType": "GEOLOGICAL",
  "category": "ROCK",
  "specimen": {
    "rockType": "BASALT",
    "formation": "PILLOW_LAVA",
    "mineralogy": ["OLIVINE", "PYROXENE", "PLAGIOCLASE"],
    "texture": "GLASSY_RIND",
    "alteration": "MINIMAL",
    "measurements": {
      "dimensions": {"x": 15, "y": 12, "z": 8, "unit": "centimeters"},
      "weight": {"value": 2.3, "unit": "kilograms"}
    }
  }
}
```

---

## 4.5 Vehicle Telemetry Format

### Comprehensive Vehicle Status

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
      "class": "WORK_CLASS",
      "manufacturer": "Oceaneering",
      "model": "Millennium Plus",
      "serialNumber": "ROV-2018-042",
      "maxDepthRating": 6000,
      "maxDepthRatingUnit": "meters"
    },
    "position": {
      "latitude": 36.7977,
      "longitude": -121.8472,
      "depth": 3547.2,
      "altitude": 12.5,
      "heading": 187.5,
      "pitch": -2.3,
      "roll": 1.7,
      "headingAccuracy": 0.5,
      "positionSource": "USBL"
    },
    "velocity": {
      "forward": 0.5,
      "lateral": 0.1,
      "vertical": -0.02,
      "unit": "m/s",
      "source": "DVL"
    },
    "propulsion": {
      "thrusterStatus": [
        {"id": "FORE_PORT", "power": 45, "rpm": 1200, "status": "OPERATIONAL", "current": 12.5},
        {"id": "FORE_STBD", "power": 45, "rpm": 1200, "status": "OPERATIONAL", "current": 12.3},
        {"id": "AFT_PORT", "power": 42, "rpm": 1150, "status": "OPERATIONAL", "current": 11.8},
        {"id": "AFT_STBD", "power": 43, "rpm": 1170, "status": "OPERATIONAL", "current": 12.0},
        {"id": "VERT_FORE", "power": 60, "rpm": 1500, "status": "OPERATIONAL", "current": 15.2},
        {"id": "VERT_AFT", "power": 58, "rpm": 1480, "status": "OPERATIONAL", "current": 14.8},
        {"id": "LAT_FORE", "power": 30, "rpm": 800, "status": "OPERATIONAL", "current": 8.5},
        {"id": "LAT_AFT", "power": 28, "rpm": 780, "status": "OPERATIONAL", "current": 8.2}
      ]
    },
    "power": {
      "source": "TETHER",
      "inputVoltage": 3000,
      "inputCurrent": 25.3,
      "inputPower": 75900,
      "batteryBackup": {
        "voltage": 48.2,
        "capacity": 85,
        "capacityUnit": "percent",
        "estimatedRuntime": 45,
        "estimatedRuntimeUnit": "minutes"
      },
      "totalConsumption": 12500,
      "consumptionUnit": "watts"
    },
    "systems": {
      "hydraulics": {
        "pressure": 3000,
        "pressureUnit": "psi",
        "temperature": 35,
        "temperatureUnit": "celsius",
        "fluidLevel": 95,
        "status": "NOMINAL"
      },
      "cameras": [
        {"id": "HD_MAIN", "status": "ACTIVE", "recording": true},
        {"id": "HD_ZOOM", "status": "ACTIVE", "zoom": 5.2},
        {"id": "SD_PILOT", "status": "ACTIVE", "recording": false},
        {"id": "STILL", "status": "STANDBY", "imagesRemaining": 2500}
      ],
      "lights": [
        {"id": "MAIN_ARRAY", "intensity": 75, "status": "ON", "power": 400},
        {"id": "AUX_LEFT", "intensity": 50, "status": "ON", "power": 150},
        {"id": "AUX_RIGHT", "intensity": 50, "status": "ON", "power": 150}
      ],
      "manipulators": {
        "port": {"position": "DEPLOYED", "grip": "OPEN", "force": 0},
        "starboard": {"position": "STOWED", "grip": "CLOSED", "force": 0}
      },
      "sonar": {
        "multibeam": {"status": "ACTIVE", "pinging": true},
        "imagingSonar": {"status": "STANDBY"},
        "forwardLooking": {"status": "ACTIVE", "range": 100}
      },
      "sampling": {
        "suctionSampler": {"status": "READY", "containersFree": 5},
        "pushCores": {"status": "READY", "coresRemaining": 6},
        "bioBoxes": {"status": "READY", "boxesFree": 3}
      }
    },
    "communication": {
      "tether": {
        "status": "CONNECTED",
        "bandwidth": 1000,
        "bandwidthUnit": "Mbps",
        "latency": 2,
        "latencyUnit": "ms"
      },
      "acoustic": {
        "status": "STANDBY",
        "lastPing": "2025-01-15T14:29:55.000Z"
      }
    },
    "environment": {
      "externalTemperature": 2.4,
      "externalPressure": 354.7,
      "internalTemperature": 22.5,
      "humidity": 45
    },
    "alerts": [],
    "missionStatus": {
      "currentWaypoint": 5,
      "totalWaypoints": 12,
      "missionElapsed": 14400,
      "missionElapsedUnit": "seconds",
      "estimatedRemaining": 10800
    }
  },
  "checksum": "SHA256:..."
}
```

---

## 4.6 Video and Image Metadata

```json
{
  "wiaVersion": "1.0",
  "messageType": "VIDEO_METADATA",
  "timestamp": "2025-01-15T14:30:00.000Z",
  "sourceId": "ROV-ATLANTIS-001",
  "payload": {
    "mediaType": "VIDEO",
    "recording": {
      "id": "VID-2025-01-15-001",
      "startTime": "2025-01-15T10:00:00.000Z",
      "duration": 16200,
      "durationUnit": "seconds",
      "format": "H.265",
      "resolution": {"width": 3840, "height": 2160},
      "frameRate": 30,
      "bitRate": 25000000,
      "colorSpace": "REC2020"
    },
    "camera": {
      "id": "HD_MAIN",
      "manufacturer": "DeepSea Power & Light",
      "model": "4K Ultra",
      "serialNumber": "DSP-4K-2023-042",
      "lens": "Wide-angle 100°",
      "settings": {
        "aperture": "f/2.8",
        "iso": 800,
        "whiteBalance": "5600K",
        "focus": "AUTO"
      }
    },
    "georeferencing": {
      "trackFile": "VID-2025-01-15-001-track.csv",
      "interpolationMethod": "LINEAR",
      "positionAtFrame": {
        "frameNumber": 486000,
        "latitude": 36.7977,
        "longitude": -121.8472,
        "depth": 3547.2
      }
    },
    "annotations": [
      {
        "frameNumber": 125000,
        "timestamp": "2025-01-15T11:09:26.667Z",
        "type": "SPECIES_ID",
        "label": "Bathynomus giganteus",
        "confidence": 0.92,
        "boundingBox": {"x": 100, "y": 200, "width": 150, "height": 80}
      }
    ],
    "storage": {
      "filename": "VID-2025-01-15-001.mp4",
      "size": 50856345600,
      "checksum": "SHA256:...",
      "location": "SHIP_NAS_01"
    }
  }
}
```

---

## 4.7 Binary Format for Low-Bandwidth

### Binary Header (32 bytes)

For acoustic communication where bandwidth is extremely limited:

```
Offset  Size  Type      Field
0       4     uint32    Magic Number (0x57494145 = "WIAE")
4       1     uint8     Version Major
5       1     uint8     Version Minor
6       1     uint8     Message Type (enum)
7       1     uint8     Priority (0-3)
8       8     int64     Timestamp (Unix nanoseconds)
16      4     uint32    Sequence Number
20      4     uint32    Source ID (hash)
24      4     uint32    Payload Length
28      4     uint32    CRC32 Checksum
```

### Compact Telemetry (48 bytes)

Essential vehicle status in minimal bytes:

```
Offset  Size  Type      Field
0       4     int32     Latitude (microdegrees)
4       4     int32     Longitude (microdegrees)
8       4     int32     Depth (millimeters)
12      2     int16     Heading (centidegrees)
14      1     int8      Pitch (degrees)
15      1     int8      Roll (degrees)
16      2     uint16    Altitude (centimeters)
18      1     uint8     Battery (percent)
19      1     uint8     Thruster Health (bitfield)
20      2     int16     Temperature (centidegrees)
22      2     uint16    Pressure (decibar)
24      1     uint8     System Status (bitfield)
25      1     uint8     Alert Count
26      2     uint16    Reserved
```

---

## 4.8 File Storage and Archival

### Mission Data Archive Structure

```
mission-HYDROTHERMAL-2025-01/
├── metadata.json
├── README.md
├── telemetry/
│   ├── vehicle-telemetry-001.jsonl
│   ├── vehicle-telemetry-002.jsonl
│   └── index.json
├── oceanographic/
│   ├── ctd-data-001.jsonl
│   ├── ctd-profiles/
│   │   └── profile-2025-01-15-001.json
│   └── index.json
├── bathymetry/
│   ├── multibeam-raw/
│   ├── multibeam-processed/
│   ├── grids/
│   │   └── bathymetry-1m.tiff
│   └── index.json
├── samples/
│   ├── biological/
│   ├── geological/
│   └── index.json
├── imagery/
│   ├── video/
│   ├── photos/
│   └── index.json
└── logs/
    ├── mission-log.txt
    └── events.jsonl
```

---

## Chapter Summary

Phase 1 of the WIA Deep Sea Exploration Standard establishes comprehensive data format specifications for all major data types encountered in underwater operations. From real-time vehicle telemetry to detailed sample metadata, the standard ensures that data is self-describing, validated, and interoperable across systems.

The JSON-based formats provide human readability and broad tool support, while the binary format enables essential data transmission over bandwidth-constrained acoustic links. Quality flags, validation ranges, and checksum requirements ensure data integrity throughout the data lifecycle.

---

## Key Takeaways

1. **Every message includes base fields** for version, type, timestamp, source, priority, and checksum
2. **Quality flags follow international conventions** (GOOD, PROBABLY_GOOD, BAD, etc.)
3. **Parameter validation prevents obvious errors** before data is stored
4. **Binary format reduces message size by 90%+** for acoustic transmission
5. **Hierarchical archive structure** organizes mission data logically

---

## Review Questions

1. What are the required fields in every WIA base message?
2. What quality flag should be assigned to data that fails validation?
3. Calculate the checksum for a sample message payload.
4. What is the purpose of the binary format and when should it be used?
5. Design a sample metadata record for a geological core sample.

---

© 2025 SmileStory Inc. / WIA
弘益人間 (Hongik Ingan) · Benefit All Humanity
