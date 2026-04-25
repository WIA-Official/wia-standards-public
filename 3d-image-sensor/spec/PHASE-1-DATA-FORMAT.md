# WIA-SEMI-013 Phase 1: Data Format Specification

**Version:** 1.0  
**Date:** 2025-01-15  
**Status:** Published  

## Overview

Phase 1 defines standardized data formats for 3D image sensor specifications, depth maps, point clouds, and calibration data. These formats enable interoperability across sensors, processing libraries, and applications.

## 1. Sensor Specification Schema

### JSON Schema for 3D Sensor Metadata

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "WIA-SEMI-013 Sensor Specification",
  "type": "object",
  "required": ["standard", "version", "sensor"],
  "properties": {
    "standard": {
      "type": "string",
      "const": "WIA-SEMI-013"
    },
    "version": {
      "type": "string",
      "pattern": "^\\d+\\.\\d+$"
    },
    "sensor": {
      "type": "object",
      "required": ["technology", "resolution", "frameRate", "depthRange"],
      "properties": {
        "technology": {
          "type": "string",
          "enum": ["iToF", "dToF", "structured_light", "stereo", "hybrid"]
        },
        "manufacturer": {"type": "string"},
        "model": {"type": "string"},
        "resolution": {
          "type": "object",
          "required": ["width", "height"],
          "properties": {
            "width": {"type": "integer", "minimum": 1},
            "height": {"type": "integer", "minimum": 1}
          }
        },
        "frameRate": {
          "type": "number",
          "minimum": 1,
          "maximum": 240
        },
        "depthRange": {
          "type": "object",
          "required": ["min", "max", "units"],
          "properties": {
            "min": {"type": "number", "minimum": 0},
            "max": {"type": "number"},
            "units": {"type": "string", "enum": ["meters", "millimeters"]}
          }
        },
        "accuracy": {
          "type": "object",
          "required": ["value", "units", "at_distance"],
          "properties": {
            "value": {"type": "number", "minimum": 0},
            "units": {"type": "string", "enum": ["mm", "cm", "percent"]},
            "at_distance": {"type": "number", "minimum": 0}
          }
        },
        "fieldOfView": {
          "type": "object",
          "properties": {
            "horizontal": {"type": "number", "minimum": 0, "maximum": 360},
            "vertical": {"type": "number", "minimum": 0, "maximum": 180},
            "units": {"type": "string", "enum": ["degrees", "radians"]}
          }
        },
        "wavelength": {
          "type": "integer",
          "description": "Illumination wavelength in nanometers",
          "minimum": 400,
          "maximum": 1550
        },
        "illumination": {
          "type": "string",
          "enum": ["active", "passive", "hybrid"]
        }
      }
    }
  }
}
```

### Example Sensor Specification

```json
{
  "standard": "WIA-SEMI-013",
  "version": "1.0",
  "sensor": {
    "technology": "iToF",
    "manufacturer": "Sony",
    "model": "IMX556PLR",
    "resolution": {"width": 640, "height": 480},
    "frameRate": 30,
    "depthRange": {"min": 0.2, "max": 5.0, "units": "meters"},
    "accuracy": {"value": 5, "units": "mm", "at_distance": 2.0},
    "fieldOfView": {"horizontal": 87, "vertical": 58, "units": "degrees"},
    "wavelength": 940,
    "illumination": "active"
  }
}
```

## 2. Depth Map Format

### Binary Depth Map Structure

Depth maps are encoded as 16-bit or 32-bit unsigned integers or 32-bit floating point values, with accompanying metadata.

**Header Structure (64 bytes):**

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 4 | magic | Magic number: 0x57494133 ("WIA3") |
| 4 | 2 | version | Format version (major.minor) |
| 6 | 2 | encoding | Encoding type (0=uint16, 1=uint32, 2=float32) |
| 8 | 4 | width | Image width in pixels |
| 12 | 4 | height | Image height in pixels |
| 16 | 4 | timestamp_sec | Unix timestamp (seconds) |
| 20 | 4 | timestamp_usec | Microseconds |
| 24 | 4 | depth_scale | Scale factor (depth_mm = value × scale) |
| 28 | 4 | min_depth | Minimum valid depth (mm) |
| 32 | 4 | max_depth | Maximum valid depth (mm) |
| 36 | 4 | confidence_offset | Offset to confidence map (0 if none) |
| 40 | 24 | reserved | Reserved for future use |

**Data follows header:**
- Depth values: width × height × sizeof(encoding type)
- Confidence map (optional): width × height × 1 byte (0-255 confidence)

### Text-Based Depth Map (CSV)

For debugging and interchange, CSV format is supported:

```
WIA-SEMI-013,1.0
timestamp,2025-01-15T10:30:45.123Z
width,640
height,480
encoding,uint16
depth_scale,1.0
x,y,depth_mm,confidence
0,0,1234,255
0,1,1235,254
...
```

## 3. Point Cloud Format

### WIA Point Cloud Binary Format

Extends PCD format with WIA-specific attributes.

**Header:**
```
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z rgb intensity confidence timestamp
SIZE 4 4 4 4 2 1 8
TYPE F F F U U U U
COUNT 1 1 1 1 1 1 1
WIDTH 307200
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 307200
DATA binary
```

**Field Definitions:**
- x, y, z: 3D coordinates in meters (float32)
- rgb: Packed RGB (uint32): R=bits 16-23, G=bits 8-15, B=bits 0-7
- intensity: Infrared intensity (uint16)
- confidence: Point confidence 0-255 (uint8)
- timestamp: Microseconds since frame start (uint64)

### JSON Point Cloud (for small datasets)

```json
{
  "standard": "WIA-SEMI-013",
  "version": "1.0",
  "coordinate_system": "camera",
  "units": "meters",
  "timestamp": "2025-01-15T10:30:45.123Z",
  "points": [
    {"x": 0.123, "y": 0.456, "z": 1.234, "r": 255, "g": 128, "b": 64, "confidence": 255},
    {"x": 0.124, "y": 0.457, "z": 1.235, "r": 254, "g": 129, "b": 65, "confidence": 254}
  ]
}
```

## 4. Calibration Data Format

### Intrinsic Calibration

```json
{
  "standard": "WIA-SEMI-013",
  "version": "1.0",
  "calibration_type": "intrinsic",
  "camera_id": "sensor-001",
  "timestamp": "2025-01-15T10:30:45.123Z",
  "resolution": {"width": 640, "height": 480},
  "camera_matrix": {
    "fx": 525.0,
    "fy": 525.0,
    "cx": 319.5,
    "cy": 239.5
  },
  "distortion": {
    "model": "brown_conrady",
    "coefficients": {
      "k1": -0.123,
      "k2": 0.045,
      "p1": 0.001,
      "p2": -0.002,
      "k3": 0.012
    }
  }
}
```

### Extrinsic Calibration (Multi-Sensor)

```json
{
  "standard": "WIA-SEMI-013",
  "version": "1.0",
  "calibration_type": "extrinsic",
  "timestamp": "2025-01-15T10:30:45.123Z",
  "transforms": [
    {
      "from": "depth_camera",
      "to": "rgb_camera",
      "rotation": {
        "format": "quaternion",
        "w": 0.9999,
        "x": 0.0001,
        "y": 0.0002,
        "z": 0.0001
      },
      "translation": {
        "x": 0.025,
        "y": 0.000,
        "z": 0.001,
        "units": "meters"
      }
    }
  ]
}
```

### Temperature Compensation

```json
{
  "standard": "WIA-SEMI-013",
  "version": "1.0",
  "compensation_type": "temperature",
  "temperature_table": [
    {"temp_celsius": -20, "depth_scale": 1.002, "baseline_mm": 50.05},
    {"temp_celsius": 0, "depth_scale": 1.001, "baseline_mm": 50.02},
    {"temp_celsius": 20, "depth_scale": 1.000, "baseline_mm": 50.00},
    {"temp_celsius": 40, "depth_scale": 0.999, "baseline_mm": 49.98},
    {"temp_celsius": 60, "depth_scale": 0.998, "baseline_mm": 49.95}
  ]
}
```

## 5. Data Format Compliance

### Validation Requirements

Implementations MUST:
1. Support all required fields in sensor specification schema
2. Correctly parse binary depth map header
3. Handle depth scale factors for unit conversion
4. Support confidence maps when present
5. Correctly interpret calibration parameters

Implementations SHOULD:
1. Support optional CSV depth map format for debugging
2. Provide utilities for format conversion
3. Validate data integrity with checksums
4. Support compression (gzip, lz4) for storage/transmission

### File Extensions

- Depth maps: `.wia-depth` (binary), `.wia-depth.csv` (text)
- Point clouds: `.pcd` (standard PCD with WIA fields)
- Calibration: `.wia-calib.json`
- Sensor specs: `.wia-sensor.json`

## 6. Coordinate Systems

### Standard Coordinate Frames

**Camera Coordinate System:**
- Origin: Optical center
- X-axis: Right (image columns)
- Y-axis: Down (image rows)
- Z-axis: Forward (depth direction)

**World Coordinate System:**
- Application-defined
- Transformation from camera to world provided in calibration

### Depth Measurement Convention

Depth is measured along the Z-axis (perpendicular distance from sensor plane), NOT Euclidean distance from optical center. This convention simplifies depth-to-3D conversion.

## 7. Versioning and Extensions

### Backward Compatibility

- Version 1.x formats are backward compatible
- Parsers must ignore unknown fields
- New optional fields may be added in minor versions

### Future Extensions

- Version 2.0 may introduce:
  - Neural radiance field (NeRF) representations
  - Compressed neural point clouds
  - Multi-spectral depth data
  - Event-based depth streams

## 7. Reference Standards Alignment

### 7.1 Geographic Metadata

When 3D sensor frames are tagged with positional metadata (mobile robotics, surveying, autonomous vehicles), the metadata block follows ISO 19115-1:2014 *Geographic information — Metadata — Part 1: Fundamentals*.

```json
{
  "geographic_metadata": {
    "iso19115_compliant": true,
    "reference_system": "EPSG:4326",
    "elevation_model": "EGM96",
    "acquisition_time_iso8601": "2026-04-26T08:14:32Z",
    "uncertainty_3d_m": [0.05, 0.05, 0.12]
  }
}
```

For raster depth maps with geographic alignment, the OGC GeoTIFF 1.1 standard is used as the on-disk container (key `GeoKeyDirectoryTag`).

### 7.2 Color and Image Encoding

RGB companion frames follow ISO/IEC 14496-12 (ISO Base Media File Format) for streaming and ISO/IEC 10918-1 (JPEG) or ISO/IEC 23008-12 (HEIF) for stills. Color spaces are reported via ICC v4 profiles per ISO 15076-1:2010, with the default working space sRGB IEC 61966-2-1:1999.

### 7.3 Time Synchronization

Frame timestamps are encoded per ISO 8601:2019 with nanosecond precision when sensor capability allows. Networked synchronization uses IEEE 1588-2019 PTPv2 (Precision Time Protocol) for hardware-synchronized arrays and RFC 5905 NTPv4 for software-synchronized clusters. Sub-microsecond synchronization between sensors in a single rack requires PTPv2 with hardware timestamping.

### 7.4 Numeric Encoding

Floating-point numbers conform to IEEE 754-2019 (binary32 and binary64). Integer types follow ISO/IEC 9899:2018 (C18) `<stdint.h>` fixed-width semantics: `uint16_t` for raw depth millimeters, `uint8_t` for confidence, `int32_t` for indexed point cloud fields.

### 7.5 Compression Profiles

| Use case | Codec | Reference |
|----------|-------|-----------|
| Lossless depth (archival) | LZ4 / Zstandard | RFC 8478 (Zstandard) |
| Lossy depth streaming | Custom delta + range coder | ISO/IEC 23090-5 (V-PCC) for compatibility |
| Point cloud archival | Draco or PLY | OGC v1.0 PLY conformance |
| Mesh archival | glTF 2.0 with KHR_draco_mesh_compression | Khronos glTF 2.0 specification |

Implementations declare their codec choice in the file header `compression.profile` field. Decoders MUST reject unknown profile identifiers and SHOULD report the unrecognized profile in their error response (RFC 9457 *Problem Details for HTTP APIs*).

### 7.6 Conformance

A producer is conformant with this Phase 1 specification when:

1. All mandatory header fields are present and validate against the v1.0 JSON Schema.
2. Numeric encodings match the declarations in §7.4.
3. Compression profile is one of the listed identifiers in §7.5.
4. Geographic metadata, when present, validates against ISO 19115-1:2014.
5. Time fields conform to ISO 8601:2019.

A consumer is conformant when it correctly decodes any v1.0 producer output and gracefully reports unrecognized optional fields per §7.5 last paragraph.

---

**© 2025 SmileStory Inc. / WIA - World Certification Industry Association**  
**弘益人間 · Benefit All Humanity**
