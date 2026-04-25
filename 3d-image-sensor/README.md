# WIA-SEMI-013: 3D Image Sensor Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


**Version:** 1.0
**Status:** Published
**Date:** 2025-01-15

## Overview

WIA-SEMI-013 is a comprehensive standard for 3D image sensor technologies, encompassing Time-of-Flight (ToF), Structured Light, and Stereo Vision systems. The standard defines data formats, API interfaces, communication protocols, and integration guidelines to enable interoperability across the 3D sensing ecosystem.

### Philosophy

**홍익인간 (弘益人間)** - *Broadly Benefiting Humanity*

This standard aims to accelerate 3D sensing technology adoption by providing a unified framework that manufacturers, developers, and end-users can rely on for consistent, high-quality depth sensing capabilities.

## Quick Start

### Web Interface

Visit the interactive simulator:
```
open index.html
```

Or access the online simulator at: [wiabooks.store/tag/wia-3d-image-sensor](https://wiabooks.store/tag/wia-3d-image-sensor/)

### TypeScript SDK

```bash
npm install @wia/semi-013-3d-image-sensor
```

```typescript
import { WIA3DImageSensor } from '@wia/semi-013-3d-image-sensor';

const sensor = new WIA3DImageSensor({
  deviceId: 'sensor-001',
  resolution: { width: 640, height: 480 },
  frameRate: 30
});

await sensor.start();

sensor.on('depth-frame', (frame) => {
  console.log(`Frame ${frame.sequenceNumber} captured`);
});
```

## Directory Structure

```
3d-image-sensor/
├── index.html                  # Landing page with dark theme
├── simulator/
│   └── index.html             # Interactive 5-tab simulator (99 languages)
├── ebook/
│   ├── en/                    # English documentation (9 chapters)
│   │   ├── index.html
│   │   ├── chapter-01.html    # Introduction to 3D Sensing
│   │   ├── chapter-02.html    # ToF Technology
│   │   ├── chapter-03.html    # Structured Light
│   │   ├── chapter-04.html    # Stereo Vision
│   │   ├── chapter-05.html    # Depth Accuracy
│   │   ├── chapter-06.html    # Point Cloud Processing
│   │   ├── chapter-07.html    # Applications
│   │   └── chapter-08.html    # Standard Specification
│   └── ko/                    # Korean documentation (9 chapters)
│       └── [same structure as en/]
├── spec/                      # Technical specifications
│   ├── PHASE-1-DATA-FORMAT.md
│   ├── PHASE-2-API-INTERFACE.md
│   ├── PHASE-3-PROTOCOL.md
│   └── PHASE-4-INTEGRATION.md
├── api/
│   └── typescript/            # TypeScript SDK
│       ├── package.json
│       └── src/
│           ├── types.ts
│           └── index.ts
└── README.md                  # This file
```

## Technology Coverage

### Time-of-Flight (ToF)

- **iToF (Indirect ToF)**: Continuous-wave modulation, phase measurement
- **dToF (Direct ToF)**: SPAD-based, single-photon sensitivity, histogram processing

**Key Players:**
- Sony DepthSense IMX series
- Samsung ISOCELL Vizion
- Infineon REAL3
- STMicroelectronics FlightSense

### Structured Light

- Pattern projection (dots, coded patterns)
- High-precision depth mapping (sub-millimeter)
- Face ID and biometric authentication

**Key Players:**
- Apple TrueDepth (Face ID)
- Orbbec Astra
- Intel RealSense (discontinued)

### Stereo Vision

- Passive and active stereo
- Wide FOV, outdoor capable
- Cost-effective for many applications

**Key Players:**
- Intel RealSense D400 series
- ZED cameras
- OAK-D

## Four-Phase Architecture

### Phase 1: Data Format

Standardized formats for:
- Sensor specifications (JSON schema)
- Depth maps (binary + CSV)
- Point clouds (PCD, PLY, XYZ, JSON)
- Calibration data (intrinsic, extrinsic, temperature compensation)

**Documentation:** [spec/PHASE-1-DATA-FORMAT.md](spec/PHASE-1-DATA-FORMAT.md)

### Phase 2: API Interface

RESTful APIs and SDKs for:
- Device discovery and management
- Configuration control
- Data streaming (WebSocket, binary)
- Processing functions
- Calibration access

**Supported Languages:**
- TypeScript/JavaScript
- Python
- C++
- ROS/ROS2

**Documentation:** [spec/PHASE-2-API-INTERFACE.md](spec/PHASE-2-API-INTERFACE.md)

### Phase 3: Protocol

Communication protocols for:
- Real-time point cloud streaming (WIA-PC-Stream)
- Depth data streaming (WIA-Depth-Stream)
- Multi-sensor synchronization (hardware and software)
- Security (authentication, encryption)
- Error handling and recovery

**Documentation:** [spec/PHASE-3-PROTOCOL.md](spec/PHASE-3-PROTOCOL.md)

### Phase 4: Integration

Guidelines and reference implementations for:
- Multi-sensor fusion (homogeneous and heterogeneous)
- SLAM integration (Visual-Inertial Odometry)
- Application templates (face auth, gesture recognition, navigation)
- Testing and certification procedures

**Documentation:** [spec/PHASE-4-INTEGRATION.md](spec/PHASE-4-INTEGRATION.md)

## Certification Levels

| Parameter | Bronze | Silver | Gold |
|-----------|--------|--------|------|
| **Depth Accuracy** | ±5mm @ 2m | ±2mm @ 2m | ±1mm @ 2m |
| **Resolution** | 320x240 | 640x480 | 1280x960 |
| **Frame Rate** | 15 fps | 30 fps | 60 fps |
| **Operating Range** | 0.5-5m | 0.2-8m | 0.1-10m |
| **Ambient Light** | Indoor | Indoor + Outdoor | Full Sunlight |
| **Multi-path** | Basic | Advanced | Expert + AI |
| **Temperature** | 0-40°C | -20-60°C | -40-85°C |

## Applications

### Consumer Electronics
- Face ID authentication
- Portrait mode photography
- AR/VR experiences
- Gesture control

### Automotive
- Driver monitoring (drowsiness, attention)
- Occupancy detection
- Gesture-based infotainment
- In-cabin safety systems

### Robotics
- Obstacle detection and avoidance
- 3D mapping and SLAM
- Object grasping and manipulation
- Human-robot collaboration

### Industrial
- Dimensional inspection
- Surface defect detection
- Assembly verification
- Quality control

### Healthcare
- 3D body scanning
- Orthotic/prosthetic fitting
- Surgical planning
- Contactless vital signs monitoring

## Performance Specifications

### Typical Performance Ranges

**ToF Sensors:**
- Accuracy: ±5-15mm @ 2m (iToF), ±1-5mm @ 2m (dToF)
- Range: 0.2-10m
- Frame Rate: 30-120 fps
- Resolution: VGA to QVGA

**Structured Light:**
- Accuracy: <1mm @ 0.5m
- Range: 0.2-2m
- Frame Rate: 15-60 fps
- Resolution: VGA to HD

**Stereo Vision:**
- Accuracy: ±2% of distance
- Range: 0.3-10m+ (baseline dependent)
- Frame Rate: 30-90 fps
- Resolution: VGA to HD

## SDK Examples

### TypeScript

```typescript
import {
  WIA3DImageSensor,
  convertToPointCloud,
  exportToPCD
} from '@wia/semi-013-3d-image-sensor';

const sensor = new WIA3DImageSensor({
  deviceId: 'sensor-001',
  resolution: { width: 640, height: 480 },
  frameRate: 30
});

await sensor.start();

sensor.on('depth-frame', async (frame) => {
  const calibration = sensor.getCalibration();

  const pointCloud = convertToPointCloud(frame, calibration, {
    removeInvalid: true,
    voxelSize: 0.01
  });

  const pcdData = exportToPCD(pointCloud);
  console.log(`Exported ${pointCloud.points.length} points`);
});
```

### Python

```python
from wia_semi_013 import WIA3DImageSensor

sensor = WIA3DImageSensor(
    device_id='sensor-001',
    resolution=(640, 480),
    frame_rate=30
)

sensor.start()

def on_depth_frame(frame):
    point_cloud = frame.to_point_cloud(remove_invalid=True)
    print(f"Points: {len(point_cloud.points)}")

sensor.on('depth-frame', on_depth_frame)
```

## Testing and Compliance

### Required Tests

1. **Accuracy Test:** Measure calibration plates at multiple distances
2. **Repeatability Test:** Statistical analysis of 1000 static frames
3. **Edge Accuracy:** Verify performance at depth discontinuities
4. **Multi-Path Test:** Corner retroreflector measurement
5. **Ambient Light Test:** Operation under specified illumination
6. **Temperature Test:** Full range thermal cycling

### Test Equipment

- Precision calibration plates (certified flatness)
- Temperature chamber (-40°C to +85°C)
- Light sources (up to 100,000 lux)
- Reference metrology system (±0.1mm accuracy)
- Corner retroreflector targets

## Market Data

- **Global 3D Sensor Market (2025):** $15B+
- **CAGR (2020-2025):** 45%
- **Units Shipped (2024):** 2.5B+
- **Applications:** Consumer (60%), Automotive (20%), Industrial (15%), Other (5%)

## Resources

### Documentation
- [Complete eBook (English)](ebook/en/index.html)
- [완전한 전자책 (한국어)](ebook/ko/index.html)
- [Technical Specifications](spec/)
- [Interactive Simulator](simulator/index.html)

### External Links
- [WIA Books Store](https://wiabooks.store/tag/wia-3d-image-sensor/)
- [WIA Standards Repository](https://github.com/WIA-Official/wia-standards)
- [WIA Official Website](https://wia-official.org)

### Key Manufacturers
- Sony Semiconductor Solutions
- Samsung Semiconductor
- Infineon Technologies
- STMicroelectronics
- Apple Inc.
- Intel Corporation (RealSense)

## Contributing

We welcome contributions to the WIA-SEMI-013 standard. Please submit issues and pull requests to the [WIA Standards repository](https://github.com/WIA-Official/wia-standards).

### Areas for Contribution

- Additional language support for simulator
- Reference implementations in other languages
- Test procedures and validation tools
- Application examples and case studies
- Performance benchmarks

## Version History

### Version 1.0 (2025-01-15)
- Initial release
- Core data formats defined
- RESTful API specification
- TypeScript SDK
- Certification criteria established

### Planned Updates

**Version 1.1 (Q2 2026):**
- Event-based sensor support
- Extended AI metadata
- Enhanced multi-sensor sync

**Version 2.0 (2027):**
- Neural radiance fields (NeRF)
- Compressed neural representations
- AR Cloud integration

## License

This standard is published under the MIT License by SmileStory Inc. / WIA.

Reference implementations and SDK code are open source. Hardware implementations may be subject to manufacturer-specific licensing.

## Contact

**World Certification Industry Association (WIA)**
SmileStory Inc.
Email: standards@wia-official.org
Website: https://wia-official.org

---

**홍익인간 (弘益人間) - Benefit All Humanity**

© 2025 SmileStory Inc. / WIA - World Certification Industry Association

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
