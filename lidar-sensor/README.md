# WIA-SEMI-014: LiDAR Sensor Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> 홍익인간 (弘益人間) - Benefit All Humanity

## Overview

WIA-SEMI-014 defines the comprehensive standard for automotive LiDAR (Light Detection and Ranging) sensors used in Advanced Driver Assistance Systems (ADAS) and autonomous vehicles. This standard covers mechanical, solid-state, and FMCW LiDAR architectures operating at 905nm and 1550nm wavelengths.

## Quick Links

- **📡 Landing Page:** [index.html](./index.html)
- **🧪 Simulator:** [simulator/index.html](./simulator/index.html)
- **📚 Ebook:** [https://wiabooks.store/tag/wia-lidar-sensor/](https://wiabooks.store/tag/wia-lidar-sensor/)
- **📋 Specifications:** [spec/](./spec/)
- **💻 TypeScript SDK:** [api/typescript/](./api/typescript/)

## Key Specifications

| Parameter | Specification |
|-----------|--------------|
| **Maximum Range** | 200m+ @ 10% reflectivity |
| **Range Accuracy** | ±2cm (1σ) |
| **Angular Resolution** | 0.1° - 0.4° |
| **Points per Second** | 1M+ typical |
| **Frame Rate** | 10-20 Hz |
| **Field of View** | 120°-360° (H) × 20°-40° (V) |
| **Eye Safety** | Class 1 or Class 1M (IEC 60825-1) |

## Architecture Types

### Mechanical Scanning LiDAR
- 360° surround coverage
- 64-128 laser channels
- Proven reliability (Velodyne, Ouster, Hesai)
- **Use Case:** Robotaxis, urban autonomy

### Solid-State MEMS LiDAR
- Compact form factor
- MEMS micro-mirror scanning
- Automotive-grade reliability (Innoviz, Valeo)
- **Use Case:** Production vehicles, ADAS

### FMCW LiDAR
- Simultaneous range and velocity measurement
- Interference immunity
- Long-range capability (Aeva, Luminar)
- **Use Case:** Highway autonomy, premium vehicles

### Flash LiDAR
- No moving parts
- Simultaneous frame capture
- Short-range applications
- **Use Case:** Cabin monitoring, parking assistance

## Wavelength Selection

### 905nm (Near-IR)
**Advantages:**
- Low-cost silicon detectors (APD, SPAD)
- Mature component ecosystem
- High quantum efficiency (80-90%)

**Limitations:**
- Class 1M eye safety (limited laser power)
- Higher solar background noise

**Typical Range:** 200-300m

### 1550nm (SWIR)
**Advantages:**
- True Class 1 eye safety (corneal absorption)
- 10-40× higher permissible laser power
- Reduced solar interference
- Superior long-range performance

**Limitations:**
- InGaAs detectors (higher cost)
- Smaller component ecosystem

**Typical Range:** 300-500m+

## Detector Technologies

### APD (Avalanche Photodiode)
- **Silicon APD:** 905nm, gain 10-100×, low cost
- **InGaAs APD:** 1550nm, gain 10-40×, higher cost
- Mature technology, automotive-qualified

### SPAD (Single Photon Avalanche Diode)
- Single-photon sensitivity
- Picosecond timing resolution
- Geiger-mode operation
- Applications: Flash LiDAR, photon counting

### SiPM (Silicon Photomultiplier)
- High gain (10^5-10^6)
- Analog output
- Photon-counting capability
- Applications: Long-range detection

## Directory Structure

```
lidar-sensor/
├── index.html              # Landing page with dark theme
├── simulator/
│   └── index.html          # 5-tab simulator (99 languages)
├── ebook/
│   ├── en/                 # 9 English chapters (15KB+ each)
│   │   ├── index.html
│   │   ├── chapter1.html   # Introduction to LiDAR Technology
│   │   ├── chapter2.html   # Architectures (Mechanical vs Solid-State vs FMCW)
│   │   ├── chapter3.html   # Wavelength Selection (905nm vs 1550nm)
│   │   ├── chapter4.html   # Detector Technologies (SPAD, APD, SiPM)
│   │   ├── chapter5.html   # Point Cloud Processing
│   │   ├── chapter6.html   # Automotive Integration & Safety
│   │   ├── chapter7.html   # Market Leaders (Velodyne, Luminar, Innoviz, Hesai)
│   │   └── chapter8.html   # Future Trends
│   └── ko/                 # 9 Korean chapters (15KB+ each)
│       └── [same structure]
├── spec/                   # 4 specification files (5KB+ each)
│   ├── lidar-sensor-spec-v1.0.html
│   ├── performance-testing.html
│   ├── interface-protocol-spec.html
│   └── safety-compliance-guide.html
├── api/
│   └── typescript/         # TypeScript SDK
│       ├── src/
│       │   ├── types.ts
│       │   ├── lidar-sensor.ts
│       │   ├── point-cloud-processor.ts
│       │   ├── utils.ts
│       │   └── index.ts
│       ├── package.json
│       └── tsconfig.json
└── README.md               # This file
```

## Safety and Standards Compliance

### Laser Safety (IEC 60825-1)
- Class 1: Safe under all conditions (1550nm typical)
- Class 1M: Safe for naked eye viewing (905nm typical)
- Maximum Permissible Exposure (MPE) compliance

### Functional Safety (ISO 26262)
- ASIL-B: ADAS features (ACC, AEB)
- ASIL-C: Conditional automation (Level 3)
- ASIL-D: High/Full automation (Level 4-5)

### Automotive Qualification (AEC-Q100)
- Operating temperature: -40°C to +85°C
- 1000 temperature cycles
- HTOL (High Temperature Operating Life): 1000 hours @ 150°C

### Environmental Protection
- IP67 minimum (IP68 or IP69K for harsh environments)
- Vibration resistance per ISO 16750-3
- EMC compliance (CISPR 25, ISO 11452)

## Market Leaders

### Velodyne
- Pioneer in mechanical rotating LiDAR
- Alpha Prime: 128 channels, 300m range, 360° FOV
- Extensive deployment in autonomous vehicle development

### Luminar
- 1550nm long-range LiDAR
- Hydra: 500m vehicle detection, 250m pedestrian detection
- Production contracts: Volvo, Mercedes, Nissan

### Innoviz
- MEMS solid-state LiDAR
- InnovizTwo: 120° FOV, 250m range, automotive-grade
- BMW production deployment

### Hesai
- Chinese market leader
- Pandar128: Competitive 128-channel system
- Strong presence in Asian autonomous vehicle programs

### Others
- **Ouster:** Digital LiDAR architecture
- **Valeo:** SCALA ADAS LiDAR (production since 2017)
- **Aeva:** FMCW 4D LiDAR with velocity measurement
- **RoboSense:** Solid-state and mechanical systems

## Integration Example (TypeScript)

```typescript
import { LiDARSensor, PointCloudProcessor } from '@wia/lidar-sensor';

// Initialize sensor
const sensor = new LiDARSensor({
  host: '192.168.1.201',
  dataPort: 2368
});

// Handle point cloud data
sensor.on('pointCloud', (cloud) => {
  console.log(`Frame ${cloud.sequenceNumber}: ${cloud.pointCount} points`);

  // Filter by range (1m - 100m)
  const filtered = PointCloudProcessor.filterByRange(cloud, 1.0, 100.0);

  // Downsample using voxel grid (10cm voxels)
  const downsampled = PointCloudProcessor.voxelGridDownsample(filtered, 0.1);

  // Estimate ground plane
  const groundPlane = PointCloudProcessor.estimateGroundPlane(downsampled);

  // Euclidean clustering for object detection
  const clusters = PointCloudProcessor.euclideanClustering(
    downsampled,
    0.5,   // cluster tolerance (meters)
    10,    // min cluster size
    10000  // max cluster size
  );

  console.log(`Detected ${clusters.length} object clusters`);
});

// Monitor sensor status
sensor.on('status', (status) => {
  console.log(`LiDAR Status: ${status.state}`);
  console.log(`Temperature: ${status.temperature}°C`);
  console.log(`Frame Rate: ${status.actualFrameRate} Hz`);
});

// Connect
await sensor.connect();
```

## Simulator Features

The included simulator ([simulator/index.html](./simulator/index.html)) provides:

### Tab 1: 📊 LiDAR Specifications
- Configure sensor type (Mechanical, MEMS, OPA, Flash, FMCW)
- Set wavelength (905nm, 1550nm)
- Adjust range, FOV, angular resolution
- Calculate performance metrics

### Tab 2: 🔢 Range Calculator
- Time-of-Flight (ToF) calculations
- Angular resolution analysis
- Range accuracy estimation
- Performance modeling

### Tab 3: 📡 Point Cloud Protocols
- PCL, LAS, PCD format configuration
- ROS/ROS2 integration
- Ethernet (UDP/TCP) protocols
- Data field selection

### Tab 4: 🔗 ADAS Integration
- Sensor fusion configuration
- ISO 26262 safety level selection
- Vehicle integration parameters
- ECU interface settings

### Tab 5: 🧪 Testing Suite
- Environmental testing scenarios
- Performance validation
- Interference testing
- Data quality metrics

**Languages:** 99 languages supported including English, Korean, Chinese, Japanese, Spanish, French, German, and 92 more.

## Performance in Adverse Weather

| Condition | Performance Impact |
|-----------|-------------------|
| **Clear Weather** | 100% of rated range |
| **Light Rain (10mm/h)** | ≥80% of rated range |
| **Heavy Rain (50mm/h)** | ≥50% of rated range |
| **Fog (50m visibility)** | ≥30m detection distance |
| **Direct Sunlight** | ≥90% of nighttime performance |
| **Snow** | Variable (point cloud noise <5%) |

## Future Trends

### Cost Reduction
- Target: Sub-$500 per sensor (from current $2,000-$8,000)
- Enables mass-market ADAS deployment
- Volume production and competition driving prices down

### Solid-State Transition
- MEMS and OPA replacing mechanical rotation
- Improved reliability (15-year automotive lifetime)
- Compact form factors for bumper/headlight integration

### Silicon Photonics Integration
- Single-chip LiDAR (laser, OPA, detector, processing)
- Dramatic size and cost reduction
- Companies: SiLC Technologies, others

### FMCW Commercialization
- Simultaneous range and velocity measurement
- Interference immunity in multi-LiDAR scenarios
- Higher SNR efficiency

### On-Edge AI Processing
- LiDAR outputs semantic understanding (not raw points)
- Object detection, classification, tracking on-sensor
- Reduced data bandwidth and latency

### Automotive Standardization
- SAE, ISO standards for interoperability
- Common interfaces and protocols
- Safety validation frameworks

## Resources

- **WIA Website:** [https://wia.org](https://wia.org)
- **Ebook Store:** [https://wiabooks.store/tag/wia-lidar-sensor/](https://wiabooks.store/tag/wia-lidar-sensor/)
- **GitHub Repository:** [https://github.com/WIA-Official/wia-standards](https://github.com/WIA-Official/wia-standards)
- **Technical Support:** support@wia.org

## Contributing

Contributions to the WIA-SEMI-014 standard are welcome. Please submit issues and pull requests to the GitHub repository.

## License

© 2025 SmileStory Inc. / WIA

This specification is released under the WIA Open Standard License. See LICENSE file for details.

**Philosophy:** 홍익인간 (弘益人間) - "Benefit All Humanity"

---

## Changelog

### Version 1.0 (2025-01-15)
- Initial release of WIA-SEMI-014 LiDAR Sensor Standard
- Complete specification covering mechanical, solid-state, and FMCW architectures
- TypeScript SDK v1.0
- Comprehensive ebook (English and Korean)
- Interactive simulator with 99 language support
- Safety and compliance documentation

---

**Document Version:** 1.0
**Last Updated:** 2025-01-15
**Status:** Active Standard

For questions or clarifications, please contact: standards@wia.org

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
