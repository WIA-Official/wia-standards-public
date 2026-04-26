# 🏭 WIA-IND-024: Manufacturing Automation Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **Standard ID:** WIA-IND-024
> **Version:** 1.0.0
> **Status:** Active
> **Category:** IND / Industry
> **Color:** Amber (#F59E0B)

---

## 🌟 Overview

The WIA-IND-024 standard defines the comprehensive framework for manufacturing automation systems, including production line automation, PLC/SCADA integration, robot arm control, conveyor systems, assembly automation, quality inspection, material handling, production scheduling, OEE monitoring, and safety interlocks.

**홍익인간 (弘益人間) - Benefit All Humanity** - This standard aims to democratize advanced manufacturing automation, enabling factories of all sizes to achieve higher productivity, quality, and safety while reducing costs and environmental impact.

## 🎯 Key Features

- **Production Line Automation**: Complete line control and orchestration
- **PLC/SCADA Integration**: Standard interfaces for industrial controllers
- **Robot Arm Control**: Universal robot programming and coordination
- **Conveyor Systems**: Material transport automation and tracking
- **Assembly Automation**: Automated assembly process management
- **Quality Inspection**: AI-powered vision and sensor-based inspection
- **Material Handling**: Warehouse and logistics automation
- **Production Scheduling**: AI-optimized production planning
- **OEE Monitoring**: Real-time Overall Equipment Effectiveness tracking
- **Safety Interlocks**: Comprehensive safety system integration

## 📊 Core Concepts

### 1. Production Line Architecture

```
Manufacturing Line Components:
- Workstations: Individual processing units
- Transport: Conveyors, AGVs, robot arms
- Inspection: Vision systems, sensors, gauges
- Storage: Buffers, warehouses, inventory
- Control: PLC, SCADA, MES, ERP integration
```

### 2. Automation Levels

| Level | Name | Description | Automation % |
|-------|------|-------------|--------------|
| 0 | Manual | Human-operated processes | 0-10% |
| 1 | Semi-Automated | Automated with manual intervention | 10-40% |
| 2 | Automated | Fully automated with supervision | 40-70% |
| 3 | Advanced | AI-optimized with predictive maintenance | 70-90% |
| 4 | Lights-Out | Fully autonomous 24/7 operation | 90-100% |

### 3. OEE Metrics

```
OEE = Availability × Performance × Quality

Where:
- Availability = Operating Time / Planned Production Time
- Performance = (Actual Output / Target Output) × 100%
- Quality = Good Units / Total Units Produced
```

## 🔧 Components

### TypeScript SDK

```typescript
import {
  ManufacturingSDK,
  ProductionLine,
  RobotArm,
  PLCController,
  QualityInspection,
  calculateOEE
} from '@wia/ind-024';

// Initialize manufacturing SDK
const mfg = new ManufacturingSDK({
  facilityId: 'FAC-001',
  plcType: 'siemens-s7-1500',
  scadaEndpoint: 'opc.tcp://192.168.1.100:4840'
});

// Configure production line
const line = await mfg.createProductionLine({
  lineId: 'LINE-A',
  workstations: [
    { id: 'WS-01', type: 'assembly', robotArms: 2 },
    { id: 'WS-02', type: 'welding', robotArms: 1 },
    { id: 'WS-03', type: 'inspection', cameras: 4 },
    { id: 'WS-04', type: 'packaging', robotArms: 1 }
  ],
  conveyorSpeed: 0.5, // m/s
  cycleTime: 45 // seconds
});

// Control robot arm
const robot = await mfg.getRobotArm('ROBOT-A1');
await robot.moveTo({ x: 500, y: 300, z: 200 });
await robot.pickPart({ partId: 'PART-12345', gripForce: 50 });
await robot.placePart({ x: 600, y: 400, z: 150 });

// Quality inspection
const inspection = await mfg.qualityInspection({
  workstationId: 'WS-03',
  partId: 'PART-12345',
  inspectionType: 'vision',
  criteria: {
    dimensions: { tolerance: 0.1 }, // mm
    surfaceDefects: { maxCount: 0 },
    colorMatch: { tolerance: 5 } // Delta E
  }
});

console.log('Inspection result:', inspection.passed);

// Calculate OEE
const oee = calculateOEE({
  plannedProductionTime: 8 * 3600, // 8 hours in seconds
  downtime: 1800, // 30 minutes
  targetOutput: 640, // units
  actualOutput: 590,
  goodUnits: 575
});

console.log('OEE:', oee.overall.toFixed(2) + '%');
console.log('Availability:', oee.availability.toFixed(2) + '%');
console.log('Performance:', oee.performance.toFixed(2) + '%');
console.log('Quality:', oee.quality.toFixed(2) + '%');
```

### CLI Tool

```bash
# Monitor production line
wia-ind-024 monitor --line LINE-A --interval 5

# Control robot arm
wia-ind-024 robot move --id ROBOT-A1 --x 500 --y 300 --z 200
wia-ind-024 robot pick --id ROBOT-A1 --part PART-12345 --force 50
wia-ind-024 robot place --id ROBOT-A1 --x 600 --y 400 --z 150

# Start production
wia-ind-024 production start --line LINE-A --target 1000

# Quality inspection
wia-ind-024 inspect --station WS-03 --part PART-12345 --type vision

# Calculate OEE
wia-ind-024 oee --line LINE-A --shift day --date 2025-12-27

# PLC integration
wia-ind-024 plc connect --type siemens --host 192.168.1.100
wia-ind-024 plc read --address DB1.DBW0
wia-ind-024 plc write --address DB1.DBW0 --value 1234

# Safety check
wia-ind-024 safety check --line LINE-A
wia-ind-024 safety estop --line LINE-A --reason "maintenance"
```

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [WIA-IND-024-v1.0.md](./spec/WIA-IND-024-v1.0.md) | Complete manufacturing automation specification |
| [TypeScript SDK](./api/typescript/) | Full TypeScript implementation |
| [CLI Tool](./cli/wia-ind-024.sh) | Command-line interface |

## 🚀 Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/manufacturing-automation

# Run installation script
./install.sh

# Verify installation
wia-ind-024 --version
```

### TypeScript Usage

```bash
# Install via npm
npm install @wia/ind-024

# Or yarn
yarn add @wia/ind-024
```

```typescript
import { ManufacturingSDK } from '@wia/ind-024';

const sdk = new ManufacturingSDK({
  facilityId: 'FAC-001',
  plcType: 'siemens-s7-1500'
});

// Monitor production line
const status = await sdk.getLineStatus('LINE-A');
console.log('Line status:', status.state);
console.log('Current output:', status.currentOutput);
console.log('Target output:', status.targetOutput);
console.log('Efficiency:', status.efficiency + '%');

// Emergency stop
if (status.alerts.length > 0) {
  await sdk.emergencyStop('LINE-A', 'Safety alert detected');
}
```

## 🔬 Technical Specifications

### Production Line Metrics

| Metric | Target | World-Class |
|--------|--------|-------------|
| OEE | >80% | >85% |
| Availability | >90% | >95% |
| Performance | >90% | >95% |
| Quality | >95% | >99% |
| Cycle Time Variance | <5% | <2% |
| Mean Time Between Failure | >200h | >500h |
| Mean Time To Repair | <30min | <15min |

### Robot Specifications

| Parameter | Range | Precision |
|-----------|-------|-----------|
| Reach | 500-2000 mm | ±0.1 mm |
| Payload | 3-25 kg | - |
| Repeatability | ±0.05 mm | - |
| Max Speed | 1-3 m/s | - |
| Axes | 6-7 DOF | - |
| Cycle Time | 1-5 s | ±0.01 s |

### Conveyor Systems

- **Speed Range**: 0.1 - 2.0 m/s
- **Load Capacity**: 10 - 500 kg/m
- **Belt Width**: 200 - 1200 mm
- **Tracking Accuracy**: ±1 mm
- **Position Control**: RFID, barcode, vision

### Quality Inspection

- **Vision Resolution**: 1-20 megapixels
- **Measurement Accuracy**: ±0.01 mm (optical)
- **Inspection Speed**: 100-500 parts/hour
- **Defect Detection**: >99% accuracy
- **False Positive Rate**: <1%

## 🏭 Use Cases

### 1. Automotive Manufacturing

- **Engine Assembly**: Automated engine block assembly with 12 robots
- **Body Welding**: 100+ spot welding robots with vision guidance
- **Paint Shop**: Automated paint application with quality inspection
- **Final Assembly**: Mixed-model assembly line with AGV material delivery

### 2. Electronics Assembly

- **PCB Manufacturing**: Pick-and-place machines with vision alignment
- **Component Insertion**: High-speed automatic insertion (>50k/hour)
- **Soldering**: Automated wave/reflow soldering with inspection
- **Testing**: Automated electrical testing and programming

### 3. Food & Beverage

- **Packaging Lines**: High-speed filling, capping, labeling
- **Quality Control**: X-ray, metal detection, weight verification
- **Palletizing**: Robotic palletizing with pattern optimization
- **Warehousing**: AS/RS (Automated Storage and Retrieval Systems)

### 4. Pharmaceutical

- **Tablet Production**: Automated mixing, pressing, coating
- **Sterile Filling**: Cleanroom automation with contamination control
- **Packaging**: Serialization and track-and-trace compliance
- **Quality Assurance**: 100% inspection with vision systems

### 5. Aerospace

- **Composite Layup**: Automated fiber placement robots
- **Machining**: 5-axis CNC with robotic loading/unloading
- **Assembly**: Large-part handling with precision alignment
- **Inspection**: 3D scanning and NDT automation

## 🛡️ Safety Features

### Safety Interlocks

```typescript
// Safety zone configuration
const safetyZones = [
  {
    zone: 'ZONE-1',
    type: 'restricted',
    sensors: ['light-curtain', 'safety-mat', 'e-stop'],
    action: 'immediate-stop',
    resetRequired: true
  },
  {
    zone: 'ZONE-2',
    type: 'collaborative',
    sensors: ['force-torque', 'vision'],
    action: 'speed-reduction',
    maxSpeed: 0.25 // m/s
  }
];

// Emergency stop
await sdk.configureSafety({
  emergencyStops: 12,
  safetyRelays: 'dual-channel',
  responseTime: 50, // milliseconds
  category: 'PLd-SIL2' // EN ISO 13849-1
});
```

### Safety Standards Compliance

- **ISO 10218**: Robots and robotic devices - Safety requirements
- **ISO/TS 15066**: Collaborative robots safety
- **ISO 13849-1**: Safety of machinery - Safety-related control systems
- **IEC 61508**: Functional safety of electrical systems
- **ANSI/RIA R15.06**: Industrial robot safety

## 📈 Production Scheduling

```typescript
// AI-optimized production scheduling
const schedule = await sdk.optimizeSchedule({
  orders: [
    { id: 'ORD-001', product: 'PROD-A', quantity: 500, dueDate: '2025-12-30' },
    { id: 'ORD-002', product: 'PROD-B', quantity: 300, dueDate: '2025-12-28' },
    { id: 'ORD-003', product: 'PROD-A', quantity: 200, dueDate: '2025-12-29' }
  ],
  constraints: {
    setupTime: { 'PROD-A': 30, 'PROD-B': 45 }, // minutes
    minBatchSize: 50,
    maxOvertimeHours: 4
  },
  objectives: {
    minimizeSetups: 0.3,
    minimizeLateness: 0.5,
    maximizeOEE: 0.2
  }
});

console.log('Optimized schedule:', schedule.timeline);
console.log('Expected completion:', schedule.completionDate);
console.log('Expected OEE:', schedule.estimatedOEE + '%');
```

## 🔄 WIA Integration

This standard integrates with:
- **WIA-INTENT**: Intent-based manufacturing control
- **WIA-OMNI-API**: Universal manufacturing API gateway
- **WIA-IOT**: Industrial IoT device integration
- **WIA-AI**: AI-powered optimization and predictive maintenance
- **WIA-DIGITAL-TWIN**: Virtual factory simulation
- **WIA-BLOCKCHAIN**: Supply chain traceability
- **WIA-ENERGY**: Energy efficiency monitoring

## ⚙️ Deployment Considerations

1. **Network Infrastructure**: Industrial Ethernet, TSN (Time-Sensitive Networking)
2. **Real-Time Requirements**: Deterministic communication for safety-critical systems
3. **Cybersecurity**: Industrial firewall, network segmentation, OT security
4. **Legacy Integration**: Gateway for older PLC/SCADA systems
5. **Scalability**: Modular architecture for factory expansion
6. **Maintenance**: Predictive maintenance with IoT sensors
7. **Training**: Operator and maintenance personnel certification

## 🤝 Contributing

Contributions welcome! Please see our [Contributing Guide](https://github.com/WIA-Official/wia-standards/CONTRIBUTING.md).

## 📄 License

MIT License - see [LICENSE](https://github.com/WIA-Official/wia-standards/LICENSE)

## 🔗 Links

- **Website**: [wiastandards.com](https://wiastandards.com)
- **GitHub**: [github.com/WIA-Official/wia-standards](https://github.com/WIA-Official/wia-standards)
- **Documentation**: [docs.wiastandards.com](https://docs.wiastandards.com)
- **Certification**: [cert.wiastandards.com](https://cert.wiastandards.com)

---

**홍익인간 (弘益人間) · Benefit All Humanity**

*WIA - World Certification Industry Association*
*© 2025 SmileStory Inc. / WIA*
*MIT License*

---

**홍익인간 (弘益人間) - Benefit All Humanity** 🌍
