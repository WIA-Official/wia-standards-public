# WIA-SOC-011: Gas Supply Standard 🔥

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)](https://github.com/WIA-Official/wia-standards)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![Standard](https://img.shields.io/badge/WIA-SOC--011-purple.svg)](https://wiastandards.com)

> **홍익인간 (弘益人間)** - Benefit All Humanity

Comprehensive standard for natural gas distribution, pipeline safety, LNG terminals, gas metering, leak detection, and hydrogen blending in modern energy infrastructure.

---

## 📋 Overview

WIA-SOC-011 establishes a complete framework for gas supply systems including:

- 🔄 **Data Format Standardization**: JSON schemas for all gas infrastructure data
- 🔌 **API Interfaces**: RESTful APIs for system integration
- 📡 **Communication Protocols**: SCADA, IoT, and streaming protocols
- 🔗 **System Integration**: Inter-operator coordination and WIA ecosystem integration
- 💚 **Hydrogen Ready**: Support for hydrogen blending and renewable gases
- 🔒 **Cybersecurity**: Comprehensive security requirements
- 🌍 **Global Interoperability**: Cross-border data exchange

---

## 🚀 Quick Start

### Installation

```bash
npm install @wia/gas-supply-sdk
```

### Basic Usage

```typescript
import { createClient } from '@wia/gas-supply-sdk';

const client = createClient({
  baseUrl: 'https://api.gas-operator.com/wia-soc-011/v1',
  apiKey: 'your-api-key'
});

// Get pipeline information
const pipelines = await client.listPipelines({
  type: 'transmission',
  status: 'active'
});

// Get real-time measurements
const measurements = await client.getRealtimeMeasurements({
  assetIds: ['PL-KR-001-2025'],
  types: ['pressure', 'flow']
});

// Subscribe to real-time updates
client.connectWebSocket(
  {
    channels: [
      { type: 'measurements', assetIds: ['PL-KR-001-2025'] },
      { type: 'alarms', severity: ['high', 'critical'] }
    ]
  },
  (data) => console.log('Update:', data)
);
```

---

## 📚 Documentation

### Specifications

- **[Phase 1: Data Format](spec/PHASE-1-DATA-FORMAT.md)** - JSON schemas and data structures
- **[Phase 2: API Interface](spec/PHASE-2-API.md)** - RESTful API specifications
- **[Phase 3: Communication Protocols](spec/PHASE-3-PROTOCOL.md)** - SCADA, IoT, and security protocols
- **[Phase 4: WIA Integration](spec/PHASE-4-INTEGRATION.md)** - System integration patterns

### Interactive Resources

- **[🎮 Live Simulator](simulator/)** - Interactive gas supply system simulator
- **[📖 English Ebook](ebook/en/)** - Complete technical guide (8 chapters)
- **[📗 Korean Ebook](ebook/ko/)** - 한국어 기술 가이드 (8개 챕터)

---

## 🏗️ Architecture

### Four-Phase Implementation

```
┌─────────────────────────────────────────────────────┐
│ PHASE 1: Data Format                                │
│ • JSON schemas for pipelines, measurements, events  │
│ • Gas composition and quality data                  │
│ • GeoJSON spatial integration                       │
└─────────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────────┐
│ PHASE 2: API Interface                              │
│ • RESTful endpoints for CRUD operations             │
│ • Real-time data streaming (WebSocket)              │
│ • Equipment control commands                        │
└─────────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────────┐
│ PHASE 3: Communication Protocols                    │
│ • SCADA integration (Modbus, DNP3, OPC UA)          │
│ • IoT protocols (MQTT, CoAP, LoRaWAN)               │
│ • TLS 1.3 encryption and security                   │
└─────────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────────┐
│ PHASE 4: WIA Integration                            │
│ • Inter-operator pipeline coordination              │
│ • LNG terminal integration                          │
│ • Smart meter networks (AMI)                        │
│ • Cross-standard integration (Energy, Water, City)  │
└─────────────────────────────────────────────────────┘
```

---

## 🔥 Key Features

### Natural Gas Distribution

- High-pressure transmission pipelines (40-100 bar)
- Local distribution networks
- Compressor station control
- Pressure regulation and metering

### LNG Terminal Operations

- Ship scheduling and berth management
- Regasification rate control
- Storage tank management
- Pipeline network integration

### Gas Quality Management

- Molecular composition tracking
- Heating value and Wobbe index calculation
- Contaminant monitoring (H2S, moisture)
- Quality specifications enforcement

### Leak Detection

- Continuous monitoring systems
- Acoustic leak detection
- Satellite methane surveillance
- Automated repair notifications

### Hydrogen Blending

- Real-time composition monitoring
- Injection rate control (up to 20% H2)
- Infrastructure compatibility assessment
- End-use equipment tolerance tracking

### Safety Systems

- Automatic shut-off valves
- Emergency response coordination
- Multi-agency integration
- Evacuation zone management

---

## 🌐 Global Standards Integration

### Complementary Standards

- **API 5L**: Pipeline materials
- **ASME B31.8**: Gas transmission and distribution piping systems
- **ISO 20765**: Natural gas measurement
- **IEC 62443**: Industrial automation security
- **OPC UA**: Industrial communications
- **ISO/TR 15916**: Hydrogen safety

### WIA Ecosystem

```
WIA-SOC-011 (Gas Supply)
    ├─→ WIA-ENE-001 (Energy Grid) - Power generation integration
    ├─→ WIA-SOC-010 (Water Supply) - LNG cooling water
    ├─→ WIA-CITY-001 (Smart City) - Infrastructure coordination
    ├─→ WIA-SEC-001 (Security) - Cybersecurity framework
    └─→ WIA-ENV-001 (Environment) - Emissions monitoring
```

---

## 💻 API Examples

### Get Pipeline Details

```typescript
const pipeline = await client.getPipeline('PL-KR-001-2025');

console.log(`Pipeline: ${pipeline.pipelineId}`);
console.log(`Type: ${pipeline.pipelineType}`);
console.log(`Material: ${pipeline.material.grade}`);
console.log(`MAOP: ${pipeline.pressureRating.maop_bar} bar`);
```

### Query Historical Data

```typescript
const history = await client.getHistoricalData({
  assetId: 'PL-KR-001-2025',
  type: 'pressure',
  start: '2025-12-01T00:00:00Z',
  end: '2025-12-26T23:59:59Z',
  interval: '1hour',
  aggregation: 'avg'
});

console.log(`Retrieved ${history.data.length} data points`);
```

### Control Compressor

```typescript
const response = await client.controlCompressor('COMP-001', {
  command: 'adjust_speed',
  parameters: {
    targetSpeed_rpm: 3600,
    rampRate_rpm_min: 100
  },
  authorization: {
    approvedBy: 'operator-123',
    reason: 'Increase pipeline pressure'
  }
});

console.log(`Command ${response.commandId}: ${response.status}`);
```

### Monitor Alarms

```typescript
client.connectWebSocket(
  {
    channels: [{
      type: 'alarms',
      severity: ['high', 'critical']
    }]
  },
  async (alarm) => {
    console.log(`ALARM: ${alarm.description}`);

    // Auto-acknowledge critical alarms
    if (alarm.severity === 'critical') {
      await client.acknowledgeEvent(
        alarm.eventId,
        'auto-system',
        'Auto-acknowledged by monitoring system'
      );
    }
  }
);
```

---

## 🔒 Security

### Authentication

- OAuth 2.0 Bearer tokens
- API key authentication
- Mutual TLS for machine-to-machine
- Multi-factor authentication support

### Encryption

- TLS 1.3 for all communications
- AES-256-GCM cipher suites
- Ed25519 message signing
- Certificate pinning

### Access Control

- Role-based access control (RBAC)
- Granular permissions
- Audit logging
- Session management

---

## 📊 Data Flow

```
┌─────────────────┐
│  Field Sensors  │ (Pressure, Flow, Temperature, Leak Detection)
└────────┬────────┘
         │ SCADA/IoT Protocols
         ↓
┌─────────────────┐
│ SCADA/RTU/PLC   │ (Data Acquisition, Local Control)
└────────┬────────┘
         │ Secure Network
         ↓
┌─────────────────┐
│  WIA-SOC-011    │ (Standard Data Format, API Gateway)
│   Gateway       │
└────────┬────────┘
         │ RESTful API / WebSocket
         ├────────────────────────────┬──────────────────┐
         ↓                            ↓                  ↓
┌─────────────────┐      ┌─────────────────┐  ┌─────────────────┐
│  Operations     │      │   Analytics     │  │   Regulatory    │
│  Dashboard      │      │   Platform      │  │   Reporting     │
└─────────────────┘      └─────────────────┘  └─────────────────┘
```

---

## 🌍 Use Cases

### 1. Cross-Border Pipeline Operations

Multiple operators manage interconnected pipelines across national borders:

- **Automated nominations** and confirmations
- **Real-time flow balancing** at interconnection points
- **Unified quality specifications**
- **Coordinated emergency response**

### 2. LNG Import Terminal Integration

LNG terminals connect global gas markets to regional pipeline networks:

- **Ship arrival scheduling** coordinated with pipeline capacity
- **Regasification rate** optimization based on demand forecasts
- **Storage management** across multiple tanks
- **Send-out coordination** to multiple pipeline systems

### 3. Smart City Gas Distribution

Modern cities integrate gas infrastructure with other utilities:

- **Coordinated excavation** planning with water, electric, telecom
- **Shared emergency response** with city services
- **Digital twin** integration for infrastructure modeling
- **Demand-side management** coordinated with pricing

### 4. Hydrogen Blending Pilot Programs

Transitioning to renewable gases while maintaining reliability:

- **Real-time composition** monitoring and control
- **Infrastructure compatibility** assessment
- **End-user equipment** tolerance tracking
- **Gradual blend ratio** increases with safety validation

---

## 📈 Benefits

### Economic

- **Reduced integration costs** - Standard interfaces eliminate custom development
- **Vendor competition** - Interoperability prevents vendor lock-in
- **Operational efficiency** - Automated data exchange and optimization
- **Faster deployment** - Reusable components and proven patterns

### Environmental

- **Methane reduction** - Better leak detection and monitoring
- **Renewable integration** - Hydrogen and biomethane support
- **Optimized operations** - Reduced energy consumption
- **Emissions tracking** - Comprehensive environmental reporting

### Safety

- **Proactive monitoring** - Early anomaly detection
- **Coordinated response** - Multi-agency integration
- **Audit compliance** - Automated regulatory reporting
- **Best practices** - Standardized safety protocols

---

## 🛠️ Implementation Guide

### Step 1: Assessment (Month 1-2)

- Evaluate current systems and identify gaps
- Define implementation scope and priorities
- Establish project team and governance

### Step 2: Pilot (Month 3-6)

- Implement Phase 1 (Data Format) on limited scope
- Validate data schemas with existing systems
- Train personnel on new standards

### Step 3: Rollout (Month 7-18)

- Deploy Phases 2-3 (API & Protocols) incrementally
- Integrate with existing SCADA and business systems
- Conduct interoperability testing

### Step 4: Integration (Month 19-24)

- Implement Phase 4 (WIA Integration)
- Connect with partner systems
- Achieve WIA certification

### Step 5: Optimization (Ongoing)

- Continuous monitoring and improvement
- Regular standard updates
- Community engagement and feedback

---

## 🏆 Certification

### Equipment Certification

Verify individual components meet standard requirements:

- Gas meters and measurement devices
- SCADA controllers and RTUs
- IoT sensors and gateways
- Safety and control systems

### System Certification

Confirm integrated systems implement standard interfaces:

- Pipeline management systems
- SCADA platforms
- Meter data management (MDM)
- Geographic information systems (GIS)

### Operator Certification

Recognize organizations implementing the standard:

- Data format compliance
- API implementation
- Security requirements
- Interoperability demonstration

**Certification Authority**: World Certification Industry Association
**Contact**: certification@wiastandards.com

---

## 👥 Contributing

We welcome contributions from the global gas industry community!

### How to Contribute

1. **Issues**: Report bugs or request features on GitHub
2. **Pull Requests**: Submit improvements to documentation or specs
3. **Working Groups**: Join technical committees
4. **Case Studies**: Share implementation experiences

### Governance

- **Technical Committee**: Reviews and approves standard changes
- **Public Comment**: 60-day review period for major revisions
- **Consensus Process**: All stakeholders have input
- **Transparent Development**: All meetings and decisions documented

---

## 📞 Support

- **Documentation**: https://wiastandards.com/docs/soc-011
- **API Reference**: https://api.wiastandards.com/soc-011/docs
- **Community Forum**: https://community.wiastandards.com
- **Email**: support@wiastandards.com
- **GitHub Issues**: https://github.com/WIA-Official/wia-standards/issues

---

## 📄 License

This standard is released under the **MIT License**, ensuring:

- ✅ Free commercial use
- ✅ Modification and derivative works
- ✅ Private use
- ✅ Distribution

See [LICENSE](LICENSE) file for full terms.

---

## 🙏 Acknowledgments

WIA-SOC-011 was developed with contributions from:

- **Gas transmission operators** worldwide
- **LNG terminal operators** and ship owners
- **Distribution companies** and utilities
- **Equipment manufacturers** and technology vendors
- **Regulatory agencies** and standards organizations
- **Academic researchers** and industry experts

Special thanks to the global gas industry for their expertise and commitment to open standards.

---

## 📚 References

1. **API Standard 5L** - Specification for Line Pipe
2. **ASME B31.8** - Gas Transmission and Distribution Piping Systems
3. **ISO 20765** - Natural Gas - Calculation of thermodynamic properties
4. **IEC 62443** - Industrial communication networks - Network and system security
5. **OPC UA Specification** - Unified Architecture
6. **ISO/TR 15916** - Basic considerations for the safety of hydrogen systems

---

<div align="center">

## 홍익인간 (弘益人間) - Benefit All Humanity

**WIA - World Certification Industry Association**

© 2025 MIT License

[Website](https://wiastandards.com) • [GitHub](https://github.com/WIA-Official/wia-standards) • [Certification](https://cert.wiastandards.com)

</div>
