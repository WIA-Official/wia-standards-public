# 🔌 WIA-IND-027: Industrial IoT Standard

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)]()
[![License](https://img.shields.io/badge/license-MIT-green.svg)]()
[![Standard](https://img.shields.io/badge/WIA-Standard-purple.svg)]()


> **Standard ID:** WIA-IND-027
> **Version:** 1.0.0
> **Status:** Active
> **Category:** Industry / Manufacturing
> **Color:** Amber (#F59E0B)

---

## 🌟 Overview

The WIA-IND-027 standard defines the comprehensive framework for Industrial Internet of Things (IIoT) systems, including device protocols, edge computing, time-series data management, real-time monitoring, and integration with manufacturing execution systems. This standard enables seamless connectivity and control of industrial devices, from sensors to entire production lines.

**홍익인간 (弘益人間) - Benefit All Humanity** - This standard aims to democratize industrial automation, improve manufacturing efficiency, enhance worker safety, and accelerate the digital transformation of factories worldwide.

## 🎯 Key Features

- **IIoT Device Protocols**: OPC-UA, MQTT, Modbus, PROFINET integration
- **Edge Computing**: Real-time processing at the manufacturing edge
- **Time-Series Data**: Efficient storage and querying of sensor data
- **Real-Time Dashboards**: Live monitoring of production metrics
- **Alert & Notifications**: Proactive anomaly detection and alerting
- **Device Management**: Remote configuration and firmware updates
- **Industrial Security**: Hardened protocols for critical infrastructure
- **Data Aggregation**: Multi-source data fusion and normalization
- **MES/ERP Integration**: Seamless connection to enterprise systems
- **Digital Twin**: Virtual replicas of physical assets

## 📊 Core Concepts

### 1. Industrial IoT Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Cloud / Enterprise Layer                  │
│  [MES] [ERP] [Analytics] [Digital Twin] [AI/ML]             │
└───────────────────────┬─────────────────────────────────────┘
                        │
┌───────────────────────┴─────────────────────────────────────┐
│                    Edge Computing Layer                      │
│  [Edge Gateway] [Data Aggregation] [Local Analytics]        │
└───────────────────────┬─────────────────────────────────────┘
                        │
┌───────────────────────┴─────────────────────────────────────┐
│                    Device / Field Layer                      │
│  [PLC] [Sensors] [Actuators] [Robots] [Machines]           │
└─────────────────────────────────────────────────────────────┘
```

### 2. Protocol Integration

| Protocol | Use Case | Transport | Typical Rate |
|----------|----------|-----------|--------------|
| OPC-UA | PLC/SCADA data | TCP/IP | 1-100 Hz |
| MQTT | Sensor telemetry | TCP/IP | 0.1-10 Hz |
| Modbus | Legacy devices | RS-485/TCP | 1-10 Hz |
| PROFINET | Real-time control | Ethernet | 100-1000 Hz |
| EtherCAT | Motion control | Ethernet | 1-10 kHz |
| BACnet | Building automation | IP/MSTP | 0.1-1 Hz |

### 3. Data Flow Pipeline

```
Sensor → Edge Gateway → Time-Series DB → Analytics → Dashboard
   ↓           ↓              ↓              ↓           ↓
 Raw Data   Filtering    Aggregation    ML Models   Alerts
```

## 🔧 Components

### TypeScript SDK

```typescript
import {
  IndustrialIoTSDK,
  OPCUAClient,
  MQTTBroker,
  TimeSeriesDB,
  DigitalTwin
} from '@wia/ind-027';

// Initialize SDK
const sdk = new IndustrialIoTSDK({
  factoryId: 'factory-001',
  edgeGateway: 'edge-001',
  apiKey: 'your-api-key'
});

// Connect to OPC-UA PLC
const plc = await sdk.connectOPCUA({
  endpoint: 'opc.tcp://192.168.1.100:4840',
  securityMode: 'SignAndEncrypt',
  credentials: {
    username: 'operator',
    password: 'secure-pass'
  }
});

// Read sensor values
const temperature = await plc.readNode({
  nodeId: 'ns=2;s=Temperature_Sensor_01',
  interval: 1000 // 1 Hz
});

console.log(`Temperature: ${temperature.value}°C`);

// Subscribe to real-time data
plc.subscribe({
  nodeId: 'ns=2;s=Production_Line_Status',
  onChange: (data) => {
    console.log(`Status changed: ${data.value}`);

    if (data.value === 'ERROR') {
      sdk.sendAlert({
        severity: 'critical',
        message: 'Production line error detected',
        device: 'Line-A',
        timestamp: new Date()
      });
    }
  }
});

// MQTT sensor integration
const mqtt = await sdk.connectMQTT({
  broker: 'mqtt://broker.factory.com:1883',
  clientId: 'factory-001',
  topics: ['sensors/+/temperature', 'sensors/+/vibration']
});

mqtt.on('message', async (topic, payload) => {
  const data = JSON.parse(payload.toString());

  // Store in time-series database
  await sdk.writeTimeSeries({
    measurement: 'sensor_data',
    tags: {
      sensor_id: data.sensorId,
      location: data.location
    },
    fields: {
      value: data.value,
      unit: data.unit
    },
    timestamp: new Date(data.timestamp)
  });
});

// Query time-series data
const metrics = await sdk.queryTimeSeries({
  measurement: 'sensor_data',
  start: '2025-01-01T00:00:00Z',
  end: '2025-01-01T23:59:59Z',
  aggregation: 'mean',
  interval: '5m',
  filter: {
    sensor_id: 'temp-001'
  }
});

console.log(`Average temperature: ${metrics.mean}°C`);

// Digital Twin integration
const twin = await sdk.createDigitalTwin({
  assetId: 'machine-A-001',
  type: 'cnc-machine',
  model: 'DMG-MORI-5000',
  sensors: [
    { id: 'temp-001', type: 'temperature', location: 'spindle' },
    { id: 'vib-001', type: 'vibration', location: 'motor' },
    { id: 'power-001', type: 'power', location: 'drive' }
  ]
});

// Sync real-time data to digital twin
await twin.sync({
  liveData: true,
  updateInterval: 1000,
  predictiveMaintenance: true
});

// Run predictive analytics
const prediction = await twin.predict({
  metric: 'remaining_useful_life',
  horizon: '30d'
});

console.log(`Predicted RUL: ${prediction.value} days`);
```

### CLI Tool

```bash
# Connect to OPC-UA device
wia-ind-027 opcua connect --endpoint opc.tcp://192.168.1.100:4840 --username operator

# Read node value
wia-ind-027 opcua read --node "ns=2;s=Temperature_Sensor_01"

# Subscribe to node changes
wia-ind-027 opcua subscribe --node "ns=2;s=Production_Status" --interval 1000

# MQTT publish sensor data
wia-ind-027 mqtt publish --topic sensors/temp-001 --message '{"value": 45.2, "unit": "celsius"}'

# MQTT subscribe to topics
wia-ind-027 mqtt subscribe --topic "sensors/+/temperature"

# Query time-series data
wia-ind-027 timeseries query --measurement sensor_data --start "2025-01-01" --end "2025-01-02" --agg mean

# Write time-series data
wia-ind-027 timeseries write --measurement vibration --sensor vib-001 --value 0.35 --unit "mm/s"

# List connected devices
wia-ind-027 devices list --protocol opcua

# Update device firmware
wia-ind-027 devices update --device plc-001 --firmware v2.3.1

# Create alert rule
wia-ind-027 alerts create --metric temperature --threshold 80 --severity critical --action notify

# View dashboard metrics
wia-ind-027 dashboard --factory factory-001 --live

# Digital twin operations
wia-ind-027 twin create --asset machine-001 --type cnc --model DMG-5000
wia-ind-027 twin sync --asset machine-001 --live
wia-ind-027 twin predict --asset machine-001 --metric rul --horizon 30d

# MES integration
wia-ind-027 mes sync --production-order PO-12345 --status in-progress
```

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [WIA-IND-027-v1.0.md](./spec/WIA-IND-027-v1.0.md) | Complete specification with protocols |
| [TypeScript SDK](./api/typescript/) | Full TypeScript implementation |
| [CLI Tool](./cli/wia-ind-027.sh) | Command-line interface |

## 🚀 Quick Start

### Installation

```bash
# Clone repository
git clone https://github.com/WIA-Official/wia-standards.git
cd wia-standards/standards/industrial-iot

# Run installation script
./install.sh

# Verify installation
wia-ind-027 --version
```

### TypeScript Usage

```bash
# Install via npm
npm install @wia/ind-027

# Or yarn
yarn add @wia/ind-027
```

```typescript
import { IndustrialIoTSDK } from '@wia/ind-027';

const sdk = new IndustrialIoTSDK();

// Connect to factory edge gateway
const gateway = sdk.connectEdgeGateway({
  host: '192.168.1.50',
  port: 8080,
  protocol: 'https',
  apiKey: 'factory-key-001'
});

// Monitor production line
const monitor = await gateway.monitorLine({
  lineId: 'line-A',
  metrics: ['oee', 'throughput', 'quality', 'downtime'],
  interval: 5000
});

monitor.on('data', (metrics) => {
  console.log(`OEE: ${metrics.oee}%`);
  console.log(`Throughput: ${metrics.throughput} units/hr`);
  console.log(`Quality: ${metrics.quality}%`);

  if (metrics.oee < 70) {
    console.log('⚠️ Low OEE detected - investigating...');
  }
});
```

## 🏭 Industrial IoT Features

### IIoT Device Protocols

- **OPC-UA (Unified Architecture)**: Modern industrial protocol
  - Client/Server and Pub/Sub models
  - Type-safe information modeling
  - Built-in security (encryption, authentication)
  - Platform-independent communication

- **MQTT (Message Queuing Telemetry Transport)**: Lightweight pub/sub
  - Low bandwidth requirements
  - Quality of Service levels (QoS 0, 1, 2)
  - Last Will and Testament (LWT)
  - Retained messages for state

- **Modbus**: Legacy industrial protocol
  - Modbus RTU (serial)
  - Modbus TCP (Ethernet)
  - Wide device compatibility
  - Simple master/slave architecture

- **PROFINET**: Real-time Ethernet
  - Deterministic communication
  - IRT (Isochronous Real-Time)
  - Motion control support
  - Siemens ecosystem integration

### Edge Computing

- **Local Processing**: Reduce cloud latency
  - Stream processing at edge
  - Real-time decision making
  - Bandwidth optimization
  - Offline capability

- **Edge Analytics**: On-site intelligence
  - Anomaly detection
  - Pattern recognition
  - Predictive maintenance
  - Quality control

- **Data Filtering**: Reduce data volume
  - Dead-band filtering
  - Change-of-value triggers
  - Statistical sampling
  - Data compression

### Time-Series Database

- **InfluxDB Integration**: High-performance TSDB
  - Millisecond precision timestamps
  - Tag-based indexing
  - Continuous queries
  - Retention policies

- **Query Optimization**: Fast analytics
  - Aggregation functions (mean, sum, max, min)
  - Downsampling for long-term storage
  - Window functions
  - Group by time intervals

- **Data Retention**: Storage management
  - Hot data (real-time): 7-30 days
  - Warm data (aggregated): 90-365 days
  - Cold data (archived): 1-7 years
  - Automatic cleanup policies

### Real-Time Monitoring

- **Dashboard Visualization**: Live metrics
  - Production KPIs (OEE, MTBF, MTTR)
  - Equipment status displays
  - Trend charts and graphs
  - Alarm lists and notifications

- **Manufacturing Metrics**:
  - **OEE (Overall Equipment Effectiveness)**: Availability × Performance × Quality
  - **Throughput**: Units produced per time period
  - **Cycle Time**: Time per production cycle
  - **Downtime**: Unplanned stops and duration
  - **Scrap Rate**: Defects per total production

- **Andon Boards**: Visual management
  - Line status indicators
  - Stop/slow/run signals
  - Help requests
  - Quality alerts

### Alert & Notification Systems

- **Threshold Alerts**: Value-based triggers
  - Temperature > 80°C
  - Vibration > 0.5 mm/s
  - Pressure < 5 bar
  - Quality < 95%

- **Anomaly Detection**: ML-powered alerts
  - Statistical process control
  - Baseline deviation
  - Pattern recognition
  - Predictive failures

- **Escalation Policies**: Multi-tier notifications
  - Immediate: SMS to operators
  - 5 minutes: Email to supervisors
  - 15 minutes: Page to managers
  - 30 minutes: Executive notification

### Device Management

- **Remote Configuration**: Over-the-air updates
  - Parameter changes
  - Setpoint adjustments
  - Recipe uploads
  - Schedule modifications

- **Firmware Updates**: Secure OTA
  - Staged rollouts
  - Rollback capability
  - Version tracking
  - Update verification

- **Device Discovery**: Auto-detection
  - Network scanning
  - Protocol detection
  - Capability negotiation
  - Automatic provisioning

### Industrial Security

- **Network Segmentation**: Defense in depth
  - DMZ for external access
  - Firewall between IT/OT
  - VLAN isolation
  - Air-gapped critical systems

- **Authentication**: Multi-factor security
  - Certificate-based auth (X.509)
  - Role-based access control (RBAC)
  - Time-based tokens
  - Hardware security modules (HSM)

- **Encryption**: Data protection
  - TLS 1.3 for transport
  - AES-256 for storage
  - Secure key management
  - Perfect forward secrecy

- **Intrusion Detection**: Threat monitoring
  - Industrial protocol analysis
  - Behavioral anomalies
  - Signature-based detection
  - SIEM integration

### MES/ERP Integration

- **Manufacturing Execution System (MES)**:
  - Production order management
  - Work-in-progress tracking
  - Material genealogy
  - Quality management
  - Performance analysis

- **Enterprise Resource Planning (ERP)**:
  - Order-to-cash flow
  - Inventory management
  - Procurement integration
  - Cost accounting
  - Demand planning

- **Integration Patterns**:
  - REST APIs for synchronous calls
  - Message queues for async events
  - ETL for batch transfers
  - Event-driven architecture
  - Service bus integration

### Digital Twin

- **Virtual Asset Model**: Digital replica
  - 3D geometry representation
  - Physical properties
  - Behavioral simulation
  - Real-time synchronization

- **Predictive Maintenance**: Failure prevention
  - Remaining useful life (RUL)
  - Condition-based monitoring
  - Prescriptive recommendations
  - Maintenance scheduling

- **What-If Analysis**: Scenario testing
  - Process optimization
  - Capacity planning
  - Layout changes
  - Equipment upgrades

## ⚠️ Implementation Considerations

1. **Network Reliability**: Industrial environments need 99.9%+ uptime
2. **Latency Requirements**: Control loops need <10ms response
3. **Deterministic Communication**: Real-time protocols for critical control
4. **Harsh Environments**: Temperature, vibration, dust, moisture
5. **Legacy Integration**: Support 10-30 year old equipment
6. **Safety Standards**: IEC 61508, ISO 13849 compliance
7. **Cybersecurity**: IEC 62443 industrial security standards
8. **Scalability**: Support 1,000-100,000 devices per site

## 🌐 WIA Integration

This standard integrates with:
- **WIA-INTENT**: Natural language control of industrial systems
- **WIA-OMNI-API**: Universal industrial API gateway
- **WIA-SECURITY**: Enhanced security for critical infrastructure
- **WIA-TIME**: Synchronized time across distributed systems
- **WIA-BLOCKCHAIN**: Immutable audit trails for quality records

## 📖 Use Cases

1. **Smart Manufacturing**: Lights-out factories with autonomous systems
2. **Predictive Maintenance**: AI-powered equipment health monitoring
3. **Quality 4.0**: Real-time quality analytics and traceability
4. **Energy Management**: Monitor and optimize factory energy usage
5. **Supply Chain Visibility**: End-to-end material tracking
6. **Remote Monitoring**: Manage distributed manufacturing sites
7. **Worker Safety**: Real-time hazard detection and prevention
8. **Process Optimization**: Continuous improvement through data analytics

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
