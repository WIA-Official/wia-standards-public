# WIA-SOC-009 Phase 3: Communication Protocol Specification

**Version:** 1.0.0  
**Status:** Approved  
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 3 defines communication protocols, security requirements, and network architecture for smart sewage systems. Systems MUST implement secure, reliable, and scalable communication infrastructure.

## 2. Network Architecture

### 2.1 Layered Architecture

```
┌─────────────────────────────────────┐
│   Application Layer (Cloud/Central) │
│   - Analytics & Visualization       │
│   - Regulatory Reporting            │
│   - Predictive Models               │
└─────────────────────────────────────┘
            ↕ HTTPS/TLS 1.3
┌─────────────────────────────────────┐
│   Edge Computing Layer              │
│   - Local Processing                │
│   - Data Aggregation                │
│   - Emergency Control               │
└─────────────────────────────────────┘
            ↕ MQTT/WebSocket
┌─────────────────────────────────────┐
│   Sensor/Device Layer               │
│   - Field Sensors                   │
│   - Controllers                     │
│   - Actuators                       │
└─────────────────────────────────────┘
```

### 2.2 Communication Technologies

**Long-Range, Low-Power:**
- LoRaWAN: Sensors in remote locations
- NB-IoT: Cellular coverage areas
- Sigfox: Ultra-low bandwidth applications

**Local Networks:**
- WiFi 6: High-bandwidth local communication
- Ethernet: Wired backbone connections
- Modbus TCP/IP: Industrial equipment integration

**Backbone:**
- Fiber Optic: High-speed inter-facility links
- 4G/5G: Mobile connectivity and redundancy

## 3. Protocol Stack

### 3.1 MQTT for Sensor Data

**Broker Configuration:**
- TLS 1.3 encryption mandatory
- Username/password + client certificates
- QoS levels: 0 (at most once), 1 (at least once), 2 (exactly once)
- Retained messages for last known state
- Last will and testament for connection monitoring

**Topic Structure:**
```
wia-soc-009/{systemId}/{location}/{sensorType}/{sensorId}
```

**Examples:**
```
wia-soc-009/SYS001/influent/flow/FLOW001
wia-soc-009/SYS001/zone3/quality/pH/PH003
wia-soc-009/SYS001/pump-station-5/equipment/PUMP017
```

**Message Payload:**
```json
{
  "timestamp": "2025-12-26T14:30:00Z",
  "value": 4.2,
  "unit": "m³/s",
  "quality": "good",
  "metadata": {
    "batteryLevel": 85,
    "signalStrength": -72
  }
}
```

### 3.2 WebSocket for Real-time Streaming

**Connection:**
```
wss://api.example.com/v1/ws/stream
```

**Authentication:**
- JWT token in connection header
- Token refresh mechanism
- Session timeout: 24 hours

**Message Types:**
- `data`: Sensor readings and events
- `control`: Commands and acknowledgments
- `heartbeat`: Keep-alive messages
- `error`: Error notifications

**Example Data Message:**
```json
{
  "type": "data",
  "channel": "flow",
  "timestamp": "2025-12-26T14:30:00Z",
  "payload": {
    "locationId": "MP-001",
    "flowRate": 4.2,
    "trend": "increasing"
  }
}
```

### 3.3 RESTful API over HTTPS

**Security Requirements:**
- TLS 1.3 minimum
- HTTP/2 or HTTP/3
- HSTS (HTTP Strict Transport Security)
- Certificate pinning for critical operations

**Rate Limiting:**
- Authenticated: 1000 requests/hour
- Public endpoints: 100 requests/hour
- Burst: 10 requests/second

**Compression:**
- gzip or brotli compression
- Reduces bandwidth by 70-85%

## 4. Security Requirements

### 4.1 Authentication & Authorization

**Authentication Methods:**
1. **OAuth 2.0** for user applications
2. **API Keys** for system-to-system integration
3. **Client Certificates** for critical infrastructure
4. **Multi-Factor Authentication** for administrative access

**Authorization:**
- Role-Based Access Control (RBAC)
- Roles: viewer, operator, engineer, administrator
- Granular permissions per resource type
- Audit logging of all access

### 4.2 Encryption

**Data in Transit:**
- TLS 1.3 for all external communications
- AES-256-GCM cipher suite
- Perfect Forward Secrecy (PFS)
- Certificate rotation every 90 days

**Data at Rest:**
- AES-256 encryption for databases
- Key management via HSM or cloud KMS
- Encrypted backups
- Secure deletion procedures

### 4.3 Network Security

**Firewalling:**
- Network segmentation (sensors, edge, management)
- Whitelist-based access control
- Intrusion Detection Systems (IDS)
- DDoS protection

**VPN for Remote Access:**
- IPSec or WireGuard
- Multi-factor authentication
- Split-tunneling prohibited
- Session logging

## 5. Data Transmission Protocols

### 5.1 Sensor to Edge Gateway

**Protocol:** MQTT over TLS

**Configuration:**
```yaml
mqtt:
  broker: edge-gateway.local:8883
  client_id: SENSOR-{id}
  tls:
    ca_cert: /path/to/ca.crt
    client_cert: /path/to/client.crt
    client_key: /path/to/client.key
  qos: 1
  retain: true
  keep_alive: 60
```

**Publish Frequency:**
- Critical parameters (flow, quality): Every 60 seconds
- Equipment status: Every 5 minutes
- Environmental sensors: Every 10 minutes
- Event-driven: Immediate on threshold violation

### 5.2 Edge to Cloud

**Protocol:** HTTPS/REST API

**Data Aggregation:**
- Batch sensor readings (up to 100 per request)
- Compress with gzip
- Send every 5 minutes or on buffer full
- Store locally if connection lost (up to 24 hours)

**Resilience:**
- Automatic retry with exponential backoff
- Store-and-forward architecture
- Dual connectivity (primary + backup)
- Offline operation capability

## 6. Edge Computing Specifications

### 6.1 Edge Gateway Requirements

**Hardware:**
- ARM Cortex-A or x86-64 processor
- 2GB+ RAM
- 16GB+ storage (SSD preferred)
- Multiple communication interfaces
- Temperature range: -20°C to +60°C

**Software:**
- Linux-based OS (Ubuntu, Debian, or similar)
- Docker containerization
- Automatic updates with rollback
- Monitoring and health reporting

### 6.2 Edge Processing Capabilities

**Local Analytics:**
- Anomaly detection (statistical methods)
- Threshold monitoring
- Data quality validation
- Sensor fusion

**Emergency Control:**
- Automatic pump control during overflows
- Valve actuation for diversion
- Chemical dosing adjustment
- Alert generation and local notification

**Data Management:**
- Time-series database (InfluxDB, TimescaleDB)
- 24-hour local retention minimum
- Automated backup to cloud
- Data compression and archiving

## 7. Device Management

### 7.1 Over-the-Air (OTA) Updates

**Firmware Update Process:**
1. Cryptographically signed firmware packages
2. Incremental updates to reduce bandwidth
3. Staged rollout (canary deployment)
4. Automatic rollback on failure
5. Version tracking and auditing

**Update Scheduling:**
- During low-activity periods
- Maintenance windows
- Emergency patches within 24 hours
- Coordination across device fleet

### 7.2 Remote Configuration

**Configuration Management:**
- Centralized configuration server
- Version control for configurations
- Validation before deployment
- Audit trail of all changes

**Configurable Parameters:**
- Sampling intervals
- Alert thresholds
- Calibration coefficients
- Communication settings

## 8. Reliability and Redundancy

### 8.1 Failover Mechanisms

**Communication Redundancy:**
- Primary and backup connectivity
- Automatic failover (< 30 seconds)
- Health monitoring of all links
- Regular failover testing

**Geographic Redundancy:**
- Data replication across regions
- Distributed edge gateways
- Multi-region cloud deployment
- Disaster recovery procedures

### 8.2 Quality of Service (QoS)

**Priority Levels:**
1. **Critical**: Safety alerts, overflow warnings (highest priority)
2. **High**: Water quality violations, equipment failures
3. **Medium**: Normal sensor readings, status updates
4. **Low**: Historical data sync, batch uploads

**Network Management:**
- Traffic shaping and prioritization
- Bandwidth reservation for critical data
- Congestion avoidance
- Jitter minimization for real-time streams

## 9. Interoperability Standards

### 9.1 Protocol Support

**Required:**
- MQTT 3.1.1 or 5.0
- HTTPS/REST (OpenAPI 3.0 documented)
- WebSocket (RFC 6455)
- JSON-LD for data exchange

**Recommended:**
- OPC UA for industrial equipment
- Modbus TCP for legacy systems
- BACnet for building automation integration
- CoAP for resource-constrained devices

### 9.2 Data Exchange Formats

**Primary:** JSON-LD with WIA-SOC-009 context

**Supported:**
- CSV for bulk data export
- Protocol Buffers for high-performance applications
- MessagePack for compact encoding
- XML for legacy system integration

## 10. Monitoring and Diagnostics

### 10.1 System Health Monitoring

**Metrics to Track:**
- Connection uptime (%)
- Message delivery rate (%)
- End-to-end latency (ms)
- Bandwidth utilization (%)
- Error rates by type
- Battery levels (for wireless sensors)

**Alerting:**
- Communication failures
- Degraded performance
- Security incidents
- Certificate expiration warnings

### 10.2 Diagnostic Tools

**Network Diagnostics:**
- Ping and traceroute utilities
- Bandwidth testing
- Packet capture for troubleshooting
- Connection quality metrics

**Protocol Analysis:**
- MQTT message inspection
- API request/response logging
- WebSocket frame analysis
- Performance profiling

---

© 2025 WIA · MIT License
