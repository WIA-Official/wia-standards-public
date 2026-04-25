# Chapter 3: Standard Overview and Architecture

## The WIA-ENE-027 Framework for Environmental Sensor Interoperability

---

## 3.1 WIA-ENE-027 Design Philosophy

The WIA-ENE-027 Environmental Sensor Standard embodies several core principles that guide its design and implementation.

### Scientific Rigor

All specifications are grounded in established scientific practice and peer-reviewed methodologies:
- **Measurement Standards**: Based on EPA, NOAA, USGS, and international guidelines
- **Quality Assurance**: QA/QC frameworks from regulatory agencies
- **Validation Methods**: Proven calibration and validation procedures
- **Units and Terminology**: Standard scientific units (SI and accepted alternatives)

### Operational Practicality

The standard balances scientific sophistication with real-world implementation:
- **Resource Constraints**: Acknowledges power, bandwidth, and cost limitations
- **Deployment Reality**: Designed for actual field conditions, not ideal laboratories
- **Maintenance Feasibility**: Calibration procedures achievable without specialized equipment
- **Scalability**: Works for 10 sensors or 10,000 sensors

### Vendor Neutrality

The standard does not favor any manufacturer or technology:
- **Open Specifications**: Publicly available, no proprietary elements
- **Technology Agnostic**: Works with any sensor meeting performance requirements
- **Protocol Flexibility**: Supports MQTT, CoAP, LoRaWAN, HTTP
- **Cloud Independence**: Can use any cloud provider or self-host

### Future-Proof Extensibility

The architecture accommodates evolution and new capabilities:
- **Versioning**: Semantic versioning ensures backward compatibility
- **Extension Points**: New sensor types can be added without breaking existing implementations
- **Protocol Evolution**: Can adopt new communication technologies
- **Analytics Integration**: Designed for machine learning and advanced analytics

---

## 3.2 Four-Phase Standard Architecture

The WIA-ENE-027 standard follows the WIA four-phase architecture, with each phase building on previous layers:

```
┌────────────────────────────────────────────────────────────────┐
│                  WIA-ENE-027 Architecture                      │
├────────────────────────────────────────────────────────────────┤
│                                                                │
│  PHASE 1: Data Format Specification                           │
│  ├── Core data model (all sensor types)                       │
│  ├── Air quality sensor schemas (PM, gases, AQI)              │
│  ├── Water quality sensor schemas (pH, DO, turbidity)         │
│  ├── Soil sensor schemas (moisture, temperature, nutrients)   │
│  ├── Meteorological schemas (temp, humidity, wind, precip)    │
│  ├── Metadata and quality flag definitions                    │
│  └── JSON Schema validation specifications                    │
│                                                                │
│  PHASE 2: API Interface Specification                         │
│  ├── RESTful endpoint design                                  │
│  ├── Sensor discovery and registration                        │
│  ├── Data submission (POST /sensors/{id}/data)                │
│  ├── Data retrieval and querying                              │
│  ├── Real-time streaming (WebSocket, SSE)                     │
│  ├── Authentication and authorization                          │
│  └── Error handling and pagination                            │
│                                                                │
│  PHASE 3: Protocol Specification                              │
│  ├── MQTT topic structure and QoS                             │
│  ├── CoAP resource paths and observe                          │
│  ├── LoRaWAN payload encoding/decoding                        │
│  ├── HTTP/REST communication patterns                         │
│  ├── TLS/DTLS security requirements                           │
│  ├── Data validation and quality control                      │
│  └── Edge computing and gateway patterns                      │
│                                                                │
│  PHASE 4: System Integration                                  │
│  ├── Cloud platform integration (AWS, Azure, GCP)             │
│  ├── Time-series database integration                         │
│  ├── Analytics and visualization platforms                    │
│  ├── Regulatory reporting automation                          │
│  ├── Integration with other WIA standards                     │
│  ├── Federated network data sharing                           │
│  └── Reference deployment architectures                       │
│                                                                │
└────────────────────────────────────────────────────────────────┘
```

### Phase Dependencies

- **Phase 1** is foundational - all implementations MUST comply
- **Phase 2** requires Phase 1 compliance - APIs transmit Phase 1 formatted data
- **Phase 3** requires Phases 1 and 2 - protocols carry Phase 1 data via Phase 2 interfaces
- **Phase 4** requires Phases 1-3 - integration assumes compliant data, APIs, and protocols

### Compliance Levels

Organizations can achieve different compliance levels:

| Level | Requirements | Use Case |
|-------|--------------|----------|
| **Phase 1 Compliant** | Data format only | Sensor manufacturers, data providers |
| **Phase 2 Compliant** | Data format + APIs | Application developers, platform providers |
| **Phase 3 Compliant** | Data + APIs + protocols | Complete sensor systems, IoT platforms |
| **Phase 4 Compliant** | All phases + integration | Enterprise deployments, smart cities |

---

## 3.3 Sensor Classification Framework

The standard organizes sensors into a hierarchical classification system:

### Sensor Type Hierarchy

```
Environmental Sensors
├── Air Quality
│   ├── Particulate Matter (PM1.0, PM2.5, PM4.0, PM10)
│   ├── Gaseous Pollutants (CO2, CO, NO2, O3, SO2, VOC)
│   └── Air Quality Index (AQI)
├── Water Quality
│   ├── Physical Parameters (pH, turbidity, temperature)
│   ├── Chemical Parameters (DO, conductivity, ORP)
│   └── Nutrient Parameters (nitrate, phosphate, etc.)
├── Soil
│   ├── Physical Parameters (moisture, temperature, EC)
│   └── Nutrient Parameters (N, P, K)
└── Meteorological
    ├── Atmospheric (temperature, humidity, pressure)
    ├── Wind (speed, direction)
    ├── Precipitation (rainfall, snow)
    └── Solar (radiation, UV index)
```

### Sensor Type Identifiers

Standard sensor type identifiers for the `sensorType` field:

| Sensor Type | Identifier | Primary Parameters |
|-------------|------------|-------------------|
| Air Quality | `"air_quality"` | PM2.5, PM10, CO2, NO2, O3 |
| Water Quality | `"water_quality"` | pH, DO, turbidity, conductivity |
| Soil | `"soil"` | Moisture, temperature, EC, nutrients |
| Meteorological | `"meteorological"` | Temperature, humidity, pressure, wind |
| Radiation | `"radiation"` | Solar radiation, UV index |
| Noise | `"noise"` | Sound pressure level, frequency |

---

## 3.4 Core Data Model Design

All WIA-ENE-027 compliant sensor data shares a common core structure.

### Mandatory Core Fields

Every sensor data message MUST include:

```json
{
  "version": "1.0.0",
  "standard": "WIA-ENE-027",
  "deviceId": "ENV-AIR-001",
  "timestamp": "2025-01-09T10:30:00.000Z",
  "sensorType": "air_quality",
  "readings": {
    // Sensor-specific measurements
  }
}
```

**Field Specifications:**

- **version**: Semantic version (MAJOR.MINOR.PATCH)
  - MAJOR: Breaking changes
  - MINOR: Backward-compatible additions
  - PATCH: Backward-compatible fixes

- **standard**: Always `"WIA-ENE-027"` for this standard

- **deviceId**: Globally unique identifier
  - Format: `[PREFIX]-[TYPE]-[IDENTIFIER]`
  - Example: `ENV-AIR-001`, `WATER-PH-SITE42`
  - Must be consistent across all messages from sensor

- **timestamp**: ISO 8601 format with timezone
  - MUST include timezone (Z for UTC or ±HH:MM)
  - Precision: milliseconds recommended
  - Example: `"2025-01-09T10:30:00.000Z"`

- **sensorType**: Standard identifier from classification
  - Must be one of defined sensor types
  - Determines expected `readings` structure

- **readings**: Sensor-specific measurements
  - Structure defined for each sensor type
  - Contains actual measurement data

### Optional Recommended Fields

Highly recommended for comprehensive data quality:

**Location:**
```json
"location": {
  "latitude": 37.5665,
  "longitude": 126.9780,
  "altitude": 38.5,
  "accuracy": 10.0,
  "datum": "WGS84"
}
```

**Metadata:**
```json
"metadata": {
  "manufacturer": "AirSense Corp",
  "model": "AS-3000",
  "firmware": "v2.4.1",
  "battery": 85,
  "signalStrength": -65,
  "uptime": 86400
}
```

**Quality:**
```json
"quality": {
  "overall": "good",
  "flags": [],
  "confidence": 0.95
}
```

**Calibration:**
```json
"calibration": {
  "lastCalibration": "2024-11-15T09:00:00Z",
  "nextCalibration": "2025-05-15T09:00:00Z",
  "method": "reference_colocated",
  "parameters": {"slope": 1.02, "intercept": -0.5},
  "certificateId": "CAL-2024-1234"
}
```

---

## 3.5 System Architecture Components

A complete WIA-ENE-027 compliant system comprises several components:

### Component Architecture

```
┌─────────────┐
│   Sensors   │ (Phase 1: Data Format)
└──────┬──────┘
       │ MQTT/CoAP/LoRaWAN/HTTP
       ↓
┌─────────────┐
│  Gateways   │ (Phase 3: Protocols)
└──────┬──────┘
       │ API Calls
       ↓
┌─────────────┐
│  API Layer  │ (Phase 2: REST APIs)
└──────┬──────┘
       │
       ├→┌──────────────┐
       │ │Time-Series DB│ (InfluxDB, Timestream)
       │ └──────────────┘
       │
       ├→┌──────────────┐
       │ │Analytics     │ (ML, Anomaly Detection)
       │ └──────────────┘
       │
       └→┌──────────────┐
         │Visualization │ (Dashboards, Apps)
         └──────────────┘
```

### Sensor Layer (Phase 1)

**Responsibilities:**
- Generate measurements according to Phase 1 data format
- Apply basic quality checks (range validation)
- Include calibration metadata
- Timestamp measurements accurately

**Example Sensors:**
- Air quality: PurpleAir, AirGradient, Clarity
- Water quality: Atlas Scientific, YSI EXO
- Soil: METER Group, Stevens Water
- Weather: Davis Vantage, Campbell Scientific

### Gateway Layer (Phase 3)

**Responsibilities:**
- Protocol translation (MQTT/CoAP/LoRaWAN → HTTP/API)
- Data buffering during connectivity loss
- Edge processing and filtering
- Local quality validation
- Security and authentication

**Example Gateways:**
- IoT gateways: AWS Greengrass, Azure IoT Edge
- LoRaWAN: The Things Gateway, RAK Wireless
- Custom: Raspberry Pi with MQTT/HTTP bridge

### API Layer (Phase 2)

**Responsibilities:**
- RESTful endpoints for data submission/retrieval
- Authentication and authorization
- Data validation against JSON schemas
- Rate limiting and quota management
- Real-time streaming interfaces

**Example Platforms:**
- Cloud: AWS IoT Core, Azure IoT Hub, Google Cloud IoT
- Self-hosted: FastAPI, Express.js, Spring Boot
- Open source: ThingsBoard, Grafana, Home Assistant

### Storage Layer (Phase 4)

**Responsibilities:**
- Persist time-series sensor data
- Enable efficient queries by time, location, sensor
- Support aggregation and downsampling
- Long-term archival

**Example Databases:**
- Time-series: InfluxDB, TimescaleDB, AWS Timestream
- Document: MongoDB, PostgreSQL with JSONB
- Object storage: AWS S3, Google Cloud Storage

### Application Layer (Phase 4)

**Responsibilities:**
- Visualization dashboards
- Alert and notification systems
- Analytics and reporting
- Integration with third-party systems
- Mobile applications

**Example Applications:**
- Grafana: Real-time dashboards
- Custom web apps: React, Vue.js
- Mobile apps: iOS, Android
- Analytics: Python/Jupyter, Apache Spark

---

## 3.6 Quality Assurance Framework

The standard establishes a comprehensive quality assurance framework:

### Quality Flag System

Standard quality flags indicating data conditions:

| Flag | Meaning | Severity | Action |
|------|---------|----------|--------|
| `calibration_due` | Calibration overdue | Warning | Schedule calibration soon |
| `calibration_overdue` | Calibration significantly overdue | Critical | Use data with caution |
| `out_of_range` | Value exceeds plausible range | Critical | Investigate sensor |
| `rapid_change` | Unrealistic rate of change | Warning | Check for malfunction |
| `low_battery` | Battery below 20% | Warning | Replace battery soon |
| `critical_battery` | Battery below 10% | Critical | Replace immediately |
| `poor_signal` | Weak connectivity | Warning | Check network |
| `sensor_fault` | Sensor self-diagnostic failure | Critical | Service required |
| `maintenance_required` | Service interval exceeded | Warning | Schedule maintenance |
| `estimated` | Value estimated/interpolated | Info | Lower confidence |

### Quality Levels

Overall quality assessment:

| Level | Description | Criteria | Use Case |
|-------|-------------|----------|----------|
| `"good"` | High quality data | Calibrated, in range, no flags | All applications |
| `"suspect"` | Questionable quality | Minor flags, borderline values | Use with caution |
| `"bad"` | Poor quality data | Major flags, out of range | Do not use for decisions |
| `"missing"` | No data available | Sensor offline, transmission failure | Gap in record |

### Validation Procedures

**Range Validation:**
- Check measurements against physically plausible ranges
- Flag values outside expected ranges
- Consider environmental context (e.g., higher PM10 in deserts)

**Temporal Validation:**
- Detect impossible rate-of-change
- Identify stuck sensors (no variation)
- Flag timestamp anomalies (future dates, duplicates)

**Cross-Parameter Validation:**
- PM2.5 should be ≤ PM10
- Dew point ≤ air temperature
- Validate relationships between parameters

---

## 3.7 Compliance and Certification

Organizations can achieve WIA-ENE-027 certification through validation:

### Certification Levels

**Level 1: Data Format Compliance**
- Requirements:
  - Generate valid Phase 1 JSON
  - Include all mandatory fields
  - Pass JSON Schema validation
  - Include quality metadata
- Validation: Automated schema validation
- Audit: Sample data review

**Level 2: API Compliance**
- Requirements:
  - Level 1 + Phase 2 API implementation
  - RESTful endpoints with OpenAPI documentation
  - Proper authentication and error handling
- Validation: API conformance testing
- Audit: Integration testing

**Level 3: Protocol Compliance**
- Requirements:
  - Level 2 + Phase 3 protocol support
  - MQTT/CoAP/LoRaWAN or HTTP implementation
  - Security (TLS/DTLS)
  - Edge processing capabilities
- Validation: Protocol conformance testing
- Audit: Security review

**Level 4: Full System Compliance**
- Requirements:
  - Level 3 + Phase 4 integration
  - Cloud platform integration
  - Analytics capabilities
  - Documentation and support
- Validation: End-to-end testing
- Audit: Production system review

### Certification Process

1. **Self-Assessment**: Organization evaluates compliance
2. **Documentation**: Submit technical documentation
3. **Testing**: Automated and manual conformance tests
4. **Audit**: WIA technical committee review
5. **Certification**: Issue compliance certificate
6. **Listing**: Add to WIA registry of compliant systems
7. **Maintenance**: Annual re-certification

---

## 3.8 Review Questions and Key Takeaways

### Review Questions

1. **Design Principles**: Explain how the WIA-ENE-027 standard balances scientific rigor with operational practicality. Give examples of each.

2. **Four-Phase Architecture**: Why is Phase 1 (Data Format) foundational to the other phases? What breaks if sensors don't comply with Phase 1?

3. **Compliance Levels**: An organization wants to build a sensor platform. They need to support multiple manufacturers but don't need protocol specifications. What compliance level should they target?

4. **Quality Flags**: Design quality flag logic for an air quality sensor. When should you set `calibration_due`, `calibration_overdue`, `out_of_range`, and `rapid_change` flags?

5. **System Components**: Diagram a complete WIA-ENE-027 deployment with 100 sensors, showing sensor layer, gateway layer, API layer, storage, and applications.

6. **Certification Value**: What are the benefits of WIA-ENE-027 certification for (a) sensor manufacturers, (b) system integrators, (c) end users?

### Key Takeaways

1. **Design Philosophy**: The standard balances scientific rigor, operational practicality, vendor neutrality, and future extensibility to serve real-world needs.

2. **Four-Phase Architecture**: Progressive layers (Data Format, API, Protocol, Integration) allow organizations to achieve compliance levels matching their needs.

3. **Phase 1 Foundation**: Data format standardization is foundational - all other phases depend on consistent, valid data structures.

4. **Sensor Classification**: Hierarchical organization (Air/Water/Soil/Meteorological) with standard identifiers enables consistent system design.

5. **Core Data Model**: All sensors share mandatory fields (version, standard, deviceId, timestamp, sensorType, readings) ensuring basic interoperability.

6. **Optional Metadata**: Location, quality, calibration, and device metadata are optional but strongly recommended for comprehensive data management.

7. **System Components**: Complete systems include sensors, gateways, APIs, storage, and applications working together through standard interfaces.

8. **Quality Assurance**: Standard quality flags and validation procedures enable automated data quality assessment and filtering.

9. **Compliance Levels**: Four certification levels allow organizations to demonstrate compliance appropriate to their role in the ecosystem.

10. **Certification Process**: Formal validation and certification by WIA ensures systems truly meet standard requirements and can interoperate.

---

## Chapter Summary

This chapter presented the WIA-ENE-027 standard architecture and design principles. The standard balances scientific rigor with practical deployment needs, maintains vendor neutrality, and provides extensibility for future evolution.

The four-phase architecture (Data Format, API, Protocol, Integration) enables progressive compliance. Phase 1 data format standardization is foundational, with each phase building on previous layers. Organizations can achieve compliance levels matching their ecosystem role.

The sensor classification framework organizes sensors hierarchically (Air Quality, Water Quality, Soil, Meteorological) with standard type identifiers. The core data model defines mandatory fields all sensors must include, plus optional recommended metadata for comprehensive data quality.

Complete systems comprise multiple layers: sensors generating Phase 1 data, gateways handling Phase 3 protocols, APIs providing Phase 2 interfaces, storage persisting data, and applications delivering user value. Standard quality flags and validation procedures enable automated quality assessment.

WIA certification validates compliance through testing and audit, with four levels corresponding to the four phases. Certification provides assurance of interoperability and standard compliance.

---

© 2025 WIA Standards Committee. 弘益人間 (홍익인간) - Benefit All Humanity

**Next Chapter: [Chapter 4: Data Formats and Schemas](04-data-format.md)**
