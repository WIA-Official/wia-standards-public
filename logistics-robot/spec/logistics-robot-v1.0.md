# WIA Logistics Robot Standard v1.0

## Document Information

- **Standard ID:** WIA-ROB-009
- **Title:** WIA Logistics Robot Standard
- **Version:** 1.0.0
- **Status:** Published
- **Publication Date:** 2025-01-15
- **Category:** Robotics & Automation (ROB)
- **Color:** #10B981 (Green)
- **Philosophy:** 弘益人間 (Hongik Ingan) - Benefit All Humanity

## Executive Summary

The WIA Logistics Robot Standard establishes a universal framework for interoperability, safety, and performance across all types of autonomous logistics robots including Automated Guided Vehicles (AGVs), Autonomous Mobile Robots (AMRs), and specialized warehouse automation systems. This standard enables seamless integration between robots from different manufacturers, fleet management systems, and enterprise warehouse software.

The standard follows a four-phase architecture enabling progressive adoption:
1. **Phase 1:** Data Format standardization
2. **Phase 2:** API Interface specifications
3. **Phase 3:** Navigation and Communication protocols
4. **Phase 4:** System Integration guidelines

## 1. Scope

### 1.1 Inclusions

This standard applies to:
- Automated Guided Vehicles (AGVs) using fixed-path navigation
- Autonomous Mobile Robots (AMRs) with free-navigation capabilities  
- Collaborative warehouse robots working alongside humans
- Automated storage and retrieval systems (AS/RS) with mobile components
- Sortation robots and package handling systems
- Fleet management systems controlling logistics robots
- Warehouse management systems (WMS) integrating with robot fleets

### 1.2 Exclusions

This standard does not cover:
- Stationary warehouse equipment (fixed conveyors, static shelving)
- Industrial robotic arms without mobile bases
- Forklifts operated by human drivers
- Outdoor autonomous vehicles (covered by separate WIA standards)
- Drone-based warehouse systems (covered by WIA-DRONE standards)

## 2. Normative References

The following standards are normatively referenced:
- **VDA 5050** - Interface for communication between AGV systems and master control
- **ISO 3691-4** - Industrial trucks - Safety requirements for driverless trucks
- **ISO 10218-1/2** - Robots and robotic devices - Safety requirements (for manipulation components)
- **ROS 2 DDS** - Robot Operating System 2 Data Distribution Service
- **MQTT 5.0** - Message Queuing Telemetry Transport protocol
- **OAuth 2.0 (RFC 6749)** - Authorization framework
- **JSON Schema Draft 2020-12** - JSON data validation

## 3. Terms and Definitions

### 3.1 Core Terms

**AGV (Automated Guided Vehicle):** Mobile robot following predetermined paths using guidance systems such as magnetic tape, wire, or laser targets.

**AMR (Autonomous Mobile Robots):** Mobile robot capable of navigating dynamically using onboard sensors (LiDAR, cameras) without fixed infrastructure.

**Fleet Management System (FMS):** Software system controlling and coordinating multiple robots.

**SLAM (Simultaneous Localization and Mapping):** Algorithmic approach enabling robots to build maps while determining their position within those maps.

**Telemetry:** Operational data transmitted by robots including position, battery status, sensor readings, and task state.

**VDA 5050:** Communication standard developed by German automotive industry for AGV/FMS interface.

**Warehouse Management System (WMS):** Enterprise software managing warehouse operations including inventory, orders, and material flow.

## 4. Phase 1: Data Format Specification

### 4.1 Robot Telemetry Schema

All robots must transmit telemetry using the following JSON schema at minimum 1 Hz frequency:

```json
{
  "$schema": "https://wiastandards.com/schemas/logistics-robot/telemetry-v1.0.json",
  "robotId": "string (required, unique identifier)",
  "timestamp": "string (required, ISO 8601 format)",
  "position": {
    "x": "number (required, meters)",
    "y": "number (required, meters)",
    "z": "number (optional, meters, default 0)",
    "frame": "string (required, coordinate frame identifier)",
    "confidence": "number (optional, 0.0-1.0)"
  },
  "orientation": {
    "yaw": "number (required, radians)",
    "pitch": "number (optional, radians, default 0)",
    "roll": "number (optional, radians, default 0)"
  },
  "battery": {
    "stateOfCharge": "number (required, 0-100 percent)",
    "voltage": "number (optional, volts)",
    "estimatedRuntime": "number (optional, minutes)"
  },
  "status": "enum (required: idle|moving|picking|charging|error)",
  "currentTask": "string (optional, task identifier)",
  "errors": "array of error objects (optional)"
}
```

### 4.2 Package Tracking Schema

```json
{
  "packageId": "string (required, unique)",
  "trackingNumbers": "array of strings (optional)",
  "dimensions": {
    "length": "number (meters)",
    "width": "number (meters)",
    "height": "number (meters)",
    "weight": "number (kilograms)"
  },
  "destination": {
    "zoneId": "string (required)",
    "locationId": "string (required)"
  },
  "priority": "enum (low|medium|high|urgent)",
  "robotAssigned": "string (optional, robot ID)",
  "status": "enum (pending|in_transit|delivered|failed)"
}
```

### 4.3 Warehouse Map Format

Maps must be provided in one or more of the following formats:

**Occupancy Grid:**
- Resolution: 0.05-0.1 meters per cell
- Format: PGM (Portable Gray Map) or PNG
- Metadata: YAML file with origin, resolution, occupancy thresholds

**Semantic Map:**
- Format: JSON with zone definitions
- Required fields: zone_id, type (aisle|storage|staging|charging), polygon vertices, restrictions

**Topological Graph:**
- Format: JSON with nodes and edges
- Nodes: waypoints with x,y coordinates
- Edges: connections with traversal cost

## 5. Phase 2: API Interface

### 5.1 RESTful API Endpoints

Fleet Management Systems must implement the following REST API:

**Robot Management:**
- `GET /api/v1/robots` - List all robots
- `GET /api/v1/robots/{id}` - Get robot status
- `POST /api/v1/robots/{id}/command` - Send command to robot
- `PUT /api/v1/robots/{id}/config` - Update robot configuration

**Task Management:**
- `POST /api/v1/tasks` - Create new task
- `GET /api/v1/tasks/{id}` - Get task status
- `DELETE /api/v1/tasks/{id}` - Cancel task
- `GET /api/v1/tasks?robot={id}` - Get robot's task queue

**Fleet Operations:**
- `GET /api/v1/fleet/status` - Get fleet-wide status
- `GET /api/v1/fleet/metrics` - Get performance metrics
- `POST /api/v1/fleet/emergency-stop` - Trigger fleet-wide emergency stop

### 5.2 VDA 5050 Implementation

Robots must support VDA 5050 version 2.0 or higher with the following WIA extensions:

**WIA Extension 1 - Dynamic Path Planning:**
- Support for waypoint modification during execution
- Real-time obstacle avoidance reporting
- Path replanning notifications

**WIA Extension 2 - Multi-Floor Navigation:**
- Elevator coordination protocol
- Floor transition state reporting
- 3D position reporting (z-coordinate mandatory)

**WIA Extension 3 - Battery Management:**
- State of charge reporting with 1% granularity
- Runtime prediction accuracy ±5 minutes
- Charging station reservation protocol

### 5.3 WebSocket Real-Time Updates

Real-time telemetry streaming via WebSocket:
- Connection: `wss://fleet.example.com/ws/telemetry`
- Authentication: JWT token in connection header
- Message rate: 1-20 Hz (configurable)
- Compression: Optional gzip compression for bandwidth optimization

### 5.4 Authentication & Authorization

**OAuth 2.0 Requirements:**
- Token type: JWT (JSON Web Tokens)
- Minimum token expiry: 1 hour
- Refresh token support: Required
- Scopes: robot:read, robot:command, robot:configure, fleet:manage, admin:all

**API Security:**
- TLS 1.2 or higher required for all API communication
- API rate limiting: 100 req/min (read), 20 req/min (write)
- Audit logging: All commands logged with timestamp, user, action

## 6. Phase 3: Navigation & Communication Protocol

### 6.1 SLAM Requirements

Robots using SLAM for navigation must:
- Achieve localization accuracy: ±10cm (95th percentile)
- Update frequency: Minimum 10 Hz
- Map resolution: Maximum 0.1m per cell
- Support map sharing with other WIA-compliant robots via standard format

Supported SLAM algorithms:
- GMapping (2D laser)
- Cartographer (2D/3D LiDAR)
- ORB-SLAM (vision-based)
- Any algorithm producing WIA-compliant output

### 6.2 Path Planning

**Global Path Planning:**
- Algorithms: A*, Dijkstra, or equivalent
- Recompute on dynamic obstacle detection
- Maximum planning time: 1 second for 100m path
- Path smoothing required for human comfort and efficiency

**Local Path Planning:**
- Algorithm: DWA, TEB, or equivalent reactive planner
- Update rate: Minimum 10 Hz
- Obstacle reaction time: <200ms from detection to velocity adjustment

### 6.3 Collision Avoidance

**Protective Fields (ISO 3691-4 compliant):**
- Warning Field: 2-3 meters, reduce speed to 50%
- Safety Field: 1-2 meters, reduce speed to 25%  
- Emergency Stop Field: 0-1 meter, immediate stop

**Multi-Robot Coordination:**
- Space-time reservation protocol
- Priority-based yielding (higher priority robots proceed first)
- Deadlock detection timeout: 30 seconds
- Automatic deadlock resolution within 60 seconds

### 6.4 Charging Protocol

**Automatic Charging:**
- Battery threshold for charging: Configurable (default 20%)
- Charging station reservation: Required to prevent queue congestion
- Docking accuracy: ±3cm position, ±5° orientation
- Charging start confirmation: Within 10 seconds of docking

**Opportunity Charging:**
- Support for partial charging during idle periods
- Minimum charging session: 5 minutes
- Charge rate reporting: kW delivered

## 7. Phase 4: System Integration

### 7.1 WMS Integration

**Integration Levels:**

**Level 1 - Task Sync:**
- WMS creates pick/putaway tasks
- Robot fleet executes tasks
- Completion status synced back to WMS
- Latency: <5 seconds for status updates

**Level 2 - Inventory Updates:**
- Real-time inventory position updates
- RFID/barcode scan data transmitted to WMS
- Inventory accuracy: >99.9%

**Level 3 - Bidirectional Optimization:**
- WMS task planning considers robot availability
- Robot fleet provides capacity forecasts to WMS
- Joint optimization of task sequencing

**Common WMS Platforms:**
- Manhattan Associates WMOS API
- Blue Yonder (JDA) WMS API
- SAP Extended Warehouse Management (EWM) API
- Oracle WMS Cloud API
- HighJump WMS API

### 7.2 IoT Sensor Integration

**Supported Sensor Types:**
- RFID Readers: ISO 15693, EPC Gen2
- Weight Scales: 0.1kg resolution minimum
- Temperature Sensors: ±1°C accuracy
- Barcode Scanners: Code 128, QR, Data Matrix
- Vision Systems: Minimum 1080p resolution

**Sensor Data Format:**
```json
{
  "sensorId": "string",
  "sensorType": "enum (rfid|weight|temperature|barcode|vision)",
  "timestamp": "ISO 8601",
  "data": "sensor-specific payload",
  "robotId": "string (robot carrying sensor)",
  "location": "position object"
}
```

### 7.3 Legacy System Adapters

**Supported Protocols:**
- Modbus TCP/IP
- Profibus DP
- EtherCAT
- OPC UA
- MQTT 3.1.1 / 5.0

**Adapter Requirements:**
- Latency overhead: <50ms
- Protocol translation accuracy: 100% (no data loss)
- Error handling with automatic retry
- Audit logging of all translated messages

## 8. Certification Requirements

### 8.1 Certification Levels

**Level 1: Data Format**
- JSON schema validation passing for all required messages
- Telemetry transmission at minimum 1 Hz
- Battery and status reporting functional

**Level 2: Full API**  
- All Phase 1 requirements
- REST API endpoints implemented and tested
- VDA 5050 compliance (with WIA extensions)
- WebSocket real-time updates functional

**Level 3: Complete Integration**
- All Phase 1 & 2 requirements
- Navigation and collision avoidance tested
- WMS integration demonstrated (Level 1 minimum)
- Safety compliance (ISO 3691-4) validated

### 8.2 Certification Process

1. **Self-Assessment** (2-4 weeks) - Review requirements, run validation tools
2. **Documentation** (1-2 weeks) - Submit technical docs, architecture, test results
3. **Automated Testing** (1-2 weeks) - Run official WIA compliance test suite  
4. **Interoperability** (2-3 weeks) - Test with reference implementations
5. **Security Audit** (1-2 weeks) - Validate authentication, encryption, access control
6. **Certification Award** (1 week) - Issue certificate and compliance marks

### 8.3 Testing Requirements

**Phase 1 Testing:**
- Schema validation: 1000 test messages, 100% pass rate
- Telemetry frequency test: 60 second continuous transmission
- Position accuracy: ±20cm vs. ground truth

**Phase 2 Testing:**
- API endpoint tests: All endpoints, 95% success rate minimum  
- Load testing: 100 concurrent connections, <200ms P95 latency
- VDA 5050 conformance: Official VDA test suite passing

**Phase 3 Testing:**
- Navigation accuracy: ±10cm, 90% of time
- Collision avoidance: Zero collisions in 100 test scenarios
- Multi-robot coordination: 10 robots, zero deadlocks in 1 hour

## 9. Performance Requirements

### 9.1 Robot Performance

| Metric | Requirement | Measurement Method |
|--------|------------|-------------------|
| Position Accuracy | ±10cm (95th percentile) | Laser tracker ground truth |
| Localization Update Rate | ≥10 Hz | Telemetry timestamp analysis |
| Maximum Speed | Per manufacturer spec | Configurable, safety-limited |
| Obstacle Detection Range | ≥5 meters | LiDAR/sensor spec |
| Emergency Stop Distance | <0.5m @ max speed | Physical testing |
| Battery Runtime | ≥4 hours continuous | Discharge test |

### 9.2 System Performance

| Metric | Requirement | Measurement Method |
|--------|------------|-------------------|
| API Latency (P95) | <200ms | Load testing |
| WebSocket Message Rate | 1-20 Hz | Network monitoring |
| Task Assignment Time | <5 seconds | Fleet manager logs |
| Fleet Utilization | >70% during operations | Analytics dashboard |
| System Uptime | >99% | Availability monitoring |

## 10. Safety Requirements

### 10.1 ISO 3691-4 Compliance

All robots must comply with ISO 3691-4 including:
- Protective fields (warning, safety, emergency stop)
- Visual and audible warnings when moving
- Emergency stop button on robot (where applicable)
- Fail-safe behavior on sensor failure
- Maximum speed limits in human-occupied zones

### 10.2 Risk Assessment

Risk assessment must address:
- Human-robot collision scenarios
- Dropped package hazards
- Fire and evacuation procedures
- Battery safety (thermal runaway, charging hazards)
- Network failure and autonomous operation

## 11. Maintenance and Support

### 11.1 Predictive Maintenance

Robots should implement predictive maintenance using:
- Motor current analysis (bearing wear detection)
- Battery cycle counting and capacity tracking
- Sensor performance degradation monitoring
- Software error rate tracking

### 11.2 Remote Diagnostics

Support for remote diagnostics including:
- Log file export (JSON format)
- Remote sensor data access
- Software version reporting
- Configuration backup and restore

## 12. Governance and Evolution

### 12.1 Standard Governance

- **Technical Committee:** Robot manufacturers, integrators, technology providers
- **User Advisory Board:** Warehouse operators, retailers, 3PL providers
- **Certification Authority:** Independent testing organizations
- **Open Source Community:** GitHub repository for reference implementations

### 12.2 Version Management

- **Semantic Versioning:** MAJOR.MINOR.PATCH
- **Backward Compatibility:** Support current + one previous major version
- **Deprecation Policy:** 24 months notice for breaking changes
- **Release Cycle:** Minor updates every 6-12 months

## 13. References

### 13.1 Normative References

- VDA 5050 Communication Interface for AGVs, Version 2.0
- ISO 3691-4:2020 Industrial trucks - Safety requirements
- RFC 6749 OAuth 2.0 Authorization Framework
- RFC 8259 JSON Data Interchange Format
- ISO/IEC 19505-2 UML Specification

### 13.2 Informative References

- ROS 2 Documentation: https://docs.ros.org
- MQTT Version 5.0 Specification
- W3C Verifiable Credentials Data Model
- ANSI/ITSDF B56.5 Safety Standard for Guided Industrial Vehicles

## Appendix A: JSON Schema Examples

Complete JSON schemas available at:
- https://wiastandards.com/schemas/logistics-robot/telemetry-v1.0.json
- https://wiastandards.com/schemas/logistics-robot/package-v1.0.json
- https://wiastandards.com/schemas/logistics-robot/map-v1.0.json

## Appendix B: API Reference

Complete API reference documentation:
- https://api.wiastandards.com/docs/logistics-robot/v1

## Appendix C: Certification Checklist

Detailed certification checklist:
- https://cert.wiastandards.com/logistics-robot/checklist-v1.0

---

**弘益人間 (Hongik Ingan) · Benefit All Humanity**

*WIA - World Certification Industry Association*  
*© 2025 SmileStory Inc. / WIA*  
*Licensed under MIT License - Open Standard for Global Interoperability*

*Document Version: 1.0.0*  
*Publication Date: 2025-01-15*  
*Next Review Date: 2026-01-15*
