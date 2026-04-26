# WIA-ENE-032: Forest Fire Detection Standards
# PHASE 1 - Foundation & Core Detection Systems

**Version:** 1.0  
**Status:** Implementation Specification  
**Date:** 2025-01-28  
**Philosophy:** 弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Executive Summary

PHASE 1 establishes the foundational infrastructure for forest fire detection systems compliant with WIA-ENE-032 standards. This phase focuses on deploying core detection technologies, establishing data collection and processing pipelines, implementing basic alert mechanisms, and creating the organizational framework necessary for effective fire management.

### 1.1 Objectives

- Deploy initial satellite fire detection integration (MODIS, VIIRS, Landsat)
- Establish ground-based sensor network with minimum 100 nodes per 1000 km²
- Implement basic AI-powered camera detection with >85% accuracy, <10% false alarm rate
- Create centralized fire detection platform with API infrastructure
- Develop initial alert and notification systems
- Train personnel on standard operating procedures
- Establish baseline performance metrics and monitoring

### 1.2 Success Criteria

- Detection of 90% of fires >100m² within 30 minutes
- Average alert delivery time <5 minutes from detection
- System uptime >95%
- False alarm rate <10%
- Complete integration of at least 2 satellite data sources
- Operational ground sensor network covering priority areas
- Trained personnel capable of operating all core systems

### 1.3 Timeline

**Duration:** 12-18 months  
**Budget:** $2-5 million (varies by jurisdiction size and existing infrastructure)

---

## 2. System Architecture

### 2.1 Overall Architecture

```
┌─────────────────────────────────────────────────────────┐
│                   FIRE DETECTION PLATFORM                │
│                                                           │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │  Data        │  │  Detection   │  │  Alert       │  │
│  │  Ingestion   │→ │  Engine      │→ │  Distribution│  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
│          ↓                 ↓                  ↓          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │  Storage &   │  │  Analytics   │  │  API         │  │
│  │  Archive     │  │  Dashboard   │  │  Services    │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
└─────────────────────────────────────────────────────────┘
         ↑                                       ↓
┌────────────────────┐                  ┌────────────────┐
│  DATA SOURCES      │                  │  CONSUMERS     │
│  • Satellites      │                  │  • Fire Depts  │
│  • Cameras         │                  │  • EOC         │
│  • Ground Sensors  │                  │  • Public      │
│  • Weather         │                  │  • GIS Systems │
└────────────────────┘                  └────────────────┘
```

### 2.2 Core Components

#### 2.2.1 Data Ingestion Layer

**Satellite Integration Module**
- MODIS data ingestion from NASA FIRMS API
- VIIRS data processing (375m I-band, 750m M-band)
- Landsat thermal band processing (Band 10, 11)
- Real-time processing pipeline (< 4 hour latency)
- Historical data archive (minimum 5 years)

**Camera Network Module**
- PTZ camera management (minimum 50 units in PHASE 1)
- H.264/H.265 video streaming
- Thermal camera integration (LWIR 8-14 μm)
- Image capture and storage (minimum 30 days retention)
- Camera health monitoring and diagnostics

**Ground Sensor Integration**
- LoRaWAN gateway deployment
- MQTT broker for sensor data
- Time-series database for sensor readings
- Smoke detector data processing
- Weather station integration (FWI calculation)

#### 2.2.2 Detection Engine

**AI Detection Module**
- Convolutional Neural Network (CNN) for smoke/fire detection
- Minimum 85% detection rate, <10% false positive rate
- GPU-accelerated inference (<2 second processing time)
- Model versioning and A/B testing infrastructure
- Continuous learning pipeline for model improvement

**Satellite Detection Processing**
- Contextual algorithm implementation for MODIS
- Dual-band thermal detection (MIR + TIR)
- Fire Radiative Power (FRP) calculation
- Confidence scoring (0-100%)
- Geographic coordinate transformation and validation

**Multi-Sensor Fusion**
- Bayesian fusion of camera, satellite, sensor data
- Confidence aggregation algorithms
- Duplicate detection elimination
- Temporal correlation (tracking same fire across multiple observations)
- Geographic clustering of nearby detections

#### 2.2.3 Alert Distribution

**Notification Channels**
- SMS/text messaging (Twilio or equivalent)
- Email delivery (SendGrid or equivalent)
- Mobile push notifications (Firebase Cloud Messaging)
- Web dashboard real-time updates (WebSocket)
- API webhooks for external system integration

**Alert Routing Logic**
- Geographic-based routing (jurisdiction boundaries)
- Priority-based escalation rules
- Confidence threshold filtering
- Duplicate suppression (time and distance based)
- Delivery confirmation and retry logic

### 2.3 Data Models

#### Fire Detection Event

```json
{
  "id": "uuid-v4",
  "timestamp": "ISO-8601 datetime UTC",
  "location": {
    "latitude": "decimal degrees",
    "longitude": "decimal degrees",
    "accuracy": "meters",
    "elevation": "meters (optional)"
  },
  "source": {
    "type": "satellite|camera|sensor|manual",
    "sensor_id": "unique identifier",
    "confidence": "0-100"
  },
  "fire_properties": {
    "frp": "megawatts (optional)",
    "area": "square meters (optional)",
    "temperature": "celsius (optional)",
    "smoke_density": "particles/m³ (optional)"
  },
  "weather": {
    "temperature": "celsius",
    "humidity": "percent",
    "wind_speed": "km/h",
    "wind_direction": "degrees",
    "fwi": "fire weather index"
  },
  "status": "detected|confirmed|responding|contained|false_alarm",
  "metadata": {}
}
```

#### Sensor Reading

```json
{
  "sensor_id": "unique identifier",
  "timestamp": "ISO-8601 datetime UTC",
  "type": "smoke|thermal|weather|fuel_moisture",
  "values": {
    "primary_value": "numeric",
    "secondary_values": {}
  },
  "quality": "0-100 (data quality score)",
  "battery_level": "percent (if applicable)",
  "signal_strength": "dbm (if wireless)"
}
```

---

## 3. Hardware Requirements

### 3.1 Server Infrastructure

**Detection Platform Servers**
- 2x Application Servers (load balanced)
  - 32 cores, 128GB RAM, 1TB SSD
  - Ubuntu 22.04 LTS or Red Hat Enterprise Linux 8+
- 2x Database Servers (primary + replica)
  - 16 cores, 64GB RAM, 4TB SSD (NVMe preferred)
  - PostgreSQL 14+ with PostGIS extension
- 1x AI Processing Server
  - NVIDIA GPU (A100, V100, or equivalent)
  - 16 cores, 128GB RAM, 2TB SSD
  - CUDA 11.x, TensorRT for inference acceleration

**Network Infrastructure**
- 10 Gbps network backbone
- Redundant internet connections (minimum 1 Gbps each)
- Firewall and security appliances
- VPN concentrator for field device management
- Network monitoring and logging systems

### 3.2 Camera Systems

**PTZ Camera Specifications**
- Optical: 30x optical zoom minimum
- Thermal: LWIR uncooled microbolometer, 640x480 minimum resolution
- Pan: 360° continuous rotation
- Tilt: -90° to +45° minimum range
- Housing: IP67 rated, operating temperature -30°C to +60°C
- Network: Gigabit Ethernet, H.265 encoding
- Power: PoE++ (90W) or solar + battery backup

**Deployment Guidelines**
- Tower-mounted at 20-50m height
- Coverage radius: 5-15 km depending on terrain
- Strategic placement on ridge tops and high points
- Minimum 50 cameras in PHASE 1, expanding to full coverage in later phases
- GPS synchronization for accurate geolocation

### 3.3 Ground Sensors

**Smoke Detector Stations**
- Optical scattering nephelometer
- Laser: 635nm wavelength, <5mW power
- Sensitivity: 0.01-100 mg/m³ particle concentration
- Accuracy: ±10% at 1 mg/m³
- Communication: LoRaWAN (EU868/US915)
- Power: Solar panel (10W) + lithium battery (20Ah)
- Operating temperature: -20°C to +50°C

**Weather Stations**
- Temperature: ±0.5°C accuracy
- Relative humidity: ±2% accuracy
- Wind speed: ±0.5 m/s accuracy, 0-50 m/s range
- Wind direction: ±5° accuracy
- Precipitation: 0.2mm resolution tipping bucket
- Solar radiation: CMP3 or equivalent pyranometer
- Data logging: 10-minute intervals
- Communication: Cellular (4G/LTE) or LoRaWAN

**LoRaWAN Gateways**
- 8-channel gateway minimum
- Coverage: 2-15 km depending on terrain
- Backhaul: Cellular 4G/LTE or Ethernet
- Power: PoE or solar + battery
- Deployment density: 1 gateway per 100-400 km²

---

## 4. Software Requirements

### 4.1 Operating Systems and Platforms

**Server Operating Systems**
- Ubuntu 22.04 LTS (recommended for cost and compatibility)
- Red Hat Enterprise Linux 8+ (for enterprise environments)
- Security hardening: CIS benchmarks compliance
- Regular security updates and patch management

**Container Orchestration**
- Docker 20.10+ for containerization
- Kubernetes 1.25+ for orchestration (optional in PHASE 1, required PHASE 2+)
- Helm 3.x for package management

### 4.2 Core Software Stack

**Databases**
- PostgreSQL 14+ with PostGIS 3.x (primary datastore)
- TimescaleDB extension for time-series sensor data
- Redis 6.x+ for caching and real-time data
- Elasticsearch 8.x for log aggregation and search

**Web Framework and API**
- Backend: Node.js 18 LTS + Express.js 4.x OR Python 3.11 + FastAPI
- API Documentation: OpenAPI 3.x (Swagger)
- Authentication: OAuth 2.0 + JWT tokens
- Rate limiting: Redis-based token bucket

**Message Queue**
- Apache Kafka 3.x OR RabbitMQ 3.x
- At-least-once delivery semantics
- Topic partitioning for scalability
- Dead letter queues for failed message handling

**AI/ML Framework**
- TensorFlow 2.x OR PyTorch 1.x for model development
- TensorRT for optimized inference
- MLflow for experiment tracking and model registry
- NVIDIA Triton Inference Server (optional, for advanced deployments)

**Frontend Dashboard**
- React 18+ with TypeScript
- Leaflet or Mapbox GL for mapping
- D3.js for data visualization
- WebSocket for real-time updates

### 4.3 Third-Party Integrations

**Satellite Data Sources**
- NASA FIRMS API (https://firms.modaps.eosdis.nasa.gov/api/)
- LANCE NRT data feeds
- Google Earth Engine (for Landsat/Sentinel processing)
- Copernicus Open Access Hub (for Sentinel data)

**Communication Services**
- Twilio (SMS/Voice) or equivalent
- SendGrid (Email) or equivalent
- Firebase Cloud Messaging (Mobile push) or equivalent

**Weather Data**
- NOAA National Weather Service API
- OpenWeatherMap API (backup/supplemental)
- Local meteorological agency APIs

---

## 5. Implementation Roadmap

### 5.1 Months 1-3: Foundation

**Infrastructure Setup**
- Procure and install server hardware (or provision cloud infrastructure)
- Configure network, security, and monitoring
- Set up development, staging, and production environments
- Establish CI/CD pipelines (GitLab CI, GitHub Actions, or Jenkins)
- Configure backup and disaster recovery procedures

**Team Assembly and Training**
- Hire or designate: System Administrator, Backend Developer, Frontend Developer, AI/ML Engineer
- Conduct initial training on fire behavior and detection principles
- Establish communication protocols and on-call rotation
- Define roles and responsibilities (RACI matrix)

**Initial Software Development**
- Set up code repositories and version control
- Implement basic data ingestion for NASA FIRMS
- Develop core database schema
- Create simple web dashboard for system monitoring
- Implement basic authentication and access control

### 5.2 Months 4-6: Core Detection

**Satellite Integration**
- Complete MODIS and VIIRS data ingestion
- Implement FRP calculation and confidence scoring
- Develop data quality checks and validation
- Create automated processing pipeline
- Archive 5 years of historical data for model training and validation

**Camera Deployment (Phase 1A)**
- Install first 10-20 cameras in highest priority locations
- Configure video streaming and storage
- Integrate cameras with platform
- Test remote control and monitoring capabilities
- Establish maintenance procedures

**AI Model Development**
- Collect and label training dataset (minimum 50,000 images)
- Train initial CNN model for smoke/fire detection
- Validate on held-out test set (target: 85% detection, <10% false positive)
- Deploy model to production with version control
- Set up continuous monitoring of model performance

### 5.3 Months 7-9: Sensor Networks

**Ground Sensor Deployment**
- Install LoRaWAN gateways in coverage areas
- Deploy smoke detection stations (minimum 100 units)
- Deploy weather stations (minimum 20 units)
- Configure data transmission and validation
- Establish sensor maintenance schedule and procedures

**Data Fusion**
- Implement Bayesian fusion algorithms
- Develop confidence aggregation logic
- Create duplicate detection and elimination
- Test fusion with synthetic and real data
- Tune thresholds for optimal performance

**Alert System Development**
- Implement SMS, email, and push notification channels
- Create alert routing logic based on jurisdiction and priorities
- Develop web dashboard with real-time alerts
- Build API webhooks for external system integration
- Test end-to-end alert delivery with simulated fires

### 5.4 Months 10-12: Integration and Testing

**System Integration**
- Complete integration of all data sources
- Implement full end-to-end detection pipeline
- Develop comprehensive API documentation
- Create admin interfaces for system configuration
- Finalize user authentication and authorization

**Testing and Validation**
- Conduct unit testing (>80% code coverage target)
- Perform integration testing of all components
- Execute end-to-end testing with simulated fire scenarios
- Conduct load testing (target: 100 concurrent fires, 10,000 sensor readings/minute)
- Security testing and penetration testing

**Training and Documentation**
- Develop user manuals and SOPs
- Conduct operator training sessions
- Create video tutorials and quick reference guides
- Document API for third-party integrators
- Establish help desk and support procedures

### 5.5 Months 13-18: Pilot Operation and Refinement

**Pilot Deployment**
- Go-live with full system in limited geographic area
- Monitor performance metrics daily
- Collect user feedback from operators and stakeholders
- Identify and document issues for resolution
- Begin 24/7 monitoring coverage

**Performance Optimization**
- Tune detection algorithms based on operational data
- Optimize database queries and indexing
- Improve system responsiveness and latency
- Refine alert routing logic based on feedback
- Update AI models with new training data

**Expansion Planning**
- Evaluate pilot results against success criteria
- Identify gaps and areas for improvement
- Plan camera and sensor expansion for full coverage
- Prepare budget and timeline for PHASE 2
- Document lessons learned and best practices

---

## 6. Performance Metrics

### 6.1 Detection Performance

- **Detection Rate:** Percentage of actual fires detected by the system
  - Target: >90% for fires >100m²
  - Measurement: Comparison against ground truth (confirmed fires)

- **False Alarm Rate:** Percentage of alerts that are not actual fires
  - Target: <10%
  - Measurement: Alert verification by responders

- **Detection Latency:** Time from fire ignition to system detection
  - Target: <30 minutes (average)
  - Measurement: Estimated ignition time vs. detection timestamp

- **Alert Delivery Time:** Time from detection to alert delivery
  - Target: <5 minutes
  - Measurement: Detection timestamp vs. delivery confirmation

### 6.2 System Performance

- **System Uptime:** Percentage of time system is operational
  - Target: >95%
  - Measurement: Automated monitoring with 1-minute granularity

- **API Response Time:** Time to respond to API requests
  - Target: <500ms (95th percentile)
  - Measurement: Application performance monitoring (APM)

- **Data Processing Throughput:** Sensor readings processed per second
  - Target: >1,000 readings/second
  - Measurement: Message queue metrics

- **Storage Utilization:** Percentage of available storage used
  - Target: <80% (with automated alerts at 70%)
  - Measurement: Disk monitoring tools

### 6.3 Operational Metrics

- **Camera Availability:** Percentage of cameras operational
  - Target: >90%
  - Measurement: Automated camera health checks

- **Sensor Availability:** Percentage of ground sensors operational
  - Target: >85%
  - Measurement: Sensor heartbeat monitoring

- **Alert Acknowledgment Rate:** Percentage of alerts acknowledged by recipients
  - Target: >95%
  - Measurement: User acknowledgment tracking

- **Response Time:** Time from alert to initial response deployment
  - Target: <20 minutes (fire department/forest service)
  - Measurement: Incident response tracking integration

---

## 7. Security and Compliance

### 7.1 Security Requirements

**Network Security**
- Firewall rules restricting access to necessary ports only
- VPN required for all remote administrative access
- Intrusion Detection System (IDS) monitoring
- Regular security scans and vulnerability assessments
- DDoS protection and rate limiting

**Application Security**
- All communications encrypted (TLS 1.2+ for HTTPS, encrypted messaging)
- Strong password policies (minimum 12 characters, complexity requirements)
- Multi-factor authentication (MFA) for administrative access
- Regular security updates and patch management
- Secure coding practices and code review

**Data Security**
- Encryption at rest for sensitive data (AES-256)
- Encryption in transit for all communications (TLS)
- Access control lists (ACLs) enforcing least privilege
- Audit logging of all data access and modifications
- Regular backup with encryption and off-site storage

### 7.2 Privacy and Data Protection

- Compliance with applicable data protection regulations (GDPR, CCPA, etc.)
- Data minimization: collect only necessary information
- User consent for personal data collection
- Data retention policies and automated deletion
- Privacy impact assessments for new features

### 7.3 Compliance and Standards

- ISO 27001 Information Security Management System (target)
- NIST Cybersecurity Framework alignment
- Industry best practices for fire detection systems
- Regular compliance audits and assessments
- Documentation of compliance efforts

---

## 8. Budget and Resource Estimates

### 8.1 Hardware Costs

- Servers and Infrastructure: $150,000 - $300,000
- Camera Systems (50 units): $250,000 - $500,000
- Ground Sensors (100 smoke + 20 weather): $200,000 - $400,000
- LoRaWAN Gateways (20 units): $40,000 - $80,000
- Network Equipment: $50,000 - $100,000
- **Total Hardware: $690,000 - $1,380,000**

### 8.2 Software and Services

- Software Licenses: $50,000 - $100,000 (if using commercial components)
- Cloud Services (if applicable): $60,000 - $120,000/year
- Communication Services (SMS, email): $20,000 - $40,000/year
- Third-Party APIs and Data: $10,000 - $30,000/year
- **Total Software/Services (Year 1): $140,000 - $290,000**

### 8.3 Personnel

- Project Manager (18 months): $135,000 - $180,000
- System Administrator (18 months): $105,000 - $150,000
- Backend Developers (2, 18 months): $180,000 - $270,000
- Frontend Developer (18 months): $90,000 - $135,000
- AI/ML Engineer (18 months): $135,000 - $180,000
- Field Technicians (2, installation/maintenance): $90,000 - $120,000
- **Total Personnel: $735,000 - $1,035,000**

### 8.4 Other Costs

- Installation and Deployment: $100,000 - $200,000
- Training and Documentation: $30,000 - $60,000
- Testing and Validation: $40,000 - $80,000
- Contingency (10%): $169,500 - $304,500
- **Total Other Costs: $339,500 - $644,500**

### 8.5 Total PHASE 1 Budget

**$1,904,500 - $3,349,500** (approximately $2-5 million)

*Note: Costs vary significantly based on geographic region, existing infrastructure, labor costs, and specific requirements. This estimate assumes a mid-sized jurisdiction (1,000-5,000 km²) in a developed country.*

---

## 9. Risk Management

### 9.1 Technical Risks

**Risk: AI Model Performance Below Target**
- Likelihood: Medium
- Impact: High
- Mitigation: Extensive training data collection, multiple model architectures, continuous monitoring and improvement
- Contingency: Rule-based detection as fallback, phased rollout with human verification

**Risk: Sensor/Camera Failures in Remote Locations**
- Likelihood: High
- Impact: Medium
- Mitigation: Redundant coverage, robust hardware selection, preventive maintenance schedule
- Contingency: Rapid response team for repairs, spare parts inventory

**Risk: Communication Network Outages**
- Likelihood: Medium
- Impact: High
- Mitigation: Multiple communication channels (cellular, satellite backup), local edge processing
- Contingency: Store-and-forward capability, degraded mode operation

### 9.2 Operational Risks

**Risk: Insufficient Training/Adoption**
- Likelihood: Medium
- Impact: High
- Mitigation: Comprehensive training program, user-friendly interfaces, ongoing support
- Contingency: Extended training period, additional user support resources

**Risk: Budget Overruns**
- Likelihood: Medium
- Impact: Medium
- Mitigation: Detailed cost estimation, regular budget tracking, phased approach allowing for adjustment
- Contingency: Descope non-critical features, extend timeline to spread costs

**Risk: Stakeholder Resistance**
- Likelihood: Low-Medium
- Impact: Medium
- Mitigation: Early stakeholder engagement, demonstration of value, pilot success communication
- Contingency: Focus on high-value quick wins, adjust approach based on feedback

### 9.3 Environmental Risks

**Risk: Extreme Weather Damaging Equipment**
- Likelihood: Medium
- Impact: Medium
- Mitigation: Ruggedized equipment selection, proper installation, lightning protection
- Contingency: Insurance coverage, rapid replacement procedures

**Risk: Wildfires Damaging Infrastructure**
- Likelihood: Low
- Impact: High
- Mitigation: Strategic placement away from highest fire risk, fire-resistant enclosures
- Contingency: Redundant coverage, rapid redeployment capability

---

## 10. Success Criteria and Go-Live Checklist

### 10.1 Technical Criteria

- ✅ All server infrastructure operational and monitored
- ✅ Satellite data ingestion functioning with <4 hour latency
- ✅ Minimum 50 cameras deployed and operational
- ✅ Minimum 100 ground sensors deployed and transmitting
- ✅ AI detection model achieving >85% detection rate, <10% false positive
- ✅ Alert delivery through all channels (SMS, email, push, webhook)
- ✅ Web dashboard operational and accessible
- ✅ API documentation complete and published
- ✅ System uptime >95% over 30-day testing period
- ✅ Security audit passed with no critical vulnerabilities

### 10.2 Operational Criteria

- ✅ All operators trained and certified
- ✅ Standard Operating Procedures documented and reviewed
- ✅ 24/7 monitoring coverage established
- ✅ Incident response procedures tested
- ✅ Help desk and support system operational
- ✅ Maintenance schedules established
- ✅ Emergency backup and recovery procedures tested

### 10.3 Performance Criteria

- ✅ Detection rate >90% validated over 3-month pilot
- ✅ False alarm rate <10% validated over 3-month pilot
- ✅ Average alert delivery time <5 minutes
- ✅ Average detection latency <30 minutes
- ✅ User satisfaction survey >80% positive
- ✅ All critical bugs resolved
- ✅ Performance metrics dashboard operational

---

## 11. Transition to PHASE 2

### 11.1 PHASE 1 Closeout Activities

- Complete final testing and validation
- Document all lessons learned
- Conduct stakeholder review and acceptance
- Archive all project documentation
- Transfer operations to steady-state team
- Celebrate successes and recognize contributions

### 11.2 PHASE 2 Preparation

- Review PHASE 1 performance against targets
- Identify gaps and areas for improvement
- Update requirements based on operational experience
- Plan expanded camera and sensor coverage
- Initiate procurement for PHASE 2 equipment
- Develop detailed PHASE 2 implementation plan

### 11.3 Continuous Improvement

- Establish regular performance review meetings (monthly)
- Create continuous improvement process for system enhancements
- Maintain AI model retraining schedule (quarterly minimum)
- Update documentation based on operational changes
- Collect and prioritize feature requests from users
- Monitor technology landscape for emerging capabilities

---

**Document Control**

- **Version:** 1.0
- **Author:** WIA Standards Committee
- **Approved By:** [To be completed]
- **Approval Date:** [To be completed]
- **Next Review Date:** [6 months after go-live]

**Change History**

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-01-28 | WIA Committee | Initial specification |

---

弘益人間 (Hongik Ingan) - Benefit All Humanity

© 2025 World Certification Industry Association (WIA) / SmileStory Inc.


## Annex E — Implementation Notes for PHASE-1

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-1.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.

## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.

The roadmap is reviewed at every minor version of this PHASE document, and the review outcomes are recorded in the version-history table at the start of the document.

## Annex G — Test Vectors and Conformance Evidence

This annex describes how implementations capture and publish conformance
evidence for PHASE-1. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-1/`. Implementations claiming
  conformance MUST run all vectors in CI and publish the resulting
  pass/fail matrix in their compliance package.
- **Evidence package** — the compliance package is a tarball containing
  the SBOM (CycloneDX 1.5 or SPDX 2.3), the OpenAPI document, the test
  vector matrix, and a signed manifest. Signatures use Sigstore (DSSE
  envelope, Rekor transparency log entry) so that downstream consumers
  can verify provenance without trusting a private CA.
- **Quarterly recheck** — implementations re-publish the evidence package
  every quarter even if no source change occurred, so that consumers can
  detect environmental drift (compiler updates, dependency updates, OS
  updates) without polling vendor changelogs.
- **Cross-vendor crosswalk** — the WIA Standards working group maintains a
  crosswalk that maps each vector to the equivalent assertion in adjacent
  industry programs (where one exists), so an implementer that already
  certifies under one program can show conformance to PHASE-1 with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-1 does not require bespoke
auditor tooling.

## Annex H — Versioning and Deprecation Policy

This annex codifies the versioning and deprecation policy for PHASE-1.
It is non-normative; the rules below describe the policy that the WIA
Standards working group commits to when amending this PHASE document.

- **Semantic versioning** — major / minor / patch components follow
  Semantic Versioning 2.0.0 (https://semver.org/spec/v2.0.0.html).
  Major bump indicates a backwards-incompatible change to a normative
  requirement; minor bump indicates new normative requirements that do
  not break existing implementations; patch bump indicates editorial
  changes only (clarifications, typo fixes, formatting).
- **Deprecation window** — when a normative requirement is removed or
  altered in a backwards-incompatible way, the prior major version is
  maintained in parallel for at least 180 days. During the parallel
  window, both major versions are marked Stable in the WIA Standards
  registry and either may be cited as "WIA-conformant".
- **Sunset notification** — deprecated major versions enter a 12-month
  sunset window during which the WIA registry marks the version as
  Deprecated. The deprecation entry includes a migration note pointing
  to the replacement requirement(s) and an explanation of why the
  change was made.
- **Editorial errata** — patch-level errata are issued without a
  deprecation window because they do not change normative behaviour.
  Errata are tracked in a public errata register and each entry is
  signed by the WIA Standards working group chair.
- **Implementation changelog mapping** — implementations SHOULD publish
  a changelog mapping each PHASE version they support to the specific
  build, container digest, or SDK version that satisfies the version.
  This allows downstream auditors to verify version conformance without
  re-running the entire test matrix on every release.

The policy is reviewed at the same cadence as the PHASE document and
any changes to the policy itself are tracked in the version-history
table at the start of the document.

## Annex I — Interoperability Profiles

This annex describes how implementations declare interoperability profiles
for PHASE-1. The profile mechanism is non-normative and exists so that
deployments of varying scope (single tenant, regional cluster, federated
network) can advertise the subset of normative requirements they satisfy
without misrepresenting partial conformance as full conformance.

- **Profile manifest** — every implementation publishes a profile manifest
  in JSON. The manifest enumerates the normative requirement IDs from this
  PHASE that are satisfied (`status: "supported"`), partially satisfied
  (`status: "partial"`, with a reason field), or excluded
  (`status: "excluded"`, with a justification). The manifest is signed
  using the same Sigstore key used for the SBOM in Annex G.
- **Federation profile** — federated deployments publish an aggregated
  manifest summarizing the union and intersection of member-implementation
  profiles. The aggregated manifest is consumed by directory services so
  that callers can route a request to the least common denominator profile
  required for an interaction.
- **Backwards-profile compatibility** — when a deployment migrates from one
  profile to a wider profile, the prior profile manifest remains valid and
  signed for the deprecation window defined in Annex H. This preserves
  audit traceability for auditors evaluating long-term interoperability.
- **Profile registry** — the WIA Standards working group maintains a
  public registry of named profiles. Common deployment shapes (e.g.,
  "Edge-only", "Federated-with-replay") are added to the registry by
  consensus. Registry entries are immutable; new shapes are added under
  new names rather than amending existing entries.
- **Profile versioning** — profile names are versioned with the same
  Semantic Versioning rules described in Annex H. A deployment that
  advertises `WIA-P1-Edge-only/2` is asserting conformance with
  the second major version of the named profile, not the second deployment
  of an unversioned profile.

The profile mechanism is intentionally lightweight; it is meant to make
real deployment shapes visible without forcing every deployment to
satisfy every normative requirement.

## Annex J — Reference Implementation Topology

The reference implementation topology described in this annex is
non-normative; it documents the deployment shape that the WIA
Standards working group used to validate the test vectors in Annex G
and is intended as a starting point, not a recommendation against
alternative topologies.

- **Single-tenant edge** — one runtime per organization, no shared
  state. Used for early-pilot deployments where conformance evidence
  is published manually. Sufficient for PHASE-1 validation when the
  organization signs the manifest itself.
- **Multi-tenant gateway** — one shared runtime serves multiple
  tenants via header-based isolation. Typically backed by a
  rate-limited gateway (Envoy or NGINX) and a shared OAuth 2.1
  identity provider. The manifest is per-tenant; the runtime
  publishes a federation manifest that aggregates tenant manifests.
- **Federated mesh** — multiple runtimes peer to one another and
  publish their manifests to a directory service. Each peer signs
  its own manifest; the directory service signs the aggregated
  index. This is the topology used by cross-organization deployments
  that need to compose conformance.
- **Air-gapped batch** — no network connection between the runtime
  and the directory service. The runtime emits a signed evidence
  package on each batch and the operator transports the package via
  out-of-band channels. This is the topology used by regulators that
  prohibit live connectivity from sensitive environments.

Implementations declare their topology in the manifest (see Annex I).
A topology change MUST be reflected in a new manifest signature; the
prior topology's manifest remains valid for the deprecation window
described in Annex H to preserve audit traceability.
