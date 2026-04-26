# WIA Building Energy Management Standard
## Phase 4: Integration Specification
### Version 1.0

---

## Document Information

- **Standard**: WIA-BEMS (Building Energy Management Standard)
- **Phase**: 4 - Integration
- **Version**: 1.0.0
- **Status**: Published
- **Date**: January 2025
- **Organization**: WIA (World Certification Industry Association)

---

## 1. Introduction

Phase 4 enables buildings to integrate with external systems, transforming individual intelligent buildings into active participants in the broader energy ecosystem. This phase defines integration frameworks for smart grids, renewable energy sources, building automation systems, EV charging, and certification platforms.

---

## 2. Smart Grid Integration

### 2.1 OpenADR 2.0b Protocol

WIA-BEMS implements OpenADR (Open Automated Demand Response) for utility communication.

**Event Reception**:
```xml
<oadrDistributeEvent>
  <eiEvent>
    <eventDescriptor>
      <eventID>utility-dr-2025-01-15-001</eventID>
      <modificationNumber>0</modificationNumber>
      <priority>0</priority>
      <eiMarketContext>http://MarketContext1</eiMarketContext>
      <createdDateTime>2025-01-15T13:00:00Z</createdDateTime>
    </eventDescriptor>
    <eiActivePeriod>
      <properties>
        <dtstart><date-time>2025-01-15T15:00:00-08:00</date-time></dtstart>
        <duration><duration>PT3H</duration></duration>
      </properties>
    </eiActivePeriod>
    <eiEventSignals>
      <eiEventSignal>
        <intervals>
          <interval>
            <duration><duration>PT1H</duration></duration>
            <signalPayload><payloadFloat><value>2</value></payloadFloat></signalPayload>
          </interval>
        </intervals>
        <signalName>SIMPLE</signalName>
        <signalType>level</signalType>
      </eiEventSignal>
    </eiEventSignals>
  </eiEvent>
</oadrDistributeEvent>
```

**Building Response**:
- optIn: Participate in event
- optOut: Decline participation
- Commitment includes reduction target and confidence level

### 2.2 Grid Services

| Service | Response Time | Duration | Compensation |
|---------|---------------|----------|--------------|
| Peak Demand Reduction | 15-60 minutes | 2-4 hours | $100-300/kW-year |
| Frequency Regulation | <1 second | Continuous | $200-500/kW-year |
| Spinning Reserve | <15 minutes | As needed | $50-150/kW-year |
| Voltage Support | <1 minute | Continuous | $30-100/kW-year |

---

## 3. Renewable Energy Integration

### 3.1 Solar PV Coordination

**Self-Consumption Optimization**:
```json
{
  "optimization_strategy": {
    "priority_order": [
      "building_consumption",
      "battery_charging",
      "grid_export"
    ],
    "battery_schedule": {
      "charge_when": "excess_solar",
      "discharge_when": "peak_prices",
      "reserve_percent": 20
    }
  }
}
```

**Forecast Integration**:
- Solar generation forecast (15-minute intervals, 7-day horizon)
- Building load forecast
- Energy price forecast
- Optimize across forecasts for maximum value

### 3.2 Energy Storage Management

**Operating Modes**:
- Self-consumption: Store excess solar
- Peak shaving: Discharge during demand peaks
- Time-of-use arbitrage: Charge off-peak, discharge on-peak
- Backup power: Reserve capacity for outages
- Frequency regulation: Fast response for grid stability

**State Machine**:
```
Idle → Charging → Idle → Discharging → Idle
       ↓                    ↓
    Grid Export        Grid Import
```

---

## 4. Building Automation System (BAS) Integration

### 4.1 BACnet Integration

**Object Type Mappings**:

| BACnet Object | WIA-BEMS Schema | Mapping |
|---------------|-----------------|---------|
| analog-input | temperature, power, etc. | Direct value mapping with unit conversion |
| analog-output | equipment_control.setpoints | Bidirectional with confirmation |
| binary-input | occupancy, status | Boolean to enum mapping |
| binary-output | equipment_control.commands | Command to binary mapping |
| multi-state-input | equipment modes | Enum to string mapping |

**Example Mapping Configuration**:
```json
{
  "bacnet_device": {
    "device_id": 1001,
    "ip_address": "192.168.1.100",
    "port": 47808
  },
  "object_mappings": [{
    "bacnet_object": {
      "type": "analog-input",
      "instance": 1,
      "property": "present-value",
      "units": "degrees-fahrenheit"
    },
    "wia_bems_mapping": {
      "schema": "temperature",
      "field": "temperature.value",
      "unit_conversion": "fahrenheit_to_celsius"
    }
  }]
}
```

### 4.2 Modbus TCP Integration

**Register Mapping**:
- Holding Registers (40001+): Read/write values
- Input Registers (30001+): Read-only measurements
- Coils (00001+): Binary outputs
- Discrete Inputs (10001+): Binary inputs

---

## 5. Electric Vehicle Charging Integration

### 5.1 Smart Charging Coordination

**Objectives**:
- Avoid exceeding building demand limit
- Utilize excess solar generation
- Minimize charging costs via TOU optimization
- Ensure vehicles charged by departure time

**Scheduling Algorithm**:
1. Gather vehicle requirements (energy needed, departure time)
2. Forecast building load and solar generation
3. Optimize charging schedule considering:
   - Demand limit constraints
   - Time-of-use rates
   - Solar availability
   - Vehicle priorities
4. Dynamically adjust as conditions change

### 5.2 OCPP Integration

Support for Open Charge Point Protocol (OCPP) 1.6/2.0:
- Remote start/stop charging
- Power limit adjustment
- Status monitoring
- Smart charging profiles

---

## 6. Certification and Compliance Integration

### 6.1 LEED/BREEAM Integration

**Automated Data Submission**:
```json
{
  "program": "LEED",
  "required_metrics": {
    "energy_use_intensity_kwh_per_sqm": 85.2,
    "baseline_comparison_percent": -28.5,
    "renewable_energy_percent": 18.5
  },
  "automated_submission": {
    "portfolio_manager_property_id": "12345678",
    "frequency": "monthly",
    "next_submission": "2025-02-05T10:30:00Z"
  }
}
```

### 6.2 Energy Star Portfolio Manager

**Web Services Integration**:
- Automated meter data submission
- Property/meter configuration
- Score retrieval
- Benchmarking reports

API Endpoints:
- POST /meter/{meterId}/consumptionData
- GET /property/{propertyId}/metrics
- GET /property/{propertyId}/energystarScore

---

## 7. Multi-Building Portfolio Management

### 7.1 Portfolio-Level Optimization

**Capabilities**:
- Aggregate energy data across buildings
- Portfolio-level demand response participation
- Shared renewable energy allocation
- Centralized procurement optimization
- Best practice identification and replication

### 7.2 Portfolio Dashboard Integration

**Aggregated Metrics**:
- Total energy consumption and cost
- Average Energy Use Intensity (EUI)
- Portfolio Energy Star score
- Total renewable capacity and generation
- Carbon footprint (Scope 1, 2, 3)

### 7.3 Load Balancing

For portfolios with shared resources:
- Shift loads between buildings
- Optimize shared chiller plants
- Coordinate demand response across portfolio
- Maximize portfolio-wide renewable utilization

---

## 8. Third-Party Platform Integration

### 8.1 Weather Services

**Integration Requirements**:
- Forecast data: 15-minute intervals, 7-day horizon
- Historical data for model training
- Parameters: temperature, humidity, solar irradiance, wind speed
- Update frequency: Hourly minimum

**Recommended Providers**:
- NOAA (National Oceanic and Atmospheric Administration)
- Weather.com API
- OpenWeatherMap
- Tomorrow.io

### 8.2 Energy Market Data

**Real-Time Pricing**:
- ISO/RTO pricing feeds (CAISO, PJM, ERCOT, etc.)
- Day-ahead and real-time markets
- Ancillary service prices
- Renewable energy certificate (REC) prices

### 8.3 Maintenance Management Systems

**Integration Points**:
- Work order creation from predictive maintenance alerts
- Asset hierarchy synchronization
- Maintenance schedule coordination
- Parts inventory integration

**Supported Systems**:
- IBM Maximo
- ServiceNow
- SAP PM
- Oracle Maintenance Cloud

---

## 9. Security and Privacy

### 9.1 External Communication Security

- All external integrations use TLS 1.3+
- Certificate-based authentication where supported
- API keys rotated quarterly minimum
- Network segmentation (DMZ for external interfaces)

### 9.2 Data Privacy

**Privacy-Preserving Techniques**:
- Differential privacy for aggregated data
- Anonymization before external sharing
- Data minimization (share only necessary fields)
- Consent management for occupancy data

### 9.3 Compliance

- GDPR compliance for EU operations
- CCPA compliance for California
- HIPAA compliance for healthcare facilities
- Industry-specific regulations as applicable

---

## 10. Implementation Architecture

### 10.1 Integration Layer Design

**Components**:
- Message Queue: Apache Kafka, RabbitMQ, Azure Event Hub
- API Gateway: Rate limiting, authentication, routing
- Protocol Adapters: BACnet, Modbus, OCPP, OpenADR
- Data Transformers: Format conversion, unit translation
- Error Handlers: Retry logic, dead letter queues

### 10.2 Resilience Patterns

- Circuit breakers for external services
- Graceful degradation when external systems unavailable
- Message persistence during outages
- Automated reconnection with exponential backoff

---

## 11. Testing and Certification

### 11.1 Integration Testing

**Test Scenarios**:
- Normal operation with all systems available
- Degraded operation with external system failures
- Recovery after outage
- High-volume data flow
- Error condition handling

### 11.2 Certification Requirements

Phase 4 certification requires:
- Successful integration with 3+ external system types
- Security audit passed
- Performance benchmarks met
- 48-hour stability test
- Interoperability demonstration

---

## 12. Deployment Considerations

### 12.1 Phased Integration

1. **Phase 4a**: Single integration (e.g., weather service)
2. **Phase 4b**: Add grid integration (OpenADR)
3. **Phase 4c**: BAS integration (BACnet/Modbus)
4. **Phase 4d**: Advanced features (portfolio, EV charging)

### 12.2 Monitoring

**Integration Health Metrics**:
- Connection status (up/down)
- Message throughput
- Error rates
- Latency (p50, p95, p99)
- Data quality scores

---

**© 2025 SmileStory Inc. / WIA**
**弘益人間 - Benefit All Humanity**

This specification is freely implementable without licensing fees or royalties.


## Annex E — Implementation Notes for PHASE-4-INTEGRATION

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-4-INTEGRATION.

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
evidence for PHASE-4-INTEGRATION. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-4-integration/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-4-INTEGRATION with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-4-INTEGRATION does not require bespoke
auditor tooling.
