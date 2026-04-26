# WIA Building Energy Management Standard
## Phase 3: Protocol Specification
### Version 1.0

---

## Document Information

- **Standard**: WIA-BEMS (Building Energy Management Standard)
- **Phase**: 3 - Protocol
- **Version**: 1.0.0
- **Status**: Published
- **Date**: January 2025
- **Organization**: WIA (World Certification Industry Association)

---

## 1. Introduction

Phase 3 defines standardized protocols for coordinated workflows in building energy management. While Phase 2 APIs enable individual transactions, Phase 3 protocols orchestrate sequences of actions to accomplish complex objectives like automated control, predictive maintenance, fault detection, and energy optimization.

---

## 2. Real-Time Monitoring Protocol

### 2.1 MQTT Topic Structure

```
buildings/{building_id}/{category}/{device_id}/{metric}
```

Examples:
- `buildings/BLDG-001/energy/METER-E-301/power`
- `buildings/BLDG-001/hvac/AHU-301/status`
- `buildings/BLDG-001/alerts/high-demand`

### 2.2 QoS Levels

- **QoS 0** (At most once): Non-critical updates, high frequency data
- **QoS 1** (At least once): Important measurements, alerts
- **QoS 2** (Exactly once): Critical commands, financial data

### 2.3 Retained Messages

Status messages should be retained to allow new subscribers to receive last known state immediately.

### 2.4 Alert Generation

Alerts published to `buildings/{building_id}/alerts/{alert_type}`:

```json
{
  "alert_id": "alert-12345",
  "severity": "warning | critical | info",
  "type": "demand_threshold_exceeded",
  "triggered_at": "2025-01-15T14:30:00Z",
  "current_value": 205.3,
  "threshold": 200.0,
  "duration_seconds": 300,
  "recommendations": ["Action 1", "Action 2"]
}
```

---

## 3. Automated Control Sequences

### 3.1 Occupancy-Based HVAC Control

**Workflow**:
1. Monitor occupancy sensors
2. Detect state change (occupied ↔ vacant)
3. Implement delay to avoid false triggers
4. Adjust setpoints based on state
5. Verify equipment response
6. Monitor performance
7. Log results

**Configuration Schema**:
```json
{
  "sequence_type": "occupancy_based_hvac_control",
  "zone": "Conference-Room-A",
  "occupancy_sensor": "OCC-301-A",
  "controlled_equipment": ["AHU-301", "VAV-301-A"],
  "delays": {
    "occupied_to_vacant_minutes": 15,
    "vacant_to_occupied_minutes": 0
  },
  "setpoints": {
    "occupied": {"temperature_c": 22.0, "airflow_cfm": 1200},
    "vacant": {"temperature_c": 26.0, "airflow_cfm": 400}
  }
}
```

### 3.2 Demand Response Protocol

**Event Notification**:
Utility sends DR event via OpenADR or custom API

**Building Response Workflow**:
1. Receive DR event notification
2. Calculate reduction target
3. Prioritize load reduction actions
4. Execute sequence progressively
5. Monitor compliance
6. Adjust if needed
7. Restore after event
8. Report performance

**Action Priority**:
1. Comfort impact: none → low → medium → high
2. Cost effectiveness: high → medium → low
3. Technical feasibility: ready → available → planned

---

## 4. Predictive Maintenance Protocol

### 4.1 Condition Monitoring

**Monitored Parameters by Equipment Type**:

| Equipment | Parameters | Indicators | Lead Time |
|-----------|-----------|-----------|-----------|
| Chillers | Efficiency, pressure, vibration | Declining COP | 2-4 weeks |
| AHUs | Fan power, filter ΔP, airflow | Rising power/ΔP | 1-2 weeks |
| Boilers | Combustion efficiency, stack temp | Declining efficiency | 2-6 weeks |
| Pumps | Power, flow, vibration | Power increase | 2-4 weeks |

### 4.2 Maintenance Alert Schema

```json
{
  "maintenance_alert_id": "maint-alert-456",
  "alert_type": "predictive_maintenance",
  "equipment_id": "AHU-301",
  "issue_detected": "filter_pressure_drop_high",
  "detection_time": "2025-01-15T14:30:00Z",
  "analysis": {
    "current_value": 225,
    "normal_value": 100,
    "max_value": 250,
    "time_to_max_estimated_days": 7,
    "confidence": 0.92
  },
  "recommendation": {
    "action": "replace_air_filters",
    "urgency": "schedule_within_week",
    "estimated_duration_hours": 2,
    "projected_cost_usd": 450
  }
}
```

---

## 5. Fault Detection and Diagnostics (FDD)

### 5.1 Common Fault Patterns

| Fault Type | Indicators | Impact | Priority |
|------------|-----------|--------|----------|
| Simultaneous Heating/Cooling | Both valves open | 30-50% waste | High |
| Night/Weekend Operation | Running when unoccupied | 20-40% waste | High |
| Stuck Damper/Valve | No position change | 10-30% waste | Medium |
| Sensor Drift | Inconsistent readings | 5-15% waste | Medium |
| Short Cycling | Frequent on/off | 10-20% waste | High |

### 5.2 FDD Alert Schema

```json
{
  "fdd_alert_id": "fdd-alert-789",
  "fault_type": "simultaneous_heating_cooling",
  "severity": "high",
  "detection_time": "2025-01-15T14:30:00Z",
  "evidence": {
    "heating_valve_position_percent": 45,
    "cooling_valve_position_percent": 30,
    "zone_temperature_c": 22.5
  },
  "diagnosis": {
    "root_cause": "control_logic_error",
    "probable_causes": [{
      "cause": "Misconfigured deadband",
      "probability": 0.65,
      "test": "Check deadband setting"
    }]
  },
  "impact": {
    "energy_waste_kwh_per_day": 85,
    "duration_days": 3,
    "total_wasted_usd": 38.25
  },
  "recommendation": {
    "immediate_action": "Set deadband to 1.0°C",
    "verification": "Monitor valve positions for 24 hours"
  }
}
```

---

## 6. Energy Optimization Protocols

### 6.1 Model Predictive Control (MPC)

**Workflow**:
1. Gather inputs (weather forecast, occupancy schedule, prices)
2. Run building thermal model
3. Optimize control strategy over planning horizon
4. Validate constraints (comfort, equipment limits)
5. Execute first interval of optimal sequence
6. Monitor actual vs. predicted performance
7. Update model based on observations
8. Repeat every optimization interval

**Optimization Objective Functions**:
- Minimize cost
- Minimize energy consumption
- Minimize peak demand
- Minimize carbon emissions
- Multi-objective optimization with weights

### 6.2 Load Shifting Strategies

| Strategy | When Used | Savings | Requirements |
|----------|-----------|---------|--------------|
| Thermal Precooling | Before peak periods | 15-30% peak reduction | Good thermal mass |
| Ice Storage | Night/day arbitrage | 40-60% peak reduction | Thermal storage |
| Battery Arbitrage | Price differentials | 30-50% peak reduction | Battery system |
| Equipment Scheduling | Flexible loads | 10-20% cost reduction | Non-critical equipment |

---

## 7. Protocol Integration and Coordination

### 7.1 Priority Hierarchy

1. **Life Safety** (highest)
2. **Equipment Protection**
3. **Occupant Comfort**
4. **Demand Response**
5. **Energy Optimization**
6. **Routine Operations** (lowest)

### 7.2 Conflict Resolution

When protocols conflict, higher priority wins. Conflicts logged for review:

```json
{
  "conflict_id": "conflict-456",
  "protocols": [
    {"name": "demand_response", "priority": 4},
    {"name": "comfort_optimization", "priority": 3}
  ],
  "resolution": {
    "winning_protocol": "demand_response",
    "compromise": {"setpoint_c": 24.0, "duration_minutes": 60}
  }
}
```

---

## 8. State Management

### 8.1 Protocol State Storage

Protocols maintain state in persistent storage:
- Current sequence step
- Historical performance data
- Model parameters
- Override conditions

### 8.2 State Recovery

Protocols must recover gracefully after system restart, using last known state and verifying current conditions before resuming operations.

---

## 9. Performance Metrics

### 9.1 Protocol Performance Tracking

Track for each protocol:
- Execution count
- Success rate
- Average savings
- Comfort impact
- Override frequency

### 9.2 Continuous Improvement

Use performance data to:
- Refine model parameters
- Adjust thresholds
- Improve predictions
- Update priorities

---

## 10. Implementation Requirements

### 10.1 Reliability

- Graceful degradation when data unavailable
- Timeout handling for commands
- Exponential backoff for retries
- Health monitoring and reporting
- Fallback to safe defaults on failure

### 10.2 Testing

- Simulation testing before production
- Progressive rollout
- Close monitoring during initial deployment
- Comprehensive logging
- Performance verification

---

**© 2025 SmileStory Inc. / WIA**
**弘益人間 - Benefit All Humanity**


## Annex E — Implementation Notes for PHASE-3-PROTOCOL

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-3-PROTOCOL.

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
evidence for PHASE-3-PROTOCOL. The procedure is non-normative; it standardizes the
shape of evidence so that auditors and downstream integrators can compare
implementations without re-running the full test matrix.

- **Test vectors** — every normative requirement in this PHASE has at least
  one positive vector and one negative vector under
  `tests/phase-vectors/phase-3-protocol/`. Implementations claiming
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
  certifies under one program can show conformance to PHASE-3-PROTOCOL with
  reduced incremental effort.
- **Negative-result reporting** — vendors MUST report negative results
  with the same fidelity as positive ones. A test that is skipped without
  recorded justification is treated by auditors as a failure.

These conventions are intended to make conformance evidence portable and
machine-readable so that adoption of PHASE-3-PROTOCOL does not require bespoke
auditor tooling.
