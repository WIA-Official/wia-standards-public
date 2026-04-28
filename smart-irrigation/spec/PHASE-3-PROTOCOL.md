# WIA-AGRI-007: Smart Irrigation Standard
## Phase 3: Control Protocol Specification

### 3.1 Overview

Phase 3 defines the control protocols for valve actuation, zone sequencing, safety interlocks, and autonomous irrigation decision-making.

**Duration**: 6-10 months
**Key Outcome**: Standardized control logic and safety protocols

### 3.2 Valve Control Protocol

#### 3.2.1 Valve State Machine

```
[CLOSED] ──open──> [OPENING] ──fully_open──> [OPEN]
    ^                                          |
    |                                          |
    └────────────── close ─────────────────────┘
                      |
                      v
                 [CLOSING] ──fully_closed──> [CLOSED]
```

**States**:
- **CLOSED**: Valve fully closed, no flow
- **OPENING**: Valve transitioning to open (2-5 seconds)
- **OPEN**: Valve fully open, active irrigation
- **CLOSING**: Valve transitioning to closed (2-5 seconds)
- **FAULT**: Error state, requires intervention

#### 3.2.2 Valve Command Structure

```json
{
  "command_id": "cmd-12345",
  "valve_id": "valve-zone-1",
  "action": "open|close|modulate",
  "parameters": {
    "target_position": 100,
    "ramp_time_seconds": 3,
    "flow_rate_setpoint_lpm": 150,
    "pressure_limit_kpa": 400
  },
  "priority": "normal|high|emergency",
  "timeout_seconds": 300,
  "retry_count": 3,
  "issued_at": "2025-01-15T18:00:00Z"
}
```

#### 3.2.3 Valve Response

```json
{
  "command_id": "cmd-12345",
  "valve_id": "valve-zone-1",
  "status": "success|failed|timeout",
  "current_state": "open",
  "position_percent": 100,
  "flow_rate_lpm": 148.5,
  "pressure_kpa": 285,
  "execution_time_ms": 2850,
  "completed_at": "2025-01-15T18:00:02.85Z"
}
```

### 3.3 Zone Sequencing Protocol

#### 3.3.1 Sequential Irrigation

To prevent pressure drops, zones are irrigated sequentially:

```python
# Pseudo-code for zone sequencing
for zone in sorted_zones_by_priority:
    if system_pressure > minimum_threshold:
        open_valve(zone.valve_id)
        wait(zone.duration_minutes)
        close_valve(zone.valve_id)
        wait(inter_zone_delay_minutes)
    else:
        log_error("Insufficient pressure for zone", zone.zone_id)
        skip_to_next_zone()
```

#### 3.3.2 Zone Priority Levels

- **Priority 1 (Emergency)**: Critical crops during drought
- **Priority 2 (High)**: High-value crops or stressed plants
- **Priority 3 (Normal)**: Standard irrigation schedule
- **Priority 4 (Low)**: Maintenance irrigation, landscaping

#### 3.3.3 Simultaneous Zone Operation

For systems with adequate pressure:

```json
{
  "simultaneous_zones": 3,
  "max_total_flow_lpm": 500,
  "pressure_monitoring_interval_seconds": 10,
  "auto_adjust_flow": true,
  "zones": [
    {"zone_id": "zone-1", "flow_allocation_lpm": 150},
    {"zone_id": "zone-2", "flow_allocation_lpm": 175},
    {"zone_id": "zone-3", "flow_allocation_lpm": 175}
  ]
}
```

### 3.4 Flow Rate Control

#### 3.4.1 Proportional Flow Control

For systems with modulating valves:

```
flow_error = target_flow - actual_flow
valve_position_adjustment = Kp × flow_error + Ki × ∫(flow_error)dt + Kd × d(flow_error)/dt

where:
  Kp = Proportional gain (typical: 0.5)
  Ki = Integral gain (typical: 0.1)
  Kd = Derivative gain (typical: 0.05)
```

#### 3.4.2 Flow Control Algorithm

```json
{
  "controller_type": "PID",
  "setpoint_lpm": 150,
  "gains": {
    "Kp": 0.5,
    "Ki": 0.1,
    "Kd": 0.05
  },
  "output_limits": {
    "min_position_percent": 10,
    "max_position_percent": 100
  },
  "sample_interval_ms": 1000,
  "deadband_lpm": 2.0
}
```

### 3.5 Pressure Management

#### 3.5.1 Pressure Regulation

**High Pressure Protection**:
```json
{
  "max_pressure_kpa": 400,
  "warning_threshold_kpa": 380,
  "action_on_overpressure": "reduce_flow|close_valve|activate_relief",
  "pressure_sensor_id": "ps-main-line"
}
```

**Low Pressure Handling**:
```json
{
  "min_pressure_kpa": 150,
  "warning_threshold_kpa": 170,
  "action_on_underpressure": "delay_irrigation|alert_operator|check_pump",
  "retry_interval_minutes": 15
}
```

#### 3.5.2 Pressure Compensation

Adjust flow rates to maintain uniform distribution across field:

```
compensated_flow = base_flow × (actual_pressure / target_pressure)^0.5

For drip emitters:
  Q = k × P^x
where:
  Q = flow rate
  P = pressure
  k = emitter coefficient
  x = emitter exponent (typically 0.5)
```

### 3.6 Safety Interlocks

#### 3.6.1 Critical Safety Rules

1. **No irrigation if leak detected** (flow without open valves)
2. **Emergency stop overrides all commands**
3. **Maximum run time limit** (4 hours default, configurable)
4. **Frost protection** (no irrigation if temp < 2°C)
5. **Rain delay** (skip irrigation if recent rainfall > threshold)

#### 3.6.2 Interlock Logic

```json
{
  "interlocks": [
    {
      "name": "leak_detection",
      "condition": "flow_rate > 5 AND all_valves_closed",
      "action": "emergency_stop",
      "enabled": true
    },
    {
      "name": "max_runtime",
      "condition": "irrigation_duration > max_duration",
      "action": "close_valve_and_alert",
      "max_duration_minutes": 240
    },
    {
      "name": "frost_protection",
      "condition": "temperature_c < frost_threshold",
      "action": "cancel_irrigation",
      "frost_threshold_c": 2.0
    },
    {
      "name": "rain_delay",
      "condition": "recent_rainfall_mm > threshold",
      "action": "skip_next_event",
      "threshold_mm": 5.0,
      "lookback_hours": 24
    }
  ]
}
```

### 3.7 Scheduling Decision Logic

#### 3.7.1 Soil Moisture-Based Triggering

```python
# Simplified decision logic
def should_irrigate(zone):
    soil_moisture = get_latest_soil_moisture(zone.sensor_id)
    threshold_low = zone.field_capacity * 0.5  # 50% of field capacity
    threshold_high = zone.field_capacity * 0.8  # 80% of field capacity

    if soil_moisture < threshold_low:
        return True, "critical"  # Irrigate immediately
    elif soil_moisture < threshold_high:
        # Check ET deficit
        et_deficit_mm = calculate_et_deficit(zone)
        if et_deficit_mm > zone.irrigation_trigger_mm:
            return True, "normal"

    return False, "adequate_moisture"
```

#### 3.7.2 ET-Based Scheduling

```python
def calculate_irrigation_depth(zone, days_since_last_irrigation):
    # Accumulate ET deficit
    et_cumulative = 0
    for day in range(days_since_last_irrigation):
        et_daily = get_reference_et(day) * zone.crop_coefficient
        effective_rain = get_effective_rainfall(day)
        et_cumulative += (et_daily - effective_rain)

    # Apply management allowed depletion (MAD)
    mad_fraction = zone.mad_fraction  # e.g., 0.5 for 50% MAD
    available_water = zone.field_capacity - zone.wilting_point
    allowable_depletion = available_water * mad_fraction

    if et_cumulative > allowable_depletion:
        # Account for irrigation efficiency
        return et_cumulative / zone.irrigation_efficiency
    else:
        return 0  # No irrigation needed
```

### 3.8 Weather-Based Adjustments

#### 3.8.1 Forecast-Based Skip

```json
{
  "weather_rules": [
    {
      "rule_id": "forecast_rain_skip",
      "condition": "forecast_24h_precipitation_mm > 10",
      "action": "skip_next_irrigation",
      "enabled": true
    },
    {
      "rule_id": "high_wind_delay",
      "condition": "forecast_wind_speed_ms > 8",
      "action": "delay_irrigation_6h",
      "enabled": true
    }
  ]
}
```

#### 3.8.2 ET Adjustment Factor

```python
def adjust_irrigation_for_weather(base_irrigation_mm, weather_forecast):
    # Adjust based on forecasted ET
    et_adjustment = forecast_et / historical_average_et

    # Adjust based on precipitation probability
    precip_reduction = 0
    if weather_forecast.precipitation_probability > 70:
        precip_reduction = 0.5  # Reduce by 50%
    elif weather_forecast.precipitation_probability > 40:
        precip_reduction = 0.25  # Reduce by 25%

    adjusted_irrigation = base_irrigation_mm * et_adjustment * (1 - precip_reduction)
    return max(0, adjusted_irrigation)
```

### 3.9 Multi-Zone Coordination

#### 3.9.1 Zone Grouping

```json
{
  "zone_groups": [
    {
      "group_id": "group-north-field",
      "zones": ["zone-1", "zone-2", "zone-3"],
      "coordination_mode": "sequential",
      "start_time_offset_minutes": [0, 45, 90]
    },
    {
      "group_id": "group-greenhouse",
      "zones": ["zone-4", "zone-5"],
      "coordination_mode": "simultaneous",
      "shared_water_source": true
    }
  ]
}
```

#### 3.9.2 Conflict Resolution

```python
def resolve_scheduling_conflicts(pending_events):
    # Sort by priority, then by scheduled time
    sorted_events = sorted(pending_events,
                          key=lambda e: (e.priority, e.scheduled_time))

    scheduled = []
    for event in sorted_events:
        # Check for resource conflicts
        if not has_conflict(event, scheduled):
            scheduled.append(event)
        else:
            # Try to reschedule
            next_available_slot = find_next_available_slot(event, scheduled)
            if next_available_slot:
                event.scheduled_time = next_available_slot
                scheduled.append(event)
            else:
                log_warning("Cannot schedule event", event.event_id)

    return scheduled
```

### 3.10 Manual Override Protocol

#### 3.10.1 Override Levels

- **Level 1 (Operator)**: Can start/stop individual zones
- **Level 2 (Supervisor)**: Can modify schedules and settings
- **Level 3 (Administrator)**: Can disable safety interlocks (with logging)
- **Level 4 (Emergency)**: Full system override, all safeties bypassed

#### 3.10.2 Override Command

```json
{
  "override_id": "ovr-456",
  "override_level": 2,
  "user_id": "supervisor-123",
  "action": "manual_start_irrigation",
  "zone_id": "zone-2",
  "duration_minutes": 60,
  "reason": "Visual inspection shows dry soil",
  "bypass_weather_check": false,
  "timestamp": "2025-01-15T16:30:00Z"
}
```

### 3.11 Fault Detection & Recovery

#### 3.11.1 Fault Types

```json
{
  "fault_codes": {
    "F001": "Valve stuck open",
    "F002": "Valve stuck closed",
    "F003": "Flow meter malfunction",
    "F004": "Pressure sensor failure",
    "F005": "Communication timeout",
    "F006": "Leak detected",
    "F007": "Pump failure"
  }
}
```

#### 3.11.2 Auto-Recovery Protocol

```python
def handle_fault(fault_code, device_id):
    if fault_code == "F005":  # Communication timeout
        # Retry communication
        for attempt in range(3):
            if ping_device(device_id):
                return "recovered"
            sleep(10)
        return "manual_intervention_required"

    elif fault_code == "F001":  # Valve stuck open
        # Emergency close sequence
        send_close_command(device_id, priority="emergency")
        if verify_valve_closed(device_id, timeout=30):
            return "recovered"
        else:
            activate_backup_shutoff()
            return "escalated"

    # Default: alert operator
    return "awaiting_operator"
```

### 3.12 Data Logging

#### 3.12.1 Event Logging

All control actions must be logged:

```json
{
  "log_id": "log-78901",
  "timestamp": "2025-01-15T18:00:00Z",
  "event_type": "valve_opened",
  "zone_id": "zone-1",
  "valve_id": "valve-zone-1",
  "user_id": "system-auto",
  "command_id": "cmd-12345",
  "result": "success",
  "duration_ms": 2850
}
```

### 3.13 Protocol Compliance Checklist

- [ ] Implement valve state machine with all transitions
- [ ] Configure safety interlocks for all zones
- [ ] Set up pressure monitoring and limits
- [ ] Implement flow rate control (if applicable)
- [ ] Configure zone sequencing logic
- [ ] Set up weather-based adjustments
- [ ] Implement soil moisture triggers
- [ ] Configure ET-based scheduling
- [ ] Set up fault detection and recovery
- [ ] Implement manual override controls
- [ ] Configure event logging
- [ ] Test emergency shutoff procedures

---

**Previous Phase**: [Phase 2: API Interface](PHASE-2-API-INTERFACE.md)
**Next Phase**: [Phase 4: Integration](PHASE-4-INTEGRATION.md)

© 2025 WIA (World Certification Industry Association)
弘益人間 (홍익인간) · Benefit All Humanity

---

## Z.1 Audit transport and observability hooks (Phase 3)

Every Phase 3 envelope SHOULD emit a structured log line at the
host's audit transport: timestamp per RFC 3339, host identifier,
tenant identifier, envelope class, envelope identifier, operation
outcome, and a W3C Trace Context `traceparent` propagated end-to-end
so a single operation can be reconstructed across hosts. Phase 2
surfaces this trace identifier as the `X-WIA-Trace-Id` response
header. Phase 3 protocol exchanges propagate the trace identifier
inside the exchange envelope so that a federation crossing remains
correlatable end-to-end. Phase 4 integrators consume the audit
stream into the operator's SIEM (Splunk, Elastic, Sumo Logic,
Wazuh, Microsoft Sentinel) per OpenTelemetry semantic conventions,
with `wia.standard.slug` = `smart-irrigation` and `wia.standard.phase` =
`3` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 3)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 Sec 5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations. The composition also lets the operator's SIEM
correlate per-tenant audit records across multiple standards
without per-standard schema-mapping work.

## Z.3 Capabilities discovery and SemVer (Phase 3)

Hosts SHOULD publish a capabilities document at
`/.well-known/wia-smart-irrigation-capabilities` enumerating per-endpoint
optionality. Clients MUST treat unsupported capabilities as absent
rather than as an error condition; a client that needs a capability
the host does not advertise MUST surface a clear configuration
error rather than silently degrade. Hosts moving from one minor
version to the next MUST publish the change in the host's release
notes with the per-capability migration window per IETF RFC 8594
(Sunset header) + RFC 9745 (Deprecation header) + RFC 9651
(Structured Field Values) so machine consumers can plan migration
without waiting for human-channel notification.

## Z.4 Privacy envelope per per-jurisdiction law (Phase 3)

Phase 3 envelopes that carry personal data MUST honour the
operator's per-jurisdiction privacy law (EU GDPR per Regulation
2016/679; UK GDPR per UK Data Protection Act 2018; California
CPRA per Cal. Civ. Code Sec 1798.100; Brazil LGPD per Lei 13.709/2018;
Canada PIPEDA per S.C. 2000 c.5; Korea PIPA per 개인정보 보호법;
Japan APPI per 個人情報の保護に関する法律; Australia Privacy Act
1988 per Cth) including data-minimisation, purpose-limitation,
storage-limitation, integrity + confidentiality, and accountability
principles. Subject-rights endpoints (access, rectification,
erasure, portability, restriction, objection) compose with
WIA-OMNI-API Sec 5 subject-rights surface and need not be
re-implemented per-standard.

## Z.5 DR / continuity envelope per ISO 22301 (Phase 3)

Hosts running this Phase MUST publish a continuity-of-operations
envelope per ISO 22301:2019 + ISO/IEC 27031 + NIST SP 800-34 Rev 1
covering: per-host RTO (Recovery Time Objective) per the operator's
business-impact analysis; per-host RPO (Recovery Point Objective)
tied to the host's audit-stream replication policy; per-host
backup envelope (per-region cross-replicated immutable backup with
the per-tier retention envelope per the per-jurisdiction record-
retention policy); per-host failover-rehearsal envelope (typically
quarterly per the operator's BC/DR program); per-host vendor-exit
envelope so the operator can migrate the host to an alternate
implementation without losing audit-trail continuity.

## Z.6 Supply-chain envelope per SLSA (Phase 3)

Every host implementation MUST publish a software-bill-of-materials
(SBOM) per SPDX 2.3 / 3.0 (per ISO/IEC 5962 + Linux Foundation SPDX)
or CycloneDX 1.6 (per OWASP Foundation). The SBOM enumerates every
direct + transitive dependency with the per-component name +
version + licence + supplier + per-component hash + per-component
PURL (Package URL per package-url spec) + per-component CPE
(Common Platform Enumeration per NIST). Supply-chain attestation
follows in-toto per CNCF in-toto + SLSA (Supply-chain Levels for
Software Artifacts) per OpenSSF SLSA Framework — typically targeting
SLSA Level 3 for hosted production deployments.

弘益人間 — Benefit All Humanity.
