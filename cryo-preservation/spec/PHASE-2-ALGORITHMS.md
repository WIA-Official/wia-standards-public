# Phase 2: Algorithms and Procedures

## WIA-CRYO-PRESERVATION Protocols and Algorithms

> Standardized procedures for cryopreservation operations.

---

## 1. Vitrification Quality Index (VQI) Algorithm

### 1.1 VQI Calculation

```python
def calculate_vqi(assessment: VQIAssessment) -> float:
    """
    Calculate Vitrification Quality Index

    Components:
    - Ice Fraction (weight: 0.35) - lower is better
    - CPA Distribution (weight: 0.25) - higher is better
    - Cooling Rate Score (weight: 0.20) - adherence to protocol
    - Integrity Score (weight: 0.20) - structural/cellular integrity

    Returns: VQI score 0.0 to 1.0
    """

    WEIGHTS = {
        'ice_fraction': 0.35,
        'cpa_distribution': 0.25,
        'cooling_rate': 0.20,
        'integrity': 0.20
    }

    # Ice fraction: 0% ice = 1.0, 10%+ ice = 0.0
    ice_score = max(0, 1 - (assessment.ice_fraction / 0.10))

    # CPA distribution: direct score (0.0 to 1.0)
    cpa_score = assessment.cpa_distribution

    # Cooling rate: based on deviation from target
    cooling_score = assessment.cooling_rate_score

    # Integrity: direct score (0.0 to 1.0)
    integrity_score = assessment.integrity_score

    # Weighted sum
    vqi = (
        ice_score * WEIGHTS['ice_fraction'] +
        cpa_score * WEIGHTS['cpa_distribution'] +
        cooling_score * WEIGHTS['cooling_rate'] +
        integrity_score * WEIGHTS['integrity']
    )

    return round(vqi, 4)


def get_vqi_grade(vqi: float) -> str:
    """Assign grade based on VQI score"""
    if vqi >= 0.95:
        return "EXCELLENT"
    elif vqi >= 0.85:
        return "GOOD"
    elif vqi >= 0.70:
        return "ACCEPTABLE"
    elif vqi >= 0.50:
        return "MARGINAL"
    elif vqi >= 0.30:
        return "POOR"
    else:
        return "CRITICAL"
```

### 1.2 Cooling Rate Score Calculation

```python
def calculate_cooling_rate_score(
    actual_profile: List[TemperatureReading],
    target_profile: CoolingProfile
) -> float:
    """
    Compare actual cooling to target profile

    Returns: Score 0.0 to 1.0
    """

    deviations = []

    for segment in target_profile.segments:
        segment_readings = get_readings_in_range(
            actual_profile,
            segment.start_temp,
            segment.end_temp
        )

        for reading in segment_readings:
            expected_temp = interpolate_target(
                target_profile,
                reading.timestamp
            )

            deviation = abs(reading.temperature - expected_temp)
            tolerance = segment.tolerance

            # Normalize deviation (0 = perfect, 1 = at tolerance limit)
            normalized = min(1.0, deviation / tolerance)
            deviations.append(normalized)

    if not deviations:
        return 0.0

    # Average deviation inverted to score
    avg_deviation = sum(deviations) / len(deviations)
    score = 1.0 - avg_deviation

    # Apply penalty for critical zone deviations
    critical_penalty = calculate_critical_zone_penalty(
        actual_profile,
        target_profile
    )

    return max(0, score - critical_penalty)


def calculate_critical_zone_penalty(
    actual: List[TemperatureReading],
    target: CoolingProfile
) -> float:
    """
    Extra penalty for deviations in critical temperature zones:
    - Around 0°C (ice formation risk)
    - Around -40°C (homogeneous nucleation)
    - Around Tg (glass transition)
    """

    CRITICAL_ZONES = [
        {"center": 0, "range": 5, "weight": 0.10},    # Freezing point
        {"center": -40, "range": 10, "weight": 0.15}, # Nucleation
        {"center": -130, "range": 10, "weight": 0.10} # Glass transition
    ]

    penalty = 0.0

    for zone in CRITICAL_ZONES:
        zone_readings = [
            r for r in actual
            if abs(r.temperature - zone["center"]) <= zone["range"]
        ]

        if zone_readings:
            # Check for excessive dwell time
            dwell_time = calculate_dwell_time(zone_readings)
            expected_dwell = target.get_expected_dwell(zone["center"])

            if dwell_time > expected_dwell * 1.5:
                penalty += zone["weight"]

    return penalty
```

---

## 2. CPA Perfusion Protocol

### 2.1 Concentration Ramping Algorithm

```python
def calculate_cpa_ramp(
    target_concentration: float,
    subject_type: SubjectType,
    perfusion_config: PerfusionConfig
) -> List[CPAStep]:
    """
    Calculate CPA loading steps to minimize osmotic stress

    Strategy: Gradual increase with equilibration periods
    """

    # Determine step size based on subject type
    if subject_type == "WHOLE_BODY":
        max_step = 0.05  # 5% concentration increase per step
        equilibration_time = 300  # 5 minutes
    elif subject_type == "ORGAN":
        max_step = 0.08
        equilibration_time = 180
    else:
        max_step = 0.10
        equilibration_time = 120

    steps = []
    current_concentration = 0.0
    step_number = 0

    while current_concentration < target_concentration:
        step_number += 1

        # Calculate next concentration
        next_concentration = min(
            current_concentration + max_step,
            target_concentration
        )

        # Adjust for final step precision
        if target_concentration - next_concentration < max_step / 2:
            next_concentration = target_concentration

        step = CPAStep(
            step_number=step_number,
            start_concentration=current_concentration,
            end_concentration=next_concentration,
            duration=calculate_perfusion_duration(
                current_concentration,
                next_concentration,
                perfusion_config
            ),
            equilibration_time=equilibration_time,
            flow_rate=calculate_flow_rate(
                next_concentration,
                perfusion_config
            ),
            temperature=perfusion_config.temperature
        )

        steps.append(step)
        current_concentration = next_concentration

    return steps


def calculate_perfusion_duration(
    start_conc: float,
    end_conc: float,
    config: PerfusionConfig
) -> int:
    """Calculate time needed to achieve concentration change"""

    concentration_delta = end_conc - start_conc

    # Base time depends on volume and flow rate
    base_time = (config.volume * 3) / config.max_flow_rate  # 3x volume exchange

    # Adjust for concentration (higher = slower diffusion)
    viscosity_factor = 1 + (end_conc * 2)  # Higher concentration = higher viscosity

    return int(base_time * viscosity_factor * 60)  # Convert to seconds
```

### 2.2 Perfusion Monitoring

```python
def monitor_perfusion(
    subject_id: str,
    perfusion_session: PerfusionSession
) -> PerfusionMonitoringResult:
    """
    Real-time monitoring of CPA perfusion

    Checks:
    - Flow rate stability
    - Pressure within limits
    - Temperature maintenance
    - CPA concentration (effluent analysis)
    - Visual inspection (color, clarity)
    """

    alerts = []
    measurements = []

    while perfusion_session.is_active:
        # Read sensors
        reading = PerfusionReading(
            timestamp=now(),
            inlet_pressure=read_sensor("inlet_pressure"),
            outlet_pressure=read_sensor("outlet_pressure"),
            flow_rate=read_sensor("flow_rate"),
            temperature=read_sensor("perfusate_temp"),
            effluent_cpa=analyze_effluent_cpa(),
            effluent_color=analyze_effluent_color()
        )

        measurements.append(reading)

        # Check for alerts
        if reading.inlet_pressure > perfusion_session.max_pressure:
            alerts.append(Alert(
                level="WARNING",
                message="Inlet pressure exceeds limit",
                value=reading.inlet_pressure,
                threshold=perfusion_session.max_pressure
            ))

        if reading.flow_rate < perfusion_session.min_flow_rate:
            alerts.append(Alert(
                level="WARNING",
                message="Flow rate below minimum",
                value=reading.flow_rate,
                threshold=perfusion_session.min_flow_rate
            ))

        # Check CPA equilibration
        if reading.effluent_cpa >= perfusion_session.current_step.target * 0.95:
            perfusion_session.step_complete = True

        # Log and wait
        log_reading(subject_id, reading)
        wait(perfusion_session.monitoring_interval)

    return PerfusionMonitoringResult(
        measurements=measurements,
        alerts=alerts,
        final_cpa_concentration=measurements[-1].effluent_cpa,
        total_duration=calculate_duration(measurements),
        success=len([a for a in alerts if a.level == "CRITICAL"]) == 0
    )
```

---

## 3. Cooling Control Algorithm

### 3.1 Controlled Rate Cooling

```python
def execute_cooling_profile(
    subject_id: str,
    profile: CoolingProfile,
    controller: CoolingController
) -> CoolingResult:
    """
    Execute a controlled cooling profile

    Manages:
    - Cooling rate in each segment
    - Hold periods at critical points
    - Emergency protocols
    """

    current_temp = read_temperature(subject_id)
    events = []

    for segment in profile.segments:
        log_event(subject_id, f"Starting segment: {segment.name}")

        segment_result = execute_cooling_segment(
            subject_id,
            segment,
            controller,
            current_temp
        )

        events.extend(segment_result.events)

        if segment_result.aborted:
            return CoolingResult(
                success=False,
                final_temperature=segment_result.final_temp,
                events=events,
                abort_reason=segment_result.abort_reason
            )

        current_temp = segment_result.final_temp

        # Check for critical points
        for critical in profile.critical_points:
            if is_at_critical_point(current_temp, critical):
                execute_critical_point_protocol(
                    subject_id,
                    critical,
                    controller
                )

    return CoolingResult(
        success=True,
        final_temperature=current_temp,
        events=events,
        vqi_preliminary=calculate_preliminary_vqi(events)
    )


def execute_cooling_segment(
    subject_id: str,
    segment: CoolingSegment,
    controller: CoolingController,
    start_temp: float
) -> SegmentResult:
    """Execute a single cooling segment with feedback control"""

    events = []
    current_temp = start_temp
    segment_start = now()

    # PID controller setup
    pid = PIDController(
        kp=segment.pid_params.kp,
        ki=segment.pid_params.ki,
        kd=segment.pid_params.kd,
        setpoint_generator=lambda t: calculate_target_temp(
            segment, segment_start, t
        )
    )

    while current_temp > segment.end_temp.value:
        # Read actual temperature
        actual_temp = read_temperature(subject_id)

        # Calculate target at this moment
        elapsed = (now() - segment_start).total_seconds()
        target_temp = calculate_target_temp(segment, segment_start, now())

        # PID output
        control_signal = pid.compute(actual_temp, target_temp)

        # Apply control
        controller.set_cooling_power(control_signal)

        # Log
        events.append(CoolingEvent(
            timestamp=now(),
            actual_temp=actual_temp,
            target_temp=target_temp,
            cooling_power=control_signal,
            deviation=actual_temp - target_temp
        ))

        # Check for abort conditions
        if abs(actual_temp - target_temp) > segment.abort_threshold:
            consecutive_errors = count_consecutive_errors(events)
            if consecutive_errors > 10:
                return SegmentResult(
                    aborted=True,
                    final_temp=actual_temp,
                    events=events,
                    abort_reason="Excessive deviation from target"
                )

        current_temp = actual_temp
        wait(segment.control_interval)

    return SegmentResult(
        aborted=False,
        final_temp=current_temp,
        events=events
    )
```

### 3.2 Glass Transition Detection

```python
def detect_glass_transition(
    temperature_log: List[TemperatureReading],
    expected_tg: float
) -> GlassTransitionResult:
    """
    Detect glass transition using thermal analysis

    Indicators:
    - Change in specific heat capacity
    - Thermal signature flattening
    - Acoustic emission changes (if equipped)
    """

    # Analyze temperature derivative
    derivatives = calculate_derivatives(temperature_log)

    # Look for characteristic Tg signature
    tg_candidates = []

    for i, reading in enumerate(temperature_log):
        if expected_tg - 10 <= reading.temperature <= expected_tg + 10:
            # Check for thermal signature
            local_derivative = derivatives[i]

            # At Tg, should see change in cooling characteristics
            if is_tg_signature(derivatives, i):
                tg_candidates.append({
                    'temperature': reading.temperature,
                    'timestamp': reading.timestamp,
                    'confidence': calculate_tg_confidence(derivatives, i)
                })

    if not tg_candidates:
        return GlassTransitionResult(
            detected=False,
            temperature=expected_tg,
            confidence=0.0,
            method="DERIVATIVE_ANALYSIS"
        )

    # Select highest confidence candidate
    best = max(tg_candidates, key=lambda x: x['confidence'])

    return GlassTransitionResult(
        detected=True,
        temperature=best['temperature'],
        timestamp=best['timestamp'],
        confidence=best['confidence'],
        deviation_from_expected=best['temperature'] - expected_tg,
        method="DERIVATIVE_ANALYSIS"
    )


def is_tg_signature(derivatives: List[float], index: int) -> bool:
    """
    Check if derivative pattern matches glass transition

    Expected: Rapid decrease in dT/dt as material becomes glassy
    """

    window = 5
    if index < window or index >= len(derivatives) - window:
        return False

    before = sum(derivatives[index-window:index]) / window
    after = sum(derivatives[index+1:index+window+1]) / window

    # Tg signature: significant decrease in cooling rate
    # (material becomes glassy, thermal properties change)
    ratio = after / before if before != 0 else 0

    return ratio < 0.7  # At least 30% decrease
```

---

## 4. Storage Monitoring Algorithms

### 4.1 LN2 Level Prediction

```python
def predict_ln2_exhaustion(
    container: StorageContainer,
    history: List[LN2Reading]
) -> LN2Prediction:
    """
    Predict when LN2 will reach critical level

    Uses:
    - Historical evaporation rate
    - Environmental factors
    - Container specifications
    """

    # Calculate recent evaporation rate
    recent_history = history[-48:]  # Last 48 hours

    if len(recent_history) < 2:
        return LN2Prediction(
            reliable=False,
            reason="Insufficient history"
        )

    # Linear regression for trend
    evaporation_rate = calculate_evaporation_rate(recent_history)

    # Adjust for environmental factors
    ambient_temp = get_ambient_temperature()
    temp_factor = 1 + (ambient_temp - 20) * 0.02  # 2% per degree above 20°C

    # Adjust for container age/condition
    age_factor = calculate_age_factor(container)

    adjusted_rate = evaporation_rate * temp_factor * age_factor

    # Current level
    current_level = history[-1].level_percent
    current_volume = (current_level / 100) * container.capacity

    # Critical level (20% for warning, 10% for critical)
    warning_volume = 0.20 * container.capacity
    critical_volume = 0.10 * container.capacity

    # Calculate times
    volume_to_warning = current_volume - warning_volume
    volume_to_critical = current_volume - critical_volume

    hours_to_warning = volume_to_warning / adjusted_rate if adjusted_rate > 0 else float('inf')
    hours_to_critical = volume_to_critical / adjusted_rate if adjusted_rate > 0 else float('inf')

    return LN2Prediction(
        reliable=True,
        current_level=current_level,
        evaporation_rate=adjusted_rate,
        hours_to_warning=hours_to_warning,
        hours_to_critical=hours_to_critical,
        recommended_refill_date=now() + timedelta(hours=hours_to_warning * 0.8),
        confidence=calculate_prediction_confidence(recent_history)
    )
```

### 4.2 Anomaly Detection

```python
def detect_monitoring_anomalies(
    container_id: str,
    readings: List[MonitoringRecord]
) -> List[Anomaly]:
    """
    Detect anomalies in monitoring data

    Types:
    - Temperature spikes
    - Unusual evaporation
    - Sensor failures
    - Pattern deviations
    """

    anomalies = []

    # Temperature analysis
    temp_readings = [r.temperature.value for r in readings]
    temp_anomalies = detect_statistical_anomalies(
        temp_readings,
        method="IQR",
        threshold=3.0
    )

    for idx, score in temp_anomalies:
        if score > 0:
            anomalies.append(Anomaly(
                type="TEMPERATURE_SPIKE",
                severity=classify_severity(score),
                timestamp=readings[idx].timestamp,
                value=temp_readings[idx],
                expected_range=calculate_expected_range(temp_readings, idx),
                score=score
            ))

    # Evaporation rate analysis
    ln2_readings = [r.ln2_level for r in readings]
    evap_rate = calculate_sliding_evaporation_rate(ln2_readings)
    evap_anomalies = detect_rate_anomalies(evap_rate)

    for idx, anomaly in evap_anomalies:
        anomalies.append(Anomaly(
            type="EVAPORATION_ANOMALY",
            severity=anomaly['severity'],
            timestamp=readings[idx].timestamp,
            value=evap_rate[idx],
            description=anomaly['description']
        ))

    # Sensor health
    for reading in readings:
        for sensor in reading.sensor_status:
            if sensor.status != "ONLINE":
                anomalies.append(Anomaly(
                    type="SENSOR_ISSUE",
                    severity="WARNING" if sensor.status == "ERROR" else "INFO",
                    timestamp=reading.timestamp,
                    sensor_id=sensor.sensor_id,
                    description=f"Sensor {sensor.sensor_id}: {sensor.status}"
                ))

    return sorted(anomalies, key=lambda a: a.timestamp, reverse=True)


def detect_statistical_anomalies(
    data: List[float],
    method: str = "IQR",
    threshold: float = 1.5
) -> List[Tuple[int, float]]:
    """
    Detect statistical outliers in time series data
    """

    if method == "IQR":
        q1 = np.percentile(data, 25)
        q3 = np.percentile(data, 75)
        iqr = q3 - q1
        lower = q1 - threshold * iqr
        upper = q3 + threshold * iqr

        anomalies = []
        for i, value in enumerate(data):
            if value < lower or value > upper:
                score = abs(value - np.median(data)) / iqr
                anomalies.append((i, score))

        return anomalies

    elif method == "Z_SCORE":
        mean = np.mean(data)
        std = np.std(data)

        anomalies = []
        for i, value in enumerate(data):
            z = abs(value - mean) / std if std > 0 else 0
            if z > threshold:
                anomalies.append((i, z))

        return anomalies
```

---

## 5. Transfer Safety Algorithms

### 5.1 Transfer Feasibility Check

```python
def assess_transfer_feasibility(
    subject_id: str,
    origin: Facility,
    destination: Facility,
    transport_config: TransportConfig
) -> TransferFeasibility:
    """
    Assess whether a transfer can be safely performed

    Checks:
    - Transport container capacity
    - Hold time vs. transit time
    - Route conditions
    - Receiving facility readiness
    """

    issues = []
    warnings = []

    # Get subject details
    subject = get_subject(subject_id)

    # Check transport container
    if not transport_config.container.can_accommodate(subject.subject_type):
        issues.append(Issue(
            category="CONTAINER",
            message="Transport container cannot accommodate subject type"
        ))

    # Calculate transit time
    transit_time = calculate_transit_time(
        origin.location,
        destination.location,
        transport_config.transport_method
    )

    # Check hold time
    safety_margin = 2.0  # Require 2x safety margin
    required_hold = transit_time.total_seconds() / 3600 * safety_margin

    if transport_config.container.hold_time_hours < required_hold:
        issues.append(Issue(
            category="HOLD_TIME",
            message=f"Container hold time ({transport_config.container.hold_time_hours}h) "
                   f"insufficient for transit ({transit_time.total_seconds()/3600}h with {safety_margin}x margin)"
        ))

    # Check destination readiness
    dest_status = destination.get_status()
    if dest_status.status != "OPERATIONAL":
        issues.append(Issue(
            category="DESTINATION",
            message=f"Destination facility not operational: {dest_status.status}"
        ))

    if not destination.has_available_slot(subject.subject_type):
        issues.append(Issue(
            category="CAPACITY",
            message="No available storage slot at destination"
        ))

    # Weather/route check
    route_assessment = assess_route_conditions(
        origin.location,
        destination.location,
        transport_config.transport_method,
        transit_time
    )

    if route_assessment.has_severe_conditions:
        warnings.append(Warning(
            category="ROUTE",
            message=f"Adverse conditions on route: {route_assessment.conditions}"
        ))

    return TransferFeasibility(
        feasible=len(issues) == 0,
        issues=issues,
        warnings=warnings,
        estimated_transit_time=transit_time,
        recommended_departure=calculate_optimal_departure(
            transit_time,
            route_assessment,
            destination.preferred_arrival_window
        ),
        risk_score=calculate_transfer_risk(
            issues,
            warnings,
            transit_time,
            transport_config
        )
    )
```

### 5.2 Transfer Monitoring

```python
def monitor_transfer(
    transfer_id: str,
    manifest: TransferManifest,
    tracking_device: TrackingDevice
) -> Generator[TransferUpdate, None, None]:
    """
    Real-time monitoring of subject transfer

    Monitors:
    - Location via GPS
    - Container temperature
    - LN2 level (if applicable)
    - Movement/shock
    - ETA updates
    """

    while True:
        # Get current readings
        location = tracking_device.get_location()
        container_data = tracking_device.get_container_data()

        # Calculate progress
        progress = calculate_transfer_progress(
            manifest.origin_facility.location,
            manifest.destination_facility.location,
            location
        )

        # Check alerts
        alerts = []

        if container_data.temperature > -180:
            alerts.append(TransferAlert(
                level="CRITICAL",
                message="Container temperature above safe threshold",
                value=container_data.temperature,
                threshold=-180
            ))

        if container_data.shock_detected:
            alerts.append(TransferAlert(
                level="WARNING",
                message="Shock/impact detected",
                details=container_data.shock_data
            ))

        # Update ETA
        eta = calculate_eta(
            location,
            manifest.destination_facility.location,
            manifest.transport_method
        )

        update = TransferUpdate(
            transfer_id=transfer_id,
            timestamp=now(),
            location=location,
            temperature=container_data.temperature,
            ln2_level=container_data.ln2_level,
            progress_percent=progress,
            eta=eta,
            alerts=alerts,
            status="IN_TRANSIT" if not alerts else "ALERT"
        )

        # Store update
        store_transfer_update(update)

        # Notify if alerts
        if alerts:
            notify_stakeholders(manifest, update)

        yield update

        # Check if arrived
        if progress >= 99.5:
            break

        # Wait before next update
        wait(60)  # 1 minute intervals
```

---

## 6. Emergency Response Algorithms

### 6.1 Facility Emergency Protocol

```python
def execute_emergency_protocol(
    facility: Facility,
    emergency_type: EmergencyType
) -> EmergencyResponse:
    """
    Execute emergency response based on emergency type

    Types:
    - POWER_FAILURE
    - LN2_SUPPLY_FAILURE
    - CONTAINER_BREACH
    - FIRE
    - NATURAL_DISASTER
    - SECURITY_BREACH
    """

    response = EmergencyResponse(
        facility_id=facility.id,
        emergency_type=emergency_type,
        started_at=now()
    )

    # Notify personnel
    notify_emergency_team(facility, emergency_type)

    if emergency_type == EmergencyType.POWER_FAILURE:
        # Check backup power
        backup_status = check_backup_power(facility)

        if backup_status.available:
            activate_backup_power(facility)
            response.actions.append("Backup power activated")
        else:
            # Calculate time to critical
            containers = get_all_containers(facility)
            critical_times = [
                estimate_thermal_decay(c) for c in containers
            ]

            response.time_to_critical = min(critical_times)
            response.actions.append(
                f"No backup power. Critical in {response.time_to_critical} hours"
            )

            # Initiate emergency transfer if needed
            if response.time_to_critical < 24:
                initiate_emergency_transfer(facility, containers)
                response.actions.append("Emergency transfer initiated")

    elif emergency_type == EmergencyType.LN2_SUPPLY_FAILURE:
        # Check current levels
        containers = get_all_containers(facility)

        for container in containers:
            time_to_critical = predict_ln2_exhaustion(container).hours_to_critical

            if time_to_critical < 48:
                # Attempt emergency LN2 delivery
                delivery = arrange_emergency_ln2(facility)
                response.actions.append(
                    f"Emergency LN2 delivery: {delivery.status}"
                )

    elif emergency_type == EmergencyType.CONTAINER_BREACH:
        # Identify affected container
        affected = identify_breached_container(facility)

        # Emergency rewarming prevention
        for container in affected:
            initiate_emergency_cooling(container)
            response.actions.append(
                f"Emergency cooling for container {container.id}"
            )

            # Assess transfer need
            if container.breach_severity == "CRITICAL":
                initiate_emergency_transfer(
                    facility,
                    [container],
                    priority="IMMEDIATE"
                )

    # Log all actions
    log_emergency_response(response)

    return response
```

---

**Phase 2 Algorithms and Procedures**
**WIA-CRYO-PRESERVATION v1.0.0**
