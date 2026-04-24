# Phase 4: Integration Guide

## WIA-CRYO-PRESERVATION Implementation Guide

> Practical integration for cryopreservation facilities and systems.

---

## 1. System Architecture

### 1.1 Reference Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    FACILITY MANAGEMENT SYSTEM                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐             │
│  │   Web UI    │  │  Mobile App │  │ Control     │             │
│  │  Dashboard  │  │  Alerts     │  │ Terminals   │             │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘             │
│         │                │                │                      │
│         └────────────────┼────────────────┘                      │
│                          │                                       │
│                 ┌────────▼────────┐                              │
│                 │   API Gateway   │                              │
│                 │  (REST + WS)    │                              │
│                 └────────┬────────┘                              │
│                          │                                       │
│    ┌─────────────────────┼─────────────────────┐                │
│    │                     │                     │                │
│  ┌─▼───────────┐  ┌──────▼──────┐  ┌──────────▼─┐              │
│  │  Subject    │  │  Storage    │  │  Protocol  │              │
│  │  Service    │  │  Service    │  │  Service   │              │
│  └─────────────┘  └─────────────┘  └────────────┘              │
│                          │                                       │
│                 ┌────────▼────────┐                              │
│                 │   Data Layer    │                              │
│                 │  (PostgreSQL)   │                              │
│                 └────────┬────────┘                              │
│                          │                                       │
├──────────────────────────┼──────────────────────────────────────┤
│                          │                                       │
│  ┌─────────────┐  ┌──────▼──────┐  ┌─────────────┐             │
│  │   Sensor    │  │  Monitoring │  │   Alert     │             │
│  │  Network    ├──┤   Service   ├──┤   Service   │             │
│  └─────────────┘  └─────────────┘  └─────────────┘             │
│                                                                  │
│  HARDWARE LAYER                                                  │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐             │
│  │ Temperature │  │  LN2 Level  │  │   Access    │             │
│  │  Sensors    │  │  Sensors    │  │   Control   │             │
│  └─────────────┘  └─────────────┘  └─────────────┘             │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 1.2 Component Overview

| Component | Purpose | Technology |
|-----------|---------|------------|
| API Gateway | Route requests, auth | Kong / AWS API Gateway |
| Subject Service | Patient/specimen management | Rust / Go |
| Storage Service | Container management | Rust / Go |
| Monitoring Service | Real-time data collection | Rust + TimescaleDB |
| Alert Service | Alert processing & dispatch | Go + Redis |
| Protocol Service | Preservation protocol management | Rust |
| Sensor Network | Hardware integration | Embedded C / Rust |

---

## 2. Sensor Integration

### 2.1 Temperature Sensor Interface

```rust
// sensor_interface.rs
use wia_cryo::sensors::{Sensor, SensorReading, SensorError};

pub trait TemperatureSensor: Sensor {
    /// Read current temperature
    fn read_temperature(&self) -> Result<Temperature, SensorError>;

    /// Get sensor accuracy specification
    fn accuracy(&self) -> f64;

    /// Get measurement range
    fn range(&self) -> (Temperature, Temperature);

    /// Perform self-calibration check
    fn self_test(&self) -> Result<SensorHealth, SensorError>;
}

// PT100 RTD implementation
pub struct PT100Sensor {
    device_id: String,
    bus: I2CBus,
    calibration: CalibrationData,
    last_reading: Option<SensorReading>,
}

impl TemperatureSensor for PT100Sensor {
    fn read_temperature(&self) -> Result<Temperature, SensorError> {
        // Read raw resistance
        let resistance = self.bus.read_register(self.address, REG_RESISTANCE)?;

        // Convert using Callendar-Van Dusen equation
        let temp_c = self.callendar_van_dusen(resistance);

        // Apply calibration offset
        let calibrated = temp_c + self.calibration.offset;

        Ok(Temperature {
            value: calibrated,
            unit: TemperatureUnit::Celsius,
            timestamp: Utc::now(),
            sensor_id: self.device_id.clone(),
        })
    }

    fn accuracy(&self) -> f64 {
        // PT100 Class A: ±(0.15 + 0.002 × |t|) °C
        0.15  // Base accuracy at 0°C
    }

    fn range(&self) -> (Temperature, Temperature) {
        (
            Temperature::celsius(-200.0),
            Temperature::celsius(850.0)
        )
    }

    fn self_test(&self) -> Result<SensorHealth, SensorError> {
        // Check response at known reference
        let reading = self.read_temperature()?;

        // Verify reading is within valid range
        let (min, max) = self.range();
        if reading.value < min.value || reading.value > max.value {
            return Err(SensorError::OutOfRange);
        }

        // Check noise level
        let noise = self.measure_noise()?;
        if noise > self.calibration.max_noise {
            return Ok(SensorHealth::Degraded {
                reason: "Excessive noise".into()
            });
        }

        Ok(SensorHealth::Healthy)
    }
}
```

### 2.2 LN2 Level Sensor Interface

```rust
// ln2_sensor.rs
pub trait LN2LevelSensor: Sensor {
    /// Read current LN2 level (0-100%)
    fn read_level(&self) -> Result<f64, SensorError>;

    /// Get volume in liters
    fn read_volume(&self, container_capacity: f64) -> Result<f64, SensorError> {
        let level = self.read_level()?;
        Ok(container_capacity * level / 100.0)
    }

    /// Estimate time to critical level
    fn estimate_time_to_critical(
        &self,
        history: &[LevelReading],
        critical_level: f64
    ) -> Result<Duration, SensorError>;
}

// Capacitive level sensor implementation
pub struct CapacitiveLN2Sensor {
    device_id: String,
    probe_length: f64,  // mm
    calibration: LN2Calibration,
}

impl LN2LevelSensor for CapacitiveLN2Sensor {
    fn read_level(&self) -> Result<f64, SensorError> {
        // Read capacitance
        let capacitance = self.read_capacitance()?;

        // Convert to level using calibration curve
        let level = self.capacitance_to_level(capacitance);

        // Clamp to valid range
        Ok(level.clamp(0.0, 100.0))
    }

    fn estimate_time_to_critical(
        &self,
        history: &[LevelReading],
        critical_level: f64
    ) -> Result<Duration, SensorError> {
        if history.len() < 2 {
            return Err(SensorError::InsufficientData);
        }

        // Calculate evaporation rate from recent history
        let recent: Vec<_> = history.iter()
            .rev()
            .take(48)  // Last 48 readings
            .collect();

        let evap_rate = calculate_evaporation_rate(&recent)?;

        if evap_rate <= 0.0 {
            return Ok(Duration::MAX);  // Not evaporating
        }

        let current_level = self.read_level()?;
        let level_to_critical = current_level - critical_level;

        if level_to_critical <= 0.0 {
            return Ok(Duration::ZERO);  // Already critical
        }

        let hours = level_to_critical / evap_rate;
        Ok(Duration::from_secs_f64(hours * 3600.0))
    }
}
```

### 2.3 Sensor Network Configuration

```yaml
# sensor_config.yaml
sensor_network:
  protocol: MODBUS_TCP
  scan_interval: 10s
  timeout: 5s

  containers:
    - id: dewar-001
      sensors:
        - id: temp-001
          type: PT100
          address: 0x48
          location: top
          calibration_date: 2024-01-01
          calibration_offset: -0.15

        - id: temp-002
          type: PT100
          address: 0x49
          location: bottom
          calibration_date: 2024-01-01
          calibration_offset: -0.12

        - id: ln2-001
          type: CAPACITIVE
          address: 0x50
          probe_length: 500
          calibration:
            empty_capacitance: 10.5
            full_capacitance: 85.2

  alert_thresholds:
    temperature:
      warning: -190.0
      critical: -180.0
      emergency: -170.0

    ln2_level:
      warning: 30.0
      critical: 20.0
      emergency: 10.0
```

---

## 3. Database Schema

### 3.1 Core Tables

```sql
-- Subject management
CREATE TABLE subjects (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    did VARCHAR(255) UNIQUE NOT NULL,
    subject_type VARCHAR(50) NOT NULL,
    species VARCHAR(50) NOT NULL DEFAULT 'HUMAN',

    -- Biological info
    biological_age DECIMAL(5,2),
    chronological_age DECIMAL(5,2),
    sex VARCHAR(20),
    blood_type JSONB,

    -- Status
    status VARCHAR(50) NOT NULL DEFAULT 'REGISTERED',
    preservation_date TIMESTAMPTZ,
    preservation_facility_id UUID REFERENCES facilities(id),

    -- Legal
    consent_document_id UUID,
    legal_status VARCHAR(50),

    -- Metadata
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    version INTEGER NOT NULL DEFAULT 1
);

CREATE INDEX idx_subjects_status ON subjects(status);
CREATE INDEX idx_subjects_facility ON subjects(preservation_facility_id);

-- Storage containers
CREATE TABLE containers (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    facility_id UUID NOT NULL REFERENCES facilities(id),
    container_type VARCHAR(50) NOT NULL,

    -- Location
    building VARCHAR(100),
    room VARCHAR(100),
    position VARCHAR(100),

    -- Specifications
    capacity_liters DECIMAL(10,2),
    manufacturer VARCHAR(100),
    model VARCHAR(100),
    serial_number VARCHAR(100),

    -- Current state
    current_ln2_level DECIMAL(5,2),
    current_temperature DECIMAL(6,2),

    -- Maintenance
    installation_date DATE,
    last_maintenance DATE,
    next_maintenance DATE,

    -- Status
    status VARCHAR(50) NOT NULL DEFAULT 'OPERATIONAL',

    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Storage slots within containers
CREATE TABLE storage_slots (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    container_id UUID NOT NULL REFERENCES containers(id),
    slot_position JSONB NOT NULL,

    -- Content
    subject_id UUID REFERENCES subjects(id),
    inner_container_type VARCHAR(50),
    inner_container_id VARCHAR(100),

    -- Status
    occupied BOOLEAN NOT NULL DEFAULT FALSE,
    reserved_for UUID REFERENCES subjects(id),

    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Monitoring data (TimescaleDB hypertable)
CREATE TABLE monitoring_data (
    time TIMESTAMPTZ NOT NULL,
    container_id UUID NOT NULL REFERENCES containers(id),

    -- Measurements
    temperature DECIMAL(6,2),
    ln2_level DECIMAL(5,2),
    pressure DECIMAL(8,2),
    humidity DECIMAL(5,2),

    -- Sensor health
    sensor_status JSONB,

    -- Calculated
    evaporation_rate DECIMAL(6,4),
    estimated_hold_time INTERVAL
);

SELECT create_hypertable('monitoring_data', 'time');
CREATE INDEX idx_monitoring_container ON monitoring_data(container_id, time DESC);

-- Preservation events
CREATE TABLE preservation_events (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    subject_id UUID NOT NULL REFERENCES subjects(id),
    event_type VARCHAR(100) NOT NULL,
    phase VARCHAR(100),

    -- Details
    description TEXT,
    parameters JSONB,

    -- Personnel
    performed_by UUID[],
    witnessed_by UUID[],

    -- Outcome
    outcome VARCHAR(50),
    deviations JSONB,

    -- Documentation
    notes TEXT,
    attachments JSONB,

    timestamp TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_events_subject ON preservation_events(subject_id, timestamp DESC);
CREATE INDEX idx_events_type ON preservation_events(event_type);

-- Alerts
CREATE TABLE alerts (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    container_id UUID REFERENCES containers(id),
    facility_id UUID REFERENCES facilities(id),

    level VARCHAR(20) NOT NULL,
    alert_type VARCHAR(100) NOT NULL,
    message TEXT NOT NULL,
    details JSONB,

    -- Status
    status VARCHAR(20) NOT NULL DEFAULT 'ACTIVE',
    acknowledged_by UUID,
    acknowledged_at TIMESTAMPTZ,
    resolved_at TIMESTAMPTZ,
    resolution_notes TEXT,

    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_alerts_status ON alerts(status, level);
CREATE INDEX idx_alerts_container ON alerts(container_id, created_at DESC);
```

### 3.2 Quality Assessment Tables

```sql
-- VQI Assessments
CREATE TABLE vqi_assessments (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    subject_id UUID NOT NULL REFERENCES subjects(id),

    -- Component scores
    ice_fraction DECIMAL(5,4),
    cpa_distribution DECIMAL(5,4),
    cooling_rate_score DECIMAL(5,4),
    integrity_score DECIMAL(5,4),

    -- Calculated VQI
    vqi DECIMAL(5,4) NOT NULL,
    vqi_grade VARCHAR(20) NOT NULL,

    -- Details
    findings JSONB,
    recommendations TEXT[],

    -- Assessor
    assessor_id UUID NOT NULL,
    assessment_method VARCHAR(100),

    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_vqi_subject ON vqi_assessments(subject_id, created_at DESC);

-- Sample analyses
CREATE TABLE sample_analyses (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    subject_id UUID NOT NULL REFERENCES subjects(id),
    sample_type VARCHAR(50) NOT NULL,

    -- Results
    cell_viability JSONB,
    ice_content JSONB,
    cpa_concentration JSONB,
    structural_integrity JSONB,

    -- Method
    analysis_method VARCHAR(100),
    equipment TEXT[],
    operator_id UUID,

    -- Raw data
    raw_data_reference VARCHAR(500),

    collection_time TIMESTAMPTZ NOT NULL,
    analysis_time TIMESTAMPTZ NOT NULL DEFAULT NOW()
);
```

---

## 4. Alert System Integration

### 4.1 Alert Configuration

```yaml
# alert_config.yaml
alert_system:
  # Escalation levels
  escalation:
    - level: WARNING
      initial_delay: 0
      repeat_interval: 30m
      channels: [dashboard, email]

    - level: CRITICAL
      initial_delay: 0
      repeat_interval: 10m
      channels: [dashboard, email, sms, phone]

    - level: EMERGENCY
      initial_delay: 0
      repeat_interval: 5m
      channels: [dashboard, email, sms, phone, pa_system]

  # On-call rotation
  on_call:
    primary:
      - name: "Dr. Smith"
        phone: "+1-555-0100"
        email: "smith@facility.org"

    secondary:
      - name: "Dr. Jones"
        phone: "+1-555-0101"
        email: "jones@facility.org"

    escalation_timeout: 15m

  # Integration endpoints
  integrations:
    email:
      smtp_host: smtp.facility.org
      smtp_port: 587
      from: alerts@facility.org

    sms:
      provider: twilio
      account_sid: ${TWILIO_SID}
      auth_token: ${TWILIO_TOKEN}
      from_number: "+1-555-0000"

    phone:
      provider: twilio
      voice_url: https://facility.org/voice/alert.xml

    pa_system:
      endpoint: http://pa-controller.local/announce
      api_key: ${PA_API_KEY}
```

### 4.2 Alert Processing

```rust
// alert_processor.rs
use wia_cryo::alerts::{Alert, AlertLevel, Notification};

pub struct AlertProcessor {
    config: AlertConfig,
    notifiers: Vec<Box<dyn Notifier>>,
    on_call: OnCallManager,
}

impl AlertProcessor {
    pub async fn process_alert(&self, alert: Alert) -> Result<(), AlertError> {
        // Deduplicate
        if self.is_duplicate(&alert) {
            return Ok(());
        }

        // Store alert
        self.store_alert(&alert).await?;

        // Get escalation config
        let escalation = self.config.get_escalation(alert.level);

        // Build notification
        let notification = self.build_notification(&alert);

        // Send to configured channels
        for channel in &escalation.channels {
            match channel.as_str() {
                "dashboard" => {
                    self.notifiers.dashboard.send(&notification).await?;
                }
                "email" => {
                    let recipients = self.get_email_recipients(&alert);
                    self.notifiers.email.send(&notification, recipients).await?;
                }
                "sms" => {
                    let phones = self.on_call.get_on_call_phones();
                    self.notifiers.sms.send(&notification, phones).await?;
                }
                "phone" => {
                    let phone = self.on_call.get_primary_phone();
                    self.notifiers.phone.call(phone, &notification).await?;
                }
                "pa_system" => {
                    self.notifiers.pa.announce(&notification).await?;
                }
                _ => {}
            }
        }

        // Schedule escalation if not acknowledged
        if alert.level >= AlertLevel::Critical {
            self.schedule_escalation(alert.id, escalation.repeat_interval).await?;
        }

        Ok(())
    }

    fn build_notification(&self, alert: &Alert) -> Notification {
        Notification {
            id: alert.id.clone(),
            level: alert.level,
            title: format!(
                "[{}] {} - {}",
                alert.level,
                alert.alert_type,
                alert.container_id.as_deref().unwrap_or("Facility")
            ),
            message: alert.message.clone(),
            details: alert.details.clone(),
            timestamp: alert.created_at,
            action_required: alert.level >= AlertLevel::Critical,
            acknowledge_url: format!(
                "https://facility.org/alerts/{}/acknowledge",
                alert.id
            ),
        }
    }
}
```

---

## 5. Backup Power Integration

### 5.1 UPS Monitoring

```rust
// ups_monitor.rs
pub struct UPSMonitor {
    ups_client: UPSClient,
    config: UPSConfig,
}

impl UPSMonitor {
    pub async fn check_status(&self) -> UPSStatus {
        let status = self.ups_client.get_status().await;

        UPSStatus {
            online: status.input_voltage > 0.0,
            battery_level: status.battery_percent,
            runtime_remaining: Duration::from_secs(status.runtime_seconds),
            load_percent: status.load_percent,
            input_voltage: status.input_voltage,
            output_voltage: status.output_voltage,
            last_test: status.last_test_date,
            alarms: status.active_alarms,
        }
    }

    pub async fn handle_power_event(&self, event: PowerEvent) {
        match event {
            PowerEvent::MainsLoss => {
                // Log event
                log_event("POWER_FAILURE", "Mains power lost, on UPS battery");

                // Alert
                send_alert(Alert {
                    level: AlertLevel::Critical,
                    alert_type: "POWER_FAILURE".into(),
                    message: "Facility running on UPS backup power".into(),
                    details: serde_json::json!({
                        "battery_level": self.check_status().await.battery_level,
                        "runtime_remaining": self.check_status().await.runtime_remaining,
                    }),
                }).await;

                // Start enhanced monitoring
                self.start_enhanced_monitoring().await;

                // Prepare generator transfer
                self.prepare_generator_transfer().await;
            }

            PowerEvent::BatteryLow => {
                send_alert(Alert {
                    level: AlertLevel::Emergency,
                    alert_type: "BATTERY_LOW".into(),
                    message: "UPS battery critically low".into(),
                    ..Default::default()
                }).await;

                // Initiate emergency procedures
                self.initiate_emergency_protocol().await;
            }

            PowerEvent::MainsRestored => {
                log_event("POWER_RESTORED", "Mains power restored");

                send_alert(Alert {
                    level: AlertLevel::Info,
                    alert_type: "POWER_RESTORED".into(),
                    message: "Mains power has been restored".into(),
                    ..Default::default()
                }).await;

                self.stop_enhanced_monitoring().await;
            }

            _ => {}
        }
    }
}
```

---

## 6. Compliance and Audit

### 6.1 Audit Logging

```rust
// audit_log.rs
#[derive(Debug, Serialize)]
pub struct AuditEntry {
    id: Uuid,
    timestamp: DateTime<Utc>,
    actor: ActorInfo,
    action: AuditAction,
    resource_type: String,
    resource_id: String,
    details: serde_json::Value,
    ip_address: Option<IpAddr>,
    user_agent: Option<String>,
    result: ActionResult,
}

pub struct AuditLogger {
    storage: AuditStorage,
    config: AuditConfig,
}

impl AuditLogger {
    pub async fn log(&self, entry: AuditEntry) -> Result<(), AuditError> {
        // Ensure immutability
        let entry = self.sign_entry(entry)?;

        // Store with replication
        self.storage.store(entry.clone()).await?;

        // Send to external SIEM if configured
        if let Some(siem) = &self.config.siem_endpoint {
            self.forward_to_siem(siem, &entry).await?;
        }

        // Check for compliance violations
        self.check_compliance(&entry).await?;

        Ok(())
    }

    fn sign_entry(&self, mut entry: AuditEntry) -> Result<AuditEntry, AuditError> {
        // Create hash of entry content
        let content = serde_json::to_string(&entry)?;
        let hash = sha256::digest(&content);

        // Sign with facility key
        let signature = self.config.signing_key.sign(&hash)?;

        entry.details["_signature"] = serde_json::json!({
            "hash": hash,
            "signature": signature,
            "algorithm": "Ed25519"
        });

        Ok(entry)
    }
}
```

### 6.2 Regulatory Compliance

```yaml
# compliance_config.yaml
compliance:
  frameworks:
    - name: FDA_21_CFR_1271
      enabled: true
      requirements:
        - id: establishment_registration
        - id: listing_of_hct_ps
        - id: donor_eligibility
        - id: cgtp_compliance
        - id: adverse_reaction_reports

    - name: EU_2004_23_EC
      enabled: true
      requirements:
        - id: quality_and_safety
        - id: traceability
        - id: serious_adverse_events

    - name: WIA_CRYO_PRESERVATION
      enabled: true
      requirements:
        - id: vqi_minimum
          threshold: 0.70
        - id: monitoring_frequency
          max_interval: 1h
        - id: temperature_tolerance
          max_deviation: 5.0

  reporting:
    - type: ANNUAL_REVIEW
      frequency: yearly
      deadline_month: 3

    - type: ADVERSE_EVENT
      frequency: as_needed
      deadline_hours: 24

    - type: QUALITY_METRICS
      frequency: quarterly
```

---

## 7. Deployment Options

### 7.1 Docker Deployment

```yaml
# docker-compose.yml
version: '3.8'

services:
  api:
    image: wia-cryo/api:latest
    ports:
      - "8080:8080"
    environment:
      - DATABASE_URL=postgres://cryo:${DB_PASSWORD}@db:5432/cryo
      - REDIS_URL=redis://redis:6379
    depends_on:
      - db
      - redis

  monitoring:
    image: wia-cryo/monitoring:latest
    environment:
      - TIMESCALE_URL=postgres://cryo:${DB_PASSWORD}@timescaledb:5432/monitoring
      - SENSOR_CONFIG=/config/sensors.yaml
    volumes:
      - ./config:/config:ro
    depends_on:
      - timescaledb

  alerts:
    image: wia-cryo/alerts:latest
    environment:
      - REDIS_URL=redis://redis:6379
      - ALERT_CONFIG=/config/alerts.yaml
    volumes:
      - ./config:/config:ro
    depends_on:
      - redis

  db:
    image: postgres:15
    environment:
      - POSTGRES_DB=cryo
      - POSTGRES_PASSWORD=${DB_PASSWORD}
    volumes:
      - pgdata:/var/lib/postgresql/data

  timescaledb:
    image: timescale/timescaledb:latest-pg15
    environment:
      - POSTGRES_DB=monitoring
      - POSTGRES_PASSWORD=${DB_PASSWORD}
    volumes:
      - tsdata:/var/lib/postgresql/data

  redis:
    image: redis:7
    volumes:
      - redisdata:/data

volumes:
  pgdata:
  tsdata:
  redisdata:
```

### 7.2 Kubernetes Deployment

```yaml
# kubernetes/deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: wia-cryo-api
spec:
  replicas: 3
  selector:
    matchLabels:
      app: wia-cryo-api
  template:
    metadata:
      labels:
        app: wia-cryo-api
    spec:
      containers:
      - name: api
        image: wia-cryo/api:latest
        ports:
        - containerPort: 8080
        env:
        - name: DATABASE_URL
          valueFrom:
            secretKeyRef:
              name: cryo-secrets
              key: database-url
        resources:
          requests:
            memory: "256Mi"
            cpu: "250m"
          limits:
            memory: "1Gi"
            cpu: "1000m"
        livenessProbe:
          httpGet:
            path: /health
            port: 8080
          initialDelaySeconds: 10
          periodSeconds: 30
        readinessProbe:
          httpGet:
            path: /ready
            port: 8080
          initialDelaySeconds: 5
          periodSeconds: 10
```

---

**Phase 4 Integration Guide**
**WIA-CRYO-PRESERVATION v1.0.0**
