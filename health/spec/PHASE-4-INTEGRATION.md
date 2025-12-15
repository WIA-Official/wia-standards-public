# Phase 4: WIA Ecosystem Integration Standard

**WIA Health Standard**
**Version**: 1.0.0
**Status**: Draft
**Last Updated**: December 2025

---

## 1. Overview

### 1.1 Purpose

Phase 4 defines the integration layer for connecting WIA Health data with external systems including:

- **Healthcare Systems**: EHR/EMR via FHIR
- **Wearable Devices**: Apple HealthKit, Google Health Connect, Fitbit, Garmin
- **Visualization**: Real-time dashboards and digital twin rendering
- **Export Formats**: CSV, JSON, PDF, HL7

### 1.2 Goals

1. Seamless data exchange with healthcare providers
2. Automatic import from consumer wearables
3. Real-time health monitoring dashboards
4. 3D digital twin visualization
5. Multi-channel health alerts

### 1.3 Relationship to Previous Phases

| Phase | Contribution |
|-------|--------------|
| Phase 1 | Data types for integration payloads |
| Phase 2 | API for data access and manipulation |
| Phase 3 | Protocol for real-time data streaming |
| Phase 4 | Adapters for external system integration |

---

## 2. Integration Architecture

### 2.1 Layer Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                     Application Layer                           │
│              (Mobile Apps, Web Dashboards, CLI)                 │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                  Integration Manager                            │
│    ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐         │
│    │  Import  │ │  Export  │ │  Stream  │ │  Alert   │         │
│    │ Manager  │ │ Manager  │ │ Manager  │ │ Manager  │         │
│    └──────────┘ └──────────┘ └──────────┘ └──────────┘         │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                     Adapter Layer                               │
│  ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐        │
│  │  FHIR  │ │HealthKit│ │ Health │ │ Fitbit │ │ Garmin │        │
│  │Adapter │ │ Adapter │ │Connect │ │Adapter │ │Adapter │        │
│  └────────┘ └────────┘ └────────┘ └────────┘ └────────┘        │
│  ┌────────┐ ┌────────┐ ┌────────┐                               │
│  │Dashboard│ │DigTwin │ │ Export │                              │
│  │ Adapter│ │Visualize│ │Adapter │                              │
│  └────────┘ └────────┘ └────────┘                               │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    External Systems                             │
│  ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐        │
│  │  EHR/  │ │  iOS   │ │Android │ │Wearable│ │ Custom │        │
│  │  FHIR  │ │ Health │ │ Health │ │ Cloud  │ │  API   │        │
│  └────────┘ └────────┘ └────────┘ └────────┘ └────────┘        │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 Component Responsibilities

| Component | Responsibility |
|-----------|----------------|
| **Integration Manager** | Orchestrates all adapters |
| **Import Manager** | Pulls data from external sources |
| **Export Manager** | Pushes data to external systems |
| **Stream Manager** | Real-time data flow coordination |
| **Alert Manager** | Multi-channel notifications |

---

## 3. Adapter Interface

### 3.1 Base Adapter Trait

```rust
/// Integration adapter trait
#[async_trait]
pub trait IntegrationAdapter: Send + Sync {
    /// Adapter type identifier
    fn adapter_type(&self) -> AdapterType;

    /// Human-readable name
    fn name(&self) -> &str;

    /// Check if adapter is available
    async fn is_available(&self) -> bool;

    /// Initialize the adapter
    async fn initialize(&mut self, config: AdapterConfig) -> Result<()>;

    /// Shutdown the adapter
    async fn shutdown(&mut self) -> Result<()>;

    /// Get adapter capabilities
    fn capabilities(&self) -> AdapterCapabilities;
}

/// Adapter type enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdapterType {
    Fhir,
    HealthKit,
    HealthConnect,
    Fitbit,
    Garmin,
    Dashboard,
    DigitalTwin,
    Export,
    Custom,
}

/// Adapter capabilities
#[derive(Debug, Clone)]
pub struct AdapterCapabilities {
    pub can_import: bool,
    pub can_export: bool,
    pub can_stream: bool,
    pub supports_auth: bool,
    pub data_types: Vec<String>,
}
```

### 3.2 Import Adapter Trait

```rust
/// Adapter for importing external data
#[async_trait]
pub trait ImportAdapter: IntegrationAdapter {
    /// Import data from external source
    async fn import(&self, options: ImportOptions) -> Result<ImportResult>;

    /// Start continuous import stream
    async fn start_import_stream(
        &self,
        callback: Box<dyn Fn(ImportEvent) + Send + Sync>,
    ) -> Result<StreamHandle>;

    /// Stop import stream
    async fn stop_import_stream(&self, handle: StreamHandle) -> Result<()>;
}

/// Import options
#[derive(Debug, Clone)]
pub struct ImportOptions {
    pub data_types: Vec<String>,
    pub start_date: Option<DateTime<Utc>>,
    pub end_date: Option<DateTime<Utc>>,
    pub limit: Option<u32>,
}

/// Import result
#[derive(Debug, Clone)]
pub struct ImportResult {
    pub records_imported: u64,
    pub data_types: Vec<String>,
    pub profile_updates: Option<HealthProfile>,
    pub errors: Vec<ImportError>,
}
```

### 3.3 Export Adapter Trait

```rust
/// Adapter for exporting data
#[async_trait]
pub trait ExportAdapter: IntegrationAdapter {
    /// Export profile to external system
    async fn export(&self, profile: &HealthProfile, options: ExportOptions) -> Result<ExportResult>;

    /// Export specific data types
    async fn export_data(
        &self,
        data: ExportData,
        options: ExportOptions,
    ) -> Result<ExportResult>;
}

/// Export options
#[derive(Debug, Clone)]
pub struct ExportOptions {
    pub format: ExportFormat,
    pub include_metadata: bool,
    pub anonymize: bool,
    pub destination: Option<String>,
}

/// Export format
#[derive(Debug, Clone, Copy)]
pub enum ExportFormat {
    FhirJson,
    FhirXml,
    Csv,
    Json,
    Pdf,
    Hl7v2,
}

/// Export result
#[derive(Debug, Clone)]
pub struct ExportResult {
    pub success: bool,
    pub records_exported: u64,
    pub destination: String,
    pub resource_ids: Vec<String>,
}
```

---

## 4. FHIR Integration

### 4.1 FHIR Adapter

```rust
/// FHIR R4 integration adapter
pub struct FhirAdapter {
    base_url: String,
    auth_token: Option<String>,
    client: HttpClient,
}

impl FhirAdapter {
    /// Create new FHIR adapter
    pub fn new(base_url: &str) -> Self;

    /// Set authentication token
    pub fn with_auth(self, token: &str) -> Self;

    /// Convert WIA profile to FHIR Bundle
    pub fn to_fhir_bundle(&self, profile: &HealthProfile) -> FhirBundle;

    /// Convert WIA biomarker to FHIR Observation
    pub fn to_fhir_observation(&self, biomarker: &Measurement) -> FhirObservation;

    /// Convert FHIR Patient to WIA Subject
    pub fn from_fhir_patient(&self, patient: &FhirPatient) -> Subject;
}
```

### 4.2 FHIR Resource Mappings

| WIA Type | FHIR Resource |
|----------|---------------|
| `Subject` | Patient |
| `Measurement` | Observation |
| `Intervention` | MedicationRequest, Procedure |
| `AgingClocks` | Observation (custom profile) |
| `GenomicVariant` | MolecularSequence, Observation |
| `TelomereMeasurement` | Observation (custom profile) |

### 4.3 FHIR Observation Example

```json
{
  "resourceType": "Observation",
  "id": "heart-rate-001",
  "status": "final",
  "category": [{
    "coding": [{
      "system": "http://terminology.hl7.org/CodeSystem/observation-category",
      "code": "vital-signs",
      "display": "Vital Signs"
    }]
  }],
  "code": {
    "coding": [{
      "system": "http://loinc.org",
      "code": "8867-4",
      "display": "Heart rate"
    }],
    "text": "Heart Rate"
  },
  "subject": {
    "reference": "Patient/example"
  },
  "effectiveDateTime": "2025-12-14T10:30:00Z",
  "valueQuantity": {
    "value": 72,
    "unit": "beats/minute",
    "system": "http://unitsofmeasure.org",
    "code": "/min"
  },
  "meta": {
    "profile": ["http://wia.live/fhir/StructureDefinition/wia-health-observation"]
  }
}
```

---

## 5. Wearable Device Integration

### 5.1 HealthKit Adapter (iOS)

```rust
/// Apple HealthKit adapter
pub struct HealthKitAdapter {
    authorized_types: Vec<String>,
    query_handlers: HashMap<String, QueryHandle>,
}

impl HealthKitAdapter {
    /// Request authorization for data types
    pub async fn request_authorization(
        &mut self,
        read_types: Vec<HealthKitType>,
        write_types: Vec<HealthKitType>,
    ) -> Result<AuthStatus>;

    /// Query samples
    pub async fn query_samples(
        &self,
        sample_type: HealthKitType,
        start: DateTime<Utc>,
        end: DateTime<Utc>,
    ) -> Result<Vec<HealthKitSample>>;

    /// Start background delivery
    pub async fn enable_background_delivery(
        &self,
        types: Vec<HealthKitType>,
        frequency: DeliveryFrequency,
    ) -> Result<()>;

    /// Convert to WIA format
    pub fn to_wia_measurement(&self, sample: &HealthKitSample) -> Measurement;
}

/// HealthKit data types
#[derive(Debug, Clone)]
pub enum HealthKitType {
    HeartRate,
    BloodGlucose,
    BloodPressureSystolic,
    BloodPressureDiastolic,
    OxygenSaturation,
    BodyTemperature,
    StepCount,
    SleepAnalysis,
    HeartRateVariability,
}
```

### 5.2 Health Connect Adapter (Android)

```rust
/// Google Health Connect adapter
pub struct HealthConnectAdapter {
    client: HealthConnectClient,
    permissions: HashSet<Permission>,
}

impl HealthConnectAdapter {
    /// Request permissions
    pub async fn request_permissions(
        &mut self,
        permissions: Vec<Permission>,
    ) -> Result<PermissionResult>;

    /// Read records
    pub async fn read_records<T: HealthRecord>(
        &self,
        request: ReadRecordsRequest,
    ) -> Result<Vec<T>>;

    /// Insert records
    pub async fn insert_records<T: HealthRecord>(
        &self,
        records: Vec<T>,
    ) -> Result<InsertResult>;

    /// Convert to WIA format
    pub fn to_wia_biomarkers(&self, records: Vec<impl HealthRecord>) -> BiomarkerProfile;
}
```

### 5.3 Wearable Data Type Mappings

| Source | Data Type | WIA Field |
|--------|-----------|-----------|
| HealthKit | HKQuantityTypeIdentifierHeartRate | `biomarkers.cardiovascular.heart_rate` |
| HealthKit | HKQuantityTypeIdentifierBloodGlucose | `biomarkers.metabolic.glucose` |
| Health Connect | HeartRateRecord | `biomarkers.cardiovascular.heart_rate` |
| Health Connect | BloodGlucoseRecord | `biomarkers.metabolic.glucose` |
| Fitbit | activities/heart | `biomarkers.cardiovascular.heart_rate` |
| Garmin | dailies/heartRate | `biomarkers.cardiovascular.heart_rate` |

---

## 6. Dashboard Integration

### 6.1 Dashboard Adapter

```rust
/// Real-time dashboard adapter
pub struct DashboardAdapter {
    websocket_url: String,
    update_interval: Duration,
    active_widgets: Vec<Widget>,
}

impl DashboardAdapter {
    /// Connect to dashboard
    pub async fn connect(&mut self) -> Result<()>;

    /// Push update to dashboard
    pub async fn push_update(&self, update: DashboardUpdate) -> Result<()>;

    /// Register widget
    pub fn register_widget(&mut self, widget: Widget);

    /// Start real-time streaming
    pub async fn start_streaming(
        &self,
        profile: &HealthProfile,
        interval: Duration,
    ) -> Result<StreamHandle>;
}

/// Dashboard update
#[derive(Debug, Clone, Serialize)]
pub struct DashboardUpdate {
    pub widget_id: String,
    pub data: serde_json::Value,
    pub timestamp: DateTime<Utc>,
    pub update_type: UpdateType,
}

/// Widget configuration
#[derive(Debug, Clone)]
pub struct Widget {
    pub id: String,
    pub widget_type: WidgetType,
    pub data_source: String,
    pub refresh_rate: Duration,
    pub config: serde_json::Value,
}

/// Widget types
#[derive(Debug, Clone)]
pub enum WidgetType {
    LineChart,
    GaugeChart,
    NumberCard,
    Table,
    Heatmap,
    DigitalTwin3D,
    AlertList,
}
```

### 6.2 Dashboard Message Format

```json
{
  "type": "dashboard_update",
  "widget_id": "heart-rate-chart",
  "timestamp": "2025-12-14T10:30:00.000Z",
  "data": {
    "series": [{
      "name": "Heart Rate",
      "data": [
        {"x": "2025-12-14T10:25:00Z", "y": 72},
        {"x": "2025-12-14T10:26:00Z", "y": 75},
        {"x": "2025-12-14T10:27:00Z", "y": 71}
      ]
    }],
    "thresholds": {
      "low": 60,
      "high": 100,
      "critical_low": 50,
      "critical_high": 120
    }
  },
  "metadata": {
    "source": "apple_watch",
    "quality": 0.95
  }
}
```

---

## 7. Digital Twin Visualization

### 7.1 Digital Twin Visualizer

```rust
/// 3D digital twin visualization adapter
pub struct DigitalTwinVisualizer {
    renderer: Renderer3D,
    body_model: BodyModel,
    organ_models: HashMap<String, OrganModel>,
}

impl DigitalTwinVisualizer {
    /// Initialize with body model
    pub fn with_model(model_path: &str) -> Result<Self>;

    /// Update visualization from profile
    pub fn update_from_profile(&mut self, profile: &HealthProfile);

    /// Highlight organ system
    pub fn highlight_system(&mut self, system: OrganSystem, color: Color);

    /// Show biomarker overlay
    pub fn show_biomarker_overlay(
        &mut self,
        biomarker: &str,
        value: f64,
        normal_range: (f64, f64),
    );

    /// Animate aging simulation
    pub async fn animate_aging_simulation(
        &mut self,
        simulation: &SimulationResult,
        duration: Duration,
    );

    /// Export as image/video
    pub fn export(&self, format: MediaFormat) -> Result<Vec<u8>>;
}

/// Organ system enumeration
#[derive(Debug, Clone)]
pub enum OrganSystem {
    Cardiovascular,
    Respiratory,
    Digestive,
    Nervous,
    Endocrine,
    Immune,
    Musculoskeletal,
    Integumentary,
}
```

### 7.2 Visualization API

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/twin/render` | POST | Render current state |
| `/twin/animate` | POST | Start animation sequence |
| `/twin/highlight` | POST | Highlight organ/system |
| `/twin/overlay` | POST | Add data overlay |
| `/twin/export` | GET | Export image/video |

---

## 8. Alert System

### 8.1 Alert Manager

```rust
/// Multi-channel alert manager
pub struct AlertManager {
    channels: Vec<Box<dyn AlertChannel>>,
    rules: Vec<AlertRule>,
    history: AlertHistory,
}

impl AlertManager {
    /// Register alert channel
    pub fn register_channel(&mut self, channel: Box<dyn AlertChannel>);

    /// Add alert rule
    pub fn add_rule(&mut self, rule: AlertRule);

    /// Evaluate profile against rules
    pub async fn evaluate(&self, profile: &HealthProfile) -> Vec<Alert>;

    /// Send alert through all channels
    pub async fn send_alert(&self, alert: Alert) -> Result<AlertDelivery>;
}

/// Alert channel trait
#[async_trait]
pub trait AlertChannel: Send + Sync {
    fn channel_type(&self) -> AlertChannelType;
    async fn send(&self, alert: &Alert) -> Result<()>;
}

/// Alert channel types
#[derive(Debug, Clone)]
pub enum AlertChannelType {
    Push,           // Mobile push notification
    Sms,            // SMS message
    Email,          // Email notification
    Webhook,        // HTTP webhook
    InApp,          // In-app notification
    VoiceCall,      // Automated voice call
}

/// Alert rule
#[derive(Debug, Clone)]
pub struct AlertRule {
    pub id: String,
    pub name: String,
    pub condition: AlertCondition,
    pub severity: AlertSeverity,
    pub channels: Vec<AlertChannelType>,
    pub cooldown: Duration,
}

/// Alert condition
#[derive(Debug, Clone)]
pub enum AlertCondition {
    Threshold {
        field: String,
        operator: ComparisonOp,
        value: f64,
    },
    RateOfChange {
        field: String,
        change_percent: f64,
        window: Duration,
    },
    Pattern {
        pattern_type: String,
        parameters: serde_json::Value,
    },
}
```

### 8.2 Alert Examples

```json
{
  "alert_id": "alert-001",
  "timestamp": "2025-12-14T10:30:00.000Z",
  "severity": "warning",
  "category": "biomarker_threshold",
  "title": "Elevated Heart Rate Detected",
  "message": "Heart rate of 120 bpm exceeds threshold of 100 bpm",
  "data": {
    "biomarker": "heart_rate",
    "value": 120,
    "threshold": 100,
    "unit": "bpm"
  },
  "actions": [
    {"type": "acknowledge", "label": "Acknowledge"},
    {"type": "view_details", "url": "/health/vitals/heart-rate"},
    {"type": "contact_provider", "phone": "+1-800-HEALTH"}
  ],
  "delivery": {
    "channels": ["push", "in_app"],
    "sent_at": "2025-12-14T10:30:01.000Z",
    "delivered": true
  }
}
```

---

## 9. Export Formats

### 9.1 Export Manager

```rust
/// Data export manager
pub struct ExportManager {
    exporters: HashMap<ExportFormat, Box<dyn Exporter>>,
}

impl ExportManager {
    /// Export to format
    pub async fn export(
        &self,
        profile: &HealthProfile,
        format: ExportFormat,
        options: ExportOptions,
    ) -> Result<ExportOutput>;

    /// Export to file
    pub async fn export_to_file(
        &self,
        profile: &HealthProfile,
        path: &Path,
        format: ExportFormat,
    ) -> Result<()>;
}

/// Exporter trait
#[async_trait]
pub trait Exporter: Send + Sync {
    fn format(&self) -> ExportFormat;
    async fn export(&self, profile: &HealthProfile, options: &ExportOptions) -> Result<Vec<u8>>;
}
```

### 9.2 Supported Formats

| Format | Extension | Use Case |
|--------|-----------|----------|
| **FHIR JSON** | .json | EHR integration |
| **FHIR XML** | .xml | Legacy EHR systems |
| **CSV** | .csv | Spreadsheet analysis |
| **JSON** | .json | Developer integration |
| **PDF** | .pdf | Patient reports |
| **HL7 v2** | .hl7 | Hospital systems |

---

## 10. Security

### 10.1 Authentication

| Method | Use Case |
|--------|----------|
| **OAuth 2.0** | FHIR servers, Cloud APIs |
| **SMART on FHIR** | Healthcare apps |
| **API Keys** | Simple integrations |
| **mTLS** | Device-to-device |
| **JWT** | Session tokens |

### 10.2 Data Protection

- All data encrypted in transit (TLS 1.3)
- PHI anonymization options
- HIPAA/GDPR compliance hooks
- Audit logging for all operations
- Consent tracking per adapter

---

## 11. Error Handling

### 11.1 Integration Errors

```rust
/// Integration error types
#[derive(Debug, Error)]
pub enum IntegrationError {
    #[error("Adapter not found: {0}")]
    AdapterNotFound(String),

    #[error("Authentication failed: {0}")]
    AuthenticationFailed(String),

    #[error("Authorization denied: {0}")]
    AuthorizationDenied(String),

    #[error("Connection failed: {0}")]
    ConnectionFailed(String),

    #[error("Data mapping error: {0}")]
    MappingError(String),

    #[error("Rate limited: retry after {0}s")]
    RateLimited(u64),

    #[error("External service error: {0}")]
    ExternalServiceError(String),
}
```

### 11.2 Retry Policy

| Error Type | Retry | Backoff |
|------------|-------|---------|
| Connection | 3x | Exponential (1s, 2s, 4s) |
| Rate Limited | 1x | Wait for retry-after |
| Auth Failed | 1x | Refresh token first |
| Mapping | 0x | Log and skip |

---

## 12. Examples

### 12.1 FHIR Export Example

```rust
use wia_health::prelude::*;
use wia_health::integration::*;

#[tokio::main]
async fn main() -> Result<()> {
    // Load profile
    let profile = load_profile()?;

    // Create FHIR adapter
    let mut fhir = FhirAdapter::new("https://fhir.example.com/r4")
        .with_auth("Bearer eyJ...");

    fhir.initialize(AdapterConfig::default()).await?;

    // Export to FHIR
    let result = fhir.export(&profile, ExportOptions {
        format: ExportFormat::FhirJson,
        include_metadata: true,
        anonymize: false,
        destination: None,
    }).await?;

    println!("Exported {} records", result.records_exported);

    Ok(())
}
```

### 12.2 Wearable Import Example

```rust
use wia_health::prelude::*;
use wia_health::integration::*;

#[tokio::main]
async fn main() -> Result<()> {
    // Create HealthKit adapter
    let mut healthkit = HealthKitAdapter::new();

    // Request authorization
    healthkit.request_authorization(
        vec![HealthKitType::HeartRate, HealthKitType::BloodGlucose],
        vec![],
    ).await?;

    // Import last 7 days
    let result = healthkit.import(ImportOptions {
        data_types: vec!["heart_rate".into(), "blood_glucose".into()],
        start_date: Some(Utc::now() - Duration::days(7)),
        end_date: Some(Utc::now()),
        limit: None,
    }).await?;

    println!("Imported {} records", result.records_imported);

    Ok(())
}
```

### 12.3 Dashboard Streaming Example

```rust
use wia_health::prelude::*;
use wia_health::integration::*;

#[tokio::main]
async fn main() -> Result<()> {
    let profile = load_profile()?;

    // Create dashboard adapter
    let mut dashboard = DashboardAdapter::new("wss://dashboard.example.com/ws");

    // Register widgets
    dashboard.register_widget(Widget {
        id: "heart-rate".into(),
        widget_type: WidgetType::LineChart,
        data_source: "biomarkers.cardiovascular.heart_rate".into(),
        refresh_rate: Duration::from_secs(1),
        config: serde_json::json!({}),
    });

    // Connect and start streaming
    dashboard.connect().await?;
    dashboard.start_streaming(&profile, Duration::from_secs(1)).await?;

    Ok(())
}
```

---

## 13. References

### 13.1 Standards

- [HL7 FHIR R4](https://hl7.org/fhir/R4/)
- [SMART on FHIR](https://docs.smarthealthit.org/)
- [Apple HealthKit](https://developer.apple.com/documentation/healthkit)
- [Google Health Connect](https://developer.android.com/health-and-fitness/guides/health-connect)

### 13.2 WIA Standards

- [Phase 1: Data Format](PHASE-1-DATA-FORMAT.md)
- [Phase 3: Protocol](PHASE-3-PROTOCOL.md)
- [Research: Phase 4](RESEARCH-PHASE-4.md)

---

**弘益人間** - Benefit All Humanity

