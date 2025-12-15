# Phase 4: WIA Physics Ecosystem Integration

**WIA Physics Standard - Ecosystem Integration Specification**

---

**Version**: 1.0.0
**Status**: Draft
**Date**: 2025-12-14
**Author**: WIA / SmileStory Inc.

---

## Table of Contents

1. [Overview](#1-overview)
2. [Integration Architecture](#2-integration-architecture)
3. [Integration Adapter Interface](#3-integration-adapter-interface)
4. [EPICS Integration](#4-epics-integration)
5. [HDF5 Integration](#5-hdf5-integration)
6. [Time-Series Database Integration](#6-time-series-database-integration)
7. [Visualization Integration](#7-visualization-integration)
8. [Integration Manager](#8-integration-manager)
9. [Events and Callbacks](#9-events-and-callbacks)
10. [Error Handling](#10-error-handling)
11. [Examples](#11-examples)
12. [References](#12-references)

---

## 1. Overview

### 1.1 Purpose

Phase 4 defines standards for integrating WIA Physics data with external systems:
- **Control Systems**: EPICS, TANGO Controls, OPC UA
- **Data Archives**: HDF5, ROOT files, time-series databases
- **Visualization**: Grafana dashboards, web-based viewers
- **External Services**: CERN Open Data, DOI services

### 1.2 Scope

```
Input: Phase 1-3 Physics Data
Output:
├─ EPICS Channels       → Accelerator/Fusion Control Systems
├─ HDF5 Files           → Long-term Data Archives
├─ InfluxDB/Grafana     → Real-time Monitoring Dashboards
└─ ROOT Files           → Particle Physics Analysis
```

### 1.3 Relationship with Previous Phases

| Phase | Role | Phase 4 Integration |
|-------|------|---------------------|
| Phase 1 | Data Format | Source data structures |
| Phase 2 | Rust SDK | API for data access |
| Phase 3 | Protocol | Real-time streaming |
| **Phase 4** | **Integration** | **External system export** |

---

## 2. Integration Architecture

### 2.1 Full Pipeline

```
┌─────────────────────────────────────────────────────────────┐
│                    Physics Experiment                        │
│          (Fusion Reactor, Accelerator, Detector)            │
└─────────────────────────────────────────────────────────────┘
                              │
                       [Sensor Data]
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│     Phase 1: Data Format → Phase 2: SDK → Phase 3: Protocol │
└─────────────────────────────────────────────────────────────┘
                              │
                     [WIA Physics Data]
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   IntegrationManager                         │
│                   (Phase 4: Integration)                     │
├─────────────┬─────────────┬─────────────┬───────────────────┤
│ EPICSAdapter│ HDF5Adapter │InfluxAdapter│ GrafanaAdapter    │
│(Control Sys)│(Data Archive)│(Time-Series)│(Visualization)   │
└──────┬──────┴──────┬──────┴──────┬──────┴────────┬──────────┘
       │             │             │               │
       ▼             ▼             ▼               ▼
  ┌─────────┐   ┌─────────┐   ┌─────────┐   ┌─────────────┐
  │EPICS IOC│   │HDF5 File│   │InfluxDB │   │Grafana Dash │
  └─────────┘   └─────────┘   └─────────┘   └─────────────┘
       │             │             │               │
       ▼             ▼             ▼               ▼
  Control Room   Data Archive   Monitoring    Operations
```

### 2.2 Layer Structure

```
Layer 0: Input
         └─ WIA Physics Data (FusionData, ParticleData, etc.)

Layer 1: Integration Management
         └─ IntegrationManager

Layer 2: Integration Adapters
         ├─ EPICSAdapter
         ├─ TANGOAdapter
         ├─ HDF5Adapter
         ├─ InfluxDBAdapter
         ├─ GrafanaAdapter
         └─ ROOTAdapter

Layer 3: External Systems
         ├─ EPICS IOCs / PVs
         ├─ HDF5 Files
         ├─ Time-Series Databases
         └─ Visualization Dashboards
```

---

## 3. Integration Adapter Interface

### 3.1 Base Interface

#### Rust Definition

```rust
use async_trait::async_trait;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Integration type enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IntegrationType {
    Epics,
    Tango,
    Hdf5,
    InfluxDb,
    Grafana,
    Root,
    OpcUa,
    Custom,
}

/// Integration adapter state
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IntegrationState {
    Disconnected,
    Connecting,
    Connected,
    Exporting,
    Error,
}

/// Integration options
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct IntegrationOptions {
    /// Connection endpoint (URL, path, etc.)
    pub endpoint: Option<String>,

    /// Authentication credentials
    pub credentials: Option<Credentials>,

    /// Timeout in milliseconds
    pub timeout_ms: Option<u64>,

    /// Additional options
    #[serde(flatten)]
    pub extra: HashMap<String, serde_json::Value>,
}

/// Authentication credentials
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Credentials {
    pub username: Option<String>,
    pub password: Option<String>,
    pub token: Option<String>,
}

/// Integration adapter trait
#[async_trait]
pub trait IntegrationAdapter: Send + Sync {
    /// Get integration type
    fn integration_type(&self) -> IntegrationType;

    /// Get adapter name
    fn name(&self) -> &str;

    /// Get current state
    fn state(&self) -> IntegrationState;

    /// Initialize adapter
    async fn initialize(&mut self, options: IntegrationOptions) -> Result<(), IntegrationError>;

    /// Connect to external system
    async fn connect(&mut self) -> Result<(), IntegrationError>;

    /// Disconnect from external system
    async fn disconnect(&mut self) -> Result<(), IntegrationError>;

    /// Export physics data
    async fn export(&self, data: &PhysicsData) -> Result<ExportResult, IntegrationError>;

    /// Import physics data (if supported)
    async fn import(&self, source: &str) -> Result<PhysicsData, IntegrationError>;

    /// Check if adapter is available
    fn is_available(&self) -> bool;

    /// Dispose resources
    async fn dispose(&mut self) -> Result<(), IntegrationError>;
}

/// Unified physics data for export
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PhysicsData {
    pub data_type: PhysicsDataType,
    pub metadata: DataMetadata,
    pub payload: serde_json::Value,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PhysicsDataType {
    Fusion,
    Particle,
    DarkMatter,
    Antimatter,
    QuantumGravity,
    TimeCrystal,
}

/// Export result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExportResult {
    pub success: bool,
    pub records_exported: u64,
    pub destination: String,
    pub timestamp: i64,
    pub details: Option<String>,
}
```

---

## 4. EPICS Integration

### 4.1 Overview

EPICS (Experimental Physics and Industrial Control System) adapter enables integration with accelerator and fusion facility control systems.

Supported protocols:
- **Channel Access (CA)**: Traditional EPICS protocol (port 5064)
- **pvAccess (PVA)**: Modern EPICS7 protocol (port 5075)

### 4.2 Interface

```rust
/// EPICS-specific adapter interface
#[async_trait]
pub trait EPICSAdapter: IntegrationAdapter {
    /// Get process variable value
    async fn get_pv(&self, pv_name: &str) -> Result<PVValue, IntegrationError>;

    /// Set process variable value
    async fn put_pv(&self, pv_name: &str, value: PVValue) -> Result<(), IntegrationError>;

    /// Monitor process variable changes
    async fn monitor_pv(
        &self,
        pv_name: &str,
        callback: Box<dyn Fn(PVValue) + Send + Sync>,
    ) -> Result<MonitorHandle, IntegrationError>;

    /// Stop monitoring
    async fn stop_monitor(&self, handle: MonitorHandle) -> Result<(), IntegrationError>;

    /// List available PVs with pattern
    async fn list_pvs(&self, pattern: &str) -> Result<Vec<String>, IntegrationError>;
}

/// EPICS Process Variable value
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PVValue {
    /// Value
    pub value: PVData,

    /// Timestamp
    pub timestamp: i64,

    /// Alarm severity
    pub severity: AlarmSeverity,

    /// Alarm status
    pub status: AlarmStatus,

    /// Units (if available)
    pub units: Option<String>,

    /// Display limits
    pub limits: Option<PVLimits>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum PVData {
    Double(f64),
    DoubleArray(Vec<f64>),
    Int(i32),
    IntArray(Vec<i32>),
    String(String),
    Enum(i16),
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum AlarmSeverity {
    NoAlarm,
    Minor,
    Major,
    Invalid,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PVLimits {
    pub low_alarm: f64,
    pub low_warn: f64,
    pub high_warn: f64,
    pub high_alarm: f64,
}
```

### 4.3 PV Naming Convention

WIA Physics data maps to EPICS PVs using this naming convention:

```
<facility>:<system>:<subsystem>:<parameter>

Examples:
  ITER:PLASMA:TEMP:CORE         → Core plasma temperature
  ITER:PLASMA:DENSITY:AVG       → Average plasma density
  ITER:MAGNETS:TF:CURRENT       → Toroidal field current
  ITER:HEATING:NBI:POWER        → NBI heating power
```

### 4.4 Data Mapping

| WIA Physics Field | EPICS PV Type | Example PV |
|-------------------|---------------|------------|
| `plasma.temperature.value` | DBR_DOUBLE | `ITER:PLASMA:TEMP:CORE` |
| `plasma.density.value` | DBR_DOUBLE | `ITER:PLASMA:DENSITY:AVG` |
| `magnetics.toroidal_field.value` | DBR_DOUBLE | `ITER:MAGNETS:TF:FIELD` |
| `energy.heating_power` | DBR_DOUBLE | `ITER:HEATING:TOTAL` |
| `energy.q_factor` | DBR_DOUBLE | `ITER:ENERGY:QFACTOR` |

### 4.5 Implementation Skeleton

```rust
pub struct EPICSIntegrationAdapter {
    state: IntegrationState,
    endpoint: String,
    protocol: EPICSProtocol,
    // connection: Option<EPICSConnection>,
}

#[derive(Debug, Clone, Copy)]
pub enum EPICSProtocol {
    ChannelAccess,
    PvAccess,
}

impl EPICSIntegrationAdapter {
    pub fn new() -> Self {
        Self {
            state: IntegrationState::Disconnected,
            endpoint: "localhost:5064".to_string(),
            protocol: EPICSProtocol::ChannelAccess,
        }
    }

    /// Map fusion data to EPICS PVs
    pub fn fusion_to_pvs(data: &FusionData, prefix: &str) -> Vec<(String, PVValue)> {
        let mut pvs = Vec::new();

        if let Some(ref plasma) = data.plasma {
            if let Some(ref temp) = plasma.temperature {
                pvs.push((
                    format!("{}:PLASMA:TEMP:CORE", prefix),
                    PVValue {
                        value: PVData::Double(temp.value),
                        timestamp: chrono::Utc::now().timestamp_millis(),
                        severity: AlarmSeverity::NoAlarm,
                        status: AlarmStatus::NoAlarm,
                        units: Some(temp.unit.clone()),
                        limits: None,
                    },
                ));
            }
        }

        pvs
    }
}

#[async_trait]
impl IntegrationAdapter for EPICSIntegrationAdapter {
    fn integration_type(&self) -> IntegrationType {
        IntegrationType::Epics
    }

    fn name(&self) -> &str {
        "EPICS Channel Access"
    }

    fn state(&self) -> IntegrationState {
        self.state
    }

    async fn initialize(&mut self, options: IntegrationOptions) -> Result<(), IntegrationError> {
        if let Some(endpoint) = options.endpoint {
            self.endpoint = endpoint;
        }
        Ok(())
    }

    async fn connect(&mut self) -> Result<(), IntegrationError> {
        self.state = IntegrationState::Connecting;
        // TODO: Actual EPICS connection
        self.state = IntegrationState::Connected;
        Ok(())
    }

    async fn disconnect(&mut self) -> Result<(), IntegrationError> {
        self.state = IntegrationState::Disconnected;
        Ok(())
    }

    async fn export(&self, data: &PhysicsData) -> Result<ExportResult, IntegrationError> {
        // Export physics data to EPICS PVs
        Ok(ExportResult {
            success: true,
            records_exported: 1,
            destination: self.endpoint.clone(),
            timestamp: chrono::Utc::now().timestamp_millis(),
            details: Some("Exported to EPICS".to_string()),
        })
    }

    async fn import(&self, source: &str) -> Result<PhysicsData, IntegrationError> {
        Err(IntegrationError::NotSupported("Import not implemented".to_string()))
    }

    fn is_available(&self) -> bool {
        self.state == IntegrationState::Connected
    }

    async fn dispose(&mut self) -> Result<(), IntegrationError> {
        self.disconnect().await
    }
}
```

---

## 5. HDF5 Integration

### 5.1 Overview

HDF5 (Hierarchical Data Format) adapter enables long-term data archival with efficient compression and self-describing metadata.

### 5.2 Interface

```rust
/// HDF5-specific adapter interface
#[async_trait]
pub trait HDF5Adapter: IntegrationAdapter {
    /// Create new HDF5 file
    async fn create_file(&mut self, path: &str) -> Result<(), IntegrationError>;

    /// Open existing HDF5 file
    async fn open_file(&mut self, path: &str, mode: FileMode) -> Result<(), IntegrationError>;

    /// Close current file
    async fn close_file(&mut self) -> Result<(), IntegrationError>;

    /// Create group
    async fn create_group(&self, path: &str) -> Result<(), IntegrationError>;

    /// Write dataset
    async fn write_dataset<T: HDF5Data>(
        &self,
        path: &str,
        data: &[T],
        options: DatasetOptions,
    ) -> Result<(), IntegrationError>;

    /// Read dataset
    async fn read_dataset<T: HDF5Data>(&self, path: &str) -> Result<Vec<T>, IntegrationError>;

    /// Write attribute
    async fn write_attribute(
        &self,
        path: &str,
        name: &str,
        value: AttributeValue,
    ) -> Result<(), IntegrationError>;

    /// Read attribute
    async fn read_attribute(&self, path: &str, name: &str) -> Result<AttributeValue, IntegrationError>;
}

#[derive(Debug, Clone, Copy)]
pub enum FileMode {
    ReadOnly,
    ReadWrite,
    Create,
    Truncate,
}

#[derive(Debug, Clone, Default)]
pub struct DatasetOptions {
    /// Chunk size for chunked storage
    pub chunk_size: Option<Vec<usize>>,

    /// Compression level (0-9, 0 = none)
    pub compression: u8,

    /// Enable shuffle filter
    pub shuffle: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum AttributeValue {
    String(String),
    Int(i64),
    Float(f64),
    IntArray(Vec<i64>),
    FloatArray(Vec<f64>),
}

/// Trait for HDF5-compatible data types
pub trait HDF5Data: Clone + Send + Sync {}
impl HDF5Data for f64 {}
impl HDF5Data for f32 {}
impl HDF5Data for i64 {}
impl HDF5Data for i32 {}
```

### 5.3 File Structure

WIA Physics HDF5 files follow this hierarchical structure:

```
/                           (root)
├─ metadata/                (experiment metadata)
│   ├─ experiment_id        [attr: string]
│   ├─ experiment_name      [attr: string]
│   ├─ created              [attr: string, ISO8601]
│   ├─ wia_version          [attr: string]
│   └─ schema_version       [attr: string]
│
├─ fusion/                  (fusion data)
│   ├─ plasma/
│   │   ├─ temperature      [dataset: float64, shape: (N,)]
│   │   ├─ temperature_err  [dataset: float64, shape: (N,)]
│   │   ├─ density          [dataset: float64, shape: (N,)]
│   │   ├─ density_err      [dataset: float64, shape: (N,)]
│   │   └─ timestamps       [dataset: int64, shape: (N,)]
│   │
│   ├─ magnetics/
│   │   ├─ toroidal_field   [dataset: float64, shape: (N,)]
│   │   ├─ plasma_current   [dataset: float64, shape: (N,)]
│   │   └─ timestamps       [dataset: int64, shape: (N,)]
│   │
│   └─ energy/
│       ├─ heating_power    [dataset: float64, shape: (N,)]
│       ├─ radiated_power   [dataset: float64, shape: (N,)]
│       ├─ q_factor         [dataset: float64, shape: (N,)]
│       └─ timestamps       [dataset: int64, shape: (N,)]
│
├─ particle/                (particle physics data)
│   ├─ events/
│   │   ├─ event_ids        [dataset: int64, shape: (N,)]
│   │   ├─ energies         [dataset: float64, shape: (N,)]
│   │   ├─ momenta          [dataset: float64, shape: (N, 4)]
│   │   └─ timestamps       [dataset: int64, shape: (N,)]
│   │
│   └─ detector/
│       ├─ hits             [dataset: int32, shape: (N, M)]
│       └─ calibration      [dataset: float64, shape: (M,)]
│
└─ dark_matter/             (dark matter data)
    ├─ events/
    │   ├─ energies         [dataset: float64, shape: (N,)]
    │   ├─ recoil_type      [dataset: int8, shape: (N,)]
    │   └─ timestamps       [dataset: int64, shape: (N,)]
    │
    └─ limits/
        ├─ mass             [dataset: float64, shape: (M,)]
        └─ cross_section    [dataset: float64, shape: (M,)]
```

### 5.4 Implementation Skeleton

```rust
pub struct HDF5IntegrationAdapter {
    state: IntegrationState,
    file_path: Option<String>,
    compression_level: u8,
}

impl HDF5IntegrationAdapter {
    pub fn new() -> Self {
        Self {
            state: IntegrationState::Disconnected,
            file_path: None,
            compression_level: 6, // Default GZIP level
        }
    }

    /// Export fusion data to HDF5
    pub async fn export_fusion(
        &self,
        data: &FusionData,
        group_path: &str,
    ) -> Result<(), IntegrationError> {
        // Create groups
        self.create_group(&format!("{}/plasma", group_path)).await?;
        self.create_group(&format!("{}/magnetics", group_path)).await?;
        self.create_group(&format!("{}/energy", group_path)).await?;

        // Write plasma data
        if let Some(ref plasma) = data.plasma {
            if let Some(ref temp) = plasma.temperature {
                self.write_dataset(
                    &format!("{}/plasma/temperature", group_path),
                    &[temp.value],
                    DatasetOptions::default(),
                ).await?;
            }
        }

        Ok(())
    }
}

#[async_trait]
impl IntegrationAdapter for HDF5IntegrationAdapter {
    fn integration_type(&self) -> IntegrationType {
        IntegrationType::Hdf5
    }

    fn name(&self) -> &str {
        "HDF5 Archive"
    }

    fn state(&self) -> IntegrationState {
        self.state
    }

    async fn initialize(&mut self, options: IntegrationOptions) -> Result<(), IntegrationError> {
        if let Some(endpoint) = options.endpoint {
            self.file_path = Some(endpoint);
        }
        Ok(())
    }

    async fn connect(&mut self) -> Result<(), IntegrationError> {
        self.state = IntegrationState::Connected;
        Ok(())
    }

    async fn disconnect(&mut self) -> Result<(), IntegrationError> {
        self.state = IntegrationState::Disconnected;
        Ok(())
    }

    async fn export(&self, data: &PhysicsData) -> Result<ExportResult, IntegrationError> {
        let path = self.file_path.as_ref()
            .ok_or(IntegrationError::NotConnected)?;

        Ok(ExportResult {
            success: true,
            records_exported: 1,
            destination: path.clone(),
            timestamp: chrono::Utc::now().timestamp_millis(),
            details: Some("Exported to HDF5".to_string()),
        })
    }

    async fn import(&self, source: &str) -> Result<PhysicsData, IntegrationError> {
        // Read HDF5 file and convert to PhysicsData
        Err(IntegrationError::NotSupported("Import not implemented".to_string()))
    }

    fn is_available(&self) -> bool {
        self.state == IntegrationState::Connected && self.file_path.is_some()
    }

    async fn dispose(&mut self) -> Result<(), IntegrationError> {
        self.disconnect().await
    }
}
```

---

## 6. Time-Series Database Integration

### 6.1 Overview

Time-series database integration enables real-time monitoring and historical data analysis.

Supported databases:
- **InfluxDB**: High-performance time-series (recommended)
- **TimescaleDB**: PostgreSQL-based
- **QuestDB**: Ultra-fast ingestion

### 6.2 Interface

```rust
/// Time-series database adapter interface
#[async_trait]
pub trait TimeSeriesAdapter: IntegrationAdapter {
    /// Write single point
    async fn write_point(&self, point: DataPoint) -> Result<(), IntegrationError>;

    /// Write batch of points
    async fn write_batch(&self, points: Vec<DataPoint>) -> Result<(), IntegrationError>;

    /// Query data
    async fn query(&self, query: TimeSeriesQuery) -> Result<QueryResult, IntegrationError>;

    /// Create measurement/table
    async fn create_measurement(&self, name: &str, schema: MeasurementSchema) -> Result<(), IntegrationError>;

    /// List measurements
    async fn list_measurements(&self) -> Result<Vec<String>, IntegrationError>;
}

/// Single data point
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DataPoint {
    /// Measurement name
    pub measurement: String,

    /// Tags (indexed strings)
    pub tags: HashMap<String, String>,

    /// Fields (values)
    pub fields: HashMap<String, FieldValue>,

    /// Timestamp (nanoseconds since epoch)
    pub timestamp: i64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum FieldValue {
    Float(f64),
    Int(i64),
    Bool(bool),
    String(String),
}

/// Query for time-series data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimeSeriesQuery {
    /// Measurement name
    pub measurement: String,

    /// Field names to select
    pub fields: Vec<String>,

    /// Time range
    pub time_range: Option<TimeRange>,

    /// Tag filters
    pub filters: HashMap<String, String>,

    /// Aggregation
    pub aggregation: Option<Aggregation>,

    /// Limit
    pub limit: Option<u64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimeRange {
    pub start: i64,
    pub end: i64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum Aggregation {
    Mean,
    Sum,
    Min,
    Max,
    Count,
    First,
    Last,
}
```

### 6.3 InfluxDB Schema

WIA Physics data maps to InfluxDB using this schema:

```
Measurement: wia_fusion_plasma
Tags:
  - experiment: string (e.g., "ITER")
  - device: string (e.g., "tokamak_01")
  - location: string (e.g., "core", "edge")
Fields:
  - temperature: float
  - temperature_err: float
  - density: float
  - density_err: float
  - confinement_time: float
Timestamp: nanosecond precision

Measurement: wia_fusion_magnetics
Tags:
  - experiment: string
  - coil_type: string (e.g., "TF", "PF")
Fields:
  - field_strength: float
  - current: float
  - stored_energy: float
Timestamp: nanosecond precision

Measurement: wia_particle_event
Tags:
  - experiment: string
  - detector: string
  - event_type: string
Fields:
  - energy: float
  - momentum_x: float
  - momentum_y: float
  - momentum_z: float
  - multiplicity: integer
Timestamp: nanosecond precision
```

### 6.4 Implementation Skeleton

```rust
pub struct InfluxDBAdapter {
    state: IntegrationState,
    endpoint: String,
    database: String,
    token: Option<String>,
}

impl InfluxDBAdapter {
    pub fn new(endpoint: &str, database: &str) -> Self {
        Self {
            state: IntegrationState::Disconnected,
            endpoint: endpoint.to_string(),
            database: database.to_string(),
            token: None,
        }
    }

    /// Convert fusion data to InfluxDB points
    pub fn fusion_to_points(data: &FusionData, tags: HashMap<String, String>) -> Vec<DataPoint> {
        let mut points = Vec::new();
        let timestamp = chrono::Utc::now().timestamp_nanos_opt().unwrap_or(0);

        if let Some(ref plasma) = data.plasma {
            let mut fields = HashMap::new();

            if let Some(ref temp) = plasma.temperature {
                fields.insert("temperature".to_string(), FieldValue::Float(temp.value));
                if let Some(ref err) = temp.uncertainty {
                    fields.insert("temperature_err".to_string(), FieldValue::Float(err.total));
                }
            }

            if let Some(ref density) = plasma.density {
                fields.insert("density".to_string(), FieldValue::Float(density.value));
            }

            if !fields.is_empty() {
                points.push(DataPoint {
                    measurement: "wia_fusion_plasma".to_string(),
                    tags: tags.clone(),
                    fields,
                    timestamp,
                });
            }
        }

        points
    }
}
```

---

## 7. Visualization Integration

### 7.1 Overview

Visualization integration enables real-time dashboards and analysis views.

Supported platforms:
- **Grafana**: Real-time monitoring dashboards
- **Web Dashboards**: Custom React/Vue applications
- **Jupyter**: Interactive analysis

### 7.2 Grafana Interface

```rust
/// Grafana dashboard adapter
#[async_trait]
pub trait GrafanaAdapter: IntegrationAdapter {
    /// Create dashboard
    async fn create_dashboard(
        &self,
        dashboard: Dashboard,
    ) -> Result<DashboardInfo, IntegrationError>;

    /// Update dashboard
    async fn update_dashboard(
        &self,
        uid: &str,
        dashboard: Dashboard,
    ) -> Result<DashboardInfo, IntegrationError>;

    /// Delete dashboard
    async fn delete_dashboard(&self, uid: &str) -> Result<(), IntegrationError>;

    /// Get dashboard
    async fn get_dashboard(&self, uid: &str) -> Result<Dashboard, IntegrationError>;

    /// Create alert rule
    async fn create_alert(&self, alert: AlertRule) -> Result<(), IntegrationError>;
}

/// Dashboard definition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Dashboard {
    pub title: String,
    pub description: Option<String>,
    pub tags: Vec<String>,
    pub panels: Vec<Panel>,
    pub refresh: Option<String>, // e.g., "5s", "1m"
    pub time_range: Option<TimeRange>,
}

/// Dashboard panel
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Panel {
    pub title: String,
    pub panel_type: PanelType,
    pub data_source: String,
    pub query: String,
    pub position: PanelPosition,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum PanelType {
    Graph,
    Gauge,
    Stat,
    Table,
    Heatmap,
    Alert,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PanelPosition {
    pub x: u32,
    pub y: u32,
    pub width: u32,
    pub height: u32,
}

/// Alert rule
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlertRule {
    pub name: String,
    pub condition: AlertCondition,
    pub threshold: f64,
    pub duration: String, // e.g., "5m"
    pub notifications: Vec<String>,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum AlertCondition {
    Above,
    Below,
    OutsideRange,
    NoData,
}
```

### 7.3 Pre-built Dashboard Templates

WIA Physics provides ready-to-use dashboard templates:

**Fusion Dashboard**:
```json
{
  "title": "WIA Fusion Monitoring",
  "panels": [
    {
      "title": "Plasma Temperature",
      "type": "graph",
      "query": "SELECT mean(temperature) FROM wia_fusion_plasma WHERE $timeFilter GROUP BY time(1s)"
    },
    {
      "title": "Q Factor",
      "type": "gauge",
      "query": "SELECT last(q_factor) FROM wia_fusion_energy"
    },
    {
      "title": "Triple Product",
      "type": "stat",
      "query": "SELECT last(triple_product) FROM wia_fusion_plasma"
    },
    {
      "title": "Confinement Time",
      "type": "graph",
      "query": "SELECT mean(confinement_time) FROM wia_fusion_plasma WHERE $timeFilter GROUP BY time(1s)"
    }
  ]
}
```

---

## 8. Integration Manager

### 8.1 Overview

IntegrationManager coordinates multiple adapters and provides a unified interface.

### 8.2 Interface

```rust
/// Integration manager
pub struct IntegrationManager {
    adapters: HashMap<IntegrationType, Box<dyn IntegrationAdapter>>,
    preferences: IntegrationPreferences,
    event_emitter: EventEmitter,
}

/// Integration preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IntegrationPreferences {
    pub enabled_integrations: Vec<IntegrationType>,
    pub primary_archive: IntegrationType,
    pub auto_export: bool,
    pub export_interval_ms: u64,
}

impl IntegrationManager {
    pub fn new() -> Self {
        Self {
            adapters: HashMap::new(),
            preferences: IntegrationPreferences::default(),
            event_emitter: EventEmitter::new(),
        }
    }

    /// Register an adapter
    pub fn register(&mut self, adapter: Box<dyn IntegrationAdapter>) {
        let integration_type = adapter.integration_type();
        self.adapters.insert(integration_type, adapter);
        self.event_emitter.emit("adapter:registered", integration_type);
    }

    /// Unregister an adapter
    pub fn unregister(&mut self, integration_type: IntegrationType) {
        self.adapters.remove(&integration_type);
        self.event_emitter.emit("adapter:unregistered", integration_type);
    }

    /// Get adapter
    pub fn get(&self, integration_type: IntegrationType) -> Option<&dyn IntegrationAdapter> {
        self.adapters.get(&integration_type).map(|a| a.as_ref())
    }

    /// Get mutable adapter
    pub fn get_mut(&mut self, integration_type: IntegrationType) -> Option<&mut Box<dyn IntegrationAdapter>> {
        self.adapters.get_mut(&integration_type)
    }

    /// Initialize all adapters
    pub async fn initialize(&mut self, options: HashMap<IntegrationType, IntegrationOptions>) -> Result<(), IntegrationError> {
        for (integration_type, opts) in options {
            if let Some(adapter) = self.adapters.get_mut(&integration_type) {
                adapter.initialize(opts).await?;
            }
        }
        Ok(())
    }

    /// Export to all enabled integrations
    pub async fn export_all(&self, data: &PhysicsData) -> Vec<ExportResult> {
        let mut results = Vec::new();

        for integration_type in &self.preferences.enabled_integrations {
            if let Some(adapter) = self.adapters.get(integration_type) {
                if adapter.is_available() {
                    match adapter.export(data).await {
                        Ok(result) => results.push(result),
                        Err(e) => {
                            self.event_emitter.emit("export:error", e);
                        }
                    }
                }
            }
        }

        results
    }

    /// Export to specific integration
    pub async fn export_to(
        &self,
        integration_type: IntegrationType,
        data: &PhysicsData,
    ) -> Result<ExportResult, IntegrationError> {
        let adapter = self.adapters.get(&integration_type)
            .ok_or(IntegrationError::AdapterNotFound(integration_type))?;

        if !adapter.is_available() {
            return Err(IntegrationError::AdapterNotAvailable(integration_type));
        }

        adapter.export(data).await
    }

    /// Dispose all adapters
    pub async fn dispose(&mut self) -> Result<(), IntegrationError> {
        for adapter in self.adapters.values_mut() {
            adapter.dispose().await?;
        }
        self.adapters.clear();
        Ok(())
    }
}
```

---

## 9. Events and Callbacks

### 9.1 Event Types

```rust
/// Integration event types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IntegrationEvent {
    /// Adapter registered
    AdapterRegistered,
    /// Adapter unregistered
    AdapterUnregistered,
    /// Connection established
    Connected,
    /// Connection lost
    Disconnected,
    /// Export started
    ExportStarted,
    /// Export completed
    ExportCompleted,
    /// Export failed
    ExportFailed,
    /// Import started
    ImportStarted,
    /// Import completed
    ImportCompleted,
    /// Import failed
    ImportFailed,
}

/// Event data
#[derive(Debug, Clone)]
pub struct IntegrationEventData {
    pub event: IntegrationEvent,
    pub integration_type: IntegrationType,
    pub timestamp: i64,
    pub details: Option<String>,
    pub error: Option<IntegrationError>,
}

/// Event handler type
pub type IntegrationEventHandler = Box<dyn Fn(IntegrationEventData) + Send + Sync>;
```

### 9.2 Event Usage

```rust
let mut manager = IntegrationManager::new();

// Register event handlers
manager.on(IntegrationEvent::ExportCompleted, |event| {
    println!("Export completed to {:?}", event.integration_type);
});

manager.on(IntegrationEvent::ExportFailed, |event| {
    eprintln!("Export failed: {:?}", event.error);
});

// Export data
manager.export_all(&physics_data).await;
```

---

## 10. Error Handling

### 10.1 Error Types

```rust
/// Integration error
#[derive(Debug, Clone)]
pub enum IntegrationError {
    /// Adapter not found
    AdapterNotFound(IntegrationType),

    /// Adapter not available
    AdapterNotAvailable(IntegrationType),

    /// Not connected
    NotConnected,

    /// Connection failed
    ConnectionFailed(String),

    /// Export failed
    ExportFailed(String),

    /// Import failed
    ImportFailed(String),

    /// Not supported
    NotSupported(String),

    /// Invalid configuration
    InvalidConfig(String),

    /// IO error
    IoError(String),

    /// Timeout
    Timeout,

    /// Authentication failed
    AuthFailed,
}

impl std::fmt::Display for IntegrationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::AdapterNotFound(t) => write!(f, "Adapter not found: {:?}", t),
            Self::AdapterNotAvailable(t) => write!(f, "Adapter not available: {:?}", t),
            Self::NotConnected => write!(f, "Not connected"),
            Self::ConnectionFailed(s) => write!(f, "Connection failed: {}", s),
            Self::ExportFailed(s) => write!(f, "Export failed: {}", s),
            Self::ImportFailed(s) => write!(f, "Import failed: {}", s),
            Self::NotSupported(s) => write!(f, "Not supported: {}", s),
            Self::InvalidConfig(s) => write!(f, "Invalid configuration: {}", s),
            Self::IoError(s) => write!(f, "IO error: {}", s),
            Self::Timeout => write!(f, "Operation timed out"),
            Self::AuthFailed => write!(f, "Authentication failed"),
        }
    }
}

impl std::error::Error for IntegrationError {}
```

### 10.2 Error Recovery

```rust
/// Error recovery strategy
pub struct ErrorRecoveryStrategy {
    pub max_retries: u32,
    pub retry_delay_ms: u64,
    pub fallback_adapters: HashMap<IntegrationType, IntegrationType>,
}

impl ErrorRecoveryStrategy {
    pub fn can_retry(&self, error: &IntegrationError) -> bool {
        matches!(
            error,
            IntegrationError::ConnectionFailed(_) |
            IntegrationError::Timeout |
            IntegrationError::IoError(_)
        )
    }

    pub fn get_fallback(&self, integration_type: IntegrationType) -> Option<IntegrationType> {
        self.fallback_adapters.get(&integration_type).copied()
    }
}
```

---

## 11. Examples

### 11.1 Full Integration Pipeline

```rust
use wia_physics::prelude::*;
use wia_physics::integration::*;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 1. Create fusion data (Phase 1-2)
    let fusion_data = FusionDataBuilder::new()
        .experiment("ITER")
        .plasma_simple(150e6, "K", 1e20, "m^-3")
        .tokamak(5.3, 6.2, 2.0)
        .build()?;

    // 2. Create integration manager
    let mut manager = IntegrationManager::new();

    // 3. Register adapters
    manager.register(Box::new(HDF5IntegrationAdapter::new()));
    manager.register(Box::new(InfluxDBAdapter::new(
        "http://localhost:8086",
        "wia_physics"
    )));
    manager.register(Box::new(EPICSIntegrationAdapter::new()));

    // 4. Initialize adapters
    let mut options = HashMap::new();
    options.insert(IntegrationType::Hdf5, IntegrationOptions {
        endpoint: Some("/data/fusion_archive.h5".to_string()),
        ..Default::default()
    });
    options.insert(IntegrationType::InfluxDb, IntegrationOptions {
        endpoint: Some("http://localhost:8086".to_string()),
        credentials: Some(Credentials {
            token: Some("my-token".to_string()),
            ..Default::default()
        }),
        ..Default::default()
    });

    manager.initialize(options).await?;

    // 5. Connect all adapters
    for adapter in manager.adapters.values_mut() {
        adapter.connect().await?;
    }

    // 6. Export to all integrations
    let physics_data = PhysicsData {
        data_type: PhysicsDataType::Fusion,
        metadata: DataMetadata {
            id: "fusion-2025-001".to_string(),
            created: chrono::Utc::now(),
            ..Default::default()
        },
        payload: serde_json::to_value(&fusion_data)?,
    };

    let results = manager.export_all(&physics_data).await;

    for result in results {
        println!(
            "Exported {} records to {}",
            result.records_exported,
            result.destination
        );
    }

    // 7. Cleanup
    manager.dispose().await?;

    Ok(())
}
```

### 11.2 HDF5 Archive Example

```rust
use wia_physics::integration::*;

async fn archive_fusion_run(
    data: Vec<FusionData>,
    filename: &str,
) -> Result<(), IntegrationError> {
    let mut hdf5 = HDF5IntegrationAdapter::new();

    // Create file
    hdf5.create_file(filename).await?;

    // Write metadata
    hdf5.write_attribute("/", "experiment", AttributeValue::String("ITER".to_string())).await?;
    hdf5.write_attribute("/", "created", AttributeValue::String(chrono::Utc::now().to_rfc3339())).await?;
    hdf5.write_attribute("/", "wia_version", AttributeValue::String("1.0".to_string())).await?;

    // Create groups
    hdf5.create_group("/fusion").await?;
    hdf5.create_group("/fusion/plasma").await?;

    // Collect all temperatures
    let temperatures: Vec<f64> = data.iter()
        .filter_map(|d| d.plasma.as_ref()?.temperature.as_ref())
        .map(|t| t.value)
        .collect();

    // Write dataset
    hdf5.write_dataset(
        "/fusion/plasma/temperature",
        &temperatures,
        DatasetOptions {
            compression: 6,
            shuffle: true,
            ..Default::default()
        },
    ).await?;

    // Close file
    hdf5.close_file().await?;

    println!("Archived {} records to {}", data.len(), filename);
    Ok(())
}
```

### 11.3 Real-time Monitoring Example

```rust
use wia_physics::integration::*;
use wia_physics::protocol::*;
use std::collections::HashMap;

async fn setup_monitoring(
    influx_endpoint: &str,
    grafana_endpoint: &str,
) -> Result<IntegrationManager, IntegrationError> {
    let mut manager = IntegrationManager::new();

    // Setup InfluxDB
    let mut influx = InfluxDBAdapter::new(influx_endpoint, "wia_physics");
    influx.initialize(IntegrationOptions::default()).await?;
    influx.connect().await?;
    manager.register(Box::new(influx));

    // Setup Grafana
    let mut grafana = GrafanaAdapter::new(grafana_endpoint);
    grafana.initialize(IntegrationOptions::default()).await?;
    grafana.connect().await?;

    // Create dashboard
    let dashboard = Dashboard {
        title: "WIA Fusion Monitoring".to_string(),
        description: Some("Real-time fusion plasma monitoring".to_string()),
        tags: vec!["fusion".to_string(), "wia".to_string()],
        panels: vec![
            Panel {
                title: "Plasma Temperature".to_string(),
                panel_type: PanelType::Graph,
                data_source: "InfluxDB".to_string(),
                query: "SELECT mean(temperature) FROM wia_fusion_plasma WHERE $timeFilter GROUP BY time(1s)".to_string(),
                position: PanelPosition { x: 0, y: 0, width: 12, height: 8 },
            },
            Panel {
                title: "Q Factor".to_string(),
                panel_type: PanelType::Gauge,
                data_source: "InfluxDB".to_string(),
                query: "SELECT last(q_factor) FROM wia_fusion_energy".to_string(),
                position: PanelPosition { x: 12, y: 0, width: 6, height: 8 },
            },
        ],
        refresh: Some("5s".to_string()),
        time_range: None,
    };

    grafana.create_dashboard(dashboard).await?;
    manager.register(Box::new(grafana));

    Ok(manager)
}

// Stream data to monitoring
async fn stream_to_monitoring(
    manager: &IntegrationManager,
    data: &FusionData,
) -> Result<(), IntegrationError> {
    let mut tags = HashMap::new();
    tags.insert("experiment".to_string(), "ITER".to_string());

    let points = InfluxDBAdapter::fusion_to_points(data, tags);

    if let Some(adapter) = manager.get(IntegrationType::InfluxDb) {
        if let Some(ts_adapter) = adapter.as_any().downcast_ref::<InfluxDBAdapter>() {
            ts_adapter.write_batch(points).await?;
        }
    }

    Ok(())
}
```

---

## 12. References

### Control Systems
1. EPICS Documentation: https://epics.anl.gov/
2. TANGO Controls: https://www.tango-controls.org/
3. OPC UA Foundation: https://opcfoundation.org/

### Data Formats
4. HDF5 Documentation: https://www.hdfgroup.org/
5. ROOT: https://root.cern/
6. Apache Parquet: https://parquet.apache.org/

### Time-Series Databases
7. InfluxDB: https://www.influxdata.com/
8. TimescaleDB: https://www.timescale.com/
9. QuestDB: https://questdb.io/

### Visualization
10. Grafana: https://grafana.com/
11. Three.js: https://threejs.org/

### External Services
12. CERN Open Data: https://opendata.cern.ch/
13. EOSC Portal: https://eosc-portal.eu/

---

<div align="center">

**WIA Physics Standard - Phase 4**

Ecosystem Integration Specification

Version 1.0.0 | 2025-12-14

© 2025 SmileStory Inc. / WIA

**弘益人間** - Benefit All Humanity

</div>
