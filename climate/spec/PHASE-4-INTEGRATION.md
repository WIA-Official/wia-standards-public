# WIA Climate - Phase 4: Ecosystem Integration Specification

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2024-12-14

## 1. Overview

Phase 4 defines the ecosystem integration layer for WIA Climate, enabling seamless connection with visualization dashboards, time-series databases, alert systems, and scientific data export formats.

### 1.1 Goals

- **Visualization**: Real-time dashboards and 3D geospatial displays
- **Storage**: Efficient time-series data persistence with compression
- **Alerting**: Threshold-based notifications with multiple delivery channels
- **Export**: Scientific data formats for research and analysis

### 1.2 Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      WIA Climate Core                           │
│  ┌─────────────────┐  ┌─────────────────┐  ┌────────────────┐  │
│  │ ClimateMessage  │  │   Validation    │  │   Protocol     │  │
│  │    (Phase 1)    │──│   (Phase 2)     │──│   (Phase 3)    │  │
│  └─────────────────┘  └─────────────────┘  └────────────────┘  │
└────────────────────────────────┬────────────────────────────────┘
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Output Manager (Phase 4)                      │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │                   Message Buffer                         │    │
│  │  [msg1] [msg2] [msg3] ... [msgN]                        │    │
│  └─────────────────────────────────────────────────────────┘    │
│                            │                                     │
│          ┌─────────────────┼─────────────────┐                  │
│          ▼                 ▼                 ▼                  │
│  ┌───────────────┐ ┌───────────────┐ ┌───────────────┐         │
│  │   Dashboard   │ │    Storage    │ │     Alert     │         │
│  │   Adapters    │ │   Adapters    │ │   Adapters    │         │
│  ├───────────────┤ ├───────────────┤ ├───────────────┤         │
│  │ • Grafana     │ │ • InfluxDB    │ │ • Webhook     │         │
│  │ • Cesium      │ │ • TimescaleDB │ │ • Slack       │         │
│  │ • Custom      │ │ • NetCDF      │ │ • PagerDuty   │         │
│  └───────────────┘ └───────────────┘ └───────────────┘         │
└─────────────────────────────────────────────────────────────────┘
```

## 2. Output Adapter Interface

### 2.1 Core Trait Definition

```rust
/// Output adapter trait for processing climate messages
#[async_trait]
pub trait OutputAdapter: Send + Sync {
    /// Returns the adapter's unique name
    fn name(&self) -> &str;

    /// Returns the adapter type category
    fn adapter_type(&self) -> AdapterType;

    /// Initialize the adapter with configuration
    async fn init(&mut self) -> Result<()>;

    /// Process a single climate message
    async fn process(&self, message: &ClimateMessage) -> Result<()>;

    /// Process a batch of messages (default: iterate and call process)
    async fn process_batch(&self, messages: &[ClimateMessage]) -> Result<BatchResult> {
        let mut success = 0;
        let mut failed = 0;
        for msg in messages {
            match self.process(msg).await {
                Ok(_) => success += 1,
                Err(_) => failed += 1,
            }
        }
        Ok(BatchResult { success, failed })
    }

    /// Flush any buffered data
    async fn flush(&self) -> Result<()>;

    /// Gracefully close the adapter
    async fn close(&mut self) -> Result<()>;

    /// Check adapter health status
    async fn health_check(&self) -> Result<AdapterHealth>;
}
```

### 2.2 Adapter Types

```rust
/// Category of output adapter
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdapterType {
    /// Visualization dashboard (Grafana, Cesium)
    Dashboard,
    /// Time-series storage (InfluxDB, TimescaleDB)
    Storage,
    /// Alert/notification (Webhook, Slack)
    Alert,
    /// Data export (NetCDF, Parquet)
    Export,
    /// Custom adapter
    Custom,
}
```

### 2.3 Health Status

```rust
/// Adapter health information
#[derive(Debug, Clone)]
pub struct AdapterHealth {
    /// Current health status
    pub status: HealthStatus,
    /// Response latency in milliseconds
    pub latency_ms: Option<u64>,
    /// Last successful operation timestamp
    pub last_success: Option<DateTime<Utc>>,
    /// Last error message
    pub last_error: Option<String>,
    /// Error count since startup
    pub error_count: u64,
    /// Messages processed since startup
    pub messages_processed: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HealthStatus {
    /// Adapter is functioning normally
    Healthy,
    /// Adapter is experiencing issues but operational
    Degraded,
    /// Adapter is not functioning
    Unhealthy,
}
```

## 3. Dashboard Adapters

### 3.1 Grafana Adapter

The Grafana adapter pushes metrics to Grafana Cloud or a Grafana instance via the Push Gateway or Live streaming API.

**Configuration:**
```rust
pub struct GrafanaConfig {
    /// Grafana server URL
    pub url: String,
    /// API key for authentication
    pub api_key: String,
    /// Organization ID (for Grafana Cloud)
    pub org_id: Option<String>,
    /// Dashboard UID for annotations
    pub dashboard_uid: Option<String>,
    /// Enable live streaming
    pub live_streaming: bool,
    /// Batch size for metric push
    pub batch_size: usize,
}
```

**Data Mapping:**
| WIA Climate Field | Grafana Metric |
|-------------------|----------------|
| `timestamp` | Metric timestamp |
| `device.device_id` | Label: `device_id` |
| `location.latitude` | Label: `lat` |
| `location.longitude` | Label: `lon` |
| `data_type` | Metric prefix |
| Data values | Metric values |

**Example Output:**
```
# HELP wia_climate_carbon_capture_rate CO2 capture rate in kg/hour
# TYPE wia_climate_carbon_capture_rate gauge
wia_climate_carbon_capture_rate{device_id="ORCA-001",lat="64.0",lon="-21.0",technology="dac"} 125.5
```

### 3.2 Cesium Adapter

The Cesium adapter exports data in CZML format for 3D visualization.

**Configuration:**
```rust
pub struct CesiumConfig {
    /// Output directory for CZML files
    pub output_dir: PathBuf,
    /// WebSocket endpoint for live updates
    pub websocket_url: Option<String>,
    /// Update interval for entity positions
    pub update_interval: Duration,
    /// Entity styling configuration
    pub styles: CesiumStyles,
}

pub struct CesiumStyles {
    /// Point size in pixels
    pub point_size: f32,
    /// Color mapping by data type
    pub color_map: HashMap<DataType, [u8; 4]>,
    /// Enable trails for moving entities
    pub show_trails: bool,
}
```

**CZML Entity Structure:**
```json
{
  "id": "device-{device_id}",
  "name": "{device_manufacturer} {device_model}",
  "position": {
    "cartographicDegrees": [longitude, latitude, altitude]
  },
  "point": {
    "color": {"rgba": [r, g, b, a]},
    "pixelSize": 10,
    "outlineColor": {"rgba": [255, 255, 255, 255]},
    "outlineWidth": 2
  },
  "properties": {
    "data_type": "carbon_capture",
    "capture_rate_kg_per_hour": 125.5,
    "timestamp": "2024-12-14T10:00:00Z"
  }
}
```

## 4. Storage Adapters

### 4.1 InfluxDB Adapter

**Configuration:**
```rust
pub struct InfluxDBConfig {
    /// InfluxDB server URL
    pub url: String,
    /// Organization name
    pub org: String,
    /// Target bucket
    pub bucket: String,
    /// Authentication token
    pub token: String,
    /// Batch size for writes
    pub batch_size: usize,
    /// Flush interval
    pub flush_interval: Duration,
    /// Enable gzip compression
    pub gzip: bool,
}
```

**Line Protocol Format:**
```
{measurement},{tags} {fields} {timestamp}

wia_climate_carbon_capture,device_id=ORCA-001,technology=dac capture_rate_kg_per_hour=125.5,co2_purity_percentage=99.2,energy_consumption_kwh=2500.0 1702551600000000000
```

**Tag Strategy:**
- Low cardinality fields → Tags (device_id, technology, crop_type)
- High cardinality fields → Fields (numeric measurements)
- Avoid tags with unbounded values

### 4.2 TimescaleDB Adapter

**Configuration:**
```rust
pub struct TimescaleDBConfig {
    /// PostgreSQL connection string
    pub connection_string: String,
    /// Connection pool size
    pub pool_size: usize,
    /// Table name for climate data
    pub table_name: String,
    /// Enable automatic compression
    pub compression_enabled: bool,
    /// Compression delay (how old data must be)
    pub compression_after: Duration,
}
```

**Schema:**
```sql
CREATE TABLE wia_climate_data (
    time TIMESTAMPTZ NOT NULL,
    message_id UUID NOT NULL,
    device_id TEXT NOT NULL,
    data_type TEXT NOT NULL,
    location GEOGRAPHY(POINT, 4326) NOT NULL,
    device_info JSONB NOT NULL,
    measurements JSONB NOT NULL,
    metadata JSONB,
    PRIMARY KEY (time, message_id)
);

-- Convert to hypertable
SELECT create_hypertable('wia_climate_data', 'time');

-- Create indexes
CREATE INDEX idx_device_id ON wia_climate_data (device_id, time DESC);
CREATE INDEX idx_data_type ON wia_climate_data (data_type, time DESC);
CREATE INDEX idx_location ON wia_climate_data USING GIST (location);
```

## 5. Alert Adapters

### 5.1 Alert Rule Engine

**Rule Definition:**
```rust
pub struct AlertRule {
    /// Unique rule identifier
    pub id: String,
    /// Human-readable name
    pub name: String,
    /// Rule description
    pub description: Option<String>,
    /// Data type filter
    pub data_type: Option<DataType>,
    /// Device filter (glob pattern)
    pub device_filter: Option<String>,
    /// Alert condition
    pub condition: AlertCondition,
    /// How long condition must be true
    pub duration: Duration,
    /// Alert severity
    pub severity: AlertSeverity,
    /// Notification targets
    pub notifications: Vec<NotificationTarget>,
    /// Is rule enabled
    pub enabled: bool,
}

pub struct AlertCondition {
    /// Field to evaluate
    pub field: String,
    /// Comparison operator
    pub operator: ComparisonOperator,
    /// Threshold value
    pub threshold: f64,
}

#[derive(Debug, Clone, Copy)]
pub enum ComparisonOperator {
    LessThan,
    LessOrEqual,
    GreaterThan,
    GreaterOrEqual,
    Equal,
    NotEqual,
}

#[derive(Debug, Clone, Copy)]
pub enum AlertSeverity {
    Info,
    Warning,
    Critical,
}
```

### 5.2 Webhook Adapter

**Configuration:**
```rust
pub struct WebhookConfig {
    /// Webhook endpoint URL
    pub url: String,
    /// HMAC secret for signature
    pub secret: Option<String>,
    /// Custom headers
    pub headers: HashMap<String, String>,
    /// Request timeout
    pub timeout: Duration,
    /// Maximum retry attempts
    pub max_retries: u32,
    /// Retry backoff configuration
    pub backoff: BackoffConfig,
}

pub struct BackoffConfig {
    /// Initial delay
    pub initial: Duration,
    /// Maximum delay
    pub max: Duration,
    /// Multiplier for exponential backoff
    pub multiplier: f64,
}
```

**Webhook Payload:**
```json
{
  "event_id": "evt_abc123",
  "event_type": "alert.firing",
  "timestamp": "2024-12-14T10:30:00Z",
  "alert": {
    "rule_id": "capture-rate-low",
    "rule_name": "Low Capture Rate Alert",
    "severity": "warning",
    "state": "firing",
    "condition": {
      "field": "capture_rate_kg_per_hour",
      "operator": "lt",
      "threshold": 100,
      "current_value": 85.5
    },
    "duration_seconds": 300,
    "fired_at": "2024-12-14T10:25:00Z"
  },
  "source": {
    "device_id": "ORCA-001",
    "device_manufacturer": "Climeworks",
    "device_model": "Orca DAC",
    "location": {
      "latitude": 64.0,
      "longitude": -21.0,
      "altitude": 100.0
    },
    "data_type": "carbon_capture"
  },
  "context": {
    "recent_values": [125.5, 110.2, 95.3, 88.1, 85.5],
    "average_1h": 100.92,
    "trend": "decreasing"
  }
}
```

**Signature Header:**
```
X-WIA-Signature: sha256=<HMAC-SHA256(secret, payload)>
X-WIA-Timestamp: 1702551600
X-WIA-Event-ID: evt_abc123
```

### 5.3 Alert State Management

```rust
pub struct AlertState {
    /// Alert rule ID
    pub rule_id: String,
    /// Current state
    pub state: AlertStateValue,
    /// When condition was first met
    pub pending_since: Option<DateTime<Utc>>,
    /// When alert started firing
    pub firing_since: Option<DateTime<Utc>>,
    /// When alert was resolved
    pub resolved_at: Option<DateTime<Utc>>,
    /// Last evaluated value
    pub last_value: f64,
    /// Notification history
    pub notifications_sent: Vec<NotificationRecord>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlertStateValue {
    /// Condition not met
    Inactive,
    /// Condition met, waiting for duration
    Pending,
    /// Alert is active
    Firing,
    /// Alert was active but is now resolved
    Resolved,
}
```

## 6. Export Adapters

### 6.1 NetCDF Exporter

**Configuration:**
```rust
pub struct NetCDFConfig {
    /// Output directory
    pub output_dir: PathBuf,
    /// File naming pattern
    pub file_pattern: String,  // e.g., "wia_climate_{date}_{data_type}.nc"
    /// Chunk size for unlimited dimensions
    pub chunk_size: usize,
    /// Compression level (0-9)
    pub compression_level: u8,
    /// CF Convention version
    pub cf_version: String,
}
```

**NetCDF Structure:**
```
dimensions:
    time = UNLIMITED ;
    location = <n_locations> ;
    str_len = 64 ;

variables:
    double time(time) ;
        time:standard_name = "time" ;
        time:units = "seconds since 1970-01-01 00:00:00 UTC" ;
        time:calendar = "standard" ;
        time:axis = "T" ;

    double latitude(location) ;
        latitude:standard_name = "latitude" ;
        latitude:units = "degrees_north" ;
        latitude:axis = "Y" ;

    double longitude(location) ;
        longitude:standard_name = "longitude" ;
        longitude:units = "degrees_east" ;
        longitude:axis = "X" ;

    char device_id(location, str_len) ;
        device_id:long_name = "Device Identifier" ;

    float capture_rate(time, location) ;
        capture_rate:long_name = "CO2 Capture Rate" ;
        capture_rate:units = "kg hour-1" ;
        capture_rate:_FillValue = -9999.0f ;
        capture_rate:coordinates = "latitude longitude" ;

// global attributes:
    :Conventions = "CF-1.8" ;
    :title = "WIA Climate Network Data" ;
    :institution = "WIA Climate" ;
    :source = "WIA Climate API v1.0" ;
    :references = "https://wia.network/climate" ;
    :history = "Created {timestamp}" ;
```

### 6.2 Parquet Exporter

**Configuration:**
```rust
pub struct ParquetConfig {
    /// Output directory
    pub output_dir: PathBuf,
    /// File naming pattern
    pub file_pattern: String,
    /// Row group size
    pub row_group_size: usize,
    /// Compression codec
    pub compression: ParquetCompression,
    /// Partition columns
    pub partition_by: Vec<String>,
}

#[derive(Debug, Clone)]
pub enum ParquetCompression {
    None,
    Snappy,
    Gzip { level: u8 },
    Zstd { level: i32 },
    Lz4,
}
```

**Schema:**
```
message wia_climate_data {
  required int64 timestamp (TIMESTAMP(MILLIS, true));
  required binary message_id (STRING);
  required binary device_id (STRING);
  required binary data_type (STRING);
  required double latitude;
  required double longitude;
  optional double altitude;
  required binary device_info (JSON);
  required binary measurements (JSON);
  optional binary metadata (JSON);
}
```

## 7. Output Manager

### 7.1 Manager Configuration

```rust
pub struct OutputConfig {
    /// Processing strategy
    pub strategy: ProcessStrategy,
    /// Message buffer size
    pub buffer_size: usize,
    /// Auto-flush interval
    pub flush_interval: Duration,
    /// Enable circuit breaker
    pub circuit_breaker: bool,
    /// Circuit breaker threshold (failures before open)
    pub circuit_breaker_threshold: u32,
    /// Circuit breaker reset timeout
    pub circuit_breaker_timeout: Duration,
}

#[derive(Debug, Clone)]
pub enum ProcessStrategy {
    /// Process through all adapters in parallel
    Fanout,
    /// Process through adapters sequentially
    Pipeline,
    /// Process only through selected adapters
    Selective(Vec<String>),
}
```

### 7.2 Manager Interface

```rust
pub struct OutputManager {
    adapters: Vec<Box<dyn OutputAdapter>>,
    config: OutputConfig,
    buffer: MessageBuffer,
    circuit_breakers: HashMap<String, CircuitBreaker>,
    metrics: OutputMetrics,
}

impl OutputManager {
    /// Create a new output manager
    pub fn new(config: OutputConfig) -> Self;

    /// Add an adapter to the manager
    pub fn add_adapter(&mut self, adapter: impl OutputAdapter + 'static);

    /// Remove an adapter by name
    pub fn remove_adapter(&mut self, name: &str) -> Option<Box<dyn OutputAdapter>>;

    /// Get adapter by name
    pub fn get_adapter(&self, name: &str) -> Option<&dyn OutputAdapter>;

    /// Process a single message
    pub async fn process(&self, message: &ClimateMessage) -> Result<ProcessResult>;

    /// Process a batch of messages
    pub async fn process_batch(&self, messages: &[ClimateMessage]) -> Result<BatchProcessResult>;

    /// Flush all adapters
    pub async fn flush_all(&self) -> Result<()>;

    /// Get health status of all adapters
    pub async fn health(&self) -> HashMap<String, AdapterHealth>;

    /// Shutdown all adapters gracefully
    pub async fn shutdown(&mut self) -> Result<()>;
}
```

### 7.3 Circuit Breaker

```rust
pub struct CircuitBreaker {
    /// Current state
    state: CircuitState,
    /// Failure count
    failure_count: u32,
    /// Last failure time
    last_failure: Option<Instant>,
    /// Configuration
    config: CircuitBreakerConfig,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CircuitState {
    /// Normal operation
    Closed,
    /// Allowing limited traffic to test recovery
    HalfOpen,
    /// Rejecting all traffic
    Open,
}

impl CircuitBreaker {
    /// Check if request should be allowed
    pub fn allow_request(&mut self) -> bool;

    /// Record success
    pub fn record_success(&mut self);

    /// Record failure
    pub fn record_failure(&mut self);
}
```

## 8. Error Handling

### 8.1 Adapter Errors

```rust
#[derive(Error, Debug)]
pub enum AdapterError {
    #[error("Connection failed: {0}")]
    Connection(String),

    #[error("Authentication failed: {0}")]
    Authentication(String),

    #[error("Write failed: {0}")]
    Write(String),

    #[error("Timeout after {0:?}")]
    Timeout(Duration),

    #[error("Rate limited: retry after {0:?}")]
    RateLimited(Duration),

    #[error("Circuit breaker open")]
    CircuitOpen,

    #[error("Invalid configuration: {0}")]
    Configuration(String),

    #[error("Serialization error: {0}")]
    Serialization(String),
}
```

### 8.2 Retry Strategy

```rust
pub struct RetryStrategy {
    /// Maximum number of retries
    pub max_retries: u32,
    /// Initial backoff duration
    pub initial_backoff: Duration,
    /// Maximum backoff duration
    pub max_backoff: Duration,
    /// Backoff multiplier
    pub multiplier: f64,
    /// Add jitter to prevent thundering herd
    pub jitter: bool,
}

impl RetryStrategy {
    /// Calculate delay for given attempt
    pub fn delay_for_attempt(&self, attempt: u32) -> Duration {
        let base = self.initial_backoff.as_millis() as f64
            * self.multiplier.powi(attempt as i32);
        let delay = Duration::from_millis(base.min(self.max_backoff.as_millis() as f64) as u64);

        if self.jitter {
            // Add up to 25% random jitter
            let jitter = rand::random::<f64>() * 0.25;
            delay + Duration::from_millis((delay.as_millis() as f64 * jitter) as u64)
        } else {
            delay
        }
    }
}
```

## 9. Metrics & Observability

### 9.1 Output Metrics

```rust
pub struct OutputMetrics {
    /// Messages processed per adapter
    pub messages_processed: HashMap<String, Counter>,
    /// Processing latency per adapter
    pub processing_latency: HashMap<String, Histogram>,
    /// Errors per adapter
    pub errors: HashMap<String, Counter>,
    /// Buffer size
    pub buffer_size: Gauge,
    /// Circuit breaker state
    pub circuit_state: HashMap<String, Gauge>,
}
```

### 9.2 Prometheus Metrics

```
# HELP wia_climate_output_messages_total Total messages processed
# TYPE wia_climate_output_messages_total counter
wia_climate_output_messages_total{adapter="influxdb",status="success"} 12345
wia_climate_output_messages_total{adapter="influxdb",status="error"} 23

# HELP wia_climate_output_latency_seconds Processing latency
# TYPE wia_climate_output_latency_seconds histogram
wia_climate_output_latency_seconds_bucket{adapter="influxdb",le="0.01"} 10000
wia_climate_output_latency_seconds_bucket{adapter="influxdb",le="0.05"} 12000
wia_climate_output_latency_seconds_bucket{adapter="influxdb",le="0.1"} 12300

# HELP wia_climate_output_circuit_state Circuit breaker state (0=closed, 1=half-open, 2=open)
# TYPE wia_climate_output_circuit_state gauge
wia_climate_output_circuit_state{adapter="influxdb"} 0
```

## 10. Configuration File Format

### 10.1 TOML Configuration

```toml
[output]
strategy = "fanout"
buffer_size = 1000
flush_interval_ms = 5000
circuit_breaker = true
circuit_breaker_threshold = 5
circuit_breaker_timeout_ms = 30000

[[output.adapters]]
name = "main-influxdb"
type = "influxdb"
enabled = true

[output.adapters.config]
url = "http://localhost:8086"
org = "wia-climate"
bucket = "climate-data"
token = "${INFLUXDB_TOKEN}"
batch_size = 100
flush_interval_ms = 1000
gzip = true

[[output.adapters]]
name = "alert-webhook"
type = "webhook"
enabled = true

[output.adapters.config]
url = "https://hooks.example.com/wia-alerts"
secret = "${WEBHOOK_SECRET}"
timeout_ms = 5000
max_retries = 3

[output.adapters.config.headers]
Authorization = "Bearer ${WEBHOOK_TOKEN}"

[[output.adapters]]
name = "cesium-export"
type = "cesium"
enabled = true

[output.adapters.config]
output_dir = "/var/lib/wia-climate/czml"
websocket_url = "ws://localhost:8765/cesium"
update_interval_ms = 1000

[output.adapters.config.styles]
point_size = 10
show_trails = true

[alerts]
evaluation_interval_ms = 10000

[[alerts.rules]]
id = "capture-rate-low"
name = "Low Carbon Capture Rate"
description = "Alert when capture rate falls below threshold"
data_type = "carbon_capture"
enabled = true

[alerts.rules.condition]
field = "capture_rate_kg_per_hour"
operator = "lt"
threshold = 100.0

[alerts.rules]
duration_seconds = 300
severity = "warning"
notifications = ["alert-webhook"]

[[alerts.rules]]
id = "energy-spike"
name = "Renewable Energy Spike"
data_type = "renewable_energy"
enabled = true

[alerts.rules.condition]
field = "current_output_kw"
operator = "gt"
threshold = 10000.0

[alerts.rules]
duration_seconds = 60
severity = "info"
notifications = ["alert-webhook"]
```

## 11. Security Considerations

### 11.1 Credential Management

- Store credentials in environment variables or secret managers
- Never log or expose credentials in error messages
- Rotate credentials regularly
- Use short-lived tokens where possible

### 11.2 Network Security

- TLS 1.3 required for all external connections
- Certificate validation enabled by default
- Support for client certificates (mTLS)
- IP allowlisting for webhooks

### 11.3 Webhook Security

- HMAC-SHA256 signature verification
- Timestamp validation (reject if > 5 minutes old)
- Idempotency keys for at-least-once delivery
- Rate limiting on webhook endpoints

## 12. References

- [InfluxDB Line Protocol](https://docs.influxdata.com/influxdb/latest/reference/syntax/line-protocol/)
- [TimescaleDB Documentation](https://docs.timescale.com/)
- [Grafana Data Source Development](https://grafana.com/docs/grafana/latest/developers/plugins/)
- [CesiumJS CZML Guide](https://cesium.com/learn/cesiumjs/ref-doc/CzmlDataSource.html)
- [NetCDF CF Conventions](https://cfconventions.org/)
- [Prometheus Metrics Best Practices](https://prometheus.io/docs/practices/naming/)

---

弘益人間 - Benefit All Humanity 🌍
