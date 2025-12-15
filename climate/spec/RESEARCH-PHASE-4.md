# WIA Climate - Phase 4 Integration Research

## Overview

This document summarizes research on ecosystem integration technologies for the WIA Climate standard. Phase 4 focuses on connecting climate data with visualization dashboards, time-series storage systems, and alert/notification infrastructure.

## 1. Dashboard & Visualization Adapters

### 1.1 Grafana Data Source Plugin

**Architecture:**
- Grafana uses a plugin architecture for custom data sources
- Plugins can be written in Go (backend) or TypeScript (frontend-only)
- Backend plugins support alerting, query caching, and server-side processing

**Data Source Plugin Components:**
```
grafana-wia-climate-datasource/
├── src/
│   ├── plugin.json         # Plugin metadata
│   ├── datasource.ts       # Data source implementation
│   ├── ConfigEditor.tsx    # Configuration UI
│   ├── QueryEditor.tsx     # Query builder UI
│   └── types.ts            # TypeScript types
├── pkg/
│   └── plugin/
│       ├── datasource.go   # Backend data source (Go)
│       └── plugin.go       # Plugin entry point
└── provisioning/           # Auto-provisioning configs
```

**Query Format:**
```typescript
interface WiaClimateQuery {
  dataType: 'carbon_capture' | 'vertical_farming' | 'renewable_energy' | ...;
  location?: {
    latitude: number;
    longitude: number;
    radius_km: number;
  };
  deviceFilter?: string[];
  aggregation?: 'avg' | 'sum' | 'min' | 'max';
  interval?: string;  // e.g., '5m', '1h', '1d'
}
```

**Grafana API Integration:**
- `/api/datasources` - Register data source
- `/api/dashboards` - Create/update dashboards
- `/api/annotations` - Add event annotations
- `/api/alerts` - Configure alerting rules

**Best Practices:**
- Use streaming queries for real-time data
- Implement query caching for historical data
- Support Grafana variables for dynamic dashboards
- Provide pre-built dashboard templates

### 1.2 CesiumJS 3D Geospatial Visualization

**Overview:**
- CesiumJS is a JavaScript library for 3D globes and 2D maps
- Supports temporal data visualization with timeline controls
- Native support for CZML (Cesium Language) format

**CZML Format for Climate Data:**
```json
[
  {
    "id": "document",
    "name": "WIA Climate Data",
    "version": "1.0"
  },
  {
    "id": "sensor-001",
    "name": "Carbon Capture Facility",
    "position": {
      "cartographicDegrees": [-21.0, 64.0, 100.0]
    },
    "point": {
      "color": {
        "rgba": [0, 255, 0, 255]
      },
      "pixelSize": 10
    },
    "properties": {
      "capture_rate_kg_per_hour": 125.5,
      "technology": "DAC"
    }
  }
]
```

**Integration Approaches:**
1. **REST API Endpoint** - Serve CZML from WIA Climate data
2. **WebSocket Streaming** - Real-time entity updates
3. **3D Tiles** - Large-scale point cloud visualization

**Visualization Types:**
- Point entities for sensor locations
- Heatmaps for regional data aggregation
- Polylines for tracking mobile sensors
- Time-dynamic properties for historical playback

## 2. Time-Series Storage Adapters

### 2.1 InfluxDB Integration

**Overview:**
- InfluxDB is a purpose-built time-series database
- Native support for high-write throughput
- Flux query language for complex analytics

**Rust Client: `influxdb2`**
```toml
[dependencies]
influxdb2 = "0.5"
influxdb2-structmap = "0.2"
```

**Data Model Mapping:**
```
WIA Climate Message → InfluxDB Line Protocol

Measurement: wia_climate_{data_type}
Tags:
  - device_id
  - device_manufacturer
  - technology (for carbon_capture)
  - crop_type (for vertical_farming)
  - energy_type (for renewable_energy)
Fields:
  - All numeric values from the data payload
Timestamp: message timestamp

Example:
wia_climate_carbon_capture,device_id=ORCA-001,technology=dac capture_rate_kg_per_hour=125.5,co2_purity_percentage=99.2 1702500000000000000
```

**Bucket Organization:**
```
org: wia-climate
├── bucket: realtime (retention: 7d)
│   └── High-frequency sensor data
├── bucket: aggregated (retention: 1y)
│   └── Hourly/daily aggregations
└── bucket: archive (retention: infinite)
    └── Important events and snapshots
```

**Flux Query Examples:**
```flux
// Get average capture rate by technology
from(bucket: "realtime")
  |> range(start: -1h)
  |> filter(fn: (r) => r._measurement == "wia_climate_carbon_capture")
  |> group(columns: ["technology"])
  |> mean(column: "capture_rate_kg_per_hour")

// Detect anomalies
from(bucket: "realtime")
  |> range(start: -24h)
  |> filter(fn: (r) => r._measurement == "wia_climate_carbon_capture")
  |> movingAverage(n: 10)
  |> derivative(unit: 1h)
```

### 2.2 TimescaleDB Integration

**Overview:**
- TimescaleDB is a PostgreSQL extension for time-series
- Full SQL support with time-series optimizations
- Automatic data compression and retention policies

**Rust Client: `tokio-postgres` with `pgx`**
```toml
[dependencies]
tokio-postgres = "0.7"
deadpool-postgres = "0.12"
```

**Schema Design:**
```sql
-- Create hypertable for climate data
CREATE TABLE wia_climate_data (
    time TIMESTAMPTZ NOT NULL,
    device_id TEXT NOT NULL,
    data_type TEXT NOT NULL,
    location GEOGRAPHY(POINT, 4326),
    metadata JSONB,
    measurements JSONB
);

SELECT create_hypertable('wia_climate_data', 'time');

-- Create continuous aggregate for hourly stats
CREATE MATERIALIZED VIEW climate_hourly
WITH (timescaledb.continuous) AS
SELECT
    time_bucket('1 hour', time) AS bucket,
    device_id,
    data_type,
    avg((measurements->>'capture_rate_kg_per_hour')::float) as avg_capture_rate,
    count(*) as sample_count
FROM wia_climate_data
WHERE data_type = 'carbon_capture'
GROUP BY bucket, device_id, data_type;

-- Compression policy (compress data older than 7 days)
ALTER TABLE wia_climate_data SET (
    timescaledb.compress,
    timescaledb.compress_segmentby = 'device_id'
);
SELECT add_compression_policy('wia_climate_data', INTERVAL '7 days');

-- Retention policy (drop data older than 1 year)
SELECT add_retention_policy('wia_climate_data', INTERVAL '1 year');
```

**Query Examples:**
```sql
-- Time-weighted average with gap filling
SELECT
    time_bucket_gapfill('1 hour', time) AS hour,
    device_id,
    locf(avg((measurements->>'capture_rate_kg_per_hour')::float)) as capture_rate
FROM wia_climate_data
WHERE time > NOW() - INTERVAL '24 hours'
GROUP BY hour, device_id;

-- Geospatial query with PostGIS
SELECT *
FROM wia_climate_data
WHERE ST_DWithin(
    location,
    ST_SetSRID(ST_MakePoint(-21.0, 64.0), 4326)::geography,
    10000  -- 10km radius
)
AND time > NOW() - INTERVAL '1 hour';
```

## 3. Alert & Notification Adapters

### 3.1 Webhook Notification System

**Webhook Payload Format:**
```json
{
  "event_type": "threshold_exceeded",
  "severity": "warning",
  "timestamp": "2024-12-14T10:30:00Z",
  "source": {
    "device_id": "ORCA-001",
    "location": {
      "latitude": 64.0,
      "longitude": -21.0
    }
  },
  "alert": {
    "rule_id": "capture-rate-low",
    "rule_name": "Low Capture Rate Alert",
    "condition": "capture_rate_kg_per_hour < 100",
    "current_value": 85.5,
    "threshold": 100
  },
  "context": {
    "data_type": "carbon_capture",
    "technology": "dac",
    "recent_average": 125.5
  }
}
```

**Webhook Security:**
- HMAC-SHA256 signature in `X-WIA-Signature` header
- Timestamp validation to prevent replay attacks
- TLS 1.3 required for all endpoints
- Retry with exponential backoff (1s, 2s, 4s, 8s, max 60s)

**Alert Rule Configuration:**
```json
{
  "rules": [
    {
      "id": "capture-rate-low",
      "name": "Low Capture Rate Alert",
      "data_type": "carbon_capture",
      "condition": {
        "field": "capture_rate_kg_per_hour",
        "operator": "lt",
        "threshold": 100
      },
      "duration": "5m",
      "severity": "warning",
      "notifications": [
        {
          "type": "webhook",
          "url": "https://alerts.example.com/wia",
          "headers": {
            "Authorization": "Bearer ${ALERT_TOKEN}"
          }
        },
        {
          "type": "email",
          "recipients": ["ops@example.com"]
        }
      ]
    }
  ]
}
```

**Integration Targets:**
- Slack/Discord webhooks
- PagerDuty integration
- OpsGenie alerts
- Email via SendGrid/SES
- SMS via Twilio

### 3.2 Alert Engine Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Alert Engine                          │
├─────────────────────────────────────────────────────────┤
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────┐   │
│  │ Rule Engine │──▶│  Evaluator  │──▶│  Notifier   │   │
│  └─────────────┘   └─────────────┘   └─────────────┘   │
│        │                  │                  │          │
│        ▼                  ▼                  ▼          │
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────┐   │
│  │ Rule Store  │   │Alert State  │   │Webhook Queue│   │
│  │   (JSON)    │   │  (Redis)    │   │  (Async)    │   │
│  └─────────────┘   └─────────────┘   └─────────────┘   │
└─────────────────────────────────────────────────────────┘
```

**Alert States:**
- `pending` - Condition met, waiting for duration
- `firing` - Alert active, notifications sent
- `resolved` - Condition cleared
- `silenced` - Manually suppressed

## 4. Scientific Data Export

### 4.1 NetCDF Export

**Overview:**
- NetCDF (Network Common Data Form) is standard for scientific data
- Self-describing format with metadata
- Efficient storage of multi-dimensional arrays

**Rust Library: `netcdf`**
```toml
[dependencies]
netcdf = "0.7"
```

**NetCDF Structure for Climate Data:**
```
netcdf wia_climate_20241214 {
dimensions:
    time = UNLIMITED ;
    location = 100 ;

variables:
    double time(time) ;
        time:units = "seconds since 1970-01-01 00:00:00" ;
        time:calendar = "standard" ;

    double latitude(location) ;
        latitude:units = "degrees_north" ;
        latitude:standard_name = "latitude" ;

    double longitude(location) ;
        longitude:units = "degrees_east" ;
        longitude:standard_name = "longitude" ;

    float capture_rate(time, location) ;
        capture_rate:units = "kg/hour" ;
        capture_rate:long_name = "CO2 Capture Rate" ;
        capture_rate:_FillValue = -9999.f ;

// global attributes:
    :Conventions = "CF-1.8" ;
    :title = "WIA Climate Carbon Capture Data" ;
    :institution = "WIA Climate Network" ;
    :source = "WIA Climate API v1.0" ;
    :history = "Created 2024-12-14" ;
}
```

**CF Conventions Compliance:**
- Follow CF-1.8 conventions for interoperability
- Use standard_name attributes where applicable
- Include coordinate reference system metadata
- Document units using UDUNITS-2 compatible strings

### 4.2 CSV/Parquet Export

**CSV Format:**
```csv
timestamp,device_id,latitude,longitude,data_type,capture_rate_kg_per_hour,co2_purity_percentage,technology
2024-12-14T10:00:00Z,ORCA-001,64.0,-21.0,carbon_capture,125.5,99.2,dac
2024-12-14T10:05:00Z,ORCA-001,64.0,-21.0,carbon_capture,126.2,99.1,dac
```

**Parquet for Big Data:**
```toml
[dependencies]
parquet = "53"
arrow = "53"
```

- Columnar storage for efficient analytics
- Compression (Snappy, ZSTD, Gzip)
- Schema evolution support
- Partition by time/device for query optimization

## 5. Rust Implementation Architecture

### 5.1 Output Adapter Trait

```rust
#[async_trait]
pub trait OutputAdapter: Send + Sync {
    /// Adapter name
    fn name(&self) -> &str;

    /// Initialize the adapter
    async fn init(&mut self) -> Result<()>;

    /// Process a climate message
    async fn process(&self, message: &ClimateMessage) -> Result<()>;

    /// Flush any buffered data
    async fn flush(&self) -> Result<()>;

    /// Close the adapter
    async fn close(&mut self) -> Result<()>;

    /// Check adapter health
    async fn health_check(&self) -> Result<AdapterHealth>;
}

pub struct AdapterHealth {
    pub status: HealthStatus,
    pub latency_ms: Option<u64>,
    pub last_success: Option<DateTime<Utc>>,
    pub error_count: u64,
}

pub enum HealthStatus {
    Healthy,
    Degraded,
    Unhealthy,
}
```

### 5.2 Adapter Implementations

```rust
// Dashboard Adapters
pub struct GrafanaAdapter { /* ... */ }
pub struct CesiumAdapter { /* ... */ }

// Storage Adapters
pub struct InfluxDBAdapter { /* ... */ }
pub struct TimescaleDBAdapter { /* ... */ }

// Alert Adapters
pub struct WebhookAdapter { /* ... */ }
pub struct AlertManagerAdapter { /* ... */ }

// Export Adapters
pub struct NetCDFExporter { /* ... */ }
pub struct ParquetExporter { /* ... */ }
```

### 5.3 Output Manager

```rust
pub struct OutputManager {
    adapters: Vec<Box<dyn OutputAdapter>>,
    buffer: MessageBuffer,
    config: OutputConfig,
}

impl OutputManager {
    /// Add an adapter to the pipeline
    pub fn add_adapter(&mut self, adapter: impl OutputAdapter + 'static);

    /// Process a message through all adapters
    pub async fn process(&self, message: &ClimateMessage) -> Result<ProcessResult>;

    /// Process with fanout (parallel) or pipeline (sequential)
    pub async fn process_with_strategy(
        &self,
        message: &ClimateMessage,
        strategy: ProcessStrategy,
    ) -> Result<ProcessResult>;
}

pub enum ProcessStrategy {
    Fanout,   // Parallel to all adapters
    Pipeline, // Sequential through adapters
    Selective(Vec<String>), // Only specified adapters
}
```

## 6. Configuration Examples

### 6.1 Full Integration Config

```toml
[output]
strategy = "fanout"
buffer_size = 1000
flush_interval_ms = 5000

[[output.adapters]]
type = "influxdb"
url = "http://localhost:8086"
org = "wia-climate"
bucket = "realtime"
token = "${INFLUXDB_TOKEN}"
batch_size = 100

[[output.adapters]]
type = "webhook"
url = "https://alerts.example.com/wia"
secret = "${WEBHOOK_SECRET}"
retry_max = 3
timeout_ms = 5000

[[output.adapters]]
type = "grafana"
url = "http://localhost:3000"
api_key = "${GRAFANA_API_KEY}"
dashboard_uid = "wia-climate-main"

[alerts]
evaluation_interval = "10s"

[[alerts.rules]]
id = "capture-rate-low"
data_type = "carbon_capture"
condition = "capture_rate_kg_per_hour < 100"
duration = "5m"
severity = "warning"
```

## 7. Key Considerations

### 7.1 Performance
- Use batch writes for storage adapters (100-1000 messages)
- Implement connection pooling for database adapters
- Use async I/O throughout for non-blocking operation
- Buffer messages during temporary outages

### 7.2 Reliability
- Implement circuit breakers for external services
- Use dead-letter queues for failed messages
- Provide health check endpoints for monitoring
- Support graceful degradation (continue with healthy adapters)

### 7.3 Security
- Encrypt sensitive config values (tokens, secrets)
- Use TLS for all external connections
- Validate webhook signatures
- Implement rate limiting for outbound calls

## References

1. Grafana Data Source Plugin Development
   - https://grafana.com/docs/grafana/latest/developers/plugins/

2. InfluxDB Rust Client
   - https://github.com/influxdata/influxdb-rust

3. TimescaleDB Documentation
   - https://docs.timescale.com/

4. CesiumJS Documentation
   - https://cesium.com/learn/cesiumjs/ref-doc/

5. NetCDF-Rust
   - https://github.com/georust/netcdf

6. CF Conventions
   - https://cfconventions.org/

7. Webhook Best Practices
   - https://webhooks.fyi/
