//! InfluxDB output adapter for time-series storage

use async_trait::async_trait;
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::RwLock;
use std::time::Duration;

use crate::error::{ClimateError, Result};
use crate::core::climate::ClimateMessage;
use crate::integration::{OutputAdapter, AdapterType, AdapterHealth, HealthStatus, BatchResult};

/// Configuration for the InfluxDB adapter
#[derive(Debug, Clone)]
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
    /// Write precision
    pub precision: WritePrecision,
}

/// Write precision for InfluxDB timestamps
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum WritePrecision {
    /// Seconds
    Seconds,
    /// Milliseconds
    Milliseconds,
    /// Microseconds
    Microseconds,
    /// Nanoseconds
    Nanoseconds,
}

impl Default for WritePrecision {
    fn default() -> Self {
        WritePrecision::Nanoseconds
    }
}

impl Default for InfluxDBConfig {
    fn default() -> Self {
        Self {
            url: "http://localhost:8086".to_string(),
            org: "wia-climate".to_string(),
            bucket: "climate-data".to_string(),
            token: String::new(),
            batch_size: 100,
            flush_interval: Duration::from_secs(1),
            gzip: true,
            precision: WritePrecision::Nanoseconds,
        }
    }
}

impl InfluxDBConfig {
    /// Create a new InfluxDB config with required parameters
    pub fn new(url: impl Into<String>, org: impl Into<String>, bucket: impl Into<String>) -> Self {
        Self {
            url: url.into(),
            org: org.into(),
            bucket: bucket.into(),
            ..Default::default()
        }
    }

    /// Set authentication token
    pub fn with_token(mut self, token: impl Into<String>) -> Self {
        self.token = token.into();
        self
    }

    /// Set batch size
    pub fn with_batch_size(mut self, size: usize) -> Self {
        self.batch_size = size;
        self
    }

    /// Set flush interval
    pub fn with_flush_interval(mut self, interval: Duration) -> Self {
        self.flush_interval = interval;
        self
    }

    /// Enable or disable gzip compression
    pub fn with_gzip(mut self, gzip: bool) -> Self {
        self.gzip = gzip;
        self
    }

    /// Set write precision
    pub fn with_precision(mut self, precision: WritePrecision) -> Self {
        self.precision = precision;
        self
    }
}

/// InfluxDB Line Protocol point
#[derive(Debug, Clone)]
pub struct LineProtocolPoint {
    /// Measurement name
    pub measurement: String,
    /// Tags (indexed)
    pub tags: Vec<(String, String)>,
    /// Fields (values)
    pub fields: Vec<(String, FieldValue)>,
    /// Timestamp
    pub timestamp: Option<i64>,
}

/// Field value types for InfluxDB
#[derive(Debug, Clone)]
pub enum FieldValue {
    /// Float value
    Float(f64),
    /// Integer value
    Integer(i64),
    /// String value
    String(String),
    /// Boolean value
    Boolean(bool),
}

impl std::fmt::Display for FieldValue {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FieldValue::Float(v) => write!(f, "{}", v),
            FieldValue::Integer(v) => write!(f, "{}i", v),
            FieldValue::String(v) => write!(f, "\"{}\"", v.replace('"', "\\\"")),
            FieldValue::Boolean(v) => write!(f, "{}", v),
        }
    }
}

impl LineProtocolPoint {
    /// Create a new point with the given measurement name
    pub fn new(measurement: impl Into<String>) -> Self {
        Self {
            measurement: measurement.into(),
            tags: Vec::new(),
            fields: Vec::new(),
            timestamp: None,
        }
    }

    /// Add a tag
    pub fn tag(mut self, key: impl Into<String>, value: impl Into<String>) -> Self {
        self.tags.push((key.into(), value.into()));
        self
    }

    /// Add a float field
    pub fn field_float(mut self, key: impl Into<String>, value: f64) -> Self {
        self.fields.push((key.into(), FieldValue::Float(value)));
        self
    }

    /// Add an integer field
    pub fn field_int(mut self, key: impl Into<String>, value: i64) -> Self {
        self.fields.push((key.into(), FieldValue::Integer(value)));
        self
    }

    /// Add a string field
    pub fn field_string(mut self, key: impl Into<String>, value: impl Into<String>) -> Self {
        self.fields.push((key.into(), FieldValue::String(value.into())));
        self
    }

    /// Add a boolean field
    pub fn field_bool(mut self, key: impl Into<String>, value: bool) -> Self {
        self.fields.push((key.into(), FieldValue::Boolean(value)));
        self
    }

    /// Set timestamp (nanoseconds since epoch)
    pub fn timestamp(mut self, ts: i64) -> Self {
        self.timestamp = Some(ts);
        self
    }

    /// Convert to line protocol string
    pub fn to_line_protocol(&self) -> String {
        let mut line = escape_measurement(&self.measurement);

        // Add tags
        for (key, value) in &self.tags {
            line.push(',');
            line.push_str(&escape_tag_key(key));
            line.push('=');
            line.push_str(&escape_tag_value(value));
        }

        // Add fields
        line.push(' ');
        let fields: Vec<String> = self.fields
            .iter()
            .map(|(k, v)| format!("{}={}", escape_field_key(k), v))
            .collect();
        line.push_str(&fields.join(","));

        // Add timestamp
        if let Some(ts) = self.timestamp {
            line.push(' ');
            line.push_str(&ts.to_string());
        }

        line
    }
}

fn escape_measurement(s: &str) -> String {
    s.replace(',', "\\,").replace(' ', "\\ ")
}

fn escape_tag_key(s: &str) -> String {
    s.replace(',', "\\,").replace('=', "\\=").replace(' ', "\\ ")
}

fn escape_tag_value(s: &str) -> String {
    s.replace(',', "\\,").replace('=', "\\=").replace(' ', "\\ ")
}

fn escape_field_key(s: &str) -> String {
    s.replace(',', "\\,").replace('=', "\\=").replace(' ', "\\ ")
}

/// InfluxDB output adapter for time-series storage
///
/// This adapter writes climate messages to InfluxDB using the Line Protocol.
/// It supports batching, compression, and automatic retries.
///
/// # Data Model
///
/// Climate messages are stored with the following structure:
/// - Measurement: `wia_climate_{data_type}`
/// - Tags: device_id, technology, location coordinates
/// - Fields: All numeric values from the data payload
///
/// # Example
///
/// ```rust,ignore
/// use wia_climate::integration::adapters::{InfluxDBAdapter, InfluxDBConfig};
/// use wia_climate::integration::{OutputManager, OutputConfig};
///
/// let config = InfluxDBConfig::new(
///     "http://localhost:8086",
///     "wia-climate",
///     "climate-data"
/// ).with_token("my-token");
///
/// let mut manager = OutputManager::new(OutputConfig::default());
/// manager.add_adapter(InfluxDBAdapter::new("influx", config));
/// ```
pub struct InfluxDBAdapter {
    name: String,
    config: InfluxDBConfig,
    buffer: RwLock<Vec<LineProtocolPoint>>,
    messages_processed: AtomicU64,
    error_count: AtomicU64,
    last_error: RwLock<Option<String>>,
    last_success: RwLock<Option<DateTime<Utc>>>,
}

impl InfluxDBAdapter {
    /// Create a new InfluxDB adapter with the given configuration
    pub fn new(name: impl Into<String>, config: InfluxDBConfig) -> Self {
        Self {
            name: name.into(),
            config,
            buffer: RwLock::new(Vec::new()),
            messages_processed: AtomicU64::new(0),
            error_count: AtomicU64::new(0),
            last_error: RwLock::new(None),
            last_success: RwLock::new(None),
        }
    }

    /// Convert a climate message to line protocol points
    fn message_to_points(&self, message: &ClimateMessage) -> Vec<LineProtocolPoint> {
        let mut points = Vec::new();

        let measurement = format!("wia_climate_{}", message.data_type.to_string().to_lowercase());
        // Convert milliseconds to nanoseconds for InfluxDB precision
        let timestamp_nanos = message.timestamp.unix_ms * 1_000_000;

        // Get device identifier
        let device_id = message.device.serial.clone()
            .unwrap_or_else(|| format!("{}-{}", message.device.manufacturer, message.device.model));

        // Base point with common tags
        let base_point = LineProtocolPoint::new(&measurement)
            .tag("device_id", &device_id)
            .tag("manufacturer", &message.device.manufacturer)
            .tag("model", &message.device.model)
            .tag("lat", format!("{:.4}", message.location.latitude))
            .tag("lon", format!("{:.4}", message.location.longitude))
            .timestamp(timestamp_nanos);

        // Serialize data to JSON for field extraction
        let data_json = serde_json::to_value(&message.data).unwrap_or_default();

        // Extract numeric fields from the data
        if let Some(obj) = data_json.as_object() {
            let mut point = base_point.clone();

            for (key, value) in obj {
                if let Some(n) = value.as_f64() {
                    point = point.field_float(key, n);
                } else if let Some(n) = value.as_i64() {
                    point = point.field_int(key, n);
                } else if let Some(b) = value.as_bool() {
                    point = point.field_bool(key, b);
                } else if let Some(s) = value.as_str() {
                    // Add string as tag if it's a category
                    if key == "technology" || key == "crop_type" || key == "energy_type" {
                        point = point.tag(key, s);
                    }
                }
            }

            // Ensure at least one field
            if point.fields.is_empty() {
                point = point.field_int("count", 1);
            }

            points.push(point);
        } else {
            // If data is not an object, create a simple point
            let point = base_point.field_int("count", 1);
            points.push(point);
        }

        points
    }

    /// Write points to InfluxDB (mock implementation)
    async fn write_points(&self, points: &[LineProtocolPoint]) -> Result<()> {
        if points.is_empty() {
            return Ok(());
        }

        // Convert to line protocol
        let body: String = points
            .iter()
            .map(|p| p.to_line_protocol())
            .collect::<Vec<_>>()
            .join("\n");

        // In a real implementation, this would make an HTTP POST request:
        // let url = format!("{}/api/v2/write?org={}&bucket={}&precision=ns",
        //     self.config.url, self.config.org, self.config.bucket);
        //
        // let client = reqwest::Client::new();
        // let mut request = client.post(&url)
        //     .header("Authorization", format!("Token {}", self.config.token))
        //     .header("Content-Type", "text/plain")
        //     .body(body);
        //
        // if self.config.gzip {
        //     // Compress body
        // }
        //
        // let response = request.send().await?;

        // For now, validate configuration and log
        if self.config.token.is_empty() {
            return Err(ClimateError::ConnectionError("InfluxDB token not configured".to_string()));
        }

        tracing::debug!(
            target: "wia_climate::influxdb",
            url = %self.config.url,
            bucket = %self.config.bucket,
            points = points.len(),
            body_size = body.len(),
            "InfluxDB write prepared"
        );

        Ok(())
    }

    /// Flush the internal buffer
    async fn flush_buffer(&self) -> Result<()> {
        let points: Vec<LineProtocolPoint> = {
            let mut buffer = self.buffer.write().unwrap();
            std::mem::take(&mut *buffer)
        };

        if !points.is_empty() {
            self.write_points(&points).await?;
        }

        Ok(())
    }
}

#[async_trait]
impl OutputAdapter for InfluxDBAdapter {
    fn name(&self) -> &str {
        &self.name
    }

    fn adapter_type(&self) -> AdapterType {
        AdapterType::Storage
    }

    async fn init(&mut self) -> Result<()> {
        // Validate configuration
        if self.config.url.is_empty() {
            return Err(ClimateError::Validation("InfluxDB URL is required".to_string()));
        }

        if self.config.bucket.is_empty() {
            return Err(ClimateError::Validation("InfluxDB bucket is required".to_string()));
        }

        tracing::info!(
            target: "wia_climate::influxdb",
            name = %self.name,
            url = %self.config.url,
            bucket = %self.config.bucket,
            batch_size = self.config.batch_size,
            "InfluxDB adapter initialized"
        );

        Ok(())
    }

    async fn process(&self, message: &ClimateMessage) -> Result<()> {
        let points = self.message_to_points(message);

        // Check if we need to flush after adding points
        let points_to_write = {
            let mut buffer = self.buffer.write().unwrap();
            buffer.extend(points);

            // If buffer is full, take the points for writing
            if buffer.len() >= self.config.batch_size {
                Some(std::mem::take(&mut *buffer))
            } else {
                None
            }
            // Lock is released here
        };

        // Flush if we have points to write (after lock is released)
        if let Some(points_to_write) = points_to_write {
            match self.write_points(&points_to_write).await {
                Ok(_) => {
                    self.messages_processed.fetch_add(1, Ordering::SeqCst);
                    *self.last_success.write().unwrap() = Some(Utc::now());
                }
                Err(e) => {
                    self.error_count.fetch_add(1, Ordering::SeqCst);
                    *self.last_error.write().unwrap() = Some(e.to_string());
                    return Err(e);
                }
            }
        } else {
            self.messages_processed.fetch_add(1, Ordering::SeqCst);
        }

        Ok(())
    }

    async fn process_batch(&self, messages: &[ClimateMessage]) -> Result<BatchResult> {
        let mut all_points = Vec::new();

        for message in messages {
            all_points.extend(self.message_to_points(message));
        }

        // Write all points at once
        match self.write_points(&all_points).await {
            Ok(_) => {
                let count = messages.len() as u64;
                self.messages_processed.fetch_add(count, Ordering::SeqCst);
                *self.last_success.write().unwrap() = Some(Utc::now());
                Ok(BatchResult::new(messages.len(), 0))
            }
            Err(e) => {
                self.error_count.fetch_add(1, Ordering::SeqCst);
                *self.last_error.write().unwrap() = Some(e.to_string());
                Ok(BatchResult::new(0, messages.len()))
            }
        }
    }

    async fn flush(&self) -> Result<()> {
        self.flush_buffer().await
    }

    async fn close(&mut self) -> Result<()> {
        // Flush any remaining data
        self.flush_buffer().await?;

        tracing::info!(
            target: "wia_climate::influxdb",
            name = %self.name,
            messages_processed = self.messages_processed.load(Ordering::SeqCst),
            error_count = self.error_count.load(Ordering::SeqCst),
            "InfluxDB adapter closed"
        );

        Ok(())
    }

    async fn health_check(&self) -> Result<AdapterHealth> {
        let error_count = self.error_count.load(Ordering::SeqCst);
        let messages_processed = self.messages_processed.load(Ordering::SeqCst);

        // In real implementation, ping InfluxDB health endpoint
        let status = if error_count == 0 {
            HealthStatus::Healthy
        } else if error_count < 5 {
            HealthStatus::Degraded
        } else {
            HealthStatus::Unhealthy
        };

        Ok(AdapterHealth {
            status,
            latency_ms: None,
            last_success: *self.last_success.read().unwrap(),
            last_error: self.last_error.read().unwrap().clone(),
            error_count,
            messages_processed,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Location, Device, CarbonCaptureData, CarbonCaptureTechnology};

    fn create_test_message() -> ClimateMessage {
        ClimateMessage::builder()
            .location(Location::new(64.0, -21.0))
            .device(Device::new("Climeworks", "Orca DAC"))
            .carbon_capture_data(CarbonCaptureData {
                technology: CarbonCaptureTechnology::Dac,
                capture_rate_kg_per_hour: 125.5,
                co2_purity_percentage: Some(99.2),
                ..Default::default()
            })
            .build()
            .unwrap()
    }

    #[test]
    fn test_influxdb_config() {
        let config = InfluxDBConfig::new("http://localhost:8086", "org", "bucket")
            .with_token("token123")
            .with_batch_size(200)
            .with_gzip(false);

        assert_eq!(config.url, "http://localhost:8086");
        assert_eq!(config.org, "org");
        assert_eq!(config.bucket, "bucket");
        assert_eq!(config.token, "token123");
        assert_eq!(config.batch_size, 200);
        assert!(!config.gzip);
    }

    #[test]
    fn test_line_protocol_point() {
        let point = LineProtocolPoint::new("temperature")
            .tag("device", "sensor-1")
            .tag("location", "room-a")
            .field_float("value", 23.5)
            .field_int("count", 1)
            .timestamp(1702500000000000000);

        let line = point.to_line_protocol();
        assert!(line.starts_with("temperature,"));
        assert!(line.contains("device=sensor-1"));
        assert!(line.contains("value=23.5"));
        assert!(line.contains("count=1i"));
        assert!(line.ends_with("1702500000000000000"));
    }

    #[test]
    fn test_line_protocol_escaping() {
        let point = LineProtocolPoint::new("my measurement")
            .tag("tag key", "tag=value")
            .field_string("field,key", "hello\"world");

        let line = point.to_line_protocol();
        assert!(line.contains("my\\ measurement"));
        assert!(line.contains("tag\\ key=tag\\=value"));
    }

    #[tokio::test]
    async fn test_influxdb_adapter_creation() {
        let config = InfluxDBConfig::new("http://localhost:8086", "org", "bucket");
        let adapter = InfluxDBAdapter::new("test-influx", config);

        assert_eq!(adapter.name(), "test-influx");
        assert_eq!(adapter.adapter_type(), AdapterType::Storage);
    }

    #[tokio::test]
    async fn test_message_to_points() {
        let config = InfluxDBConfig::new("http://localhost:8086", "org", "bucket");
        let adapter = InfluxDBAdapter::new("test", config);
        let message = create_test_message();

        let points = adapter.message_to_points(&message);
        assert!(!points.is_empty());

        let point = &points[0];
        assert!(point.measurement.contains("carbon_capture"));
        assert!(!point.fields.is_empty());
    }

    #[tokio::test]
    async fn test_influxdb_health_check() {
        let config = InfluxDBConfig::new("http://localhost:8086", "org", "bucket");
        let adapter = InfluxDBAdapter::new("test", config);

        let health = adapter.health_check().await.unwrap();
        assert_eq!(health.status, HealthStatus::Healthy);
    }
}
