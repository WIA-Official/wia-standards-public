//! Distributed tracing configuration (OpenTelemetry compatible)

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Tracing configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TracingConfig {
    /// Service name
    pub service_name: String,

    /// Service version
    pub service_version: String,

    /// OTLP exporter configuration
    pub exporter: ExporterConfig,

    /// Sampling configuration
    pub sampling: SamplingConfig,

    /// Propagation formats
    pub propagation: Vec<PropagationFormat>,

    /// Resource attributes
    pub resource_attributes: HashMap<String, String>,
}

impl Default for TracingConfig {
    fn default() -> Self {
        let mut resource_attributes = HashMap::new();
        resource_attributes.insert("service.namespace".to_string(), "wia".to_string());
        resource_attributes.insert(
            "deployment.environment".to_string(),
            "development".to_string(),
        );

        Self {
            service_name: "voice-sign-api".to_string(),
            service_version: env!("CARGO_PKG_VERSION").to_string(),
            exporter: ExporterConfig::default(),
            sampling: SamplingConfig::default(),
            propagation: vec![
                PropagationFormat::TraceContext,
                PropagationFormat::Baggage,
            ],
            resource_attributes,
        }
    }
}

/// Exporter configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExporterConfig {
    /// Exporter type
    pub exporter_type: ExporterType,

    /// OTLP endpoint
    pub endpoint: String,

    /// Protocol (grpc or http)
    pub protocol: ExporterProtocol,

    /// Headers
    pub headers: HashMap<String, String>,

    /// Timeout in seconds
    pub timeout_seconds: u64,
}

impl Default for ExporterConfig {
    fn default() -> Self {
        Self {
            exporter_type: ExporterType::Otlp,
            endpoint: "http://localhost:4317".to_string(),
            protocol: ExporterProtocol::Grpc,
            headers: HashMap::new(),
            timeout_seconds: 10,
        }
    }
}

/// Exporter type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ExporterType {
    Otlp,
    Jaeger,
    Zipkin,
    Console,
    None,
}

/// Exporter protocol
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ExporterProtocol {
    Grpc,
    Http,
}

/// Sampling configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SamplingConfig {
    /// Sampling type
    pub sampler_type: SamplerType,

    /// Sample rate (0.0 - 1.0)
    pub rate: f64,

    /// Parent-based sampling
    pub parent_based: bool,
}

impl Default for SamplingConfig {
    fn default() -> Self {
        Self {
            sampler_type: SamplerType::Probability,
            rate: 0.1,
            parent_based: true,
        }
    }
}

/// Sampler type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SamplerType {
    /// Always sample
    AlwaysOn,
    /// Never sample
    AlwaysOff,
    /// Probability-based sampling
    Probability,
    /// Rate limiting sampler
    RateLimiting,
}

/// Propagation format
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum PropagationFormat {
    /// W3C Trace Context
    TraceContext,
    /// W3C Baggage
    Baggage,
    /// B3 (Zipkin)
    B3,
    /// Jaeger
    Jaeger,
}

/// Span context
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpanContext {
    /// Trace ID
    pub trace_id: String,

    /// Span ID
    pub span_id: String,

    /// Parent span ID
    pub parent_span_id: Option<String>,

    /// Trace flags
    pub trace_flags: u8,

    /// Trace state
    pub trace_state: String,
}

/// Span data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpanData {
    /// Span name
    pub name: String,

    /// Span context
    pub context: SpanContext,

    /// Span kind
    pub kind: SpanKind,

    /// Start time (Unix timestamp in nanoseconds)
    pub start_time_unix_nano: u64,

    /// End time (Unix timestamp in nanoseconds)
    pub end_time_unix_nano: u64,

    /// Attributes
    pub attributes: HashMap<String, AttributeValue>,

    /// Events
    pub events: Vec<SpanEvent>,

    /// Links
    pub links: Vec<SpanLink>,

    /// Status
    pub status: SpanStatus,
}

/// Span kind
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum SpanKind {
    Internal,
    Server,
    Client,
    Producer,
    Consumer,
}

/// Attribute value
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum AttributeValue {
    String(String),
    Bool(bool),
    Int(i64),
    Float(f64),
    StringArray(Vec<String>),
    IntArray(Vec<i64>),
}

/// Span event
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpanEvent {
    /// Event name
    pub name: String,

    /// Timestamp
    pub timestamp_unix_nano: u64,

    /// Attributes
    pub attributes: HashMap<String, AttributeValue>,
}

/// Span link
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpanLink {
    /// Linked span context
    pub context: SpanContext,

    /// Attributes
    pub attributes: HashMap<String, AttributeValue>,
}

/// Span status
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpanStatus {
    /// Status code
    pub code: StatusCode,

    /// Message
    pub message: Option<String>,
}

/// Status code
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum StatusCode {
    Unset,
    Ok,
    Error,
}

/// Trace context headers
#[derive(Debug, Clone)]
pub struct TraceContextHeaders {
    /// traceparent header
    pub traceparent: String,

    /// tracestate header
    pub tracestate: Option<String>,
}

impl TraceContextHeaders {
    /// Parse from headers
    pub fn from_headers(headers: &HashMap<String, String>) -> Option<Self> {
        let traceparent = headers.get("traceparent")?.clone();
        let tracestate = headers.get("tracestate").cloned();
        Some(Self {
            traceparent,
            tracestate,
        })
    }

    /// Convert to headers
    pub fn to_headers(&self) -> HashMap<String, String> {
        let mut headers = HashMap::new();
        headers.insert("traceparent".to_string(), self.traceparent.clone());
        if let Some(ref state) = self.tracestate {
            headers.insert("tracestate".to_string(), state.clone());
        }
        headers
    }

    /// Generate a new trace context
    pub fn new_root() -> Self {
        let trace_id = Self::generate_trace_id();
        let span_id = Self::generate_span_id();
        Self {
            traceparent: format!("00-{}-{}-01", trace_id, span_id),
            tracestate: None,
        }
    }

    /// Generate a child context
    pub fn new_child(&self) -> Self {
        let parts: Vec<&str> = self.traceparent.split('-').collect();
        if parts.len() != 4 {
            return Self::new_root();
        }

        let trace_id = parts[1];
        let span_id = Self::generate_span_id();

        Self {
            traceparent: format!("00-{}-{}-01", trace_id, span_id),
            tracestate: self.tracestate.clone(),
        }
    }

    fn generate_trace_id() -> String {
        let bytes: [u8; 16] = rand_bytes();
        hex::encode(bytes)
    }

    fn generate_span_id() -> String {
        let bytes: [u8; 8] = rand_bytes();
        hex::encode(bytes)
    }
}

fn rand_bytes<const N: usize>() -> [u8; N] {
    let mut bytes = [0u8; N];
    for byte in &mut bytes {
        *byte = (std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .subsec_nanos()
            & 0xFF) as u8;
    }
    bytes
}

fn hex_encode(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{:02x}", b)).collect()
}

mod hex {
    pub fn encode(bytes: impl AsRef<[u8]>) -> String {
        bytes.as_ref().iter().map(|b| format!("{:02x}", b)).collect()
    }
}

/// Simple span builder
pub struct SpanBuilder {
    name: String,
    kind: SpanKind,
    attributes: HashMap<String, AttributeValue>,
}

impl SpanBuilder {
    /// Create a new span builder
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            kind: SpanKind::Internal,
            attributes: HashMap::new(),
        }
    }

    /// Set span kind
    pub fn kind(mut self, kind: SpanKind) -> Self {
        self.kind = kind;
        self
    }

    /// Add attribute
    pub fn attribute(mut self, key: impl Into<String>, value: impl Into<AttributeValue>) -> Self {
        self.attributes.insert(key.into(), value.into());
        self
    }
}

impl From<String> for AttributeValue {
    fn from(s: String) -> Self {
        AttributeValue::String(s)
    }
}

impl From<&str> for AttributeValue {
    fn from(s: &str) -> Self {
        AttributeValue::String(s.to_string())
    }
}

impl From<i64> for AttributeValue {
    fn from(i: i64) -> Self {
        AttributeValue::Int(i)
    }
}

impl From<f64> for AttributeValue {
    fn from(f: f64) -> Self {
        AttributeValue::Float(f)
    }
}

impl From<bool> for AttributeValue {
    fn from(b: bool) -> Self {
        AttributeValue::Bool(b)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = TracingConfig::default();
        assert_eq!(config.service_name, "voice-sign-api");
        assert_eq!(config.sampling.rate, 0.1);
    }

    #[test]
    fn test_trace_context() {
        let ctx = TraceContextHeaders::new_root();
        assert!(ctx.traceparent.starts_with("00-"));

        let child = ctx.new_child();
        // Child should have same trace ID
        let parent_parts: Vec<&str> = ctx.traceparent.split('-').collect();
        let child_parts: Vec<&str> = child.traceparent.split('-').collect();
        assert_eq!(parent_parts[1], child_parts[1]); // Same trace ID
        assert_ne!(parent_parts[2], child_parts[2]); // Different span ID
    }

    #[test]
    fn test_span_builder() {
        let span = SpanBuilder::new("test_operation")
            .kind(SpanKind::Server)
            .attribute("http.method", "POST")
            .attribute("http.status_code", 200i64);

        assert_eq!(span.name, "test_operation");
        assert_eq!(span.kind, SpanKind::Server);
        assert!(span.attributes.contains_key("http.method"));
    }
}
