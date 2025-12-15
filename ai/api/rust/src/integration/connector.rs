//! WIA Connector trait and implementations

use async_trait::async_trait;
use std::pin::Pin;
use tokio_stream::Stream;

use crate::error::WiaAiError;
use super::{WiaMessage, WiaStandardType};

/// WIA Standard Connector trait
///
/// Connectors provide the bridge between WIA AI and other WIA standards.
/// Each connector handles communication with a specific WIA standard.
///
/// # Example
///
/// ```rust,no_run
/// use wia_ai::integration::{WiaConnector, WiaStandardType, WiaMessage};
/// use wia_ai::error::WiaAiError;
/// use async_trait::async_trait;
/// use std::pin::Pin;
/// use tokio_stream::Stream;
///
/// struct MyConnector {
///     connected: bool,
/// }
///
/// #[async_trait]
/// impl WiaConnector for MyConnector {
///     fn id(&self) -> &str { "my-connector" }
///     fn standard_type(&self) -> WiaStandardType { WiaStandardType::Aac }
///     fn is_connected(&self) -> bool { self.connected }
///     // ... implement other methods
/// #    async fn connect(&mut self) -> Result<(), WiaAiError> { Ok(()) }
/// #    async fn disconnect(&mut self) -> Result<(), WiaAiError> { Ok(()) }
/// #    async fn receive(&self) -> Result<WiaMessage, WiaAiError> { unimplemented!() }
/// #    async fn send(&self, message: WiaMessage) -> Result<(), WiaAiError> { Ok(()) }
/// #    fn receive_stream(&self) -> Pin<Box<dyn Stream<Item = Result<WiaMessage, WiaAiError>> + Send>> { unimplemented!() }
/// }
/// ```
#[async_trait]
pub trait WiaConnector: Send + Sync {
    /// Get the connector's unique identifier
    fn id(&self) -> &str;

    /// Get the WIA standard type this connector handles
    fn standard_type(&self) -> WiaStandardType;

    /// Check if the connector is currently connected
    fn is_connected(&self) -> bool;

    /// Establish connection to the WIA standard endpoint
    async fn connect(&mut self) -> Result<(), WiaAiError>;

    /// Disconnect from the WIA standard endpoint
    async fn disconnect(&mut self) -> Result<(), WiaAiError>;

    /// Receive a single message (blocking)
    async fn receive(&self) -> Result<WiaMessage, WiaAiError>;

    /// Send a message to the WIA standard endpoint
    async fn send(&self, message: WiaMessage) -> Result<(), WiaAiError>;

    /// Get a stream of incoming messages
    fn receive_stream(&self) -> Pin<Box<dyn Stream<Item = Result<WiaMessage, WiaAiError>> + Send>>;
}

/// AI Input Adapter trait
///
/// Input adapters convert WIA messages into AI-processable inputs.
#[async_trait]
pub trait AiInputAdapter: Send + Sync {
    /// Get the adapter name
    fn name(&self) -> &str;

    /// Get the input type this adapter produces
    fn input_type(&self) -> super::AiInputType;

    /// Get the WIA standards this adapter supports
    fn supported_standards(&self) -> Vec<WiaStandardType>;

    /// Convert a WIA message to AI input
    async fn to_ai_input(&self, message: WiaMessage) -> Result<super::AiInput, WiaAiError>;

    /// Check if this adapter can handle the given message
    fn can_handle(&self, message: &WiaMessage) -> bool;
}

/// AI Output Adapter trait
///
/// Output adapters convert AI outputs into WIA messages.
#[async_trait]
pub trait AiOutputAdapter: Send + Sync {
    /// Get the adapter name
    fn name(&self) -> &str;

    /// Get the output type this adapter handles
    fn output_type(&self) -> super::AiOutputType;

    /// Get the target WIA standard
    fn target_standard(&self) -> WiaStandardType;

    /// Convert AI output to WIA message
    async fn from_ai_output(&self, output: super::AiOutput) -> Result<WiaMessage, WiaAiError>;

    /// Check if streaming output is supported
    fn supports_streaming(&self) -> bool {
        false
    }

    /// Handle streaming output
    async fn stream_output(
        &self,
        _output: Pin<Box<dyn Stream<Item = super::AiOutput> + Send>>,
    ) -> Result<(), WiaAiError> {
        Err(WiaAiError::NotSupported("streaming output".into()))
    }
}

/// Connection configuration
#[derive(Debug, Clone)]
pub struct ConnectionConfig {
    /// Endpoint URL or address
    pub endpoint: String,
    /// Connection timeout in milliseconds
    pub timeout_ms: u64,
    /// Retry count on connection failure
    pub retry_count: u32,
    /// Additional options
    pub options: std::collections::HashMap<String, String>,
}

impl Default for ConnectionConfig {
    fn default() -> Self {
        Self {
            endpoint: "localhost".into(),
            timeout_ms: 5000,
            retry_count: 3,
            options: std::collections::HashMap::new(),
        }
    }
}

impl ConnectionConfig {
    /// Create a new connection config with endpoint
    pub fn new(endpoint: impl Into<String>) -> Self {
        Self {
            endpoint: endpoint.into(),
            ..Default::default()
        }
    }

    /// Set timeout
    pub fn with_timeout(mut self, timeout_ms: u64) -> Self {
        self.timeout_ms = timeout_ms;
        self
    }

    /// Set retry count
    pub fn with_retries(mut self, count: u32) -> Self {
        self.retry_count = count;
        self
    }

    /// Add an option
    pub fn with_option(mut self, key: impl Into<String>, value: impl Into<String>) -> Self {
        self.options.insert(key.into(), value.into());
        self
    }
}
