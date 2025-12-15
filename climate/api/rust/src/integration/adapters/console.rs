//! Console output adapter for testing and debugging

use async_trait::async_trait;
use chrono::Utc;
use std::sync::atomic::{AtomicU64, Ordering};

use crate::error::Result;
use crate::core::climate::ClimateMessage;
use crate::integration::{OutputAdapter, AdapterType, AdapterHealth, HealthStatus};

/// Console output adapter that prints messages to stdout
///
/// This adapter is useful for testing and debugging. It formats and prints
/// each climate message to the console with optional verbosity levels.
///
/// # Example
///
/// ```rust,ignore
/// use wia_climate::integration::adapters::ConsoleAdapter;
/// use wia_climate::integration::{OutputManager, OutputConfig};
///
/// let mut manager = OutputManager::new(OutputConfig::default());
/// manager.add_adapter(ConsoleAdapter::new("console").verbose());
/// ```
pub struct ConsoleAdapter {
    name: String,
    verbose: bool,
    json_output: bool,
    messages_processed: AtomicU64,
}

impl ConsoleAdapter {
    /// Create a new console adapter with the given name
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            verbose: false,
            json_output: false,
            messages_processed: AtomicU64::new(0),
        }
    }

    /// Enable verbose output
    pub fn verbose(mut self) -> Self {
        self.verbose = true;
        self
    }

    /// Enable JSON output format
    pub fn json(mut self) -> Self {
        self.json_output = true;
        self
    }
}

#[async_trait]
impl OutputAdapter for ConsoleAdapter {
    fn name(&self) -> &str {
        &self.name
    }

    fn adapter_type(&self) -> AdapterType {
        AdapterType::Console
    }

    async fn init(&mut self) -> Result<()> {
        if self.verbose {
            println!("[{}] Console adapter initialized", self.name);
        }
        Ok(())
    }

    async fn process(&self, message: &ClimateMessage) -> Result<()> {
        self.messages_processed.fetch_add(1, Ordering::SeqCst);

        if self.json_output {
            let json = serde_json::to_string_pretty(message)
                .unwrap_or_else(|_| "Failed to serialize".to_string());
            println!("{}", json);
        } else {
            println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
            println!("📊 WIA Climate Message");
            println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
            println!("  Type:      {}", message.data_type);
            println!("  Timestamp: {}", message.timestamp.iso8601.as_deref().unwrap_or("N/A"));
            println!("  Location:  ({:.4}, {:.4})",
                message.location.latitude,
                message.location.longitude
            );
            if let Some(alt) = message.location.altitude_m {
                println!("  Altitude:  {:.1}m", alt);
            }
            println!("  Device:    {} {}",
                message.device.manufacturer,
                message.device.model
            );
            if let Some(ref serial) = message.device.serial {
                println!("  Serial:    {}", serial);
            }

            if self.verbose {
                println!("  Data:");
                if let Ok(data_json) = serde_json::to_string_pretty(&message.data) {
                    for line in data_json.lines() {
                        println!("    {}", line);
                    }
                }
            }
            println!();
        }

        Ok(())
    }

    async fn flush(&self) -> Result<()> {
        // Console doesn't buffer, nothing to flush
        Ok(())
    }

    async fn close(&mut self) -> Result<()> {
        if self.verbose {
            println!("[{}] Console adapter closed. Messages processed: {}",
                self.name,
                self.messages_processed.load(Ordering::SeqCst)
            );
        }
        Ok(())
    }

    async fn health_check(&self) -> Result<AdapterHealth> {
        Ok(AdapterHealth {
            status: HealthStatus::Healthy,
            latency_ms: Some(0),
            last_success: Some(Utc::now()),
            last_error: None,
            error_count: 0,
            messages_processed: self.messages_processed.load(Ordering::SeqCst),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Location, Device, CarbonCaptureData, CarbonCaptureTechnology};
    use crate::core::climate::ClimateMessage;

    fn create_test_message() -> ClimateMessage {
        ClimateMessage::builder()
            .location(Location::new(64.0, -21.0))
            .device(Device::new("Test", "Device"))
            .carbon_capture_data(CarbonCaptureData {
                technology: CarbonCaptureTechnology::Dac,
                capture_rate_kg_per_hour: 125.5,
                ..Default::default()
            })
            .build()
            .unwrap()
    }

    #[tokio::test]
    async fn test_console_adapter_creation() {
        let adapter = ConsoleAdapter::new("test-console");
        assert_eq!(adapter.name(), "test-console");
        assert_eq!(adapter.adapter_type(), AdapterType::Console);
    }

    #[tokio::test]
    async fn test_console_adapter_health() {
        let adapter = ConsoleAdapter::new("test");
        let health = adapter.health_check().await.unwrap();
        assert_eq!(health.status, HealthStatus::Healthy);
    }

    #[tokio::test]
    async fn test_console_adapter_process() {
        let adapter = ConsoleAdapter::new("test");
        let message = create_test_message();

        // Process should succeed
        let result = adapter.process(&message).await;
        assert!(result.is_ok());

        // Check message count
        let health = adapter.health_check().await.unwrap();
        assert_eq!(health.messages_processed, 1);
    }
}
