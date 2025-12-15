//! # WIA Climate Standard - Rust API
//!
//! A comprehensive Rust library for working with climate and environment data
//! following the WIA Climate Standard specification.
//!
//! ## Features
//!
//! - **Type-safe data structures** for all climate domains
//! - **Async-first design** with tokio runtime support
//! - **Validation** of data against the standard schema
//! - **Serialization** to/from JSON
//! - **Simulator adapters** for testing and development
//! - **Communication protocol** (Phase 3) with WebSocket support
//! - **Ecosystem integration** (Phase 4) with dashboards, storage, and alerts
//!
//! ## Supported Domains
//!
//! - Carbon Capture (CCUS, DAC)
//! - Weather Control (Cloud Seeding)
//! - Geoengineering (SRM, CDR)
//! - Vertical Farming (CEA)
//! - Ocean Cleanup
//! - Climate Modeling (CMIP6 compatible)
//!
//! ## Quick Start
//!
//! ```rust,ignore
//! use wia_climate::prelude::*;
//!
//! fn main() -> Result<()> {
//!     // Create a carbon capture data message
//!     let message = ClimateMessage::builder()
//!         .location(Location::new(64.0, -21.0))
//!         .device(Device::new("Climeworks", "Orca DAC"))
//!         .carbon_capture_data(CarbonCaptureData {
//!             technology: CarbonCaptureTechnology::Dac,
//!             capture_rate_kg_per_hour: 125.5,
//!             ..Default::default()
//!         })
//!         .build()?;
//!
//!     // Serialize to JSON
//!     let json = message.to_json()?;
//!     Ok(())
//! }
//! ```
//!
//! ## Protocol Usage (Phase 3)
//!
//! ```rust,no_run
//! use wia_climate::prelude::*;
//! use wia_climate::protocol::*;
//! use wia_climate::transport::*;
//!
//! # async fn example() -> Result<()> {
//! // Create a WebSocket client
//! let mut client = WebSocketClient::new("my-sensor");
//! client.connect("wss://api.example.com/wia-climate/v1/ws").await?;
//!
//! // Send climate data
//! let message = ClimateMessage::builder()
//!     .location(Location::new(64.0, -21.0))
//!     .device(Device::new("Climeworks", "Orca DAC"))
//!     .carbon_capture_data(CarbonCaptureData::default())
//!     .build()?;
//!
//! client.send_data(&message).await?;
//! # Ok(())
//! # }
//! ```
//!
//! ## License
//!
//! MIT License - This standard belongs to humanity.
//!
//! 弘益人間 - Benefit All Humanity

#![warn(missing_docs)]
#![warn(rustdoc::missing_crate_level_docs)]

pub mod types;
pub mod core;
pub mod adapters;
pub mod error;
pub mod protocol;
pub mod transport;
pub mod integration;

// Re-exports for convenience
pub use types::*;
pub use core::climate::*;
pub use error::{ClimateError, Result};

/// Prelude module for convenient imports
pub mod prelude {
    pub use crate::types::*;
    pub use crate::core::climate::*;
    pub use crate::adapters::*;
    pub use crate::error::{ClimateError, Result};
    pub use crate::protocol::{
        ProtocolMessage, MessageType, ConnectBuilder, CommandBuilder, SubscribeBuilder,
        ConnectPayload, CommandPayload, SubscribePayload, ErrorPayload, ErrorCode,
        ClientType, AuthMethod, QoS,
    };
    pub use crate::transport::{Transport, WebSocketTransport, WebSocketClient, MockTransport};
    pub use crate::integration::{
        OutputAdapter, OutputManager, OutputConfig, ProcessStrategy,
        AdapterType, AdapterHealth, HealthStatus, BatchResult, RetryStrategy,
        AlertEngine, AlertRule, AlertCondition, AlertSeverity, ComparisonOperator,
    };
}

/// Library version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Standard specification version
pub const SPEC_VERSION: &str = "1.0.0";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_version() {
        assert_eq!(VERSION, "1.0.0");
        assert_eq!(SPEC_VERSION, "1.0.0");
    }
}
