//! # WIA Climate Integration Module (Phase 4)
//!
//! This module provides ecosystem integration capabilities for the WIA Climate standard,
//! including adapters for dashboards, storage systems, and alert notifications.
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────┐
//! │             Output Manager                   │
//! │  ┌─────────────────────────────────────┐    │
//! │  │         Message Buffer              │    │
//! │  └─────────────────────────────────────┘    │
//! │              │                               │
//! │    ┌────────┼────────┐                      │
//! │    ▼        ▼        ▼                      │
//! │ Dashboard Storage  Alert                    │
//! │ Adapters  Adapters Adapters                 │
//! └─────────────────────────────────────────────┘
//! ```
//!
//! ## Example
//!
//! ```rust,ignore
//! use wia_climate::integration::*;
//! use wia_climate::prelude::*;
//!
//! #[tokio::main]
//! async fn main() -> Result<()> {
//!     // Create output manager
//!     let mut manager = OutputManager::new(OutputConfig::default());
//!
//!     // Add console adapter for testing
//!     let console = ConsoleAdapter::new("console");
//!     manager.add_adapter(console);
//!
//!     // Process climate data
//!     let message = ClimateMessage::builder()
//!         .location(Location::new(64.0, -21.0))
//!         .device(Device::new("Climeworks", "Orca DAC"))
//!         .carbon_capture_data(CarbonCaptureData::default())
//!         .build()?;
//!
//!     manager.process(&message).await?;
//!     Ok(())
//! }
//! ```

mod adapter;
mod manager;
mod alert;
pub mod adapters;

pub use adapter::*;
pub use manager::*;
pub use alert::*;

/// Integration module version
pub const INTEGRATION_VERSION: &str = "1.0.0";
