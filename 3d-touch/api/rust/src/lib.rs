//! # WIA 3D Touch Standard - Rust SDK
//!
//! 弘益人間 (홍익인간) - Benefit All Humanity
//!
//! This SDK provides a comprehensive interface for 3D touch and haptic feedback systems,
//! enabling immersive tactile experiences across devices and applications.
//!
//! ## Features
//!
//! - Multi-dimensional touch detection (pressure, position, gesture)
//! - Haptic feedback control and patterns
//! - Gesture recognition and tracking
//! - Force sensing and calibration
//! - Touch surface mapping and virtualization
//!
//! ## Example
//!
//! ```rust,no_run
//! use wia_3d_touch::{Client, TouchEvent, HapticPattern};
//!
//! #[tokio::main]
//! async fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     let client = Client::new("https://api.wia.global", "your-api-key")?;
//!
//!     // Register touch event handler
//!     let event = TouchEvent::new(100.0, 200.0, 0.8);
//!     client.process_touch_event(&event).await?;
//!
//!     // Trigger haptic feedback
//!     let haptic = HapticPattern::click();
//!     client.trigger_haptic(&haptic).await?;
//!
//!     Ok(())
//! }
//! ```

pub mod types;
pub mod client;
pub mod error;
pub mod validators;
pub mod utils;

pub use client::Client;
pub use error::{Error, Result};
pub use types::*;
pub use validators::*;
pub use utils::*;

/// WIA 3D Touch Standard Version
pub const VERSION: &str = "1.0.0";

/// Standard philosophy - Benefit All Humanity
pub const PHILOSOPHY: &str = "弘益人間 (홍익인간) - Benefit All Humanity";
