//! # WIA Smart Home API
//!
//! **Accessible Smart Home Standards for Everyone**
//!
//! 弘益人間 - Benefit All Humanity
//!
//! This crate provides a Rust implementation of the WIA Smart Home Accessibility
//! Standard, enabling developers to create accessible smart home applications.
//!
//! ## Features
//!
//! - **Multi-modal Interaction**: Support for voice, touch, switch, eye-gaze, and BCI
//! - **Disability-Aware**: Built-in support for visual, hearing, motor, and cognitive accessibility
//! - **Matter Protocol Compatible**: Designed for interoperability with Matter-based devices
//! - **WCAG Aligned**: Following web accessibility guidelines principles
//! - **Korean Language Support**: Native ko-KR TTS and voice command support
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use wia_smarthome::{SmartHomeController, types::*, adapters::*};
//! use std::sync::Arc;
//!
//! #[tokio::main]
//! async fn main() {
//!     // Create controller with simulator adapter
//!     let device_adapter = Arc::new(SimulatorDeviceAdapter::new());
//!     let notification_service = Arc::new(SimulatorNotificationService::new());
//!
//!     let controller = SmartHomeController::new()
//!         .with_device_adapter(device_adapter)
//!         .with_notification_service(notification_service);
//!
//!     // Create user profile
//!     let profile = controller.create_profile().await.unwrap();
//!
//!     // Create home
//!     let home = controller
//!         .create_home("My Home".to_string(), profile.profile_id)
//!         .await
//!         .unwrap();
//!
//!     println!("Created home: {}", home.name);
//! }
//! ```
//!
//! ## Modules
//!
//! - [`types`]: Type definitions for smart home entities
//! - [`core`]: Core smart home controller and traits
//! - [`adapters`]: Device adapter implementations
//! - [`protocol`]: Communication protocol (Matter + accessibility extensions)
//! - [`ecosystem`]: WIA ecosystem integrations (Eye Gaze, BCI, AAC, etc.)
//! - [`error`]: Error types
//!
//! ## Accessibility Support
//!
//! The API is designed with accessibility at its core:
//!
//! - **Input Modalities**: Voice, Touch, Switch, Gaze, Gesture, BCI, Sip/Puff
//! - **Output Modalities**: Visual Screen, Visual LED, Audio TTS, Audio Tone, Haptic, Braille
//! - **Disability Types**: Visual, Hearing, Motor, Cognitive
//!
//! ```rust
//! use wia_smarthome::types::*;
//!
//! // Create accessibility requirements for a user with low vision
//! let requirements = AccessibilityRequirements {
//!     primary_disabilities: vec![DisabilityType::VisualLowVision],
//!     wcag_level: WcagLevel::AA,
//!     specific_needs: SpecificNeeds {
//!         visual: VisualNeeds {
//!             screen_reader_required: true,
//!             magnification_required: true,
//!             high_contrast_required: true,
//!             audio_descriptions_required: false,
//!         },
//!         ..Default::default()
//!     },
//!     ..Default::default()
//! };
//! ```

pub mod adapters;
pub mod core;
pub mod ecosystem;
pub mod error;
pub mod protocol;
pub mod types;

// Re-exports for convenience
pub use core::SmartHomeController;
pub use error::{Result, SmartHomeError};

/// Library version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// WIA Smart Home standard version
pub const STANDARD_VERSION: &str = "1.0.0";
