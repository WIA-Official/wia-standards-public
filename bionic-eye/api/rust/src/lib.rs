//! WIA Bionic Eye API
//!
//! A comprehensive Rust API for visual prosthesis systems.
//!
//! # Modules
//!
//! - `types` - Core type definitions
//! - `frame` - Image capture and processing
//! - `electrode` - Electrode array management
//! - `stimulation` - Stimulation control
//! - `safety` - Safety systems

pub mod types;
pub mod electrode;
pub mod stimulation;
pub mod safety;

pub use types::*;
pub use electrode::*;
pub use stimulation::*;
pub use safety::*;

/// Library version
pub const VERSION: &str = "1.0.0";
