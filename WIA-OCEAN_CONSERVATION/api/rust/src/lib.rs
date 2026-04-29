//! Ocean Conservation SDK
//!
//! 弘益人間 (Benefit All Humanity)

pub mod types;
pub mod client;
pub mod error;
pub mod validators;
pub mod utils;

pub use client::OceanConservationClient;
pub use error::{Result, OceanConservationError};
pub use types::*;

pub const VERSION: &str = env!("CARGO_PKG_VERSION");
