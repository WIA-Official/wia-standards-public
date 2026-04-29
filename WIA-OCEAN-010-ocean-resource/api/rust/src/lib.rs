//! Ocean Resource SDK
//!
//! 弘益人間 (Benefit All Humanity)

pub mod types;
pub mod client;
pub mod error;
pub mod validators;
pub mod utils;

pub use client::OceanResourceClient;
pub use error::{Result, OceanResourceError};
pub use types::*;

pub const VERSION: &str = env!("CARGO_PKG_VERSION");
