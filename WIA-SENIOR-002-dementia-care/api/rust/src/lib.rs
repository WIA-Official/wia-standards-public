//! Dementia Care SDK
//!
//! 弘益人間 (Benefit All Humanity)

pub mod types;
pub mod client;
pub mod error;
pub mod validators;
pub mod utils;

pub use client::DementiaCareClient;
pub use error::{Result, DementiaCareError};
pub use types::*;

pub const VERSION: &str = env!("CARGO_PKG_VERSION");
