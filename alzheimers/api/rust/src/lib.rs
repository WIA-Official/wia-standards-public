//! Alzheimers SDK
//!
//! 弘익人間 (Benefit All Humanity)

pub mod types;
pub mod client;
pub mod error;
pub mod validators;
pub mod utils;

pub use client::AlzheimersClient;
pub use error::{Result, AlzheimersError};
pub use types::*;

pub const VERSION: &str = env!("CARGO_PKG_VERSION");
