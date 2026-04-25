//! # WIA-DATA_QUALITY Standard
//!
//! 데이터 품질 관리 표준
//!
//! ## 철학: 홍익인간 (弘益人間)
//! "널리 인간을 이롭게 하라" - 정확한 데이터로 올바른 결정

pub mod types;
pub mod client;
pub mod error;
pub mod validators;
pub mod utils;

pub use client::DataQualityClient;
pub use error::{Error, Result};
pub use types::*;

pub const VERSION: &str = env!("CARGO_PKG_VERSION");
