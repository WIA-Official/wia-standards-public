//! # WIA-CONTACT-010 Galactic Registry Standard
//!
//! 은하계 문명 레지스트리 표준
//!
//! ## 철학: 홍익인간 (弘益人間)
//! "널리 인간을 이롭게 하라" - 우주 문명들의 평화로운 교류
//!
//! ## Features
//! - Civilization cataloging and registration
//! - Technology level classification (Kardashev scale)
//! - Contact protocol management
//! - Diplomatic relations tracking

pub mod types;
pub mod client;
pub mod error;
pub mod validators;
pub mod utils;

pub use client::GalacticRegistryClient;
pub use error::{Error, Result};
pub use types::*;

pub const VERSION: &str = env!("CARGO_PKG_VERSION");
