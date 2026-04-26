//! # WIA 3D Printing Construction Standard - Rust SDK
//!
//! 弘益人間 (홍익인간) - Benefit All Humanity
//!
//! This SDK provides a comprehensive interface for 3D printing construction systems,
//! enabling automated building fabrication with precision and safety.
//!
//! ## Features
//!
//! - Building design management and validation
//! - Material specification and tracking
//! - Print job orchestration and monitoring
//! - Quality control and structural integrity verification
//! - Safety compliance and environmental monitoring
//!
//! ## Example
//!
//! ```rust,no_run
//! use wia_3d_printing_construction::{Client, BuildingDesign, MaterialType};
//!
//! #[tokio::main]
//! async fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     let client = Client::new("https://api.wia.global", "your-api-key")?;
//!
//!     let design = BuildingDesign::new(
//!         "Modern House",
//!         100.0, // square meters
//!         MaterialType::ReinforcedConcrete,
//!     );
//!
//!     let job = client.submit_print_job(&design).await?;
//!     println!("Print job submitted: {}", job.id);
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

/// WIA 3D Printing Construction Standard Version
pub const VERSION: &str = "1.0.0";

/// Standard philosophy - Benefit All Humanity
pub const PHILOSOPHY: &str = "弘益人間 (홍익인간) - Benefit All Humanity";
