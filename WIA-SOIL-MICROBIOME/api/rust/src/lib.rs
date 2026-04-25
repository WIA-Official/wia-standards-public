//! WIA-SOIL-MICROBIOME Rust SDK
//!
//! This library provides a Rust SDK for interacting with the WIA-SOIL-MICROBIOME Standard API.
//!
//! # Philosophy
//!
//! 弘益人間 (Benefit All Humanity)
//!
//! # Example
//!
//! ```rust,no_run
//! use wia_soil_microbiome_sdk::{WiaSoilMicrobiomeClient, Config, Environment};
//!
//! #[tokio::main]
//! async fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     let client = WiaSoilMicrobiomeClient::new(Config {
//!         api_key: "your-api-key".to_string(),
//!         environment: Environment::Production,
//!         ..Default::default()
//!     });
//!
//!     let samples = client.samples().list(None).await?;
//!     println!("Found {} samples", samples.data.len());
//!
//!     Ok(())
//! }
//! ```

pub mod client;
pub mod error;
pub mod streaming;
pub mod types;

pub use client::WiaSoilMicrobiomeClient;
pub use error::{Error, Result};
pub use streaming::StreamingClient;
pub use types::*;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_shannon_diversity() {
        let abundances = vec![10.0, 20.0, 30.0, 40.0];
        let shannon = calculate_shannon_index(&abundances);
        assert!(shannon > 0.0);
        assert!(shannon < 2.0);
    }

    #[test]
    fn test_simpson_diversity() {
        let abundances = vec![25.0, 25.0, 25.0, 25.0];
        let simpson = calculate_simpson_index(&abundances);
        assert!(simpson > 0.7);
        assert!(simpson < 0.8);
    }
}

/// Calculate Shannon diversity index
///
/// # Arguments
/// * `abundances` - Vector of abundance values
///
/// # Returns
/// Shannon diversity index
pub fn calculate_shannon_index(abundances: &[f64]) -> f64 {
    let total: f64 = abundances.iter().sum();
    if total == 0.0 {
        return 0.0;
    }

    let proportions: Vec<f64> = abundances.iter().map(|&a| a / total).collect();
    -proportions
        .iter()
        .filter(|&&p| p > 0.0)
        .map(|&p| p * p.ln())
        .sum::<f64>()
}

/// Calculate Simpson diversity index
///
/// # Arguments
/// * `abundances` - Vector of abundance values
///
/// # Returns
/// Simpson diversity index
pub fn calculate_simpson_index(abundances: &[f64]) -> f64 {
    let total: f64 = abundances.iter().sum();
    if total == 0.0 {
        return 0.0;
    }

    let proportions: Vec<f64> = abundances.iter().map(|&a| a / total).collect();
    1.0 - proportions.iter().map(|&p| p * p).sum::<f64>()
}

/// Calculate species richness
///
/// # Arguments
/// * `abundances` - Vector of abundance values
///
/// # Returns
/// Number of species with non-zero abundance
pub fn calculate_richness(abundances: &[f64]) -> usize {
    abundances.iter().filter(|&&a| a > 0.0).count()
}
