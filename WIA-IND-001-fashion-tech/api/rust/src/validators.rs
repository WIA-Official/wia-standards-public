//! Validation utilities for WIA-IND-001

use crate::error::{FashionTechError, Result};

pub fn validate_url(url: &str) -> Result<()> {
    if url.is_empty() {
        return Err(FashionTechError::InvalidUrl("URL cannot be empty".to_string()));
    }
    if !url.starts_with("http://") && !url.starts_with("https://") {
        return Err(FashionTechError::InvalidUrl("URL must start with http:// or https://".to_string()));
    }
    Ok(())
}

pub fn validate_api_key(api_key: &str) -> Result<()> {
    if api_key.is_empty() {
        return Err(FashionTechError::InvalidApiKey("API key cannot be empty".to_string()));
    }
    if api_key.len() < 16 {
        return Err(FashionTechError::InvalidApiKey("API key too short".to_string()));
    }
    Ok(())
}

pub fn validate_measurement(value: f64, name: &str) -> Result<()> {
    if value <= 0.0 {
        return Err(FashionTechError::ValidationError(format!("{} must be positive", name)));
    }
    Ok(())
}
