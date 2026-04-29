//! Validation utilities for WIA-HYDROPONICS SDK
//!
//! 弘益人間 - Ensuring data integrity for sustainable agriculture

use crate::error::{HydroponicsError, Result};
use uuid::Uuid;

/// Validate URL format
pub fn validate_url(url: &str) -> Result<()> {
    if url.is_empty() {
        return Err(HydroponicsError::InvalidUrl("URL cannot be empty".to_string()));
    }

    if !url.starts_with("http://") && !url.starts_with("https://") {
        return Err(HydroponicsError::InvalidUrl("URL must start with http:// or https://".to_string()));
    }

    Ok(())
}

/// Validate API key format
pub fn validate_api_key(api_key: &str) -> Result<()> {
    if api_key.is_empty() {
        return Err(HydroponicsError::InvalidApiKey("API key cannot be empty".to_string()));
    }

    if api_key.len() < 16 {
        return Err(HydroponicsError::InvalidApiKey("API key too short".to_string()));
    }

    Ok(())
}

/// Validate pH level (0-14)
pub fn validate_ph(ph: f64) -> Result<()> {
    if !(0.0..=14.0).contains(&ph) {
        return Err(HydroponicsError::InvalidPh(ph));
    }
    Ok(())
}

/// Validate EC level (0-10 mS/cm)
pub fn validate_ec(ec: f64) -> Result<()> {
    if !(0.0..=10.0).contains(&ec) {
        return Err(HydroponicsError::InvalidEc(ec));
    }
    Ok(())
}

/// Validate temperature (0-50°C for water)
pub fn validate_temperature(temp: f64) -> Result<()> {
    if !(0.0..=50.0).contains(&temp) {
        return Err(HydroponicsError::ValidationError("Temperature out of range".to_string()));
    }
    Ok(())
}

/// Validate humidity (0-100%)
pub fn validate_humidity(humidity: f64) -> Result<()> {
    if !(0.0..=100.0).contains(&humidity) {
        return Err(HydroponicsError::ValidationError("Humidity out of range".to_string()));
    }
    Ok(())
}

/// Validate system capacity
pub fn validate_capacity(liters: f64) -> Result<()> {
    if liters <= 0.0 {
        return Err(HydroponicsError::ValidationError("Capacity must be positive".to_string()));
    }
    if liters > 10000.0 {
        return Err(HydroponicsError::ValidationError("Capacity unreasonably large".to_string()));
    }
    Ok(())
}

/// Validate plant count
pub fn validate_plant_count(count: u32) -> Result<()> {
    if count > 10000 {
        return Err(HydroponicsError::ValidationError("Plant count unreasonably high".to_string()));
    }
    Ok(())
}

/// Validate health score (0-100)
pub fn validate_health_score(score: f64) -> Result<()> {
    if !(0.0..=100.0).contains(&score) {
        return Err(HydroponicsError::ValidationError("Health score out of range".to_string()));
    }
    Ok(())
}

/// Validate UUID
pub fn validate_uuid(id: &str) -> Result<Uuid> {
    Uuid::parse_str(id).map_err(|e| HydroponicsError::ValidationError(format!("Invalid UUID: {}", e)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_validate_ph() {
        assert!(validate_ph(7.0).is_ok());
        assert!(validate_ph(0.0).is_ok());
        assert!(validate_ph(14.0).is_ok());
        assert!(validate_ph(-1.0).is_err());
        assert!(validate_ph(15.0).is_err());
    }

    #[test]
    fn test_validate_ec() {
        assert!(validate_ec(2.0).is_ok());
        assert!(validate_ec(0.0).is_ok());
        assert!(validate_ec(10.0).is_ok());
        assert!(validate_ec(-1.0).is_err());
        assert!(validate_ec(11.0).is_err());
    }

    #[test]
    fn test_validate_temperature() {
        assert!(validate_temperature(25.0).is_ok());
        assert!(validate_temperature(-1.0).is_err());
        assert!(validate_temperature(51.0).is_err());
    }

    #[test]
    fn test_validate_humidity() {
        assert!(validate_humidity(50.0).is_ok());
        assert!(validate_humidity(-1.0).is_err());
        assert!(validate_humidity(101.0).is_err());
    }
}
