//! Validation utilities for WIA-HOME SDK
//!
//! 弘益人間 - Ensuring data integrity for reliable home automation

use crate::error::{HomeError, Result};
use uuid::Uuid;

/// Validate URL format
pub fn validate_url(url: &str) -> Result<()> {
    if url.is_empty() {
        return Err(HomeError::InvalidUrl("URL cannot be empty".to_string()));
    }

    if !url.starts_with("http://") && !url.starts_with("https://") {
        return Err(HomeError::InvalidUrl("URL must start with http:// or https://".to_string()));
    }

    Ok(())
}

/// Validate API key format
pub fn validate_api_key(api_key: &str) -> Result<()> {
    if api_key.is_empty() {
        return Err(HomeError::InvalidApiKey("API key cannot be empty".to_string()));
    }

    if api_key.len() < 16 {
        return Err(HomeError::InvalidApiKey("API key too short".to_string()));
    }

    Ok(())
}

/// Validate device name
pub fn validate_device_name(name: &str) -> Result<()> {
    if name.is_empty() {
        return Err(HomeError::ValidationError("Device name cannot be empty".to_string()));
    }

    if name.len() > 100 {
        return Err(HomeError::ValidationError("Device name too long (max 100 chars)".to_string()));
    }

    Ok(())
}

/// Validate UUID
pub fn validate_uuid(id: &str) -> Result<Uuid> {
    Uuid::parse_str(id).map_err(|e| HomeError::ValidationError(format!("Invalid UUID: {}", e)))
}

/// Validate room name
pub fn validate_room_name(name: &str) -> Result<()> {
    if name.is_empty() {
        return Err(HomeError::ValidationError("Room name cannot be empty".to_string()));
    }

    if name.len() > 50 {
        return Err(HomeError::ValidationError("Room name too long (max 50 chars)".to_string()));
    }

    Ok(())
}

/// Validate scene name
pub fn validate_scene_name(name: &str) -> Result<()> {
    if name.is_empty() {
        return Err(HomeError::ValidationError("Scene name cannot be empty".to_string()));
    }

    if name.len() > 100 {
        return Err(HomeError::ValidationError("Scene name too long (max 100 chars)".to_string()));
    }

    Ok(())
}

/// Validate energy consumption value
pub fn validate_energy_value(value: f64) -> Result<()> {
    if value < 0.0 {
        return Err(HomeError::ValidationError("Energy value cannot be negative".to_string()));
    }

    if value > 1_000_000.0 {
        return Err(HomeError::ValidationError("Energy value unreasonably high".to_string()));
    }

    Ok(())
}

/// Validate timezone string
pub fn validate_timezone(tz: &str) -> Result<()> {
    if tz.is_empty() {
        return Err(HomeError::ValidationError("Timezone cannot be empty".to_string()));
    }

    // Basic validation - in production, use chrono-tz
    if !tz.contains('/') {
        return Err(HomeError::ValidationError("Invalid timezone format".to_string()));
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_validate_url() {
        assert!(validate_url("https://api.wia-home.org").is_ok());
        assert!(validate_url("http://localhost:8080").is_ok());
        assert!(validate_url("").is_err());
        assert!(validate_url("not-a-url").is_err());
    }

    #[test]
    fn test_validate_api_key() {
        assert!(validate_api_key("1234567890abcdef").is_ok());
        assert!(validate_api_key("").is_err());
        assert!(validate_api_key("short").is_err());
    }

    #[test]
    fn test_validate_device_name() {
        assert!(validate_device_name("Living Room Light").is_ok());
        assert!(validate_device_name("").is_err());
        assert!(validate_device_name(&"a".repeat(101)).is_err());
    }

    #[test]
    fn test_validate_energy_value() {
        assert!(validate_energy_value(100.5).is_ok());
        assert!(validate_energy_value(-1.0).is_err());
        assert!(validate_energy_value(2_000_000.0).is_err());
    }

    #[test]
    fn test_validate_timezone() {
        assert!(validate_timezone("America/New_York").is_ok());
        assert!(validate_timezone("Asia/Seoul").is_ok());
        assert!(validate_timezone("").is_err());
        assert!(validate_timezone("Invalid").is_err());
    }
}
