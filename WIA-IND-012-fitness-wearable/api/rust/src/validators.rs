//! Validation utilities
use crate::error::{Error, Result};

pub fn validate_url(url: &str) -> Result<()> {
    if url.is_empty() {
        return Err(Error::InvalidUrl("URL cannot be empty".to_string()));
    }
    if !url.starts_with("http://") && !url.starts_with("https://") {
        return Err(Error::InvalidUrl("URL must start with http:// or https://".to_string()));
    }
    Ok(())
}

pub fn validate_api_key(api_key: &str) -> Result<()> {
    if api_key.is_empty() {
        return Err(Error::InvalidApiKey("API key cannot be empty".to_string()));
    }
    if api_key.len() < 16 {
        return Err(Error::InvalidApiKey("API key too short".to_string()));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_validate_url() {
        assert!(validate_url("https://api.example.com").is_ok());
        assert!(validate_url("").is_err());
    }
    #[test]
    fn test_validate_api_key() {
        assert!(validate_api_key("1234567890abcdef").is_ok());
        assert!(validate_api_key("short").is_err());
    }
}
