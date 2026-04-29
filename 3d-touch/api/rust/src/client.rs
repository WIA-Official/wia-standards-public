//! Client for WIA 3D Touch API
//!
//! 弘益人間 - Technology that touches all humanity

use crate::error::{Error, Result};
use crate::types::*;
use async_trait::async_trait;
use reqwest;
use uuid::Uuid;

/// API client for 3D touch operations
#[derive(Debug, Clone)]
pub struct Client {
    base_url: String,
    api_key: String,
    http_client: reqwest::Client,
}

impl Client {
    /// Create a new client instance
    pub fn new(base_url: impl Into<String>, api_key: impl Into<String>) -> Result<Self> {
        let base_url = base_url.into();
        let api_key = api_key.into();

        if api_key.is_empty() {
            return Err(Error::InvalidInput("API key cannot be empty".into()));
        }

        Ok(Self {
            base_url,
            api_key,
            http_client: reqwest::Client::new(),
        })
    }

    /// Process a touch event
    pub async fn process_touch_event(&self, event: &TouchEvent) -> Result<TouchEvent> {
        let url = format!("{}/api/v1/touch-events", self.base_url);

        let response = self.http_client
            .post(&url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .json(event)
            .send()
            .await
            .map_err(|e| Error::NetworkError(e.to_string()))?;

        if !response.status().is_success() {
            return Err(Error::ApiError(format!("API error: {}", response.status())));
        }

        let processed: TouchEvent = response.json().await
            .map_err(|e| Error::ParseError(e.to_string()))?;

        Ok(processed)
    }

    /// Trigger haptic feedback
    pub async fn trigger_haptic(&self, pattern: &HapticPattern) -> Result<()> {
        let url = format!("{}/api/v1/haptic", self.base_url);

        let response = self.http_client
            .post(&url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .json(pattern)
            .send()
            .await
            .map_err(|e| Error::NetworkError(e.to_string()))?;

        if !response.status().is_success() {
            return Err(Error::ApiError(format!("API error: {}", response.status())));
        }

        Ok(())
    }

    /// Recognize gesture from touch events
    pub async fn recognize_gesture(&self, events: &[TouchEvent]) -> Result<GestureResult> {
        let url = format!("{}/api/v1/gestures/recognize", self.base_url);

        let response = self.http_client
            .post(&url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .json(events)
            .send()
            .await
            .map_err(|e| Error::NetworkError(e.to_string()))?;

        if !response.status().is_success() {
            return Err(Error::ApiError(format!("API error: {}", response.status())));
        }

        let gesture: GestureResult = response.json().await
            .map_err(|e| Error::ParseError(e.to_string()))?;

        Ok(gesture)
    }

    /// Get touch surface configuration
    pub async fn get_surface_config(&self, surface_id: Uuid) -> Result<TouchSurface> {
        let url = format!("{}/api/v1/surfaces/{}", self.base_url, surface_id);

        let response = self.http_client
            .get(&url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .send()
            .await
            .map_err(|e| Error::NetworkError(e.to_string()))?;

        if !response.status().is_success() {
            return Err(Error::ApiError(format!("API error: {}", response.status())));
        }

        let surface: TouchSurface = response.json().await
            .map_err(|e| Error::ParseError(e.to_string()))?;

        Ok(surface)
    }

    /// Calibrate touch surface
    pub async fn calibrate_surface(&self, surface_id: Uuid) -> Result<CalibrationData> {
        let url = format!("{}/api/v1/surfaces/{}/calibrate", self.base_url, surface_id);

        let response = self.http_client
            .post(&url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .send()
            .await
            .map_err(|e| Error::NetworkError(e.to_string()))?;

        if !response.status().is_success() {
            return Err(Error::ApiError(format!("API error: {}", response.status())));
        }

        let calibration: CalibrationData = response.json().await
            .map_err(|e| Error::ParseError(e.to_string()))?;

        Ok(calibration)
    }

    /// Get device capabilities
    pub async fn get_device_capabilities(&self, device_id: &str) -> Result<DeviceCapabilities> {
        let url = format!("{}/api/v1/devices/{}/capabilities", self.base_url, device_id);

        let response = self.http_client
            .get(&url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .send()
            .await
            .map_err(|e| Error::NetworkError(e.to_string()))?;

        if !response.status().is_success() {
            return Err(Error::ApiError(format!("API error: {}", response.status())));
        }

        let capabilities: DeviceCapabilities = response.json().await
            .map_err(|e| Error::ParseError(e.to_string()))?;

        Ok(capabilities)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_client_creation() {
        let client = Client::new("https://api.wia.global", "test-key");
        assert!(client.is_ok());
    }

    #[test]
    fn test_client_empty_api_key() {
        let client = Client::new("https://api.wia.global", "");
        assert!(client.is_err());
    }
}
