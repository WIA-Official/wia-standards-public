//! WIA-HOME Client implementation
//!
//! 弘益人間 - Connecting humanity through smart home technology

use crate::error::{HomeError, Result};
use crate::types::*;
use crate::validators;
use reqwest::{Client, Method};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

/// WIA-HOME API Client
#[derive(Debug, Clone)]
pub struct HomeClient {
    base_url: String,
    api_key: String,
    client: Client,
}

impl HomeClient {
    /// Create a new HomeClient instance
    ///
    /// # Arguments
    ///
    /// * `base_url` - The base URL of the WIA-HOME API
    /// * `api_key` - Your API key for authentication
    ///
    /// # Example
    ///
    /// ```no_run
    /// use wia_home::HomeClient;
    ///
    /// let client = HomeClient::new("https://api.wia-home.org", "your-api-key")?;
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn new(base_url: impl Into<String>, api_key: impl Into<String>) -> Result<Self> {
        let base_url = base_url.into();
        let api_key = api_key.into();

        validators::validate_url(&base_url)?;
        validators::validate_api_key(&api_key)?;

        Ok(Self {
            base_url,
            api_key,
            client: Client::new(),
        })
    }

    /// Discover devices in the home network
    pub async fn discover_devices(&self) -> Result<Vec<Device>> {
        let response: ApiResponse<Vec<Device>> = self
            .request(Method::GET, "/devices/discover", None::<&()>)
            .await?;

        response.data.ok_or(HomeError::ApiError("No data returned".to_string()))
    }

    /// Get device by ID
    pub async fn get_device(&self, device_id: Uuid) -> Result<Device> {
        let response: ApiResponse<Device> = self
            .request(Method::GET, &format!("/devices/{}", device_id), None::<&()>)
            .await?;

        response.data.ok_or(HomeError::ApiError("Device not found".to_string()))
    }

    /// Control device
    pub async fn control_device(&self, device_id: Uuid, action: &str, params: serde_json::Value) -> Result<()> {
        #[derive(Serialize)]
        struct ControlRequest {
            action: String,
            parameters: serde_json::Value,
        }

        let request = ControlRequest {
            action: action.to_string(),
            parameters: params,
        };

        let _response: ApiResponse<()> = self
            .request(Method::POST, &format!("/devices/{}/control", device_id), Some(&request))
            .await?;

        Ok(())
    }

    /// List all rooms
    pub async fn list_rooms(&self) -> Result<Vec<Room>> {
        let response: ApiResponse<Vec<Room>> = self
            .request(Method::GET, "/rooms", None::<&()>)
            .await?;

        response.data.ok_or(HomeError::ApiError("No rooms found".to_string()))
    }

    /// Create a scene
    pub async fn create_scene(&self, scene: &Scene) -> Result<Scene> {
        let response: ApiResponse<Scene> = self
            .request(Method::POST, "/scenes", Some(scene))
            .await?;

        response.data.ok_or(HomeError::ApiError("Failed to create scene".to_string()))
    }

    /// Execute a scene
    pub async fn execute_scene(&self, scene_id: Uuid) -> Result<()> {
        let _response: ApiResponse<()> = self
            .request(Method::POST, &format!("/scenes/{}/execute", scene_id), None::<&()>)
            .await?;

        Ok(())
    }

    /// Get energy consumption data
    pub async fn get_energy_data(&self, device_id: Uuid, days: u32) -> Result<Vec<EnergyData>> {
        let response: ApiResponse<Vec<EnergyData>> = self
            .request(
                Method::GET,
                &format!("/devices/{}/energy?days={}", device_id, days),
                None::<&()>,
            )
            .await?;

        response.data.ok_or(HomeError::ApiError("No energy data found".to_string()))
    }

    /// Get home configuration
    pub async fn get_home_config(&self) -> Result<HomeConfig> {
        let response: ApiResponse<HomeConfig> = self
            .request(Method::GET, "/config", None::<&()>)
            .await?;

        response.data.ok_or(HomeError::ApiError("No config found".to_string()))
    }

    /// Internal request helper
    async fn request<T, B>(&self, method: Method, path: &str, body: Option<&B>) -> Result<T>
    where
        T: for<'de> Deserialize<'de>,
        B: Serialize,
    {
        let url = format!("{}{}", self.base_url, path);
        let mut request = self.client.request(method, &url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .header("Content-Type", "application/json");

        if let Some(body) = body {
            request = request.json(body);
        }

        let response = request.send().await?;

        if !response.status().is_success() {
            return Err(HomeError::HttpError(response.status()));
        }

        let data = response.json::<T>().await?;
        Ok(data)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_client_creation() {
        let result = HomeClient::new("https://api.wia-home.org", "test-key");
        assert!(result.is_ok());
    }

    #[test]
    fn test_invalid_url() {
        let result = HomeClient::new("invalid-url", "test-key");
        assert!(result.is_err());
    }
}
