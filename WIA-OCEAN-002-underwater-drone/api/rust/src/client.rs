//! Client for underwater drone API

use crate::error::{Result, UnderwaterDroneError};
use crate::types::*;
use reqwest::Client;
use async_trait::async_trait;

/// Main client for underwater drone operations
pub struct UnderwaterDroneClient {
    base_url: String,
    client: Client,
    api_key: Option<String>,
}

impl UnderwaterDroneClient {
    /// Create a new client
    pub fn new(base_url: String) -> Self {
        Self {
            base_url,
            client: Client::new(),
            api_key: None,
        }
    }

    /// Set API key for authentication
    pub fn with_api_key(mut self, api_key: String) -> Self {
        self.api_key = Some(api_key);
        self
    }

    /// Register a drone
    pub async fn register_drone(&self, drone: UnderwaterDrone) -> Result<UnderwaterDrone> {
        let url = format!("{}/drones", self.base_url);
        let response = self.client
            .post(&url)
            .json(&drone)
            .send()
            .await
            .map_err(|e| UnderwaterDroneError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| UnderwaterDroneError::ParseError(e.to_string()))
    }

    /// Send navigation command
    pub async fn send_command(&self, command: NavigationCommand) -> Result<()> {
        let url = format!("{}/commands", self.base_url);
        self.client
            .post(&url)
            .json(&command)
            .send()
            .await
            .map_err(|e| UnderwaterDroneError::NetworkError(e.to_string()))?;

        Ok(())
    }

    /// Get drone telemetry
    pub async fn get_telemetry(&self, drone_id: Uuid) -> Result<Telemetry> {
        let url = format!("{}/drones/{}/telemetry", self.base_url, drone_id);
        let response = self.client
            .get(&url)
            .send()
            .await
            .map_err(|e| UnderwaterDroneError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| UnderwaterDroneError::ParseError(e.to_string()))
    }
}

#[async_trait]
pub trait DroneService {
    async fn control_drone(&self, drone_id: Uuid, command: NavigationCommand) -> Result<()>;
}
