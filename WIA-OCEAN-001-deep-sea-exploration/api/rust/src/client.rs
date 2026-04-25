//! Client for deep sea exploration API

use crate::error::{Result, DeepSeaExplorationError};
use crate::types::*;
use reqwest::Client;
use async_trait::async_trait;
use uuid::Uuid;

/// Main client for deep sea exploration
pub struct DeepSeaExplorationClient {
    base_url: String,
    client: Client,
    api_key: Option<String>,
}

impl DeepSeaExplorationClient {
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

    /// Create a new exploration mission
    pub async fn create_mission(&self, mission: ExplorationMission) -> Result<ExplorationMission> {
        let url = format!("{}/missions", self.base_url);
        let response = self.client
            .post(&url)
            .json(&mission)
            .send()
            .await
            .map_err(|e| DeepSeaExplorationError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| DeepSeaExplorationError::ParseError(e.to_string()))
    }

    /// Get mission by ID
    pub async fn get_mission(&self, id: Uuid) -> Result<ExplorationMission> {
        let url = format!("{}/missions/{}", self.base_url, id);
        let response = self.client
            .get(&url)
            .send()
            .await
            .map_err(|e| DeepSeaExplorationError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| DeepSeaExplorationError::ParseError(e.to_string()))
    }

    /// Submit exploration data
    pub async fn submit_data(&self, data: ExplorationData) -> Result<()> {
        let url = format!("{}/data", self.base_url);
        self.client
            .post(&url)
            .json(&data)
            .send()
            .await
            .map_err(|e| DeepSeaExplorationError::NetworkError(e.to_string()))?;

        Ok(())
    }
}

#[async_trait]
pub trait ExplorationService {
    async fn plan_mission(&self, location: OceanLocation, depth: f64) -> Result<ExplorationMission>;
}
