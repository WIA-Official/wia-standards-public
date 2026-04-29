//! Client for microplastic detection API

use crate::error::{Result, MicroplasticDetectionError};
use crate::types::*;
use reqwest::Client;
use async_trait::async_trait;

/// Main client for microplastic detection
pub struct MicroplasticDetectionClient {
    base_url: String,
    client: Client,
    api_key: Option<String>,
}

impl MicroplasticDetectionClient {
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

    /// Detect microplastics in a sample
    pub async fn detect(&self, config: DetectionConfig) -> Result<Vec<MicroplasticParticle>> {
        let url = format!("{}/detect", self.base_url);
        let response = self.client
            .post(&url)
            .json(&config)
            .send()
            .await
            .map_err(|e| MicroplasticDetectionError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| MicroplasticDetectionError::ParseError(e.to_string()))
    }

    /// Analyze detected particles
    pub async fn analyze(&self, particles: Vec<MicroplasticParticle>) -> Result<AnalysisResult> {
        let url = format!("{}/analyze", self.base_url);
        let response = self.client
            .post(&url)
            .json(&particles)
            .send()
            .await
            .map_err(|e| MicroplasticDetectionError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| MicroplasticDetectionError::ParseError(e.to_string()))
    }
}

#[async_trait]
pub trait DetectionService {
    async fn detect_microplastics(&self, config: DetectionConfig) -> Result<Vec<MicroplasticParticle>>;
}
