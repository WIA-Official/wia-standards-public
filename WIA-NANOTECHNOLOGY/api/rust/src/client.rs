//! Client for nanotechnology API

use crate::error::{Result, NanotechnologyError};
use crate::types::*;
use reqwest::Client;
use async_trait::async_trait;

/// Main client for nanotechnology operations
pub struct NanotechnologyClient {
    base_url: String,
    client: Client,
    api_key: Option<String>,
}

impl NanotechnologyClient {
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

    /// Design a nanoparticle
    pub async fn design_nanoparticle(&self, params: SynthesisParams) -> Result<Nanoparticle> {
        let url = format!("{}/design", self.base_url);
        let response = self.client
            .post(&url)
            .json(&params)
            .send()
            .await
            .map_err(|e| NanotechnologyError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| NanotechnologyError::ParseError(e.to_string()))
    }

    /// Simulate nanoparticle properties
    pub async fn simulate(&self, nanoparticle: Nanoparticle) -> Result<NanoProperties> {
        let url = format!("{}/simulate", self.base_url);
        let response = self.client
            .post(&url)
            .json(&nanoparticle)
            .send()
            .await
            .map_err(|e| NanotechnologyError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| NanotechnologyError::ParseError(e.to_string()))
    }

    /// Get nanoparticle by ID
    pub async fn get_nanoparticle(&self, id: Uuid) -> Result<Nanoparticle> {
        let url = format!("{}/nanoparticles/{}", self.base_url, id);
        let response = self.client
            .get(&url)
            .send()
            .await
            .map_err(|e| NanotechnologyError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| NanotechnologyError::ParseError(e.to_string()))
    }
}

#[async_trait]
pub trait NanoService {
    async fn synthesize(&self, params: SynthesisParams) -> Result<Nanoparticle>;
}
