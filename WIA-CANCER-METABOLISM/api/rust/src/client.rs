//! Client for WIA Cancer Metabolism Research API

use crate::error::{Error, Result};
use crate::types::*;
use uuid::Uuid;

#[derive(Debug, Clone)]
pub struct Client {
    base_url: String,
    api_key: String,
    http_client: reqwest::Client,
}

impl Client {
    pub fn new(base_url: impl Into<String>, api_key: impl Into<String>) -> Result<Self> {
        let api_key = api_key.into();
        if api_key.is_empty() {
            return Err(Error::InvalidInput("API key cannot be empty".into()));
        }
        Ok(Self {
            base_url: base_url.into(),
            api_key,
            http_client: reqwest::Client::new(),
        })
    }

    pub async fn analyze_metabolic_profile(&self, profile: &MetabolicProfile) -> Result<Vec<TherapeuticStrategy>> {
        let url = format!("{}/api/v1/analyze", self.base_url);
        let response = self.http_client
            .post(&url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .json(profile)
            .send()
            .await
            .map_err(|e| Error::NetworkError(e.to_string()))?;

        Ok(response.json().await.map_err(|e| Error::ParseError(e.to_string()))?)
    }

    pub async fn get_metabolic_profile(&self, profile_id: Uuid) -> Result<MetabolicProfile> {
        let url = format!("{}/api/v1/profiles/{}", self.base_url, profile_id);
        let response = self.http_client
            .get(&url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .send()
            .await
            .map_err(|e| Error::NetworkError(e.to_string()))?;

        Ok(response.json().await.map_err(|e| Error::ParseError(e.to_string()))?)
    }
}
