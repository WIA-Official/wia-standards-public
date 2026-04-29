//! Client for WIA Art Authentication API

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

    pub async fn verify_artwork(&self, artwork: &Artwork) -> Result<bool> {
        let url = format!("{}/api/v1/verify", self.base_url);
        let response = self.http_client
            .post(&url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .json(artwork)
            .send()
            .await
            .map_err(|e| Error::NetworkError(e.to_string()))?;

        if !response.status().is_success() {
            return Err(Error::ApiError(format!("API error: {}", response.status())));
        }

        Ok(response.json().await.map_err(|e| Error::ParseError(e.to_string()))?)
    }

    pub async fn get_certificate(&self, artwork_id: Uuid) -> Result<Certificate> {
        let url = format!("{}/api/v1/certificates/{}", self.base_url, artwork_id);
        let response = self.http_client
            .get(&url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .send()
            .await
            .map_err(|e| Error::NetworkError(e.to_string()))?;

        Ok(response.json().await.map_err(|e| Error::ParseError(e.to_string()))?)
    }
}
