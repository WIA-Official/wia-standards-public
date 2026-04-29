//! Client for SETI Data Standard API

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
            return Err(Error::InvalidInput("API key required".into()));
        }
        Ok(Self {
            base_url: base_url.into(),
            api_key,
            http_client: reqwest::Client::new(),
        })
    }

    pub async fn get_record(&self, id: Uuid) -> Result<Record> {
        let url = format!("{}/api/v1/records/{}", self.base_url, id);
        let response = self.http_client.get(&url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .send().await
            .map_err(|e| Error::NetworkError(e.to_string()))?;
        Ok(response.json().await.map_err(|e| Error::ParseError(e.to_string()))?)
    }
}
