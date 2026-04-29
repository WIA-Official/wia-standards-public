//! Galactic registry client implementation

use crate::{error::{Error, Result}, types::*};
use reqwest::Client;

#[derive(Debug, Clone)]
pub struct GalacticRegistryClient {
    api_key: String,
    base_url: String,
    client: Client,
}

impl GalacticRegistryClient {
    pub fn new(api_key: String) -> Self {
        Self::with_base_url(api_key, "https://api.wia.org/contact-010".to_string())
    }

    pub fn with_base_url(api_key: String, base_url: String) -> Self {
        Self {
            api_key,
            base_url,
            client: Client::new(),
        }
    }

    pub async fn register_civilization(&self, civ: &Civilization) -> Result<String> {
        let url = format!("{}/civilizations", self.base_url);
        let response = self.client
            .post(&url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .json(civ)
            .send()
            .await
            .map_err(|e| Error::NetworkError(e.to_string()))?;

        if !response.status().is_success() {
            return Err(Error::ApiError(format!("API error: {}", response.status())));
        }

        let result: serde_json::Value = response.json().await
            .map_err(|e| Error::ParseError(e.to_string()))?;
        Ok(result["civilization_id"].as_str().unwrap_or("").to_string())
    }

    pub async fn get_civilization(&self, id: &str) -> Result<Civilization> {
        let url = format!("{}/civilizations/{}", self.base_url, id);
        let response = self.client
            .get(&url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .send()
            .await
            .map_err(|e| Error::NetworkError(e.to_string()))?;

        response.json::<Civilization>().await
            .map_err(|e| Error::ParseError(e.to_string()))
    }

    pub async fn list_civilizations(&self, limit: u32) -> Result<Vec<Civilization>> {
        let url = format!("{}/civilizations?limit={}", self.base_url, limit);
        let response = self.client
            .get(&url)
            .header("Authorization", format!("Bearer {}", self.api_key))
            .send()
            .await
            .map_err(|e| Error::NetworkError(e.to_string()))?;

        response.json::<Vec<Civilization>>().await
            .map_err(|e| Error::ParseError(e.to_string()))
    }
}
