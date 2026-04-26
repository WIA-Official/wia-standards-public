//! Client for beauty-tech API

use crate::error::{Result, BeautyTechError};
use crate::types::*;
use reqwest::Client;

pub struct BeautyTechClient {
    base_url: String,
    client: Client,
}

impl BeautyTechClient {
    pub fn new(base_url: String) -> Self {
        Self {
            base_url,
            client: Client::new(),
        }
    }

    pub async fn create(&self, item: Resource) -> Result<Resource> {
        let url = format!("{}/resources", self.base_url);
        let response = self.client
            .post(&url)
            .json(&item)
            .send()
            .await
            .map_err(|e| BeautyTechError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| BeautyTechError::ParseError(e.to_string()))
    }
}
