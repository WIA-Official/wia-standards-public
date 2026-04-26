//! Client for additive-manufacturing API

use crate::error::{Result, AdditiveManufacturingError};
use crate::types::*;
use reqwest::Client;

pub struct AdditiveManufacturingClient {
    base_url: String,
    client: Client,
}

impl AdditiveManufacturingClient {
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
            .map_err(|e| AdditiveManufacturingError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| AdditiveManufacturingError::ParseError(e.to_string()))
    }
}
