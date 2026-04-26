//! Client for augmentation-safety API

use crate::error::{Result, AugmentationSafetyError};
use crate::types::*;
use reqwest::Client;

pub struct AugmentationSafetyClient {
    base_url: String,
    client: Client,
}

impl AugmentationSafetyClient {
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
            .map_err(|e| AugmentationSafetyError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| AugmentationSafetyError::ParseError(e.to_string()))
    }
}
