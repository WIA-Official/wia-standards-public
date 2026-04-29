//! Client for senior wearable API

use crate::error::{Result, SeniorWearableError};
use crate::types::*;
use reqwest::Client;

pub struct SeniorWearableClient {
    base_url: String,
    client: Client,
}

impl SeniorWearableClient {
    pub fn new(base_url: String) -> Self {
        Self {
            base_url,
            client: Client::new(),
        }
    }

    pub async fn create(&self, item: SeniorWearable) -> Result<SeniorWearable> {
        let url = format!("{}/items", self.base_url);
        let response = self.client
            .post(&url)
            .json(&item)
            .send()
            .await
            .map_err(|e| SeniorWearableError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| SeniorWearableError::ParseError(e.to_string()))
    }
}
