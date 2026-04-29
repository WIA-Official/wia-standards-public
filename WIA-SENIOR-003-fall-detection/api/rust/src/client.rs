//! Client for fall detection API

use crate::error::{Result, FallDetectionError};
use crate::types::*;
use reqwest::Client;

pub struct FallDetectionClient {
    base_url: String,
    client: Client,
}

impl FallDetectionClient {
    pub fn new(base_url: String) -> Self {
        Self {
            base_url,
            client: Client::new(),
        }
    }

    pub async fn create(&self, item: FallDetection) -> Result<FallDetection> {
        let url = format!("{}/items", self.base_url);
        let response = self.client
            .post(&url)
            .json(&item)
            .send()
            .await
            .map_err(|e| FallDetectionError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| FallDetectionError::ParseError(e.to_string()))
    }
}
