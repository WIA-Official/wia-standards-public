//! Client for memory assistance API

use crate::error::{Result, MemoryAssistanceError};
use crate::types::*;
use reqwest::Client;

pub struct MemoryAssistanceClient {
    base_url: String,
    client: Client,
}

impl MemoryAssistanceClient {
    pub fn new(base_url: String) -> Self {
        Self {
            base_url,
            client: Client::new(),
        }
    }

    pub async fn create(&self, item: MemoryAssistance) -> Result<MemoryAssistance> {
        let url = format!("{}/items", self.base_url);
        let response = self.client
            .post(&url)
            .json(&item)
            .send()
            .await
            .map_err(|e| MemoryAssistanceError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| MemoryAssistanceError::ParseError(e.to_string()))
    }
}
