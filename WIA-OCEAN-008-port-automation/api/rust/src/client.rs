//! Client for port automation API

use crate::error::{Result, PortAutomationError};
use crate::types::*;
use reqwest::Client;

pub struct PortAutomationClient {
    base_url: String,
    client: Client,
}

impl PortAutomationClient {
    pub fn new(base_url: String) -> Self {
        Self {
            base_url,
            client: Client::new(),
        }
    }

    pub async fn create(&self, item: PortAutomation) -> Result<PortAutomation> {
        let url = format!("{}/items", self.base_url);
        let response = self.client
            .post(&url)
            .json(&item)
            .send()
            .await
            .map_err(|e| PortAutomationError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| PortAutomationError::ParseError(e.to_string()))
    }
}
