//! Client for organ shortage API

use crate::error::{Result, OrganShortageError};
use crate::types::*;
use reqwest::Client;

pub struct OrganShortageClient {
    base_url: String,
    client: Client,
}

impl OrganShortageClient {
    pub fn new(base_url: String) -> Self {
        Self {
            base_url,
            client: Client::new(),
        }
    }

    pub async fn create(&self, item: OrganShortage) -> Result<OrganShortage> {
        let url = format!("{}/items", self.base_url);
        let response = self.client
            .post(&url)
            .json(&item)
            .send()
            .await
            .map_err(|e| OrganShortageError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| OrganShortageError::ParseError(e.to_string()))
    }
}
