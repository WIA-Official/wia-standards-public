//! Client for ocean resource API

use crate::error::{Result, OceanResourceError};
use crate::types::*;
use reqwest::Client;

pub struct OceanResourceClient {
    base_url: String,
    client: Client,
}

impl OceanResourceClient {
    pub fn new(base_url: String) -> Self {
        Self {
            base_url,
            client: Client::new(),
        }
    }

    pub async fn create(&self, item: OceanResource) -> Result<OceanResource> {
        let url = format!("{}/items", self.base_url);
        let response = self.client
            .post(&url)
            .json(&item)
            .send()
            .await
            .map_err(|e| OceanResourceError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| OceanResourceError::ParseError(e.to_string()))
    }
}
