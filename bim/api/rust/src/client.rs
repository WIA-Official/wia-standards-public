//! Client for bim API

use crate::error::{Result, BimError};
use crate::types::*;
use reqwest::Client;

pub struct BimClient {
    base_url: String,
    client: Client,
}

impl BimClient {
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
            .map_err(|e| BimError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| BimError::ParseError(e.to_string()))
    }
}
