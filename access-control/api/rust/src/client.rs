//! Client for access-control API

use crate::error::{Result, AccessControlError};
use crate::types::*;
use reqwest::Client;

pub struct AccessControlClient {
    base_url: String,
    client: Client,
}

impl AccessControlClient {
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
            .map_err(|e| AccessControlError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| AccessControlError::ParseError(e.to_string()))
    }
}
