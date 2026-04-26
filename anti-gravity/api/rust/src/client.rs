//! Client for anti-gravity API

use crate::error::{Result, AntiGravityError};
use crate::types::*;
use reqwest::Client;

pub struct AntiGravityClient {
    base_url: String,
    client: Client,
}

impl AntiGravityClient {
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
            .map_err(|e| AntiGravityError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| AntiGravityError::ParseError(e.to_string()))
    }
}
