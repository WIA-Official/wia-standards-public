//! Client for senior mobility API

use crate::error::{Result, SeniorMobilityError};
use crate::types::*;
use reqwest::Client;

pub struct SeniorMobilityClient {
    base_url: String,
    client: Client,
}

impl SeniorMobilityClient {
    pub fn new(base_url: String) -> Self {
        Self {
            base_url,
            client: Client::new(),
        }
    }

    pub async fn create(&self, item: SeniorMobility) -> Result<SeniorMobility> {
        let url = format!("{}/items", self.base_url);
        let response = self.client
            .post(&url)
            .json(&item)
            .send()
            .await
            .map_err(|e| SeniorMobilityError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| SeniorMobilityError::ParseError(e.to_string()))
    }
}
