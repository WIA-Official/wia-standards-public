//! Client for air-quality API

use crate::error::{Result, AirQualityError};
use crate::types::*;
use reqwest::Client;

pub struct AirQualityClient {
    base_url: String,
    client: Client,
}

impl AirQualityClient {
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
            .map_err(|e| AirQualityError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| AirQualityError::ParseError(e.to_string()))
    }
}
