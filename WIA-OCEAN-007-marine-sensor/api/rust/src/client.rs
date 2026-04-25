//! Client for marine sensor API

use crate::error::{Result, MarineSensorError};
use crate::types::*;
use reqwest::Client;

pub struct MarineSensorClient {
    base_url: String,
    client: Client,
}

impl MarineSensorClient {
    pub fn new(base_url: String) -> Self {
        Self {
            base_url,
            client: Client::new(),
        }
    }

    pub async fn create(&self, item: MarineSensor) -> Result<MarineSensor> {
        let url = format!("{}/items", self.base_url);
        let response = self.client
            .post(&url)
            .json(&item)
            .send()
            .await
            .map_err(|e| MarineSensorError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| MarineSensorError::ParseError(e.to_string()))
    }
}
