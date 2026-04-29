//! Client for ocean conservation API

use crate::error::{Result, OceanConservationError};
use crate::types::*;
use reqwest::Client;

pub struct OceanConservationClient {
    base_url: String,
    client: Client,
}

impl OceanConservationClient {
    pub fn new(base_url: String) -> Self {
        Self {
            base_url,
            client: Client::new(),
        }
    }

    pub async fn create(&self, item: OceanConservation) -> Result<OceanConservation> {
        let url = format!("{}/items", self.base_url);
        let response = self.client
            .post(&url)
            .json(&item)
            .send()
            .await
            .map_err(|e| OceanConservationError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| OceanConservationError::ParseError(e.to_string()))
    }
}
