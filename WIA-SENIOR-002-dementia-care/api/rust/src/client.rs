//! Client for dementia care API

use crate::error::{Result, DementiaCareError};
use crate::types::*;
use reqwest::Client;

pub struct DementiaCareClient {
    base_url: String,
    client: Client,
}

impl DementiaCareClient {
    pub fn new(base_url: String) -> Self {
        Self {
            base_url,
            client: Client::new(),
        }
    }

    pub async fn create(&self, item: DementiaCare) -> Result<DementiaCare> {
        let url = format!("{}/items", self.base_url);
        let response = self.client
            .post(&url)
            .json(&item)
            .send()
            .await
            .map_err(|e| DementiaCareError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| DementiaCareError::ParseError(e.to_string()))
    }
}
