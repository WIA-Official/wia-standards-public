//! Client for loneliness prevention API

use crate::error::{Result, LonelinessPreventionError};
use crate::types::*;
use reqwest::Client;

pub struct LonelinessPreventionClient {
    base_url: String,
    client: Client,
}

impl LonelinessPreventionClient {
    pub fn new(base_url: String) -> Self {
        Self {
            base_url,
            client: Client::new(),
        }
    }

    pub async fn create(&self, item: LonelinessPrevention) -> Result<LonelinessPrevention> {
        let url = format!("{}/items", self.base_url);
        let response = self.client
            .post(&url)
            .json(&item)
            .send()
            .await
            .map_err(|e| LonelinessPreventionError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| LonelinessPreventionError::ParseError(e.to_string()))
    }
}
