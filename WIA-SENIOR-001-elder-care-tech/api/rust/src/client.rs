//! Client for elder care technology API

use crate::error::{Result, ElderCareError};
use crate::types::*;
use reqwest::Client;

pub struct ElderCareClient {
    base_url: String,
    client: Client,
}

impl ElderCareClient {
    pub fn new(base_url: String) -> Self {
        Self {
            base_url,
            client: Client::new(),
        }
    }

    pub async fn create(&self, item: ElderCare) -> Result<ElderCare> {
        let url = format!("{}/items", self.base_url);
        let response = self.client
            .post(&url)
            .json(&item)
            .send()
            .await
            .map_err(|e| ElderCareError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| ElderCareError::ParseError(e.to_string()))
    }
}
