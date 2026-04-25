//! Client for plastic enzyme API

use crate::error::{Result, PlasticEnzymeError};
use crate::types::*;
use reqwest::Client;

pub struct PlasticEnzymeClient {
    base_url: String,
    client: Client,
}

impl PlasticEnzymeClient {
    pub fn new(base_url: String) -> Self {
        Self {
            base_url,
            client: Client::new(),
        }
    }

    pub async fn create(&self, item: PlasticEnzyme) -> Result<PlasticEnzyme> {
        let url = format!("{}/items", self.base_url);
        let response = self.client
            .post(&url)
            .json(&item)
            .send()
            .await
            .map_err(|e| PlasticEnzymeError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| PlasticEnzymeError::ParseError(e.to_string()))
    }
}
