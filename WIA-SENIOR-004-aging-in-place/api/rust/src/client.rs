//! Client for aging in place API

use crate::error::{Result, AgingInPlaceError};
use crate::types::*;
use reqwest::Client;

pub struct AgingInPlaceClient {
    base_url: String,
    client: Client,
}

impl AgingInPlaceClient {
    pub fn new(base_url: String) -> Self {
        Self {
            base_url,
            client: Client::new(),
        }
    }

    pub async fn create(&self, item: AgingInPlace) -> Result<AgingInPlace> {
        let url = format!("{}/items", self.base_url);
        let response = self.client
            .post(&url)
            .json(&item)
            .send()
            .await
            .map_err(|e| AgingInPlaceError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| AgingInPlaceError::ParseError(e.to_string()))
    }
}
