//! Client for submarine technology API

use crate::error::{Result, SubmarineTechError};
use crate::types::*;
use reqwest::Client;

pub struct SubmarineTechClient {
    base_url: String,
    client: Client,
}

impl SubmarineTechClient {
    pub fn new(base_url: String) -> Self {
        Self {
            base_url,
            client: Client::new(),
        }
    }

    pub async fn create(&self, item: SubmarineTech) -> Result<SubmarineTech> {
        let url = format!("{}/items", self.base_url);
        let response = self.client
            .post(&url)
            .json(&item)
            .send()
            .await
            .map_err(|e| SubmarineTechError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| SubmarineTechError::ParseError(e.to_string()))
    }
}
