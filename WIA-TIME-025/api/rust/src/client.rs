//! Client for time management 025 API

use crate::error::{Result, TimeManagementError};
use crate::types::*;
use reqwest::Client;

pub struct TimeManagementClient {
    base_url: String,
    client: Client,
}

impl TimeManagementClient {
    pub fn new(base_url: String) -> Self {
        Self {
            base_url,
            client: Client::new(),
        }
    }

    pub async fn create(&self, item: TimeManagement) -> Result<TimeManagement> {
        let url = format!("{}/items", self.base_url);
        let response = self.client
            .post(&url)
            .json(&item)
            .send()
            .await
            .map_err(|e| TimeManagementError::NetworkError(e.to_string()))?;

        response.json().await
            .map_err(|e| TimeManagementError::ParseError(e.to_string()))
    }
}
